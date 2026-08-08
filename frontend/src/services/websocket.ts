import { serialize, deserialize } from '@variegated-coffee/serde-postcard-ts';
import {
  WsMessageSchema,
  Status,
  Configuration,
  MachineDefinition,
  RoutineStorage,
  WsMessage,
  MachineCommand,
  BoilerControlMode,
  BoilerControlTargetValuesUpdate,
  GroupBrewControlMode,
  GroupBrewControlTargetValuesUpdate,
  PidParameterTarget,
  PidParameters,
  PumpConfiguration,
  Routine,
  RoutineIndex,
  ScheduleItem,
  BluetoothPeripheralAssociation
} from '../schemas/schemas';

export interface WebSocketServiceCallbacks {
  onStatusUpdate?: (status: Status) => void;
  onConfigurationUpdate?: (config: Configuration) => void;
  onMachineDefinition?: (def: MachineDefinition) => void;
  onRoutinesUpdate?: (routines: RoutineStorage) => void;
  onCommandAck?: (id: number, success: boolean, error?: string) => void;
  onConnect?: () => void;
  onDisconnect?: () => void;
  onError?: (error: Error) => void;
}

export class WebSocketService {
  private ws: WebSocket | null = null;
  private url: string;
  private callbacks: WebSocketServiceCallbacks;
  private reconnectTimer: number | null = null;
  private reconnectAttempts: number = 0;
  private maxReconnectAttempts: number = 10;
  private baseReconnectDelay: number = 1000;

  // Retry timers for initial data requests
  private machineDefinitionRetryTimer: number | null = null;
  private routinesRetryTimer: number | null = null;
  private machineDefinitionReceived: boolean = false;
  private routinesReceived: boolean = false;
  private readonly retryDelay: number = 3000;

  constructor(url: string, callbacks: WebSocketServiceCallbacks) {
    this.url = url;
    this.callbacks = callbacks;
  }

  connect(): void {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      return;
    }

    try {
      this.ws = new WebSocket(this.url);
      this.ws.binaryType = 'arraybuffer';

      this.ws.onopen = () => {
        console.log('WebSocket connected');
        this.reconnectAttempts = 0;
        this.machineDefinitionReceived = false;
        this.routinesReceived = false;
        this.callbacks.onConnect?.();

        // Request initial data with retry
        this.requestMachineDefinitionWithRetry();
        this.requestRoutinesWithRetry();
      };

      this.ws.onclose = (event) => {
        console.log('WebSocket disconnected:', event.code, event.reason);
        this.callbacks.onDisconnect?.();
        this.scheduleReconnect();
      };

      this.ws.onerror = (event) => {
        console.error('WebSocket error:', event);
        this.callbacks.onError?.(new Error('WebSocket connection error'));
      };

      // `MessageEvent.data` is `any` in the DOM lib because it depends on
      // `binaryType`. This socket sets it to 'arraybuffer' (and the server only ever
      // sends binary frames), so naming the type here is accurate rather than a
      // convenient lie.
      this.ws.onmessage = (event: MessageEvent<ArrayBuffer>) => {
        this.handleMessage(event.data);
      };
    } catch (error) {
      console.error('Failed to create WebSocket:', error);
      this.callbacks.onError?.(error as Error);
      this.scheduleReconnect();
    }
  }

  disconnect(): void {
    if (this.reconnectTimer !== null) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }

    if (this.machineDefinitionRetryTimer !== null) {
      clearTimeout(this.machineDefinitionRetryTimer);
      this.machineDefinitionRetryTimer = null;
    }

    if (this.routinesRetryTimer !== null) {
      clearTimeout(this.routinesRetryTimer);
      this.routinesRetryTimer = null;
    }

    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
  }

  isConnected(): boolean {
    return this.ws !== null && this.ws.readyState === WebSocket.OPEN;
  }

  private scheduleReconnect(): void {
    if (this.reconnectAttempts >= this.maxReconnectAttempts) {
      console.log('Max reconnect attempts reached');
      return;
    }

    // Exponential backoff
    const delay = this.baseReconnectDelay * Math.pow(2, this.reconnectAttempts);
    this.reconnectAttempts++;

    console.log(`Scheduling reconnect in ${delay}ms (attempt ${this.reconnectAttempts})`);

    this.reconnectTimer = window.setTimeout(() => {
      this.reconnectTimer = null;
      this.connect();
    }, delay);
  }

  private handleMessage(data: ArrayBuffer): void {
    try {
      const uint8Array = new Uint8Array(data);
      const result = deserialize(WsMessageSchema, uint8Array);
      // No cast needed: WsMessageSchema infers a discriminated union, so switching on
      // `type` below narrows `value` on its own.
      const message: WsMessage = result.value;

      // Handle different message types based on the type field
      switch (message.type) {
        case 'StatusUpdate':
          this.callbacks.onStatusUpdate?.(message.value);
          break;
        case 'ConfigurationUpdate':
          console.log('Received ConfigurationUpdate');
          this.callbacks.onConfigurationUpdate?.(message.value);
          break;
        case 'MachineDefinition':
          console.log('Received MachineDefinition:', message.value);
          this.machineDefinitionReceived = true;
          if (this.machineDefinitionRetryTimer !== null) {
            clearTimeout(this.machineDefinitionRetryTimer);
            this.machineDefinitionRetryTimer = null;
          }
          this.callbacks.onMachineDefinition?.(message.value);
          break;
        case 'RoutinesUpdate':
          console.log('Received RoutinesUpdate:', message.value);
          this.routinesReceived = true;
          if (this.routinesRetryTimer !== null) {
            clearTimeout(this.routinesRetryTimer);
            this.routinesRetryTimer = null;
          }
          this.callbacks.onRoutinesUpdate?.(message.value);
          break;
        case 'CommandAck': {
          const ack = message.value as { id: number; success: boolean; error: string | null };
          console.log('Received CommandAck:', ack);
          this.callbacks.onCommandAck?.(ack.id, ack.success, ack.error ?? undefined);
          break;
        }
        default:
          console.log('Received unknown message type:', message.type);
      }
    } catch (error) {
      console.error('Failed to deserialize WebSocket message:', error);
    }
  }

  private sendMessage(message: WsMessage): void {
    if (!this.isConnected()) {
      console.warn('WebSocket not connected, cannot send message');
      return;
    }

    try {
      const binary = serialize(WsMessageSchema, message);
      console.log('Sending binary message:', message, 'bytes:', Array.from(binary));
      this.ws!.send(binary);
    } catch (error) {
      console.error('Failed to serialize/send WebSocket message:', error);
    }
  }

  // Request methods with retry logic
  private requestMachineDefinitionWithRetry(): void {
    if (this.machineDefinitionReceived) {
      return;
    }

    console.log('Sending RequestMachineDefinition');
    this.sendMessage({ type: 'RequestMachineDefinition' });

    // Schedule retry if not received
    if (this.machineDefinitionRetryTimer !== null) {
      clearTimeout(this.machineDefinitionRetryTimer);
    }
    this.machineDefinitionRetryTimer = window.setTimeout(() => {
      this.machineDefinitionRetryTimer = null;
      if (!this.machineDefinitionReceived && this.isConnected()) {
        console.log('Retrying RequestMachineDefinition...');
        this.requestMachineDefinitionWithRetry();
      }
    }, this.retryDelay);
  }

  private requestRoutinesWithRetry(): void {
    if (this.routinesReceived) {
      return;
    }

    console.log('Sending RequestRoutines');
    this.sendMessage({ type: 'RequestRoutines' });

    // Schedule retry if not received
    if (this.routinesRetryTimer !== null) {
      clearTimeout(this.routinesRetryTimer);
    }
    this.routinesRetryTimer = window.setTimeout(() => {
      this.routinesRetryTimer = null;
      if (!this.routinesReceived && this.isConnected()) {
        console.log('Retrying RequestRoutines...');
        this.requestRoutinesWithRetry();
      }
    }, this.retryDelay);
  }

  // Public request methods (for manual refresh)
  requestMachineDefinition(): void {
    console.log('Sending RequestMachineDefinition');
    this.sendMessage({ type: 'RequestMachineDefinition' });
  }

  requestRoutines(): void {
    console.log('Sending RequestRoutines');
    this.sendMessage({ type: 'RequestRoutines' });
  }

  // Command methods
  sendMachineCommand(command: MachineCommand): void {
    this.sendMessage({ type: 'SendMachineCommand', value: command });
  }

  // Convenience methods for common commands
  setMode(mode: 'On' | 'Off' | 'PowerSaveStandby'): void {
    this.sendMachineCommand({
      type: 'SetMachineMode',
      value: { type: mode }
    });
  }

  startBrewing(groupIndex: number): void {
    this.sendMachineCommand({
      type: 'StartBrewing',
      value: groupIndex
    });
  }

  stopBrewing(groupIndex: number): void {
    this.sendMachineCommand({
      type: 'StopBrewing',
      value: groupIndex
    });
  }

  runRoutine(
    routineIndex: { type: 'Internal'; value: number } | { type: 'Function'; value: number } | { type: 'Custom'; value: number },
    params?: Map<number, number>
  ): void {
    this.sendMachineCommand({
      type: 'RunRoutine',
      value: [routineIndex, params ?? null]
    });
  }

  cancelRoutine(): void {
    this.sendMachineCommand({
      type: 'CancelRoutine'
    });
  }

  tareGroupScale(groupIndex: number): void {
    this.sendMachineCommand({
      type: 'TareGroupScale',
      value: groupIndex
    });
  }

  zeroCalibrateGroupScale(groupIndex: number): void {
    this.sendMachineCommand({
      type: 'ZeroCalibrateGroupScale',
      value: groupIndex
    });
  }

  calibrateGroupScale100g(groupIndex: number): void {
    this.sendMachineCommand({
      type: 'CalibrateGroupScale100g',
      value: groupIndex
    });
  }

  setSteamValveOpenness(steamWandIndex: number, openness: number): void {
    this.sendMachineCommand({
      type: 'SetSteamValveOpenness',
      value: [steamWandIndex, openness]
    });
  }

  optimizeRoutineStorage(): void {
    this.sendMachineCommand({
      type: 'OptimizeRoutineStorage'
    });
  }

  optimizeScheduleStorage(): void {
    this.sendMachineCommand({
      type: 'OptimizeScheduleStorage'
    });
  }

  optimizeConfigurationStorage(): void {
    this.sendMachineCommand({
      type: 'OptimizeConfigurationStorage'
    });
  }

  // Boiler and group control methods
  setBoilerControlTarget(
    boilerIndex: number,
    mode: BoilerControlMode,
    values?: BoilerControlTargetValuesUpdate
  ): void {
    this.sendMachineCommand({
      type: 'SetBoilerControlTarget',
      value: [boilerIndex, mode, values ?? null]
    });
  }

  setGroupBrewControlTarget(
    groupIndex: number,
    mode: GroupBrewControlMode,
    values?: GroupBrewControlTargetValuesUpdate
  ): void {
    this.sendMachineCommand({
      type: 'SetGroupBrewControlTarget',
      value: [groupIndex, mode, values ?? null]
    });
  }

  // PID parameters
  setPidParameters(target: PidParameterTarget, params: PidParameters): void {
    this.sendMachineCommand({
      type: 'SetPidParameters',
      value: [target, params]
    });
  }

  // Pump configuration methods
  setGroupPumpConfiguration(groupIndex: number, config: PumpConfiguration): void {
    this.sendMachineCommand({
      type: 'SetGroupPumpConfiguration',
      value: [groupIndex, config]
    });
  }

  setWaterTapPumpConfiguration(waterTapIndex: number, config: PumpConfiguration): void {
    this.sendMachineCommand({
      type: 'SetWaterTapPumpConfiguration',
      value: [waterTapIndex, config]
    });
  }

  setFillPumpConfiguration(boilerIndex: number, config: PumpConfiguration): void {
    this.sendMachineCommand({
      type: 'SetFillPumpConfiguration',
      value: [boilerIndex, config]
    });
  }

  // Routine CRUD methods
  addRoutine(routine: Routine): void {
    this.sendMachineCommand({
      type: 'AddRoutine',
      value: routine
    });
  }

  updateRoutine(index: RoutineIndex, routine: Routine): void {
    this.sendMachineCommand({
      type: 'UpdateRoutine',
      value: [index, routine]
    });
  }

  removeRoutine(index: RoutineIndex): void {
    this.sendMachineCommand({
      type: 'RemoveRoutine',
      value: index
    });
  }

  // Schedule CRUD methods
  addScheduleItem(item: ScheduleItem): void {
    this.sendMachineCommand({
      type: 'AddScheduleItem',
      value: item
    });
  }

  updateScheduleItem(index: number, item: ScheduleItem): void {
    this.sendMachineCommand({
      type: 'UpdateScheduleItem',
      value: [index, item]
    });
  }

  removeScheduleItem(index: number): void {
    this.sendMachineCommand({
      type: 'RemoveScheduleItem',
      value: index
    });
  }

  associateBluetoothPeripheral(association: BluetoothPeripheralAssociation): void {
    this.sendMachineCommand({
      type: 'AssociateBluetoothPeripheral',
      value: association
    });
  }

  removeBluetoothPeripheral(peripheralId: number): void {
    this.sendMachineCommand({
      type: 'RemoveBluetoothPeripheral',
      value: peripheralId
    });
  }

  setBluetoothPeripheralEnabled(peripheralId: number, enabled: boolean): void {
    this.sendMachineCommand({
      type: 'SetBluetoothPeripheralEnabled',
      value: [peripheralId, enabled]
    });
  }

  scanForBluetoothPeripherals(): void {
    this.sendMachineCommand({
      type: 'ScanForBluetoothPeripherals'
    });
  }
}

// Singleton instance for global use
let wsServiceInstance: WebSocketService | null = null;

export function createWebSocketService(url: string, callbacks: WebSocketServiceCallbacks): WebSocketService {
  if (wsServiceInstance) {
    wsServiceInstance.disconnect();
  }
  wsServiceInstance = new WebSocketService(url, callbacks);
  return wsServiceInstance;
}

export function getWebSocketService(): WebSocketService | null {
  return wsServiceInstance;
}
