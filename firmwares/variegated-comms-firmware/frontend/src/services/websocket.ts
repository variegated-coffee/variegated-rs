import { serialize, deserialize } from '@variegated-coffee/serde-postcard-ts';
import {
  WsMessageSchema,
  Status,
  Configuration,
  MachineDefinition,
  RoutineSummaryStorage,
  WsMessage,
  MachineCommand,
  BoilerControlMode,
  BoilerControlTargetValuesUpdate,
  GroupBrewControlMode,
  GroupBrewControlTargetValuesUpdate,
  PidParameterTarget,
  PidParameters,
  PumpConfiguration,
  RoutineIndex,
  ScheduleItem,
  BluetoothPeripheralAssociation,
  ShotLogEvent,
  ClientQuery,
  QueryOk,
  QueryOutcome,
  QueryError
} from '../schemas/schemas';

export interface WebSocketServiceCallbacks {
  onStatusUpdate?: (status: Status) => void;
  onConfigurationUpdate?: (config: Configuration) => void;
  onMachineDefinition?: (def: MachineDefinition) => void;
  onRoutinesUpdate?: (routines: RoutineSummaryStorage) => void;
  /**
   * A shot was stored or deleted on the card.
   *
   * Unprompted, and the only shot-log traffic on this socket -- listings and downloads
   * are HTTP. There is no retry and no initial request: an event is a fact about a
   * moment, and a client that connects afterwards fetches a page instead.
   */
  onShotLogEvent?: (event: ShotLogEvent) => void;
  onCommandAck?: (id: number, success: boolean, error?: string) => void;
  onConnect?: () => void;
  onDisconnect?: () => void;
  onError?: (error: Error) => void;
}

/**
 * How long a correlated command waits for its `CommandAck` before giving up.
 *
 * A failsafe, not an operational deadline. The ack is written by the firmware immediately
 * after `try_send` on a channel it does not block on, so on a working machine it comes back
 * within a round trip -- there is no legitimate path that takes seconds. Five is sized so it
 * can only ever fire when something is genuinely wrong (socket wedged, firmware stopped
 * servicing the receive half), never on a slow-but-fine machine.
 */
const COMMAND_ACK_TIMEOUT_MS = 5000;

/**
 * How long a query waits for its answer.
 *
 * Longer than an ack, and for a concrete reason rather than caution: the firmware's own
 * timeouts are 5 s for a routine or shot-log read and **10 s for a routine write**, and it
 * answers with `QueryError::Unavailable` when they expire. This has to outlast the longest of
 * them or the browser gives up first and the real answer — the one that says *why* — arrives
 * to a promise nobody is holding.
 */
const QUERY_TIMEOUT_MS = 12000;

/**
 * Turn a `QueryError` into something worth showing a person.
 *
 * The point of the whole query mechanism: over HTTP every one of these was
 * `HTTP error! status: 400`, because the status code was the only part the frontend read.
 */
function describeQueryError(error: QueryError): string {
  switch (error.type) {
    case 'NotFound':
      return 'The machine has nothing stored there';
    case 'Unavailable':
      return 'The machine did not answer in time';
    case 'RoutineWrite':
      switch (error.value.type) {
        case 'TooLarge':
          return 'That routine is too large for the machine to store';
        case 'Malformed':
          return 'The machine could not read that routine';
        case 'Immutable':
          return 'Internal routines are read-only';
        case 'Storage':
          return 'The machine could not write to its storage';
        case 'UnsupportedVersion':
          return 'That routine is in a format this machine does not support';
        default:
          return 'The machine refused the routine';
      }
    case 'ShotLogStorage':
      return `The machine could not read the card (${error.value.type})`;
    default:
      return 'The machine refused the request';
  }
}

export class WebSocketService {
  private ws: WebSocket | null = null;
  private url: string;
  private callbacks: WebSocketServiceCallbacks;
  private reconnectTimer: number | null = null;
  private reconnectAttempts: number = 0;
  private maxReconnectAttempts: number = 10;
  private baseReconnectDelay: number = 1000;

  // Correlated commands awaiting their ack.
  //
  // `nextCommandId` starts at 1 because the firmware answers the two "not available yet"
  // cases with `id: 0` -- those reply to requests that carry no id, so 0 is effectively
  // taken. Never reset on reconnect: an ack for a command from a previous connection can
  // only arrive if the id is still pending, and `rejectPendingCommands` clears those on
  // close, so a monotonic counter cannot alias a stale reply onto a fresh command.
  private nextCommandId: number = 1;
  private pendingCommands = new Map<
    number,
    { resolve: () => void; reject: (error: Error) => void; timer: number }
  >();
  private pendingQueries = new Map<
    number,
    { resolve: (value: QueryOk) => void; reject: (error: Error) => void; timer: number }
  >();

  // Retry timers for initial data requests
  private machineDefinitionRetryTimer: number | null = null;
  private routinesRetryTimer: number | null = null;
  private configurationRetryTimer: number | null = null;
  private machineDefinitionReceived: boolean = false;
  private routinesReceived: boolean = false;
  private configurationReceived: boolean = false;
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
        this.configurationReceived = false;
        this.callbacks.onConnect?.();

        // Request initial data with retry
        this.requestMachineDefinitionWithRetry();
        this.requestRoutinesWithRetry();
        this.requestConfigurationWithRetry();
      };

      this.ws.onclose = (event) => {
        console.log('WebSocket disconnected:', event.code, event.reason);
        this.rejectPendingCommands('Lost the connection to the machine');
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

    if (this.configurationRetryTimer !== null) {
      clearTimeout(this.configurationRetryTimer);
      this.configurationRetryTimer = null;
    }

    this.rejectPendingCommands('Disconnected from the machine');

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
          this.configurationReceived = true;
          if (this.configurationRetryTimer !== null) {
            clearTimeout(this.configurationRetryTimer);
            this.configurationRetryTimer = null;
          }
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
        case 'ShotLogEvent':
          console.log('Received ShotLogEvent:', message.value);
          this.callbacks.onShotLogEvent?.(message.value);
          break;
        case 'CommandAck': {
          const ack = message.value as { id: number; success: boolean; error: string | null };
          console.log('Received CommandAck:', ack);
          this.settlePendingCommand(ack.id, ack.success, ack.error ?? undefined);
          this.callbacks.onCommandAck?.(ack.id, ack.success, ack.error ?? undefined);
          break;
        }
        case 'QueryReply': {
          const reply = message.value as { id: number; outcome: QueryOutcome };
          this.settlePendingQuery(reply.id, reply.outcome);
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

  /**
   * Send a command and wait for the machine to say whether it took it.
   *
   * # What resolving means, which is less than it looks
   *
   * The ack says the **comms processor queued the command**, not that the machine applied
   * it. The inter-processor link carries no correlation ids, so nothing further back can be
   * matched to a request; see `WsMessage::SendMachineCommandWithId` in the firmware. What
   * this does catch is the failure that actually happens — a full command channel, which
   * previously dropped the command in silence.
   *
   * For "did the setting take", watch the `ConfigurationUpdate` that follows. The machine
   * republishes its configuration after a settings change, so the state arriving is the
   * confirmation. `ShotUploadPanel` is built on that pair: this promise clears the button,
   * the update confirms the value.
   */
  sendCommandAwaitingAck(command: MachineCommand): Promise<void> {
    if (!this.isConnected()) {
      return Promise.reject(new Error('Not connected to the machine'));
    }

    const id = this.nextCommandId++;

    return new Promise<void>((resolve, reject) => {
      const timer = window.setTimeout(() => {
        this.pendingCommands.delete(id);
        reject(new Error('The machine did not acknowledge the command'));
      }, COMMAND_ACK_TIMEOUT_MS);

      this.pendingCommands.set(id, { resolve, reject, timer });
      this.sendMessage({ type: 'SendMachineCommandWithId', value: { id, command } });
    });
  }

  /**
   * Ask the machine something and wait for the answer.
   *
   * Unlike {@link sendCommandAwaitingAck}, this resolves with a *result*, and rejecting
   * means the machine said no rather than that nobody was listening. `QueryError` carries
   * the five `RoutineWriteError` variants intact, so "internal routines are read-only" and
   * "this routine is too large to persist" are distinguishable again — over HTTP both
   * arrived as `status: 400` and the body was never read.
   *
   * Shares the pending map with the command acks. The ids come from one counter, so a
   * `CommandAck` and a `QueryReply` can never collide on one.
   */
  sendQuery(query: ClientQuery): Promise<QueryOk> {
    if (!this.isConnected()) {
      return Promise.reject(new Error('Not connected to the machine'));
    }

    const id = this.nextCommandId++;

    return new Promise<QueryOk>((resolve, reject) => {
      const timer = window.setTimeout(() => {
        this.pendingQueries.delete(id);
        reject(new Error('The machine did not answer'));
      }, QUERY_TIMEOUT_MS);

      this.pendingQueries.set(id, { resolve, reject, timer });
      this.sendMessage({ type: 'Query', value: { id, query } });
    });
  }

  /** Settle the promise waiting on a `QueryReply`, if there is one. */
  private settlePendingQuery(id: number, outcome: QueryOutcome): void {
    const pending = this.pendingQueries.get(id);
    if (!pending) {
      // Already timed out, or an answer to a question this client did not ask.
      return;
    }

    clearTimeout(pending.timer);
    this.pendingQueries.delete(id);

    if (outcome.type === 'Ok') {
      pending.resolve(outcome.value);
    } else {
      pending.reject(new Error(describeQueryError(outcome.value)));
    }
  }

  /** Resolve or reject the promise waiting on `id`, if there is one. */
  private settlePendingCommand(id: number, success: boolean, error?: string): void {
    const pending = this.pendingCommands.get(id);
    if (!pending) {
      // Either an uncorrelated ack (the firmware answers the two "not available yet"
      // cases with id 0) or one that already timed out. Neither is an error.
      return;
    }

    clearTimeout(pending.timer);
    this.pendingCommands.delete(id);

    if (success) {
      pending.resolve();
    } else {
      pending.reject(new Error(error ?? 'The machine rejected the command'));
    }
  }

  /**
   * Fail everything still waiting, because nothing can arrive to settle it.
   *
   * Without this a save in flight when the socket drops hangs until its timeout and then
   * reports a timeout, which describes the symptom rather than the cause. The disconnect is
   * the fact worth reporting.
   */
  private rejectPendingCommands(reason: string): void {
    for (const pending of this.pendingCommands.values()) {
      clearTimeout(pending.timer);
      pending.reject(new Error(reason));
    }
    this.pendingCommands.clear();

    for (const pending of this.pendingQueries.values()) {
      clearTimeout(pending.timer);
      pending.reject(new Error(reason));
    }
    this.pendingQueries.clear();
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

  /**
   * Ask for the configuration once per connection, retrying until one arrives.
   *
   * Unlike the two above, this is not answered from a cache on the machine: the comms
   * processor forwards it to the application processor and the reply comes back on the
   * ordinary broadcast path. So it is a real round trip, and worth exactly one per
   * connection -- every subsequent change arrives unprompted.
   *
   * It has to be asked for at all because a configuration reaches the browser only by
   * that broadcast, and a broadcast carries nothing to a client that was not yet
   * subscribed. A page opened between two publishes had an empty configuration until the
   * next one happened to come along.
   */
  private requestConfigurationWithRetry(): void {
    if (this.configurationReceived) {
      return;
    }

    console.log('Sending RequestConfiguration');
    this.sendMessage({ type: 'RequestConfiguration' });

    if (this.configurationRetryTimer !== null) {
      clearTimeout(this.configurationRetryTimer);
    }
    this.configurationRetryTimer = window.setTimeout(() => {
      this.configurationRetryTimer = null;
      if (!this.configurationReceived && this.isConnected()) {
        console.log('Retrying RequestConfiguration...');
        this.requestConfigurationWithRetry();
      }
    }, this.retryDelay);
  }

  // Public request methods (for manual refresh)
  requestMachineDefinition(): void {
    console.log('Sending RequestMachineDefinition');
    this.sendMessage({ type: 'RequestMachineDefinition' });
  }

  // No `requestRoutines()`. The list is asked for once per connection, above, and every
  // change after that arrives unprompted -- the application processor pushes a fresh
  // summary list whenever its repository is mutated. A manual refresh could only re-read
  // the comms processor's cache, which is the same thing the push already updated; called
  // straight after a write, as it was, it read that cache *before* the write had reached
  // the machine and reliably returned the stale list.

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

  runRoutine(routineIndex: RoutineIndex, params?: Map<number, number>): void {
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

  /**
   * Drive the scale's own timer.
   *
   * Write-only: the timer runs on the scale and drives the scale's display, and nothing
   * reads it back, so there is no state here to keep in sync and no response to await.
   *
   * Whether the fitted scale has a timer at all is a property of its driver -- both
   * Bluetooth protocols do, a load cell wired to the machine does not -- and a scale
   * without one logs the command and drops it.
   */
  controlScaleTimer(
    groupIndex: number,
    command: 'Start' | 'Stop' | 'Reset' | 'TareAndStart'
  ): void {
    this.sendMachineCommand({
      type: 'ControlScaleTimer',
      value: [{ type: 'GroupScale', value: groupIndex }, { type: command }]
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

  // Routine CRUD lives in `api/routines.ts`, over HTTP.
  //
  // It moved there for two reasons and only one of them has gone away. `AddRoutine` and
  // `UpdateRoutine` serialise to several kilobytes, and the server's inbound frame buffer
  // was a fixed 256 bytes that rejected anything longer *by closing the connection* -- so
  // every save made this way failed and took the socket with it. Inbound frames are now
  // heap-backed and bounded at 8 KiB, which a routine clears.
  //
  // The reason that remains: all four operations answer with whether they actually worked.
  // `sendCommandAwaitingAck` below is a step toward that, but its ack means "the comms
  // processor queued it", not "the machine stored it" -- and for a routine write, stored is
  // the only answer worth having. Moving these back needs an end-to-end reply, which the
  // inter-processor link cannot currently express.

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

  /**
   * What the machine does to a group's scale when a brew starts.
   *
   * `actions` is the bitfield from `utils/brewActions`, not a list: the firmware stores it in
   * one byte and the schema carries it as a newtype over `u8`.
   */
  setGroupBrewActions(groupIndex: number, actions: number): void {
    this.sendMachineCommand({
      type: 'SetGroupBrewActions',
      value: [groupIndex, actions]
    });
  }

  setBluetoothPeripheralEnabled(peripheralId: number, enabled: boolean): void {
    this.sendMachineCommand({
      type: 'SetBluetoothPeripheralEnabled',
      value: [peripheralId, enabled]
    });
  }

  /**
   * Record the scale's current reading as the dose for the next shot.
   *
   * Awaits an ack rather than firing and forgetting, because this is a *moment* — the
   * operator has just put the portafilter on the scale — and silence would be
   * indistinguishable from success.
   *
   * **The ack is weaker than the HTTP route it replaced, and knowingly so.** `POST
   * /command/tag-dose-from-scale/{group}` answered 503 when the scale had no reading to
   * tag; this resolves as soon as the command is queued, and a scale that was not reporting
   * fails silently on the machine. Recovering that needs an end-to-end reply the
   * inter-processor link cannot express — see `WsMessage::SendMachineCommandWithId`.
   */
  tagDoseFromScale(groupIndex: number): Promise<void> {
    return this.sendCommandAwaitingAck({
      type: 'TagDoseFromScale',
      value: { type: 'GroupScale', value: groupIndex },
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
