import { BluetoothPeripheralAssociation } from '../schemas/schemas';
import { getWebSocketService } from '../services/websocket';

/**
 * Bind a Bluetooth address and driver to a peripheral role.
 *
 * An upsert, keyed on the association's peripheral id: a role cannot be filled twice, so
 * associating a device with a peripheral that already has one replaces it.
 */
export function associatePeripheral(association: BluetoothPeripheralAssociation): void {
  const ws = getWebSocketService();
  if (!ws) {
    throw new Error('WebSocket not connected');
  }
  ws.associateBluetoothPeripheral(association);
}

/**
 * Forget the association for a peripheral role.
 */
export function removePeripheral(peripheralId: number): void {
  const ws = getWebSocketService();
  if (!ws) {
    throw new Error('WebSocket not connected');
  }
  ws.removeBluetoothPeripheral(peripheralId);
}

/**
 * Stop or resume connecting to an associated peripheral, keeping the association.
 */
export function setPeripheralEnabled(peripheralId: number, enabled: boolean): void {
  const ws = getWebSocketService();
  if (!ws) {
    throw new Error('WebSocket not connected');
  }
  ws.setBluetoothPeripheralEnabled(peripheralId, enabled);
}

/**
 * Ask the machine to scan for nearby Bluetooth devices.
 *
 * May be refused: a scan monopolises a radio shared with WiFi and with the live links to
 * the peripherals themselves, so the machine declines while brewing, dispensing or
 * running a routine. The refusal comes back as `status.bluetooth.blocked` rather than as
 * an error here — this call is fire-and-forget, like every other command in this UI.
 */
export function scanForPeripherals(): void {
  const ws = getWebSocketService();
  if (!ws) {
    throw new Error('WebSocket not connected');
  }
  ws.scanForBluetoothPeripherals();
}

/**
 * A six-byte Bluetooth device address.
 *
 * Taken from the generated association type rather than written out as a tuple, so it
 * cannot drift from the wire format.
 */
export type BluetoothAddress = BluetoothPeripheralAssociation['address'];

/**
 * Format an address the way every log line and scanner in this project does.
 */
export function formatAddress(address: readonly number[]): string {
  return address.map((b) => b.toString(16).toUpperCase().padStart(2, '0')).join(':');
}
