import { ScheduleItem } from '../schemas/schemas';
import { getWebSocketService } from '../services/websocket';

/**
 * Add a new schedule item to the machine configuration
 */
export function addSchedule(item: ScheduleItem): void {
  const ws = getWebSocketService();
  if (!ws) {
    throw new Error('WebSocket not connected');
  }
  ws.addScheduleItem(item);
}

/**
 * Update an existing schedule item at the given index
 */
export function updateSchedule(index: number, item: ScheduleItem): void {
  const ws = getWebSocketService();
  if (!ws) {
    throw new Error('WebSocket not connected');
  }
  ws.updateScheduleItem(index, item);
}

/**
 * Delete a schedule item at the given index
 */
export function deleteSchedule(index: number): void {
  const ws = getWebSocketService();
  if (!ws) {
    throw new Error('WebSocket not connected');
  }
  ws.removeScheduleItem(index);
}
