import { ScheduleItem, ScheduleItemSchema } from '../schemas/schemas';
import { postPostcard, putPostcard, deleteRequest } from '../utils/postcard';

/**
 * Add a new schedule item to the machine configuration
 */
export async function addSchedule(item: ScheduleItem): Promise<void> {
  await postPostcard('/schedules', item, ScheduleItemSchema);
}

/**
 * Update an existing schedule item at the given index
 */
export async function updateSchedule(index: number, item: ScheduleItem): Promise<void> {
  await putPostcard(`/schedules/${index}`, item, ScheduleItemSchema);
}

/**
 * Delete a schedule item at the given index
 */
export async function deleteSchedule(index: number): Promise<void> {
  await deleteRequest(`/schedules/${index}`);
}
