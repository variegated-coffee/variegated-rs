import { TimezoneSetting } from '../schemas/schemas';
import { getWebSocketService } from '../services/websocket';

/**
 * Set the machine's timezone, by IANA zone name.
 *
 * # Scheduling only
 *
 * The zone governs when schedules fire and what the machine's own clock reads. **Every log
 * timestamp stays UTC** — shot logs record `recorded_at_unix_millis` and the SD card files
 * them by UTC day — so changing this never moves a timestamp that has already been written,
 * and never reinterprets one that has.
 *
 * # The machine may refuse it
 *
 * The firmware's timezone database is trimmed at build time by `CHRONO_TZ_TIMEZONE_FILTER`,
 * so it does not contain every IANA zone — the shipped default carries little more than
 * `Europe/Stockholm`. A name it does not know is **refused and logged**, and the machine keeps
 * the zone it had: storing an unresolvable name would leave this UI displaying a zone the
 * scheduler is not actually using.
 *
 * Nothing on this transport can report that refusal directly — the inter-processor link
 * carries no correlation ids, so `sendCommandAwaitingAck` resolving means *queued*, not
 * *applied*, exactly as it does for the shot-upload settings. The confirmation is the
 * `ConfigurationUpdate` that follows: if `configuration.timezone` comes back unchanged, the
 * machine refused it.
 *
 * # The empty string is UTC
 *
 * That is the stored default, so an unconfigured machine and a machine explicitly set to UTC
 * read the same — deliberately, since they mean the same thing.
 */
export async function setTimezone(name: string): Promise<void> {
  const ws = getWebSocketService();
  if (!ws) {
    throw new Error('Not connected to the machine');
  }

  const setting: TimezoneSetting = { name };

  await ws.sendCommandAwaitingAck({
    type: 'SetTimezone',
    value: setting,
  });
}
