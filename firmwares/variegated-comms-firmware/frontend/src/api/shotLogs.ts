import {
  ClientQuery,
  MachineCommand,
  QueryOk,
  ShotAnnotations,
  ShotLogId,
  ShotLogList,
} from '../schemas/schemas';
import { getWebSocketService } from '../services/websocket';

/**
 * How many entries one page asks for — `SHOT_LOG_PAGE_LEN` in the Rust types.
 *
 * A maximum, not a promise. The machine ends a page early when the entries are heavy enough
 * to threaten `SHOT_LOG_LIST_BUDGET`, which is set by the *inter-processor* link's 4 kB
 * accumulator rather than by anything on this side. Hand-mirrored, like
 * `ROUTINE_FORMAT_VERSION` in `routineHelpers.ts`: the schema exporter emits types, not
 * constants.
 */
const SHOT_LOG_PAGE_LEN = 10;

/**
 * Shot logs live on the machine's SD card, not in the WebSocket status stream.
 *
 * That is why this module talks HTTP while `api/bluetooth.ts` next to it talks over the
 * socket: a page is ten entries and a download is tens of kilobytes, which is well past
 * what belongs in a single message on a status stream regardless of what the frame limit
 * happens to be. (It was 256 bytes when this split was made; it is 8 KiB now, and a
 * download still does not belong here.)
 *
 * Nothing here is live -- a page is a snapshot from when it was asked for. What keeps a
 * rendered list current is the other direction: the machine pushes a `ShotLogEvent` when
 * a shot is stored or deleted, and `state/shotLogEvents.ts` is where that arrives.
 *
 * This is the first consumer of `utils/postcard.ts`, which existed unused until now.
 */

/**
 * The directory name a shot's day maps to on the card, and in its URL.
 *
 * `null` means the shot was recorded before the machine's clock had synced -- the RTC
 * starts in 1980 and only becomes wall-clock once the comms processor has been through
 * SNTP, so every shot pulled before the machine has network is undated. Those live under
 * `SHOTS/NODATE/`, and the literal string is what the firmware's `parse_dir_name`
 * accepts. Rendering `null` here as "null" or "" would produce a URL that 400s.
 */
export function shotDayPath(id: ShotLogId): string {
  return id.day === null ? 'NODATE' : String(id.day).padStart(8, '0');
}

/**
 * The file name half of a shot's path: `HHMMSSxx`, zero padded to eight digits.
 *
 * The padding is load-bearing rather than cosmetic. The firmware rejects a time
 * component that is not exactly eight digits, so a shot stored as `00000042` cannot be
 * addressed as `42`.
 */
export function shotTimePath(id: ShotLogId): string {
  return String(id.time).padStart(8, '0');
}

/** Where a shot can be downloaded from. Same origin, port 80. */
export function shotDownloadUrl(id: ShotLogId): string {
  return `/shots/${shotDayPath(id)}/${shotTimePath(id)}`;
}

/** Which shots a listing covers. `'NODATE'` is the undated directory. */
export interface ShotLogPageOptions {
  day?: number | 'NODATE';
  /** Resume strictly after this shot. Take it from the last entry of the previous page. */
  before?: ShotLogId;
}

/**
 * One page of shots, newest first.
 *
 * Ten at a time, or fewer -- the machine cuts a page short when the entries are heavy
 * enough to threaten the inter-processor link's frame limit. **Check `truncated` rather
 * than the entry count** to decide whether another page exists; a short page is not the
 * last page.
 *
 * The count is the machine's, not ours: `limit` is fixed at `SHOT_LOG_PAGE_LEN`, so a client
 * cannot ask for a page the link cannot carry.
 *
 * Three HTTP routes collapsed into this one query when the listing moved onto the socket —
 * `/shots`, `/shots/before/…` and `/shots/day/…` were only ever three spellings of one
 * `ShotLogListRequest`, because the firmware's router matches paths exactly and parses no
 * query string. The filter is a field again.
 *
 * `day` is callable but nothing in the UI passes it yet.
 */
export async function fetchShotLogs(options: ShotLogPageOptions = {}): Promise<ShotLogList> {
  const { day, before } = options;

  const result = await query({
    type: 'ShotLogPage',
    value: {
      limit: SHOT_LOG_PAGE_LEN,
      before: before ?? null,
      day:
        day === undefined
          ? { type: 'All' }
          : day === 'NODATE'
            ? { type: 'Undated' }
            : { type: 'Day', value: day },
    },
  });

  if (result.type !== 'ShotLogPage') {
    throw new Error('The machine answered a different question');
  }
  return result.value;
}

/**
 * Remove a shot from the card.
 *
 * **Queued, not confirmed, exactly as before.** The ack means the command reached the
 * machine's command channel — which is all the old `DELETE /shots/…` 200 meant too, since
 * that route also only `try_send`'d. Whether the file is gone arrives afterwards as a
 * `ShotLogEvent` of kind `Deleted`; a delete that fails on the card produces no event, and
 * the row stays.
 */
export async function deleteShotLog(id: ShotLogId): Promise<void> {
  return command({ type: 'DeleteShotLog', value: id });
}

/**
 * Replace the annotations on a shot already stored on the card.
 *
 * The whole block, not one key: the machine rewrites the entire record for any edit, so
 * a per-field call would cost one rewrite per field. Anything omitted from `annotations`
 * is therefore *removed* -- including the routine the machine stamped on itself.
 */
export async function setShotAnnotations(
  id: ShotLogId,
  annotations: ShotAnnotations
): Promise<void> {
  return command({ type: 'SetShotAnnotations', value: [id, annotations] });
}

/**
 * Replace the annotations that will be stamped onto the next shot.
 *
 * Cleared by the machine when a shot finishes, so this is set per shot rather than once.
 */
export async function setPendingAnnotations(annotations: ShotAnnotations): Promise<void> {
  return command({ type: 'SetPendingShotAnnotations', value: annotations });
}

/** One query, or a failure message worth showing. */
async function query(q: ClientQuery): Promise<QueryOk> {
  const ws = getWebSocketService();
  if (!ws) {
    throw new Error('Not connected to the machine');
  }
  return ws.sendQuery(q);
}

/**
 * One command, awaiting its ack.
 *
 * Every shot *mutation* is a `MachineCommand` and always was — the HTTP routes that carried
 * them did nothing but `try_send`. So these are commands rather than queries, and the ack
 * carries precisely as much information as the status code it replaces.
 */
async function command(c: MachineCommand): Promise<void> {
  const ws = getWebSocketService();
  if (!ws) {
    throw new Error('Not connected to the machine');
  }
  return ws.sendCommandAwaitingAck(c);
}

/**
 * Read a group's scale and record what it says as the dose for the next shot.
 *
 * **This lost a failure mode when it moved off HTTP.** `POST
 * /command/tag-dose-from-scale/{group}` was refused with a 503 when the scale had no reading,
 * rather than recording a zero — so the caller learned the dose had not been tagged. The
 * WebSocket ack resolves once the command is queued, which a machine with a silent scale does
 * just as readily. The dose is still not recorded as 0 g; the difference is that nothing tells
 * the browser so.
 *
 * Recovering it needs a reply from the application processor, which the inter-processor link
 * has no correlation id to carry. Worth revisiting if operators start losing doses silently.
 */
export async function tagDoseFromScale(groupIndex: number): Promise<void> {
  const ws = getWebSocketService();
  if (!ws) {
    throw new Error('Not connected to the machine');
  }
  return ws.tagDoseFromScale(groupIndex);
}
