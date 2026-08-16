import { ShotAnnotations, ShotAnnotationsSchema, ShotLogId, ShotLogList, ShotLogListSchema } from '../schemas/schemas';
import { deleteRequest, fetchPostcard, postEmpty, putPostcard } from '../utils/postcard';

/**
 * Shot logs live on the machine's SD card, not in the WebSocket status stream.
 *
 * That is why this module talks HTTP while `api/bluetooth.ts` next to it talks over the
 * socket: a page is ten entries and a download is tens of kilobytes, and the WebSocket's
 * inbound frames are capped at 256 bytes.
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
 * The count is the machine's, not ours: there is no `limit` parameter, so a client cannot
 * ask for a page the link cannot carry.
 *
 * Paths are segments rather than a query string because the firmware's router matches
 * paths exactly and parses no query at all.
 *
 * `day` is callable but nothing in the UI passes it yet.
 */
export async function fetchShotLogs(options: ShotLogPageOptions = {}): Promise<ShotLogList> {
  const { day, before } = options;

  let path: string;
  if (day !== undefined) {
    const dayPath = day === 'NODATE' ? 'NODATE' : String(day).padStart(8, '0');
    path = before
      ? `/shots/day/${dayPath}/before/${shotTimePath(before)}`
      : `/shots/day/${dayPath}`;
  } else {
    path = before
      ? `/shots/before/${shotDayPath(before)}/${shotTimePath(before)}`
      : '/shots';
  }

  return fetchPostcard(path, ShotLogListSchema);
}

/**
 * Remove a shot from the card.
 *
 * **Queued, not confirmed.** A 200 means the command reached the machine's command
 * channel; whether the file is gone arrives afterwards as a `ShotLogEvent` of kind
 * `Deleted`. A delete that fails on the card produces no event, and the row stays.
 */
export async function deleteShotLog(id: ShotLogId): Promise<void> {
  return deleteRequest(`/shots/${shotDayPath(id)}/${shotTimePath(id)}`);
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
  return putPostcard(
    `/shots/${shotDayPath(id)}/${shotTimePath(id)}/annotations`,
    annotations,
    ShotAnnotationsSchema
  );
}

/**
 * Replace the annotations that will be stamped onto the next shot.
 *
 * Cleared by the machine when a shot finishes, so this is set per shot rather than once.
 */
export async function setPendingAnnotations(annotations: ShotAnnotations): Promise<void> {
  return putPostcard('/shots/pending', annotations, ShotAnnotationsSchema);
}

/**
 * Read a group's scale and record what it says as the dose for the next shot.
 *
 * Refused by the machine if that scale has no reading, rather than recording a zero --
 * the failure surfaces as a 503, not as a dose of 0 g.
 */
export async function tagDoseFromScale(groupIndex: number): Promise<void> {
  return postEmpty(`/command/tag-dose-from-scale/${groupIndex}`);
}
