import { serialize, deserialize } from '@variegated-coffee/serde-postcard-ts';
import { ClientQuery, QueryOk, Routine, RoutineIndex, RoutineSchema } from '../schemas/schemas';
import { RoutineIdentifier, indexFromIdentifier } from '../utils/routineHelpers';
import { CRC_TRAILER_BYTES, crc32c, readU32LE } from '../utils/crc32c';
import { getWebSocketService } from '../services/websocket';

/**
 * Routine definitions, over the WebSocket.
 *
 * The socket carries the *summary* list — names, types and counts, enough to render the
 * picker — and definitions are fetched one at a time. That split is the whole point of the
 * design and has not changed: the comms processor has a 56 kB heap shared with Wi-Fi, BLE and
 * the ESPHome server, and it used to hold every routine's full definition permanently in
 * order to serve an editor that opens one of them occasionally.
 *
 * # This was HTTP, for two reasons, and both are gone
 *
 * The first was size: the server's inbound frame buffer was a fixed 256 bytes and a real
 * routine serialises to kilobytes, so every save was rejected and took the connection down
 * with it. Inbound frames are heap-backed now and bounded at `MAX_CLIENT_FRAME_LEN`, which is
 * `ROUTINE_MAX_ENCODED_LEN + 256` — sized for exactly this.
 *
 * The second was answers: a write wants to know whether the routine was stored, and
 * `CommandAck` only ever said "queued". `WsMessage::Query` answers the question itself, and
 * carries `RoutineWriteError` whole. **The socket now reports this better than HTTP did** —
 * HTTP projected six outcomes onto three status codes and this module read only the number,
 * so `Immutable` and `TooLarge` both surfaced as `status: 400`.
 *
 * A routine travels as opaque postcard bytes in both directions, so the machine never decodes
 * one. See `EncodedPayload` in `ws_types.rs`.
 *
 * A *served* definition carries the CRC-32C trailer flash carries, and `fetchRoutine` checks
 * it. Writes go up bare: `store_routine` decodes them with `postcard::from_bytes`, which
 * ignores trailing bytes, and re-frames the routine itself on the way to flash — so a
 * trailer here would be neither read nor stored.
 */

/**
 * Exactly one routine request in flight, ever.
 *
 * The device answers by waking the application processor over UART, collecting the
 * definition a kilobyte at a time behind `ROUTINE_LOCK`, which already serialises the
 * exchange machine-wide. A second concurrent request would therefore not go faster — it
 * would queue *inside the firmware*, where nothing can reorder it and a user's click would
 * sit behind however many prefetches happened to be ahead of it.
 *
 * Queueing on this side instead means the queue is ours to reorder, which is what `priority`
 * is for.
 *
 * **The reason got stronger when this moved off HTTP, not weaker.** It used to be that the
 * device had two HTTP handler slots and the UART round trip behind them was the scarce part.
 * Now there is one WebSocket connection, and serving a query blocks the 5 Hz status push for
 * its duration — so an unqueued burst of prefetches would visibly freeze the live readouts.
 * `PREFETCH_GAP_MS` in `state/routineBodies.ts` is the other half of that.
 */
type Priority = 'user' | 'prefetch';

interface QueueEntry {
  key: string;
  run: () => Promise<unknown>;
  resolve: (value: never) => void;
  reject: (reason: unknown) => void;
  priority: Priority;
}

const queue: QueueEntry[] = [];
/** Entries currently queued or running, so a second caller joins rather than re-asks. */
const inFlight = new Map<string, Promise<unknown>>();
let running = false;

export function routineKey(id: RoutineIdentifier): string {
  return `${id.type}:${id.index}`;
}

async function drain(): Promise<void> {
  if (running) return;
  running = true;

  try {
    while (queue.length > 0) {
      // A user-priority entry overtakes every prefetch, but never preempts the request
      // already running -- the firmware's lock is held for its duration and cancelling
      // here would not release it any sooner.
      let next = queue.findIndex((entry) => entry.priority === 'user');
      if (next === -1) next = 0;
      const entry = queue.splice(next, 1)[0];

      try {
        const value = await entry.run();
        entry.resolve(value as never);
      } catch (e) {
        entry.reject(e);
      } finally {
        inFlight.delete(entry.key);
      }
    }
  } finally {
    running = false;
  }
}

function enqueue<T>(key: string, priority: Priority, run: () => Promise<T>): Promise<T> {
  // De-duplicated by key. Without this, clicking a routine the prefetcher is already
  // fetching would issue a second identical GET and make the user wait for both.
  const existing = inFlight.get(key);
  if (existing) {
    // A click on something already queued as background work promotes it rather than
    // waiting its turn.
    if (priority === 'user') {
      const queued = queue.find((entry) => entry.key === key);
      if (queued) queued.priority = 'user';
    }
    return existing as Promise<T>;
  }

  const promise = new Promise<T>((resolve, reject) => {
    queue.push({
      key,
      run,
      resolve: resolve as (value: never) => void,
      reject,
      priority,
    });
  });

  inFlight.set(key, promise);
  void drain();
  return promise;
}

/**
 * One routine's full definition.
 *
 * `priority` decides only where this sits in *our* queue. Pass `'user'` when someone is
 * waiting on a screen for it and `'prefetch'` when it is background work.
 */
export function fetchRoutine(
  id: RoutineIdentifier,
  priority: Priority = 'user'
): Promise<Routine> {
  return enqueue(routineKey(id), priority, async () => {
    const result = await query({ type: 'RoutineDefinition', value: indexFromIdentifier(id) });
    if (result.type !== 'RoutineDefinition') {
      throw new Error('The machine answered a different question');
    }
    // The machine returns the routine still postcard-encoded, so it never has to decode one
    // itself — see `EncodedPayload` in `ws_types.rs`. `seq(u8)` decodes to a number array,
    // hence the conversion; the bytes are the same either way.
    const bytes = new Uint8Array(result.value);

    // The trailer is checked, not skipped. postcard is positional and non-self-describing,
    // so a definition that lost or repeated bytes on the way here does not fail to decode —
    // it decodes into a *different routine*, which then renders as though it were the one
    // that was asked for. The definition crosses two hops to reach this point, and the
    // application processor computes this checksum before either of them.
    if (bytes.length <= CRC_TRAILER_BYTES) {
      throw new Error('The machine sent too few bytes to be a routine');
    }
    const body = bytes.subarray(0, bytes.length - CRC_TRAILER_BYTES);
    const expected = readU32LE(bytes, bytes.length - CRC_TRAILER_BYTES);
    const actual = crc32c(body);
    if (actual !== expected) {
      throw new Error(
        `The machine sent a damaged routine: its checksum says ` +
          `${expected.toString(16).padStart(8, '0')}, but the ${body.length} bytes before ` +
          `it hash to ${actual.toString(16).padStart(8, '0')}`
      );
    }

    return deserialize(RoutineSchema, body).value;
  });
}

/**
 * Writes share the queue with reads, and that is not merely tidiness.
 *
 * A routine larger than one kilobyte comes back as several chunks, and the device's lock
 * is held per chunk rather than for the whole download. A save landing between two of them
 * would leave the reader splicing the first half of the old routine onto the second half
 * of the new one -- and postcard is positional, so the result would *decode*, into a
 * routine nobody wrote. Serialising here means at least this client can never do that to
 * itself. The firmware also compares each chunk's declared total against the first, which
 * catches the same interleave from a second browser.
 *
 * Keyed distinctly from reads so a write never de-duplicates against a fetch of the same
 * routine: they are different operations that happen to name the same thing.
 */
function enqueueWrite<T>(key: string, run: () => Promise<T>): Promise<T> {
  return enqueue(`write:${key}`, 'user', run);
}

/**
 * Create a routine, and learn where it landed.
 *
 * The index is assigned by the machine's repository -- the first free custom slot -- so
 * the response body carries it back. Nothing else can tell the caller which routine it
 * just made.
 */
export function createRoutine(routine: Routine): Promise<RoutineIndex> {
  return enqueueWrite('custom:new', () => writeRoutine(null, routine));
}

/** Replace the routine at an index, or place one there if the slot is empty. */
export async function saveRoutine(id: RoutineIdentifier, routine: Routine): Promise<void> {
  await enqueueWrite(routineKey(id), () => writeRoutine(indexFromIdentifier(id), routine));
}

/**
 * Send a routine and find out whether it was stored.
 *
 * `index === null` creates a custom routine and the machine chooses the slot, which is why
 * both paths return a `RoutineIndex` rather than only the create.
 *
 * The routine is encoded here and travels as bytes, so the comms processor passes it through
 * to the validator untouched. That is what keeps a `Malformed` answer meaning "this is not a
 * routine" rather than "the middle hop re-encoded it".
 */
async function writeRoutine(index: RoutineIndex | null, routine: Routine): Promise<RoutineIndex> {
  const result = await query({
    type: 'WriteRoutine',
    value: { index, routine: Array.from(serialize(RoutineSchema, routine)) },
  });
  if (result.type !== 'RoutineStored') {
    throw new Error('The machine answered a different question');
  }
  return result.value;
}

export async function deleteRoutine(id: RoutineIdentifier): Promise<void> {
  // A command rather than a query, because deletion always was one: `DELETE /routines/...`
  // answered 204 the moment `RemoveRoutine` was queued, never having waited to hear whether
  // the routine went. The ack says exactly as much, so nothing is lost — and the list that
  // arrives afterwards is what actually confirms it.
  await enqueueWrite(routineKey(id), async () => {
    const ws = getWebSocketService();
    if (!ws) {
      throw new Error('Not connected to the machine');
    }
    await ws.sendCommandAwaitingAck({ type: 'RemoveRoutine', value: indexFromIdentifier(id) });
  });
}

/** Run one query, or fail with a message worth showing. */
function query(q: ClientQuery): Promise<QueryOk> {
  const ws = getWebSocketService();
  if (!ws) {
    return Promise.reject(new Error('Not connected to the machine'));
  }
  return ws.sendQuery(q);
}
