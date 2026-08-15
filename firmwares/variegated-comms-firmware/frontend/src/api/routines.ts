import { Routine, RoutineIndex, RoutineIndexSchema, RoutineSchema } from '../schemas/schemas';
import { RoutineIdentifier, buildRoutineUrl } from '../utils/routineHelpers';
import { deleteRequest, fetchPostcard, postPostcard, putPostcard } from '../utils/postcard';

/**
 * Routine definitions, over HTTP.
 *
 * The WebSocket carries the *summary* list -- names, types and counts, enough to render
 * the picker -- and definitions come from here, one at a time. That split is the whole
 * point of the design: the comms processor has a 56 kB heap shared with Wi-Fi, BLE and the
 * ESPHome server, and it used to hold every routine's full definition permanently in order
 * to serve an editor that opens one of them occasionally.
 *
 * Writes are here too, and not only for symmetry. `MachineCommand::AddRoutine` over the
 * socket *could not work*: the server's inbound frame buffer is 256 bytes and a real
 * routine serialises to several kilobytes, so every save was rejected and took the
 * connection down with it. See the `frame_buf` note in the firmware's `websocket.rs`.
 */

/**
 * Exactly one routine request in flight, ever.
 *
 * The device answers a routine GET by waking the application processor over UART and
 * streaming the definition back a kilobyte at a time, behind a lock that already
 * serialises the exchange. A second concurrent request would therefore not go faster --
 * it would queue *inside the firmware*, where nothing can reorder it and a user's click
 * would sit behind however many prefetches happened to be ahead of it.
 *
 * Queueing on this side instead means the queue is ours to reorder, which is what
 * `priority` is for.
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
  return enqueue(routineKey(id), priority, () =>
    fetchPostcard(buildRoutineUrl(id), RoutineSchema)
  );
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
  return enqueueWrite('custom:new', () =>
    postPostcard('/routines/custom', routine, RoutineSchema, RoutineIndexSchema)
  );
}

/** Replace the routine at an index, or place one there if the slot is empty. */
export function saveRoutine(id: RoutineIdentifier, routine: Routine): Promise<void> {
  return enqueueWrite(routineKey(id), () =>
    putPostcard(buildRoutineUrl(id), routine, RoutineSchema)
  );
}

export function deleteRoutine(id: RoutineIdentifier): Promise<void> {
  return enqueueWrite(routineKey(id), () => deleteRequest(buildRoutineUrl(id)));
}
