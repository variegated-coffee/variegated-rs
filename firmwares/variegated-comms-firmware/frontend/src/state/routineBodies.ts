import { useEffect, useState } from 'preact/hooks';
import { Routine, RoutineSummaryStorage } from '../schemas/schemas';
import { fetchRoutine, routineKey } from '../api/routines';
import { RoutineIdentifier } from '../utils/routineHelpers';

/**
 * Every routine's definition, fetched in the background one at a time.
 *
 * **Prefetched, not fetched on demand**, and the reason is which side is short of
 * resources. The browser has orders of magnitude more memory than either processor; what
 * is scarce is the device's HTTP server -- two handler slots -- and the UART round trip
 * behind it. Walking the list serially while the user reads it spends time that was going
 * to be idle anyway, so by the time they open the editor the definition is already here,
 * and the machine never sees more than one routine request at once.
 *
 * Fetching on click would put that round trip on the critical path of every interaction,
 * and a screen that renders several routines would fire a burst of them at a server that
 * can only answer one.
 */

/** How long to wait between prefetches. */
const PREFETCH_GAP_MS = 250;

type Listener = () => void;

const bodies = new Map<string, Routine>();
const listeners = new Set<Listener>();

/**
 * The summary list the current cache was built against.
 *
 * Compared by value, not identity: a client-requested `RequestRoutines` is always
 * answered, so an arriving list is not by itself evidence that anything changed.
 */
let lastSummaryJson: string | null = null;

/** Bumped to abandon an in-flight walk. A stale walk checks this and stops. */
let walkGeneration = 0;

function notify(): void {
  for (const listener of listeners) listener();
}

export function getRoutineBody(id: RoutineIdentifier): Routine | undefined {
  return bodies.get(routineKey(id));
}

/**
 * Forget one routine, because we just changed it.
 *
 * Called before the refresh round trip rather than after, so nothing can read the
 * pre-edit body in the window between the save landing and the new summaries arriving.
 */
export function invalidateRoutineBody(id: RoutineIdentifier): void {
  if (bodies.delete(routineKey(id))) notify();
}

/**
 * Fetch a definition now, jumping the prefetch queue, and cache it.
 *
 * Normally a cache hit, since the walk will have been there first. The miss cases are a
 * page opened on a routine before the walk reached it, and a routine invalidated by a
 * save.
 */
export async function loadRoutineBody(id: RoutineIdentifier): Promise<Routine> {
  const cached = bodies.get(routineKey(id));
  if (cached) return cached;

  const routine = await fetchRoutine(id, 'user');
  bodies.set(routineKey(id), routine);
  notify();
  return routine;
}

function identifiersOf(storage: RoutineSummaryStorage): RoutineIdentifier[] {
  const ids: RoutineIdentifier[] = [];
  for (const index of storage.internal?.keys() ?? []) ids.push({ type: 'internal', index });
  for (const index of storage.function?.keys() ?? []) ids.push({ type: 'function', index });
  for (const index of storage.custom?.keys() ?? []) ids.push({ type: 'custom', index });
  return ids;
}

/**
 * Point the cache at a summary list, and start or restart the background walk.
 *
 * When the list has changed the whole cache is dropped rather than diffed. A summary
 * carries a name, a type and four counts, so it cannot tell whether a step's text was
 * edited -- and a stale definition shown as current is worse than a refetch that costs a
 * few hundred bytes on a machine that is idle.
 *
 * The residual: a *second* browser editing only prose -- a step description, with every
 * count and the name unchanged -- produces an identical summary list, and this client
 * keeps its stale copy until reload. Closing that would mean a per-routine fingerprint on
 * the wire, which would mean re-serialising every routine on the application processor on
 * every poll: exactly the work this design removed.
 */
export function syncRoutineBodies(storage: RoutineSummaryStorage | null): void {
  if (storage === null) return;

  const ids = identifiersOf(storage);
  const fingerprint = JSON.stringify(
    ids.map((id) => {
      const summary = summaryFor(storage, id);
      return [
        id.type,
        id.index,
        summary?.name,
        summary?.step_count,
        summary?.parameter_count,
        summary?.derived_parameter_count,
        summary?.finally_count,
      ];
    })
  );

  if (fingerprint === lastSummaryJson) return;
  lastSummaryJson = fingerprint;

  bodies.clear();
  notify();
  void walk(ids, ++walkGeneration);
}

function summaryFor(storage: RoutineSummaryStorage, id: RoutineIdentifier) {
  if (id.type === 'internal') return storage.internal?.get(id.index);
  if (id.type === 'function') return storage.function?.get(id.index);
  return storage.custom?.get(id.index);
}

/**
 * Fetch every definition, in list order, one at a time.
 *
 * The gap between them is not throttling for its own sake: it leaves room between round
 * trips for the 5 Hz status push and for anything the user does. On a machine that is
 * pulling a shot, the server has more important work than filling this cache.
 */
async function walk(ids: RoutineIdentifier[], generation: number): Promise<void> {
  for (const id of ids) {
    // A newer list arrived, or the socket dropped and re-synced. Stop rather than fill a
    // cache that has already been replaced.
    if (generation !== walkGeneration) return;
    if (bodies.has(routineKey(id))) continue;

    try {
      const routine = await fetchRoutine(id, 'prefetch');
      if (generation !== walkGeneration) return;
      bodies.set(routineKey(id), routine);
      notify();
    } catch {
      // One routine failing does not stop the walk. A definition the machine cannot
      // serve is a routine the editor will fetch again -- and be told about properly --
      // when someone actually opens it.
    }

    await new Promise((resolve) => setTimeout(resolve, PREFETCH_GAP_MS));
  }
}

/** Re-render as bodies land. */
export function useRoutineBody(id: RoutineIdentifier | null): Routine | undefined {
  const [, setTick] = useState(0);

  useEffect(() => {
    const listener = () => setTick((t) => t + 1);
    listeners.add(listener);
    return () => {
      listeners.delete(listener);
    };
  }, []);

  return id === null ? undefined : bodies.get(routineKey(id));
}
