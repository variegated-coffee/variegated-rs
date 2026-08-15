import { RoutineIndex, RoutineSummary, RoutineSummaryStorage } from '../schemas/schemas';

// Helper type for routine identification
export type RoutineIdentifier = {
  type: 'internal' | 'function' | 'custom';
  index: number;
};

/**
 * Get a routine's *summary* from storage using a routine identifier.
 *
 * Summaries, not definitions: the list the machine pushes carries names, types and counts.
 * A definition comes from `api/routines.ts` and lives in `state/routineBodies.ts`.
 */
export function getRoutineSummary(
  storage: RoutineSummaryStorage,
  identifier: RoutineIdentifier
): RoutineSummary | undefined {
  switch (identifier.type) {
    case 'internal':
      return storage.internal?.get(identifier.index);
    case 'function':
      return storage.function?.get(identifier.index);
    case 'custom':
      return storage.custom?.get(identifier.index);
  }
}

/**
 * Get a routine's summary from a RoutineIndex, as carried in the Status.
 */
export function getRoutineSummaryFromIndex(
  storage: RoutineSummaryStorage,
  routineIndex: RoutineIndex
): RoutineSummary | undefined {
  const identifier = identifierFromIndex(routineIndex);
  return identifier === null ? undefined : getRoutineSummary(storage, identifier);
}

/**
 * The path-shaped identifier for a wire `RoutineIndex`.
 *
 * The two spellings exist because one is the postcard enum and the other is a URL
 * segment. This is the single place they are related, so a fourth index kind is one edit.
 */
export function identifierFromIndex(routineIndex: RoutineIndex): RoutineIdentifier | null {
  if (routineIndex.type === 'Internal') return { type: 'internal', index: routineIndex.value };
  if (routineIndex.type === 'Function') return { type: 'function', index: routineIndex.value };
  if (routineIndex.type === 'Custom') return { type: 'custom', index: routineIndex.value };
  return null;
}

/** The wire `RoutineIndex` for a path-shaped identifier. */
export function indexFromIdentifier(identifier: RoutineIdentifier): RoutineIndex {
  if (identifier.type === 'internal') return { type: 'Internal', value: identifier.index };
  if (identifier.type === 'function') return { type: 'Function', value: identifier.index };
  return { type: 'Custom', value: identifier.index };
}

/**
 * Where a routine's definition is fetched, saved and deleted.
 *
 * `GET` streams the definition, `PUT` replaces it, `DELETE` removes it. Creating uses
 * `/routines/custom` with no index, because the machine assigns it.
 */
export function buildRoutineUrl(identifier: RoutineIdentifier): string {
  return `/routines/${identifier.type}/${identifier.index}`;
}

/**
 * Build a URL for running a routine
 */
export function buildRunRoutineUrl(identifier: RoutineIdentifier): string {
  return `/command/run-routine/${identifier.type}/${identifier.index}`;
}

/**
 * Get the display name for a routine type
 */
export function getRoutineTypeLabel(type: 'internal' | 'function' | 'custom'): string {
  switch (type) {
    case 'internal':
      return 'Internal';
    case 'function':
      return 'Function';
    case 'custom':
      return 'Custom';
  }
}

/**
 * Get the display label for a RoutineIndex
 */
export function getRoutineIndexLabel(routineIndex: RoutineIndex): string {
  if (routineIndex.type === 'Internal') {
    return `Internal #${routineIndex.value}`;
  } else if (routineIndex.type === 'Function') {
    return `Function #${routineIndex.value}`;
  } else if (routineIndex.type === 'Custom') {
    return `Custom #${routineIndex.value}`;
  }
  return 'Unknown';
}
