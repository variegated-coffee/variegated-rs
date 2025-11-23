import { Routine, RoutineIndex, RoutineStorage } from '../schemas/schemas';

// Helper type for routine identification
export type RoutineIdentifier = {
  type: 'internal' | 'function' | 'custom';
  index: number;
};

/**
 * Get a routine from storage using a routine identifier
 */
export function getRoutine(
  storage: RoutineStorage,
  identifier: RoutineIdentifier
): Routine | undefined {
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
 * Get a routine from storage using a RoutineIndex from the Status
 */
export function getRoutineFromIndex(
  storage: RoutineStorage,
  routineIndex: RoutineIndex
): Routine | undefined {
  if (routineIndex.type === 'Internal') {
    return storage.internal?.get(routineIndex.value);
  } else if (routineIndex.type === 'Function') {
    return storage.function?.get(routineIndex.value);
  } else if (routineIndex.type === 'Custom') {
    return storage.custom?.get(routineIndex.value);
  }
  return undefined;
}

/**
 * Build a URL for routine operations (PUT, DELETE)
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

/**
 * Count total routines across all types
 */
export function getTotalRoutineCount(storage: RoutineStorage): number {
  return (storage.internal?.size || 0) +
         (storage.function?.size || 0) +
         (storage.custom?.size || 0);
}

/**
 * Check if a routine exists at the given identifier
 */
export function routineExists(storage: RoutineStorage, identifier: RoutineIdentifier): boolean {
  return getRoutine(storage, identifier) !== undefined;
}
