import type { RoutineIndex } from '../schemas/schemas';

/**
 * Type guards for discriminated unions in the application.
 * These functions help TypeScript narrow types safely without using 'any'.
 */

// RoutineIndex type guards
export function isInternalRoutineIndex(idx: RoutineIndex): idx is { type: 'Internal'; value: number } {
  return idx.type === 'Internal';
}

export function isFunctionRoutineIndex(idx: RoutineIndex): idx is { type: 'Function'; value: number } {
  return idx.type === 'Function';
}

export function isCustomRoutineIndex(idx: RoutineIndex): idx is { type: 'Custom'; value: number } {
  return idx.type === 'Custom';
}
