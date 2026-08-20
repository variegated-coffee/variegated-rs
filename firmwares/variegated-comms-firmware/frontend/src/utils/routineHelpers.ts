import {
  MachineDefinition,
  PeripheralStatus,
  RoutineIndex,
  RoutineParameter,
  RoutinePrerequisite,
  RoutineSummary,
  RoutineSummaryStorage,
  SensorCapability
} from '../schemas/schemas';

// Helper type for routine identification
export type RoutineIdentifier = {
  type: 'internal' | 'function' | 'custom';
  index: number;
};

/**
 * The routine encoding this bundle speaks.
 *
 * Duplicated from `ROUTINE_FORMAT_VERSION` in `variegated-controller-types`, because the
 * generated schemas describe shapes and not constants. If the two disagree the machine
 * answers 400 on every save, which is the loud failure and the intended one -- the quiet
 * alternative would be a routine stored in a format nothing can read back.
 */
export const ROUTINE_FORMAT_VERSION = 5;

/**
 * How many static shot attributes a routine may carry. Mirrors
 * `MAX_ROUTINE_SHOT_ANNOTATIONS`; see that constant for why it is four and not eight.
 */
export const MAX_ROUTINE_SHOT_ANNOTATIONS = 4;

/**
 * Whether a parameter mirrors a given shot attribute.
 */
export function isLinkedTo(parameter: RoutineParameter, key: string): boolean {
  return parameter.linked_attribute?.type === key;
}

/**
 * Whether this machine can currently do this kind of sensing, from any source.
 *
 * The TypeScript twin of `variegated-controller-lib::routine_prerequisites`, and it has to
 * stay one: this decides whether a card is greyed, and the machine decides whether the run
 * is refused. A disagreement shows up as a button that does nothing.
 *
 * Two kinds of source, with different liveness rules — see the Rust doc for the full
 * reasoning. A *peripheral* connects and disconnects, so it counts only while
 * `PeripheralStatus` says it is answering. A sensor declared on a boiler, group, water tap
 * or tank is soldered to the board and counts whenever it is declared.
 *
 * Checking only peripherals makes `OutputFlowRate`, `InputFlowRate` and `WaterLevel`
 * permanently unsatisfiable on both machines in this tree, because those live on the group
 * and the tank.
 */
export function capabilityAvailable(
  capability: SensorCapability,
  definition: MachineDefinition | null,
  peripherals: PeripheralStatus | null
): boolean {
  if (!definition || !peripherals) return true;

  for (const [id, peripheral] of definition.peripherals ?? []) {
    if (!peripheral.capabilities?.some(c => c.type === capability.type)) continue;
    if (peripherals.peripherals?.get(id)?.is_available) return true;
  }

  const declares = (sensors: SensorCapability[] | undefined) =>
    sensors?.some(c => c.type === capability.type) ?? false;

  for (const group of [definition.boilers, definition.groups, definition.water_taps, definition.tanks]) {
    for (const [, component] of group ?? []) {
      if (declares(component.sensors)) return true;
    }
  }
  return false;
}

/**
 * The prerequisites a routine declares that the machine cannot currently meet.
 *
 * Empty means it can run. Returned rather than a boolean so a card can say *what* is
 * missing -- "needs a scale" is actionable where "unavailable" is not.
 */
export function unmetPrerequisites(
  prerequisites: RoutinePrerequisite[] | undefined,
  definition: MachineDefinition | null,
  peripherals: PeripheralStatus | null
): RoutinePrerequisite[] {
  return (prerequisites ?? []).filter(
    p => !capabilityAvailable(p.capability, definition, peripherals)
  );
}

/**
 * A short, human label for a capability, for "needs a scale"-style messages.
 *
 * Exhaustive with no `default`, deliberately: a new `SensorCapability` is then a compile
 * error here rather than a raw enum name leaking into the UI. The same rule the Rust side
 * uses for `StateCondition` in `routine_progress`.
 */
export function capabilityLabel(capability: SensorCapability): string {
  switch (capability.type) {
    case 'Weight':
      return 'a scale';
    case 'ElectricalConductivity':
      return 'a conductivity probe';
    case 'OutputFlowRate':
      return 'output flow sensing';
    case 'InputFlowRate':
      return 'input flow sensing';
    case 'WaterLevel':
      return 'a tank level sensor';
    case 'Temperature':
      return 'a temperature sensor';
    case 'Pressure':
      return 'a pressure sensor';
  }
}

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

// `buildRunRoutineUrl` was here, pointing at `POST /command/run-routine/{type}/{index}`. It had
// **no callers** — running a routine has gone through `ws.runRoutine()` since the WebSocket
// grew a method for it, and the route it named is gone.

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
