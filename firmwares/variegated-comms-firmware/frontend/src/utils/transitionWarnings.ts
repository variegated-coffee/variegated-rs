import { RoutineCommand, RoutineStep } from '../schemas/schemas';

/**
 * Static checks on a routine's transitions.
 *
 * # What this is for
 *
 * `TransitionOrigin.CurrentTarget` means "start the ramp where the setpoint already is".
 * That is well defined only while the group is being controlled in the *same quantity*: a
 * duty cycle of 50 is not 50 bar, so a pressure transition entered from duty control has no
 * pressure setpoint to continue from.
 *
 * The firmware handles it -- it falls through to the measured pressure, which is the only
 * honest answer to "where am I, in pressure", and keeps the ramp. But it is a step whose
 * behaviour depends on how it was entered, and that is worth seeing while writing the
 * routine rather than discovering in a shot log.
 *
 * # What it cannot know
 *
 * Which step actually ran before this one. The analysis follows fall-through and
 * `JumpToStep`, so a step reachable two ways is checked against both; but an exit whose
 * condition never fires in practice still counts as a path, so a warning here is "this can
 * happen", not "this will".
 */

/** The quantity a group is being controlled in, as far as this analysis cares. */
type Quantity = 'flow rate' | 'pressure' | 'output flow rate' | 'duty cycle' | 'nothing';

export interface TransitionWarning {
  /** Index into `routine.steps`. */
  step: number;
  /** What the transition is ramping. */
  quantity: Quantity;
  /** What the group is controlling when the step begins, where that is knowable. */
  incoming: Quantity[];
  message: string;
}

/** What a command leaves the group controlling, or `null` if it does not touch that. */
function quantityAfter(command: RoutineCommand): Quantity | null {
  switch (command.type) {
    case 'SetGroupFlowRate':
    case 'SetGroupFlowRateWithTransition':
      return 'flow rate';
    case 'SetGroupPressure':
    case 'SetGroupPressureWithTransition':
      return 'pressure';
    case 'SetGroupOutputFlowRate':
    case 'SetGroupOutputFlowRateWithTransition':
      return 'output flow rate';
    case 'SetGroupFixedDutyCycle':
    case 'SetGroupFixedDutyCycleWithTransition':
      return 'duty cycle';
    // Both leave the group in a mode that is not any of the four quantities, so a
    // `CurrentTarget` transition after one of these has nothing to continue from.
    case 'SetGroupFullOn':
    case 'SetGroupOff':
      return 'nothing';
    default:
      return null;
  }
}

/** The quantity a transition command ramps, or `null` if it is not a transition. */
function transitionQuantity(command: RoutineCommand): Quantity | null {
  switch (command.type) {
    case 'SetGroupFlowRateWithTransition':
      return 'flow rate';
    case 'SetGroupPressureWithTransition':
      return 'pressure';
    case 'SetGroupOutputFlowRateWithTransition':
      return 'output flow rate';
    case 'SetGroupFixedDutyCycleWithTransition':
      return 'duty cycle';
    default:
      return null;
  }
}

/** A transition's declared origin, or `null` if the command is not a transition. */
function originOf(command: RoutineCommand): string | null {
  if (transitionQuantity(command) === null) return null;
  // Every transition variant carries the origin as its fourth tuple element, and the four
  // are structurally identical -- so reading it positionally costs one cast and saves four
  // near-identical branches that would have to be kept in step with each other.
  const value = (command as { value: unknown[] }).value;
  const origin = value[3] as { type: string } | undefined;
  return origin?.type ?? null;
}

/** What a step leaves the group controlling, or `null` if it changes nothing. */
function stepOutgoing(step: RoutineStep): Quantity | null {
  let quantity: Quantity | null = null;
  for (const command of step.entry_command) {
    const next = quantityAfter(command);
    if (next !== null) quantity = next;
  }
  return quantity;
}

/** Which steps can run immediately before `target`. */
function predecessors(steps: RoutineStep[], target: number): number[] {
  const found: number[] = [];
  steps.forEach((step, index) => {
    for (const exit of step.exits) {
      if (exit.then.type === 'NextStep' && index + 1 === target) found.push(index);
      if (exit.then.type === 'JumpToStep' && exit.then.value === target) found.push(index);
    }
  });
  return [...new Set(found)];
}

/**
 * What the group is controlling when `step` begins, over every path that reaches it.
 *
 * Walks back through predecessors until each path finds a step that sets a quantity. The
 * visited set is what stops a routine whose steps loop -- which is ordinary, since
 * `JumpToStep` is how a repeat is written -- from recursing forever.
 */
function incomingQuantities(
  steps: RoutineStep[],
  step: number,
  visited: Set<number> = new Set(),
): Quantity[] {
  if (visited.has(step)) return [];
  visited.add(step);

  // Step 0 is entered from a machine that is not brewing yet.
  const previous = predecessors(steps, step);
  if (step === 0 && previous.length === 0) return ['nothing'];

  const quantities: Quantity[] = [];
  for (const index of previous) {
    const outgoing = stepOutgoing(steps[index]);
    if (outgoing !== null) {
      quantities.push(outgoing);
    } else {
      quantities.push(...incomingQuantities(steps, index, visited));
    }
  }

  // Unreachable except from the start, or every path looped without setting anything.
  if (quantities.length === 0) return ['nothing'];
  return [...new Set(quantities)];
}

/**
 * Every `CurrentTarget` transition whose origin is not well defined on some path into it.
 *
 * Commands within a step are walked in order, so a step that sets pressure control and
 * *then* ramps pressure is fine however it was entered -- which is the natural way to write
 * a step that does not want to depend on its predecessor.
 */
export function transitionWarnings(steps: RoutineStep[]): TransitionWarning[] {
  const warnings: TransitionWarning[] = [];

  steps.forEach((step, index) => {
    let current: Quantity[] = incomingQuantities(steps, index);

    for (const command of step.entry_command) {
      const quantity = transitionQuantity(command);

      if (quantity !== null && originOf(command) === 'CurrentTarget') {
        const mismatched = current.filter((q) => q !== quantity);
        if (mismatched.length > 0) {
          const controlling = mismatched
            .map((q) => (q === 'nothing' ? 'not being controlled' : `controlling ${q}`))
            .join(' or ');
          warnings.push({
            step: index,
            quantity,
            incoming: current,
            message:
              `This ${quantity} transition starts from the current target, but the group is ` +
              `${controlling} when this step begins — so there is no ${quantity} setpoint to ` +
              `continue from. It will start from the measured ${quantity} instead. Set an ` +
              `explicit start value if that is not what you want.`,
          });
        }
      }

      const after = quantityAfter(command);
      if (after !== null) current = [after];
    }
  });

  return warnings;
}
