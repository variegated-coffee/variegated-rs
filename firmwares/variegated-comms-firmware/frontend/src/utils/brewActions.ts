/**
 * What the machine does to a group's scale when a brew starts.
 *
 * `BrewActions` is a newtype over `u8` in the firmware, so it arrives here as a plain number
 * and the bits have to be spelled out. They are the same two the panel's `Settings > Scale`
 * rows drive, and the same byte `auto_tare_enabled` used to occupy.
 *
 * Kept together in one module rather than inlined at each call site: three places read this
 * set -- the group card, the group detail and the editor -- and a bit index copied three times
 * is a bit index that eventually disagrees with itself.
 */

/** Zero the scale. Bit 0. */
export const BREW_ACTION_TARE = 1 << 0;
/** Return the scale's own timer to zero and start it running. Bit 1. */
export const BREW_ACTION_RESET_AND_START_TIMER = 1 << 1;

/** Whether the set includes an action. */
export function hasBrewAction(actions: number, action: number): boolean {
  return (actions & action) === action;
}

/** The same set with an action added or removed. */
export function withBrewAction(actions: number, action: number, on: boolean): number {
  return on ? actions | action : actions & ~action;
}

/**
 * The set as a sentence, for a row that summarises rather than edits.
 *
 * "Nothing" rather than an empty string for the empty set: a description that disappears reads
 * as a value that failed to load, and doing nothing to the scale is a real choice here.
 */
export function describeBrewActions(actions: number): string {
  const chosen: string[] = [];
  if (hasBrewAction(actions, BREW_ACTION_TARE)) chosen.push('Tare');
  if (hasBrewAction(actions, BREW_ACTION_RESET_AND_START_TIMER)) {
    chosen.push('Reset and start timer');
  }
  return chosen.length > 0 ? chosen.join(', ') : 'Nothing';
}
