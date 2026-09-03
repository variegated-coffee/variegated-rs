import { useState } from 'preact/hooks';
import { Button, tokens } from '@variegated-coffee/ui';
import {
  BREW_ACTION_RESET_AND_START_TIMER,
  BREW_ACTION_SYNC_DOSE,
  BREW_ACTION_TARE,
  hasBrewAction,
  withBrewAction,
} from '../utils/brewActions';

interface BrewActionsEditorProps {
  /** The set as stored, a bitfield over `BREW_ACTION_*`. */
  actions: number;
  onSave: (actions: number) => void;
  onCancel: () => void;
}

/**
 * The actions, their labels and the caveat each one carries.
 *
 * A module-level descriptor array rather than markup per row, copying `ContextTab`'s
 * prerequisite list: the generated schema gives us a number and nothing else, so the human
 * labels have to live somewhere, and somewhere is better than three times.
 */
const ACTIONS: { value: number; label: string; hint: string }[] = [
  {
    value: BREW_ACTION_TARE,
    label: 'Tare',
    hint: 'zero the scale, so the shot is weighed from nothing',
  },
  {
    value: BREW_ACTION_RESET_AND_START_TIMER,
    label: 'Reset and start timer',
    hint: "the scale's own timer, on its display. Scales without one ignore this",
  },
  {
    value: BREW_ACTION_SYNC_DOSE,
    label: 'Sync dose',
    hint: 'send the dose to the scale so it can show a ratio. Only a BooKoo Themis Ultra accepts this, and the machine cannot tell one from a Mini — so it is offered here and quietly ignored by scales that lack it',
  },
];

/**
 * Which things happen to the group's scale when a shot starts.
 *
 * Checkboxes rather than a mode select, because the two are genuinely independent: a machine
 * may reasonably do both, one, or neither, and flattening that into four named modes would
 * read as a choice between alternatives and grow badly the moment a third action exists.
 */
export const BrewActionsEditor = ({ actions, onSave, onCancel }: BrewActionsEditorProps) => {
  const [local, setLocal] = useState(actions);

  const toggle = (action: number) => {
    setLocal(current => withBrewAction(current, action, !hasBrewAction(current, action)));
  };

  return (
    <div>
      <p style={{ color: tokens.color.inkMuted, fontSize: '0.9rem', marginTop: 0 }}>
        What the machine does to the scale the moment a shot starts. This is saved on the
        machine and applies to every shot, including the ones a routine starts.
      </p>

      <fieldset
        style={{
          border: `1px solid ${tokens.color.border}`,
          borderRadius: tokens.radius.sm,
          padding: tokens.space.md,
        }}
      >
        <legend style={{ padding: `0 ${tokens.space.xs}`, fontSize: '0.9rem' }}>
          When a brew starts
        </legend>
        {ACTIONS.map(action => (
          <label
            key={action.value}
            style={{
              display: 'flex',
              alignItems: 'baseline',
              gap: tokens.space.sm,
              marginBottom: tokens.space.sm,
              cursor: 'pointer',
            }}
          >
            <input
              type="checkbox"
              checked={hasBrewAction(local, action.value)}
              onChange={() => toggle(action.value)}
            />
            <span>
              {action.label}
              <span style={{ color: tokens.color.inkMuted, fontSize: '0.85rem' }}>
                {' '}&mdash; {action.hint}
              </span>
            </span>
          </label>
        ))}
      </fieldset>

      {/*
        No warning for the empty set. Doing nothing to the scale is a real choice -- someone
        who tares by hand before every shot wants exactly that -- and an alert on a valid
        selection is an alert people learn to click past.
      */}

      <div
        style={{
          display: 'flex',
          gap: tokens.space.sm,
          justifyContent: 'flex-end',
          marginTop: tokens.space.lg,
        }}
      >
        <Button variant="secondary" onClick={onCancel}>
          Cancel
        </Button>
        <Button variant="primary" onClick={() => onSave(local)}>
          Save changes
        </Button>
      </div>
    </div>
  );
};
