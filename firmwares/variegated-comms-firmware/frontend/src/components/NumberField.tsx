import { useState } from 'preact/hooks';
import { Field, TextInput } from '@variegated-coffee/ui';

/**
 * A number the firmware will act on, typed by a human.
 *
 * Six editors in this frontend had their own copy of this, and every copy had the same two
 * defects:
 *
 * - **`type="number"`**, which the *browser* renders and parses in its own locale. A PID
 *   gain of `1.4` displayed as `1,4` next to a limit reading `100`, and the same firmware
 *   value looked different on two machines in the same kitchen. `TextInput`'s `numeric`
 *   gives a text input with `inputMode="decimal"` and monospace tabular figures instead.
 * - **`parseFloat(value) || 0`**, which silently substitutes zero for anything it cannot
 *   parse — a typo turning an integral gain off, or a Kalman measurement-noise term to
 *   zero, with the field showing `0` as though that had been the intent. Here an
 *   unparseable entry is an error on the field and the last good value is kept, so nothing
 *   invalid can reach the machine.
 *
 * The draft text is held separately from the committed number for the whole time the field
 * is focused, because a half-typed `0.` or `-` is not a number and reformatting it into one
 * moves the caret.
 */
export interface NumberFieldProps {
  label: string;
  value: number;
  onChange: (value: number) => void;
  /** Rendered below the control, never inside the label. */
  help?: string;
  /** °C, bar, %, mL/s — whatever the firmware is actually parsing. */
  unit?: string;
  /** Rejected below this, rather than clamped: a clamp hides that the entry was wrong. */
  min?: number;
  max?: number;
  disabled?: boolean;
  /**
   * Told to the parent whenever validity changes, so a form can disable Save.
   *
   * Without this the field can be left in an invalid state and the *last good* value saved
   * silently — which is the `|| 0` bug wearing a different hat.
   */
  onValidityChange?: (valid: boolean) => void;
}

export function NumberField({
  label,
  value,
  onChange,
  help,
  unit,
  min,
  max,
  disabled = false,
  onValidityChange,
}: NumberFieldProps) {
  const [draft, setDraft] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);

  const fail = (message: string) => {
    setError(message);
    onValidityChange?.(false);
  };

  const succeed = (parsed: number) => {
    setError(null);
    onValidityChange?.(true);
    onChange(parsed);
  };

  const handleInput = (raw: string) => {
    setDraft(raw);
    const trimmed = raw.trim();

    if (trimmed === '') {
      fail('Enter a number.');
      return;
    }

    const parsed = Number.parseFloat(trimmed);
    if (!Number.isFinite(parsed)) {
      fail('Enter a number, using a full stop for the decimal point.');
      return;
    }
    if (min !== undefined && parsed < min) {
      fail(`Must be ${min} or more.`);
      return;
    }
    if (max !== undefined && parsed > max) {
      fail(`Must be ${max} or less.`);
      return;
    }

    succeed(parsed);
  };

  return (
    <Field label={label} help={error ? undefined : help} error={error ?? undefined} unit={unit}>
      {(control) => (
        <TextInput
          {...control}
          numeric
          disabled={disabled}
          value={draft ?? String(value)}
          onInput={handleInput}
          // Hand the box back to the committed value once focus leaves, so a valid but
          // oddly-typed entry (`.5`, `1.`) settles into its canonical form. A field left
          // in error keeps its text, so the mistake stays visible next to the message.
          onBlur={() => {
            if (!error) setDraft(null);
          }}
        />
      )}
    </Field>
  );
}
