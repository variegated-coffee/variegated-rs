import { useState } from 'preact/hooks';
import { Alert, Button, Field, TextInput, tokens } from '@variegated-coffee/ui';
import { PidParameters, Limits } from '../schemas/schemas';

interface PidParametersEditorProps {
  title: string;
  parameters: PidParameters;
  onSave: (params: PidParameters) => void;
  onCancel: () => void;
  /**
   * What the gains act on, so each field can say what it is in.
   *
   * A PID gain is not dimensionless: `kp` for a temperature loop is percent output per
   * degree of error. Twelve near-identical fields per boiler said nothing about their
   * units, which was half of finding 10.
   */
  errorUnit?: string;
  outputUnit?: string;
}

// Editable representation of limits - allows null during editing
type EditableLimits = {
  upper: number | null;
  lower: number | null;
};

// Convert schema Limits to editable form (Infinity becomes null for UI)
const fromLimits = (limits: Limits | null | undefined): EditableLimits => {
  if (!limits) {
    return { upper: null, lower: null };
  }
  return {
    upper: limits.upper === Infinity ? null : limits.upper,
    lower: limits.lower === -Infinity ? null : limits.lower
  };
};

/**
 * Back to the schema's form, where an absent bound is an infinity.
 *
 * # The bug this replaces
 *
 * The previous version required **both** bounds to be finite and threw away *both* if only
 * one was: a term clamped to "at most 40, no lower bound" was stored as unbounded in both
 * directions. That is a silent loss of a safety limit on a boiler — the field kept showing
 * 40 until the next configuration push replaced it, so nothing on screen said the limit had
 * gone.
 *
 * The two bounds are independent, so they convert independently.
 */
const toLimits = (editable: EditableLimits): Limits => ({
  upper: editable.upper !== null && isFinite(editable.upper) ? editable.upper : Infinity,
  lower: editable.lower !== null && isFinite(editable.lower) ? editable.lower : -Infinity,
});

// Editable representation of a PID term
type EditablePidTerm = {
  positive_scale: number;
  negative_scale: number;
  limits: EditableLimits;
};

// Editable representation of PID parameters
type EditablePidParameters = {
  kp: EditablePidTerm;
  ki: EditablePidTerm;
  kd: EditablePidTerm;
};

// Convert schema parameters to editable form
const fromParameters = (params: PidParameters): EditablePidParameters => ({
  kp: {
    positive_scale: params.kp.positive_scale ?? 0,
    negative_scale: params.kp.negative_scale ?? 0,
    limits: fromLimits(params.kp.limits)
  },
  ki: {
    positive_scale: params.ki.positive_scale ?? 0,
    negative_scale: params.ki.negative_scale ?? 0,
    limits: fromLimits(params.ki.limits)
  },
  kd: {
    positive_scale: params.kd.positive_scale ?? 0,
    negative_scale: params.kd.negative_scale ?? 0,
    limits: fromLimits(params.kd.limits)
  }
});

// Convert editable parameters back to schema form
const toParameters = (editable: EditablePidParameters): PidParameters => ({
  kp: {
    positive_scale: editable.kp.positive_scale,
    negative_scale: editable.kp.negative_scale,
    limits: toLimits(editable.kp.limits)
  },
  ki: {
    positive_scale: editable.ki.positive_scale,
    negative_scale: editable.ki.negative_scale,
    limits: toLimits(editable.ki.limits)
  },
  kd: {
    positive_scale: editable.kd.positive_scale,
    negative_scale: editable.kd.negative_scale,
    limits: toLimits(editable.kd.limits)
  }
});

/**
 * Parse a gain the user typed.
 *
 * Returns `null` for anything unparseable rather than falling back to a number. The old
 * `parseFloat(value) || 0` turned a typo into a **zero gain** and saved it: an integral
 * term silently switched off, on a boiler, with the field showing `0` as though that had
 * been the intent. `null` here becomes a validation error at the field instead.
 *
 * Note this also catches the `|| 0` operator's other victim: `parseFloat("0")` is `0`,
 * which is falsy, so the old code took the fallback branch for a legitimately-typed zero
 * too. Same result by luck rather than by design.
 */
function parseNumber(raw: string): number | null {
  const trimmed = raw.trim();
  if (trimmed === '') return null;
  const parsed = Number.parseFloat(trimmed);
  return Number.isFinite(parsed) ? parsed : null;
}

export const PidParametersEditor = ({
  title,
  parameters,
  onSave,
  onCancel,
  errorUnit,
  outputUnit,
}: PidParametersEditorProps) => {
  // Use editable types for state management
  const [localParams, setLocalParams] = useState<EditablePidParameters>(
    fromParameters(parameters)
  );
  /*
   * What is in each box, keyed by field.
   *
   * Held as text rather than as numbers for the whole time the field is focused, because a
   * half-typed `0.` or `-` is not a number and reformatting it into one moves the caret.
   * The committed value in `localParams` is only updated when the text parses.
   */
  const [editingValues, setEditingValues] = useState<Record<string, string>>({});
  const [errors, setErrors] = useState<Record<string, string>>({});

  const invalid = Object.keys(errors).length > 0;

  const handleSave = () => {
    if (invalid) return;
    onSave(toParameters(localParams));
  };

  const updateTerm = (
    term: 'kp' | 'ki' | 'kd',
    field: 'positive_scale' | 'negative_scale',
    value: number
  ) => {
    setLocalParams((prev) => ({ ...prev, [term]: { ...prev[term], [field]: value } }));
  };

  const updateLimit = (term: 'kp' | 'ki' | 'kd', subfield: 'upper' | 'lower', value: number | null) => {
    setLocalParams((prev) => ({
      ...prev,
      [term]: { ...prev[term], limits: { ...prev[term].limits, [subfield]: value } },
    }));
  };

  const setError = (key: string, message: string | null) => {
    setErrors((prev) => {
      if (message === null) {
        const { [key]: _unused, ...rest } = prev;
        return rest;
      }
      return { ...prev, [key]: message };
    });
  };

  const renderTermEditor = (
    termKey: 'kp' | 'ki' | 'kd',
    termName: string,
    term: EditablePidTerm
  ) => {
    /**
     * A gain field. Required, so an empty box is an error rather than a zero.
     *
     * `numeric` on `TextInput` is what fixes finding 04: it renders `type="text"` with
     * `inputMode="decimal"` and monospace tabular figures. The old `type="number"` was
     * formatted and parsed by the *browser's* locale, so `1.4` displayed as `1,4` beside a
     * limit reading `100` — the same firmware value shown two ways on two machines in the
     * same kitchen.
     */
    const gainField = (field: 'positive_scale' | 'negative_scale', label: string, help: string) => {
      const key = `${termKey}_${field}`;
      return (
        <Field
          label={label}
          help={errors[key] ? undefined : help}
          error={errors[key]}
          unit={errorUnit && outputUnit ? `${outputUnit}/${errorUnit}` : undefined}
          required
        >
          {(control) => (
            <TextInput
              {...control}
              numeric
              value={editingValues[key] ?? String(term[field])}
              onInput={(value) => {
                setEditingValues((prev) => ({ ...prev, [key]: value }));
                const parsed = parseNumber(value);
                if (parsed === null) {
                  setError(key, 'Enter a number, using a full stop for the decimal point.');
                } else {
                  setError(key, null);
                  updateTerm(termKey, field, parsed);
                }
              }}
              onBlur={() => {
                // Give the box back to the committed value, so a valid but oddly-typed
                // entry (`.5`, `1.`) settles into its canonical form once focus leaves.
                setEditingValues((prev) => {
                  const { [key]: _unused, ...rest } = prev;
                  return rest;
                });
              }}
            />
          )}
        </Field>
      );
    };

    /** A limit field. Optional, and empty genuinely means unbounded. */
    const limitField = (subfield: 'upper' | 'lower', label: string) => {
      const key = `${termKey}_limits_${subfield}`;
      const committed = term.limits[subfield];
      return (
        <Field
          label={label}
          // The help moves below the control. "Upper Limit (empty = no limit)" put it
          // inside the label, where it wrapped onto a second line and pushed the label
          // away from its own input.
          help={errors[key] ? undefined : 'Empty means no limit.'}
          error={errors[key]}
          unit={outputUnit}
        >
          {(control) => (
            <TextInput
              {...control}
              numeric
              value={editingValues[key] ?? (committed === null ? '' : String(committed))}
              onInput={(value) => {
                setEditingValues((prev) => ({ ...prev, [key]: value }));
                if (value.trim() === '') {
                  setError(key, null);
                  updateLimit(termKey, subfield, null);
                  return;
                }
                const parsed = parseNumber(value);
                if (parsed === null) {
                  setError(key, 'Enter a number, or leave empty for no limit.');
                } else {
                  setError(key, null);
                  updateLimit(termKey, subfield, parsed);
                }
              }}
              onBlur={() => {
                setEditingValues((prev) => {
                  const { [key]: _unused, ...rest } = prev;
                  return rest;
                });
              }}
            />
          )}
        </Field>
      );
    };

    return (
      <div
        style={{
          marginBottom: tokens.space.lg,
          padding: tokens.space.md,
          backgroundColor: tokens.color.surfaceSunken,
          borderRadius: tokens.radius.md,
          border: `1px solid ${tokens.color.border}`,
        }}
      >
        <h4 style={{ marginTop: 0, marginBottom: tokens.space.sm, fontSize: '0.95rem' }}>
          {termName}
        </h4>

        <div
          style={{
            display: 'grid',
            gridTemplateColumns: 'repeat(auto-fit, minmax(11rem, 1fr))',
            gap: tokens.space.sm,
          }}
        >
          {/* The asymmetry is the reason these are two fields rather than one gain, so it
              is worth saying which direction each acts in -- a boiler heats but cannot
              cool, so the negative scale is usually zero and is not a typo. */}
          {gainField('positive_scale', 'Positive scale', 'Applied when the reading is below target.')}
          {gainField('negative_scale', 'Negative scale', 'Applied when the reading is above target.')}
          {limitField('upper', 'Upper limit')}
          {limitField('lower', 'Lower limit')}
        </div>
      </div>
    );
  };

  return (
    <div>
      <h3 style={{ marginTop: 0, marginBottom: tokens.space.md }}>{title}</h3>

      {renderTermEditor('kp', 'Proportional (Kp)', localParams.kp)}
      {renderTermEditor('ki', 'Integral (Ki)', localParams.ki)}
      {renderTermEditor('kd', 'Derivative (Kd)', localParams.kd)}

      {invalid && (
        <div style={{ marginBottom: tokens.space.md }}>
          <Alert role="danger">
            Some fields do not hold a number. Fix them before saving — these gains go
            straight to a boiler's control loop.
          </Alert>
        </div>
      )}

      <div style={{ display: 'flex', gap: tokens.space.sm, justifyContent: 'flex-end' }}>
        <Button variant="secondary" onClick={onCancel}>
          Cancel
        </Button>
        <Button variant="primary" onClick={handleSave} disabled={invalid}>
          Save changes
        </Button>
      </div>
    </div>
  );
};
