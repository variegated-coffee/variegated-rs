import { useState } from 'preact/hooks';
import { Alert, Button, Field, TextInput, tokens } from '@variegated-coffee/ui';
import { PidParameters, Limits } from '../schemas/schemas';
import { NumberField } from './NumberField';

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
  const [invalidFields, setInvalidFields] = useState<Record<string, true>>({});
  /*
   * Limit fields keep their own draft text, because they are the one case `NumberField`
   * does not cover: empty is a legitimate value here (it means unbounded), where every
   * other numeric field in this frontend treats an empty box as an error.
   */
  const [limitDrafts, setLimitDrafts] = useState<Record<string, string>>({});

  const invalid = Object.keys(invalidFields).length > 0;

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

  const validity = (key: string) => (valid: boolean) =>
    setInvalidFields((prev) => {
      if (valid) {
        const { [key]: _unused, ...rest } = prev;
        return rest;
      }
      return { ...prev, [key]: true };
    });

  const renderTermEditor = (
    termKey: 'kp' | 'ki' | 'kd',
    termName: string,
    term: EditablePidTerm
  ) => {
    /** A gain. Required, so an empty box is an error rather than a zero. */
    const gainField = (field: 'positive_scale' | 'negative_scale', label: string, help: string) => (
      <NumberField
        label={label}
        value={term[field]}
        onChange={(value) => updateTerm(termKey, field, value)}
        onValidityChange={validity(`${termKey}_${field}`)}
        help={help}
        unit={errorUnit && outputUnit ? `${outputUnit}/${errorUnit}` : undefined}
      />
    );

    /** A limit. Optional, and an empty box genuinely means unbounded. */
    const limitField = (subfield: 'upper' | 'lower', label: string) => {
      const key = `${termKey}_limits_${subfield}`;
      const committed = term.limits[subfield];
      const invalidHere = invalidFields[key] === true;

      return (
        <Field
          label={label}
          // The help moves below the control. "Upper Limit (empty = no limit)" put it
          // inside the label, where it wrapped onto a second line and pushed the label
          // away from its own input.
          help={invalidHere ? undefined : 'Empty means no limit.'}
          error={invalidHere ? 'Enter a number, or leave empty for no limit.' : undefined}
          unit={outputUnit}
        >
          {(control) => (
            <TextInput
              {...control}
              numeric
              value={limitDrafts[key] ?? (committed === null ? '' : String(committed))}
              onInput={(value) => {
                setLimitDrafts((prev) => ({ ...prev, [key]: value }));
                if (value.trim() === '') {
                  validity(key)(true);
                  updateLimit(termKey, subfield, null);
                  return;
                }
                const parsed = Number.parseFloat(value.trim());
                if (!Number.isFinite(parsed)) {
                  validity(key)(false);
                } else {
                  validity(key)(true);
                  updateLimit(termKey, subfield, parsed);
                }
              }}
              onBlur={() => {
                if (invalidHere) return;
                setLimitDrafts((prev) => {
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
