import { useState } from 'preact/hooks';
import { Alert, Button, Field, Select, tokens } from '@variegated-coffee/ui';
import { BoilerControlMode } from '../schemas/schemas';
import { NumberField } from './NumberField';

interface BoilerControlEditorProps {
  name: string;
  currentMode: BoilerControlMode;
  currentTargetTemperature: number;
  currentTargetPressure: number;
  onSave: (data: { mode: BoilerControlMode; target_temperature?: number; target_pressure?: number }) => void;
  onCancel: () => void;
}

/*
 * The ranges the firmware will accept.
 *
 * Named rather than repeated in the validator and the input attributes, which is how the
 * two came to be stated twice in the old version -- `min`/`max` on the input for the
 * spinner, and the same numbers again inside `handleSave`.
 */
const TEMPERATURE_MAX = 200;
const PRESSURE_MAX = 20;

export const BoilerControlEditor = ({
  name,
  currentMode,
  currentTargetTemperature,
  currentTargetPressure,
  onSave,
  onCancel
}: BoilerControlEditorProps) => {
  const [mode, setMode] = useState<BoilerControlMode>(currentMode);
  const [targetTemperature, setTargetTemperature] = useState(currentTargetTemperature);
  const [targetPressure, setTargetPressure] = useState(currentTargetPressure);
  const [invalidFields, setInvalidFields] = useState<Record<string, true>>({});

  const invalid = Object.keys(invalidFields).length > 0;

  const validity = (key: string) => (valid: boolean) =>
    setInvalidFields((prev) => {
      if (valid) {
        const { [key]: _unused, ...rest } = prev;
        return rest;
      }
      return { ...prev, [key]: true };
    });

  const handleSave = () => {
    // Validation now lives on the fields, so by the time Save is reachable both values are
    // in range. It used to run here, which meant a bad entry was only reported after the
    // button was pressed -- and reported at the top of the form rather than at the field.
    if (invalid) return;
    onSave({
      mode,
      target_temperature: targetTemperature,
      target_pressure: targetPressure
    });
  };

  return (
    <div
      style={{
        display: 'flex',
        flexDirection: 'column',
        gap: tokens.space.lg,
        padding: tokens.space.lg,
        backgroundColor: tokens.color.surfaceSunken,
        borderRadius: tokens.radius.md,
        border: `1px solid ${tokens.color.border}`,
      }}
    >
      <h3 style={{ marginTop: 0, marginBottom: 0, fontSize: '1.2rem' }}>
        Boiler control — {name}
      </h3>

      <Field label="Control mode" help="What this boiler holds steady.">
        {(control) => (
          <Select
            {...control}
            value={mode.type}
            onChange={(value) => setMode({ type: value } as BoilerControlMode)}
            options={[
              { value: 'Temperature', label: 'Temperature control' },
              { value: 'Pressure', label: 'Pressure control' },
              { value: 'Off', label: 'Off' },
            ]}
          />
        )}
      </Field>

      {/* Both targets stay visible whichever mode is selected, because switching modes is
          how you use them and hiding one would lose an edit in progress. Which one is in
          use is said in the help text rather than by disabling the other. */}
      <NumberField
        label="Target temperature"
        unit="°C"
        value={targetTemperature}
        onChange={setTargetTemperature}
        onValidityChange={validity('temperature')}
        min={0}
        max={TEMPERATURE_MAX}
        help={
          mode.type === 'Temperature'
            ? 'In use — this is what the boiler holds.'
            : 'Kept for when the mode is set back to Temperature.'
        }
      />

      <NumberField
        label="Target pressure"
        unit="bar"
        value={targetPressure}
        onChange={setTargetPressure}
        onValidityChange={validity('pressure')}
        min={0}
        max={PRESSURE_MAX}
        help={
          mode.type === 'Pressure'
            ? 'In use — this is what the boiler holds.'
            : 'Kept for when the mode is set back to Pressure.'
        }
      />

      {invalid && <Alert role="danger">Fix the fields marked above before saving.</Alert>}

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
