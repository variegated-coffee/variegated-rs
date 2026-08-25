import { useState } from 'preact/hooks';
import { Alert, Button, Field, Select, tokens } from '@variegated-coffee/ui';
import { GroupBrewControlMode, GroupBrewControlTargetValues, ControlCurve } from '../schemas/schemas';
import { NumberField } from './NumberField';

interface GroupControlEditorProps {
  name: string;
  currentMode: GroupBrewControlMode;
  currentValues: GroupBrewControlTargetValues;
  onSave: (data: {
    mode: GroupBrewControlMode;
    flow_rate?: number | null;
    pressure?: number | null;
    output_flow_rate?: number | null;
    duty_cycle?: number | null;
    duty_cycle_curve?: ControlCurve | null;
    flow_rate_curve?: ControlCurve | null;
    pressure_curve?: ControlCurve | null;
    output_flow_rate_curve?: ControlCurve | null;
  }) => void;
  onCancel: () => void;
}

/*
 * The ten modes, with what each one means beside it.
 *
 * A flat list of ten options named after their enum variants -- "GroupFlowRateCurve" next
 * to "OutputFlowRate" -- makes the reader work out which pairs differ by "which side of
 * the puck" and which by "fixed or curve". Grouping by what is being controlled puts the
 * pairs next to each other, and `Select` renders the groups as optgroups.
 */
const MODES = [
  { value: 'GroupFlowRate', label: 'Fixed target', group: 'Flow into the puck' },
  { value: 'GroupFlowRateCurve', label: 'Follow a curve', group: 'Flow into the puck' },
  { value: 'OutputFlowRate', label: 'Fixed target', group: 'Flow out of the puck' },
  { value: 'OutputFlowRateCurve', label: 'Follow a curve', group: 'Flow out of the puck' },
  { value: 'Pressure', label: 'Fixed target', group: 'Pressure' },
  { value: 'PressureCurve', label: 'Follow a curve', group: 'Pressure' },
  { value: 'FixedDutyCycle', label: 'Fixed duty cycle', group: 'Pump directly' },
  { value: 'FixedDutyCycleCurve', label: 'Follow a curve', group: 'Pump directly' },
  { value: 'FullOn', label: 'Full on', group: 'Pump directly' },
  { value: 'Off', label: 'Off', group: 'Pump directly' },
];

export const GroupControlEditor = ({
  name,
  currentMode,
  currentValues,
  onSave,
  onCancel
}: GroupControlEditorProps) => {
  const [mode, setMode] = useState<GroupBrewControlMode>(currentMode);
  const [flowRate, setFlowRate] = useState(currentValues.flow_rate);
  const [pressure, setPressure] = useState(currentValues.pressure);
  const [outputFlowRate, setOutputFlowRate] = useState(currentValues.output_flow_rate);
  const [dutyCycle, setDutyCycle] = useState(currentValues.duty_cycle);
  const [invalidFields, setInvalidFields] = useState<Record<string, true>>({});

  // Determine which input to show based on mode
  const showFlowRate = mode.type === 'GroupFlowRate';
  const showPressure = mode.type === 'Pressure';
  const showOutputFlowRate = mode.type === 'OutputFlowRate';
  const showDutyCycle = mode.type === 'FixedDutyCycle';
  const isCurveMode = mode.type.includes('Curve');
  const isFullOnOrOff = mode.type === 'FullOn' || mode.type === 'Off';

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
    if (invalid) return;

    // Initialize ALL optional fields to null for postcard serialization
    // (postcard requires null for Option<T>, not undefined)
    const data: {
      mode: GroupBrewControlMode;
      flow_rate?: number | null;
      pressure?: number | null;
      output_flow_rate?: number | null;
      duty_cycle?: number | null;
      duty_cycle_curve?: ControlCurve | null;
      flow_rate_curve?: ControlCurve | null;
      pressure_curve?: ControlCurve | null;
      output_flow_rate_curve?: ControlCurve | null;
    } = {
      mode,
      flow_rate: null,
      pressure: null,
      output_flow_rate: null,
      duty_cycle: null,
      duty_cycle_curve: null,
      flow_rate_curve: null,
      pressure_curve: null,
      output_flow_rate_curve: null
    };

    // Only the value the selected mode actually uses is sent. The ranges that used to be
    // checked here are on the fields now, so a bad entry is reported where it was typed
    // rather than at the top of the form after Save was pressed.
    if (showFlowRate) data.flow_rate = flowRate;
    if (showPressure) data.pressure = pressure;
    if (showOutputFlowRate) data.output_flow_rate = outputFlowRate;
    if (showDutyCycle) data.duty_cycle = dutyCycle;

    onSave(data);
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
        Brew control — {name}
      </h3>

      <Field label="Control mode" help="What the pump is driven by during a shot.">
        {(control) => (
          <Select
            {...control}
            value={mode.type}
            onChange={(value) => setMode({ type: value } as GroupBrewControlMode)}
            options={MODES}
          />
        )}
      </Field>

      {isCurveMode && (
        <Alert role="info">
          This mode follows the configured curve. Edit it from the group's Control curves
          section.
        </Alert>
      )}

      {isFullOnOrOff && (
        <Alert role="info">
          {mode.type === 'FullOn'
            ? 'The pump runs at full duty for the whole shot, with nothing regulating it.'
            : 'The pump stays off.'}
        </Alert>
      )}

      {showFlowRate && (
        <NumberField
          label="Target flow rate"
          unit="mL/s"
          value={flowRate}
          onChange={setFlowRate}
          onValidityChange={validity('flow_rate')}
          min={0}
          max={20}
          help="Measured on the input side, before the puck."
        />
      )}

      {showPressure && (
        <NumberField
          label="Target pressure"
          unit="bar"
          value={pressure}
          onChange={setPressure}
          onValidityChange={validity('pressure')}
          min={0}
          max={20}
          help="Brewing pressure at the group."
        />
      )}

      {showOutputFlowRate && (
        <NumberField
          label="Target output flow rate"
          unit="mL/s"
          value={outputFlowRate}
          onChange={setOutputFlowRate}
          onValidityChange={validity('output_flow_rate')}
          min={0}
          max={20}
          help="Measured at the scale, after the puck."
        />
      )}

      {showDutyCycle && (
        <NumberField
          label="Duty cycle"
          unit="%"
          value={dutyCycle}
          onChange={setDutyCycle}
          onValidityChange={validity('duty_cycle')}
          min={0}
          max={100}
          help="The pump runs at this duty regardless of what the sensors read."
        />
      )}

      {invalid && <Alert role="danger">Fix the field marked above before saving.</Alert>}

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
