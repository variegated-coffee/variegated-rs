import { useState } from 'preact/hooks';
import { RoutineCommand, ParameterValue, RoutineParameter, DerivedParameter, ParameterUnit, TransitionOrigin } from '../../schemas/schemas';
import { ParameterValueEditor } from './ParameterValueEditor';
import { EntitySelector, EntityType } from '../EntitySelector';

interface RoutineCommandBuilderProps {
  command: RoutineCommand | null;
  onSave: (command: RoutineCommand) => void;
  onCancel: () => void;
  parameters: RoutineParameter[];
  derivedParameters: DerivedParameter[];
}

type CommandType =
  | 'StartBrewing'
  | 'StopBrewing'
  | 'TareGroupScale'
  | 'StartPumpingToWaterTap'
  | 'StopPumpingToWaterTap'
  | 'StartSteaming'
  | 'StopSteaming'
  | 'SetSteamValveOpenness'
  | 'SetBoilerTemperature'
  | 'SetBoilerPressure'
  | 'SetGroupFlowRate'
  | 'SetGroupPressure'
  | 'SetGroupOutputFlowRate'
  | 'SetGroupFixedDutyCycle'
  | 'SetGroupFullOn'
  | 'SetGroupOff'
  | 'SetBoilerOff'
  | 'SetGroupFlowRateWithTransition'
  | 'SetGroupPressureWithTransition'
  | 'SetGroupOutputFlowRateWithTransition'
  | 'SetGroupFixedDutyCycleWithTransition'
  | 'InferGroupPressureIntegral'
  | 'InferGroupFlowRateIntegral'
  | 'InferGroupOutputFlowRateIntegral';

function getCommandType(command: RoutineCommand): CommandType {
  return command.type as CommandType;
}

/** Whether `v` is a tagged variant with one of the given tags. */
function hasTag<T extends string>(v: unknown, tags: readonly T[]): v is { type: T } {
  return (
    typeof v === 'object' &&
    v !== null &&
    tags.includes((v as { type?: unknown }).type as T)
  );
}

// Enumerated rather than "anything with a `type`", because the hazard being guarded against
// is reading one of these *as the other* if a payload's positions ever shift. A tag outside
// the set falls back to the default, which is the safe direction -- but it does mean a new
// variant must be added here or it will not load back into the editor.
const PARAMETER_VALUE_TAGS = ['Static', 'Parameter', 'DerivedParameter'] as const;
const TRANSITION_ORIGIN_TAGS = ['Value', 'CurrentTarget', 'CurrentValue'] as const;

/**
 * What an existing command was carrying, for the editor to open on.
 *
 * **This is the whole of the bug it was written for.** The `command` prop was read for
 * exactly two things -- which type to select, and whether the dialog says "Edit" or "Add" --
 * so every field reverted to its default the moment a saved command was reopened. A
 * transition saved at 9 bar over 5 seconds came back as 0 bar over 1 second, and pressing
 * save wrote those numbers back over the real ones. Every value-carrying command was
 * affected; a transition is simply where it is most visible, having three fields to lose.
 *
 * Read positionally rather than by a per-variant switch, mirroring `handleSave` in reverse.
 * Every command this builder offers is either a bare index or `[index, value, time?,
 * origin?]`, so one path serves all of them -- and a switch would be twenty branches that
 * have to stay in step with the twenty that write them.
 */
export function fieldsOf(command: RoutineCommand | null): {
  index: number;
  value: ParameterValue;
  transitionTime: ParameterValue;
  origin: TransitionOrigin;
} {
  const defaults = {
    index: 0,
    value: { type: 'Static', value: 0 } as ParameterValue,
    transitionTime: { type: 'Static', value: 1 } as ParameterValue,
    // See the note on the state below for why the default is the target.
    origin: { type: 'CurrentTarget' } as TransitionOrigin,
  };

  if (!command || !('value' in command)) return defaults;

  const payload = (command as { value: unknown }).value;
  // A command that carries nothing but which boiler or group it acts on.
  if (typeof payload === 'number') return { ...defaults, index: payload };
  if (!Array.isArray(payload)) return defaults;

  const [index, value, time, origin] = payload as unknown[];
  return {
    index: typeof index === 'number' ? index : defaults.index,
    value: hasTag(value, PARAMETER_VALUE_TAGS) ? (value as ParameterValue) : defaults.value,
    transitionTime: hasTag(time, PARAMETER_VALUE_TAGS)
      ? (time as ParameterValue)
      : defaults.transitionTime,
    origin: hasTag(origin, TRANSITION_ORIGIN_TAGS)
      ? (origin as TransitionOrigin)
      : defaults.origin,
  };
}

export function RoutineCommandBuilder({ command, onSave, onCancel, parameters, derivedParameters }: RoutineCommandBuilderProps) {
  const [commandType, setCommandType] = useState<CommandType>(
    command ? getCommandType(command) : 'StartBrewing'
  );
  // Seeded from the command being edited rather than from fixed defaults -- see `fieldsOf`.
  const initial = fieldsOf(command);
  const [index, setIndex] = useState<number>(initial.index);
  const [value, setValue] = useState<ParameterValue>(initial.value);
  const [transitionTime, setTransitionTime] = useState<ParameterValue>(initial.transitionTime);
  // Where the ramp starts. `CurrentTarget` for a new command, deliberately: it means
  // "continue from wherever the last ramp left the setpoint", which is what a transition
  // almost always means. The old implicit behaviour was `CurrentValue` -- anchoring to the
  // *measurement* -- which turned a "decline to 4 bar over 30 s" into a flat line on a real
  // machine, because pressure had only reached 3.995 bar by the time the command ran.
  const [transitionOrigin, setTransitionOrigin] = useState<TransitionOrigin>(initial.origin);
  // Only `Value` carries a payload; the other two are unit variants. Seeded from the origin
  // when it is one, so reopening a transition that started from an explicit 6 bar shows 6
  // rather than the placeholder.
  const [originValue, setOriginValue] = useState<ParameterValue>(
    initial.origin.type === 'Value' ? initial.origin.value : { type: 'Static', value: 0 }
  );
  const origin: TransitionOrigin =
    transitionOrigin.type === 'Value' ? { type: 'Value', value: originValue } : transitionOrigin;

  const handleSave = () => {
    let cmd: RoutineCommand;

    switch (commandType) {
      case 'StartBrewing':
        cmd = { type: 'StartBrewing', value: index };
        break;
      case 'StopBrewing':
        cmd = { type: 'StopBrewing', value: index };
        break;
      case 'TareGroupScale':
        cmd = { type: 'TareGroupScale', value: index };
        break;
      case 'StartPumpingToWaterTap':
        cmd = { type: 'StartPumpingToWaterTap', value: index };
        break;
      case 'StopPumpingToWaterTap':
        cmd = { type: 'StopPumpingToWaterTap', value: index };
        break;
      case 'StartSteaming':
        cmd = { type: 'StartSteaming', value: index };
        break;
      case 'StopSteaming':
        cmd = { type: 'StopSteaming', value: index };
        break;
      case 'SetSteamValveOpenness':
        cmd = { type: 'SetSteamValveOpenness', value: [index, value] };
        break;
      case 'SetBoilerTemperature':
        cmd = { type: 'SetBoilerTemperature', value: [index, value] };
        break;
      case 'SetBoilerPressure':
        cmd = { type: 'SetBoilerPressure', value: [index, value] };
        break;
      case 'SetGroupFlowRate':
        cmd = { type: 'SetGroupFlowRate', value: [index, value] };
        break;
      case 'SetGroupPressure':
        cmd = { type: 'SetGroupPressure', value: [index, value] };
        break;
      case 'SetGroupOutputFlowRate':
        cmd = { type: 'SetGroupOutputFlowRate', value: [index, value] };
        break;
      case 'SetGroupFixedDutyCycle':
        cmd = { type: 'SetGroupFixedDutyCycle', value: [index, value] };
        break;
      case 'SetGroupFullOn':
        cmd = { type: 'SetGroupFullOn', value: index };
        break;
      case 'SetGroupOff':
        cmd = { type: 'SetGroupOff', value: index };
        break;
      case 'SetBoilerOff':
        cmd = { type: 'SetBoilerOff', value: index };
        break;
      case 'SetGroupFlowRateWithTransition':
        cmd = { type: 'SetGroupFlowRateWithTransition', value: [index, value, transitionTime, origin] };
        break;
      case 'SetGroupPressureWithTransition':
        cmd = { type: 'SetGroupPressureWithTransition', value: [index, value, transitionTime, origin] };
        break;
      case 'SetGroupOutputFlowRateWithTransition':
        cmd = { type: 'SetGroupOutputFlowRateWithTransition', value: [index, value, transitionTime, origin] };
        break;
      case 'SetGroupFixedDutyCycleWithTransition':
        cmd = { type: 'SetGroupFixedDutyCycleWithTransition', value: [index, value, transitionTime, origin] };
        break;
      case 'InferGroupPressureIntegral':
        cmd = { type: 'InferGroupPressureIntegral', value: [index, value] };
        break;
      case 'InferGroupFlowRateIntegral':
        cmd = { type: 'InferGroupFlowRateIntegral', value: [index, value] };
        break;
      case 'InferGroupOutputFlowRateIntegral':
        cmd = { type: 'InferGroupOutputFlowRateIntegral', value: [index, value] };
        break;
    }

    onSave(cmd);
  };

  const needsIndex = true; // All commands need an index
  const needsValue = [
    'SetSteamValveOpenness',
    'SetBoilerTemperature',
    'SetBoilerPressure',
    'SetGroupFlowRate',
    'SetGroupPressure',
    'SetGroupOutputFlowRate',
    'SetGroupFixedDutyCycle',
    'SetGroupFlowRateWithTransition',
    'SetGroupPressureWithTransition',
    'SetGroupOutputFlowRateWithTransition',
    'SetGroupFixedDutyCycleWithTransition',
    'InferGroupPressureIntegral',
    'InferGroupFlowRateIntegral',
    'InferGroupOutputFlowRateIntegral'
  ].includes(commandType);

  const needsTransition = commandType.includes('WithTransition');

  const getValueUnit = (): ParameterUnit | null => {
    if (commandType.includes('Temperature')) return { type: 'Celsius' };
    if (commandType.includes('Pressure')) return { type: 'Bar' };
    if (commandType.includes('FlowRate')) return { type: 'MillilitersPerSecond' };
    if (commandType.includes('DutyCycle')) return { type: 'Percent' };
    if (commandType.includes('Openness')) return { type: 'Percent' };
    return null;
  };

  const getEntityType = (): EntityType => {
    if (commandType.includes('Boiler')) return 'boiler';
    if (commandType.includes('WaterTap')) return 'water_tap';
    if (commandType.includes('Steam')) return 'steam_wand';
    return 'group'; // Default for brewing, scale, and group commands
  };

  return (
    <div style={{
      position: 'fixed',
      top: 0,
      left: 0,
      right: 0,
      bottom: 0,
      backgroundColor: 'rgba(0,0,0,0.5)',
      display: 'flex',
      alignItems: 'center',
      justifyContent: 'center',
      zIndex: 1002
    }}>
      <div style={{
        backgroundColor: 'white',
        borderRadius: '8px',
        padding: '2rem',
        maxWidth: '600px',
        width: '90%',
        maxHeight: '80vh',
        overflow: 'auto'
      }}>
        <h2 style={{ marginBottom: '1.5rem' }}>
          {command ? 'Edit Command' : 'Add Command'}
        </h2>

        {/* Command Type */}
        <div style={{ marginBottom: '1.5rem' }}>
          <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>Command Type</label>
          <select
            value={commandType}
            onChange={(e) => setCommandType(e.currentTarget.value as CommandType)}
            style={{
              width: '100%',
              padding: '0.5rem',
              border: '1px solid #ccc',
              borderRadius: '4px',
              fontSize: '1rem'
            }}
          >
            <optgroup label="Brewing">
              <option value="StartBrewing">Start Brewing</option>
              <option value="StopBrewing">Stop Brewing</option>
            </optgroup>
            <optgroup label="Boiler">
              <option value="SetBoilerTemperature">Set Boiler Temperature</option>
              <option value="SetBoilerPressure">Set Boiler Pressure</option>
              <option value="SetBoilerOff">Set Boiler Off</option>
            </optgroup>
            <optgroup label="Group - Direct">
              <option value="SetGroupFlowRate">Set Group Flow Rate</option>
              <option value="SetGroupPressure">Set Group Pressure</option>
              <option value="SetGroupOutputFlowRate">Set Group Output Flow Rate</option>
              <option value="SetGroupFixedDutyCycle">Set Group Fixed Duty Cycle</option>
              <option value="SetGroupFullOn">Set Group Full On</option>
              <option value="SetGroupOff">Set Group Off</option>
            </optgroup>
            <optgroup label="Group - With Transition">
              <option value="SetGroupFlowRateWithTransition">Set Group Flow Rate (Transition)</option>
              <option value="SetGroupPressureWithTransition">Set Group Pressure (Transition)</option>
              <option value="SetGroupOutputFlowRateWithTransition">Set Group Output Flow Rate (Transition)</option>
              <option value="SetGroupFixedDutyCycleWithTransition">Set Group Duty Cycle (Transition)</option>
            </optgroup>
            <optgroup label="Group - Bumpless Transfer">
              <option value="InferGroupPressureIntegral">Infer Group Pressure Integral</option>
              <option value="InferGroupFlowRateIntegral">Infer Group Flow Rate Integral</option>
              <option value="InferGroupOutputFlowRateIntegral">Infer Group Output Flow Rate Integral</option>
            </optgroup>
            <optgroup label="Scale">
              <option value="TareGroupScale">Tare Group Scale</option>
            </optgroup>
            <optgroup label="Water Tap">
              <option value="StartPumpingToWaterTap">Start Pumping to Water Tap</option>
              <option value="StopPumpingToWaterTap">Stop Pumping to Water Tap</option>
            </optgroup>
            <optgroup label="Steam Wand">
              <option value="StartSteaming">Start Steaming</option>
              <option value="StopSteaming">Stop Steaming</option>
              <option value="SetSteamValveOpenness">Set Steam Valve Openness</option>
            </optgroup>
          </select>
        </div>

        {/* Entity selector */}
        {needsIndex && (
          <EntitySelector
            entityType={getEntityType()}
            index={index}
            onChange={setIndex}
          />
        )}

        {/* Value (for parameterizable commands) */}
        {needsValue && (
          <ParameterValueEditor
            value={value}
            onChange={setValue}
            label={commandType.includes('Temperature') ? 'Temperature' :
                   commandType.includes('Pressure') ? 'Pressure' :
                   commandType.includes('FlowRate') ? 'Flow Rate' :
                   commandType.includes('DutyCycle') ? 'Duty Cycle' : 'Value'}
            unit={getValueUnit()}
            parameters={parameters}
            derivedParameters={derivedParameters}
          />
        )}

        {/* Transition Time */}
        {needsTransition && (
          <ParameterValueEditor
            value={transitionTime}
            onChange={setTransitionTime}
            label="Transition Time"
            unit={{ type: 'Seconds' }}
            parameters={parameters}
            derivedParameters={derivedParameters}
          />
        )}

        {/* Where the ramp starts. See the note on `transitionOrigin` above for why this is
            a choice rather than an assumption, and why the default is the target. */}
        {needsTransition && (
          <div style={{ marginBottom: '1rem' }}>
            <label style={{ display: 'block', marginBottom: '0.25rem' }}>Transition From</label>
            <select
              value={transitionOrigin.type}
              onChange={(e) => {
                const type = (e.target as HTMLSelectElement).value as TransitionOrigin['type'];
                setTransitionOrigin(
                  type === 'Value' ? { type: 'Value', value: originValue } : { type },
                );
              }}
              style={{ width: '100%', padding: '0.5rem' }}
            >
              <option value="CurrentTarget">Current target &mdash; continue from the last ramp</option>
              <option value="CurrentValue">Current value &mdash; resync to what the machine measures</option>
              <option value="Value">A specific value</option>
            </select>
            <div style={{ fontSize: '0.85em', opacity: 0.75, marginTop: '0.25rem' }}>
              {transitionOrigin.type === 'CurrentTarget'
                ? 'Starts where the previous step left the setpoint. Usually what you want.'
                : transitionOrigin.type === 'CurrentValue'
                ? 'Starts at the measured reading. If the machine is lagging behind its setpoint, a declining transition can end up ramping upward.'
                : 'Starts at a value you name, whatever the machine is doing.'}
            </div>
            {transitionOrigin.type === 'Value' && (
              <div style={{ marginTop: '0.5rem' }}>
                <ParameterValueEditor
                  value={originValue}
                  onChange={(v) => {
                    setOriginValue(v);
                    setTransitionOrigin({ type: 'Value', value: v });
                  }}
                  label="Start From"
                  unit={getValueUnit()}
                  parameters={parameters}
                  derivedParameters={derivedParameters}
                />
              </div>
            )}
          </div>
        )}

        {/* Action Buttons */}
        <div style={{ display: 'flex', gap: '1rem', marginTop: '2rem' }}>
          <button
            onClick={handleSave}
            style={{
              flex: 1,
              padding: '0.75rem',
              backgroundColor: '#0066cc',
              color: 'white',
              border: 'none',
              borderRadius: '4px',
              fontSize: '1rem',
              cursor: 'pointer'
            }}
          >
            Save
          </button>
          <button
            onClick={onCancel}
            style={{
              flex: 1,
              padding: '0.75rem',
              backgroundColor: '#666',
              color: 'white',
              border: 'none',
              borderRadius: '4px',
              fontSize: '1rem',
              cursor: 'pointer'
            }}
          >
            Cancel
          </button>
        </div>
      </div>
    </div>
  );
}
