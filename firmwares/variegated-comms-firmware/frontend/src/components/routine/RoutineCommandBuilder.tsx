import { useState } from 'preact/hooks';
import { Button, Dialog, Field, Select, tokens } from '@variegated-coffee/ui';
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
    <Dialog
      title={command ? 'Edit command' : 'Add command'}
      onClose={onCancel}
      width="600px"
      footer={
        <>
          <Button variant="secondary" onClick={onCancel}>
            Cancel
          </Button>
          <Button variant="primary" onClick={handleSave}>
            Save
          </Button>
        </>
      }
    >
      <div style={{ display: 'flex', flexDirection: 'column', gap: tokens.space.md }}>
        {/* The command names drop the "Set Group" prefix that every option in a group
            already carries in its heading -- twenty-three options each repeating the
            category they are filed under is what made this list hard to read. */}
        <Field label="Command">
          {(control) => (
            <Select
              {...control}
              value={commandType}
              onChange={(value) => setCommandType(value as CommandType)}
              options={[
                { value: 'StartBrewing', label: 'Start brewing', group: 'Brewing' },
                { value: 'StopBrewing', label: 'Stop brewing', group: 'Brewing' },

                { value: 'SetBoilerTemperature', label: 'Hold a temperature', group: 'Boiler' },
                { value: 'SetBoilerPressure', label: 'Hold a pressure', group: 'Boiler' },
                { value: 'SetBoilerOff', label: 'Turn off', group: 'Boiler' },

                { value: 'SetGroupFlowRate', label: 'Hold a flow rate', group: 'Group' },
                { value: 'SetGroupPressure', label: 'Hold a pressure', group: 'Group' },
                { value: 'SetGroupOutputFlowRate', label: 'Hold an output flow rate', group: 'Group' },
                { value: 'SetGroupFixedDutyCycle', label: 'Hold a duty cycle', group: 'Group' },
                { value: 'SetGroupFullOn', label: 'Full on', group: 'Group' },
                { value: 'SetGroupOff', label: 'Off', group: 'Group' },

                { value: 'SetGroupFlowRateWithTransition', label: 'Ramp to a flow rate', group: 'Group — ramped' },
                { value: 'SetGroupPressureWithTransition', label: 'Ramp to a pressure', group: 'Group — ramped' },
                { value: 'SetGroupOutputFlowRateWithTransition', label: 'Ramp to an output flow rate', group: 'Group — ramped' },
                { value: 'SetGroupFixedDutyCycleWithTransition', label: 'Ramp to a duty cycle', group: 'Group — ramped' },

                { value: 'InferGroupPressureIntegral', label: 'Infer the pressure integral', group: 'Group — bumpless transfer' },
                { value: 'InferGroupFlowRateIntegral', label: 'Infer the flow rate integral', group: 'Group — bumpless transfer' },
                { value: 'InferGroupOutputFlowRateIntegral', label: 'Infer the output flow rate integral', group: 'Group — bumpless transfer' },

                { value: 'TareGroupScale', label: 'Tare the scale', group: 'Scale' },

                { value: 'StartPumpingToWaterTap', label: 'Start pumping', group: 'Water tap' },
                { value: 'StopPumpingToWaterTap', label: 'Stop pumping', group: 'Water tap' },

                { value: 'StartSteaming', label: 'Start steaming', group: 'Steam wand' },
                { value: 'StopSteaming', label: 'Stop steaming', group: 'Steam wand' },
                { value: 'SetSteamValveOpenness', label: 'Set valve openness', group: 'Steam wand' },
              ]}
            />
          )}
        </Field>

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
          <div>
            <Field
              label="Ramp from"
              help={
                transitionOrigin.type === 'CurrentTarget'
                  ? 'Starts where the previous step left the setpoint. Usually what you want.'
                  : transitionOrigin.type === 'CurrentValue'
                    ? 'Starts at the measured reading. If the machine is lagging behind its setpoint, a declining transition can end up ramping upward.'
                    : 'Starts at a value you name, whatever the machine is doing.'
              }
            >
              {(control) => (
                <Select
                  {...control}
                  value={transitionOrigin.type}
                  onChange={(value) => {
                    const type = value as TransitionOrigin['type'];
                    setTransitionOrigin(
                      type === 'Value' ? { type: 'Value', value: originValue } : { type },
                    );
                  }}
                  options={[
                    { value: 'CurrentTarget', label: 'The current target' },
                    { value: 'CurrentValue', label: 'The current measured value' },
                    { value: 'Value', label: 'A specific value' },
                  ]}
                />
              )}
            </Field>
            {transitionOrigin.type === 'Value' && (
              <div style={{ marginTop: tokens.space.sm }}>
                <ParameterValueEditor
                  value={originValue}
                  onChange={(v) => {
                    setOriginValue(v);
                    setTransitionOrigin({ type: 'Value', value: v });
                  }}
                  label="Start from"
                  unit={getValueUnit()}
                  parameters={parameters}
                  derivedParameters={derivedParameters}
                />
              </div>
            )}
          </div>
        )}

      </div>
    </Dialog>
  );
}
