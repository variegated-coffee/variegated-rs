import { useState } from 'preact/hooks';
import { Button, Dialog, Field, Select, TextInput, tokens } from '@variegated-coffee/ui';
import { ScheduleAction, BoilerControlMode, MachineMode, RoutineIndex } from '../schemas/schemas';
import { EntitySelector } from './EntitySelector';

/**
 * Parse an optional decimal the user typed.
 *
 * `parseFloat(x)` alone returns `NaN` for anything unparseable, and `NaN` reaching the
 * firmware as a boiler target is worse than the field being ignored. An empty box means
 * "no value", which is a real state here -- both target fields are optional.
 */
function parseOptional(raw: string): number | null {
  const trimmed = raw.trim();
  if (trimmed === '') return null;
  const parsed = Number.parseFloat(trimmed);
  return Number.isFinite(parsed) ? parsed : null;
}

/** What is in the box, so a half-typed "9." is not reformatted out from under the cursor. */
function show(value: number | null): string {
  return value === null ? '' : String(value);
}

interface CommandBuilderProps {
  command: ScheduleAction | null;
  onSave: (command: ScheduleAction) => void;
  onCancel: () => void;
}

type CommandType =
  | 'RunRoutine'
  | 'CancelRoutine'
  | 'SetMachineMode'
  | 'SetBoilerControlTarget'
  | 'SetBoilerControlTargetValues';

function getCommandType(command: ScheduleAction): CommandType {
  if (typeof command === 'string') return 'CancelRoutine';
  if (command.type === 'RunRoutine') return 'RunRoutine';
  if (command.type === 'SetMachineMode') return 'SetMachineMode';
  if (command.type === 'SetBoilerControlTarget') return 'SetBoilerControlTarget';
  if (command.type === 'SetBoilerControlTargetValues') return 'SetBoilerControlTargetValues';
  if (command.type === 'CancelRoutine') return 'CancelRoutine';
  return 'CancelRoutine';
}

export function CommandBuilder({ command, onSave, onCancel }: CommandBuilderProps) {
  const [commandType, setCommandType] = useState<CommandType>(
    command ? getCommandType(command) : 'RunRoutine'
  );

  // Boiler control target
  const [boilerIndex, setBoilerIndex] = useState<number>(0);
  const [boilerMode, setBoilerMode] = useState<string>('Temperature');
  const [boilerTemp, setBoilerTemp] = useState<number | null>(null);
  const [boilerPressure, setBoilerPressure] = useState<number | null>(null);

  // Routine
  const [routineType, setRoutineType] = useState<'internal' | 'function' | 'custom'>('custom');
  const [routineIndex, setRoutineIndex] = useState<number>(0);

  // Machine mode
  const [machineMode, setMachineMode] = useState<string>(
    command && typeof command === 'object' && command.type === 'SetMachineMode' ? command.value.type : 'On'
  );

  const handleSave = () => {
    let cmd: ScheduleAction;

    switch (commandType) {
      case 'SetBoilerControlTarget':
        cmd = {
          type: 'SetBoilerControlTarget',
          value: [
            boilerIndex,
            { type: boilerMode } as BoilerControlMode,
            (boilerTemp !== null || boilerPressure !== null)
              ? { temperature: boilerTemp, pressure: boilerPressure }
              : null
          ]
        };
        break;
      case 'SetBoilerControlTargetValues':
        cmd = {
          type: 'SetBoilerControlTargetValues',
          value: [
            boilerIndex,
            { temperature: boilerTemp, pressure: boilerPressure }
          ]
        };
        break;
      case 'RunRoutine': {
        let routineIndexEnum: RoutineIndex;
        if (routineType === 'internal') {
          routineIndexEnum = { type: 'Internal', value: routineIndex };
        } else if (routineType === 'function') {
          routineIndexEnum = { type: 'Function', value: routineIndex };
        } else {
          routineIndexEnum = { type: 'Custom', value: routineIndex };
        }
        cmd = { type: 'RunRoutine', value: [routineIndexEnum, null] };
        break;
      }
      case 'SetMachineMode':
        cmd = { type: 'SetMachineMode', value: { type: machineMode } as MachineMode };
        break;
      case 'CancelRoutine':
        cmd = { type: 'CancelRoutine' };
        break;
      default:
        cmd = { type: 'CancelRoutine' };
    }

    onSave(cmd);
  };

  const needsBoilerIndex = [
    'SetBoilerControlTarget',
    'SetBoilerControlTargetValues'
  ].includes(commandType);

  return (
    <Dialog
      title={command ? 'Edit command' : 'Add command'}
      onClose={onCancel}
      footer={
        <>
          <Button variant="secondary" onClick={onCancel}>
            Cancel
          </Button>
          {/* Cancel first, primary last -- the order the footer of every other dialog in
              the app uses. This one had Save on the left and Cancel on the right, both
              filled and equally wide. */}
          <Button variant="primary" onClick={handleSave}>
            Save
          </Button>
        </>
      }
    >
      <div style={{ display: 'flex', flexDirection: 'column', gap: tokens.space.lg }}>
        <Field label="Command type">
          {(control) => (
            <Select
              {...control}
              value={commandType}
              onChange={(value) => setCommandType(value as CommandType)}
              options={[
                { value: 'RunRoutine', label: 'Run routine', group: 'Routine' },
                { value: 'CancelRoutine', label: 'Cancel routine', group: 'Routine' },
                { value: 'SetBoilerControlTarget', label: 'Set control target', group: 'Boiler' },
                { value: 'SetBoilerControlTargetValues', label: 'Set control values', group: 'Boiler' },
                { value: 'SetMachineMode', label: 'Set machine mode', group: 'Machine' },
              ]}
            />
          )}
        </Field>

        {needsBoilerIndex && (
          <EntitySelector entityType="boiler" index={boilerIndex} onChange={setBoilerIndex} />
        )}

        {commandType === 'SetBoilerControlTarget' && (
          <Field label="Control mode">
            {(control) => (
              <Select
                {...control}
                value={boilerMode}
                onChange={setBoilerMode}
                options={[
                  { value: 'Temperature', label: 'Temperature' },
                  { value: 'Pressure', label: 'Pressure' },
                  { value: 'Off', label: 'Off' },
                ]}
              />
            )}
          </Field>
        )}

        {/* The two target fields are identical in both branches, so they are written once.
            The unit is on the field rather than inside the label -- "(°C, optional)" put
            two different kinds of information into the label and wrapped it onto a second
            line, which pushed the label away from its own input. */}
        {(commandType === 'SetBoilerControlTarget' ||
          commandType === 'SetBoilerControlTargetValues') && (
          <>
            <Field label="Target temperature" unit="°C" help="Leave empty to not set one.">
              {(control) => (
                <TextInput
                  {...control}
                  numeric
                  value={show(boilerTemp)}
                  onInput={(value) => setBoilerTemp(parseOptional(value))}
                />
              )}
            </Field>
            <Field label="Target pressure" unit="bar" help="Leave empty to not set one.">
              {(control) => (
                <TextInput
                  {...control}
                  numeric
                  value={show(boilerPressure)}
                  onInput={(value) => setBoilerPressure(parseOptional(value))}
                />
              )}
            </Field>
          </>
        )}

        {commandType === 'RunRoutine' && (
          <>
            <Field label="Routine type">
              {(control) => (
                <Select
                  {...control}
                  value={routineType}
                  onChange={(value) => setRoutineType(value as 'internal' | 'function' | 'custom')}
                  options={[
                    { value: 'internal', label: 'Internal' },
                    { value: 'function', label: 'Function' },
                    { value: 'custom', label: 'Custom' },
                  ]}
                />
              )}
            </Field>
            <Field label="Routine index" help="The slot the routine is stored in.">
              {(control) => (
                <TextInput
                  {...control}
                  numeric
                  value={String(routineIndex)}
                  onInput={(value) => setRoutineIndex(Math.max(0, parseOptional(value) ?? 0))}
                />
              )}
            </Field>
          </>
        )}

        {commandType === 'SetMachineMode' && (
          <Field label="Machine mode">
            {(control) => (
              <Select
                {...control}
                value={machineMode}
                onChange={setMachineMode}
                options={[
                  { value: 'On', label: 'On' },
                  { value: 'Off', label: 'Off' },
                  { value: 'PowerSaveStandby', label: 'Power save standby' },
                ]}
              />
            )}
          </Field>
        )}
      </div>
    </Dialog>
  );
}
