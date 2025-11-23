import { useState } from 'preact/hooks';
import { ScheduleAction, BoilerControlMode, MachineMode, RoutineIndex } from '../schemas/schemas';
import { EntitySelector } from './EntitySelector';

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
      zIndex: 1000
    }}>
      <div style={{
        backgroundColor: 'white',
        borderRadius: '8px',
        padding: '2rem',
        maxWidth: '500px',
        width: '90%',
        maxHeight: '80vh',
        overflow: 'auto'
      }}>
        <h2 style={{ marginBottom: '1.5rem' }}>
          {command ? 'Edit Command' : 'Add Command'}
        </h2>

        {/* Command Type Selector */}
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
            <optgroup label="Routine">
              <option value="RunRoutine">Run Routine</option>
              <option value="CancelRoutine">Cancel Routine</option>
            </optgroup>
            <optgroup label="Boiler">
              <option value="SetBoilerControlTarget">Set Boiler Control Target</option>
              <option value="SetBoilerControlTargetValues">Set Boiler Control Values</option>
            </optgroup>
            <optgroup label="Machine">
              <option value="SetMachineMode">Set Machine Mode</option>
            </optgroup>
          </select>
        </div>

        {/* Entity selector for commands that need it */}
        {needsBoilerIndex && (
          <EntitySelector
            entityType="boiler"
            index={boilerIndex}
            onChange={setBoilerIndex}
          />
        )}

        {/* Boiler Control Target */}
        {commandType === 'SetBoilerControlTarget' && (
          <>
            <div style={{ marginBottom: '1.5rem' }}>
              <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>Control Mode</label>
              <select
                value={boilerMode}
                onChange={(e) => setBoilerMode(e.currentTarget.value)}
                style={{
                  width: '100%',
                  padding: '0.5rem',
                  border: '1px solid #ccc',
                  borderRadius: '4px',
                  fontSize: '1rem'
                }}
              >
                <option value="Temperature">Temperature</option>
                <option value="Pressure">Pressure</option>
                <option value="Off">Off</option>
              </select>
            </div>
            <div style={{ marginBottom: '1.5rem' }}>
              <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>
                Target Temperature (°C, optional)
              </label>
              <input
                type="number"
                step="0.1"
                value={boilerTemp ?? ''}
                onChange={(e) => setBoilerTemp(e.currentTarget.value ? parseFloat(e.currentTarget.value) : null)}
                style={{
                  width: '100%',
                  padding: '0.5rem',
                  border: '1px solid #ccc',
                  borderRadius: '4px',
                  fontSize: '1rem'
                }}
              />
            </div>
            <div style={{ marginBottom: '1.5rem' }}>
              <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>
                Target Pressure (bar, optional)
              </label>
              <input
                type="number"
                step="0.1"
                value={boilerPressure ?? ''}
                onChange={(e) => setBoilerPressure(e.currentTarget.value ? parseFloat(e.currentTarget.value) : null)}
                style={{
                  width: '100%',
                  padding: '0.5rem',
                  border: '1px solid #ccc',
                  borderRadius: '4px',
                  fontSize: '1rem'
                }}
              />
            </div>
          </>
        )}

        {/* Boiler Control Values */}
        {commandType === 'SetBoilerControlTargetValues' && (
          <>
            <div style={{ marginBottom: '1.5rem' }}>
              <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>
                Temperature (°C, optional)
              </label>
              <input
                type="number"
                step="0.1"
                value={boilerTemp ?? ''}
                onChange={(e) => setBoilerTemp(e.currentTarget.value ? parseFloat(e.currentTarget.value) : null)}
                style={{
                  width: '100%',
                  padding: '0.5rem',
                  border: '1px solid #ccc',
                  borderRadius: '4px',
                  fontSize: '1rem'
                }}
              />
            </div>
            <div style={{ marginBottom: '1.5rem' }}>
              <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>
                Pressure (bar, optional)
              </label>
              <input
                type="number"
                step="0.1"
                value={boilerPressure ?? ''}
                onChange={(e) => setBoilerPressure(e.currentTarget.value ? parseFloat(e.currentTarget.value) : null)}
                style={{
                  width: '100%',
                  padding: '0.5rem',
                  border: '1px solid #ccc',
                  borderRadius: '4px',
                  fontSize: '1rem'
                }}
              />
            </div>
          </>
        )}

        {/* Run Routine */}
        {commandType === 'RunRoutine' && (
          <>
            <div style={{ marginBottom: '1.5rem' }}>
              <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>Routine Type</label>
              <select
                value={routineType}
                onChange={(e) => setRoutineType(e.currentTarget.value as 'internal' | 'function' | 'custom')}
                style={{
                  width: '100%',
                  padding: '0.5rem',
                  border: '1px solid #ccc',
                  borderRadius: '4px',
                  fontSize: '1rem'
                }}
              >
                <option value="internal">Internal</option>
                <option value="function">Function</option>
                <option value="custom">Custom</option>
              </select>
            </div>
            <div style={{ marginBottom: '1.5rem' }}>
              <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>Routine Index</label>
              <input
                type="number"
                min="0"
                value={routineIndex}
                onChange={(e) => setRoutineIndex(parseInt(e.currentTarget.value) || 0)}
                style={{
                  width: '100%',
                  padding: '0.5rem',
                  border: '1px solid #ccc',
                  borderRadius: '4px',
                  fontSize: '1rem'
                }}
              />
            </div>
          </>
        )}

        {/* Set Machine Mode */}
        {commandType === 'SetMachineMode' && (
          <div style={{ marginBottom: '1.5rem' }}>
            <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>
              Machine Mode
            </label>
            <select
              value={machineMode}
              onChange={(e) => setMachineMode(e.currentTarget.value)}
              style={{
                width: '100%',
                padding: '0.5rem',
                border: '1px solid #ccc',
                borderRadius: '4px',
                fontSize: '1rem'
              }}
            >
              <option value="On">On</option>
              <option value="Off">Off</option>
              <option value="PowerSaveStandby">Power Save Standby</option>
            </select>
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
