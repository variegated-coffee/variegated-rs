import { useState } from 'preact/hooks';
import { RoutineCommand, ParameterValue, RoutineParameter, DerivedParameter, ParameterUnit } from '../../schemas/schemas';
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

export function RoutineCommandBuilder({ command, onSave, onCancel, parameters, derivedParameters }: RoutineCommandBuilderProps) {
  const [commandType, setCommandType] = useState<CommandType>(
    command ? getCommandType(command) : 'StartBrewing'
  );
  const [index, setIndex] = useState<number>(0);
  const [value, setValue] = useState<ParameterValue>({ type: 'Static', value: 0 });
  const [transitionTime, setTransitionTime] = useState<ParameterValue>({ type: 'Static', value: 1 });

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
        cmd = { type: 'SetGroupFlowRateWithTransition', value: [index, value, transitionTime] };
        break;
      case 'SetGroupPressureWithTransition':
        cmd = { type: 'SetGroupPressureWithTransition', value: [index, value, transitionTime] };
        break;
      case 'SetGroupOutputFlowRateWithTransition':
        cmd = { type: 'SetGroupOutputFlowRateWithTransition', value: [index, value, transitionTime] };
        break;
      case 'SetGroupFixedDutyCycleWithTransition':
        cmd = { type: 'SetGroupFixedDutyCycleWithTransition', value: [index, value, transitionTime] };
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
