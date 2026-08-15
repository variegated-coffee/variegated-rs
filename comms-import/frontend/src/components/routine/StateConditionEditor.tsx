import { useState, useEffect } from 'preact/hooks';
import { StateCondition, ParameterValue, RoutineParameter, DerivedParameter, ParameterUnit } from '../../schemas/schemas';
import { ParameterValueEditor } from './ParameterValueEditor';
import { EntitySelector, EntityType } from '../EntitySelector';

interface StateConditionEditorProps {
  condition: StateCondition;
  onChange: (condition: StateCondition) => void;
  parameters: RoutineParameter[];
  derivedParameters: DerivedParameter[];
}

type ConditionType =
  | 'Brewing'
  | 'NotBrewing'
  | 'BoilerTemperatureAbove'
  | 'BoilerTemperatureBelow'
  | 'BoilerPressureAbove'
  | 'BoilerPressureBelow'
  | 'GroupInputFlowRateAbove'
  | 'GroupInputFlowRateBelow'
  | 'GroupPressureAbove'
  | 'GroupPressureBelow'
  | 'WaterTapFlowRateAbove'
  | 'WaterTapFlowRateBelow'
  | 'OutputWeightAbove'
  | 'OutputWeightBelow'
  | 'InputVolumeAboveRelativeToStart';

function getConditionType(condition: StateCondition): ConditionType {
  if (condition.type === 'Brewing') return 'Brewing';
  if (condition.type === 'NotBrewing') return 'NotBrewing';
  if (condition.type === 'BoilerTemperatureAbove') return 'BoilerTemperatureAbove';
  if (condition.type === 'BoilerTemperatureBelow') return 'BoilerTemperatureBelow';
  if (condition.type === 'BoilerPressureAbove') return 'BoilerPressureAbove';
  if (condition.type === 'BoilerPressureBelow') return 'BoilerPressureBelow';
  if (condition.type === 'GroupInputFlowRateAbove') return 'GroupInputFlowRateAbove';
  if (condition.type === 'GroupInputFlowRateBelow') return 'GroupInputFlowRateBelow';
  if (condition.type === 'GroupPressureAbove') return 'GroupPressureAbove';
  if (condition.type === 'GroupPressureBelow') return 'GroupPressureBelow';
  if (condition.type === 'WaterTapFlowRateAbove') return 'WaterTapFlowRateAbove';
  if (condition.type === 'WaterTapFlowRateBelow') return 'WaterTapFlowRateBelow';
  if (condition.type === 'OutputWeightAbove') return 'OutputWeightAbove';
  if (condition.type === 'OutputWeightBelow') return 'OutputWeightBelow';
  if (condition.type === 'InputVolumeAboveRelativeToStart') return 'InputVolumeAboveRelativeToStart';
  return 'Brewing';
}

// Helper functions to extract values from condition
function extractIndex(condition: StateCondition): number {
  if (condition.type === 'Brewing') return condition.value;
  if (condition.type === 'NotBrewing') return condition.value;
  if (condition.type === 'BoilerTemperatureAbove') return condition.value[0];
  if (condition.type === 'BoilerTemperatureBelow') return condition.value[0];
  if (condition.type === 'BoilerPressureAbove') return condition.value[0];
  if (condition.type === 'BoilerPressureBelow') return condition.value[0];
  if (condition.type === 'GroupInputFlowRateAbove') return condition.value[0];
  if (condition.type === 'GroupInputFlowRateBelow') return condition.value[0];
  if (condition.type === 'GroupPressureAbove') return condition.value[0];
  if (condition.type === 'GroupPressureBelow') return condition.value[0];
  if (condition.type === 'WaterTapFlowRateAbove') return condition.value[0];
  if (condition.type === 'WaterTapFlowRateBelow') return condition.value[0];
  if (condition.type === 'OutputWeightAbove') return condition.value[0];
  if (condition.type === 'OutputWeightBelow') return condition.value[0];
  if (condition.type === 'InputVolumeAboveRelativeToStart') return condition.value[0];
  return 0;
}

function extractThreshold(condition: StateCondition): ParameterValue {
  if (condition.type === 'BoilerTemperatureAbove') return condition.value[1];
  if (condition.type === 'BoilerTemperatureBelow') return condition.value[1];
  if (condition.type === 'BoilerPressureAbove') return condition.value[1];
  if (condition.type === 'BoilerPressureBelow') return condition.value[1];
  if (condition.type === 'GroupInputFlowRateAbove') return condition.value[1];
  if (condition.type === 'GroupInputFlowRateBelow') return condition.value[1];
  if (condition.type === 'GroupPressureAbove') return condition.value[1];
  if (condition.type === 'GroupPressureBelow') return condition.value[1];
  if (condition.type === 'WaterTapFlowRateAbove') return condition.value[1];
  if (condition.type === 'WaterTapFlowRateBelow') return condition.value[1];
  if (condition.type === 'OutputWeightAbove') return condition.value[1];
  if (condition.type === 'OutputWeightBelow') return condition.value[1];
  if (condition.type === 'InputVolumeAboveRelativeToStart') return condition.value[1];
  return { type: 'Static', value: 0 };
}

export function StateConditionEditor({ condition, onChange, parameters, derivedParameters }: StateConditionEditorProps) {
  const conditionType = getConditionType(condition);
  const [index, setIndex] = useState<number>(extractIndex(condition));
  const [threshold, setThreshold] = useState<ParameterValue>(extractThreshold(condition));

  // Update state when condition prop changes (e.g., during parent re-renders from status updates)
  useEffect(() => {
    setIndex(extractIndex(condition));
    setThreshold(extractThreshold(condition));
  }, [condition]);

  const handleTypeChange = (type: ConditionType) => {
    const newIndex = 0;
    const newThreshold: ParameterValue = { type: 'Static', value: 0 };

    let newCondition: StateCondition;
    switch (type) {
      case 'Brewing':
        newCondition = { type: 'Brewing', value: newIndex };
        break;
      case 'NotBrewing':
        newCondition = { type: 'NotBrewing', value: newIndex };
        break;
      case 'BoilerTemperatureAbove':
        newCondition = { type: 'BoilerTemperatureAbove', value: [newIndex, newThreshold] };
        break;
      case 'BoilerTemperatureBelow':
        newCondition = { type: 'BoilerTemperatureBelow', value: [newIndex, newThreshold] };
        break;
      case 'BoilerPressureAbove':
        newCondition = { type: 'BoilerPressureAbove', value: [newIndex, newThreshold] };
        break;
      case 'BoilerPressureBelow':
        newCondition = { type: 'BoilerPressureBelow', value: [newIndex, newThreshold] };
        break;
      case 'GroupInputFlowRateAbove':
        newCondition = { type: 'GroupInputFlowRateAbove', value: [newIndex, newThreshold] };
        break;
      case 'GroupInputFlowRateBelow':
        newCondition = { type: 'GroupInputFlowRateBelow', value: [newIndex, newThreshold] };
        break;
      case 'GroupPressureAbove':
        newCondition = { type: 'GroupPressureAbove', value: [newIndex, newThreshold] };
        break;
      case 'GroupPressureBelow':
        newCondition = { type: 'GroupPressureBelow', value: [newIndex, newThreshold] };
        break;
      case 'WaterTapFlowRateAbove':
        newCondition = { type: 'WaterTapFlowRateAbove', value: [newIndex, newThreshold] };
        break;
      case 'WaterTapFlowRateBelow':
        newCondition = { type: 'WaterTapFlowRateBelow', value: [newIndex, newThreshold] };
        break;
      case 'OutputWeightAbove':
        newCondition = { type: 'OutputWeightAbove', value: [newIndex, newThreshold] };
        break;
      case 'OutputWeightBelow':
        newCondition = { type: 'OutputWeightBelow', value: [newIndex, newThreshold] };
        break;
      case 'InputVolumeAboveRelativeToStart':
        newCondition = { type: 'InputVolumeAboveRelativeToStart', value: [newIndex, newThreshold] };
        break;
      default:
        newCondition = { type: 'Brewing', value: 0 };
    }

    onChange(newCondition);
    setIndex(newIndex);
    setThreshold(newThreshold);
  };

  const handleIndexChange = (newIndex: number) => {
    setIndex(newIndex);
    if (conditionType === 'Brewing') {
      onChange({ type: 'Brewing', value: newIndex });
    } else if (conditionType === 'NotBrewing') {
      onChange({ type: 'NotBrewing', value: newIndex });
    } else {
      // For threshold-based conditions, update the tuple
      onChange({ type: conditionType, value: [newIndex, threshold] } as StateCondition);
    }
  };

  const handleThresholdChange = (newThreshold: ParameterValue) => {
    setThreshold(newThreshold);
    onChange({ type: conditionType, value: [index, newThreshold] } as StateCondition);
  };

  const needsThreshold = !['Brewing', 'NotBrewing'].includes(conditionType);

  const getThresholdUnit = () => {
    if (conditionType.includes('Temperature')) return 'Celsius';
    if (conditionType.includes('Pressure')) return 'Bar';
    if (conditionType.includes('FlowRate')) return 'MillilitersPerSecond';
    if (conditionType.includes('Weight')) return 'Grams';
    if (conditionType.includes('Volume')) return 'MillilitersPerSecond';
    return null;
  };

  const getEntityType = (): EntityType => {
    if (conditionType.includes('Boiler')) return 'boiler';
    if (conditionType.includes('WaterTap')) return 'water_tap';
    return 'group'; // Default for brewing and group conditions
  };

  return (
    <div>
      <div style={{ marginBottom: '1rem' }}>
        <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>Condition Type</label>
        <select
          value={conditionType}
          onChange={(e) => handleTypeChange(e.currentTarget.value as ConditionType)}
          style={{
            width: '100%',
            padding: '0.5rem',
            border: '1px solid #ccc',
            borderRadius: '4px',
            fontSize: '1rem'
          }}
        >
          <optgroup label="Brewing State">
            <option value="Brewing">Brewing</option>
            <option value="NotBrewing">Not Brewing</option>
          </optgroup>
          <optgroup label="Boiler Conditions">
            <option value="BoilerTemperatureAbove">Boiler Temperature Above</option>
            <option value="BoilerTemperatureBelow">Boiler Temperature Below</option>
            <option value="BoilerPressureAbove">Boiler Pressure Above</option>
            <option value="BoilerPressureBelow">Boiler Pressure Below</option>
          </optgroup>
          <optgroup label="Group Conditions">
            <option value="GroupInputFlowRateAbove">Group Input Flow Rate Above</option>
            <option value="GroupInputFlowRateBelow">Group Input Flow Rate Below</option>
            <option value="GroupPressureAbove">Group Pressure Above</option>
            <option value="GroupPressureBelow">Group Pressure Below</option>
            <option value="OutputWeightAbove">Output Weight Above</option>
            <option value="OutputWeightBelow">Output Weight Below</option>
            <option value="InputVolumeAboveRelativeToStart">Input Volume Above (Relative to Start)</option>
          </optgroup>
          <optgroup label="Water Tap Conditions">
            <option value="WaterTapFlowRateAbove">Water Tap Flow Rate Above</option>
            <option value="WaterTapFlowRateBelow">Water Tap Flow Rate Below</option>
          </optgroup>
        </select>
      </div>

      <EntitySelector
        entityType={getEntityType()}
        index={index}
        onChange={handleIndexChange}
      />

      {needsThreshold && (
        <ParameterValueEditor
          value={threshold}
          onChange={handleThresholdChange}
          label="Threshold"
          unit={getThresholdUnit() ? { type: getThresholdUnit()! } as ParameterUnit : null}
          parameters={parameters}
          derivedParameters={derivedParameters}
        />
      )}
    </div>
  );
}
