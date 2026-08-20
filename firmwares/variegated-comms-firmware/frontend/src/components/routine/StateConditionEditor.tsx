import { useState, useEffect } from 'preact/hooks';
import { StateCondition, ParameterValue, RoutineParameter, DerivedParameter, ParameterUnit, ShotState } from '../../schemas/schemas';
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
  | 'InputVolumeAboveRelativeToStart'
  | 'GroupOutputConductivityAbove'
  | 'GroupOutputConductivityBelow'
  | 'GroupExtractionRateAbove'
  | 'GroupExtractionRateBelow'
  | 'ExtractedSolidsAbove'
  | 'ExtractedSolidsBelow'
  | 'ShotStateReached';

/// The shot phases, in the order a shot passes through them.
//
// `HeadspaceFill` is deliberately absent: it is the phase a shot *starts* in, so a step
// waiting to reach it would exit immediately and mean nothing. The two offered here are the
// two transitions a routine has any reason to wait for.
const SHOT_PHASES: { value: ShotState['type']; label: string; hint: string }[] = [
  {
    value: 'Saturation',
    label: 'Puck is saturated',
    hint: 'Flow has fallen off its peak and pressure has risen off its floor'
  },
  {
    value: 'PostFirstDrop',
    label: 'First drop',
    hint: 'Weight on the scale, or conductivity at the spout'
  }
];

/**
 * Every condition shaped `(index, threshold)`.
 *
 * All of them except `Brewing`/`NotBrewing`, which carry a bare group index. Listing them
 * once replaces three parallel if-chains that had to be extended in lockstep -- the six
 * extraction conditions were added by appending to this array and nothing else.
 */
const THRESHOLD_CONDITIONS = [
  'BoilerTemperatureAbove',
  'BoilerTemperatureBelow',
  'BoilerPressureAbove',
  'BoilerPressureBelow',
  'GroupInputFlowRateAbove',
  'GroupInputFlowRateBelow',
  'GroupPressureAbove',
  'GroupPressureBelow',
  'WaterTapFlowRateAbove',
  'WaterTapFlowRateBelow',
  'OutputWeightAbove',
  'OutputWeightBelow',
  'InputVolumeAboveRelativeToStart',
  'GroupOutputConductivityAbove',
  'GroupOutputConductivityBelow',
  'GroupExtractionRateAbove',
  'GroupExtractionRateBelow',
  'ExtractedSolidsAbove',
  'ExtractedSolidsBelow'
] as const;

function isThresholdCondition(condition: StateCondition): condition is Extract<
  StateCondition,
  { value: [number, ParameterValue] }
> {
  return (THRESHOLD_CONDITIONS as readonly string[]).includes(condition.type);
}

function getConditionType(condition: StateCondition): ConditionType {
  return condition.type as ConditionType;
}

// Helper functions to extract values from condition
function extractIndex(condition: StateCondition): number {
  if (condition.type === 'Brewing' || condition.type === 'NotBrewing') return condition.value;
  if (isThresholdCondition(condition)) return condition.value[0];
  return 0;
}

function extractThreshold(condition: StateCondition): ParameterValue {
  if (isThresholdCondition(condition)) return condition.value[1];
  return { type: 'Static', value: 0 };
}

function extractPhase(condition: StateCondition): ShotState {
  if (condition.type === 'ShotStateReached') return condition.value[1];
  return { type: 'Saturation' };
}

export function StateConditionEditor({ condition, onChange, parameters, derivedParameters }: StateConditionEditorProps) {
  const conditionType = getConditionType(condition);
  const [index, setIndex] = useState<number>(extractIndex(condition));
  const [threshold, setThreshold] = useState<ParameterValue>(extractThreshold(condition));
  const [phase, setPhase] = useState<ShotState>(extractPhase(condition));

  // Update state when condition prop changes (e.g., during parent re-renders from status updates)
  useEffect(() => {
    setIndex(extractIndex(condition));
    setThreshold(extractThreshold(condition));
    setPhase(extractPhase(condition));
  }, [condition]);

  const handleTypeChange = (type: ConditionType) => {
    const newIndex = 0;
    const newThreshold: ParameterValue = { type: 'Static', value: 0 };
    // Saturation rather than HeadspaceFill: a step waiting to reach the phase a shot starts
    // in would exit immediately, which is never what someone picking this meant.
    const newPhase: ShotState = { type: 'Saturation' };

    // Three shapes: `Brewing`/`NotBrewing` carry a bare group index, `ShotStateReached`
    // carries a phase, and everything else carries `(index, threshold)`. The switch this
    // replaced had one arm per variant, all but two of them identical.
    const newCondition: StateCondition = (
      type === 'Brewing' || type === 'NotBrewing'
        ? { type, value: newIndex }
        : type === 'ShotStateReached'
          ? { type, value: [newIndex, newPhase] }
          : { type, value: [newIndex, newThreshold] }
    ) as StateCondition;

    onChange(newCondition);
    setIndex(newIndex);
    setThreshold(newThreshold);
    setPhase(newPhase);
  };

  const handlePhaseChange = (next: ShotState['type']) => {
    const newPhase = { type: next } as ShotState;
    setPhase(newPhase);
    onChange({ type: 'ShotStateReached', value: [index, newPhase] });
  };

  const handleIndexChange = (newIndex: number) => {
    setIndex(newIndex);
    if (conditionType === 'Brewing') {
      onChange({ type: 'Brewing', value: newIndex });
    } else if (conditionType === 'NotBrewing') {
      onChange({ type: 'NotBrewing', value: newIndex });
    } else if (conditionType === 'ShotStateReached') {
      onChange({ type: 'ShotStateReached', value: [newIndex, phase] });
    } else {
      // For threshold-based conditions, update the tuple
      onChange({ type: conditionType, value: [newIndex, threshold] } as StateCondition);
    }
  };

  const handleThresholdChange = (newThreshold: ParameterValue) => {
    setThreshold(newThreshold);
    onChange({ type: conditionType, value: [index, newThreshold] } as StateCondition);
  };

  const needsThreshold = !['Brewing', 'NotBrewing', 'ShotStateReached'].includes(conditionType);

  // An explicit map, not substring matching on the variant name. The version this replaced
  // read `includes('Volume')` and answered `MillilitersPerSecond` for a condition measured in
  // millilitres, and it answered `null` for all six extraction conditions because none of
  // their names happen to contain one of its five magic words.
  const getThresholdUnit = (): ParameterUnit['type'] | null => {
    switch (conditionType) {
      case 'BoilerTemperatureAbove':
      case 'BoilerTemperatureBelow':
        return 'Celsius';
      case 'BoilerPressureAbove':
      case 'BoilerPressureBelow':
      case 'GroupPressureAbove':
      case 'GroupPressureBelow':
        return 'Bar';
      case 'GroupInputFlowRateAbove':
      case 'GroupInputFlowRateBelow':
      case 'WaterTapFlowRateAbove':
      case 'WaterTapFlowRateBelow':
        return 'MillilitersPerSecond';
      case 'OutputWeightAbove':
      case 'OutputWeightBelow':
        return 'Grams';
      case 'InputVolumeAboveRelativeToStart':
        return 'Milliliters';
      case 'GroupOutputConductivityAbove':
      case 'GroupOutputConductivityBelow':
        return 'MillisiemensPerCentimeter';
      case 'GroupExtractionRateAbove':
      case 'GroupExtractionRateBelow':
        return 'ExtractionRate';
      case 'ExtractedSolidsAbove':
      case 'ExtractedSolidsBelow':
        return 'ExtractedSolids';
      // All three are a phase, not a level. Nothing to label.
      case 'Brewing':
      case 'NotBrewing':
      case 'ShotStateReached':
        return null;
    }
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
          <optgroup label="Shot Phase">
            <option value="ShotStateReached">Shot has reached a phase</option>
          </optgroup>
          <optgroup label="Extraction Conditions">
            <option value="GroupOutputConductivityAbove">Output Conductivity Above</option>
            <option value="GroupOutputConductivityBelow">Output Conductivity Below</option>
            <option value="GroupExtractionRateAbove">Extraction Rate Above</option>
            <option value="GroupExtractionRateBelow">Extraction Rate Below</option>
            <option value="ExtractedSolidsAbove">Extracted Solids Above</option>
            <option value="ExtractedSolidsBelow">Extracted Solids Below</option>
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

      {conditionType === 'ShotStateReached' && (
        <div style={{ marginBottom: '1rem' }}>
          <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>Phase</label>
          <select
            value={phase.type}
            onChange={(e) => handlePhaseChange(e.currentTarget.value as ShotState['type'])}
            style={{
              width: '100%',
              padding: '0.5rem',
              border: '1px solid #ccc',
              borderRadius: '4px',
              fontSize: '1rem'
            }}
          >
            {SHOT_PHASES.map(p => (
              <option key={p.value} value={p.value}>{p.label}</option>
            ))}
          </select>
          <div style={{ fontSize: '0.85rem', color: '#666', marginTop: '0.25rem' }}>
            {SHOT_PHASES.find(p => p.value === phase.type)?.hint}
            {' '}Fires once the shot has reached this phase or passed it, and only while a
            shot is running.
          </div>
        </div>
      )}

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
