import { useState, useEffect } from 'preact/hooks';
import {
  RoutineExit,
  RoutineExitCondition,
  RoutineStepExitType,
  StateCondition,
  ParameterValue,
  RoutineParameter,
  DerivedParameter
} from '../../schemas/schemas';
import { ParameterValueEditor } from './ParameterValueEditor';
import { StateConditionEditor } from './StateConditionEditor';

interface ExitConditionEditorProps {
  exit: RoutineExit | null;
  onSave: (exit: RoutineExit) => void;
  onCancel: () => void;
  parameters: RoutineParameter[];
  derivedParameters: DerivedParameter[];
  totalSteps: number;
}

type ExitConditionType = 'Always' | 'Never' | 'After' | 'AfterDurationRelativeToStart' | 'StateConditionMet' | 'UserAction';
type ExitType = 'NextStep' | 'JumpToStep' | 'Finished';

function getExitConditionType(condition: RoutineExitCondition): ExitConditionType {
  return condition.type as ExitConditionType;
}

function getExitType(exitType: RoutineStepExitType): ExitType {
  return exitType.type as ExitType;
}

// Helper functions to extract values from exit conditions
function extractAfterTime(condition: RoutineExitCondition): ParameterValue {
  if (condition.type === 'After' || condition.type === 'AfterDurationRelativeToStart') {
    return condition.value;
  }
  return { type: 'Static', value: 1 };
}

function extractStateCondition(condition: RoutineExitCondition): StateCondition {
  if (condition.type === 'StateConditionMet') {
    return condition.value;
  }
  return { type: 'Brewing', value: 0 };
}

function extractUserActionIndex(condition: RoutineExitCondition): number {
  if (condition.type === 'UserAction') {
    return condition.value;
  }
  return 0;
}

function extractJumpToStep(then: RoutineStepExitType): number {
  if (then.type === 'JumpToStep') {
    return then.value;
  }
  return 0;
}

export function ExitConditionEditor({
  exit,
  onSave,
  onCancel,
  parameters,
  derivedParameters,
  totalSteps
}: ExitConditionEditorProps) {
  const [conditionType, setConditionType] = useState<ExitConditionType>(
    exit ? getExitConditionType(exit.condition) : 'Always'
  );
  const [exitType, setExitType] = useState<ExitType>(
    exit ? getExitType(exit.then) : 'NextStep'
  );
  const [description, setDescription] = useState(exit?.description || '');

  // Condition-specific state - initialized from exit prop
  const [afterTime, setAfterTime] = useState<ParameterValue>(() =>
    exit ? extractAfterTime(exit.condition) : { type: 'Static', value: 1 }
  );
  const [stateCondition, setStateCondition] = useState<StateCondition>(() =>
    exit ? extractStateCondition(exit.condition) : { type: 'Brewing', value: 0 }
  );
  const [userActionIndex, setUserActionIndex] = useState<number>(
    exit ? extractUserActionIndex(exit.condition) : 0
  );
  const [jumpToStep, setJumpToStep] = useState<number>(
    exit ? extractJumpToStep(exit.then) : 0
  );

  // Update state when exit prop changes (e.g., during parent re-renders from status updates)
  useEffect(() => {
    if (exit) {
      setConditionType(getExitConditionType(exit.condition));
      setExitType(getExitType(exit.then));
      setDescription(exit.description || '');
      setAfterTime(extractAfterTime(exit.condition));
      setStateCondition(extractStateCondition(exit.condition));
      setUserActionIndex(extractUserActionIndex(exit.condition));
      setJumpToStep(extractJumpToStep(exit.then));
    }
  }, [exit]);

  const handleSave = () => {
    let condition: RoutineExitCondition;

    switch (conditionType) {
      case 'Always':
        condition = { type: 'Always' };
        break;
      case 'Never':
        condition = { type: 'Never' };
        break;
      case 'After':
        condition = { type: 'After', value: afterTime };
        break;
      case 'AfterDurationRelativeToStart':
        condition = { type: 'AfterDurationRelativeToStart', value: afterTime };
        break;
      case 'StateConditionMet':
        condition = { type: 'StateConditionMet', value: stateCondition };
        break;
      case 'UserAction':
        condition = { type: 'UserAction', value: userActionIndex };
        break;
    }

    let then: RoutineStepExitType;
    switch (exitType) {
      case 'NextStep':
        then = { type: 'NextStep' };
        break;
      case 'Finished':
        then = { type: 'Finished' };
        break;
      case 'JumpToStep':
        then = { type: 'JumpToStep', value: jumpToStep };
        break;
    }

    onSave({
      condition,
      then,
      description: description.trim() || null
    });
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
      zIndex: 1003
    }}>
      <div style={{
        backgroundColor: 'white',
        borderRadius: '8px',
        padding: '2rem',
        maxWidth: '700px',
        width: '90%',
        maxHeight: '85vh',
        overflow: 'auto'
      }}>
        <h2 style={{ marginBottom: '1.5rem' }}>
          {exit ? 'Edit Exit Condition' : 'Add Exit Condition'}
        </h2>

        {/* Condition Type */}
        <div style={{ marginBottom: '1.5rem' }}>
          <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>
            Exit Condition Type
          </label>
          <select
            value={conditionType}
            onChange={(e) => setConditionType(e.currentTarget.value as ExitConditionType)}
            style={{
              width: '100%',
              padding: '0.5rem',
              border: '1px solid #ccc',
              borderRadius: '4px',
              fontSize: '1rem'
            }}
          >
            <option value="Always">Always (immediate)</option>
            <option value="Never">Never (wait forever)</option>
            <option value="After">After Duration</option>
            <option value="AfterDurationRelativeToStart">After Duration (from routine start)</option>
            <option value="StateConditionMet">When State Condition Met</option>
            <option value="UserAction">On User Action</option>
          </select>
        </div>

        {/* Condition-specific inputs */}
        <div style={{
          padding: '1rem',
          backgroundColor: '#f5f5f5',
          borderRadius: '4px',
          marginBottom: '1.5rem'
        }}>
          {conditionType === 'Always' && (
            <div style={{ color: '#666', fontSize: '0.9rem' }}>
              Exit immediately upon entering this step
            </div>
          )}

          {conditionType === 'Never' && (
            <div style={{ color: '#666', fontSize: '0.9rem' }}>
              Never exit automatically (requires external trigger)
            </div>
          )}

          {(conditionType === 'After' || conditionType === 'AfterDurationRelativeToStart') && (
            <ParameterValueEditor
              value={afterTime}
              onChange={setAfterTime}
              label={conditionType === 'After' ? 'Duration (from step entry)' : 'Duration (from routine start)'}
              unit={{ type: 'Seconds' }}
              parameters={parameters}
              derivedParameters={derivedParameters}
            />
          )}

          {conditionType === 'StateConditionMet' && (
            <StateConditionEditor
              condition={stateCondition}
              onChange={setStateCondition}
              parameters={parameters}
              derivedParameters={derivedParameters}
            />
          )}

          {conditionType === 'UserAction' && (
            <div>
              <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>
                User Action Index
              </label>
              <input
                type="number"
                min="0"
                value={userActionIndex}
                onChange={(e) => setUserActionIndex(parseInt(e.currentTarget.value) || 0)}
                style={{
                  width: '100%',
                  padding: '0.5rem',
                  border: '1px solid #ccc',
                  borderRadius: '4px',
                  fontSize: '1rem'
                }}
              />
              <div style={{ fontSize: '0.85rem', color: '#666', marginTop: '0.25rem' }}>
                Reference to a user-triggered action (e.g., button press)
              </div>
            </div>
          )}
        </div>

        {/* Then Action */}
        <div style={{ marginBottom: '1.5rem' }}>
          <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>
            Then Action
          </label>
          <select
            value={exitType}
            onChange={(e) => setExitType(e.currentTarget.value as ExitType)}
            style={{
              width: '100%',
              padding: '0.5rem',
              border: '1px solid #ccc',
              borderRadius: '4px',
              fontSize: '1rem'
            }}
          >
            <option value="NextStep">Go to Next Step</option>
            <option value="JumpToStep">Jump to Specific Step</option>
            <option value="Finished">Finish Routine</option>
          </select>
        </div>

        {/* Jump target */}
        {exitType === 'JumpToStep' && (
          <div style={{ marginBottom: '1.5rem', marginLeft: '1rem' }}>
            <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>
              Jump to Step
            </label>
            <input
              type="number"
              min="0"
              max={totalSteps - 1}
              value={jumpToStep}
              onChange={(e) => setJumpToStep(parseInt(e.currentTarget.value) || 0)}
              style={{
                width: '100%',
                padding: '0.5rem',
                border: '1px solid #ccc',
                borderRadius: '4px',
                fontSize: '1rem'
              }}
            />
            <div style={{ fontSize: '0.85rem', color: '#666', marginTop: '0.25rem' }}>
              Step index (0-{totalSteps - 1})
            </div>
          </div>
        )}

        {/* Description */}
        <div style={{ marginBottom: '1.5rem' }}>
          <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>
            Description (optional)
          </label>
          <input
            type="text"
            value={description}
            onChange={(e) => setDescription(e.currentTarget.value)}
            placeholder="e.g., When temperature reached, End preinfusion"
            style={{
              width: '100%',
              padding: '0.5rem',
              border: '1px solid #ccc',
              borderRadius: '4px',
              fontSize: '1rem'
            }}
          />
        </div>

        {/* Action Buttons */}
        <div style={{ display: 'flex', gap: '1rem' }}>
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
