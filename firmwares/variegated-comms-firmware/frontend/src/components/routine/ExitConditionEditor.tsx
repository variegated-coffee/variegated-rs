import { useState, useEffect } from 'preact/hooks';
import { Button, Dialog, Field, Select, TextInput, tokens } from '@variegated-coffee/ui';
import { NumberField } from '../NumberField';
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

  const jumpOutOfRange = exitType === 'JumpToStep' && (jumpToStep < 0 || jumpToStep > totalSteps - 1);

  return (
    <Dialog
      title={exit ? 'Edit exit condition' : 'Add exit condition'}
      onClose={onCancel}
      width="700px"
      footer={
        <>
          <Button variant="secondary" onClick={onCancel}>
            Cancel
          </Button>
          <Button variant="primary" onClick={handleSave} disabled={jumpOutOfRange}>
            Save
          </Button>
        </>
      }
    >
      <div style={{ display: 'flex', flexDirection: 'column', gap: tokens.space.lg }}>
        {/* An exit condition is a sentence -- "when X, then Y" -- so the two selects are
            labelled as the two halves of one rather than as "Exit Condition Type" and
            "Then Action". */}
        <Field label="Leave this step when">
          {(control) => (
            <Select
              {...control}
              value={conditionType}
              onChange={(value) => setConditionType(value as ExitConditionType)}
              options={[
                { value: 'After', label: 'A time has passed since entering the step' },
                { value: 'AfterDurationRelativeToStart', label: 'A time has passed since the routine started' },
                { value: 'StateConditionMet', label: 'The machine reaches a condition' },
                { value: 'UserAction', label: 'The user presses something' },
                { value: 'Always', label: 'Immediately' },
                { value: 'Never', label: 'Never — hold here' },
              ]}
            />
          )}
        </Field>

        <div style={{
          padding: tokens.space.md,
          backgroundColor: tokens.color.surfaceSunken,
          border: `1px solid ${tokens.color.border}`,
          borderRadius: tokens.radius.sm,
        }}>
          {conditionType === 'Always' && (
            <div style={{ color: tokens.color.inkMuted, fontSize: '0.9rem' }}>
              The step is entered and left in the same cycle. Useful for a step that only
              runs its entry commands.
            </div>
          )}

          {conditionType === 'Never' && (
            <div style={{ color: tokens.color.inkMuted, fontSize: '0.9rem' }}>
              The routine holds here until something outside it intervenes — another exit
              condition on this step, or a cancel.
            </div>
          )}

          {(conditionType === 'After' || conditionType === 'AfterDurationRelativeToStart') && (
            <ParameterValueEditor
              value={afterTime}
              onChange={setAfterTime}
              label={conditionType === 'After' ? 'Duration from step entry' : 'Duration from routine start'}
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
            <NumberField
              label="User action"
              value={userActionIndex}
              onChange={setUserActionIndex}
              min={0}
              help="Which user-triggered action this waits for — a button on the machine."
            />
          )}
        </div>

        <Field label="Then">
          {(control) => (
            <Select
              {...control}
              value={exitType}
              onChange={(value) => setExitType(value as ExitType)}
              options={[
                { value: 'NextStep', label: 'Go to the next step' },
                { value: 'JumpToStep', label: 'Jump to a specific step' },
                { value: 'Finished', label: 'Finish the routine' },
              ]}
            />
          )}
        </Field>

        {exitType === 'JumpToStep' && (
          <div style={{ marginLeft: tokens.space.md }}>
            <NumberField
              label="Jump to step"
              value={jumpToStep}
              onChange={setJumpToStep}
              min={0}
              // The bound was on the input's `max` attribute only, which constrains the
              // spinner and not what can be typed -- so a jump past the end of the routine
              // could be saved.
              max={Math.max(0, totalSteps - 1)}
              help={`This routine has ${totalSteps} step${totalSteps === 1 ? '' : 's'}, numbered from 0.`}
            />
          </div>
        )}

        <Field label="Description" help="Optional. Shown instead of the raw condition wherever this exit appears.">
          {(control) => (
            <TextInput
              {...control}
              value={description}
              onInput={setDescription}
              placeholder="When the puck is saturated, End preinfusion, …"
            />
          )}
        </Field>
      </div>
    </Dialog>
  );
}
