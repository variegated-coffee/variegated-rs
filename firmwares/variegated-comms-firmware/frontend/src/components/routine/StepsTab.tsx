import { useState } from 'preact/hooks';
import { Alert, Badge, Button, EmptyState, tokens, useDialogs } from '@variegated-coffee/ui';
import { RoutineStep, RoutineParameter, DerivedParameter } from '../../schemas/schemas';
import { StepEditor } from './StepEditor';
import { getRoutineCommandSummary } from './RoutineCommandSummary';
import { useMachine } from '../../contexts/MachineContext';
import { formatExitConditionsSummary } from '../../utils/exitConditionFormatter';
import { transitionWarnings } from '../../utils/transitionWarnings';

interface StepsTabProps {
  steps: RoutineStep[];
  onStepsChange: (steps: RoutineStep[]) => void;
  parameters: RoutineParameter[];
  derivedParameters: DerivedParameter[];
}

export function StepsTab({ steps, onStepsChange, parameters, derivedParameters }: StepsTabProps) {
  const machine = useMachine();
  const { confirm } = useDialogs();
  const [editingIndex, setEditingIndex] = useState<number | null>(null);
  const [isAdding, setIsAdding] = useState(false);

  // Recomputed on every render rather than memoised: routines are a handful of steps, and a
  // stale warning about a step the user just edited is worse than the work saved.
  const warningsByStep = new Map<number, string[]>();
  for (const warning of transitionWarnings(steps)) {
    const existing = warningsByStep.get(warning.step) ?? [];
    existing.push(warning.message);
    warningsByStep.set(warning.step, existing);
  }

  const handleSaveStep = (step: RoutineStep) => {
    if (editingIndex !== null) {
      const newSteps = [...steps];
      newSteps[editingIndex] = step;
      onStepsChange(newSteps);
    } else {
      onStepsChange([...steps, step]);
    }
    setEditingIndex(null);
    setIsAdding(false);
  };

  const handleDeleteStep = async (index: number) => {
    const step = steps[index];
    const ok = await confirm({
      title: step?.description ? `Delete step ${index} — ${step.description}?` : `Delete step ${index}?`,
      // Steps are addressed by position, so removing one renumbers everything after it.
      body:
        index < steps.length - 1
          ? 'The steps after it move up, and any exit condition that jumps to a step by number will point somewhere else.'
          : undefined,
      confirmLabel: 'Delete',
      destructive: true,
    });
    if (ok) onStepsChange(steps.filter((_, i) => i !== index));
  };

  const handleMoveUp = (index: number) => {
    if (index === 0) return;
    const newSteps = [...steps];
    [newSteps[index - 1], newSteps[index]] = [newSteps[index], newSteps[index - 1]];
    onStepsChange(newSteps);
  };

  const handleMoveDown = (index: number) => {
    if (index === steps.length - 1) return;
    const newSteps = [...steps];
    [newSteps[index], newSteps[index + 1]] = [newSteps[index + 1], newSteps[index]];
    onStepsChange(newSteps);
  };

  return (
    <div style={{ padding: tokens.space.lg }}>
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', gap: tokens.space.sm, marginBottom: tokens.space.md, flexWrap: 'wrap' }}>
        <div style={{ display: 'flex', alignItems: 'center', gap: tokens.space.sm }}>
          <h3 style={{ margin: 0 }}>Steps</h3>
          <Badge numeric>{steps.length}</Badge>
        </div>
        <Button variant="secondary" size="sm" onClick={() => setIsAdding(true)}>
          Add step
        </Button>
      </div>

      {steps.length === 0 ? (
        <EmptyState
          title="No steps"
          detail="A step sets the machine up, then waits for a condition before moving on. A routine needs at least one."
          action={{ label: 'Add step', onClick: () => setIsAdding(true) }}
        />
      ) : (
        <div style={{ display: 'flex', flexDirection: 'column', gap: tokens.space.sm }}>
          {steps.map((step, index) => (
            <div
              key={index}
              style={{
                padding: tokens.space.md,
                backgroundColor: tokens.color.surfaceSunken,
                border: `1px solid ${tokens.color.border}`,
                borderRadius: tokens.radius.sm,
                borderLeft: `4px solid ${tokens.color.info}`,
              }}
            >
              <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'flex-start', gap: tokens.space.md, flexWrap: 'wrap' }}>
                <div style={{ flex: 1, minWidth: '14rem' }}>
                  <div style={{ fontWeight: 500, marginBottom: tokens.space.sm, fontSize: '1rem' }}>
                    <span style={{ fontFamily: tokens.font.mono }}>Step {index}</span>
                    {step.description && (
                      <span style={{ color: tokens.color.inkMuted, fontWeight: 'normal' }}>
                        {' '}— {step.description}
                      </span>
                    )}
                  </div>

                  {/* A transition whose starting point depends on how the step was entered.
                      Advisory rather than blocking: the firmware has defined behaviour for
                      it, and it is occasionally exactly what was meant. See
                      `utils/transitionWarnings`. */}
                  {(warningsByStep.get(index) ?? []).map((message, i) => (
                    <div key={i} style={{ marginBottom: tokens.space.sm }}>
                      <Alert role="warn">{message}</Alert>
                    </div>
                  ))}

                  {step.entry_command.length > 0 && (
                    <div style={{ fontSize: '0.85rem', color: tokens.color.ink, marginBottom: tokens.space.xs }}>
                      <span style={{ fontWeight: 500 }}>Entry:</span>{' '}
                      {step.entry_command.length === 1
                        ? getRoutineCommandSummary(step.entry_command[0], machine.getBoilerName, machine.getGroupName)
                        : `${step.entry_command.length} commands: ${step.entry_command.map((cmd, i) => `${i + 1}. ${getRoutineCommandSummary(cmd, machine.getBoilerName, machine.getGroupName)}`).join('; ')}`
                      }
                    </div>
                  )}

                  <div style={{ fontSize: '0.85rem', color: tokens.color.inkMuted }}>
                    {formatExitConditionsSummary(step.exits, machine.getBoilerName, machine.getGroupName)}
                  </div>
                </div>

                <div style={{ display: 'flex', gap: tokens.space.sm, flexWrap: 'wrap' }}>
                  <Button
                    variant="secondary"
                    size="sm"
                    onClick={() => handleMoveUp(index)}
                    disabled={index === 0}
                    ariaLabel={`Move step ${index} earlier`}
                  >
                    <span aria-hidden="true">↑</span>
                  </Button>
                  <Button
                    variant="secondary"
                    size="sm"
                    onClick={() => handleMoveDown(index)}
                    disabled={index === steps.length - 1}
                    ariaLabel={`Move step ${index} later`}
                  >
                    <span aria-hidden="true">↓</span>
                  </Button>
                  <Button
                    variant="secondary"
                    size="sm"
                    onClick={() => setEditingIndex(index)}
                    ariaLabel={`Edit step ${index}`}
                  >
                    Edit
                  </Button>
                  <Button
                    variant="destructive"
                    size="sm"
                    onClick={() => void handleDeleteStep(index)}
                    ariaLabel={`Delete step ${index}`}
                  >
                    Delete
                  </Button>
                </div>
              </div>
            </div>
          ))}
        </div>
      )}

      {/* Step Editor Modal */}
      {isAdding && (
        <StepEditor
          step={null}
          onSave={handleSaveStep}
          onCancel={() => setIsAdding(false)}
          parameters={parameters}
          derivedParameters={derivedParameters}
          totalSteps={steps.length}
        />
      )}

      {editingIndex !== null && (
        <StepEditor
          step={steps[editingIndex]}
          onSave={handleSaveStep}
          onCancel={() => setEditingIndex(null)}
          parameters={parameters}
          derivedParameters={derivedParameters}
          totalSteps={steps.length}
        />
      )}
    </div>
  );
}
