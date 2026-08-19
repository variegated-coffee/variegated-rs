import { useState } from 'preact/hooks';
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

  const handleDeleteStep = (index: number) => {
    if (confirm(`Delete step ${index}?`)) {
      onStepsChange(steps.filter((_, i) => i !== index));
    }
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
    <div style={{ padding: '1.5rem' }}>
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '1rem' }}>
        <h3 style={{ margin: 0 }}>Steps ({steps.length})</h3>
        <button
          onClick={() => setIsAdding(true)}
          style={{
            padding: '0.5rem 1rem',
            backgroundColor: '#0066cc',
            color: 'white',
            border: 'none',
            borderRadius: '4px',
            cursor: 'pointer',
            fontSize: '0.9rem'
          }}
        >
          + Add Step
        </button>
      </div>

      {steps.length === 0 ? (
        <div style={{
          padding: '2rem',
          textAlign: 'center',
          color: '#666',
          border: '2px dashed #ccc',
          borderRadius: '4px'
        }}>
          No steps defined. Click "Add Step" to create the first step.
        </div>
      ) : (
        <div style={{ display: 'flex', flexDirection: 'column', gap: '0.75rem' }}>
          {steps.map((step, index) => (
            <div
              key={index}
              style={{
                padding: '1rem',
                backgroundColor: '#f5f5f5',
                borderRadius: '4px',
                borderLeft: '4px solid #0066cc'
              }}
            >
              <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'start' }}>
                <div style={{ flex: 1 }}>
                  <div style={{ fontWeight: '500', marginBottom: '0.5rem', fontSize: '1rem' }}>
                    Step {index}
                    {step.description && <span style={{ color: '#666', fontWeight: 'normal' }}>: {step.description}</span>}
                  </div>

                  {/* A transition whose starting point depends on how the step was entered.
                      Advisory rather than blocking: the firmware has defined behaviour for
                      it, and it is occasionally exactly what was meant. See
                      `utils/transitionWarnings`. */}
                  {(warningsByStep.get(index) ?? []).map((message, i) => (
                    <div
                      key={i}
                      style={{
                        fontSize: '0.85rem',
                        color: '#8a6d00',
                        backgroundColor: '#fff8e1',
                        border: '1px solid #ffe082',
                        borderRadius: '3px',
                        padding: '0.4rem 0.6rem',
                        marginBottom: '0.5rem',
                      }}
                    >
                      ⚠ {message}
                    </div>
                  ))}

                  {step.entry_command.length > 0 && (
                    <div style={{ fontSize: '0.85rem', color: '#444', marginBottom: '0.25rem' }}>
                      <span style={{ fontWeight: '500' }}>Entry:</span>{' '}
                      {step.entry_command.length === 1
                        ? getRoutineCommandSummary(step.entry_command[0], machine.getBoilerName, machine.getGroupName)
                        : `${step.entry_command.length} commands: ${step.entry_command.map((cmd, i) => `${i + 1}. ${getRoutineCommandSummary(cmd, machine.getBoilerName, machine.getGroupName)}`).join('; ')}`
                      }
                    </div>
                  )}

                  <div style={{ fontSize: '0.85rem', color: '#666' }}>
                    {formatExitConditionsSummary(step.exits, machine.getBoilerName, machine.getGroupName)}
                  </div>
                </div>

                <div style={{ display: 'flex', gap: '0.5rem', marginLeft: '1rem' }}>
                  <button
                    onClick={() => handleMoveUp(index)}
                    disabled={index === 0}
                    style={{
                      padding: '0.25rem 0.5rem',
                      backgroundColor: index === 0 ? '#e0e0e0' : 'white',
                      border: '1px solid #ccc',
                      borderRadius: '4px',
                      cursor: index === 0 ? 'not-allowed' : 'pointer',
                      fontSize: '0.8rem'
                    }}
                    title="Move up"
                  >
                    ↑
                  </button>
                  <button
                    onClick={() => handleMoveDown(index)}
                    disabled={index === steps.length - 1}
                    style={{
                      padding: '0.25rem 0.5rem',
                      backgroundColor: index === steps.length - 1 ? '#e0e0e0' : 'white',
                      border: '1px solid #ccc',
                      borderRadius: '4px',
                      cursor: index === steps.length - 1 ? 'not-allowed' : 'pointer',
                      fontSize: '0.8rem'
                    }}
                    title="Move down"
                  >
                    ↓
                  </button>
                  <button
                    onClick={() => setEditingIndex(index)}
                    style={{
                      padding: '0.25rem 0.5rem',
                      backgroundColor: 'white',
                      border: '1px solid #ccc',
                      borderRadius: '4px',
                      cursor: 'pointer',
                      fontSize: '0.8rem'
                    }}
                  >
                    Edit
                  </button>
                  <button
                    onClick={() => handleDeleteStep(index)}
                    style={{
                      padding: '0.25rem 0.5rem',
                      backgroundColor: '#dc3545',
                      color: 'white',
                      border: 'none',
                      borderRadius: '4px',
                      cursor: 'pointer',
                      fontSize: '0.8rem'
                    }}
                  >
                    ✕
                  </button>
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
