import { useState } from 'preact/hooks';
import { RoutineStep, RoutineCommand, RoutineExit, RoutineParameter, DerivedParameter } from '../../schemas/schemas';
import { RoutineCommandBuilder } from './RoutineCommandBuilder';
import { ExitConditionEditor } from './ExitConditionEditor';
import { getRoutineCommandSummary } from './RoutineCommandSummary';
import { useMachine } from '../../contexts/MachineContext';

interface StepEditorProps {
  step: RoutineStep | null;
  onSave: (step: RoutineStep) => void;
  onCancel: () => void;
  parameters: RoutineParameter[];
  derivedParameters: DerivedParameter[];
  totalSteps: number;
}

function getExitSummary(exit: RoutineExit): string {
  let condition = '';
  if (exit.condition.type === 'Always') condition = 'Always';
  else if (exit.condition.type === 'Never') condition = 'Never';
  else if (exit.condition.type === 'After') condition = 'After duration';
  else if (exit.condition.type === 'AfterDurationRelativeToStart') condition = 'After duration (from start)';
  else if (exit.condition.type === 'StateConditionMet') condition = 'When state condition met';
  else if (exit.condition.type === 'UserAction') condition = `User action ${exit.condition.value}`;

  let then = '';
  if (exit.then.type === 'NextStep') then = 'Next step';
  else if (exit.then.type === 'Finished') then = 'Finish';
  else if (exit.then.type === 'JumpToStep') then = `Jump to step ${exit.then.value}`;

  return `${condition} → ${then}`;
}

export function StepEditor({ step, onSave, onCancel, parameters, derivedParameters, totalSteps }: StepEditorProps) {
  const machine = useMachine();
  const [description, setDescription] = useState(step?.description || '');
  const [entryCommands, setEntryCommands] = useState<RoutineCommand[]>(step?.entry_command || []);
  const [exits, setExits] = useState<RoutineExit[]>(step?.exits || []);

  const [editingCommandIndex, setEditingCommandIndex] = useState<number | null>(null);
  const [isAddingCommand, setIsAddingCommand] = useState(false);
  const [editingExitIndex, setEditingExitIndex] = useState<number | null>(null);
  const [isAddingExit, setIsAddingExit] = useState(false);

  const handleSave = () => {
    if (exits.length === 0) {
      alert('At least one exit condition is required');
      return;
    }

    onSave({
      description: description.trim() || null,
      entry_command: entryCommands,
      exits
    });
  };

  const handleSaveCommand = (cmd: RoutineCommand) => {
    if (editingCommandIndex !== null) {
      const newCommands = [...entryCommands];
      newCommands[editingCommandIndex] = cmd;
      setEntryCommands(newCommands);
    } else {
      setEntryCommands([...entryCommands, cmd]);
    }
    setEditingCommandIndex(null);
    setIsAddingCommand(false);
  };

  const handleDeleteCommand = (index: number) => {
    setEntryCommands(entryCommands.filter((_, i) => i !== index));
  };

  const handleMoveCommandUp = (index: number) => {
    if (index === 0) return;
    const newCommands = [...entryCommands];
    [newCommands[index - 1], newCommands[index]] = [newCommands[index], newCommands[index - 1]];
    setEntryCommands(newCommands);
  };

  const handleMoveCommandDown = (index: number) => {
    if (index === entryCommands.length - 1) return;
    const newCommands = [...entryCommands];
    [newCommands[index], newCommands[index + 1]] = [newCommands[index + 1], newCommands[index]];
    setEntryCommands(newCommands);
  };

  const handleAddExit = (exit: RoutineExit) => {
    setExits([...exits, exit]);
    setIsAddingExit(false);
  };

  const handleEditExit = (index: number, exit: RoutineExit) => {
    const newExits = [...exits];
    newExits[index] = exit;
    setExits(newExits);
    setEditingExitIndex(null);
  };

  const handleDeleteExit = (index: number) => {
    setExits(exits.filter((_, i) => i !== index));
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
      zIndex: 1000,
      padding: '1rem'
    }}>
      <div style={{
        backgroundColor: 'white',
        borderRadius: '8px',
        maxWidth: '900px',
        width: '100%',
        maxHeight: '90vh',
        display: 'flex',
        flexDirection: 'column',
        overflow: 'hidden'
      }}>
        {/* Header */}
        <div style={{ padding: '1.5rem', borderBottom: '1px solid #ddd' }}>
          <h2 style={{ margin: 0 }}>{step ? 'Edit Step' : 'New Step'}</h2>
        </div>

        {/* Content */}
        <div style={{ flex: 1, overflow: 'auto', padding: '1.5rem' }}>
          {/* Description */}
          <div style={{ marginBottom: '1.5rem' }}>
            <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>
              Description (optional)
            </label>
            <input
              type="text"
              value={description}
              onChange={(e) => setDescription(e.currentTarget.value)}
              placeholder="e.g., Preinfusion, Main extraction, Pressure ramp"
              style={{
                width: '100%',
                padding: '0.5rem',
                border: '1px solid #ccc',
                borderRadius: '4px',
                fontSize: '1rem'
              }}
            />
          </div>

          {/* Entry Commands */}
          <div style={{ marginBottom: '1.5rem' }}>
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '0.5rem' }}>
              <label style={{ fontWeight: '500' }}>
                Entry Commands ({entryCommands.length}) <span style={{ fontWeight: 'normal', fontSize: '0.85rem', color: '#666' }}>- optional, execute in order</span>
              </label>
              <button
                onClick={() => setIsAddingCommand(true)}
                style={{
                  padding: '0.5rem 1rem',
                  backgroundColor: '#28a745',
                  color: 'white',
                  border: 'none',
                  borderRadius: '4px',
                  cursor: 'pointer',
                  fontSize: '0.9rem'
                }}
              >
                + Add Command
              </button>
            </div>

            {entryCommands.length === 0 ? (
              <div style={{
                padding: '1rem',
                textAlign: 'center',
                color: '#666',
                border: '2px dashed #ccc',
                borderRadius: '4px',
                fontSize: '0.9rem'
              }}>
                No entry commands. Commands execute when the step starts.
              </div>
            ) : (
              <div style={{ display: 'flex', flexDirection: 'column', gap: '0.5rem' }}>
                {entryCommands.map((cmd, index) => (
                  <div
                    key={index}
                    style={{
                      padding: '0.75rem',
                      backgroundColor: '#f5f5f5',
                      borderRadius: '4px',
                      display: 'flex',
                      justifyContent: 'space-between',
                      alignItems: 'center'
                    }}
                  >
                    <div style={{ flex: 1, fontSize: '0.9rem' }}>
                      {index + 1}. {getRoutineCommandSummary(cmd, machine.getBoilerName, machine.getGroupName)}
                    </div>

                    <div style={{ display: 'flex', gap: '0.5rem', marginLeft: '1rem' }}>
                      <button
                        onClick={() => handleMoveCommandUp(index)}
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
                        onClick={() => handleMoveCommandDown(index)}
                        disabled={index === entryCommands.length - 1}
                        style={{
                          padding: '0.25rem 0.5rem',
                          backgroundColor: index === entryCommands.length - 1 ? '#e0e0e0' : 'white',
                          border: '1px solid #ccc',
                          borderRadius: '4px',
                          cursor: index === entryCommands.length - 1 ? 'not-allowed' : 'pointer',
                          fontSize: '0.8rem'
                        }}
                        title="Move down"
                      >
                        ↓
                      </button>
                      <button
                        onClick={() => setEditingCommandIndex(index)}
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
                        onClick={() => handleDeleteCommand(index)}
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
                ))}
              </div>
            )}
          </div>

          {/* Exit Conditions */}
          <div>
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '0.5rem' }}>
              <label style={{ fontWeight: '500' }}>Exit Conditions ({exits.length})</label>
              <button
                onClick={() => setIsAddingExit(true)}
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
                + Add Exit
              </button>
            </div>

            {exits.length === 0 ? (
              <div style={{
                padding: '2rem',
                textAlign: 'center',
                color: '#666',
                border: '2px dashed #ccc',
                borderRadius: '4px'
              }}>
                No exit conditions. Click "Add Exit" to add one.
              </div>
            ) : (
              <div style={{ display: 'flex', flexDirection: 'column', gap: '0.5rem' }}>
                {exits.map((exit, index) => (
                  <div
                    key={index}
                    style={{
                      padding: '0.75rem',
                      backgroundColor: '#f5f5f5',
                      borderRadius: '4px',
                      display: 'flex',
                      justifyContent: 'space-between',
                      alignItems: 'center'
                    }}
                  >
                    <div>
                      <div style={{ fontSize: '0.9rem' }}>{getExitSummary(exit)}</div>
                      {exit.description && (
                        <div style={{ fontSize: '0.8rem', color: '#666', marginTop: '0.25rem' }}>
                          {exit.description}
                        </div>
                      )}
                    </div>
                    <div style={{ display: 'flex', gap: '0.5rem' }}>
                      <button
                        onClick={() => setEditingExitIndex(index)}
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
                        onClick={() => handleDeleteExit(index)}
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
                ))}
              </div>
            )}
          </div>
        </div>

        {/* Footer */}
        <div style={{
          padding: '1rem 1.5rem',
          borderTop: '1px solid #ddd',
          display: 'flex',
          gap: '1rem',
          justifyContent: 'flex-end'
        }}>
          <button
            onClick={onCancel}
            style={{
              padding: '0.75rem 1.5rem',
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
          <button
            onClick={handleSave}
            disabled={exits.length === 0}
            style={{
              padding: '0.75rem 1.5rem',
              backgroundColor: exits.length === 0 ? '#ccc' : '#0066cc',
              color: 'white',
              border: 'none',
              borderRadius: '4px',
              fontSize: '1rem',
              cursor: exits.length === 0 ? 'not-allowed' : 'pointer'
            }}
          >
            Save Step
          </button>
        </div>
      </div>

      {/* Nested modals */}
      {isAddingCommand && (
        <RoutineCommandBuilder
          command={null}
          onSave={handleSaveCommand}
          onCancel={() => setIsAddingCommand(false)}
          parameters={parameters}
          derivedParameters={derivedParameters}
        />
      )}

      {editingCommandIndex !== null && (
        <RoutineCommandBuilder
          command={entryCommands[editingCommandIndex]}
          onSave={handleSaveCommand}
          onCancel={() => setEditingCommandIndex(null)}
          parameters={parameters}
          derivedParameters={derivedParameters}
        />
      )}

      {isAddingExit && (
        <ExitConditionEditor
          exit={null}
          onSave={handleAddExit}
          onCancel={() => setIsAddingExit(false)}
          parameters={parameters}
          derivedParameters={derivedParameters}
          totalSteps={totalSteps}
        />
      )}

      {editingExitIndex !== null && (
        <ExitConditionEditor
          exit={exits[editingExitIndex]}
          onSave={(exit) => handleEditExit(editingExitIndex, exit)}
          onCancel={() => setEditingExitIndex(null)}
          parameters={parameters}
          derivedParameters={derivedParameters}
          totalSteps={totalSteps}
        />
      )}
    </div>
  );
}
