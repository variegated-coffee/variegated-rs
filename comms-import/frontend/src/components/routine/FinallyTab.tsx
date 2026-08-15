import { useState } from 'preact/hooks';
import { RoutineCommand, RoutineParameter, DerivedParameter } from '../../schemas/schemas';
import { RoutineCommandBuilder } from './RoutineCommandBuilder';
import { getRoutineCommandSummary } from './RoutineCommandSummary';
import { useMachine } from '../../contexts/MachineContext';

interface FinallyTabProps {
  finallyCommands: RoutineCommand[];
  onFinallyCommandsChange: (commands: RoutineCommand[]) => void;
  parameters: RoutineParameter[];
  derivedParameters: DerivedParameter[];
}

export function FinallyTab({
  finallyCommands,
  onFinallyCommandsChange,
  parameters,
  derivedParameters
}: FinallyTabProps) {
  const machine = useMachine();
  const [editingIndex, setEditingIndex] = useState<number | null>(null);
  const [isAdding, setIsAdding] = useState(false);

  const handleSaveCommand = (cmd: RoutineCommand) => {
    if (editingIndex !== null) {
      const newCommands = [...finallyCommands];
      newCommands[editingIndex] = cmd;
      onFinallyCommandsChange(newCommands);
    } else {
      onFinallyCommandsChange([...finallyCommands, cmd]);
    }
    setEditingIndex(null);
    setIsAdding(false);
  };

  const handleDeleteCommand = (index: number) => {
    if (confirm('Delete this command?')) {
      onFinallyCommandsChange(finallyCommands.filter((_, i) => i !== index));
    }
  };

  const handleMoveUp = (index: number) => {
    if (index === 0) return;
    const newCommands = [...finallyCommands];
    [newCommands[index - 1], newCommands[index]] = [newCommands[index], newCommands[index - 1]];
    onFinallyCommandsChange(newCommands);
  };

  const handleMoveDown = (index: number) => {
    if (index === finallyCommands.length - 1) return;
    const newCommands = [...finallyCommands];
    [newCommands[index], newCommands[index + 1]] = [newCommands[index + 1], newCommands[index]];
    onFinallyCommandsChange(newCommands);
  };

  return (
    <div style={{ padding: '1.5rem' }}>
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '1rem' }}>
        <div>
          <h3 style={{ margin: 0, marginBottom: '0.25rem' }}>Finally Commands ({finallyCommands.length})</h3>
          <p style={{ margin: 0, fontSize: '0.85rem', color: '#666' }}>
            These commands run when the routine completes
          </p>
        </div>
        <button
          onClick={() => setIsAdding(true)}
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

      {finallyCommands.length === 0 ? (
        <div style={{
          padding: '2rem',
          textAlign: 'center',
          color: '#666',
          border: '2px dashed #ccc',
          borderRadius: '4px'
        }}>
          No finally commands defined. These are optional cleanup commands.
        </div>
      ) : (
        <div style={{ display: 'flex', flexDirection: 'column', gap: '0.5rem' }}>
          {finallyCommands.map((cmd, index) => (
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
              <div style={{ flex: 1 }}>
                <div style={{ fontSize: '0.9rem' }}>
                  {index + 1}. {getRoutineCommandSummary(cmd, machine.getBoilerName, machine.getGroupName)}
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
                  disabled={index === finallyCommands.length - 1}
                  style={{
                    padding: '0.25rem 0.5rem',
                    backgroundColor: index === finallyCommands.length - 1 ? '#e0e0e0' : 'white',
                    border: '1px solid #ccc',
                    borderRadius: '4px',
                    cursor: index === finallyCommands.length - 1 ? 'not-allowed' : 'pointer',
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

      {/* Command Builder Modal */}
      {isAdding && (
        <RoutineCommandBuilder
          command={null}
          onSave={handleSaveCommand}
          onCancel={() => setIsAdding(false)}
          parameters={parameters}
          derivedParameters={derivedParameters}
        />
      )}

      {editingIndex !== null && (
        <RoutineCommandBuilder
          command={finallyCommands[editingIndex]}
          onSave={handleSaveCommand}
          onCancel={() => setEditingIndex(null)}
          parameters={parameters}
          derivedParameters={derivedParameters}
        />
      )}
    </div>
  );
}
