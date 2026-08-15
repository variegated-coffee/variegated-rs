import { useState } from 'preact/hooks';
import { ScheduleAction } from '../schemas/schemas';
import { getCommandSummary } from './CommandSummary';
import { CommandBuilder } from './CommandBuilder';
import { useMachine } from '../contexts/MachineContext';

interface CommandListProps {
  commands: ScheduleAction[];
  onChange: (commands: ScheduleAction[]) => void;
}

const MAX_COMMANDS = 8;

export function CommandList({ commands, onChange }: CommandListProps) {
  const machine = useMachine();
  const [editingIndex, setEditingIndex] = useState<number | null>(null);
  const [isAdding, setIsAdding] = useState(false);

  const handleAdd = (command: ScheduleAction) => {
    onChange([...commands, command]);
    setIsAdding(false);
  };

  const handleEdit = (index: number, command: ScheduleAction) => {
    const newCommands = [...commands];
    newCommands[index] = command;
    onChange(newCommands);
    setEditingIndex(null);
  };

  const handleDelete = (index: number) => {
    const newCommands = commands.filter((_, i) => i !== index);
    onChange(newCommands);
  };

  const handleMoveUp = (index: number) => {
    if (index === 0) return;
    const newCommands = [...commands];
    [newCommands[index - 1], newCommands[index]] = [newCommands[index], newCommands[index - 1]];
    onChange(newCommands);
  };

  const handleMoveDown = (index: number) => {
    if (index === commands.length - 1) return;
    const newCommands = [...commands];
    [newCommands[index], newCommands[index + 1]] = [newCommands[index + 1], newCommands[index]];
    onChange(newCommands);
  };

  return (
    <div style={{ padding: '1rem' }}>
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '1rem' }}>
        <h3 style={{ fontSize: '1.25rem' }}>Commands ({commands.length}/{MAX_COMMANDS})</h3>
        <button
          onClick={() => setIsAdding(true)}
          disabled={commands.length >= MAX_COMMANDS}
          style={{
            padding: '0.5rem 1rem',
            backgroundColor: commands.length >= MAX_COMMANDS ? '#ccc' : '#0066cc',
            color: 'white',
            border: 'none',
            borderRadius: '4px',
            cursor: commands.length >= MAX_COMMANDS ? 'not-allowed' : 'pointer',
            fontSize: '0.9rem'
          }}
        >
          + Add Command
        </button>
      </div>

      {commands.length === 0 ? (
        <div style={{
          padding: '2rem',
          textAlign: 'center',
          color: '#666',
          border: '2px dashed #ccc',
          borderRadius: '4px'
        }}>
          No commands added. Click "Add Command" to get started.
        </div>
      ) : (
        <div style={{ display: 'flex', flexDirection: 'column', gap: '0.5rem' }}>
          {commands.map((command, index) => (
            <div
              key={index}
              style={{
                display: 'flex',
                alignItems: 'center',
                gap: '0.5rem',
                padding: '0.75rem',
                backgroundColor: '#f5f5f5',
                borderRadius: '4px',
                border: '1px solid #ddd'
              }}
            >
              <div style={{ flex: 1, fontSize: '0.9rem' }}>
                <span style={{ fontWeight: '500', marginRight: '0.5rem' }}>#{index + 1}</span>
                {getCommandSummary(command, machine.getBoilerName)}
              </div>

              <div style={{ display: 'flex', gap: '0.25rem' }}>
                <button
                  onClick={() => handleMoveUp(index)}
                  disabled={index === 0}
                  title="Move up"
                  style={{
                    padding: '0.25rem 0.5rem',
                    backgroundColor: index === 0 ? '#eee' : 'white',
                    border: '1px solid #ccc',
                    borderRadius: '4px',
                    cursor: index === 0 ? 'not-allowed' : 'pointer',
                    fontSize: '0.8rem'
                  }}
                >
                  ↑
                </button>
                <button
                  onClick={() => handleMoveDown(index)}
                  disabled={index === commands.length - 1}
                  title="Move down"
                  style={{
                    padding: '0.25rem 0.5rem',
                    backgroundColor: index === commands.length - 1 ? '#eee' : 'white',
                    border: '1px solid #ccc',
                    borderRadius: '4px',
                    cursor: index === commands.length - 1 ? 'not-allowed' : 'pointer',
                    fontSize: '0.8rem'
                  }}
                >
                  ↓
                </button>
                <button
                  onClick={() => setEditingIndex(index)}
                  title="Edit"
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
                  onClick={() => handleDelete(index)}
                  title="Delete"
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

      {isAdding && (
        <CommandBuilder
          command={null}
          onSave={handleAdd}
          onCancel={() => setIsAdding(false)}
        />
      )}

      {editingIndex !== null && (
        <CommandBuilder
          command={commands[editingIndex]}
          onSave={(cmd) => handleEdit(editingIndex, cmd)}
          onCancel={() => setEditingIndex(null)}
        />
      )}
    </div>
  );
}
