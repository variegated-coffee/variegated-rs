import { useState } from 'preact/hooks';
import { Badge, Button, EmptyState, tokens } from '@variegated-coffee/ui';
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
    <div style={{ display: 'flex', flexDirection: 'column', gap: tokens.space.md, padding: tokens.space.md }}>
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', gap: tokens.space.sm, flexWrap: 'wrap' }}>
        <div style={{ display: 'flex', alignItems: 'center', gap: tokens.space.sm }}>
          <h3 style={{ margin: 0, fontSize: '1.25rem' }}>Commands</h3>
          <Badge numeric>{commands.length}/{MAX_COMMANDS}</Badge>
        </div>
        <Button
          variant="primary"
          size="sm"
          onClick={() => setIsAdding(true)}
          disabled={commands.length >= MAX_COMMANDS}
        >
          Add command
        </Button>
      </div>

      {commands.length === 0 ? (
        <EmptyState
          title="No commands yet"
          detail="A schedule needs at least one command — what the machine should do when it fires."
          action={{ label: 'Add command', onClick: () => setIsAdding(true) }}
        />
      ) : (
        <div style={{ display: 'flex', flexDirection: 'column', gap: tokens.space.sm }}>
          {commands.map((command, index) => (
            <div
              key={index}
              style={{
                display: 'flex',
                alignItems: 'center',
                gap: tokens.space.sm,
                padding: tokens.space.sm,
                backgroundColor: tokens.color.surfaceSunken,
                borderRadius: tokens.radius.sm,
                border: `1px solid ${tokens.color.border}`,
                flexWrap: 'wrap',
              }}
            >
              <div style={{ flex: 1, minWidth: '12rem', fontSize: '0.9rem' }}>
                <span
                  style={{
                    fontWeight: 500,
                    marginRight: tokens.space.sm,
                    fontFamily: tokens.font.mono,
                    fontVariantNumeric: 'tabular-nums',
                    color: tokens.color.inkMuted,
                  }}
                >
                  #{index + 1}
                </span>
                {getCommandSummary(command, machine.getBoilerName)}
              </div>

              {/* The arrows keep their glyphs -- they are compact and the direction is the
                  whole meaning -- but the name is now on the button rather than in a
                  `title` a touchscreen cannot surface. */}
              <div style={{ display: 'flex', gap: tokens.space.xs }}>
                <Button
                  variant="secondary"
                  size="sm"
                  onClick={() => handleMoveUp(index)}
                  disabled={index === 0}
                  ariaLabel={`Move command ${index + 1} up`}
                >
                  <span aria-hidden="true">↑</span>
                </Button>
                <Button
                  variant="secondary"
                  size="sm"
                  onClick={() => handleMoveDown(index)}
                  disabled={index === commands.length - 1}
                  ariaLabel={`Move command ${index + 1} down`}
                >
                  <span aria-hidden="true">↓</span>
                </Button>
                <Button
                  variant="secondary"
                  size="sm"
                  onClick={() => setEditingIndex(index)}
                  ariaLabel={`Edit command ${index + 1}`}
                >
                  Edit
                </Button>
                {/* Was a filled red `✕` with no label. Removing one command from a list
                    being edited is not irreversible -- nothing is saved until the schedule
                    is -- so it does not need a confirmation, but it does need to stop
                    being the loudest thing in the row. */}
                <Button
                  variant="destructive"
                  size="sm"
                  onClick={() => handleDelete(index)}
                  ariaLabel={`Remove command ${index + 1}`}
                >
                  Remove
                </Button>
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
