import { useState } from 'preact/hooks';
import { Badge, Button, EmptyState, tokens, useDialogs } from '@variegated-coffee/ui';
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
  const { confirm } = useDialogs();
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

  const handleDeleteCommand = async (index: number) => {
    const ok = await confirm({
      title: `Remove finally command ${index + 1}?`,
      body: getRoutineCommandSummary(
        finallyCommands[index],
        machine.getBoilerName,
        machine.getGroupName
      ),
      confirmLabel: 'Remove',
      destructive: true,
    });
    if (ok) onFinallyCommandsChange(finallyCommands.filter((_, i) => i !== index));
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
    <div style={{ padding: tokens.space.lg }}>
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'flex-start', gap: tokens.space.sm, marginBottom: tokens.space.md, flexWrap: 'wrap' }}>
        <div>
          <div style={{ display: 'flex', alignItems: 'center', gap: tokens.space.sm, marginBottom: tokens.space.xs }}>
            <h3 style={{ margin: 0 }}>Finally</h3>
            <Badge numeric>{finallyCommands.length}</Badge>
          </div>
          <p style={{ margin: 0, fontSize: '0.85rem', color: tokens.color.inkMuted }}>
            Run when the routine ends, however it ends.
          </p>
        </div>
        <Button variant="secondary" size="sm" onClick={() => setIsAdding(true)}>
          Add command
        </Button>
      </div>

      {finallyCommands.length === 0 ? (
        <EmptyState
          title="No finally commands"
          detail="Optional. Use these to put the machine back to a known state — pump off, valve closed — whether the routine finished or was cancelled."
          action={{ label: 'Add command', onClick: () => setIsAdding(true) }}
        />
      ) : (
        <div style={{ display: 'flex', flexDirection: 'column', gap: tokens.space.sm }}>
          {finallyCommands.map((cmd, index) => (
            <div
              key={index}
              style={{
                padding: tokens.space.sm,
                backgroundColor: tokens.color.surfaceSunken,
                border: `1px solid ${tokens.color.border}`,
                borderRadius: tokens.radius.sm,
                display: 'flex',
                justifyContent: 'space-between',
                alignItems: 'center',
                gap: tokens.space.sm,
                flexWrap: 'wrap',
              }}
            >
              <div style={{ flex: 1, minWidth: '12rem', fontSize: '0.9rem' }}>
                <span
                  style={{
                    fontFamily: tokens.font.mono,
                    fontVariantNumeric: 'tabular-nums',
                    color: tokens.color.inkMuted,
                    marginRight: tokens.space.sm,
                  }}
                >
                  #{index + 1}
                </span>
                {getRoutineCommandSummary(cmd, machine.getBoilerName, machine.getGroupName)}
              </div>

              <div style={{ display: 'flex', gap: tokens.space.sm, flexWrap: 'wrap' }}>
                <Button
                  variant="secondary"
                  size="sm"
                  onClick={() => handleMoveUp(index)}
                  disabled={index === 0}
                  ariaLabel={`Move command ${index + 1} earlier`}
                >
                  <span aria-hidden="true">↑</span>
                </Button>
                <Button
                  variant="secondary"
                  size="sm"
                  onClick={() => handleMoveDown(index)}
                  disabled={index === finallyCommands.length - 1}
                  ariaLabel={`Move command ${index + 1} later`}
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
                <Button
                  variant="destructive"
                  size="sm"
                  onClick={() => void handleDeleteCommand(index)}
                  ariaLabel={`Remove command ${index + 1}`}
                >
                  Remove
                </Button>
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
