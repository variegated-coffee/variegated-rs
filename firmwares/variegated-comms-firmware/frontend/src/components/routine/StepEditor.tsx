import { useState } from 'preact/hooks';
import { Badge, Button, Dialog, EmptyState, Field, TextInput, tokens } from '@variegated-coffee/ui';
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
    // Save is disabled while this holds, and the exit-conditions section says why in
    // place. The `alert()` that used to fire here was telling the user about a rule the
    // form could simply enforce.
    if (exits.length === 0) return;

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

  const rowStyle = {
    padding: tokens.space.sm,
    backgroundColor: tokens.color.surfaceSunken,
    border: `1px solid ${tokens.color.border}`,
    borderRadius: tokens.radius.sm,
    display: 'flex',
    justifyContent: 'space-between',
    alignItems: 'center',
    gap: tokens.space.sm,
    flexWrap: 'wrap' as const,
  };

  return (
    <>
      <Dialog
        title={step ? 'Edit step' : 'New step'}
        onClose={onCancel}
        width="900px"
        footer={
          <>
            <Button variant="secondary" onClick={onCancel}>
              Cancel
            </Button>
            <Button variant="primary" onClick={handleSave} disabled={exits.length === 0}>
              Save step
            </Button>
          </>
        }
      >
        <div style={{ display: 'flex', flexDirection: 'column', gap: tokens.space.lg }}>
          <Field
            label="Description"
            help="Optional. Shown on the step list and while the routine runs."
          >
            {(control) => (
              <TextInput
                {...control}
                value={description}
                onInput={setDescription}
                placeholder="Preinfusion, Main extraction, Pressure ramp, …"
              />
            )}
          </Field>

          <div>
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'flex-start', gap: tokens.space.sm, marginBottom: tokens.space.sm, flexWrap: 'wrap' }}>
              <div>
                <div style={{ display: 'flex', alignItems: 'center', gap: tokens.space.sm }}>
                  <strong>Entry commands</strong>
                  <Badge numeric>{entryCommands.length}</Badge>
                </div>
                <div style={{ fontSize: '0.85rem', color: tokens.color.inkMuted, marginTop: tokens.space.xs }}>
                  Optional. Run in order the moment the step is entered.
                </div>
              </div>
              <Button variant="secondary" size="sm" onClick={() => setIsAddingCommand(true)}>
                Add command
              </Button>
            </div>

            {entryCommands.length === 0 ? (
              <EmptyState
                title="No entry commands"
                detail="A step with none simply waits for its exit condition."
                action={{ label: 'Add command', onClick: () => setIsAddingCommand(true) }}
              />
            ) : (
              <div style={{ display: 'flex', flexDirection: 'column', gap: tokens.space.sm }}>
                {entryCommands.map((cmd, index) => (
                  <div key={index} style={rowStyle}>
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
                        onClick={() => handleMoveCommandUp(index)}
                        disabled={index === 0}
                        ariaLabel={`Move command ${index + 1} earlier`}
                      >
                        <span aria-hidden="true">↑</span>
                      </Button>
                      <Button
                        variant="secondary"
                        size="sm"
                        onClick={() => handleMoveCommandDown(index)}
                        disabled={index === entryCommands.length - 1}
                        ariaLabel={`Move command ${index + 1} later`}
                      >
                        <span aria-hidden="true">↓</span>
                      </Button>
                      <Button
                        variant="secondary"
                        size="sm"
                        onClick={() => setEditingCommandIndex(index)}
                        ariaLabel={`Edit command ${index + 1}`}
                      >
                        Edit
                      </Button>
                      <Button
                        variant="destructive"
                        size="sm"
                        onClick={() => handleDeleteCommand(index)}
                        ariaLabel={`Remove command ${index + 1}`}
                      >
                        Remove
                      </Button>
                    </div>
                  </div>
                ))}
              </div>
            )}
          </div>

          <div>
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'flex-start', gap: tokens.space.sm, marginBottom: tokens.space.sm, flexWrap: 'wrap' }}>
              <div>
                <div style={{ display: 'flex', alignItems: 'center', gap: tokens.space.sm }}>
                  <strong>Exit conditions</strong>
                  <Badge numeric role={exits.length === 0 ? 'warn' : undefined}>
                    {exits.length}
                  </Badge>
                </div>
                <div style={{ fontSize: '0.85rem', color: tokens.color.inkMuted, marginTop: tokens.space.xs }}>
                  Required. The first one to be met decides where the routine goes next.
                </div>
              </div>
              <Button variant="secondary" size="sm" onClick={() => setIsAddingExit(true)}>
                Add exit
              </Button>
            </div>

            {exits.length === 0 ? (
              <EmptyState
                title="No exit conditions"
                detail="Without one the routine would stop here for good, so a step cannot be saved until it has at least one."
                action={{ label: 'Add exit', onClick: () => setIsAddingExit(true) }}
              />
            ) : (
              <div style={{ display: 'flex', flexDirection: 'column', gap: tokens.space.sm }}>
                {exits.map((exit, index) => (
                  <div key={index} style={rowStyle}>
                    <div style={{ flex: 1, minWidth: '12rem' }}>
                      <div style={{ fontSize: '0.9rem' }}>{getExitSummary(exit)}</div>
                      {exit.description && (
                        <div style={{ fontSize: '0.8rem', color: tokens.color.inkMuted, marginTop: tokens.space.xs }}>
                          {exit.description}
                        </div>
                      )}
                    </div>
                    <div style={{ display: 'flex', gap: tokens.space.sm }}>
                      <Button
                        variant="secondary"
                        size="sm"
                        onClick={() => setEditingExitIndex(index)}
                        ariaLabel={`Edit exit condition ${index + 1}`}
                      >
                        Edit
                      </Button>
                      <Button
                        variant="destructive"
                        size="sm"
                        onClick={() => handleDeleteExit(index)}
                        ariaLabel={`Remove exit condition ${index + 1}`}
                      >
                        Remove
                      </Button>
                    </div>
                  </div>
                ))}
              </div>
            )}
          </div>
        </div>
      </Dialog>

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
          // Keyed so switching straight from one command's editor to another's remounts it.
          // The builder seeds its fields from this prop in `useState` initialisers, which
          // React runs once per mount -- without the key, editing command 0 and then command
          // 1 without closing in between would show command 0's values and save them over
          // command 1's.
          key={editingCommandIndex}
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
    </>
  );
}
