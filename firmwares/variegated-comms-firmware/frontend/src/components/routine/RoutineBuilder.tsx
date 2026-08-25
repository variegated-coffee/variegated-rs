import type { ComponentChildren } from 'preact';
import { useEffect, useState } from 'preact/hooks';
import { memo } from 'preact/compat';
import {
  Alert,
  Badge,
  Button,
  EmptyState,
  Tabs,
  tokens,
  useDialogs,
} from '@variegated-coffee/ui';
import { Routine, MachineDefinition, PeripheralStatus, RoutineSummary, RoutineSummaryStorage } from '../../schemas/schemas';
import {
  capabilityLabel,
  getRoutineTypeLabel,
  indexFromIdentifier,
  RoutineIdentifier,
  unmetPrerequisites
} from '../../utils/routineHelpers';
import { RoutineEditor } from './RoutineEditor';
import { getWebSocketService } from '../../services/websocket';
import { createRoutine, deleteRoutine, saveRoutine } from '../../api/routines';
import { invalidateRoutineBody, loadRoutineBody, useRoutineBody } from '../../state/routineBodies';

// No `onRefresh`. Saving, deleting and duplicating used to call one, and it could only
// ever re-read the comms processor's cache -- which, called immediately after a write,
// had not yet heard about it. The application processor now pushes a fresh summary list
// whenever its repository changes, so `routines` updates on its own a few milliseconds
// later, from the machine rather than from a cache that was asked too early.
interface RoutineBuilderProps {
  routines: RoutineSummaryStorage;
  machineDefinition: MachineDefinition | null;
  /**
   * Which peripherals are answering, for deciding whether a routine can run.
   *
   * Together with `machineDefinition` this is what greys a card. `null` means "cannot tell"
   * and everything reads as runnable -- the machine's own backstop still refuses anything
   * that really cannot run, and greying the whole list because a status has not arrived yet
   * would be the worse lie.
   */
  peripheralStatus: PeripheralStatus | null;
}

const RoutineBuilderComponent = ({ routines, machineDefinition, peripheralStatus }: RoutineBuilderProps) => {
  const { confirm } = useDialogs();
  const [editingRoutine, setEditingRoutine] = useState<RoutineIdentifier | null>(null);
  const [isAdding, setIsAdding] = useState(false);
  const [addingType, setAddingType] = useState<'custom' | 'function'>('custom');
  const [addingFunctionIndex, setAddingFunctionIndex] = useState<number>(0);
  const [activeTab, setActiveTab] = useState<'custom' | 'function' | 'internal'>('custom');
  const [error, setError] = useState<string | null>(null);
  const [successMessage, setSuccessMessage] = useState<string | null>(null);

  const showSuccess = (message: string) => {
    setSuccessMessage(message);
    setTimeout(() => setSuccessMessage(null), 3000);
  };

  const showError = (message: string) => {
    setError(message);
    setTimeout(() => setError(null), 5000);
  };

  // Every mutation awaits a real answer from the machine and reports it. These used to be
  // fire-and-forget commands on the WebSocket followed by an unconditional success toast --
  // which was wrong twice over: the save could not work at all (a routine exceeded the
  // socket's then-256-byte inbound frame), and nothing would have said so if it had merely
  // failed. The size limit is gone, but "await a real answer" is the part that mattered.
  //
  // No `setTimeout` refresh either. The application processor pushes fresh summaries as
  // soon as a write lands, so waiting a second was a guess in place of an answer.
  const handleSave = async (routine: Routine) => {
    try {
      if (editingRoutine !== null) {
        await saveRoutine(editingRoutine, routine);
        // Dropped before the new summaries arrive, so nothing can render the pre-edit
        // body in the meantime.
        invalidateRoutineBody(editingRoutine);
        showSuccess('Routine updated successfully');
      } else if (addingType === 'custom') {
        await createRoutine(routine);
        showSuccess('Custom routine added successfully');
      } else {
        const target: RoutineIdentifier = { type: 'function', index: addingFunctionIndex };
        await saveRoutine(target, routine);
        invalidateRoutineBody(target);
        showSuccess('Function routine added successfully');
      }

      setEditingRoutine(null);
      setIsAdding(false);
    } catch (e) {
      showError(`Could not save routine: ${e instanceof Error ? e.message : String(e)}`);
    }
  };

  // The editor's `onSave` is synchronous by signature, and saving is now a round trip.
  // Discarding the promise here is safe because `handleSave` reports both outcomes
  // itself -- a toast on success, an error banner on failure -- and has nothing to hand
  // back to the caller.
  const onSaveRoutine = (routine: Routine) => {
    void handleSave(routine);
  };

  const handleDelete = async (identifier: RoutineIdentifier, routineName: string) => {
    const ok = await confirm({
      title: `Delete “${routineName}”?`,
      body: 'This cannot be undone. Any schedule that runs it will stop working.',
      confirmLabel: 'Delete',
      destructive: true,
    });
    if (!ok) return;

    try {
      await deleteRoutine(identifier);
      invalidateRoutineBody(identifier);
      showSuccess('Routine deleted successfully');
    } catch (e) {
      showError(`Could not delete routine: ${e instanceof Error ? e.message : String(e)}`);
    }
  };

  /**
   * Duplicating needs the *definition*, which the list no longer carries.
   *
   * Normally already prefetched, so this resolves immediately. The `await` matters
   * anyway: copying a routine whose body had not arrived would previously have spread a
   * summary and stored a routine with no steps.
   */
  const handleDuplicate = async (identifier: RoutineIdentifier) => {
    try {
      const routine = await loadRoutineBody(identifier);
      await createRoutine({ ...routine, name: `${routine.name} (copy)` });
      showSuccess('Routine duplicated successfully');
    } catch (e) {
      showError(`Could not duplicate routine: ${e instanceof Error ? e.message : String(e)}`);
    }
  };

  // These two are the fire-and-forget pair, so they say the command was sent rather than
  // that it worked. The routine execution card appearing on the main screen is what says
  // a routine actually started.
  const handleRun = (identifier: RoutineIdentifier) => {
    const ws = getWebSocketService();
    if (!ws) {
      showError('Not connected to the machine');
      return;
    }

    ws.runRoutine(indexFromIdentifier(identifier));
    showSuccess('Start sent');
  };

  const handleOptimizeStorage = () => {
    const ws = getWebSocketService();
    if (!ws) {
      showError('Not connected to the machine');
      return;
    }
    ws.optimizeRoutineStorage();
    showSuccess('Compaction sent');
  };

  // Rendered entirely from the summary. The counts below are the reason a summary carries
  // them rather than deriving them: a card that showed "8 steps" by consulting the
  // definition would have to fetch every routine on the machine just to draw a list.
  const renderRoutineCard = (routine: RoutineSummary, identifier: RoutineIdentifier, allowEdit: boolean, allowDelete: boolean) => {
    // What this machine cannot currently sense. Answered from the summary rather than the
    // definition, which is why prerequisites are carried there: a list that had to fetch
    // every routine to know which ones it could run is exactly what the summary exists to
    // avoid.
    const missing = unmetPrerequisites(routine.prerequisites, machineDefinition, peripheralStatus);
    const runnable = missing.length === 0;

    // For function routines, use the function name from machine definition
    const getRoutineLabel = () => {
      if (identifier.type === 'function' && machineDefinition?.function_routines) {
        const functionName = machineDefinition.function_routines.get(identifier.index);
        return functionName ? `Function: ${functionName}` : `Function #${identifier.index}`;
      }
      return `${getRoutineTypeLabel(identifier.type)} #${identifier.index}`;
    };

    const missingLabel = missing.map(p => capabilityLabel(p.capability)).join(' and ');

    return (
      <div
        key={`${identifier.type}-${identifier.index}`}
        style={{
          padding: tokens.space.md,
          border: `1px solid ${tokens.color.border}`,
          borderRadius: tokens.radius.md,
          backgroundColor: tokens.color.surfaceSunken,
          display: 'flex',
          justifyContent: 'space-between',
          alignItems: 'center',
          gap: tokens.space.md,
          flexWrap: 'wrap',
        }}
      >
        <div style={{ flex: 1, minWidth: '14rem' }}>
          <div style={{ fontWeight: 500, fontSize: '1.1rem', marginBottom: tokens.space.xs }}>
            {routine.name}
          </div>
          <div style={{ fontSize: '0.75rem', color: tokens.color.inkMuted, marginBottom: tokens.space.xs }}>
            {getRoutineLabel()}
          </div>
          <div style={{ fontSize: '0.85rem', color: tokens.color.inkMuted }}>
            {routine.parameter_count} parameter{routine.parameter_count !== 1 ? 's' : ''}
            {routine.derived_parameter_count > 0 && ` + ${routine.derived_parameter_count} derived`}
            {' • '}
            {routine.step_count} step{routine.step_count !== 1 ? 's' : ''}
            {routine.finally_count > 0 && ` • ${routine.finally_count} finally command${routine.finally_count !== 1 ? 's' : ''}`}
          </div>
          {!runnable && (
            // Named, not just disabled. "Needs a scale" tells someone what to go and do;
            // a greyed button with no reason reads as a broken page. It is a badge rather
            // than a coloured sentence so the reason travels with the row.
            <div style={{ marginTop: tokens.space.xs }}>
              <Badge role="warn">Needs {missingLabel}</Badge>
            </div>
          )}
        </div>

        <div style={{ display: 'flex', gap: tokens.space.sm, flexWrap: 'wrap' }}>
          <Button
            variant="primary"
            size="sm"
            onClick={() => void handleRun(identifier)}
            disabled={!runnable}
            // The reason is on the row as a badge, so this repeats it only for a screen
            // reader landing on the disabled button itself.
            ariaLabel={runnable ? `Run ${routine.name}` : `Run ${routine.name} — needs ${missingLabel}`}
          >
            Run
          </Button>
          {allowEdit && (
            <Button
              variant="secondary"
              size="sm"
              onClick={() => setEditingRoutine(identifier)}
              ariaLabel={`Edit ${routine.name}`}
            >
              Edit
            </Button>
          )}
          {allowDelete && (
            <>
              <Button
                variant="secondary"
                size="sm"
                onClick={() => void handleDuplicate(identifier)}
                ariaLabel={`Duplicate ${routine.name}`}
              >
                Duplicate
              </Button>
              <Button
                variant="destructive"
                size="sm"
                onClick={() => void handleDelete(identifier, routine.name)}
                ariaLabel={`Delete ${routine.name}`}
              >
                Delete
              </Button>
            </>
          )}
        </div>
      </div>
    );
  };

  const totalRoutines = (routines.internal?.size ?? 0) +
                        (routines.function?.size ?? 0) +
                        (routines.custom?.size ?? 0);

  const startAdding = () => {
    setAddingType(activeTab === 'function' ? 'function' : 'custom');
    // For function routines, set default index to first available slot
    if (activeTab === 'function' && machineDefinition?.function_routines) {
      const availableIndices = Array.from(machineDefinition.function_routines.keys());
      if (availableIndices.length > 0) {
        setAddingFunctionIndex(Math.min(...availableIndices));
      }
    }
    setIsAdding(true);
  };

  const cardList = (cards: ComponentChildren) => (
    <div style={{ display: 'flex', flexDirection: 'column', gap: tokens.space.md, paddingTop: tokens.space.md }}>
      {cards}
    </div>
  );

  return (
    <div style={{
      backgroundColor: tokens.color.surfaceRaised,
      border: `1px solid ${tokens.color.border}`,
      borderRadius: tokens.radius.md,
      padding: tokens.space.lg,
    }}>
      <div style={{
        display: 'flex',
        justifyContent: 'space-between',
        alignItems: 'center',
        gap: tokens.space.sm,
        marginBottom: tokens.space.lg,
        flexWrap: 'wrap',
      }}>
        <div style={{ display: 'flex', alignItems: 'center', gap: tokens.space.sm }}>
          <h2 style={{ margin: 0 }}>Routines</h2>
          <Badge numeric>{totalRoutines}</Badge>
        </div>
        {activeTab !== 'internal' && (
          <Button variant="primary" onClick={startAdding}>
            Create {activeTab === 'function' ? 'function' : 'custom'} routine
          </Button>
        )}
      </div>

      {error && (
        <div style={{ marginBottom: tokens.space.md }}>
          <Alert role="danger" onDismiss={() => setError(null)}>{error}</Alert>
        </div>
      )}

      {successMessage && (
        <div style={{ marginBottom: tokens.space.md }}>
          <Alert role="ok">{successMessage}</Alert>
        </div>
      )}

      {/* The second of the frontend's two hand-built tab strips, and it had the same
          problems as the routine editor's: three buttons, no roles, no keyboard. It also
          marked the selected tab by colour alone -- blue text against grey. */}
      <Tabs
        label="Routine kinds"
        active={activeTab}
        onChange={setActiveTab}
        tabs={[
          { id: 'custom' as const, label: 'Custom', badge: routines.custom?.size ?? 0 },
          { id: 'function' as const, label: 'Function', badge: routines.function?.size ?? 0 },
          { id: 'internal' as const, label: 'Internal', badge: routines.internal?.size ?? 0 },
        ]}
      >
        {activeTab === 'custom' &&
          ((routines.custom?.size ?? 0) === 0 ? (
            <div style={{ paddingTop: tokens.space.md }}>
              <EmptyState
                title="No custom routines"
                detail="A routine is a sequence of steps the machine runs on its own — a backflush, a preinfusion profile."
                action={{ label: 'Create custom routine', onClick: startAdding }}
              />
            </div>
          ) : (
            cardList(
              Array.from(routines.custom?.entries() ?? [])
                .sort(([a], [b]) => Number(a) - Number(b))
                .map(([index, routine]) =>
                  renderRoutineCard(routine, { type: 'custom', index }, true, true)
                )
            )
          ))}

        {activeTab === 'function' &&
          ((routines.function?.size ?? 0) === 0 ? (
            <div style={{ paddingTop: tokens.space.md }}>
              <EmptyState
                title="No function routines"
                detail={
                  machineDefinition?.function_routines
                    ? `Function routines fill slots the machine defines. ${machineDefinition.function_routines.size} slot${machineDefinition.function_routines.size === 1 ? '' : 's'} available.`
                    : 'This machine defines no function routine slots.'
                }
                action={
                  machineDefinition?.function_routines
                    ? { label: 'Create function routine', onClick: startAdding }
                    : undefined
                }
              />
            </div>
          ) : (
            cardList(
              Array.from(routines.function?.entries() ?? [])
                .sort(([a], [b]) => Number(a) - Number(b))
                .map(([index, routine]) =>
                  renderRoutineCard(routine, { type: 'function', index }, true, false)
                )
            )
          ))}

        {activeTab === 'internal' &&
          ((routines.internal?.size ?? 0) === 0 ? (
            <div style={{ paddingTop: tokens.space.md }}>
              <EmptyState
                title="No internal routines"
                detail="Internal routines are built into the firmware. This build has none."
              />
            </div>
          ) : (
            cardList(
              Array.from(routines.internal?.entries() ?? [])
                .sort(([a], [b]) => Number(a) - Number(b))
                .map(([index, routine]) =>
                  renderRoutineCard(routine, { type: 'internal', index }, false, false)
                )
            )
          ))}
      </Tabs>

      {totalRoutines > 0 && (
        <div style={{
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'space-between',
          gap: tokens.space.sm,
          flexWrap: 'wrap',
          marginTop: tokens.space.lg,
          paddingTop: tokens.space.md,
          borderTop: `1px solid ${tokens.color.border}`,
        }}>
          <span style={{ fontSize: '0.8rem', color: tokens.color.inkMuted, maxWidth: '48ch' }}>
            Deleting routines leaves gaps in the machine's storage. Compacting reclaims
            them; it does not change any routine.
          </span>
          <Button variant="quiet" size="sm" onClick={() => void handleOptimizeStorage()}>
            Compact storage
          </Button>
        </div>
      )}

      {/* Routine Editor */}
      {isAdding && (
        <RoutineEditor
          routine={null}
          onSave={onSaveRoutine}
          onCancel={() => setIsAdding(false)}
          functionSlotConfig={addingType === 'function' ? {
            index: addingFunctionIndex,
            onChange: setAddingFunctionIndex,
            availableSlots: machineDefinition?.function_routines
              ? Object.fromEntries(machineDefinition.function_routines.entries())
              : {}
          } : undefined}
        />
      )}

      {editingRoutine !== null && (
        <RoutineEditorForRoutine
          identifier={editingRoutine}
          onSave={onSaveRoutine}
          onCancel={() => setEditingRoutine(null)}
        />
      )}
    </div>
  );
};

/**
 * Open the editor on a routine, once its definition is actually in hand.
 *
 * **The editor must never be given a stub**, and this component exists to make that
 * impossible rather than unlikely. `RoutineEditor` seeds its state with
 * `routine?.steps || []`, which is right for the create-new case it was written for --
 * and catastrophic for a half-loaded one: the fallbacks cannot tell "new" from "not
 * arrived yet", so saving would replace a real routine with an empty one and there would
 * be no error to see. The definition either exists here or the editor is not rendered.
 *
 * Normally there is nothing to wait for. The background walk fetches every definition
 * shortly after connecting, so this is a cache hit and the editor opens with no request
 * and no spinner. The loading state is for the two cases that outrun the walk: a routine
 * near the end of a long list, and one just invalidated by a save.
 */
interface RoutineEditorForRoutineProps {
  identifier: RoutineIdentifier;
  onSave: (routine: Routine) => void;
  onCancel: () => void;
}

const RoutineEditorForRoutine = ({ identifier, onSave, onCancel }: RoutineEditorForRoutineProps) => {
  const cached = useRoutineBody(identifier);
  const [loadError, setLoadError] = useState<string | null>(null);

  useEffect(() => {
    if (cached) return;

    let cancelled = false;
    setLoadError(null);
    loadRoutineBody(identifier).catch((e) => {
      if (!cancelled) {
        setLoadError(e instanceof Error ? e.message : String(e));
      }
    });

    return () => {
      cancelled = true;
    };
    // The two fields rather than the object: `identifier` is state held by the parent and
    // stable in practice, but depending on the object would tie this effect to the
    // parent's re-render rather than to which routine is being edited.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [identifier.type, identifier.index, cached]);

  if (loadError !== null) {
    return (
      <div style={{ padding: tokens.space.md }}>
        <Alert
          role="danger"
          title="Could not load this routine"
          action={{ label: 'Close', onClick: onCancel }}
        >
          {loadError}
        </Alert>
      </div>
    );
  }

  if (!cached) {
    return (
      <div style={{ padding: tokens.space.md, color: tokens.color.inkMuted }}>Loading routine…</div>
    );
  }

  return <RoutineEditor routine={cached} onSave={onSave} onCancel={onCancel} />;
};

export const RoutineBuilder = memo(RoutineBuilderComponent);
