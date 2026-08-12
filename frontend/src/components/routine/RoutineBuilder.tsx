import { useEffect, useState } from 'preact/hooks';
import { memo } from 'preact/compat';
import { Routine, MachineDefinition, RoutineSummary, RoutineSummaryStorage } from '../../schemas/schemas';
import { getRoutineTypeLabel, indexFromIdentifier, RoutineIdentifier } from '../../utils/routineHelpers';
import { RoutineEditor } from './RoutineEditor';
import { getWebSocketService } from '../../services/websocket';
import { createRoutine, deleteRoutine, saveRoutine } from '../../api/routines';
import { invalidateRoutineBody, loadRoutineBody, useRoutineBody } from '../../state/routineBodies';

interface RoutineBuilderProps {
  routines: RoutineSummaryStorage;
  machineDefinition: MachineDefinition | null;
  onRefresh?: () => void;
}

const RoutineBuilderComponent = ({ routines, machineDefinition, onRefresh }: RoutineBuilderProps) => {
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

  // Every mutation now awaits a real answer from the machine and reports it. These used
  // to be fire-and-forget commands on the WebSocket followed by an unconditional success
  // toast -- which was wrong twice over: the save could not work at all (a routine
  // exceeds the socket's 256-byte inbound frame), and nothing would have said so if it
  // had merely failed.
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
      onRefresh?.();
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
    if (!confirm(`Delete routine "${routineName}"?`)) {
      return;
    }

    try {
      await deleteRoutine(identifier);
      invalidateRoutineBody(identifier);
      showSuccess('Routine deleted successfully');
      onRefresh?.();
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
      onRefresh?.();
    } catch (e) {
      showError(`Could not duplicate routine: ${e instanceof Error ? e.message : String(e)}`);
    }
  };

  const handleRun = (identifier: RoutineIdentifier) => {
    const ws = getWebSocketService();
    if (!ws) {
      showError('WebSocket not connected');
      return;
    }

    ws.runRoutine(indexFromIdentifier(identifier));
    showSuccess('Routine started successfully');
  };

  const handleOptimizeStorage = () => {
    const ws = getWebSocketService();
    if (!ws) {
      showError('WebSocket not connected');
      return;
    }
    ws.optimizeRoutineStorage();
    showSuccess('Storage optimized successfully');
  };

  // Rendered entirely from the summary. The counts below are the reason a summary carries
  // them rather than deriving them: a card that showed "8 steps" by consulting the
  // definition would have to fetch every routine on the machine just to draw a list.
  const renderRoutineCard = (routine: RoutineSummary, identifier: RoutineIdentifier, allowEdit: boolean, allowDelete: boolean) => {
    // For function routines, use the function name from machine definition
    const getRoutineLabel = () => {
      if (identifier.type === 'function' && machineDefinition?.function_routines) {
        const functionName = machineDefinition.function_routines.get(identifier.index);
        return functionName ? `Function: ${functionName}` : `Function #${identifier.index}`;
      }
      return `${getRoutineTypeLabel(identifier.type)} #${identifier.index}`;
    };

    return (
      <div
        key={`${identifier.type}-${identifier.index}`}
        style={{
          padding: '1rem',
          border: '1px solid #ddd',
          borderRadius: '4px',
          backgroundColor: '#fafafa',
          display: 'flex',
          justifyContent: 'space-between',
          alignItems: 'center'
        }}
      >
        <div style={{ flex: 1 }}>
          <div style={{ fontWeight: '500', fontSize: '1.1rem', marginBottom: '0.25rem' }}>
            {routine.name}
          </div>
          <div style={{ fontSize: '0.75rem', color: '#999', marginBottom: '0.25rem' }}>
            {getRoutineLabel()}
          </div>
          <div style={{ fontSize: '0.85rem', color: '#666' }}>
            {routine.parameter_count} parameter{routine.parameter_count !== 1 ? 's' : ''}
            {routine.derived_parameter_count > 0 && ` + ${routine.derived_parameter_count} derived`}
            {' • '}
            {routine.step_count} step{routine.step_count !== 1 ? 's' : ''}
            {routine.finally_count > 0 && ` • ${routine.finally_count} finally command${routine.finally_count !== 1 ? 's' : ''}`}
          </div>
        </div>

        <div style={{ display: 'flex', gap: '0.5rem', marginLeft: '1rem' }}>
          <button
            onClick={() => void handleRun(identifier)}
            style={{
              padding: '0.5rem 1rem',
              backgroundColor: '#28a745',
              color: 'white',
              border: 'none',
              borderRadius: '4px',
              cursor: 'pointer',
              fontSize: '0.9rem',
              fontWeight: '500'
            }}
          >
            Run
          </button>
          {allowEdit && (
            <button
              onClick={() => setEditingRoutine(identifier)}
              style={{
                padding: '0.5rem 1rem',
                backgroundColor: 'white',
                border: '1px solid #ccc',
                borderRadius: '4px',
                cursor: 'pointer',
                fontSize: '0.9rem'
              }}
            >
              Edit
            </button>
          )}
          {allowDelete && (
            <>
              <button
                onClick={() => void handleDuplicate(identifier)}
                style={{
                  padding: '0.5rem 1rem',
                  backgroundColor: 'white',
                  border: '1px solid #ccc',
                  borderRadius: '4px',
                  cursor: 'pointer',
                  fontSize: '0.9rem'
                }}
              >
                Duplicate
              </button>
              <button
                onClick={() => void handleDelete(identifier, routine.name)}
                style={{
                  padding: '0.5rem 1rem',
                  backgroundColor: '#dc3545',
                  color: 'white',
                  border: 'none',
                  borderRadius: '4px',
                  cursor: 'pointer',
                  fontSize: '0.9rem'
                }}
              >
                Delete
              </button>
            </>
          )}
        </div>
      </div>
    );
  };

  const totalRoutines = (routines.internal?.size ?? 0) +
                        (routines.function?.size ?? 0) +
                        (routines.custom?.size ?? 0);

  return (
    <div style={{
      backgroundColor: 'white',
      borderRadius: '8px',
      padding: '1.5rem',
      boxShadow: '0 2px 4px rgba(0,0,0,0.1)'
    }}>
      <div style={{
        display: 'flex',
        justifyContent: 'space-between',
        alignItems: 'center',
        marginBottom: '1.5rem'
      }}>
        <h2 style={{ margin: 0 }}>Routines ({totalRoutines})</h2>
        {activeTab !== 'internal' && (
          <button
            onClick={() => {
              setAddingType(activeTab === 'function' ? 'function' : 'custom');
              // For function routines, set default index to first available slot
              if (activeTab === 'function' && machineDefinition?.function_routines) {
                const availableIndices = Array.from(machineDefinition.function_routines.keys());
                if (availableIndices.length > 0) {
                  setAddingFunctionIndex(Math.min(...availableIndices));
                }
              }
              setIsAdding(true);
            }}
            style={{
              padding: '0.75rem 1.5rem',
              backgroundColor: '#0066cc',
              color: 'white',
              border: 'none',
              borderRadius: '4px',
              fontSize: '1rem',
              cursor: 'pointer'
            }}
          >
            + Create {activeTab === 'function' ? 'Function' : 'Custom'} Routine
          </button>
        )}
      </div>

      {error && (
        <div style={{
          fontSize: '0.9rem',
          color: '#721c24',
          marginBottom: '1rem',
          padding: '0.75rem',
          backgroundColor: '#f8d7da',
          borderRadius: '4px',
          border: '1px solid #f5c6cb'
        }}>
          ❌ {error}
        </div>
      )}

      {successMessage && (
        <div style={{
          fontSize: '0.9rem',
          color: '#155724',
          marginBottom: '1rem',
          padding: '0.75rem',
          backgroundColor: '#d4edda',
          borderRadius: '4px',
          border: '1px solid #c3e6cb'
        }}>
          ✅ {successMessage}
        </div>
      )}

      {/* Tabs */}
      <div style={{ display: 'flex', borderBottom: '2px solid #ddd', marginBottom: '1rem' }}>
        <button
          onClick={() => setActiveTab('custom')}
          style={{
            padding: '0.75rem 1.5rem',
            backgroundColor: 'transparent',
            border: 'none',
            borderBottom: activeTab === 'custom' ? '2px solid #0066cc' : '2px solid transparent',
            color: activeTab === 'custom' ? '#0066cc' : '#666',
            fontWeight: activeTab === 'custom' ? '600' : 'normal',
            cursor: 'pointer',
            fontSize: '1rem',
            marginBottom: '-2px'
          }}
        >
          Custom ({routines.custom?.size ?? 0})
        </button>
        <button
          onClick={() => setActiveTab('function')}
          style={{
            padding: '0.75rem 1.5rem',
            backgroundColor: 'transparent',
            border: 'none',
            borderBottom: activeTab === 'function' ? '2px solid #0066cc' : '2px solid transparent',
            color: activeTab === 'function' ? '#0066cc' : '#666',
            fontWeight: activeTab === 'function' ? '600' : 'normal',
            cursor: 'pointer',
            fontSize: '1rem',
            marginBottom: '-2px'
          }}
        >
          Function ({routines.function?.size ?? 0})
        </button>
        <button
          onClick={() => setActiveTab('internal')}
          style={{
            padding: '0.75rem 1.5rem',
            backgroundColor: 'transparent',
            border: 'none',
            borderBottom: activeTab === 'internal' ? '2px solid #0066cc' : '2px solid transparent',
            color: activeTab === 'internal' ? '#0066cc' : '#666',
            fontWeight: activeTab === 'internal' ? '600' : 'normal',
            cursor: 'pointer',
            fontSize: '1rem',
            marginBottom: '-2px'
          }}
        >
          Internal ({routines.internal?.size ?? 0})
        </button>
      </div>

      {/* Custom Routines Tab */}
      {activeTab === 'custom' && (
        <>
          {(routines.custom?.size ?? 0) === 0 ? (
            <div style={{
              padding: '3rem',
              textAlign: 'center',
              color: '#999',
              border: '2px dashed #ddd',
              borderRadius: '4px'
            }}>
              No custom routines defined. Click "Create Custom Routine" to get started.
            </div>
          ) : (
            <div style={{ display: 'flex', flexDirection: 'column', gap: '1rem' }}>
              {Array.from(routines.custom?.entries() ?? [])
                .sort(([a], [b]) => Number(a) - Number(b))
                .map(([index, routine]) => {
                return renderRoutineCard(routine, { type: 'custom', index }, true, true);
              })}
            </div>
          )}
        </>
      )}

      {/* Function Routines Tab */}
      {activeTab === 'function' && (
        <>
          {(routines.function?.size ?? 0) === 0 && !machineDefinition?.function_routines ? (
            <div style={{
              padding: '3rem',
              textAlign: 'center',
              color: '#999',
              border: '2px dashed #ddd',
              borderRadius: '4px'
            }}>
              No function routines available.
            </div>
          ) : (
            <div style={{ display: 'flex', flexDirection: 'column', gap: '1rem' }}>
              {Array.from(routines.function?.entries() ?? [])
                .sort(([a], [b]) => Number(a) - Number(b))
                .map(([index, routine]) => {
                return renderRoutineCard(routine, { type: 'function', index }, true, false);
              })}
              {(routines.function?.size ?? 0) === 0 && (
                <div style={{
                  padding: '2rem',
                  textAlign: 'center',
                  color: '#666',
                  border: '1px dashed #ccc',
                  borderRadius: '4px',
                  fontSize: '0.9rem'
                }}>
                  Function routines are defined in the machine configuration. {machineDefinition?.function_routines ?
                    `${machineDefinition.function_routines.size} function routine slot(s) available.` :
                    'No function routine slots configured.'}
                </div>
              )}
            </div>
          )}
        </>
      )}

      {/* Internal Routines Tab */}
      {activeTab === 'internal' && (
        <>
          {(routines.internal?.size ?? 0) === 0 ? (
            <div style={{
              padding: '3rem',
              textAlign: 'center',
              color: '#999',
              border: '2px dashed #ddd',
              borderRadius: '4px'
            }}>
              No internal system routines.
            </div>
          ) : (
            <div style={{ display: 'flex', flexDirection: 'column', gap: '1rem' }}>
              {Array.from(routines.internal?.entries() ?? [])
                .sort(([a], [b]) => Number(a) - Number(b))
                .map(([index, routine]) => {
                return renderRoutineCard(routine, { type: 'internal', index }, false, false);
              })}
            </div>
          )}
        </>
      )}

      {/* Storage Optimization - Housekeeping */}
      {totalRoutines > 0 && (
        <div style={{
          marginTop: '1.5rem',
          paddingTop: '1rem',
          borderTop: '1px solid #eee',
          display: 'flex',
          justifyContent: 'flex-end'
        }}>
          <button
            onClick={() => void handleOptimizeStorage()}
            style={{
              padding: '0.5rem 0.75rem',
              fontSize: '0.8rem',
              color: '#666',
              backgroundColor: 'transparent',
              border: '1px solid #ddd',
              borderRadius: '4px',
              cursor: 'pointer',
              display: 'flex',
              alignItems: 'center',
              gap: '0.5rem'
            }}
            onMouseEnter={(e) => { e.currentTarget.style.backgroundColor = '#f5f5f5'; }}
            onMouseLeave={(e) => { e.currentTarget.style.backgroundColor = 'transparent'; }}
          >
            🗜️ Optimize Storage
          </button>
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
      <div style={{ padding: '1rem', color: '#dc3545' }}>
        Could not load this routine: {loadError}
        <button onClick={onCancel} style={{ marginLeft: '1rem' }}>Close</button>
      </div>
    );
  }

  if (!cached) {
    return <div style={{ padding: '1rem', color: '#666' }}>Loading routine…</div>;
  }

  return <RoutineEditor routine={cached} onSave={onSave} onCancel={onCancel} />;
};

export const RoutineBuilder = memo(RoutineBuilderComponent);
