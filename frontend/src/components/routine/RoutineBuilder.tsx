import { useState } from 'preact/hooks';
import { memo } from 'preact/compat';
import { Routine, MachineDefinition, RoutineStorage, RoutineIndex } from '../../schemas/schemas';
import { getRoutineTypeLabel, RoutineIdentifier } from '../../utils/routineHelpers';
import { RoutineEditor } from './RoutineEditor';
import { getWebSocketService } from '../../services/websocket';

interface RoutineBuilderProps {
  routines: RoutineStorage;
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

  const handleSave = (routine: Routine) => {
    const ws = getWebSocketService();
    if (!ws) {
      showError('WebSocket not connected');
      return;
    }

    if (editingRoutine !== null) {
      // Update existing routine
      const routineIndex: RoutineIndex = editingRoutine.type === 'custom'
        ? { type: 'Custom', value: editingRoutine.index }
        : editingRoutine.type === 'function'
        ? { type: 'Function', value: editingRoutine.index }
        : { type: 'Internal', value: editingRoutine.index };
      ws.updateRoutine(routineIndex, routine);
      showSuccess('Routine updated successfully');
    } else {
      // Add new routine
      if (addingType === 'custom') {
        ws.addRoutine(routine);
        showSuccess('Custom routine added successfully');
      } else {
        // Function routine - update at specific index
        const routineIndex: RoutineIndex = { type: 'Function', value: addingFunctionIndex };
        ws.updateRoutine(routineIndex, routine);
        showSuccess('Function routine added successfully');
      }
    }

    setEditingRoutine(null);
    setIsAdding(false);

    // Trigger refresh after a short delay to allow backend to process
    setTimeout(() => {
      onRefresh?.();
    }, 1000);
  };

  const handleDelete = (identifier: RoutineIdentifier, routineName: string) => {
    if (!confirm(`Delete routine "${routineName}"?`)) {
      return;
    }

    const ws = getWebSocketService();
    if (!ws) {
      showError('WebSocket not connected');
      return;
    }

    const routineIndex: RoutineIndex = identifier.type === 'custom'
      ? { type: 'Custom', value: identifier.index }
      : identifier.type === 'function'
      ? { type: 'Function', value: identifier.index }
      : { type: 'Internal', value: identifier.index };
    ws.removeRoutine(routineIndex);
    showSuccess('Routine deleted successfully');

    // Trigger refresh after a short delay
    setTimeout(() => {
      onRefresh?.();
    }, 1000);
  };

  const handleDuplicate = (routineProp: Routine) => {
    const ws = getWebSocketService();
    if (!ws) {
      showError('WebSocket not connected');
      return;
    }

    // Type assertion needed: Postcard's InferType fails on Routine type
    const routine = routineProp;
    const newRoutine: Routine = {
      ...routine,
      name: `${routine.name} (copy)`
    } as Routine;

    ws.addRoutine(newRoutine);
    showSuccess('Routine duplicated successfully');

    // Trigger refresh after a short delay
    setTimeout(() => {
      onRefresh?.();
    }, 1000);
  };

  const handleRun = (identifier: RoutineIdentifier) => {
    const ws = getWebSocketService();
    if (!ws) {
      showError('WebSocket not connected');
      return;
    }

    const routineIndex: RoutineIndex = identifier.type === 'custom'
      ? { type: 'Custom', value: identifier.index }
      : identifier.type === 'function'
      ? { type: 'Function', value: identifier.index }
      : { type: 'Internal', value: identifier.index };
    ws.runRoutine(routineIndex);
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

  const renderRoutineCard = (routineProp: Routine, identifier: RoutineIdentifier, allowEdit: boolean, allowDelete: boolean) => {
    // Type assertion needed: Postcard's InferType fails on Routine type
    const routine = routineProp;

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
            {routine.parameters.length} parameter{routine.parameters.length !== 1 ? 's' : ''}
            {routine.derived_parameters.length > 0 && ` + ${routine.derived_parameters.length} derived`}
            {' • '}
            {routine.steps.length} step{routine.steps.length !== 1 ? 's' : ''}
            {routine.finally.length > 0 && ` • ${routine.finally.length} finally command${routine.finally.length !== 1 ? 's' : ''}`}
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
                onClick={() => void handleDuplicate(routine)}
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
          onSave={handleSave}
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

      {editingRoutine !== null && (() => {
        let routine: Routine | undefined;
        if (editingRoutine.type === 'custom') {
          routine = routines.custom?.get(editingRoutine.index);
        } else if (editingRoutine.type === 'function') {
          routine = routines.function?.get(editingRoutine.index);
        } else if (editingRoutine.type === 'internal') {
          routine = routines.internal?.get(editingRoutine.index);
        }

        return routine ? (
          <RoutineEditor
            routine={routine}
            onSave={handleSave}
            onCancel={() => setEditingRoutine(null)}
          />
        ) : null;
      })()}
    </div>
  );
};

export const RoutineBuilder = memo(RoutineBuilderComponent);
