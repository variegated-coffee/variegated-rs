import { useState } from 'preact/hooks';
import { Routine, RoutineParameter, DerivedParameter, RoutineStep, RoutineCommand } from '../../schemas/schemas';
import { ParametersTab } from './ParametersTab';
import { StepsTab } from './StepsTab';
import { FinallyTab } from './FinallyTab';

interface RoutineEditorProps {
  routine: Routine | null;
  onSave: (routine: Routine) => void | Promise<void>;
  onCancel: () => void;
  functionSlotConfig?: {
    index: number;
    onChange: (index: number) => void;
    availableSlots: { [key: string]: string };
  };
}

type TabType = 'parameters' | 'steps' | 'finally';

export function RoutineEditor({ routine, onSave, onCancel, functionSlotConfig }: RoutineEditorProps) {
  const [activeTab, setActiveTab] = useState<TabType>('parameters');
  const [name, setName] = useState(routine?.name || '');
  const [parameters, setParameters] = useState<RoutineParameter[]>(routine?.parameters || []);
  const [derivedParameters, setDerivedParameters] = useState<DerivedParameter[]>(routine?.derived_parameters || []);
  const [steps, setSteps] = useState<RoutineStep[]>(routine?.steps || []);
  const [finallyCommands, setFinallyCommands] = useState<RoutineCommand[]>(routine?.finally || []);

  const handleSave = () => {
    if (!name.trim()) {
      alert('Routine name is required');
      return;
    }

    if (steps.length === 0) {
      alert('At least one step is required');
      return;
    }

    void onSave({
      routine_type: routine?.routine_type || { type: 'UserDefined' },
      name: name.trim(),
      parameters,
      derived_parameters: derivedParameters,
      steps,
      finally: finallyCommands
    });
  };

  const getTabStyle = (tab: TabType) => ({
    flex: 1,
    padding: '0.75rem 1rem',
    backgroundColor: activeTab === tab ? 'white' : '#e0e0e0',
    border: 'none',
    borderBottom: activeTab === tab ? '3px solid #0066cc' : '3px solid transparent',
    cursor: 'pointer',
    fontSize: '0.95rem',
    fontWeight: activeTab === tab ? '500' : 'normal',
    transition: 'all 0.2s'
  });

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
        maxWidth: '1200px',
        width: '100%',
        maxHeight: '90vh',
        display: 'flex',
        flexDirection: 'column',
        overflow: 'hidden'
      }}>
        {/* Header */}
        <div style={{ padding: '1.5rem', borderBottom: '1px solid #ddd' }}>
          <h2 style={{ margin: 0, marginBottom: '0.75rem' }}>
            {routine ? 'Edit Routine' : 'Create New Routine'}
          </h2>

          {/* Function Slot Selector */}
          {functionSlotConfig && (
            <div style={{ marginBottom: '1rem' }}>
              <label style={{
                display: 'block',
                marginBottom: '0.5rem',
                fontWeight: '500',
                fontSize: '0.9rem',
                color: '#333'
              }}>
                Function Routine Slot
              </label>
              <select
                value={functionSlotConfig.index}
                onChange={(e) => functionSlotConfig.onChange(parseInt(e.currentTarget.value))}
                style={{
                  width: '100%',
                  padding: '0.5rem',
                  border: '1px solid #ccc',
                  borderRadius: '4px',
                  fontSize: '1rem'
                }}
              >
                {Object.entries(functionSlotConfig.availableSlots).map(([index, name]) => (
                  <option key={index} value={index}>
                    Slot {index}: {name}
                  </option>
                ))}
              </select>
              <div style={{
                fontSize: '0.8rem',
                color: '#666',
                marginTop: '0.25rem'
              }}>
                Note: This will overwrite any existing routine in this slot.
              </div>
            </div>
          )}

          <input
            type="text"
            value={name}
            onChange={(e) => setName(e.currentTarget.value)}
            placeholder="Routine name (e.g., Turbo Shot, Lungo)"
            style={{
              width: '100%',
              padding: '0.5rem',
              border: '1px solid #ccc',
              borderRadius: '4px',
              fontSize: '1rem'
            }}
          />
        </div>

        {/* Tabs */}
        <div style={{
          display: 'flex',
          borderBottom: '1px solid #ddd',
          backgroundColor: '#f5f5f5'
        }}>
          <button onClick={() => setActiveTab('parameters')} style={getTabStyle('parameters')}>
            Parameters ({parameters.length + derivedParameters.length})
          </button>
          <button onClick={() => setActiveTab('steps')} style={getTabStyle('steps')}>
            Steps ({steps.length})
          </button>
          <button onClick={() => setActiveTab('finally')} style={getTabStyle('finally')}>
            Finally ({finallyCommands.length})
          </button>
        </div>

        {/* Tab Content */}
        <div style={{ flex: 1, overflow: 'auto', backgroundColor: 'white' }}>
          {activeTab === 'parameters' && (
            <ParametersTab
              parameters={parameters}
              derivedParameters={derivedParameters}
              onParametersChange={setParameters}
              onDerivedParametersChange={setDerivedParameters}
            />
          )}
          {activeTab === 'steps' && (
            <StepsTab
              steps={steps}
              onStepsChange={setSteps}
              parameters={parameters}
              derivedParameters={derivedParameters}
            />
          )}
          {activeTab === 'finally' && (
            <FinallyTab
              finallyCommands={finallyCommands}
              onFinallyCommandsChange={setFinallyCommands}
              parameters={parameters}
              derivedParameters={derivedParameters}
            />
          )}
        </div>

        {/* Footer */}
        <div style={{
          padding: '1rem 1.5rem',
          borderTop: '1px solid #ddd',
          display: 'flex',
          gap: '1rem',
          justifyContent: 'flex-end',
          backgroundColor: '#f9f9f9'
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
            disabled={!name.trim() || steps.length === 0}
            style={{
              padding: '0.75rem 1.5rem',
              backgroundColor: !name.trim() || steps.length === 0 ? '#ccc' : '#0066cc',
              color: 'white',
              border: 'none',
              borderRadius: '4px',
              fontSize: '1rem',
              cursor: !name.trim() || steps.length === 0 ? 'not-allowed' : 'pointer'
            }}
          >
            Save Routine
          </button>
        </div>
      </div>
    </div>
  );
}
