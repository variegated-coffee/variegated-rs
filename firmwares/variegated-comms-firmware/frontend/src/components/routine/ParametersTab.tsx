import { useState } from 'preact/hooks';
import { RoutineParameter, DerivedParameter } from '../../schemas/schemas';
import { ParameterEditor } from './ParameterEditor';
import { DerivedParameterEditor } from './DerivedParameterEditor';

interface ParametersTabProps {
  parameters: RoutineParameter[];
  derivedParameters: DerivedParameter[];
  onParametersChange: (parameters: RoutineParameter[]) => void;
  onDerivedParametersChange: (derivedParameters: DerivedParameter[]) => void;
}

function getDerivedFormulaDescription(param: DerivedParameter): string {
  const formula = param.formula;
  if (formula.type === 'Linear') {
    return `${formula.value.multiplier} × P${formula.value.base_param} + ${formula.value.offset}`;
  }
  if (formula.type === 'Sum') {
    return formula.value.params.map((p: number) => `P${p}`).join(' + ');
  }
  if (formula.type === 'Difference') {
    return `P${formula.value.param_a} - P${formula.value.param_b}`;
  }
  if (formula.type === 'Product') {
    return formula.value.params.map((p: number) => `P${p}`).join(' × ');
  }
  return '';
}

export function ParametersTab({
  parameters,
  derivedParameters,
  onParametersChange,
  onDerivedParametersChange
}: ParametersTabProps) {
  const [editingParam, setEditingParam] = useState<RoutineParameter | null>(null);
  const [isAddingParam, setIsAddingParam] = useState(false);
  const [editingDerived, setEditingDerived] = useState<DerivedParameter | null>(null);
  const [isAddingDerived, setIsAddingDerived] = useState(false);

  const handleSaveParameter = (param: RoutineParameter) => {
    const existingIndex = parameters.findIndex(p => p.index === param.index);
    if (existingIndex >= 0) {
      const newParams = [...parameters];
      newParams[existingIndex] = param;
      onParametersChange(newParams);
    } else {
      onParametersChange([...parameters, param]);
    }
    setEditingParam(null);
    setIsAddingParam(false);
  };

  const handleDeleteParameter = (index: number) => {
    if (confirm('Delete this parameter?')) {
      onParametersChange(parameters.filter(p => p.index !== index));
    }
  };

  const handleSaveDerivedParameter = (param: DerivedParameter) => {
    const existingIndex = derivedParameters.findIndex(p => p.index === param.index);
    if (existingIndex >= 0) {
      const newParams = [...derivedParameters];
      newParams[existingIndex] = param;
      onDerivedParametersChange(newParams);
    } else {
      onDerivedParametersChange([...derivedParameters, param]);
    }
    setEditingDerived(null);
    setIsAddingDerived(false);
  };

  const handleDeleteDerivedParameter = (index: number) => {
    if (confirm('Delete this derived parameter?')) {
      onDerivedParametersChange(derivedParameters.filter(p => p.index !== index));
    }
  };

  return (
    <div style={{ padding: '1.5rem', display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '2rem' }}>
      {/* Regular Parameters */}
      <div>
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '1rem' }}>
          <h3 style={{ margin: 0 }}>Parameters ({parameters.length}/8)</h3>
          <button
            onClick={() => setIsAddingParam(true)}
            disabled={parameters.length >= 8}
            style={{
              padding: '0.5rem 1rem',
              backgroundColor: parameters.length >= 8 ? '#ccc' : '#0066cc',
              color: 'white',
              border: 'none',
              borderRadius: '4px',
              cursor: parameters.length >= 8 ? 'not-allowed' : 'pointer',
              fontSize: '0.9rem'
            }}
          >
            + Add Parameter
          </button>
        </div>

        {parameters.length === 0 ? (
          <div style={{
            padding: '2rem',
            textAlign: 'center',
            color: '#666',
            border: '2px dashed #ccc',
            borderRadius: '4px'
          }}>
            No parameters defined
          </div>
        ) : (
          <div style={{ display: 'flex', flexDirection: 'column', gap: '0.5rem' }}>
            {parameters.map(param => (
              <div
                key={param.index}
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
                  <div style={{ fontWeight: '500', marginBottom: '0.25rem' }}>
                    P{param.index}: {param.name}
                  </div>
                  <div style={{ fontSize: '0.85rem', color: '#666' }}>
                    Default: {param.default}{param.unit ? ` ${param.unit.type}` : ''}
                  </div>
                </div>
                <div style={{ display: 'flex', gap: '0.5rem' }}>
                  <button
                    onClick={() => setEditingParam(param)}
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
                    onClick={() => handleDeleteParameter(param.index)}
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

      {/* Derived Parameters */}
      <div>
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '1rem' }}>
          <h3 style={{ margin: 0 }}>Derived Parameters ({derivedParameters.length}/16)</h3>
          <button
            onClick={() => setIsAddingDerived(true)}
            disabled={derivedParameters.length >= 16 || parameters.length === 0}
            style={{
              padding: '0.5rem 1rem',
              backgroundColor: derivedParameters.length >= 16 || parameters.length === 0 ? '#ccc' : '#28a745',
              color: 'white',
              border: 'none',
              borderRadius: '4px',
              cursor: derivedParameters.length >= 16 || parameters.length === 0 ? 'not-allowed' : 'pointer',
              fontSize: '0.9rem'
            }}
            title={parameters.length === 0 ? 'Add regular parameters first' : ''}
          >
            + Add Derived
          </button>
        </div>

        {derivedParameters.length === 0 ? (
          <div style={{
            padding: '2rem',
            textAlign: 'center',
            color: '#666',
            border: '2px dashed #ccc',
            borderRadius: '4px'
          }}>
            No derived parameters defined
          </div>
        ) : (
          <div style={{ display: 'flex', flexDirection: 'column', gap: '0.5rem' }}>
            {derivedParameters.map(param => (
              <div
                key={param.index}
                style={{
                  padding: '0.75rem',
                  backgroundColor: '#e8f5e9',
                  borderRadius: '4px',
                  display: 'flex',
                  justifyContent: 'space-between',
                  alignItems: 'center'
                }}
              >
                <div>
                  <div style={{ fontWeight: '500', marginBottom: '0.25rem' }}>
                    D{param.index}: {param.name}
                  </div>
                  <div style={{ fontSize: '0.85rem', color: '#666' }}>
                    {getDerivedFormulaDescription(param)}
                    {param.unit && ` (${param.unit.type})`}
                  </div>
                </div>
                <div style={{ display: 'flex', gap: '0.5rem' }}>
                  <button
                    onClick={() => setEditingDerived(param)}
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
                    onClick={() => handleDeleteDerivedParameter(param.index)}
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

      {/* Editors */}
      {(isAddingParam || editingParam) && (
        <ParameterEditor
          parameter={editingParam}
          onSave={handleSaveParameter}
          onCancel={() => {
            setEditingParam(null);
            setIsAddingParam(false);
          }}
          existingIndices={parameters.map(p => p.index)}
        />
      )}

      {(isAddingDerived || editingDerived) && (
        <DerivedParameterEditor
          parameter={editingDerived}
          onSave={handleSaveDerivedParameter}
          onCancel={() => {
            setEditingDerived(null);
            setIsAddingDerived(false);
          }}
          existingIndices={derivedParameters.map(p => p.index)}
          availableParameters={parameters}
        />
      )}
    </div>
  );
}
