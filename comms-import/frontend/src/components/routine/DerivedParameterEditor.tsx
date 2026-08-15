import { useState } from 'preact/hooks';
import { DerivedParameter, DerivedFormula, ParameterUnit, RoutineParameter } from '../../schemas/schemas';

interface DerivedParameterEditorProps {
  parameter: DerivedParameter | null;
  onSave: (parameter: DerivedParameter) => void;
  onCancel: () => void;
  existingIndices: number[];
  availableParameters: RoutineParameter[];
}

type FormulaType = 'Linear' | 'Sum' | 'Difference' | 'Product';

const PARAMETER_UNITS: ParameterUnit[] = [
  { type: 'Seconds' },
  { type: 'Celsius' },
  { type: 'Bar' },
  { type: 'MillilitersPerSecond' },
  { type: 'Grams' },
  { type: 'Percent' }
];

function getFormulaType(formula: DerivedFormula): FormulaType {
  return formula.type as FormulaType;
}

export function DerivedParameterEditor({
  parameter,
  onSave,
  onCancel,
  existingIndices,
  availableParameters
}: DerivedParameterEditorProps) {
  const [index, setIndex] = useState<number>(
    parameter?.index ?? (existingIndices.length > 0 ? Math.max(...existingIndices) + 1 : 0)
  );
  const [name, setName] = useState(parameter?.name || '');
  const [unit, setUnit] = useState<ParameterUnit | null>(parameter?.unit || null);

  const initialFormula: DerivedFormula = parameter?.formula || {
    type: 'Linear',
    value: { base_param: 0, multiplier: 1, offset: 0 }
  };
  const [formulaType, setFormulaType] = useState<FormulaType>(getFormulaType(initialFormula));

  // Linear formula state
  const [linearBaseParam, setLinearBaseParam] = useState(
    initialFormula.type === 'Linear' ? initialFormula.value.base_param : 0
  );
  const [linearMultiplier, setLinearMultiplier] = useState(
    initialFormula.type === 'Linear' ? initialFormula.value.multiplier : 1
  );
  const [linearOffset, setLinearOffset] = useState(
    initialFormula.type === 'Linear' ? initialFormula.value.offset : 0
  );

  // Sum/Product formula state
  const [listParams, setListParams] = useState<number[]>(
    initialFormula.type === 'Sum' ? initialFormula.value.params :
    initialFormula.type === 'Product' ? initialFormula.value.params : [0]
  );

  // Difference formula state
  const [diffParamA, setDiffParamA] = useState(
    initialFormula.type === 'Difference' ? initialFormula.value.param_a : 0
  );
  const [diffParamB, setDiffParamB] = useState(
    initialFormula.type === 'Difference' ? initialFormula.value.param_b : 0
  );

  const handleSave = () => {
    if (!name.trim()) {
      alert('Parameter name is required');
      return;
    }

    if (!parameter && existingIndices.includes(index)) {
      alert(`Derived parameter index ${index} is already in use`);
      return;
    }

    if (availableParameters.length === 0) {
      alert('You must define at least one regular parameter first');
      return;
    }

    let formula: DerivedFormula;
    switch (formulaType) {
      case 'Linear':
        formula = { type: 'Linear', value: { base_param: linearBaseParam, multiplier: linearMultiplier, offset: linearOffset } };
        break;
      case 'Sum':
        formula = { type: 'Sum', value: { params: listParams } };
        break;
      case 'Difference':
        formula = { type: 'Difference', value: { param_a: diffParamA, param_b: diffParamB } };
        break;
      case 'Product':
        formula = { type: 'Product', value: { params: listParams } };
        break;
    }

    onSave({
      index,
      name: name.trim(),
      unit,
      formula
    });
  };

  const handleFormulaTypeChange = (type: FormulaType) => {
    setFormulaType(type);
    // Reset to sensible defaults
    if (type === 'Linear') {
      setLinearBaseParam(availableParameters.length > 0 ? availableParameters[0].index : 0);
      setLinearMultiplier(1);
      setLinearOffset(0);
    } else if (type === 'Sum' || type === 'Product') {
      setListParams(availableParameters.length > 0 ? [availableParameters[0].index] : [0]);
    } else if (type === 'Difference') {
      setDiffParamA(availableParameters.length > 0 ? availableParameters[0].index : 0);
      setDiffParamB(availableParameters.length > 1 ? availableParameters[1].index : 0);
    }
  };

  const addListParam = () => {
    setListParams([...listParams, availableParameters.length > 0 ? availableParameters[0].index : 0]);
  };

  const removeListParam = (idx: number) => {
    setListParams(listParams.filter((_, i) => i !== idx));
  };

  const updateListParam = (idx: number, value: number) => {
    const newParams = [...listParams];
    newParams[idx] = value;
    setListParams(newParams);
  };

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
      zIndex: 1001
    }}>
      <div style={{
        backgroundColor: 'white',
        borderRadius: '8px',
        padding: '2rem',
        maxWidth: '600px',
        width: '90%',
        maxHeight: '90vh',
        overflow: 'auto'
      }}>
        <h2 style={{ marginBottom: '1.5rem' }}>
          {parameter ? 'Edit Derived Parameter' : 'New Derived Parameter'}
        </h2>

        <div style={{ marginBottom: '1.5rem' }}>
          <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>
            Index (0-15)
          </label>
          <input
            type="number"
            min="0"
            max="15"
            value={index}
            onChange={(e) => setIndex(parseInt(e.currentTarget.value) || 0)}
            disabled={parameter !== null}
            style={{
              width: '100%',
              padding: '0.5rem',
              border: '1px solid #ccc',
              borderRadius: '4px',
              fontSize: '1rem',
              backgroundColor: parameter ? '#f5f5f5' : 'white'
            }}
          />
          <div style={{ fontSize: '0.85rem', color: '#666', marginTop: '0.25rem' }}>
            This will be referenced as D{index}
          </div>
        </div>

        <div style={{ marginBottom: '1.5rem' }}>
          <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>
            Name
          </label>
          <input
            type="text"
            value={name}
            onChange={(e) => setName(e.currentTarget.value)}
            placeholder="e.g. Total Time, Flow Rate Ratio"
            style={{
              width: '100%',
              padding: '0.5rem',
              border: '1px solid #ccc',
              borderRadius: '4px',
              fontSize: '1rem'
            }}
          />
        </div>

        <div style={{ marginBottom: '1.5rem' }}>
          <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>
            Unit (optional)
          </label>
          <select
            value={unit?.type || ''}
            onChange={(e) => setUnit(e.currentTarget.value ? { type: e.currentTarget.value } as ParameterUnit : null)}
            style={{
              width: '100%',
              padding: '0.5rem',
              border: '1px solid #ccc',
              borderRadius: '4px',
              fontSize: '1rem'
            }}
          >
            <option value="">None</option>
            {PARAMETER_UNITS.map(u => (
              <option key={u.type} value={u.type}>{u.type}</option>
            ))}
          </select>
        </div>

        <div style={{ marginBottom: '1.5rem' }}>
          <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>
            Formula Type
          </label>
          <select
            value={formulaType}
            onChange={(e) => handleFormulaTypeChange(e.currentTarget.value as FormulaType)}
            style={{
              width: '100%',
              padding: '0.5rem',
              border: '1px solid #ccc',
              borderRadius: '4px',
              fontSize: '1rem'
            }}
          >
            <option value="Linear">Linear (a * P + b)</option>
            <option value="Sum">Sum (P1 + P2 + ...)</option>
            <option value="Difference">Difference (P1 - P2)</option>
            <option value="Product">Product (P1 * P2 * ...)</option>
          </select>
        </div>

        {/* Linear Formula */}
        {formulaType === 'Linear' && (
          <div style={{ padding: '1rem', backgroundColor: '#f5f5f5', borderRadius: '4px', marginBottom: '1.5rem' }}>
            <div style={{ marginBottom: '1rem' }}>
              <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>
                Base Parameter
              </label>
              <select
                value={linearBaseParam}
                onChange={(e) => setLinearBaseParam(parseInt(e.currentTarget.value))}
                style={{
                  width: '100%',
                  padding: '0.5rem',
                  border: '1px solid #ccc',
                  borderRadius: '4px'
                }}
              >
                {availableParameters.map(p => (
                  <option key={p.index} value={p.index}>P{p.index}: {p.name}</option>
                ))}
              </select>
            </div>
            <div style={{ marginBottom: '1rem' }}>
              <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>
                Multiplier
              </label>
              <input
                type="number"
                step="0.1"
                value={linearMultiplier}
                onChange={(e) => setLinearMultiplier(parseFloat(e.currentTarget.value) || 0)}
                style={{
                  width: '100%',
                  padding: '0.5rem',
                  border: '1px solid #ccc',
                  borderRadius: '4px'
                }}
              />
            </div>
            <div>
              <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>
                Offset
              </label>
              <input
                type="number"
                step="0.1"
                value={linearOffset}
                onChange={(e) => setLinearOffset(parseFloat(e.currentTarget.value) || 0)}
                style={{
                  width: '100%',
                  padding: '0.5rem',
                  border: '1px solid #ccc',
                  borderRadius: '4px'
                }}
              />
            </div>
            <div style={{ marginTop: '0.5rem', fontSize: '0.9rem', color: '#666' }}>
              Formula: D{index} = {linearMultiplier} * P{linearBaseParam} + {linearOffset}
            </div>
          </div>
        )}

        {/* Sum/Product Formula */}
        {(formulaType === 'Sum' || formulaType === 'Product') && (
          <div style={{ padding: '1rem', backgroundColor: '#f5f5f5', borderRadius: '4px', marginBottom: '1.5rem' }}>
            <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>
              Parameters
            </label>
            {listParams.map((paramIdx, idx) => (
              <div key={idx} style={{ display: 'flex', gap: '0.5rem', marginBottom: '0.5rem' }}>
                <select
                  value={paramIdx}
                  onChange={(e) => updateListParam(idx, parseInt(e.currentTarget.value))}
                  style={{
                    flex: 1,
                    padding: '0.5rem',
                    border: '1px solid #ccc',
                    borderRadius: '4px'
                  }}
                >
                  {availableParameters.map(p => (
                    <option key={p.index} value={p.index}>P{p.index}: {p.name}</option>
                  ))}
                </select>
                <button
                  onClick={() => removeListParam(idx)}
                  disabled={listParams.length === 1}
                  style={{
                    padding: '0.5rem 0.75rem',
                    backgroundColor: '#dc3545',
                    color: 'white',
                    border: 'none',
                    borderRadius: '4px',
                    cursor: listParams.length === 1 ? 'not-allowed' : 'pointer',
                    opacity: listParams.length === 1 ? 0.5 : 1
                  }}
                >
                  ✕
                </button>
              </div>
            ))}
            <button
              onClick={addListParam}
              style={{
                padding: '0.5rem 1rem',
                backgroundColor: '#28a745',
                color: 'white',
                border: 'none',
                borderRadius: '4px',
                cursor: 'pointer',
                fontSize: '0.9rem',
                marginTop: '0.5rem'
              }}
            >
              + Add Parameter
            </button>
            <div style={{ marginTop: '0.5rem', fontSize: '0.9rem', color: '#666' }}>
              Formula: D{index} = {listParams.map(p => `P${p}`).join(formulaType === 'Sum' ? ' + ' : ' × ')}
            </div>
          </div>
        )}

        {/* Difference Formula */}
        {formulaType === 'Difference' && (
          <div style={{ padding: '1rem', backgroundColor: '#f5f5f5', borderRadius: '4px', marginBottom: '1.5rem' }}>
            <div style={{ marginBottom: '1rem' }}>
              <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>
                Parameter A
              </label>
              <select
                value={diffParamA}
                onChange={(e) => setDiffParamA(parseInt(e.currentTarget.value))}
                style={{
                  width: '100%',
                  padding: '0.5rem',
                  border: '1px solid #ccc',
                  borderRadius: '4px'
                }}
              >
                {availableParameters.map(p => (
                  <option key={p.index} value={p.index}>P{p.index}: {p.name}</option>
                ))}
              </select>
            </div>
            <div>
              <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>
                Parameter B
              </label>
              <select
                value={diffParamB}
                onChange={(e) => setDiffParamB(parseInt(e.currentTarget.value))}
                style={{
                  width: '100%',
                  padding: '0.5rem',
                  border: '1px solid #ccc',
                  borderRadius: '4px'
                }}
              >
                {availableParameters.map(p => (
                  <option key={p.index} value={p.index}>P{p.index}: {p.name}</option>
                ))}
              </select>
            </div>
            <div style={{ marginTop: '0.5rem', fontSize: '0.9rem', color: '#666' }}>
              Formula: D{index} = P{diffParamA} - P{diffParamB}
            </div>
          </div>
        )}

        <div style={{ display: 'flex', gap: '1rem' }}>
          <button
            onClick={handleSave}
            style={{
              flex: 1,
              padding: '0.75rem',
              backgroundColor: '#0066cc',
              color: 'white',
              border: 'none',
              borderRadius: '4px',
              fontSize: '1rem',
              cursor: 'pointer'
            }}
          >
            Save
          </button>
          <button
            onClick={onCancel}
            style={{
              flex: 1,
              padding: '0.75rem',
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
        </div>
      </div>
    </div>
  );
}
