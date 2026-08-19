import { useState } from 'preact/hooks';
import { RoutineParameter, ParameterUnit } from '../../schemas/schemas';

interface ParameterEditorProps {
  parameter: RoutineParameter | null;
  onSave: (parameter: RoutineParameter) => void;
  onCancel: () => void;
  existingIndices: number[];
}

const PARAMETER_UNITS: ParameterUnit[] = [
  { type: 'Seconds' },
  { type: 'Celsius' },
  { type: 'Bar' },
  { type: 'MillilitersPerSecond' },
  { type: 'Grams' },
  { type: 'Percent' },
  { type: 'Milliliters' },
  { type: 'MillisiemensPerCentimeter' },
  { type: 'ExtractionRate' },
  { type: 'ExtractedSolids' }
];

/// Shot attributes a parameter can mirror.
//
// Numeric keys only. A parameter is an f32, so only annotations holding a Number round-trip
// through one -- `Beans` and `GrindSize` are Text on purpose, because grinders number their
// settings incompatibly, and are deliberately absent from this list rather than offered and
// then rejected.
const LINKABLE_ATTRIBUTES = [
  { value: 'DoseWeight', label: 'Dose weight (g)' }
] as const;

export function ParameterEditor({ parameter, onSave, onCancel, existingIndices }: ParameterEditorProps) {
  const [index, setIndex] = useState<number>(
    parameter?.index ?? (existingIndices.length > 0 ? Math.max(...existingIndices) + 1 : 0)
  );
  const [name, setName] = useState(parameter?.name || '');
  const [defaultValue, setDefaultValue] = useState(parameter?.default || 0);
  const [unit, setUnit] = useState<ParameterUnit | null>(parameter?.unit || null);
  const [linkedAttribute, setLinkedAttribute] = useState<RoutineParameter['linked_attribute']>(
    parameter?.linked_attribute ?? null
  );

  const handleSave = () => {
    if (!name.trim()) {
      alert('Parameter name is required');
      return;
    }

    if (!parameter && existingIndices.includes(index)) {
      alert(`Parameter index ${index} is already in use`);
      return;
    }

    onSave({
      index,
      name: name.trim(),
      default: defaultValue,
      unit,
      linked_attribute: linkedAttribute
    });
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
        maxWidth: '500px',
        width: '90%'
      }}>
        <h2 style={{ marginBottom: '1.5rem' }}>
          {parameter ? 'Edit Parameter' : 'New Parameter'}
        </h2>

        <div style={{ marginBottom: '1.5rem' }}>
          <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>
            Index (0-7)
          </label>
          <input
            type="number"
            min="0"
            max="7"
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
            This will be referenced as P{index}
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
            placeholder="e.g. Preinfusion Time, Target Pressure"
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
            Default Value
          </label>
          <input
            type="number"
            step="0.1"
            value={defaultValue}
            onChange={(e) => setDefaultValue(parseFloat(e.currentTarget.value) || 0)}
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
            Linked shot attribute (optional)
          </label>
          <select
            value={linkedAttribute?.type ?? ''}
            onChange={(e) => setLinkedAttribute(
              e.currentTarget.value
                ? { type: e.currentTarget.value } as RoutineParameter['linked_attribute']
                : null
            )}
            style={{
              width: '100%',
              padding: '0.5rem',
              border: '1px solid #ccc',
              borderRadius: '4px',
              fontSize: '1rem'
            }}
          >
            <option value="">None</option>
            {LINKABLE_ATTRIBUTES.map(a => (
              <option key={a.value} value={a.value}>{a.label}</option>
            ))}
          </select>
          <div style={{ fontSize: '0.85rem', color: '#666', marginTop: '0.25rem' }}>
            A linked parameter is seeded from the shot attribute before the routine runs, and
            the value it runs with is recorded against the shot. Only numeric attributes can
            be linked &mdash; beans and grind size are text.
          </div>
        </div>

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
