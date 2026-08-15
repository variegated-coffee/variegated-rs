import { useState } from 'preact/hooks';
import { ControlCurve } from '../schemas/schemas';

interface ControlCurveEditorProps {
  title: string;
  curve: ControlCurve;
  unit: string;
  onSave: (curve: ControlCurve) => void;
  onCancel: () => void;
}

export const ControlCurveEditor = ({ title, curve, unit, onSave, onCancel }: ControlCurveEditorProps) => {
  const [localCurve, setLocalCurve] = useState<ControlCurve>(JSON.parse(JSON.stringify(curve)) as ControlCurve);
  const [editingValues, setEditingValues] = useState<Record<string, string>>({});

  const handleSave = () => {
    onSave(localCurve);
  };

  const updateField = (field: keyof ControlCurve, value: number) => {
    setLocalCurve(prev => ({ ...prev, [field]: value }));
  };

  // Simple curve preview - evaluate at t=0, 10, 20, 30, 40, 50 seconds
  const generatePreviewPoints = () => {
    const points = [];
    for (let t = 0; t <= 50; t += 5) {
      const y = localCurve.a + localCurve.b * t + localCurve.c * t * t;
      const clamped = Math.max(localCurve.min, Math.min(localCurve.max, y));
      points.push({ t, y: clamped });
    }
    return points;
  };

  const previewPoints = generatePreviewPoints();
  const maxY = Math.max(...previewPoints.map(p => p.y));
  const minY = Math.min(...previewPoints.map(p => p.y));
  const rangeY = maxY - minY || 1;

  return (
    <div>
      <div style={{ marginBottom: '1rem' }}>
        <h3 style={{ margin: 0 }}>{title}</h3>
      </div>

      <div
        style={{
          padding: '1rem',
          backgroundColor: '#f8f9fa',
          borderRadius: '6px',
          border: '1px solid #e0e0e0',
          marginBottom: '1rem'
        }}
      >
        <div style={{ fontSize: '0.9rem', color: '#666', marginBottom: '1rem' }}>
          Curve formula: <strong>y = a + b·t + c·t²</strong>
          <div style={{ fontSize: '0.8rem', marginTop: '0.25rem' }}>
            where <em>t</em> is time in seconds from brew start
          </div>
        </div>

        <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: '0.75rem' }}>
          <div>
            <label style={{ display: 'block', fontSize: '0.85rem', color: '#666', marginBottom: '0.25rem' }}>
              Coefficient a
            </label>
            <input
              type="number"
              step="0.1"
              value={editingValues['a'] ?? localCurve.a.toString()}
              onChange={(e) => {
                const val = e.currentTarget.value;
                setEditingValues(prev => ({ ...prev, a: val }));
              }}
              onBlur={(e) => {
                const parsed = parseFloat(e.currentTarget.value) || 0;
                updateField('a', parsed);
                setEditingValues(prev => {
                  const { a: _a, ...rest } = prev;
                  return rest;
                });
              }}
              style={{
                width: '100%',
                padding: '0.5rem',
                border: '1px solid #ccc',
                borderRadius: '4px',
                fontSize: '0.9rem'
              }}
            />
            <div style={{ fontSize: '0.75rem', color: '#999', marginTop: '0.25rem' }}>
              Constant term
            </div>
          </div>

          <div>
            <label style={{ display: 'block', fontSize: '0.85rem', color: '#666', marginBottom: '0.25rem' }}>
              Coefficient b
            </label>
            <input
              type="number"
              step="0.01"
              value={editingValues['b'] ?? localCurve.b.toString()}
              onChange={(e) => {
                const val = e.currentTarget.value;
                setEditingValues(prev => ({ ...prev, b: val }));
              }}
              onBlur={(e) => {
                const parsed = parseFloat(e.currentTarget.value) || 0;
                updateField('b', parsed);
                setEditingValues(prev => {
                  const { b: _b, ...rest } = prev;
                  return rest;
                });
              }}
              style={{
                width: '100%',
                padding: '0.5rem',
                border: '1px solid #ccc',
                borderRadius: '4px',
                fontSize: '0.9rem'
              }}
            />
            <div style={{ fontSize: '0.75rem', color: '#999', marginTop: '0.25rem' }}>
              Linear term
            </div>
          </div>

          <div>
            <label style={{ display: 'block', fontSize: '0.85rem', color: '#666', marginBottom: '0.25rem' }}>
              Coefficient c
            </label>
            <input
              type="number"
              step="0.001"
              value={editingValues['c'] ?? localCurve.c.toString()}
              onChange={(e) => {
                const val = e.currentTarget.value;
                setEditingValues(prev => ({ ...prev, c: val }));
              }}
              onBlur={(e) => {
                const parsed = parseFloat(e.currentTarget.value) || 0;
                updateField('c', parsed);
                setEditingValues(prev => {
                  const { c: _c, ...rest } = prev;
                  return rest;
                });
              }}
              style={{
                width: '100%',
                padding: '0.5rem',
                border: '1px solid #ccc',
                borderRadius: '4px',
                fontSize: '0.9rem'
              }}
            />
            <div style={{ fontSize: '0.75rem', color: '#999', marginTop: '0.25rem' }}>
              Quadratic term
            </div>
          </div>
        </div>

        <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '0.75rem', marginTop: '0.75rem' }}>
          <div>
            <label style={{ display: 'block', fontSize: '0.85rem', color: '#666', marginBottom: '0.25rem' }}>
              Minimum Value ({unit})
            </label>
            <input
              type="number"
              step="0.1"
              value={editingValues['min'] ?? localCurve.min.toString()}
              onChange={(e) => {
                const val = e.currentTarget.value;
                setEditingValues(prev => ({ ...prev, min: val }));
              }}
              onBlur={(e) => {
                const parsed = parseFloat(e.currentTarget.value) || 0;
                updateField('min', parsed);
                setEditingValues(prev => {
                  const { min: _min, ...rest } = prev;
                  return rest;
                });
              }}
              style={{
                width: '100%',
                padding: '0.5rem',
                border: '1px solid #ccc',
                borderRadius: '4px',
                fontSize: '0.9rem'
              }}
            />
            <div style={{ fontSize: '0.75rem', color: '#999', marginTop: '0.25rem' }}>
              Lower clamp
            </div>
          </div>

          <div>
            <label style={{ display: 'block', fontSize: '0.85rem', color: '#666', marginBottom: '0.25rem' }}>
              Maximum Value ({unit})
            </label>
            <input
              type="number"
              step="0.1"
              value={editingValues['max'] ?? localCurve.max.toString()}
              onChange={(e) => {
                const val = e.currentTarget.value;
                setEditingValues(prev => ({ ...prev, max: val }));
              }}
              onBlur={(e) => {
                const parsed = parseFloat(e.currentTarget.value) || 0;
                updateField('max', parsed);
                setEditingValues(prev => {
                  const { max: _max, ...rest } = prev;
                  return rest;
                });
              }}
              style={{
                width: '100%',
                padding: '0.5rem',
                border: '1px solid #ccc',
                borderRadius: '4px',
                fontSize: '0.9rem'
              }}
            />
            <div style={{ fontSize: '0.75rem', color: '#999', marginTop: '0.25rem' }}>
              Upper clamp
            </div>
          </div>
        </div>
      </div>

      {/* Simple Preview */}
      <div
        style={{
          padding: '1rem',
          backgroundColor: 'white',
          border: '1px solid #ddd',
          borderRadius: '6px',
          marginBottom: '1rem'
        }}
      >
        <div style={{ fontSize: '0.9rem', fontWeight: '500', marginBottom: '0.75rem' }}>Curve Preview</div>
        <div style={{ position: 'relative', height: '200px', backgroundColor: '#f8f9fa', borderRadius: '4px', padding: '1rem' }}>
          <svg width="100%" height="100%" style={{ display: 'block' }}>
            {/* Y axis labels */}
            <text x="5" y="15" fontSize="10" fill="#666">{maxY.toFixed(1)} {unit}</text>
            <text x="5" y="185" fontSize="10" fill="#666">{minY.toFixed(1)} {unit}</text>

            {/* X axis labels */}
            <text x="5" y="195" fontSize="10" fill="#666">0s</text>
            <text x="95%" y="195" fontSize="10" fill="#666" textAnchor="end">50s</text>

            {/* Plot points */}
            {previewPoints.map((point, idx) => {
              const x = (point.t / 50) * 100;
              const y = 100 - ((point.y - minY) / rangeY) * 80;

              if (idx === 0) return null;

              const prevPoint = previewPoints[idx - 1];
              const prevX = (prevPoint.t / 50) * 100;
              const prevY = 100 - ((prevPoint.y - minY) / rangeY) * 80;

              return (
                <line
                  key={idx}
                  x1={`${prevX}%`}
                  y1={`${prevY}%`}
                  x2={`${x}%`}
                  y2={`${y}%`}
                  stroke="#0066cc"
                  strokeWidth="2"
                />
              );
            })}

            {/* Plot points as circles */}
            {previewPoints.map((point, idx) => {
              const x = (point.t / 50) * 100;
              const y = 100 - ((point.y - minY) / rangeY) * 80;

              return (
                <circle
                  key={idx}
                  cx={`${x}%`}
                  cy={`${y}%`}
                  r="3"
                  fill="#0066cc"
                />
              );
            })}
          </svg>
        </div>
      </div>

      <div style={{ display: 'flex', gap: '1rem', justifyContent: 'flex-end' }}>
        <button
          onClick={onCancel}
          style={{
            padding: '0.5rem 1.5rem',
            backgroundColor: 'white',
            border: '1px solid #ccc',
            borderRadius: '4px',
            cursor: 'pointer',
            fontSize: '0.9rem'
          }}
        >
          Cancel
        </button>
        <button
          onClick={handleSave}
          style={{
            padding: '0.5rem 1.5rem',
            backgroundColor: '#0066cc',
            color: 'white',
            border: 'none',
            borderRadius: '4px',
            cursor: 'pointer',
            fontSize: '0.9rem',
            fontWeight: '500'
          }}
        >
          Save Changes
        </button>
      </div>
    </div>
  );
};
