import { useState } from 'preact/hooks';
import { KalmanParameters } from '../schemas/schemas';

interface KalmanFilterEditorProps {
  title: string;
  parameters: KalmanParameters | null;
  onSave: (params: KalmanParameters | null) => void;
  onCancel: () => void;
}

export const KalmanFilterEditor = ({ title, parameters, onSave, onCancel }: KalmanFilterEditorProps) => {
  const defaultParams: KalmanParameters = {
    process_noise: 0.01,
    measurement_noise: 0.1,
    estimated_error: 1.0,
    posterior_estimate: 20.0
  };

  const [enabled, setEnabled] = useState(parameters !== null);
  const [localParams, setLocalParams] = useState<KalmanParameters>(
    parameters ? (JSON.parse(JSON.stringify(parameters)) as KalmanParameters) : defaultParams
  );
  const [editingValues, setEditingValues] = useState<Record<string, string>>({});

  const handleSave = () => {
    onSave(enabled ? localParams : null);
  };

  const updateField = (field: keyof KalmanParameters, value: number) => {
    setLocalParams(prev => ({ ...prev, [field]: value }));
  };

  return (
    <div>
      <div style={{ marginBottom: '1rem' }}>
        <h3 style={{ margin: 0 }}>{title}</h3>
      </div>

      <div style={{ marginBottom: '1.5rem' }}>
        <label style={{ display: 'flex', alignItems: 'center', gap: '0.5rem', cursor: 'pointer' }}>
          <input
            type="checkbox"
            checked={enabled}
            onChange={(e) => setEnabled((e.target as HTMLInputElement).checked)}
            style={{ width: '1.2rem', height: '1.2rem' }}
          />
          <span style={{ fontSize: '0.95rem', fontWeight: '500' }}>Enable Kalman Filtering</span>
        </label>
      </div>

      {enabled && (
        <div
          style={{
            padding: '1rem',
            backgroundColor: '#f8f9fa',
            borderRadius: '6px',
            border: '1px solid #e0e0e0'
          }}
        >
          <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '0.75rem' }}>
            <div>
              <label style={{ display: 'block', fontSize: '0.85rem', color: '#666', marginBottom: '0.25rem' }}>
                Process Noise
              </label>
              <input
                type="number"
                step="0.001"
                value={editingValues['process_noise'] ?? localParams.process_noise.toString()}
                onChange={(e) => {
                  const val = e.currentTarget.value;
                  setEditingValues(prev => ({ ...prev, process_noise: val }));
                }}
                onBlur={(e) => {
                  const parsed = parseFloat(e.currentTarget.value) || 0;
                  updateField('process_noise', parsed);
                  setEditingValues(prev => {
                    const { process_noise: _process_noise, ...rest } = prev;
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
                Model uncertainty
              </div>
            </div>

            <div>
              <label style={{ display: 'block', fontSize: '0.85rem', color: '#666', marginBottom: '0.25rem' }}>
                Measurement Noise
              </label>
              <input
                type="number"
                step="0.01"
                value={editingValues['measurement_noise'] ?? localParams.measurement_noise.toString()}
                onChange={(e) => {
                  const val = e.currentTarget.value;
                  setEditingValues(prev => ({ ...prev, measurement_noise: val }));
                }}
                onBlur={(e) => {
                  const parsed = parseFloat(e.currentTarget.value) || 0;
                  updateField('measurement_noise', parsed);
                  setEditingValues(prev => {
                    const { measurement_noise: _measurement_noise, ...rest } = prev;
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
                Sensor noise level
              </div>
            </div>

            <div>
              <label style={{ display: 'block', fontSize: '0.85rem', color: '#666', marginBottom: '0.25rem' }}>
                Estimated Error
              </label>
              <input
                type="number"
                step="0.1"
                value={editingValues['estimated_error'] ?? localParams.estimated_error.toString()}
                onChange={(e) => {
                  const val = e.currentTarget.value;
                  setEditingValues(prev => ({ ...prev, estimated_error: val }));
                }}
                onBlur={(e) => {
                  const parsed = parseFloat(e.currentTarget.value) || 0;
                  updateField('estimated_error', parsed);
                  setEditingValues(prev => {
                    const { estimated_error: _estimated_error, ...rest } = prev;
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
                Initial error estimate
              </div>
            </div>

            <div>
              <label style={{ display: 'block', fontSize: '0.85rem', color: '#666', marginBottom: '0.25rem' }}>
                Posterior Estimate
              </label>
              <input
                type="number"
                step="0.1"
                value={editingValues['posterior_estimate'] ?? localParams.posterior_estimate.toString()}
                onChange={(e) => {
                  const val = e.currentTarget.value;
                  setEditingValues(prev => ({ ...prev, posterior_estimate: val }));
                }}
                onBlur={(e) => {
                  const parsed = parseFloat(e.currentTarget.value) || 0;
                  updateField('posterior_estimate', parsed);
                  setEditingValues(prev => {
                    const { posterior_estimate: _posterior_estimate, ...rest } = prev;
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
                Initial state estimate
              </div>
            </div>
          </div>
        </div>
      )}

      <div style={{ display: 'flex', gap: '1rem', justifyContent: 'flex-end', marginTop: '1.5rem' }}>
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
