import { useState } from 'preact/hooks';
import { PumpConfiguration } from '../schemas/schemas';

interface PumpConfigurationEditorProps {
  title: string;
  configuration: PumpConfiguration | null;
  onSave: (config: PumpConfiguration | null) => void;
  onCancel: () => void;
}

export const PumpConfigurationEditor = ({ title, configuration, onSave, onCancel }: PumpConfigurationEditorProps) => {
  const defaultConfig: PumpConfiguration = {
    min_duty_cycle: 10,
    max_duty_cycle: 100,
    ramp_up_time_ms: 500,
    ramp_down_time_ms: 500,
    tacho_pulses_per_liter: null
  };

  const [enabled, setEnabled] = useState(configuration !== null);
  const [localConfig, setLocalConfig] = useState<PumpConfiguration>(
    configuration ? (JSON.parse(JSON.stringify(configuration)) as PumpConfiguration) : defaultConfig
  );
  const [tachoEnabled, setTachoEnabled] = useState(
    configuration?.tacho_pulses_per_liter !== null && configuration?.tacho_pulses_per_liter !== undefined
  );
  const [editingValues, setEditingValues] = useState<Record<string, string>>({});

  const handleSave = () => {
    const configToSave = enabled ? {
      ...localConfig,
      tacho_pulses_per_liter: tachoEnabled ? localConfig.tacho_pulses_per_liter : null
    } : null;
    onSave(configToSave);
  };

  const updateField = (field: keyof PumpConfiguration, value: number | null) => {
    setLocalConfig(prev => ({ ...prev, [field]: value }));
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
          <span style={{ fontSize: '0.95rem', fontWeight: '500' }}>Configure Pump Parameters</span>
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
                Min Duty Cycle (%)
              </label>
              <input
                type="number"
                min="0"
                max="100"
                step="1"
                value={editingValues['min_duty_cycle'] ?? (localConfig.min_duty_cycle?.toString() ?? '')}
                onChange={(e) => {
                  const val = e.currentTarget.value;
                  setEditingValues(prev => ({ ...prev, min_duty_cycle: val }));
                }}
                onBlur={(e) => {
                  const parsed = parseFloat(e.currentTarget.value) || null;
                  updateField('min_duty_cycle', parsed);
                  setEditingValues(prev => {
                    const { min_duty_cycle: _min_duty_cycle, ...rest } = prev;
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
                Minimum pump speed
              </div>
            </div>

            <div>
              <label style={{ display: 'block', fontSize: '0.85rem', color: '#666', marginBottom: '0.25rem' }}>
                Max Duty Cycle (%)
              </label>
              <input
                type="number"
                min="0"
                max="100"
                step="1"
                value={editingValues['max_duty_cycle'] ?? (localConfig.max_duty_cycle?.toString() ?? '')}
                onChange={(e) => {
                  const val = e.currentTarget.value;
                  setEditingValues(prev => ({ ...prev, max_duty_cycle: val }));
                }}
                onBlur={(e) => {
                  const parsed = parseFloat(e.currentTarget.value) || null;
                  updateField('max_duty_cycle', parsed);
                  setEditingValues(prev => {
                    const { max_duty_cycle: _max_duty_cycle, ...rest } = prev;
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
                Maximum pump speed
              </div>
            </div>

            <div>
              <label style={{ display: 'block', fontSize: '0.85rem', color: '#666', marginBottom: '0.25rem' }}>
                Ramp Up Time (ms)
              </label>
              <input
                type="number"
                min="0"
                step="100"
                value={editingValues['ramp_up_time_ms'] ?? (localConfig.ramp_up_time_ms?.toString() ?? '')}
                onChange={(e) => {
                  const val = e.currentTarget.value;
                  setEditingValues(prev => ({ ...prev, ramp_up_time_ms: val }));
                }}
                onBlur={(e) => {
                  const parsed = parseFloat(e.currentTarget.value) || null;
                  updateField('ramp_up_time_ms', parsed);
                  setEditingValues(prev => {
                    const { ramp_up_time_ms: _ramp_up_time_ms, ...rest } = prev;
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
                Acceleration time
              </div>
            </div>

            <div>
              <label style={{ display: 'block', fontSize: '0.85rem', color: '#666', marginBottom: '0.25rem' }}>
                Ramp Down Time (ms)
              </label>
              <input
                type="number"
                min="0"
                step="100"
                value={editingValues['ramp_down_time_ms'] ?? (localConfig.ramp_down_time_ms?.toString() ?? '')}
                onChange={(e) => {
                  const val = e.currentTarget.value;
                  setEditingValues(prev => ({ ...prev, ramp_down_time_ms: val }));
                }}
                onBlur={(e) => {
                  const parsed = parseFloat(e.currentTarget.value) || null;
                  updateField('ramp_down_time_ms', parsed);
                  setEditingValues(prev => {
                    const { ramp_down_time_ms: _ramp_down_time_ms, ...rest } = prev;
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
                Deceleration time
              </div>
            </div>
          </div>

          <div style={{ marginTop: '1rem', paddingTop: '1rem', borderTop: '1px solid #ddd' }}>
            <label style={{ display: 'flex', alignItems: 'center', gap: '0.5rem', cursor: 'pointer', marginBottom: '0.75rem' }}>
              <input
                type="checkbox"
                checked={tachoEnabled}
                onChange={(e) => setTachoEnabled((e.target as HTMLInputElement).checked)}
                style={{ width: '1.2rem', height: '1.2rem' }}
              />
              <span style={{ fontSize: '0.9rem', fontWeight: '500' }}>Enable Flow Sensor Calibration</span>
            </label>

            {tachoEnabled && (
              <div>
                <label style={{ display: 'block', fontSize: '0.85rem', color: '#666', marginBottom: '0.25rem' }}>
                  Tacho Pulses per Liter
                </label>
                <input
                  type="number"
                  min="0"
                  step="1"
                  value={editingValues['tacho_pulses_per_liter'] ?? (localConfig.tacho_pulses_per_liter?.toString() ?? '')}
                  onChange={(e) => {
                    const val = e.currentTarget.value;
                    setEditingValues(prev => ({ ...prev, tacho_pulses_per_liter: val }));
                  }}
                  onBlur={(e) => {
                    const parsed = parseFloat(e.currentTarget.value) || null;
                    updateField('tacho_pulses_per_liter', parsed);
                    setEditingValues(prev => {
                      const { tacho_pulses_per_liter: _tacho_pulses_per_liter, ...rest } = prev;
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
                  Flow sensor calibration factor
                </div>
              </div>
            )}
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
