import { useState } from 'preact/hooks';
import { BoilerControlMode } from '../schemas/schemas';

interface BoilerControlEditorProps {
  name: string;
  currentMode: BoilerControlMode;
  currentTargetTemperature: number;
  currentTargetPressure: number;
  onSave: (data: { mode: BoilerControlMode; target_temperature?: number; target_pressure?: number }) => void;
  onCancel: () => void;
}

export const BoilerControlEditor = ({
  name,
  currentMode,
  currentTargetTemperature,
  currentTargetPressure,
  onSave,
  onCancel
}: BoilerControlEditorProps) => {
  const [mode, setMode] = useState<BoilerControlMode>(currentMode);
  const [targetTemperature, setTargetTemperature] = useState<string>(currentTargetTemperature.toFixed(1));
  const [targetPressure, setTargetPressure] = useState<string>(currentTargetPressure.toFixed(2));
  const [error, setError] = useState<string | null>(null);

  const handleSave = () => {
    setError(null);

    // Validate inputs
    const temp = parseFloat(targetTemperature);
    const pressure = parseFloat(targetPressure);

    if (isNaN(temp) || temp < 0 || temp > 200) {
      setError('Target temperature must be between 0 and 200°C');
      return;
    }

    if (isNaN(pressure) || pressure < 0 || pressure > 20) {
      setError('Target pressure must be between 0 and 20 bar');
      return;
    }

    // Call onSave with the new values
    onSave({
      mode,
      target_temperature: temp,
      target_pressure: pressure
    });
  };

  return (
    <div
      style={{
        padding: '1.5rem',
        backgroundColor: '#f8f9fa',
        borderRadius: '8px',
        border: '1px solid #ddd'
      }}
    >
      <h3 style={{ marginTop: 0, marginBottom: '1.5rem', fontSize: '1.2rem' }}>
        Edit Boiler Control - {name}
      </h3>

      {error && (
        <div
          style={{
            padding: '0.75rem',
            marginBottom: '1rem',
            backgroundColor: '#f8d7da',
            color: '#721c24',
            border: '1px solid #f5c6cb',
            borderRadius: '4px',
            fontSize: '0.9rem'
          }}
        >
          {error}
        </div>
      )}

      {/* Control Mode */}
      <div style={{ marginBottom: '1.5rem' }}>
        <label
          style={{
            display: 'block',
            marginBottom: '0.5rem',
            fontSize: '0.9rem',
            fontWeight: '500',
            color: '#333'
          }}
        >
          Control Mode
        </label>
        <select
          value={mode.type}
          onChange={(e) => {
            const modeType = (e.target as HTMLSelectElement).value;
            setMode({ type: modeType } as BoilerControlMode);
          }}
          style={{
            width: '100%',
            padding: '0.5rem',
            fontSize: '0.9rem',
            border: '1px solid #ccc',
            borderRadius: '4px',
            backgroundColor: 'white'
          }}
        >
          <option value="Temperature">Temperature Control</option>
          <option value="Pressure">Pressure Control</option>
          <option value="Off">Off</option>
        </select>
        <div style={{ fontSize: '0.75rem', color: '#666', marginTop: '0.25rem' }}>
          Select the control mode for this boiler
        </div>
      </div>

      {/* Target Temperature */}
      <div style={{ marginBottom: '1.5rem' }}>
        <label
          style={{
            display: 'block',
            marginBottom: '0.5rem',
            fontSize: '0.9rem',
            fontWeight: '500',
            color: '#333'
          }}
        >
          Target Temperature (°C)
        </label>
        <input
          type="number"
          step="0.1"
          min="0"
          max="200"
          value={targetTemperature}
          onChange={(e) => setTargetTemperature((e.target as HTMLInputElement).value)}
          style={{
            width: '100%',
            padding: '0.5rem',
            fontSize: '0.9rem',
            border: '1px solid #ccc',
            borderRadius: '4px'
          }}
        />
        <div style={{ fontSize: '0.75rem', color: '#666', marginTop: '0.25rem' }}>
          Used when control mode is set to Temperature
        </div>
      </div>

      {/* Target Pressure */}
      <div style={{ marginBottom: '1.5rem' }}>
        <label
          style={{
            display: 'block',
            marginBottom: '0.5rem',
            fontSize: '0.9rem',
            fontWeight: '500',
            color: '#333'
          }}
        >
          Target Pressure (bar)
        </label>
        <input
          type="number"
          step="0.01"
          min="0"
          max="20"
          value={targetPressure}
          onChange={(e) => setTargetPressure((e.target as HTMLInputElement).value)}
          style={{
            width: '100%',
            padding: '0.5rem',
            fontSize: '0.9rem',
            border: '1px solid #ccc',
            borderRadius: '4px'
          }}
        />
        <div style={{ fontSize: '0.75rem', color: '#666', marginTop: '0.25rem' }}>
          Used when control mode is set to Pressure
        </div>
      </div>

      {/* Buttons */}
      <div style={{ display: 'flex', gap: '1rem', marginTop: '2rem' }}>
        <button
          onClick={handleSave}
          style={{
            flex: 1,
            padding: '0.75rem',
            backgroundColor: '#28a745',
            color: 'white',
            border: 'none',
            borderRadius: '4px',
            fontSize: '0.9rem',
            fontWeight: '500',
            cursor: 'pointer'
          }}
        >
          Save Changes
        </button>
        <button
          onClick={onCancel}
          style={{
            flex: 1,
            padding: '0.75rem',
            backgroundColor: '#6c757d',
            color: 'white',
            border: 'none',
            borderRadius: '4px',
            fontSize: '0.9rem',
            fontWeight: '500',
            cursor: 'pointer'
          }}
        >
          Cancel
        </button>
      </div>
    </div>
  );
};
