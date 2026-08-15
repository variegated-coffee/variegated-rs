import { useState } from 'preact/hooks';
import { GroupBrewControlMode, GroupBrewControlTargetValues, ControlCurve } from '../schemas/schemas';

interface GroupControlEditorProps {
  name: string;
  currentMode: GroupBrewControlMode;
  currentValues: GroupBrewControlTargetValues;
  onSave: (data: {
    mode: GroupBrewControlMode;
    flow_rate?: number | null;
    pressure?: number | null;
    output_flow_rate?: number | null;
    duty_cycle?: number | null;
    duty_cycle_curve?: ControlCurve | null;
    flow_rate_curve?: ControlCurve | null;
    pressure_curve?: ControlCurve | null;
    output_flow_rate_curve?: ControlCurve | null;
  }) => void;
  onCancel: () => void;
}

export const GroupControlEditor = ({
  name,
  currentMode,
  currentValues,
  onSave,
  onCancel
}: GroupControlEditorProps) => {
  const [mode, setMode] = useState<GroupBrewControlMode>(currentMode);
  const [flowRate, setFlowRate] = useState<string>(currentValues.flow_rate.toFixed(1));
  const [pressure, setPressure] = useState<string>(currentValues.pressure.toFixed(2));
  const [outputFlowRate, setOutputFlowRate] = useState<string>(currentValues.output_flow_rate.toFixed(1));
  const [dutyCycle, setDutyCycle] = useState<string>(currentValues.duty_cycle.toString());
  const [error, setError] = useState<string | null>(null);

  // Determine which input to show based on mode
  const showFlowRate = mode.type === 'GroupFlowRate';
  const showPressure = mode.type === 'Pressure';
  const showOutputFlowRate = mode.type === 'OutputFlowRate';
  const showDutyCycle = mode.type === 'FixedDutyCycle';
  const isCurveMode = mode.type.includes('Curve');
  const isFullOnOrOff = mode.type === 'FullOn' || mode.type === 'Off';

  const handleSave = () => {
    setError(null);

    // Validate inputs based on mode
    // Initialize ALL optional fields to null for postcard serialization
    // (postcard requires null for Option<T>, not undefined)
    const data: {
      mode: GroupBrewControlMode;
      flow_rate?: number | null;
      pressure?: number | null;
      output_flow_rate?: number | null;
      duty_cycle?: number | null;
      duty_cycle_curve?: ControlCurve | null;
      flow_rate_curve?: ControlCurve | null;
      pressure_curve?: ControlCurve | null;
      output_flow_rate_curve?: ControlCurve | null;
    } = {
      mode,
      flow_rate: null,
      pressure: null,
      output_flow_rate: null,
      duty_cycle: null,
      duty_cycle_curve: null,
      flow_rate_curve: null,
      pressure_curve: null,
      output_flow_rate_curve: null
    };

    if (showFlowRate) {
      const val = parseFloat(flowRate);
      if (isNaN(val) || val < 0 || val > 20) {
        setError('Flow rate must be between 0 and 20 mL/s');
        return;
      }
      data.flow_rate = val;
    }

    if (showPressure) {
      const val = parseFloat(pressure);
      if (isNaN(val) || val < 0 || val > 20) {
        setError('Pressure must be between 0 and 20 bar');
        return;
      }
      data.pressure = val;
    }

    if (showOutputFlowRate) {
      const val = parseFloat(outputFlowRate);
      if (isNaN(val) || val < 0 || val > 20) {
        setError('Output flow rate must be between 0 and 20 mL/s');
        return;
      }
      data.output_flow_rate = val;
    }

    if (showDutyCycle) {
      const val = parseInt(dutyCycle);
      if (isNaN(val) || val < 0 || val > 100) {
        setError('Duty cycle must be between 0 and 100%');
        return;
      }
      data.duty_cycle = val;
    }

    onSave(data);
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
        Edit Group Brew Control - {name}
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
            setMode({ type: modeType } as GroupBrewControlMode);
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
          <option value="GroupFlowRate">Group Flow Rate</option>
          <option value="GroupFlowRateCurve">Group Flow Rate Curve</option>
          <option value="Pressure">Pressure</option>
          <option value="PressureCurve">Pressure Curve</option>
          <option value="OutputFlowRate">Output Flow Rate</option>
          <option value="OutputFlowRateCurve">Output Flow Rate Curve</option>
          <option value="FixedDutyCycle">Fixed Duty Cycle</option>
          <option value="FixedDutyCycleCurve">Fixed Duty Cycle Curve</option>
          <option value="FullOn">Full On</option>
          <option value="Off">Off</option>
        </select>
        <div style={{ fontSize: '0.75rem', color: '#666', marginTop: '0.25rem' }}>
          Select the brew control mode for this group
        </div>
      </div>

      {/* Curve mode info */}
      {isCurveMode && (
        <div
          style={{
            padding: '0.75rem',
            marginBottom: '1.5rem',
            backgroundColor: '#d1ecf1',
            color: '#0c5460',
            border: '1px solid #bee5eb',
            borderRadius: '4px',
            fontSize: '0.85rem'
          }}
        >
          <strong>Note:</strong> Curve modes use the configured curve. To edit the curve, use the
          corresponding curve editor from the main configuration screen.
        </div>
      )}

      {/* Full On / Off info */}
      {isFullOnOrOff && (
        <div
          style={{
            padding: '0.75rem',
            marginBottom: '1.5rem',
            backgroundColor: '#d1ecf1',
            color: '#0c5460',
            border: '1px solid #bee5eb',
            borderRadius: '4px',
            fontSize: '0.85rem'
          }}
        >
          <strong>Note:</strong> {mode.type === 'FullOn' ? 'Pump will run at 100% duty cycle' : 'Pump will be off'}
        </div>
      )}

      {/* Conditional target value inputs */}
      {showFlowRate && (
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
            Target Flow Rate (mL/s)
          </label>
          <input
            type="number"
            step="0.1"
            min="0"
            max="20"
            value={flowRate}
            onChange={(e) => setFlowRate((e.target as HTMLInputElement).value)}
            style={{
              width: '100%',
              padding: '0.5rem',
              fontSize: '0.9rem',
              border: '1px solid #ccc',
              borderRadius: '4px'
            }}
          />
          <div style={{ fontSize: '0.75rem', color: '#666', marginTop: '0.25rem' }}>
            Target flow rate at the group (input side)
          </div>
        </div>
      )}

      {showPressure && (
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
            value={pressure}
            onChange={(e) => setPressure((e.target as HTMLInputElement).value)}
            style={{
              width: '100%',
              padding: '0.5rem',
              fontSize: '0.9rem',
              border: '1px solid #ccc',
              borderRadius: '4px'
            }}
          />
          <div style={{ fontSize: '0.75rem', color: '#666', marginTop: '0.25rem' }}>
            Target brewing pressure
          </div>
        </div>
      )}

      {showOutputFlowRate && (
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
            Target Output Flow Rate (mL/s)
          </label>
          <input
            type="number"
            step="0.1"
            min="0"
            max="20"
            value={outputFlowRate}
            onChange={(e) => setOutputFlowRate((e.target as HTMLInputElement).value)}
            style={{
              width: '100%',
              padding: '0.5rem',
              fontSize: '0.9rem',
              border: '1px solid #ccc',
              borderRadius: '4px'
            }}
          />
          <div style={{ fontSize: '0.75rem', color: '#666', marginTop: '0.25rem' }}>
            Target flow rate at the output (scale)
          </div>
        </div>
      )}

      {showDutyCycle && (
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
            Duty Cycle (%)
          </label>
          <input
            type="number"
            step="1"
            min="0"
            max="100"
            value={dutyCycle}
            onChange={(e) => setDutyCycle((e.target as HTMLInputElement).value)}
            style={{
              width: '100%',
              padding: '0.5rem',
              fontSize: '0.9rem',
              border: '1px solid #ccc',
              borderRadius: '4px'
            }}
          />
          <div style={{ fontSize: '0.75rem', color: '#666', marginTop: '0.25rem' }}>
            Fixed pump duty cycle (0-100%)
          </div>
        </div>
      )}

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
