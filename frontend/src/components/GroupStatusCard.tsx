import { memo } from 'preact/compat';
import { useState } from 'preact/hooks';
import { useMachine } from '../contexts/MachineContext';
import { GroupStatus } from '../schemas/schemas';
import { getWebSocketService } from '../services/websocket';

interface GroupStatusCardProps {
  index: number;
  status: GroupStatus;
}

const GroupStatusCardComponent = ({ index, status }: GroupStatusCardProps) => {
  const { getGroupName, hasGroupSensor } = useMachine();
  const name = getGroupName(index);
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

  const handleTareScale = () => {
    const ws = getWebSocketService();
    if (!ws) {
      showError('WebSocket not connected');
      return;
    }
    ws.tareGroupScale(index);
    showSuccess('Scale tared successfully');
  };

  const handleZeroCalibrateScale = () => {
    const ws = getWebSocketService();
    if (!ws) {
      showError('WebSocket not connected');
      return;
    }
    ws.zeroCalibrateGroupScale(index);
    showSuccess('Scale zero calibrated successfully');
  };

  const handleCalibrateScale100g = () => {
    const ws = getWebSocketService();
    if (!ws) {
      showError('WebSocket not connected');
      return;
    }
    ws.calibrateGroupScale100g(index);
    showSuccess('Scale calibrated with 100g successfully');
  };

  // Determine status color based on brewing state
  const getStatusColor = () => {
    if (status.is_brewing) {
      return '#28a745'; // green - active
    }
    return '#6c757d'; // gray - idle
  };

  // Format shot state display
  const getShotStateDisplay = () => {
    // Shot state is a property of the brew in progress, not of the group, so it
    // only exists while `current_brew` does. Bound to a local because optional
    // chaining does not narrow across statements.
    const shotState = status.current_brew?.shot_state;
    if (!shotState) return null;

    const stateInfo: Record<string, { label: string; color: string; description: string }> = {
      HeadspaceFill: { label: 'Headspace Fill', color: '#ffc107', description: 'Filling headspace & wetting puck' },
      Saturation: { label: 'Saturation', color: '#fd7e14', description: 'Puck saturating, pressure building' },
      PostFirstDrop: { label: 'Extracting', color: '#28a745', description: 'First drops detected, extracting' }
    };

    const info = stateInfo[shotState.type];
    return info || null;
  };

  const shotStateDisplay = getShotStateDisplay();

  // Format brew time
  const formatBrewTime = (brew_time: { secs: bigint; nanos: number } | null | undefined) => {
    if (!brew_time) return '0s';
    const totalMs = Number(brew_time.secs) * 1000 + brew_time.nanos / 1000000;
    return `${(totalMs / 1000).toFixed(1)}s`;
  };

  // Format output display
  const getOutputDisplay = () => {
    if (status.pump_output.type === 'Off') {
      return 'Off';
    } else if (status.pump_output.type === 'FixedDutyCycle') {
      return `${status.pump_output.value.toFixed(1)}%`;
    } else if (status.pump_output.type === 'PidOutput') {
      return `${status.pump_output.value.out.toFixed(1)}%`;
    }
    return 'Unknown';
  };

  return (
    <div
      style={{
        padding: '1rem',
        backgroundColor: 'white',
        border: status.is_brewing ? '2px solid #28a745' : '1px solid #ddd',
        borderRadius: '8px',
        flex: '1 1 300px',
        minWidth: '250px'
      }}
    >
      {/* Header */}
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '0.75rem' }}>
        <h3 style={{ margin: 0, fontSize: '1.1rem', fontWeight: '600' }}>{name}</h3>
        <span
          style={{
            padding: '0.25rem 0.75rem',
            backgroundColor: getStatusColor(),
            color: 'white',
            borderRadius: '12px',
            fontSize: '0.75rem',
            fontWeight: '500'
          }}
        >
          {status.is_brewing ? `BREWING (${formatBrewTime(status.current_brew?.brew_time)})` : 'IDLE'}
        </span>
      </div>

      {/* Control Mode */}
      <div style={{ marginBottom: '0.75rem' }}>
        <span style={{ fontSize: '0.85rem', color: '#666' }}>Mode: </span>
        <span style={{ fontSize: '0.85rem', fontWeight: '500' }}>{status.control_state.mode.type}</span>
      </div>

      {/* Sensor Readings */}
      <div style={{ display: 'flex', flexDirection: 'column', gap: '0.5rem' }}>
        {hasGroupSensor(index, { type: 'Temperature' }) && status.temperature !== null && status.temperature !== undefined && (
          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.9rem' }}>
            <span style={{ color: '#666' }}>Temperature:</span>
            <span style={{ fontWeight: '500' }}>{status.temperature.toFixed(1)}°C</span>
          </div>
        )}

        {hasGroupSensor(index, { type: 'Pressure' }) && status.pressure !== null && status.pressure !== undefined && (
          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.9rem' }}>
            <span style={{ color: '#666' }}>Pressure:</span>
            <span style={{ fontWeight: '500' }}>{status.pressure.toFixed(2)} bar</span>
          </div>
        )}

        {hasGroupSensor(index, { type: 'InputFlowRate' }) && status.input_flow_rate !== null && status.input_flow_rate !== undefined && (
          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.9rem' }}>
            <span style={{ color: '#666' }}>Input Flow:</span>
            <span style={{ fontWeight: '500' }}>{status.input_flow_rate.toFixed(1)} mL/s</span>
          </div>
        )}

        {hasGroupSensor(index, { type: 'OutputFlowRate' }) && status.output_flow_rate !== null && status.output_flow_rate !== undefined && (
          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.9rem' }}>
            <span style={{ color: '#666' }}>Output Flow:</span>
            <span style={{ fontWeight: '500' }}>{status.output_flow_rate.toFixed(1)} mL/s</span>
          </div>
        )}

        {hasGroupSensor(index, { type: 'Weight' }) && status.output_weight !== null && status.output_weight !== undefined && (
          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.9rem' }}>
            <span style={{ color: '#666' }}>Output Weight:</span>
            <span style={{ fontWeight: '500' }}>{status.output_weight.toFixed(1)} g</span>
          </div>
        )}

        {/* Brew-sensor readings. Output temperature and extraction rate have no
            SensorCapability of their own to gate on, so they rely on the value
            being present, which it only is once a BrewSensor reports. */}
        {status.output_temperature !== null && status.output_temperature !== undefined && (
          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.9rem' }}>
            <span style={{ color: '#666' }}>Output Temp:</span>
            <span style={{ fontWeight: '500' }}>{status.output_temperature.toFixed(1)}°C</span>
          </div>
        )}

        {hasGroupSensor(index, { type: 'ElectricalConductivity' }) && status.output_electrical_conductivity !== null && status.output_electrical_conductivity !== undefined && (
          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.9rem' }}>
            <span style={{ color: '#666' }}>EC:</span>
            <span style={{ fontWeight: '500' }}>{status.output_electrical_conductivity.toFixed(0)} µS/cm</span>
          </div>
        )}

        {status.extraction_rate !== null && status.extraction_rate !== undefined && (
          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.9rem' }}>
            <span style={{ color: '#666' }}>Extraction:</span>
            <span style={{ fontWeight: '500' }}>{status.extraction_rate.toFixed(1)}%</span>
          </div>
        )}
      </div>

      {/* Brew Stats (if brewing or previous brew exists) */}
      {status.is_brewing && (
        <div style={{ marginTop: '0.75rem', paddingTop: '0.75rem', borderTop: '1px solid #eee' }}>
          <div style={{ fontSize: '0.85rem', color: '#666', marginBottom: '0.25rem' }}>Current Shot:</div>

          {/* Shot State Indicator */}
          {shotStateDisplay && (
            <div style={{
              marginBottom: '0.5rem',
              padding: '0.375rem 0.5rem',
              backgroundColor: `${shotStateDisplay.color}22`,
              borderLeft: `3px solid ${shotStateDisplay.color}`,
              borderRadius: '4px'
            }}>
              <div style={{
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'space-between',
                gap: '0.5rem'
              }}>
                <span style={{
                  fontWeight: 600,
                  color: shotStateDisplay.color,
                  fontSize: '0.85rem'
                }}>
                  {shotStateDisplay.label}
                </span>
              </div>
              <div style={{
                fontSize: '0.75rem',
                color: '#666',
                marginTop: '0.125rem'
              }}>
                {shotStateDisplay.description}
              </div>
            </div>
          )}

          <div style={{ display: 'flex', flexDirection: 'column', gap: '0.25rem', fontSize: '0.85rem' }}>
            {status.current_brew != null && status.current_brew.brew_input_volume != null && (
              <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                <span style={{ color: '#888' }}>Input Volume:</span>
                <span>{status.current_brew.brew_input_volume.toFixed(1)} mL</span>
              </div>
            )}

            {status.current_brew != null && status.current_brew.output_volume != null && (
              <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                <span style={{ color: '#888' }}>Output Volume:</span>
                <span>{status.current_brew.output_volume.toFixed(1)} mL</span>
              </div>
            )}

            {status.current_brew != null && status.current_brew.extracted_solids != null && (
              <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                <span style={{ color: '#888' }}>Extracted Solids:</span>
                <span>{status.current_brew.extracted_solids.toFixed(1)} g</span>
              </div>
            )}
          </div>
        </div>
      )}

      {!status.is_brewing && status.previous_brew && (
        <div style={{ marginTop: '0.75rem', paddingTop: '0.75rem', borderTop: '1px solid #eee' }}>
          <div style={{ fontSize: '0.85rem', color: '#666', marginBottom: '0.25rem' }}>Last Shot:</div>
          <div style={{ display: 'flex', flexDirection: 'column', gap: '0.25rem', fontSize: '0.85rem' }}>
            <div style={{ display: 'flex', justifyContent: 'space-between' }}>
              <span style={{ color: '#888' }}>Time:</span>
              <span>{formatBrewTime(status.previous_brew.brew_time)}</span>
            </div>
            {status.previous_brew.output_weight !== null && status.previous_brew.output_weight !== undefined && (
              <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                <span style={{ color: '#888' }}>Weight:</span>
                <span>{status.previous_brew.output_weight.toFixed(1)} g</span>
              </div>
            )}
          </div>
        </div>
      )}

      {/* Pump Output */}
      <div style={{ marginTop: '0.75rem', paddingTop: '0.75rem', borderTop: '1px solid #eee' }}>
        <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.9rem' }}>
          <span style={{ color: '#666' }}>Pump Output:</span>
          <span style={{ fontWeight: '500' }}>{getOutputDisplay()}</span>
        </div>

        {/* PID Details - show when using PID control */}
        {status.pump_output.type === 'PidOutput' && (
          <div style={{ marginTop: '0.75rem', padding: '0.5rem', backgroundColor: '#f8f9fa', borderRadius: '4px' }}>
            <div style={{ fontSize: '0.75rem', color: '#666', marginBottom: '0.5rem', fontWeight: '500' }}>
              PID Terms:
            </div>
            <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '0.5rem', fontSize: '0.8rem' }}>
              <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                <span style={{ color: '#888' }}>P:</span>
                <span style={{ fontFamily: 'monospace' }}>{status.pump_output.value.p.toFixed(2)}</span>
              </div>
              <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                <span style={{ color: '#888' }}>I:</span>
                <span style={{ fontFamily: 'monospace' }}>{status.pump_output.value.i.toFixed(2)}</span>
              </div>
              <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                <span style={{ color: '#888' }}>D:</span>
                <span style={{ fontFamily: 'monospace' }}>{status.pump_output.value.d.toFixed(2)}</span>
              </div>
              <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                <span style={{ color: '#888' }}>Sum:</span>
                <span style={{ fontFamily: 'monospace', fontWeight: '500' }}>{status.pump_output.value.out.toFixed(2)}</span>
              </div>
            </div>
            <div style={{ fontSize: '0.75rem', color: '#666', marginTop: '0.5rem', marginBottom: '0.25rem', fontWeight: '500' }}>
              Acting Parameters:
            </div>
            <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '0.5rem', fontSize: '0.75rem' }}>
              <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                <span style={{ color: '#888' }}>Kp:</span>
                <span style={{ fontFamily: 'monospace' }}>{status.pump_output.value.acting_kp.toFixed(4)}</span>
              </div>
              <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                <span style={{ color: '#888' }}>Ki:</span>
                <span style={{ fontFamily: 'monospace' }}>{status.pump_output.value.acting_ki.toFixed(4)}</span>
              </div>
              <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                <span style={{ color: '#888' }}>Kd:</span>
                <span style={{ fontFamily: 'monospace' }}>{status.pump_output.value.acting_kd.toFixed(4)}</span>
              </div>
            </div>
          </div>
        )}
      </div>

      {/* Scale Calibration Actions */}
      {hasGroupSensor(index, { type: 'Weight' }) && (
        <div style={{ marginTop: '0.75rem', paddingTop: '0.75rem', borderTop: '1px solid #eee' }}>
          <div style={{ fontSize: '0.85rem', color: '#666', marginBottom: '0.5rem', fontWeight: '500' }}>
            Scale Actions:
          </div>

          {/* Success Message */}
          {successMessage && (
            <div style={{
              fontSize: '0.85rem',
              color: '#155724',
              marginBottom: '0.5rem',
              padding: '0.5rem',
              backgroundColor: '#d4edda',
              borderRadius: '4px',
              border: '1px solid #c3e6cb'
            }}>
              ✅ {successMessage}
            </div>
          )}

          {/* Error Message */}
          {error && (
            <div style={{
              fontSize: '0.85rem',
              color: '#721c24',
              marginBottom: '0.5rem',
              padding: '0.5rem',
              backgroundColor: '#f8d7da',
              borderRadius: '4px',
              border: '1px solid #f5c6cb'
            }}>
              ❌ {error}
            </div>
          )}

          <div style={{ display: 'flex', flexDirection: 'column', gap: '0.5rem' }}>
            <button
              onClick={() => void handleTareScale()}
              style={{
                padding: '0.5rem 0.75rem',
                backgroundColor: '#0066cc',
                color: 'white',
                border: 'none',
                borderRadius: '4px',
                cursor: 'pointer',
                fontSize: '0.85rem',
                fontWeight: '500'
              }}
            >
              Tare Scale
            </button>
            <button
              onClick={() => void handleZeroCalibrateScale()}
              style={{
                padding: '0.5rem 0.75rem',
                backgroundColor: '#fd7e14',
                color: 'white',
                border: 'none',
                borderRadius: '4px',
                cursor: 'pointer',
                fontSize: '0.85rem',
                fontWeight: '500'
              }}
            >
              Zero Calibrate
            </button>
            <button
              onClick={() => void handleCalibrateScale100g()}
              style={{
                padding: '0.5rem 0.75rem',
                backgroundColor: '#28a745',
                color: 'white',
                border: 'none',
                borderRadius: '4px',
                cursor: 'pointer',
                fontSize: '0.85rem',
                fontWeight: '500'
              }}
            >
              Calibrate (100g)
            </button>
          </div>
        </div>
      )}
    </div>
  );
};

export const GroupStatusCard = memo(GroupStatusCardComponent);
