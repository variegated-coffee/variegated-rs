import { memo } from 'preact/compat';
import { useMachine } from '../contexts/MachineContext';
import { BoilerStatus } from '../schemas/schemas';

interface BoilerStatusCardProps {
  index: number;
  status: BoilerStatus;
}

const BoilerStatusCardComponent = ({ index, status }: BoilerStatusCardProps) => {
  const { getBoilerName, hasBoilerSensor } = useMachine();
  const name = getBoilerName(index);

  // Determine status color based on control mode
  const getStatusColor = () => {
    switch (status.control_state.mode.type) {
      case 'Temperature':
        return '#28a745'; // green
      case 'Pressure':
        return '#007bff'; // blue
      case 'Off':
        return '#6c757d'; // gray
      default:
        return '#6c757d';
    }
  };

  // Format output display
  const getOutputDisplay = () => {
    if (status.output.type === 'Off') {
      return 'Off';
    } else if (status.output.type === 'FixedDutyCycle') {
      return `Fixed: ${status.output.value.toFixed(1)}%`;
    } else if (status.output.type === 'PidOutput') {
      return `PID: ${status.output.value.out.toFixed(1)}%`;
    }
    return 'Unknown';
  };

  return (
    <div
      style={{
        padding: '1rem',
        backgroundColor: 'white',
        border: '1px solid #ddd',
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
          {status.control_state.mode.type}
        </span>
      </div>

      {/* Sensor Readings */}
      <div style={{ display: 'flex', flexDirection: 'column', gap: '0.5rem' }}>
        {hasBoilerSensor(index, { type: 'Temperature' }) && status.temperature !== null && status.temperature !== undefined && (
          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.9rem' }}>
            <span style={{ color: '#666' }}>Temperature:</span>
            <span style={{ fontWeight: '500' }}>{status.temperature.toFixed(1)}°C</span>
          </div>
        )}

        {hasBoilerSensor(index, { type: 'Pressure' }) && status.pressure !== null && status.pressure !== undefined && (
          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.9rem' }}>
            <span style={{ color: '#666' }}>Pressure:</span>
            <span style={{ fontWeight: '500' }}>{status.pressure.toFixed(2)} bar</span>
          </div>
        )}

        {hasBoilerSensor(index, { type: 'WaterLevel' }) && status.water_level !== null && status.water_level !== undefined && (
          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.9rem' }}>
            <span style={{ color: '#666' }}>Water Level:</span>
            <span style={{ fontWeight: '500' }}>{status.water_level.toFixed(1)}%</span>
          </div>
        )}
      </div>

      {/* Control Target - only show relevant target based on mode */}
      {status.control_state.mode.type !== 'Off' && (
        <div style={{ marginTop: '0.75rem', paddingTop: '0.75rem', borderTop: '1px solid #eee' }}>
          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.9rem' }}>
            <span style={{ color: '#666' }}>Target:</span>
            <span style={{ fontWeight: '500' }}>
              {status.control_state.mode.type === 'Temperature'
                ? `${status.control_state.values.target_temperature.toFixed(1)}°C`
                : `${status.control_state.values.target_pressure.toFixed(2)} bar`
              }
            </span>
          </div>
        </div>
      )}

      {/* Output */}
      <div style={{ marginTop: '0.75rem', paddingTop: '0.75rem', borderTop: '1px solid #eee' }}>
        <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.9rem' }}>
          <span style={{ color: '#666' }}>Output:</span>
          <span style={{ fontWeight: '500' }}>{getOutputDisplay()}</span>
        </div>

        {/* PID Details - show when using PID control */}
        {status.output.type === 'PidOutput' && (
          <div style={{ marginTop: '0.75rem', padding: '0.5rem', backgroundColor: '#f8f9fa', borderRadius: '4px' }}>
            <div style={{ fontSize: '0.75rem', color: '#666', marginBottom: '0.5rem', fontWeight: '500' }}>
              PID Terms:
            </div>
            <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '0.5rem', fontSize: '0.8rem' }}>
              <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                <span style={{ color: '#888' }}>P:</span>
                <span style={{ fontFamily: 'monospace' }}>{status.output.value.p.toFixed(2)}</span>
              </div>
              <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                <span style={{ color: '#888' }}>I:</span>
                <span style={{ fontFamily: 'monospace' }}>{status.output.value.i.toFixed(2)}</span>
              </div>
              <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                <span style={{ color: '#888' }}>D:</span>
                <span style={{ fontFamily: 'monospace' }}>{status.output.value.d.toFixed(2)}</span>
              </div>
              <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                <span style={{ color: '#888' }}>Sum:</span>
                <span style={{ fontFamily: 'monospace', fontWeight: '500' }}>{status.output.value.out.toFixed(2)}</span>
              </div>
            </div>
            <div style={{ fontSize: '0.75rem', color: '#666', marginTop: '0.5rem', marginBottom: '0.25rem', fontWeight: '500' }}>
              Acting Parameters:
            </div>
            <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '0.5rem', fontSize: '0.75rem' }}>
              <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                <span style={{ color: '#888' }}>Kp:</span>
                <span style={{ fontFamily: 'monospace' }}>{status.output.value.acting_kp.toFixed(4)}</span>
              </div>
              <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                <span style={{ color: '#888' }}>Ki:</span>
                <span style={{ fontFamily: 'monospace' }}>{status.output.value.acting_ki.toFixed(4)}</span>
              </div>
              <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                <span style={{ color: '#888' }}>Kd:</span>
                <span style={{ fontFamily: 'monospace' }}>{status.output.value.acting_kd.toFixed(4)}</span>
              </div>
            </div>
          </div>
        )}
      </div>
    </div>
  );
};

export const BoilerStatusCard = memo(BoilerStatusCardComponent);
