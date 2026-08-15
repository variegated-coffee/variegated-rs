import { Configuration } from '../schemas/schemas';
import { ConfigurationSection } from './ConfigurationSection';
import { useMachine } from '../contexts/MachineContext';

interface GroupConfigurationDetailProps {
  entityKey: number;
  name: string;
  configuration: Configuration;
  onNavigateToParameter: (category: string) => void;
}

export const GroupConfigurationDetail = ({
  entityKey,
  name,
  configuration,
  onNavigateToParameter
}: GroupConfigurationDetailProps) => {
  const { getTankName } = useMachine();
  const groupConfig = configuration.group_configurations.get(entityKey);

  if (!groupConfig) {
    return <div>Group configuration not found</div>;
  }

  const formatControlMode = (mode: string) => {
    const modes: Record<string, string> = {
      GroupFlowRate: 'Group Flow Rate',
      GroupFlowRateCurve: 'Group Flow Rate Curve',
      Pressure: 'Pressure',
      PressureCurve: 'Pressure Curve',
      OutputFlowRate: 'Output Flow Rate',
      OutputFlowRateCurve: 'Output Flow Rate Curve',
      FixedDutyCycle: 'Fixed Duty Cycle',
      FixedDutyCycleCurve: 'Fixed Duty Cycle Curve',
      FullOn: 'Full On',
      Off: 'Off'
    };
    return modes[mode] || mode;
  };

  const getTargetValueForMode = () => {
    const mode = groupConfig.brew_control_state.mode;
    const values = groupConfig.brew_control_state.values;

    switch (mode.type) {
      case 'GroupFlowRate':
        return `${values.flow_rate.toFixed(1)} mL/s`;
      case 'Pressure':
        return `${values.pressure.toFixed(2)} bar`;
      case 'OutputFlowRate':
        return `${values.output_flow_rate.toFixed(1)} mL/s`;
      case 'FixedDutyCycle':
        return `${values.duty_cycle.toFixed(1)}%`;
      case 'GroupFlowRateCurve':
      case 'PressureCurve':
      case 'OutputFlowRateCurve':
      case 'FixedDutyCycleCurve':
        return 'Using curve';
      case 'FullOn':
        return '100%';
      case 'Off':
        return 'Off';
      default:
        return 'N/A';
    }
  };

  return (
    <div>
      <div style={{ marginBottom: '1.5rem' }}>
        <h2 style={{ margin: 0, fontSize: '1.3rem' }}>{name}</h2>
        <div style={{ fontSize: '0.85rem', color: '#666', marginTop: '0.25rem' }}>
          Group ID: {entityKey}
        </div>
      </div>

      {/* Basic Settings */}
      <ConfigurationSection title="Basic Settings">
        <div style={{ display: 'flex', flexDirection: 'column', gap: '0.75rem' }}>
          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
            <span style={{ fontSize: '0.9rem', color: '#666' }}>Auto-Tare Scale:</span>
            <span
              style={{
                padding: '0.25rem 0.75rem',
                backgroundColor: groupConfig.auto_tare_enabled ? '#28a745' : '#6c757d',
                color: 'white',
                borderRadius: '12px',
                fontSize: '0.8rem',
                fontWeight: '500'
              }}
            >
              {groupConfig.auto_tare_enabled ? 'Enabled' : 'Disabled'}
            </span>
          </div>

          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.9rem' }}>
            <span style={{ color: '#666' }}>Supply Tank:</span>
            <span style={{ fontWeight: '500' }}>
              {groupConfig.supply_tank_index !== null && groupConfig.supply_tank_index !== undefined
                ? getTankName(groupConfig.supply_tank_index)
                : 'Not configured'}
            </span>
          </div>

          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.9rem' }}>
            <span style={{ color: '#666' }}>Max Brew Time:</span>
            <span style={{ fontWeight: '500' }}>
              {groupConfig.max_brew_time_seconds !== null && groupConfig.max_brew_time_seconds !== undefined
                ? `${groupConfig.max_brew_time_seconds}s`
                : 'Not set'}
            </span>
          </div>

          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.9rem' }}>
            <span style={{ color: '#666' }}>Flow Sensor Calibration:</span>
            <span style={{ fontWeight: '500' }}>
              {groupConfig.flow_sensor_pulses_per_liter !== null && groupConfig.flow_sensor_pulses_per_liter !== undefined
                ? `${groupConfig.flow_sensor_pulses_per_liter} pulses/L`
                : 'Not set'}
            </span>
          </div>
        </div>
      </ConfigurationSection>

      {/* Control State */}
      <ConfigurationSection title="Control State">
        <div style={{ display: 'flex', flexDirection: 'column', gap: '0.75rem' }}>
          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
            <span style={{ fontSize: '0.9rem', color: '#666' }}>Control Mode:</span>
            <span
              style={{
                padding: '0.25rem 0.75rem',
                backgroundColor: groupConfig.brew_control_state.mode.type === 'Off' ? '#6c757d' : '#007bff',
                color: 'white',
                borderRadius: '12px',
                fontSize: '0.8rem',
                fontWeight: '500'
              }}
            >
              {formatControlMode(groupConfig.brew_control_state.mode.type)}
            </span>
          </div>

          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.9rem' }}>
            <span style={{ color: '#666' }}>Current Target:</span>
            <span style={{ fontWeight: '500' }}>{getTargetValueForMode()}</span>
          </div>

          <div style={{ marginTop: '0.5rem' }}>
            <button
              onClick={() => onNavigateToParameter('group_control')}
              style={{
                width: '100%',
                padding: '0.5rem 1rem',
                backgroundColor: '#0066cc',
                color: 'white',
                border: 'none',
                borderRadius: '4px',
                cursor: 'pointer',
                fontSize: '0.85rem',
                fontWeight: '500'
              }}
            >
              Edit Control Settings →
            </button>
          </div>
        </div>
      </ConfigurationSection>

      {/* PID Controllers */}
      <ConfigurationSection title="PID Controllers">
        <div style={{ display: 'flex', flexDirection: 'column', gap: '0.75rem' }}>
          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
            <span style={{ fontSize: '0.9rem' }}>Group Flow Rate PID</span>
            <button
              onClick={() => onNavigateToParameter('flow_rate_pid')}
              style={{
                padding: '0.4rem 1rem',
                backgroundColor: '#0066cc',
                color: 'white',
                border: 'none',
                borderRadius: '4px',
                cursor: 'pointer',
                fontSize: '0.85rem',
                fontWeight: '500'
              }}
            >
              Edit Parameters →
            </button>
          </div>

          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
            <span style={{ fontSize: '0.9rem' }}>Output Flow Rate PID</span>
            <button
              onClick={() => onNavigateToParameter('output_flow_rate_pid')}
              style={{
                padding: '0.4rem 1rem',
                backgroundColor: '#0066cc',
                color: 'white',
                border: 'none',
                borderRadius: '4px',
                cursor: 'pointer',
                fontSize: '0.85rem',
                fontWeight: '500'
              }}
            >
              Edit Parameters →
            </button>
          </div>

          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
            <span style={{ fontSize: '0.9rem' }}>Pressure PID</span>
            <button
              onClick={() => onNavigateToParameter('pressure_pid')}
              style={{
                padding: '0.4rem 1rem',
                backgroundColor: '#0066cc',
                color: 'white',
                border: 'none',
                borderRadius: '4px',
                cursor: 'pointer',
                fontSize: '0.85rem',
                fontWeight: '500'
              }}
            >
              Edit Parameters →
            </button>
          </div>
        </div>
      </ConfigurationSection>

      {/* Control Curves */}
      <ConfigurationSection title="Control Curves">
        <div style={{ display: 'flex', flexDirection: 'column', gap: '0.75rem' }}>
          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
            <span style={{ fontSize: '0.9rem' }}>Flow Rate Curve</span>
            <button
              onClick={() => onNavigateToParameter('flow_rate_curve')}
              style={{
                padding: '0.4rem 1rem',
                backgroundColor: '#0066cc',
                color: 'white',
                border: 'none',
                borderRadius: '4px',
                cursor: 'pointer',
                fontSize: '0.85rem',
                fontWeight: '500'
              }}
            >
              Edit Curve →
            </button>
          </div>

          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
            <span style={{ fontSize: '0.9rem' }}>Output Flow Rate Curve</span>
            <button
              onClick={() => onNavigateToParameter('output_flow_rate_curve')}
              style={{
                padding: '0.4rem 1rem',
                backgroundColor: '#0066cc',
                color: 'white',
                border: 'none',
                borderRadius: '4px',
                cursor: 'pointer',
                fontSize: '0.85rem',
                fontWeight: '500'
              }}
            >
              Edit Curve →
            </button>
          </div>

          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
            <span style={{ fontSize: '0.9rem' }}>Pressure Curve</span>
            <button
              onClick={() => onNavigateToParameter('pressure_curve')}
              style={{
                padding: '0.4rem 1rem',
                backgroundColor: '#0066cc',
                color: 'white',
                border: 'none',
                borderRadius: '4px',
                cursor: 'pointer',
                fontSize: '0.85rem',
                fontWeight: '500'
              }}
            >
              Edit Curve →
            </button>
          </div>

          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
            <span style={{ fontSize: '0.9rem' }}>Duty Cycle Curve</span>
            <button
              onClick={() => onNavigateToParameter('duty_cycle_curve')}
              style={{
                padding: '0.4rem 1rem',
                backgroundColor: '#0066cc',
                color: 'white',
                border: 'none',
                borderRadius: '4px',
                cursor: 'pointer',
                fontSize: '0.85rem',
                fontWeight: '500'
              }}
            >
              Edit Curve →
            </button>
          </div>
        </div>
      </ConfigurationSection>

      {/* Sensor Filtering */}
      <ConfigurationSection title="Sensor Filtering">
        <div style={{ display: 'flex', flexDirection: 'column', gap: '0.75rem' }}>
          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
            <div>
              <div style={{ fontSize: '0.9rem' }}>Pressure Kalman Filter</div>
              <div style={{ fontSize: '0.75rem', color: '#999', marginTop: '0.1rem' }}>
                {groupConfig.pressure_sensor_kalman_parameters ? 'Enabled' : 'Disabled'}
              </div>
            </div>
            <button
              onClick={() => onNavigateToParameter('pressure_kalman')}
              style={{
                padding: '0.4rem 1rem',
                backgroundColor: '#0066cc',
                color: 'white',
                border: 'none',
                borderRadius: '4px',
                cursor: 'pointer',
                fontSize: '0.85rem',
                fontWeight: '500'
              }}
            >
              Edit →
            </button>
          </div>
        </div>
      </ConfigurationSection>

      {/* Pump Configuration */}
      <ConfigurationSection title="Pump Configuration">
        {groupConfig.pump_configuration ? (
          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
            <div>
              <div style={{ fontSize: '0.9rem' }}>Pump Parameters</div>
              <div style={{ fontSize: '0.75rem', color: '#999', marginTop: '0.1rem' }}>
                Duty cycle, ramp times, flow sensor
              </div>
            </div>
            <button
              onClick={() => onNavigateToParameter('pump_config')}
              style={{
                padding: '0.4rem 1rem',
                backgroundColor: '#0066cc',
                color: 'white',
                border: 'none',
                borderRadius: '4px',
                cursor: 'pointer',
                fontSize: '0.85rem',
                fontWeight: '500'
              }}
            >
              Edit →
            </button>
          </div>
        ) : (
          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
            <div>
              <div style={{ fontSize: '0.9rem', color: '#999' }}>Not configured</div>
              <div style={{ fontSize: '0.75rem', color: '#999', marginTop: '0.1rem' }}>
                Configure pump duty cycle, ramp times, and flow sensor
              </div>
            </div>
            <button
              onClick={() => onNavigateToParameter('pump_config')}
              style={{
                padding: '0.4rem 1rem',
                backgroundColor: '#28a745',
                color: 'white',
                border: 'none',
                borderRadius: '4px',
                cursor: 'pointer',
                fontSize: '0.85rem',
                fontWeight: '500'
              }}
            >
              Configure
            </button>
          </div>
        )}
      </ConfigurationSection>
    </div>
  );
};
