import { Configuration } from '../schemas/schemas';
import { ConfigurationSection } from './ConfigurationSection';
import { useMachine } from '../contexts/MachineContext';

interface BoilerConfigurationDetailProps {
  entityKey: number;
  name: string;
  configuration: Configuration;
  onNavigateToParameter: (category: string) => void;
}

export const BoilerConfigurationDetail = ({
  entityKey,
  name,
  configuration,
  onNavigateToParameter
}: BoilerConfigurationDetailProps) => {
  const { getTankName } = useMachine();
  const boilerConfig = configuration.boiler_configurations.get(entityKey);

  if (!boilerConfig) {
    return <div>Boiler configuration not found</div>;
  }

  const formatControlMode = (modeType: string) => {
    const modes: Record<string, string> = {
      Temperature: 'Temperature Control',
      Pressure: 'Pressure Control',
      Off: 'Off'
    };
    return modes[modeType] || modeType;
  };

  return (
    <div>
      <div style={{ marginBottom: '1.5rem' }}>
        <h2 style={{ margin: 0, fontSize: '1.3rem' }}>{name}</h2>
        <div style={{ fontSize: '0.85rem', color: '#666', marginTop: '0.25rem' }}>
          Boiler ID: {entityKey}
        </div>
      </div>

      {/* Control State */}
      <ConfigurationSection title="Control State">
        <div style={{ display: 'flex', flexDirection: 'column', gap: '0.75rem' }}>
          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
            <span style={{ fontSize: '0.9rem', color: '#666' }}>Control Mode:</span>
            <span
              style={{
                padding: '0.25rem 0.75rem',
                backgroundColor: boilerConfig.control_state.mode.type === 'Off' ? '#6c757d' : '#28a745',
                color: 'white',
                borderRadius: '12px',
                fontSize: '0.8rem',
                fontWeight: '500'
              }}
            >
              {formatControlMode(boilerConfig.control_state.mode.type)}
            </span>
          </div>

          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.9rem' }}>
            <span style={{ color: '#666' }}>Target Temperature:</span>
            <span style={{ fontWeight: '500' }}>
              {boilerConfig.control_state.values.target_temperature.toFixed(1)}°C
            </span>
          </div>

          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.9rem' }}>
            <span style={{ color: '#666' }}>Target Pressure:</span>
            <span style={{ fontWeight: '500' }}>
              {boilerConfig.control_state.values.target_pressure.toFixed(2)} bar
            </span>
          </div>

          <div style={{ marginTop: '0.5rem' }}>
            <button
              onClick={() => onNavigateToParameter('boiler_control')}
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

      {/* Safety Limits */}
      <ConfigurationSection title="Safety Limits">
        <div style={{ display: 'flex', flexDirection: 'column', gap: '0.75rem' }}>
          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.9rem' }}>
            <span style={{ color: '#666' }}>Supply Tank:</span>
            <span style={{ fontWeight: '500' }}>
              {boilerConfig.supply_tank_index !== null && boilerConfig.supply_tank_index !== undefined
                ? getTankName(boilerConfig.supply_tank_index)
                : 'Not configured'}
            </span>
          </div>

          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.9rem' }}>
            <span style={{ color: '#666' }}>Minimum Safe Level:</span>
            <span style={{ fontWeight: '500' }}>
              {boilerConfig.minimum_safe_level !== null && boilerConfig.minimum_safe_level !== undefined
                ? `${boilerConfig.minimum_safe_level.toFixed(1)}%`
                : 'Not set'}
            </span>
          </div>

          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.9rem' }}>
            <span style={{ color: '#666' }}>Max Temperature:</span>
            <span style={{ fontWeight: '500' }}>
              {boilerConfig.max_temperature !== null && boilerConfig.max_temperature !== undefined
                ? `${boilerConfig.max_temperature.toFixed(1)}°C`
                : 'Not set'}
            </span>
          </div>

          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.9rem' }}>
            <span style={{ color: '#666' }}>Max Pressure:</span>
            <span style={{ fontWeight: '500' }}>
              {boilerConfig.max_pressure !== null && boilerConfig.max_pressure !== undefined
                ? `${boilerConfig.max_pressure.toFixed(2)} bar`
                : 'Not set'}
            </span>
          </div>
        </div>
      </ConfigurationSection>

      {/* PID Controllers */}
      <ConfigurationSection title="PID Controllers">
        <div style={{ display: 'flex', flexDirection: 'column', gap: '0.75rem' }}>
          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
            <span style={{ fontSize: '0.9rem' }}>Temperature PID</span>
            <button
              onClick={() => onNavigateToParameter('temperature_pid')}
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

      {/* Sensor Filtering */}
      <ConfigurationSection title="Sensor Filtering">
        <div style={{ display: 'flex', flexDirection: 'column', gap: '0.75rem' }}>
          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
            <div>
              <div style={{ fontSize: '0.9rem' }}>Temperature Kalman Filter</div>
              <div style={{ fontSize: '0.75rem', color: '#999', marginTop: '0.1rem' }}>
                {boilerConfig.temperature_sensor_kalman_parameters ? 'Enabled' : 'Disabled'}
              </div>
            </div>
            <button
              onClick={() => onNavigateToParameter('temperature_kalman')}
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

          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
            <div>
              <div style={{ fontSize: '0.9rem' }}>Pressure Kalman Filter</div>
              <div style={{ fontSize: '0.75rem', color: '#999', marginTop: '0.1rem' }}>
                {boilerConfig.pressure_sensor_kalman_parameters ? 'Enabled' : 'Disabled'}
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

      {/* Fill Configuration */}
      {boilerConfig.fill_config && (
        <ConfigurationSection title="Fill Configuration">
          <div style={{ display: 'flex', flexDirection: 'column', gap: '0.75rem' }}>
            <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.9rem' }}>
              <span style={{ color: '#666' }}>Fill Threshold:</span>
              <span style={{ fontWeight: '500' }}>
                {boilerConfig.fill_config.fill_threshold !== null && boilerConfig.fill_config.fill_threshold !== undefined
                  ? `${boilerConfig.fill_config.fill_threshold.toFixed(1)}%`
                  : 'Not set'}
              </span>
            </div>

            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
              <span style={{ fontSize: '0.9rem' }}>Pump Configuration</span>
              {boilerConfig.fill_config.pump_configuration ? (
                <button
                  onClick={() => onNavigateToParameter('fill_pump_config')}
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
              ) : (
                <button
                  onClick={() => onNavigateToParameter('fill_pump_config')}
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
              )}
            </div>
          </div>
        </ConfigurationSection>
      )}
    </div>
  );
};
