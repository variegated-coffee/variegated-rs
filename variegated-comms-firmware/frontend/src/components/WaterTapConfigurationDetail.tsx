import { Configuration } from '../schemas/schemas';
import { ConfigurationSection } from './ConfigurationSection';
import { useMachine } from '../contexts/MachineContext';

interface WaterTapConfigurationDetailProps {
  entityKey: number;
  name: string;
  configuration: Configuration;
  onNavigateToParameter: (category: string) => void;
}

export const WaterTapConfigurationDetail = ({
  entityKey,
  name,
  configuration,
  onNavigateToParameter
}: WaterTapConfigurationDetailProps) => {
  const { getTankName } = useMachine();
  const tapConfig = configuration.water_tap_configurations.get(entityKey);

  if (!tapConfig) {
    return <div>Water tap configuration not found</div>;
  }

  const formatPumpStrategy = (strategyType: string) => {
    const strategies: Record<string, string> = {
      AlwaysPump: 'Always Pump',
      NoPump: 'No Pump (Gravity Fed)'
    };
    return strategies[strategyType] || strategyType;
  };

  return (
    <div>
      <div style={{ marginBottom: '1.5rem' }}>
        <h2 style={{ margin: 0, fontSize: '1.3rem' }}>{name}</h2>
        <div style={{ fontSize: '0.85rem', color: '#666', marginTop: '0.25rem' }}>
          Water Tap ID: {entityKey}
        </div>
      </div>

      {/* Basic Settings */}
      <ConfigurationSection title="Basic Settings">
        <div style={{ display: 'flex', flexDirection: 'column', gap: '0.75rem' }}>
          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
            <span style={{ fontSize: '0.9rem', color: '#666' }}>Pump Strategy:</span>
            <span
              style={{
                padding: '0.25rem 0.75rem',
                backgroundColor: '#007bff',
                color: 'white',
                borderRadius: '12px',
                fontSize: '0.8rem',
                fontWeight: '500'
              }}
            >
              {formatPumpStrategy(tapConfig.pump_strategy.type)}
            </span>
          </div>

          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.9rem' }}>
            <span style={{ color: '#666' }}>Supply Tank:</span>
            <span style={{ fontWeight: '500' }}>
              {tapConfig.supply_tank_index !== null && tapConfig.supply_tank_index !== undefined
                ? getTankName(tapConfig.supply_tank_index)
                : 'Not configured'}
            </span>
          </div>

          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.9rem' }}>
            <span style={{ color: '#666' }}>Temperature Target:</span>
            <span style={{ fontWeight: '500' }}>
              {tapConfig.temperature_target !== null && tapConfig.temperature_target !== undefined
                ? `${tapConfig.temperature_target.toFixed(1)}°C`
                : 'Not set'}
            </span>
          </div>

          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.9rem' }}>
            <span style={{ color: '#666' }}>Flow Rate Limit:</span>
            <span style={{ fontWeight: '500' }}>
              {tapConfig.flow_rate_limit !== null && tapConfig.flow_rate_limit !== undefined
                ? `${tapConfig.flow_rate_limit.toFixed(1)} mL/s`
                : 'Not set'}
            </span>
          </div>

          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.9rem' }}>
            <span style={{ color: '#666' }}>Max Dispense Time:</span>
            <span style={{ fontWeight: '500' }}>
              {tapConfig.max_dispense_time_seconds !== null && tapConfig.max_dispense_time_seconds !== undefined
                ? `${tapConfig.max_dispense_time_seconds}s`
                : 'Not set'}
            </span>
          </div>
        </div>
      </ConfigurationSection>

      {/* Pump Configuration */}
      <ConfigurationSection title="Pump Configuration">
        {tapConfig.pump_configuration ? (
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
