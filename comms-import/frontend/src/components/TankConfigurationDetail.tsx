import { Configuration } from '../schemas/schemas';
import { ConfigurationSection } from './ConfigurationSection';

interface TankConfigurationDetailProps {
  entityKey: number;
  name: string;
  configuration: Configuration;
  onNavigateToParameter: (category: string) => void;
}

export const TankConfigurationDetail = ({
  entityKey,
  name,
  configuration,
  onNavigateToParameter
}: TankConfigurationDetailProps) => {
  const tankConfig = configuration.tank_configurations.get(entityKey);

  if (!tankConfig) {
    return <div>Tank configuration not found</div>;
  }

  return (
    <div>
      <div style={{ marginBottom: '1.5rem' }}>
        <h2 style={{ margin: 0, fontSize: '1.3rem' }}>{name}</h2>
        <div style={{ fontSize: '0.85rem', color: '#666', marginTop: '0.25rem' }}>
          Tank ID: {entityKey}
        </div>
      </div>

      {/* Basic Settings */}
      <ConfigurationSection title="Basic Settings">
        <div style={{ display: 'flex', flexDirection: 'column', gap: '0.75rem' }}>
          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.9rem' }}>
            <span style={{ color: '#666' }}>Low Level Warning Threshold:</span>
            <span style={{ fontWeight: '500' }}>
              {tankConfig.low_level_warning_threshold !== null && tankConfig.low_level_warning_threshold !== undefined
                ? `${tankConfig.low_level_warning_threshold.toFixed(1)}%`
                : 'Not set'}
            </span>
          </div>
        </div>
      </ConfigurationSection>

      {/* Sensor Filtering */}
      <ConfigurationSection title="Sensor Filtering">
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
          <div>
            <div style={{ fontSize: '0.9rem' }}>Water Level Kalman Filter</div>
            <div style={{ fontSize: '0.75rem', color: '#999', marginTop: '0.1rem' }}>
              {tankConfig.water_level_sensor_kalman_parameters ? 'Enabled' : 'Disabled'}
            </div>
          </div>
          <button
            onClick={() => onNavigateToParameter('water_level_kalman')}
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
      </ConfigurationSection>
    </div>
  );
};
