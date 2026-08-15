import { Configuration } from '../schemas/schemas';
import { ConfigurationSection } from './ConfigurationSection';
import { useMachine } from '../contexts/MachineContext';

interface SteamWandConfigurationDetailProps {
  entityKey: number;
  name: string;
  configuration: Configuration;
}

export const SteamWandConfigurationDetail = ({
  entityKey,
  name,
  configuration
}: SteamWandConfigurationDetailProps) => {
  const { getTankName } = useMachine();
  const steamWandConfig = configuration.steam_wand_configurations.get(entityKey);

  if (!steamWandConfig) {
    return <div>Steam wand configuration not found</div>;
  }

  return (
    <div>
      <div style={{ marginBottom: '1.5rem' }}>
        <h2 style={{ margin: 0, fontSize: '1.3rem' }}>{name}</h2>
        <div style={{ fontSize: '0.85rem', color: '#666', marginTop: '0.25rem' }}>
          Steam Wand ID: {entityKey}
        </div>
      </div>

      {/* Basic Settings */}
      <ConfigurationSection title="Basic Settings">
        <div style={{ display: 'flex', flexDirection: 'column', gap: '0.75rem' }}>
          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
            <span style={{ fontSize: '0.9rem', color: '#666' }}>Auto-Purge:</span>
            <span
              style={{
                padding: '0.25rem 0.75rem',
                backgroundColor: steamWandConfig.auto_purge_enabled ? '#28a745' : '#6c757d',
                color: 'white',
                borderRadius: '12px',
                fontSize: '0.8rem',
                fontWeight: '500'
              }}
            >
              {steamWandConfig.auto_purge_enabled ? 'Enabled' : 'Disabled'}
            </span>
          </div>

          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.9rem' }}>
            <span style={{ color: '#666' }}>Supply Tank:</span>
            <span style={{ fontWeight: '500' }}>
              {steamWandConfig.supply_tank_index !== null && steamWandConfig.supply_tank_index !== undefined
                ? getTankName(steamWandConfig.supply_tank_index)
                : 'Not configured'}
            </span>
          </div>

          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.9rem' }}>
            <span style={{ color: '#666' }}>Temperature Target:</span>
            <span style={{ fontWeight: '500' }}>
              {steamWandConfig.temperature_target !== null && steamWandConfig.temperature_target !== undefined
                ? `${steamWandConfig.temperature_target.toFixed(1)}°C`
                : 'Not set'}
            </span>
          </div>

          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.9rem' }}>
            <span style={{ color: '#666' }}>Purge Time:</span>
            <span style={{ fontWeight: '500' }}>
              {steamWandConfig.purge_time_seconds !== null && steamWandConfig.purge_time_seconds !== undefined
                ? `${steamWandConfig.purge_time_seconds.toFixed(1)}s`
                : 'Not set'}
            </span>
          </div>

          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.9rem' }}>
            <span style={{ color: '#666' }}>Max Steam Time:</span>
            <span style={{ fontWeight: '500' }}>
              {steamWandConfig.max_steam_time_seconds !== null && steamWandConfig.max_steam_time_seconds !== undefined
                ? `${steamWandConfig.max_steam_time_seconds}s`
                : 'Not set'}
            </span>
          </div>
        </div>
      </ConfigurationSection>
    </div>
  );
};
