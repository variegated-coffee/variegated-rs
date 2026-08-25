import { Alert, Badge, Readout, ReadoutGroup, tokens } from '@variegated-coffee/ui';
import { Configuration } from '../schemas/schemas';
import {
  ConfigurationSection,
  EntityDetailHeader,
  optionalUnit,
  optionalValue,
} from './ConfigurationSection';
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
    return <Alert role="warn">No configuration for this steam wand.</Alert>;
  }

  const { temperature_target, purge_time_seconds, max_steam_time_seconds } = steamWandConfig;

  return (
    <div>
      <EntityDetailHeader name={name} index={entityKey} />

      <ConfigurationSection title="Basic settings">
        <ReadoutGroup>
          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', gap: tokens.space.sm }}>
            <span style={{ fontSize: '0.9rem', color: tokens.color.inkMuted }}>Auto-purge</span>
            <Badge role={steamWandConfig.auto_purge_enabled ? 'ok' : undefined}>
              {steamWandConfig.auto_purge_enabled ? 'Enabled' : 'Disabled'}
            </Badge>
          </div>

          <Readout
            label="Supply tank"
            value={
              steamWandConfig.supply_tank_index !== null &&
              steamWandConfig.supply_tank_index !== undefined
                ? getTankName(steamWandConfig.supply_tank_index)
                : '—'
            }
          />

          <Readout
            label="Temperature target"
            value={optionalValue(temperature_target, 1)}
            unit={optionalUnit(temperature_target, '°C')}
          />

          <Readout
            label="Purge time"
            value={optionalValue(purge_time_seconds, 1)}
            unit={optionalUnit(purge_time_seconds, 's')}
          />

          <Readout
            label="Max steam time"
            value={optionalValue(max_steam_time_seconds, 0)}
            unit={optionalUnit(max_steam_time_seconds, 's')}
          />
        </ReadoutGroup>
      </ConfigurationSection>
    </div>
  );
};
