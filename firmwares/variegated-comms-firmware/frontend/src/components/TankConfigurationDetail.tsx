import { Alert, Reading, ReadingGroup } from '@variegated-coffee/ui';
import { Configuration } from '../schemas/schemas';
import {
  ConfigurationSection,
  EntityDetailHeader,
  SettingRow,
  optionalUnit,
  optionalValue,
} from './ConfigurationSection';

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
    return <Alert role="warn">No configuration for this tank.</Alert>;
  }

  const threshold = tankConfig.low_level_warning_threshold;

  return (
    <div>
      <EntityDetailHeader name={name} index={entityKey} />

      <ConfigurationSection title="Basic settings">
        <ReadingGroup>
          <Reading
            label="Low level warning threshold"
            value={optionalValue(threshold, 1)}
            unit={optionalUnit(threshold, '%')}
          />
        </ReadingGroup>
      </ConfigurationSection>

      <ConfigurationSection title="Sensor filtering">
        <SettingRow
          title="Water level Kalman filter"
          description="Smooths the level reading, which is noisy while water is moving"
          configured={
            tankConfig.water_level_sensor_kalman_parameters !== null &&
            tankConfig.water_level_sensor_kalman_parameters !== undefined
          }
          onEdit={() => onNavigateToParameter('water_level_kalman')}
        />
      </ConfigurationSection>
    </div>
  );
};
