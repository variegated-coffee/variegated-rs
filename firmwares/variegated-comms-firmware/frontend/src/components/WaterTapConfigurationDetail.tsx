import { Alert, Badge, Readout, ReadoutGroup, tokens } from '@variegated-coffee/ui';
import { Configuration } from '../schemas/schemas';
import {
  ConfigurationSection,
  EntityDetailHeader,
  SettingRow,
  optionalUnit,
  optionalValue,
} from './ConfigurationSection';
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
    return <Alert role="warn">No configuration for this water tap.</Alert>;
  }

  const formatPumpStrategy = (strategyType: string) => {
    const strategies: Record<string, string> = {
      AlwaysPump: 'Always pump',
      NoPump: 'No pump (gravity fed)'
    };
    return strategies[strategyType] || strategyType;
  };

  const { temperature_target, flow_rate_limit, max_dispense_time_seconds } = tapConfig;

  return (
    <div>
      <EntityDetailHeader name={name} index={entityKey} />

      <ConfigurationSection title="Basic settings">
        <ReadoutGroup>
          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', gap: tokens.space.sm }}>
            <span style={{ fontSize: '0.9rem', color: tokens.color.inkMuted }}>Pump strategy</span>
            {/* Role-less: which strategy a tap uses is a configuration choice, not a
                status. It was a filled blue pill, which read as "this one is active". */}
            <Badge>{formatPumpStrategy(tapConfig.pump_strategy.type)}</Badge>
          </div>

          <Readout
            label="Supply tank"
            value={
              tapConfig.supply_tank_index !== null && tapConfig.supply_tank_index !== undefined
                ? getTankName(tapConfig.supply_tank_index)
                : '—'
            }
          />

          <Readout
            label="Temperature target"
            value={optionalValue(temperature_target, 1)}
            unit={optionalUnit(temperature_target, '°C')}
          />

          <Readout
            label="Flow rate limit"
            value={optionalValue(flow_rate_limit, 1)}
            unit={optionalUnit(flow_rate_limit, 'mL/s')}
          />

          <Readout
            label="Max dispense time"
            value={optionalValue(max_dispense_time_seconds, 0)}
            unit={optionalUnit(max_dispense_time_seconds, 's')}
          />
        </ReadoutGroup>
      </ConfigurationSection>

      <ConfigurationSection title="Pump">
        <SettingRow
          title="Pump parameters"
          description="Duty cycle, ramp times, flow sensor"
          configured={tapConfig.pump_configuration !== null && tapConfig.pump_configuration !== undefined}
          onEdit={() => onNavigateToParameter('pump_config')}
        />
      </ConfigurationSection>
    </div>
  );
};
