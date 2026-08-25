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
    return <Alert role="warn">No configuration for this boiler.</Alert>;
  }

  const formatControlMode = (modeType: string) => {
    const modes: Record<string, string> = {
      Temperature: 'Temperature control',
      Pressure: 'Pressure control',
      Off: 'Off'
    };
    return modes[modeType] || modeType;
  };

  const { minimum_safe_level, max_temperature, max_pressure } = boilerConfig;
  const fill = boilerConfig.fill_config;

  return (
    <div>
      <EntityDetailHeader name={name} index={entityKey} />

      <ConfigurationSection title="Control state">
        <div style={{ display: 'flex', flexDirection: 'column', gap: tokens.space.sm }}>
          <ReadoutGroup>
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', gap: tokens.space.sm }}>
              <span style={{ fontSize: '0.9rem', color: tokens.color.inkMuted }}>Control mode</span>
              <Badge role={boilerConfig.control_state.mode.type === 'Off' ? undefined : 'ok'}>
                {formatControlMode(boilerConfig.control_state.mode.type)}
              </Badge>
            </div>
            <Readout
              label="Target temperature"
              value={boilerConfig.control_state.values.target_temperature.toFixed(1)}
              unit="°C"
            />
            <Readout
              label="Target pressure"
              value={boilerConfig.control_state.values.target_pressure.toFixed(2)}
              unit="bar"
            />
          </ReadoutGroup>

          <SettingRow
            title="Control settings"
            description="Mode and the targets it holds"
            onEdit={() => onNavigateToParameter('boiler_control')}
          />
        </div>
      </ConfigurationSection>

      {/* These are the limits that stop a boiler doing damage, so they read as a group of
          their own rather than as four more rows of settings. */}
      <ConfigurationSection title="Safety limits">
        <ReadoutGroup>
          <Readout
            label="Supply tank"
            value={
              boilerConfig.supply_tank_index !== null && boilerConfig.supply_tank_index !== undefined
                ? getTankName(boilerConfig.supply_tank_index)
                : '—'
            }
          />
          <Readout
            label="Minimum safe level"
            value={optionalValue(minimum_safe_level, 1)}
            unit={optionalUnit(minimum_safe_level, '%')}
          />
          <Readout
            label="Max temperature"
            value={optionalValue(max_temperature, 1)}
            unit={optionalUnit(max_temperature, '°C')}
          />
          <Readout
            label="Max pressure"
            value={optionalValue(max_pressure, 2)}
            unit={optionalUnit(max_pressure, 'bar')}
          />
        </ReadoutGroup>
      </ConfigurationSection>

      <ConfigurationSection title="PID controllers">
        <div style={{ display: 'flex', flexDirection: 'column', gap: tokens.space.sm }}>
          <SettingRow
            title="Temperature PID"
            description="Gains and limits for the temperature loop"
            onEdit={() => onNavigateToParameter('temperature_pid')}
          />
          <SettingRow
            title="Pressure PID"
            description="Gains and limits for the pressure loop"
            onEdit={() => onNavigateToParameter('pressure_pid')}
          />
        </div>
      </ConfigurationSection>

      <ConfigurationSection title="Sensor filtering">
        <div style={{ display: 'flex', flexDirection: 'column', gap: tokens.space.sm }}>
          <SettingRow
            title="Temperature Kalman filter"
            description="Smooths the temperature reading before the loop sees it"
            configured={
              boilerConfig.temperature_sensor_kalman_parameters !== null &&
              boilerConfig.temperature_sensor_kalman_parameters !== undefined
            }
            onEdit={() => onNavigateToParameter('temperature_kalman')}
          />
          <SettingRow
            title="Pressure Kalman filter"
            description="Smooths the pressure reading before the loop sees it"
            configured={
              boilerConfig.pressure_sensor_kalman_parameters !== null &&
              boilerConfig.pressure_sensor_kalman_parameters !== undefined
            }
            onEdit={() => onNavigateToParameter('pressure_kalman')}
          />
        </div>
      </ConfigurationSection>

      {fill && (
        <ConfigurationSection title="Fill">
          <div style={{ display: 'flex', flexDirection: 'column', gap: tokens.space.sm }}>
            <ReadoutGroup>
              <Readout
                label="Fill threshold"
                value={optionalValue(fill.fill_threshold, 1)}
                unit={optionalUnit(fill.fill_threshold, '%')}
              />
            </ReadoutGroup>
            <SettingRow
              title="Fill pump"
              description="Duty cycle, ramp times, flow sensor"
              configured={
                fill.pump_configuration !== null && fill.pump_configuration !== undefined
              }
              onEdit={() => onNavigateToParameter('fill_pump_config')}
            />
          </div>
        </ConfigurationSection>
      )}
    </div>
  );
};
