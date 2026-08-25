import { Alert, Badge, Reading, ReadingGroup, tokens } from '@variegated-coffee/ui';
import { Configuration } from '../schemas/schemas';
import {
  ConfigurationSection,
  EntityDetailHeader,
  SettingRow,
  optionalUnit,
  optionalValue,
} from './ConfigurationSection';
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
    return <Alert role="warn">No configuration for this group.</Alert>;
  }

  const formatControlMode = (mode: string) => {
    const modes: Record<string, string> = {
      GroupFlowRate: 'Group flow rate',
      GroupFlowRateCurve: 'Group flow rate curve',
      Pressure: 'Pressure',
      PressureCurve: 'Pressure curve',
      OutputFlowRate: 'Output flow rate',
      OutputFlowRateCurve: 'Output flow rate curve',
      FixedDutyCycle: 'Fixed duty cycle',
      FixedDutyCycleCurve: 'Fixed duty cycle curve',
      FullOn: 'Full on',
      Off: 'Off'
    };
    return modes[mode] || mode;
  };

  /**
   * The target, split into a figure and its unit.
   *
   * Returned as a pair rather than a formatted string so `Reading` can align a column of
   * these on the number. The curve and on/off modes have no figure at all, which is why
   * `unit` is optional rather than always present.
   */
  const targetForMode = (): { value: string; unit?: string } => {
    const mode = groupConfig.brew_control_state.mode;
    const values = groupConfig.brew_control_state.values;

    switch (mode.type) {
      case 'GroupFlowRate':
        return { value: values.flow_rate.toFixed(1), unit: 'mL/s' };
      case 'Pressure':
        return { value: values.pressure.toFixed(2), unit: 'bar' };
      case 'OutputFlowRate':
        return { value: values.output_flow_rate.toFixed(1), unit: 'mL/s' };
      case 'FixedDutyCycle':
        return { value: values.duty_cycle.toFixed(1), unit: '%' };
      case 'GroupFlowRateCurve':
      case 'PressureCurve':
      case 'OutputFlowRateCurve':
      case 'FixedDutyCycleCurve':
        return { value: 'From curve' };
      case 'FullOn':
        return { value: '100', unit: '%' };
      case 'Off':
        return { value: 'Off' };
      default:
        return { value: '—' };
    }
  };

  const target = targetForMode();
  const { max_brew_time_seconds, flow_sensor_pulses_per_liter } = groupConfig;

  return (
    <div>
      <EntityDetailHeader name={name} index={entityKey} />

      <ConfigurationSection title="Basic settings">
        <ReadingGroup>
          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', gap: tokens.space.sm }}>
            <span style={{ fontSize: '0.9rem', color: tokens.color.inkMuted }}>Auto-tare scale</span>
            <Badge role={groupConfig.auto_tare_enabled ? 'ok' : undefined}>
              {groupConfig.auto_tare_enabled ? 'Enabled' : 'Disabled'}
            </Badge>
          </div>

          <Reading
            label="Supply tank"
            value={
              groupConfig.supply_tank_index !== null && groupConfig.supply_tank_index !== undefined
                ? getTankName(groupConfig.supply_tank_index)
                : '—'
            }
          />

          <Reading
            label="Max brew time"
            value={optionalValue(max_brew_time_seconds, 0)}
            unit={optionalUnit(max_brew_time_seconds, 's')}
          />

          <Reading
            label="Flow sensor calibration"
            value={optionalValue(flow_sensor_pulses_per_liter, 0)}
            unit={optionalUnit(flow_sensor_pulses_per_liter, 'pulses/L')}
          />
        </ReadingGroup>
      </ConfigurationSection>

      <ConfigurationSection title="Control state">
        <div style={{ display: 'flex', flexDirection: 'column', gap: tokens.space.sm }}>
          <ReadingGroup>
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', gap: tokens.space.sm }}>
              <span style={{ fontSize: '0.9rem', color: tokens.color.inkMuted }}>Control mode</span>
              <Badge role={groupConfig.brew_control_state.mode.type === 'Off' ? undefined : 'ok'}>
                {formatControlMode(groupConfig.brew_control_state.mode.type)}
              </Badge>
            </div>
            <Reading label="Current target" value={target.value} unit={target.unit} />
          </ReadingGroup>

          <SettingRow
            title="Control settings"
            description="Mode and the target it holds"
            onEdit={() => onNavigateToParameter('group_control')}
          />
        </div>
      </ConfigurationSection>

      <ConfigurationSection title="PID controllers">
        <div style={{ display: 'flex', flexDirection: 'column', gap: tokens.space.sm }}>
          <SettingRow
            title="Group flow rate PID"
            description="Gains and limits for flow into the puck"
            onEdit={() => onNavigateToParameter('flow_rate_pid')}
          />
          <SettingRow
            title="Output flow rate PID"
            description="Gains and limits for flow out of the puck"
            onEdit={() => onNavigateToParameter('output_flow_rate_pid')}
          />
          <SettingRow
            title="Pressure PID"
            description="Gains and limits for the pressure loop"
            onEdit={() => onNavigateToParameter('pressure_pid')}
          />
        </div>
      </ConfigurationSection>

      <ConfigurationSection title="Control curves">
        <div style={{ display: 'flex', flexDirection: 'column', gap: tokens.space.sm }}>
          <SettingRow
            title="Flow rate curve"
            description="Target flow over the course of a shot"
            onEdit={() => onNavigateToParameter('flow_rate_curve')}
          />
          <SettingRow
            title="Output flow rate curve"
            description="Target output flow over the course of a shot"
            onEdit={() => onNavigateToParameter('output_flow_rate_curve')}
          />
          <SettingRow
            title="Pressure curve"
            description="Target pressure over the course of a shot"
            onEdit={() => onNavigateToParameter('pressure_curve')}
          />
          <SettingRow
            title="Duty cycle curve"
            description="Pump duty over the course of a shot"
            onEdit={() => onNavigateToParameter('duty_cycle_curve')}
          />
        </div>
      </ConfigurationSection>

      <ConfigurationSection title="Sensor filtering">
        <SettingRow
          title="Pressure Kalman filter"
          description="Smooths the pressure reading before the loop sees it"
          configured={
            groupConfig.pressure_sensor_kalman_parameters !== null &&
            groupConfig.pressure_sensor_kalman_parameters !== undefined
          }
          onEdit={() => onNavigateToParameter('pressure_kalman')}
        />
      </ConfigurationSection>

      <ConfigurationSection title="Pump">
        <SettingRow
          title="Pump parameters"
          description="Duty cycle, ramp times, flow sensor"
          configured={
            groupConfig.pump_configuration !== null && groupConfig.pump_configuration !== undefined
          }
          onEdit={() => onNavigateToParameter('pump_config')}
        />
      </ConfigurationSection>
    </div>
  );
};
