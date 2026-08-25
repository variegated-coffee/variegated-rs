import { memo } from 'preact/compat';
import { Badge, Button, Readout, ReadoutGroup, tokens } from '@variegated-coffee/ui';
import {
  BoilerConfiguration,
  GroupConfiguration,
  WaterTapConfiguration,
  SteamWandConfiguration,
  TankConfiguration
} from '../schemas/schemas';

type EntityConfig =
  | BoilerConfiguration
  | GroupConfiguration
  | WaterTapConfiguration
  | SteamWandConfiguration
  | TankConfiguration;

interface ConfigurationCardProps {
  name: string;
  entityType: 'boiler' | 'group' | 'water_tap' | 'steam_wand' | 'tank';
  configuration: EntityConfig;
  onConfigure: () => void;
}

const ConfigurationCardComponent = ({ name, entityType, configuration, onConfigure }: ConfigurationCardProps) => {
  const renderBoilerInfo = (config: BoilerConfiguration) => {
    const formatMode = (modeType: string) => {
      return modeType === 'Temperature' ? 'Temp' : modeType === 'Pressure' ? 'Press' : 'Off';
    };

    return (
      <ReadoutGroup>
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', gap: tokens.space.sm }}>
          <span style={{ fontSize: '0.9rem', color: tokens.color.inkMuted }}>Mode</span>
          <Badge role={config.control_state.mode.type === 'Off' ? undefined : 'ok'}>
            {formatMode(config.control_state.mode.type)}
          </Badge>
        </div>
        {/* Two targets, two rows. They were one string joined by a slash, so neither
            number could line up with anything and the units read as part of the value. */}
        <Readout
          label="Target temperature"
          value={config.control_state.values.target_temperature.toFixed(1)}
          unit="°C"
        />
        <Readout
          label="Target pressure"
          value={config.control_state.values.target_pressure.toFixed(1)}
          unit="bar"
        />
      </ReadoutGroup>
    );
  };

  const renderGroupInfo = (config: GroupConfiguration) => {
    const modeShortNames: Record<string, string> = {
      GroupFlowRate: 'Flow',
      GroupFlowRateCurve: 'Flow ○',
      Pressure: 'Press',
      PressureCurve: 'Press ○',
      OutputFlowRate: 'Out Flow',
      OutputFlowRateCurve: 'Out Flow ○',
      FixedDutyCycle: 'Duty',
      FixedDutyCycleCurve: 'Duty ○',
      FullOn: 'Full',
      Off: 'Off'
    };

    return (
      <ReadoutGroup>
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', gap: tokens.space.sm }}>
          <span style={{ fontSize: '0.9rem', color: tokens.color.inkMuted }}>Mode</span>
          <Badge role={config.brew_control_state.mode.type === 'Off' ? undefined : 'ok'}>
            {modeShortNames[config.brew_control_state.mode.type] || config.brew_control_state.mode.type}
          </Badge>
        </div>
        <Readout label="Auto-tare" value={config.auto_tare_enabled ? 'On' : 'Off'} />
      </ReadoutGroup>
    );
  };

  /**
   * An optional measurement, shown as an em dash when the machine has none.
   *
   * "N/A" was the old spelling, in four places. A dash reads as "no value" without
   * claiming that a value was looked up and found inapplicable, which is not what a null
   * target means here -- it means nothing has been set.
   */
  const optional = (value: number | null | undefined, decimals: number) =>
    value !== null && value !== undefined ? value.toFixed(decimals) : '—';

  const renderWaterTapInfo = (config: WaterTapConfiguration) => {
    const target = config.temperature_target;
    return (
      <ReadoutGroup>
        <Readout
          label="Strategy"
          value={config.pump_strategy.type === 'AlwaysPump' ? 'Pump' : 'Gravity'}
        />
        <Readout
          label="Target temperature"
          value={optional(target, 1)}
          unit={target !== null && target !== undefined ? '°C' : undefined}
        />
      </ReadoutGroup>
    );
  };

  const renderSteamWandInfo = (config: SteamWandConfiguration) => {
    const target = config.temperature_target;
    return (
      <ReadoutGroup>
        <Readout label="Auto-purge" value={config.auto_purge_enabled ? 'On' : 'Off'} />
        <Readout
          label="Target temperature"
          value={optional(target, 0)}
          unit={target !== null && target !== undefined ? '°C' : undefined}
        />
      </ReadoutGroup>
    );
  };

  const renderTankInfo = (config: TankConfiguration) => {
    const threshold = config.low_level_warning_threshold;
    return (
      <ReadoutGroup>
        <Readout
          label="Low level alert"
          value={optional(threshold, 1)}
          unit={threshold !== null && threshold !== undefined ? '%' : undefined}
        />
      </ReadoutGroup>
    );
  };

  const renderInfo = () => {
    switch (entityType) {
      case 'boiler':
        return renderBoilerInfo(configuration as BoilerConfiguration);
      case 'group':
        return renderGroupInfo(configuration as GroupConfiguration);
      case 'water_tap':
        return renderWaterTapInfo(configuration as WaterTapConfiguration);
      case 'steam_wand':
        return renderSteamWandInfo(configuration as SteamWandConfiguration);
      case 'tank':
        return renderTankInfo(configuration as TankConfiguration);
      default:
        return null;
    }
  };

  return (
    <div
      style={{
        display: 'flex',
        flexDirection: 'column',
        gap: tokens.space.sm,
        padding: tokens.space.md,
        backgroundColor: tokens.color.surfaceRaised,
        border: `1px solid ${tokens.color.border}`,
        borderRadius: tokens.radius.md,
        flex: '1 1 280px',
        minWidth: '250px',
        maxWidth: '350px',
      }}
    >
      <h3 style={{ margin: 0, fontSize: '1rem', fontWeight: 600 }}>{name}</h3>

      {renderInfo()}

      {/* Secondary, not primary. One of these cards per entity means five or six filled
          blue buttons stacked down the configuration screen, all equally loud, none of
          them the thing the screen is for. The hand-rolled hover handlers go with it --
          `Button` owns that now, and owned it inconsistently before: this was the only
          button in the frontend that had one. */}
      <Button variant="secondary" block onClick={onConfigure} ariaLabel={`Configure ${name}`}>
        Configure
      </Button>
    </div>
  );
};

export const ConfigurationCard = memo(ConfigurationCardComponent);
