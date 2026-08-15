import { memo } from 'preact/compat';
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
      <>
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '0.5rem' }}>
          <span style={{ fontSize: '0.85rem', color: '#666' }}>Mode:</span>
          <span
            style={{
              padding: '0.2rem 0.6rem',
              backgroundColor: config.control_state.mode.type === 'Off' ? '#6c757d' : '#28a745',
              color: 'white',
              borderRadius: '10px',
              fontSize: '0.75rem',
              fontWeight: '500'
            }}
          >
            {formatMode(config.control_state.mode.type)}
          </span>
        </div>
        <div style={{ fontSize: '0.8rem', color: '#666', display: 'flex', justifyContent: 'space-between' }}>
          <span>Target:</span>
          <span>{config.control_state.values.target_temperature.toFixed(1)}°C / {config.control_state.values.target_pressure.toFixed(1)} bar</span>
        </div>
      </>
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
      <>
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '0.5rem' }}>
          <span style={{ fontSize: '0.85rem', color: '#666' }}>Mode:</span>
          <span
            style={{
              padding: '0.2rem 0.6rem',
              backgroundColor: config.brew_control_state.mode.type === 'Off' ? '#6c757d' : '#007bff',
              color: 'white',
              borderRadius: '10px',
              fontSize: '0.75rem',
              fontWeight: '500'
            }}
          >
            {modeShortNames[config.brew_control_state.mode.type] || config.brew_control_state.mode.type}
          </span>
        </div>
        <div style={{ fontSize: '0.8rem', color: '#666', display: 'flex', justifyContent: 'space-between' }}>
          <span>Auto-Tare:</span>
          <span>{config.auto_tare_enabled ? 'On' : 'Off'}</span>
        </div>
      </>
    );
  };

  const renderWaterTapInfo = (config: WaterTapConfiguration) => {
    return (
      <>
        <div style={{ fontSize: '0.8rem', color: '#666', display: 'flex', justifyContent: 'space-between', marginBottom: '0.5rem' }}>
          <span>Strategy:</span>
          <span>{config.pump_strategy.type === 'AlwaysPump' ? 'Pump' : 'Gravity'}</span>
        </div>
        <div style={{ fontSize: '0.8rem', color: '#666', display: 'flex', justifyContent: 'space-between' }}>
          <span>Target Temp:</span>
          <span>{config.temperature_target !== null && config.temperature_target !== undefined ? `${config.temperature_target.toFixed(1)}°C` : 'N/A'}</span>
        </div>
      </>
    );
  };

  const renderSteamWandInfo = (config: SteamWandConfiguration) => {
    return (
      <>
        <div style={{ fontSize: '0.8rem', color: '#666', display: 'flex', justifyContent: 'space-between', marginBottom: '0.5rem' }}>
          <span>Auto-Purge:</span>
          <span>{config.auto_purge_enabled ? 'On' : 'Off'}</span>
        </div>
        <div style={{ fontSize: '0.8rem', color: '#666', display: 'flex', justifyContent: 'space-between' }}>
          <span>Target Temp:</span>
          <span>
            {config.temperature_target !== null && config.temperature_target !== undefined ? `${config.temperature_target.toFixed(0)}°C` : 'N/A'}
          </span>
        </div>
      </>
    );
  };

  const renderTankInfo = (config: TankConfiguration) => {
    return (
      <div style={{ fontSize: '0.8rem', color: '#666', display: 'flex', justifyContent: 'space-between' }}>
        <span>Low Level Alert:</span>
        <span>{config.low_level_warning_threshold !== null && config.low_level_warning_threshold !== undefined ? `${config.low_level_warning_threshold.toFixed(1)}%` : 'N/A'}</span>
      </div>
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
        padding: '1rem',
        backgroundColor: 'white',
        border: '1px solid #ddd',
        borderRadius: '8px',
        flex: '1 1 280px',
        minWidth: '250px',
        maxWidth: '350px'
      }}
    >
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '0.75rem' }}>
        <h3 style={{ margin: 0, fontSize: '1rem', fontWeight: '600' }}>{name}</h3>
      </div>

      <div style={{ marginBottom: '0.75rem' }}>
        {renderInfo()}
      </div>

      <button
        onClick={onConfigure}
        style={{
          width: '100%',
          padding: '0.5rem',
          backgroundColor: '#0066cc',
          color: 'white',
          border: 'none',
          borderRadius: '4px',
          cursor: 'pointer',
          fontSize: '0.85rem',
          fontWeight: '500',
          marginTop: '0.5rem'
        }}
        onMouseEnter={(e) => {
          (e.target as HTMLElement).style.backgroundColor = '#0052a3';
        }}
        onMouseLeave={(e) => {
          (e.target as HTMLElement).style.backgroundColor = '#0066cc';
        }}
      >
        Configure →
      </button>
    </div>
  );
};

export const ConfigurationCard = memo(ConfigurationCardComponent);
