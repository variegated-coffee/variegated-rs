import { useState } from 'preact/hooks';
import { memo } from 'preact/compat';
import {
  Configuration,
  SetBoilerControlRequest,
  SetBoilerControlRequestSchema,
  SetGroupControlRequest,
  SetGroupControlRequestSchema,
  SetPidParametersRequest,
  SetPidParametersRequestSchema,
  SetGroupPumpConfigurationRequest,
  SetGroupPumpConfigurationRequestSchema,
  SetWaterTapPumpConfigurationRequest,
  SetWaterTapPumpConfigurationRequestSchema,
  SetFillPumpConfigurationRequest,
  SetFillPumpConfigurationRequestSchema,
  PidParameters_for_float,
  PumpConfiguration,
  BoilerControlMode,
  GroupBrewControlMode,
  ControlCurve,
  KalmanParameters
} from '../schemas/schemas';
import { useMachine } from '../contexts/MachineContext';
import { ConfigurationCard } from './ConfigurationCard';
import { BoilerConfigurationDetail } from './BoilerConfigurationDetail';
import { GroupConfigurationDetail } from './GroupConfigurationDetail';
import { WaterTapConfigurationDetail } from './WaterTapConfigurationDetail';
import { SteamWandConfigurationDetail } from './SteamWandConfigurationDetail';
import { TankConfigurationDetail } from './TankConfigurationDetail';
import { PidParametersEditor } from './PidParametersEditor';
import { KalmanFilterEditor } from './KalmanFilterEditor';
import { PumpConfigurationEditor } from './PumpConfigurationEditor';
import { ControlCurveEditor } from './ControlCurveEditor';
import { BoilerControlEditor } from './BoilerControlEditor';
import { GroupControlEditor } from './GroupControlEditor';
import { postPostcard } from '../utils/postcard';

interface ConfigurationPanelProps {
  configuration: Configuration;
}

type EntityType = 'boilers' | 'groups' | 'water_taps' | 'steam_wands' | 'tanks' | 'machine';
type NavigationLevel = 1 | 2 | 3;

interface NavigationState {
  level: NavigationLevel;
  entityType: EntityType | null;
  entityKey: number | null;
  parameterCategory: string | null;
}

const ConfigurationPanelComponent = ({ configuration }: ConfigurationPanelProps) => {
  const machineContext = useMachine();
  const { getBoilerEntries, getGroupEntries, getWaterTapEntries } = machineContext;
  const [isExpanded, setIsExpanded] = useState(false);
  const [navigation, setNavigation] = useState<NavigationState>({
    level: 1,
    entityType: null,
    entityKey: null,
    parameterCategory: null
  });
  const [optimizeMessage, setOptimizeMessage] = useState<{ type: 'success' | 'error', text: string } | null>(null);

  // Navigate to entity detail (level 2)
  const navigateToEntity = (type: EntityType, key: number) => {
    setNavigation({
      level: 2,
      entityType: type,
      entityKey: key,
      parameterCategory: null
    });
  };

  // Navigate to parameter editor (level 3)
  const navigateToParameter = (category: string) => {
    setNavigation({
      ...navigation,
      level: 3,
      parameterCategory: category
    });
  };

  // Go back one level
  const goBack = () => {
    if (navigation.level === 3) {
      setNavigation({
        ...navigation,
        level: 2,
        parameterCategory: null
      });
    } else if (navigation.level === 2) {
      setNavigation({
        level: 1,
        entityType: null,
        entityKey: null,
        parameterCategory: null
      });
    }
  };

  // Optimize storage handler
  const handleOptimizeStorage = async () => {
    setOptimizeMessage(null);
    try {
      const response = await fetch('/command/optimize-configuration-storage', {
        method: 'POST',
      });

      if (!response.ok) {
        throw new Error(`Failed to optimize storage: ${response.statusText}`);
      }

      setOptimizeMessage({ type: 'success', text: 'Storage optimized successfully' });
      setTimeout(() => setOptimizeMessage(null), 2000);
    } catch (err) {
      const errorMsg = err instanceof Error ? err.message : 'Failed to optimize storage';
      setOptimizeMessage({ type: 'error', text: errorMsg });
      setTimeout(() => setOptimizeMessage(null), 4000);
    }
  };

  // Breadcrumb display
  const renderBreadcrumb = () => {
    const parts: string[] = ['Configuration'];

    if (navigation.entityType && navigation.entityKey) {
      const typeNames: Record<EntityType, string> = {
        boilers: 'Boilers',
        groups: 'Groups',
        water_taps: 'Water Taps',
        steam_wands: 'Steam Wands',
        tanks: 'Tanks',
        machine: 'Machine'
      };
      parts.push(typeNames[navigation.entityType]);

      // Get entity name
      let entityName = String(navigation.entityKey);
      if (navigation.entityType === 'boilers') {
        const boilerDef = getBoilerEntries().find(([key]) => key === navigation.entityKey);
        if (boilerDef) entityName = boilerDef[1].name;
      } else if (navigation.entityType === 'groups') {
        const groupDef = getGroupEntries().find(([key]) => key === navigation.entityKey);
        if (groupDef) entityName = groupDef[1].name;
      } else if (navigation.entityType === 'water_taps') {
        const tapDef = getWaterTapEntries().find(([key]) => key === navigation.entityKey);
        if (tapDef) entityName = tapDef[1].name;
      }

      parts.push(entityName);

      if (navigation.parameterCategory) {
        const categoryNames: Record<string, string> = {
          temperature_pid: 'Temperature PID',
          pressure_pid: 'Pressure PID',
          flow_rate_pid: 'Flow Rate PID',
          output_flow_rate_pid: 'Output Flow Rate PID',
          temperature_kalman: 'Temperature Kalman Filter',
          pressure_kalman: 'Pressure Kalman Filter',
          water_level_kalman: 'Water Level Kalman Filter',
          pump_config: 'Pump Configuration',
          fill_pump_config: 'Fill Pump Configuration',
          pressure_curve: 'Pressure Curve',
          flow_rate_curve: 'Flow Rate Curve',
          output_flow_rate_curve: 'Output Flow Rate Curve',
          duty_cycle_curve: 'Duty Cycle Curve',
          boiler_control: 'Boiler Control',
          group_control: 'Group Control'
        };
        parts.push(categoryNames[navigation.parameterCategory] || navigation.parameterCategory);
      }
    }

    return (
      <div style={{ fontSize: '0.9rem', color: '#666', marginBottom: '1rem' }}>
        {parts.join(' > ')}
      </div>
    );
  };

  // Level 1: Dashboard with all entities
  const renderLevel1 = () => {
    const boilerEntries = getBoilerEntries();
    const groupEntries = getGroupEntries();
    const waterTapEntries = getWaterTapEntries();
    const steamWandEntries = Array.from(configuration.steam_wand_configurations.entries());
    const tankEntries = Array.from(configuration.tank_configurations.entries());

    return (
      <div>
        {/* Machine Configuration Summary */}
        <div
          style={{
            marginBottom: '1.5rem',
            padding: '1rem',
            backgroundColor: '#f8f9fa',
            borderRadius: '8px',
            border: '1px solid #ddd'
          }}
        >
          <h3 style={{ marginTop: 0, marginBottom: '0.75rem', fontSize: '1.1rem' }}>Machine Configuration</h3>
          <div style={{ display: 'flex', alignItems: 'center', gap: '0.5rem', fontSize: '0.9rem' }}>
            <label style={{ fontWeight: '500' }}>Heating Element Interlock:</label>
            <span
              style={{
                padding: '0.25rem 0.75rem',
                backgroundColor: configuration.machine_config.heating_element_interlock ? '#28a745' : '#6c757d',
                color: 'white',
                borderRadius: '12px',
                fontSize: '0.75rem',
                fontWeight: '500'
              }}
            >
              {configuration.machine_config.heating_element_interlock ? 'Enabled' : 'Disabled'}
            </span>
          </div>
        </div>

        {/* Boilers */}
        {boilerEntries.length > 0 && (
          <div style={{ marginBottom: '1.5rem' }}>
            <h3 style={{ marginTop: 0, marginBottom: '0.75rem', fontSize: '1.1rem', color: '#333' }}>
              Boilers ({boilerEntries.length})
            </h3>
            <div style={{ display: 'flex', gap: '1rem', flexWrap: 'wrap' }}>
              {boilerEntries.map(([key, def]) => {
                const entityConfig = configuration.boiler_configurations.get(key);
                if (!entityConfig) return null;
                return (
                  <ConfigurationCard
                    key={key}
                    name={def.name}
                    entityType="boiler"
                    configuration={entityConfig}
                    onConfigure={() => navigateToEntity('boilers', key)}
                  />
                );
              })}
            </div>
          </div>
        )}

        {/* Groups */}
        {groupEntries.length > 0 && (
          <div style={{ marginBottom: '1.5rem' }}>
            <h3 style={{ marginTop: 0, marginBottom: '0.75rem', fontSize: '1.1rem', color: '#333' }}>
              Groups ({groupEntries.length})
            </h3>
            <div style={{ display: 'flex', gap: '1rem', flexWrap: 'wrap' }}>
              {groupEntries.map(([key, def]) => {
                const entityConfig = configuration.group_configurations.get(key);
                if (!entityConfig) return null;
                return (
                  <ConfigurationCard
                    key={key}
                    name={def.name}
                    entityType="group"
                    configuration={entityConfig}
                    onConfigure={() => navigateToEntity('groups', key)}
                  />
                );
              })}
            </div>
          </div>
        )}

        {/* Water Taps */}
        {waterTapEntries.length > 0 && (
          <div style={{ marginBottom: '1.5rem' }}>
            <h3 style={{ marginTop: 0, marginBottom: '0.75rem', fontSize: '1.1rem', color: '#333' }}>
              Water Taps ({waterTapEntries.length})
            </h3>
            <div style={{ display: 'flex', gap: '1rem', flexWrap: 'wrap' }}>
              {waterTapEntries.map(([key, def]) => {
                const entityConfig = configuration.water_tap_configurations.get(key);
                if (!entityConfig) return null;
                return (
                  <ConfigurationCard
                    key={key}
                    name={def.name}
                    entityType="water_tap"
                    configuration={entityConfig}
                    onConfigure={() => navigateToEntity('water_taps', key)}
                  />
                );
              })}
            </div>
          </div>
        )}

        {/* Steam Wands */}
        {steamWandEntries.length > 0 && (
          <div style={{ marginBottom: '1.5rem' }}>
            <h3 style={{ marginTop: 0, marginBottom: '0.75rem', fontSize: '1.1rem', color: '#333' }}>
              Steam Wands ({steamWandEntries.length})
            </h3>
            <div style={{ display: 'flex', gap: '1rem', flexWrap: 'wrap' }}>
              {steamWandEntries.map(([key, config]) => (
                <ConfigurationCard
                  key={key}
                  name={String(key)}
                  entityType="steam_wand"
                  configuration={config}
                  onConfigure={() => navigateToEntity('steam_wands', key)}
                />
              ))}
            </div>
          </div>
        )}

        {/* Tanks */}
        {tankEntries.length > 0 && (
          <div style={{ marginBottom: '1.5rem' }}>
            <h3 style={{ marginTop: 0, marginBottom: '0.75rem', fontSize: '1.1rem', color: '#333' }}>
              Tanks ({tankEntries.length})
            </h3>
            <div style={{ display: 'flex', gap: '1rem', flexWrap: 'wrap' }}>
              {tankEntries.map(([key, config]) => (
                <ConfigurationCard
                  key={key}
                  name={String(key)}
                  entityType="tank"
                  configuration={config}
                  onConfigure={() => navigateToEntity('tanks', key)}
                />
              ))}
            </div>
          </div>
        )}
      </div>
    );
  };

  // Level 2: Entity Detail
  const renderLevel2 = () => {
    if (!navigation.entityType || navigation.entityKey === null) return null;

    let entityName = String(navigation.entityKey);
    let detailView = null;

    switch (navigation.entityType) {
      case 'boilers': {
        const boilerDef = getBoilerEntries().find(([key]) => key === navigation.entityKey);
        if (boilerDef) entityName = boilerDef[1].name;
        // entityConfig is available via configuration.boiler_configurations.get(navigation.entityKey)
        if (configuration) {
          detailView = (
            <BoilerConfigurationDetail
              entityKey={navigation.entityKey}
              name={entityName}
              configuration={configuration}
              onNavigateToParameter={navigateToParameter}
            />
          );
        }
        break;
      }
      case 'groups': {
        const groupDef = getGroupEntries().find(([key]) => key === navigation.entityKey);
        if (groupDef) entityName = groupDef[1].name;
        // entityConfig is available via configuration.group_configurations.get(navigation.entityKey)
        if (configuration) {
          detailView = (
            <GroupConfigurationDetail
              entityKey={navigation.entityKey}
              name={entityName}
              configuration={configuration}
              onNavigateToParameter={navigateToParameter}
            />
          );
        }
        break;
      }
      case 'water_taps': {
        const tapDef = getWaterTapEntries().find(([key]) => key === navigation.entityKey);
        if (tapDef) entityName = tapDef[1].name;
        // entityConfig is available via configuration.water_tap_configurations.get(navigation.entityKey)
        if (configuration) {
          detailView = (
            <WaterTapConfigurationDetail
              entityKey={navigation.entityKey}
              name={entityName}
              configuration={configuration}
              onNavigateToParameter={navigateToParameter}
            />
          );
        }
        break;
      }
      case 'steam_wands': {
        // entityConfig is available via configuration.steam_wand_configurations.get(navigation.entityKey)
        if (configuration) {
          detailView = (
            <SteamWandConfigurationDetail
              entityKey={navigation.entityKey}
              name={entityName}
              configuration={configuration}
            />
          );
        }
        break;
      }
      case 'tanks': {
        // entityConfig is available via configuration.tank_configurations.get(navigation.entityKey)
        if (configuration) {
          detailView = (
            <TankConfigurationDetail
              entityKey={navigation.entityKey}
              name={entityName}
              configuration={configuration}
              onNavigateToParameter={navigateToParameter}
            />
          );
        }
        break;
      }
    }

    return (
      <div>
        <button
          onClick={goBack}
          style={{
            padding: '0.5rem 1rem',
            backgroundColor: 'white',
            border: '1px solid #ccc',
            borderRadius: '4px',
            cursor: 'pointer',
            marginBottom: '1rem',
            fontSize: '0.9rem'
          }}
        >
          ← Back
        </button>
        {detailView}
      </div>
    );
  };

  // Level 3: Parameter Editor
  const renderLevel3 = () => {
    if (!navigation.entityType || navigation.entityKey === null || !navigation.parameterCategory) return null;

    // Types for data returned by each editor
    type BoilerControlEditorData = {
      mode: BoilerControlMode;
      target_temperature?: number;
      target_pressure?: number;
    };

    type GroupControlEditorData = {
      mode: GroupBrewControlMode;
      flow_rate?: number | null;
      pressure?: number | null;
      output_flow_rate?: number | null;
      duty_cycle?: number | null;
      duty_cycle_curve?: ControlCurve | null;
      flow_rate_curve?: ControlCurve | null;
      pressure_curve?: ControlCurve | null;
      output_flow_rate_curve?: ControlCurve | null;
    };

    // Union of all possible editor data types
    type EditorData =
      | BoilerControlEditorData
      | GroupControlEditorData
      | PidParameters_for_float
      | PumpConfiguration
      | ControlCurve
      | KalmanParameters
      | null;

    const handleSave = async (data: EditorData) => {
      // Handle special cases for boiler and group control
      if (navigation.parameterCategory === 'boiler_control' && navigation.entityType === 'boilers' && navigation.entityKey !== null) {
        try {
          const boilerData = data as BoilerControlEditorData;
          const request: SetBoilerControlRequest = {
            boiler_index: navigation.entityKey,
            mode: boilerData.mode,
            target_temperature: boilerData.target_temperature ?? null,
            target_pressure: boilerData.target_pressure ?? null
          };
          await postPostcard('/command/set-boiler-control', request, SetBoilerControlRequestSchema);
          alert('Boiler control updated successfully!');
          goBack();
        } catch (err) {
          alert(err instanceof Error ? err.message : 'Failed to update boiler control');
        }
        return;
      }

      if (navigation.parameterCategory === 'group_control' && navigation.entityType === 'groups' && navigation.entityKey !== null) {
        try {
          const groupData = data as GroupControlEditorData;
          const request: SetGroupControlRequest = {
            group_index: navigation.entityKey,
            mode: groupData.mode,
            duty_cycle: groupData.duty_cycle ?? null,
            flow_rate: groupData.flow_rate ?? null,
            pressure: groupData.pressure ?? null,
            output_flow_rate: groupData.output_flow_rate ?? null,
            duty_cycle_curve: groupData.duty_cycle_curve ?? null,
            flow_rate_curve: groupData.flow_rate_curve ?? null,
            pressure_curve: groupData.pressure_curve ?? null,
            output_flow_rate_curve: groupData.output_flow_rate_curve ?? null
          };
          await postPostcard('/command/set-group-control', request, SetGroupControlRequestSchema);
          alert('Group control updated successfully!');
          goBack();
        } catch (err) {
          alert(err instanceof Error ? err.message : 'Failed to update group control');
        }
        return;
      }

      // Handle PID parameter saves
      if (navigation.parameterCategory?.endsWith('_pid') && navigation.entityKey !== null) {
        try {
          let targetType: string;

          if (navigation.entityType === 'boilers') {
            targetType = navigation.parameterCategory === 'temperature_pid'
              ? 'BoilerTemperature'
              : 'BoilerPressure';
          } else if (navigation.entityType === 'groups') {
            if (navigation.parameterCategory === 'flow_rate_pid') {
              targetType = 'GroupFlowRate';
            } else if (navigation.parameterCategory === 'output_flow_rate_pid') {
              targetType = 'GroupOutputFlowRate';
            } else {
              targetType = 'GroupPressure';
            }
          } else {
            throw new Error('Invalid entity type for PID parameters');
          }

          const request: SetPidParametersRequest = {
            target_type: targetType,
            index: navigation.entityKey,
            pid_parameters: data as PidParameters_for_float
          };
          await postPostcard('/command/set-pid-parameters', request, SetPidParametersRequestSchema);
          alert('PID parameters updated successfully!');
          goBack();
        } catch (err) {
          alert(err instanceof Error ? err.message : 'Failed to update PID parameters');
        }
        return;
      }

      // Handle pump configuration saves
      if (navigation.parameterCategory === 'pump_config' && navigation.entityKey !== null) {
        try {
          if (navigation.entityType === 'groups') {
            const request: SetGroupPumpConfigurationRequest = {
              group_index: navigation.entityKey,
              pump_configuration: (data as PumpConfiguration | null) || { tacho_pulses_per_liter: null, max_duty_cycle: null, min_duty_cycle: null, ramp_up_time_ms: null, ramp_down_time_ms: null }
            };
            await postPostcard('/command/set-group-pump-configuration', request, SetGroupPumpConfigurationRequestSchema);
            alert('Group pump configuration updated successfully!');
            goBack();
          } else if (navigation.entityType === 'water_taps') {
            const request: SetWaterTapPumpConfigurationRequest = {
              water_tap_index: navigation.entityKey,
              pump_configuration: (data as PumpConfiguration | null) || { tacho_pulses_per_liter: null, max_duty_cycle: null, min_duty_cycle: null, ramp_up_time_ms: null, ramp_down_time_ms: null }
            };
            await postPostcard('/command/set-water-tap-pump-configuration', request, SetWaterTapPumpConfigurationRequestSchema);
            alert('Water tap pump configuration updated successfully!');
            goBack();
          } else {
            throw new Error('Invalid entity type for pump configuration');
          }
        } catch (err) {
          alert(err instanceof Error ? err.message : 'Failed to update pump configuration');
        }
        return;
      }

      // Handle fill pump configuration saves (boiler)
      if (navigation.parameterCategory === 'fill_pump_config' && navigation.entityType === 'boilers' && navigation.entityKey !== null) {
        try {
          const request: SetFillPumpConfigurationRequest = {
            boiler_index: navigation.entityKey,
            pump_configuration: (data as PumpConfiguration | null) || { tacho_pulses_per_liter: null, max_duty_cycle: null, min_duty_cycle: null, ramp_up_time_ms: null, ramp_down_time_ms: null }
          };
          await postPostcard('/command/set-fill-pump-configuration', request, SetFillPumpConfigurationRequestSchema);
          alert('Fill pump configuration updated successfully!');
          goBack();
        } catch (err) {
          alert(err instanceof Error ? err.message : 'Failed to update fill pump configuration');
        }
        return;
      }

      // TODO: Implement actual save via API for other parameter types
      console.log('Saving configuration:', {
        entityType: navigation.entityType,
        entityKey: navigation.entityKey,
        parameterCategory: navigation.parameterCategory,
        data
      });
      alert('Configuration saved! (Backend integration needed)');
      goBack();
    };

    const handleCancel = () => {
      goBack();
    };

    // Wrapper to handle async save without returning promise
    const handleSaveWrapper = (data: EditorData) => {
      void handleSave(data);
    };

    let editor = null;

    // Get configuration based on entity type and parameter category
    switch (navigation.entityType) {
      case 'boilers': {
        const entityConfig = configuration.boiler_configurations.get(navigation.entityKey);
        if (!entityConfig) break;

        switch (navigation.parameterCategory) {
          case 'temperature_pid':
            editor = (
              <PidParametersEditor
                title="Temperature PID Parameters"
                parameters={entityConfig.temperature_pid_parameters}
                onSave={handleSaveWrapper}
                onCancel={handleCancel}
              />
            );
            break;
          case 'pressure_pid':
            editor = (
              <PidParametersEditor
                title="Pressure PID Parameters"
                parameters={entityConfig.pressure_pid_parameters}
                onSave={handleSaveWrapper}
                onCancel={handleCancel}
              />
            );
            break;
          case 'temperature_kalman':
            editor = (
              <KalmanFilterEditor
                title="Temperature Kalman Filter"
                parameters={entityConfig.temperature_sensor_kalman_parameters ?? null}
                onSave={handleSaveWrapper}
                onCancel={handleCancel}
              />
            );
            break;
          case 'pressure_kalman':
            editor = (
              <KalmanFilterEditor
                title="Pressure Kalman Filter"
                parameters={entityConfig.pressure_sensor_kalman_parameters ?? null}
                onSave={handleSaveWrapper}
                onCancel={handleCancel}
              />
            );
            break;
          case 'fill_pump_config':
            editor = (
              <PumpConfigurationEditor
                title="Fill Pump Configuration"
                configuration={entityConfig.fill_config?.pump_configuration ?? null}
                onSave={handleSaveWrapper}
                onCancel={handleCancel}
              />
            );
            break;
          case 'boiler_control': {
            const boilerDef = getBoilerEntries().find(([key]) => key === navigation.entityKey);
            const entityName = boilerDef ? boilerDef[1].name : `Boiler ${navigation.entityKey}`;
            editor = (
              <BoilerControlEditor
                name={entityName}
                currentMode={entityConfig.control_state.mode}
                currentTargetTemperature={entityConfig.control_state.values.target_temperature}
                currentTargetPressure={entityConfig.control_state.values.target_pressure}
                onSave={handleSaveWrapper}
                onCancel={handleCancel}
              />
            );
            break;
          }
        }
        break;
      }
      case 'groups': {
        const entityConfig = configuration.group_configurations.get(navigation.entityKey);
        if (!entityConfig) break;

        switch (navigation.parameterCategory) {
          case 'flow_rate_pid':
            editor = (
              <PidParametersEditor
                title="Flow Rate PID Parameters"
                parameters={entityConfig.flow_rate_pid_parameters}
                onSave={handleSaveWrapper}
                onCancel={handleCancel}
              />
            );
            break;
          case 'output_flow_rate_pid':
            editor = (
              <PidParametersEditor
                title="Output Flow Rate PID Parameters"
                parameters={entityConfig.output_flow_rate_pid_parameters}
                onSave={handleSaveWrapper}
                onCancel={handleCancel}
              />
            );
            break;
          case 'pressure_pid':
            editor = (
              <PidParametersEditor
                title="Pressure PID Parameters"
                parameters={entityConfig.pressure_pid_parameters}
                onSave={handleSaveWrapper}
                onCancel={handleCancel}
              />
            );
            break;
          case 'pressure_kalman':
            editor = (
              <KalmanFilterEditor
                title="Pressure Kalman Filter"
                parameters={entityConfig.pressure_sensor_kalman_parameters ?? null}
                onSave={handleSaveWrapper}
                onCancel={handleCancel}
              />
            );
            break;
          case 'pump_config':
            editor = (
              <PumpConfigurationEditor
                title="Pump Configuration"
                configuration={entityConfig.pump_configuration ?? null}
                onSave={handleSaveWrapper}
                onCancel={handleCancel}
              />
            );
            break;
          case 'pressure_curve':
            editor = (
              <ControlCurveEditor
                title="Pressure Curve"
                curve={entityConfig.brew_control_state.values.pressure_curve}
                unit="bar"
                onSave={handleSaveWrapper}
                onCancel={handleCancel}
              />
            );
            break;
          case 'flow_rate_curve':
            editor = (
              <ControlCurveEditor
                title="Flow Rate Curve"
                curve={entityConfig.brew_control_state.values.flow_rate_curve}
                unit="mL/s"
                onSave={handleSaveWrapper}
                onCancel={handleCancel}
              />
            );
            break;
          case 'output_flow_rate_curve':
            editor = (
              <ControlCurveEditor
                title="Output Flow Rate Curve"
                curve={entityConfig.brew_control_state.values.output_flow_rate_curve}
                unit="mL/s"
                onSave={handleSaveWrapper}
                onCancel={handleCancel}
              />
            );
            break;
          case 'duty_cycle_curve':
            editor = (
              <ControlCurveEditor
                title="Duty Cycle Curve"
                curve={entityConfig.brew_control_state.values.duty_cycle_curve}
                unit="%"
                onSave={handleSaveWrapper}
                onCancel={handleCancel}
              />
            );
            break;
          case 'group_control': {
            const groupDef = getGroupEntries().find(([key]) => key === navigation.entityKey);
            const entityName = groupDef ? groupDef[1].name : `Group ${navigation.entityKey}`;
            editor = (
              <GroupControlEditor
                name={entityName}
                currentMode={entityConfig.brew_control_state.mode}
                currentValues={entityConfig.brew_control_state.values}
                onSave={handleSaveWrapper}
                onCancel={handleCancel}
              />
            );
            break;
          }
        }
        break;
      }
      case 'water_taps': {
        const entityConfig = configuration.water_tap_configurations.get(navigation.entityKey);
        if (!entityConfig) break;

        switch (navigation.parameterCategory) {
          case 'pump_config':
            editor = (
              <PumpConfigurationEditor
                title="Pump Configuration"
                configuration={entityConfig.pump_configuration ?? null}
                onSave={handleSaveWrapper}
                onCancel={handleCancel}
              />
            );
            break;
        }
        break;
      }
      case 'tanks': {
        const entityConfig = configuration.tank_configurations.get(navigation.entityKey);
        if (!entityConfig) break;

        switch (navigation.parameterCategory) {
          case 'water_level_kalman':
            editor = (
              <KalmanFilterEditor
                title="Water Level Kalman Filter"
                parameters={entityConfig.water_level_sensor_kalman_parameters ?? null}
                onSave={handleSaveWrapper}
                onCancel={handleCancel}
              />
            );
            break;
        }
        break;
      }
    }

    return (
      <div>
        <button
          onClick={goBack}
          style={{
            padding: '0.5rem 1rem',
            backgroundColor: 'white',
            border: '1px solid #ccc',
            borderRadius: '4px',
            cursor: 'pointer',
            marginBottom: '1rem',
            fontSize: '0.9rem'
          }}
        >
          ← Back
        </button>
        {editor || (
          <div
            style={{
              padding: '1.5rem',
              backgroundColor: '#f8f9fa',
              borderRadius: '8px',
              border: '1px solid #ddd'
            }}
          >
            <p>Parameter editor not found for: {navigation.parameterCategory}</p>
          </div>
        )}
      </div>
    );
  };

  if (!isExpanded) {
    return (
      <div
        style={{
          backgroundColor: 'white',
          borderRadius: '8px',
          padding: '1.5rem',
          boxShadow: '0 2px 4px rgba(0,0,0,0.1)',
          marginTop: '1.5rem'
        }}
      >
        <button
          onClick={() => setIsExpanded(true)}
          style={{
            width: '100%',
            padding: '0.75rem',
            backgroundColor: '#f8f9fa',
            border: '1px solid #ddd',
            borderRadius: '6px',
            cursor: 'pointer',
            display: 'flex',
            justifyContent: 'space-between',
            alignItems: 'center',
            fontSize: '1.1rem',
            fontWeight: '500'
          }}
        >
          <span>Configuration</span>
          <span style={{ fontSize: '1.2rem' }}>▶</span>
        </button>
      </div>
    );
  }

  return (
    <div
      style={{
        backgroundColor: 'white',
        borderRadius: '8px',
        padding: '1.5rem',
        boxShadow: '0 2px 4px rgba(0,0,0,0.1)',
        marginTop: '1.5rem'
      }}
    >
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '1rem' }}>
        <h2 style={{ margin: 0 }}>Configuration</h2>
        <button
          onClick={() => {
            setIsExpanded(false);
            setNavigation({ level: 1, entityType: null, entityKey: null, parameterCategory: null });
          }}
          style={{
            padding: '0.5rem 1rem',
            backgroundColor: 'white',
            border: '1px solid #ccc',
            borderRadius: '4px',
            cursor: 'pointer',
            fontSize: '0.9rem'
          }}
        >
          Collapse
        </button>
      </div>

      {renderBreadcrumb()}

      {/* Success/Error Message */}
      {optimizeMessage && (
        <div style={{
          padding: '0.75rem',
          marginTop: '1rem',
          backgroundColor: optimizeMessage.type === 'success' ? '#d4edda' : '#f8d7da',
          color: optimizeMessage.type === 'success' ? '#155724' : '#721c24',
          border: `1px solid ${optimizeMessage.type === 'success' ? '#c3e6cb' : '#f5c6cb'}`,
          borderRadius: '4px',
          fontSize: '0.9rem'
        }}>
          {optimizeMessage.type === 'success' ? '✅' : '❌'} {optimizeMessage.text}
        </div>
      )}

      {navigation.level === 1 && renderLevel1()}
      {navigation.level === 2 && renderLevel2()}
      {navigation.level === 3 && renderLevel3()}

      {/* Storage Optimization - Housekeeping */}
      {isExpanded && navigation.level === 1 && (
        <div style={{
          marginTop: '1.5rem',
          paddingTop: '1rem',
          borderTop: '1px solid #eee',
          display: 'flex',
          justifyContent: 'flex-end'
        }}>
          <button
            onClick={() => void handleOptimizeStorage()}
            style={{
              padding: '0.5rem 0.75rem',
              fontSize: '0.8rem',
              color: '#666',
              backgroundColor: 'transparent',
              border: '1px solid #ddd',
              borderRadius: '4px',
              cursor: 'pointer',
              display: 'flex',
              alignItems: 'center',
              gap: '0.5rem'
            }}
            onMouseEnter={(e) => { e.currentTarget.style.backgroundColor = '#f5f5f5'; }}
            onMouseLeave={(e) => { e.currentTarget.style.backgroundColor = 'transparent'; }}
          >
            🗜️ Optimize Storage
          </button>
        </div>
      )}
    </div>
  );
};

export const ConfigurationPanel = memo(ConfigurationPanelComponent);
