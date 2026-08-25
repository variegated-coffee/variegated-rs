import { useState } from 'preact/hooks';
import { memo } from 'preact/compat';
import { Alert, Badge, Button, Section, tokens } from '@variegated-coffee/ui';
import {
  Configuration,
  PidParameters,
  PumpConfiguration,
  BoilerControlMode,
  GroupBrewControlMode,
  ControlCurve,
  KalmanParameters,
  PidParameterTarget
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
import { getWebSocketService } from '../services/websocket';

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
  const handleOptimizeStorage = () => {
    setOptimizeMessage(null);
    const ws = getWebSocketService();
    if (!ws) {
      setOptimizeMessage({ type: 'error', text: 'WebSocket not connected' });
      setTimeout(() => setOptimizeMessage(null), 4000);
      return;
    }
    ws.optimizeConfigurationStorage();
    setOptimizeMessage({ type: 'success', text: 'Storage optimized successfully' });
    setTimeout(() => setOptimizeMessage(null), 2000);
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
      // A breadcrumb, so it is announced as navigation rather than as a stray sentence,
      // and the separator is `aria-hidden` so it is not read as "greater than" between
      // every level.
      <nav
        aria-label="Configuration breadcrumb"
        style={{ fontSize: '0.9rem', color: tokens.color.inkMuted, marginBottom: tokens.space.md }}
      >
        {parts.map((part, i) => (
          <span key={i}>
            {i > 0 && <span aria-hidden="true"> › </span>}
            {part}
          </span>
        ))}
      </nav>
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
            marginBottom: tokens.space.lg,
            padding: tokens.space.md,
            backgroundColor: tokens.color.surfaceSunken,
            borderRadius: tokens.radius.md,
            border: `1px solid ${tokens.color.border}`,
          }}
        >
          <h3 style={{ marginTop: 0, marginBottom: tokens.space.sm, fontSize: '1.1rem' }}>Machine</h3>
          {/* Was a `<label>` with nothing to label -- it pointed at no control, because
              this is a readout rather than a field. */}
          <div style={{ display: 'flex', alignItems: 'center', gap: tokens.space.sm, fontSize: '0.9rem' }}>
            <span style={{ fontWeight: 500 }}>Heating element interlock</span>
            <Badge role={configuration.machine_config.heating_element_interlock ? 'ok' : undefined}>
              {configuration.machine_config.heating_element_interlock ? 'Enabled' : 'Disabled'}
            </Badge>
          </div>
        </div>

        {/* Boilers */}
        {boilerEntries.length > 0 && (
          <div style={{ marginBottom: '1.5rem' }}>
            <h3 style={{ marginTop: 0, marginBottom: tokens.space.sm, fontSize: '1.1rem' }}>
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
            <h3 style={{ marginTop: 0, marginBottom: tokens.space.sm, fontSize: '1.1rem' }}>
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
            <h3 style={{ marginTop: 0, marginBottom: tokens.space.sm, fontSize: '1.1rem' }}>
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
            <h3 style={{ marginTop: 0, marginBottom: tokens.space.sm, fontSize: '1.1rem' }}>
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
            <h3 style={{ marginTop: 0, marginBottom: tokens.space.sm, fontSize: '1.1rem' }}>
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
        <div style={{ marginBottom: tokens.space.md }}>
          <Button variant="secondary" size="sm" onClick={goBack}>
            <span aria-hidden="true">←</span> Back
          </Button>
        </div>
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
      | PidParameters
      | PumpConfiguration
      | ControlCurve
      | KalmanParameters
      | null;

    const handleSave = (data: EditorData) => {
      const ws = getWebSocketService();
      if (!ws) {
        alert('WebSocket not connected');
        return;
      }

      // Handle special cases for boiler and group control
      if (navigation.parameterCategory === 'boiler_control' && navigation.entityType === 'boilers' && navigation.entityKey !== null) {
        const boilerData = data as BoilerControlEditorData;
        ws.setBoilerControlTarget(
          navigation.entityKey,
          boilerData.mode,
          {
            temperature: boilerData.target_temperature ?? null,
            pressure: boilerData.target_pressure ?? null
          }
        );
        alert('Boiler control updated successfully!');
        goBack();
        return;
      }

      if (navigation.parameterCategory === 'group_control' && navigation.entityType === 'groups' && navigation.entityKey !== null) {
        const groupData = data as GroupControlEditorData;
        ws.setGroupBrewControlTarget(
          navigation.entityKey,
          groupData.mode,
          {
            duty_cycle: groupData.duty_cycle ?? null,
            flow_rate: groupData.flow_rate ?? null,
            pressure: groupData.pressure ?? null,
            output_flow_rate: groupData.output_flow_rate ?? null,
            duty_cycle_curve: groupData.duty_cycle_curve ?? null,
            flow_rate_curve: groupData.flow_rate_curve ?? null,
            pressure_curve: groupData.pressure_curve ?? null,
            output_flow_rate_curve: groupData.output_flow_rate_curve ?? null,
            // This panel edits the control mode and its setpoints. Limits are carried in
            // the same update struct but are not editable here -- nothing in this frontend
            // arms one yet -- and `null` means "leave unchanged", so saving this form
            // cannot move a cap that was set from somewhere else.
            max_pressure: null,
            max_group_flow_rate: null,
            max_output_flow_rate: null
          }
        );
        alert('Group control updated successfully!');
        goBack();
        return;
      }

      // Handle PID parameter saves
      if (navigation.parameterCategory?.endsWith('_pid') && navigation.entityKey !== null) {
        let target: PidParameterTarget;

        if (navigation.entityType === 'boilers') {
          target = navigation.parameterCategory === 'temperature_pid'
            ? { type: 'BoilerTemperature', value: navigation.entityKey }
            : { type: 'BoilerPressure', value: navigation.entityKey };
        } else if (navigation.entityType === 'groups') {
          if (navigation.parameterCategory === 'flow_rate_pid') {
            target = { type: 'GroupFlowRate', value: navigation.entityKey };
          } else if (navigation.parameterCategory === 'output_flow_rate_pid') {
            target = { type: 'GroupOutputFlowRate', value: navigation.entityKey };
          } else {
            target = { type: 'GroupPressure', value: navigation.entityKey };
          }
        } else {
          alert('Invalid entity type for PID parameters');
          return;
        }

        ws.setPidParameters(target, data as PidParameters);
        alert('PID parameters updated successfully!');
        goBack();
        return;
      }

      // Handle pump configuration saves
      if (navigation.parameterCategory === 'pump_config' && navigation.entityKey !== null) {
        const pumpConfig = (data as PumpConfiguration | null) || { tacho_pulses_per_liter: null, max_duty_cycle: null, min_duty_cycle: null, ramp_up_time_ms: null, ramp_down_time_ms: null };

        if (navigation.entityType === 'groups') {
          ws.setGroupPumpConfiguration(navigation.entityKey, pumpConfig);
          alert('Group pump configuration updated successfully!');
          goBack();
        } else if (navigation.entityType === 'water_taps') {
          ws.setWaterTapPumpConfiguration(navigation.entityKey, pumpConfig);
          alert('Water tap pump configuration updated successfully!');
          goBack();
        } else {
          alert('Invalid entity type for pump configuration');
        }
        return;
      }

      // Handle fill pump configuration saves (boiler)
      if (navigation.parameterCategory === 'fill_pump_config' && navigation.entityType === 'boilers' && navigation.entityKey !== null) {
        const pumpConfig = (data as PumpConfiguration | null) || { tacho_pulses_per_liter: null, max_duty_cycle: null, min_duty_cycle: null, ramp_up_time_ms: null, ramp_down_time_ms: null };
        ws.setFillPumpConfiguration(navigation.entityKey, pumpConfig);
        alert('Fill pump configuration updated successfully!');
        goBack();
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
                title="Temperature PID parameters"
                parameters={entityConfig.temperature_pid_parameters}
                // A boiler's temperature loop drives a heating element, so its gains are
                // percent of output per degree of error.
                errorUnit="°C"
                outputUnit="%"
                onSave={handleSaveWrapper}
                onCancel={handleCancel}
              />
            );
            break;
          case 'pressure_pid':
            editor = (
              <PidParametersEditor
                title="Pressure PID parameters"
                parameters={entityConfig.pressure_pid_parameters}
                errorUnit="bar"
                outputUnit="%"
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
                title="Flow rate PID parameters"
                parameters={entityConfig.flow_rate_pid_parameters}
                // A group's loops drive the pump, whose output is its duty cycle.
                errorUnit="mL/s"
                outputUnit="%"
                onSave={handleSaveWrapper}
                onCancel={handleCancel}
              />
            );
            break;
          case 'output_flow_rate_pid':
            editor = (
              <PidParametersEditor
                title="Output flow rate PID parameters"
                parameters={entityConfig.output_flow_rate_pid_parameters}
                errorUnit="mL/s"
                outputUnit="%"
                onSave={handleSaveWrapper}
                onCancel={handleCancel}
              />
            );
            break;
          case 'pressure_pid':
            editor = (
              <PidParametersEditor
                title="Pressure PID parameters"
                parameters={entityConfig.pressure_pid_parameters}
                errorUnit="bar"
                outputUnit="%"
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
        <div style={{ marginBottom: tokens.space.md }}>
          <Button variant="secondary" size="sm" onClick={goBack}>
            <span aria-hidden="true">←</span> Back
          </Button>
        </div>
        {editor || (
          // A build that knows a parameter category the UI has no editor for. Not an empty
          // state -- there is nothing to add -- so it is a warning naming the category, so
          // the gap is reportable rather than just blank.
          <Alert role="warn" title="No editor for this setting">
            This build has no editor for <code>{navigation.parameterCategory}</code>. The
            machine still holds the setting; it just cannot be changed from here.
          </Alert>
        )}
      </div>
    );
  };

  return (
    <div
      style={{
        backgroundColor: tokens.color.surfaceRaised,
        border: `1px solid ${tokens.color.border}`,
        borderRadius: tokens.radius.md,
        padding: tokens.space.lg,
        marginTop: tokens.space.lg,
      }}
    >
      {/*
        One control, in one place, in both states.

        This was finding 08's headline example: collapsed, the whole header was a button
        with a `▶`; expanded, that button vanished and a bordered *Collapse* appeared in
        the opposite corner. Two controls, two positions, one action -- and the expanded
        header printed "Configuration" twice, as title and as description.

        Collapsing resets the navigation as it did before, which is why `onToggle` is used
        rather than letting `Section` own the state alone: reopening at level 3 of a
        breadcrumb the user cannot see the top of is disorienting.
      */}
      <Section
        title="Configuration"
        defaultOpen={false}
        open={isExpanded}
        onToggle={(open) => {
          setIsExpanded(open);
          if (!open) {
            setNavigation({ level: 1, entityType: null, entityKey: null, parameterCategory: null });
          }
        }}
      >
        {renderBreadcrumb()}

        {optimizeMessage && (
          <div style={{ marginTop: tokens.space.md }}>
            <Alert role={optimizeMessage.type === 'success' ? 'ok' : 'danger'}>
              {optimizeMessage.text}
            </Alert>
          </div>
        )}

        {navigation.level === 1 && renderLevel1()}
        {navigation.level === 2 && renderLevel2()}
        {navigation.level === 3 && renderLevel3()}

        {navigation.level === 1 && (
          <div
            style={{
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'space-between',
              gap: tokens.space.sm,
              flexWrap: 'wrap',
              marginTop: tokens.space.lg,
              paddingTop: tokens.space.md,
              borderTop: `1px solid ${tokens.color.border}`,
            }}
          >
            {/* Named for what it does, like the schedule list's. "🗜️ Optimize Storage"
                was an unexplained action with a clamp emoji for a label. */}
            <span style={{ fontSize: '0.8rem', color: tokens.color.inkMuted, maxWidth: '48ch' }}>
              Changing settings leaves gaps in the machine's storage. Compacting reclaims
              them; it does not change any setting.
            </span>
            <Button variant="quiet" size="sm" onClick={() => void handleOptimizeStorage()}>
              Compact storage
            </Button>
          </div>
        )}
      </Section>
    </div>
  );
};

export const ConfigurationPanel = memo(ConfigurationPanelComponent);
