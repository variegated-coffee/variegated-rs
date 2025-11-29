use alloc::collections::BTreeSet;
use esphome_device::{BinarySensorState, StateChange, EntityConfig};
use esphome_device::entity_type::sensor::SensorState;
use esphome_device::entity_type::number::NumberState;
use esphome_device::entity_type::switch::SwitchState;
use esphome_device::entity_type::select::SelectState;
use defmt::{debug, info};
use variegated_controller_types::{MachineDefinition, SensorCapability, ActuatorCapability, ControlModeCapability};

use crate::channels::{
    ApplicationStatusSubscriber, ApplicationConfigurationSubscriber,
    StateChangeSender,
};
use super::entity_builder::{*, boiler_control_mode_to_string};


/// Create a BTreeSet of entity keys for fast lookup during state updates
fn build_entity_key_set(entities: &[EntityConfig]) -> BTreeSet<u32> {
    entities.iter().map(|entity| match entity {
        EntityConfig::Sensor(config) => config.key,
        EntityConfig::BinarySensor(config) => config.key,
        EntityConfig::Number(config) => config.key,
        EntityConfig::Switch(config) => config.key,
        EntityConfig::Select(config) => config.key,
        // Add other entity types as needed
        _ => 0, // Skip unknown entity types
    }).filter(|&key| key != 0).collect()
}

pub async fn configuration_task(
    sender: StateChangeSender,
    mut receiver: ApplicationConfigurationSubscriber,
    machine_def: &MachineDefinition
) {
    loop {
        // Wait for a configuration update
        let config = receiver.next_message_pure().await;

        info!("Processing configuration update with machine definition");

        // Update machine-level configuration
        update_machine_configuration(&sender, &config.machine_config);

        // Update boiler configurations
        for (&boiler_index, boiler_config) in config.iter_boilers() {
            if let Some(boiler_def) = machine_def.boilers.get(&boiler_index) {
                update_boiler_configuration(&sender, boiler_index, boiler_config, boiler_def);
            }
        }

        // Update group configurations
        for (&group_index, group_config) in config.iter_groups() {
            if let Some(group_def) = machine_def.groups.get(&group_index) {
                update_group_configuration(&sender, group_index, group_config, group_def);
            }
        }

        // Update water tap configurations
        for (&water_tap_index, water_tap_config) in config.iter_water_taps() {
            if let Some(water_tap_def) = machine_def.water_taps.get(&water_tap_index) {
                update_water_tap_configuration(&sender, water_tap_index, water_tap_config, water_tap_def);
            }
        }

        // Update steam wand configurations
        for (&steam_wand_index, steam_wand_config) in config.iter_steam_wands() {
            if let Some(steam_wand_def) = machine_def.steam_wands.get(&steam_wand_index) {
                update_steam_wand_configuration(&sender, steam_wand_index, steam_wand_config, steam_wand_def);
            }
        }

        // Update tank configurations
        for (&tank_index, tank_config) in config.iter_tanks() {
            if let Some(tank_def) = machine_def.tanks.get(&tank_index) {
                update_tank_configuration(&sender, tank_index, tank_config, tank_def);
            }
        }
    }
}

pub async fn status_task(
    sender: StateChangeSender,
    mut receiver: ApplicationStatusSubscriber,
    entities: &[EntityConfig<'_>],
    _machine_def: &MachineDefinition
) {
    // Build a set of valid entity keys for fast lookup
    let valid_entity_keys = build_entity_key_set(entities);
    info!("Status task initialized with {} valid entity keys", valid_entity_keys.len());

    // Helper macro to send state updates only for valid entities
    macro_rules! send_if_valid {
        ($key:expr, $state_change:expr) => {
            if valid_entity_keys.contains(&$key) {
                let _ = sender.try_send($state_change);
            }
        };
    }

    let mut i: u8 = 0;
    loop {
        i += 1;
        // Wait for a status update
        let status = receiver.next_message_pure().await;

        // Throttle messages to avoid flooding
        if 10 == i {
            // Update all boiler statuses dynamically
            for (&boiler_index, boiler_status) in status.boiler_statuses.iter() {
                let temp_key = (ENTITY_TYPE_BOILER_CONST as u32) << 24 |
                              ((boiler_index as u32) << 16) |
                              (BOILER_TEMPERATURE_CONST as u32);
                let pressure_key = (ENTITY_TYPE_BOILER_CONST as u32) << 24 |
                                  ((boiler_index as u32) << 16) |
                                  (BOILER_PRESSURE_CONST as u32);
                let water_level_key = (ENTITY_TYPE_BOILER_CONST as u32) << 24 |
                                     ((boiler_index as u32) << 16) |
                                     (BOILER_WATER_LEVEL_CONST as u32);
                let duty_cycle_key = (ENTITY_TYPE_BOILER_CONST as u32) << 24 |
                                    ((boiler_index as u32) << 16) |
                                    (BOILER_DUTY_CYCLE_CONST as u32);
                let temp_setpoint_key = (ENTITY_TYPE_BOILER_CONST as u32) << 24 |
                                       ((boiler_index as u32) << 16) |
                                       (BOILER_TEMP_SETPOINT_CONST as u32);

                send_if_valid!(temp_key, StateChange::SensorStateChange(
                    SensorState::new(temp_key, boiler_status.temperature)
                ));
                send_if_valid!(pressure_key, StateChange::SensorStateChange(
                    SensorState::new(pressure_key, boiler_status.pressure)
                ));
                send_if_valid!(water_level_key, StateChange::SensorStateChange(
                    SensorState::new(water_level_key, boiler_status.water_level.map(|wl| wl as f32))
                ));
                send_if_valid!(duty_cycle_key, StateChange::SensorStateChange(SensorState::new(
                    duty_cycle_key,
                    Some(boiler_status.output.duty_cycle() as f32)
                )));

                // Extract and publish PID terms (only for PID output)
                if let variegated_controller_types::Output::PidOutput(pid_out) = &boiler_status.output {
                    let pid_p_key = (ENTITY_TYPE_BOILER_CONST as u32) << 24 |
                                   ((boiler_index as u32) << 16) |
                                   (BOILER_PID_P_TERM_CONST as u32);
                    let pid_i_key = (ENTITY_TYPE_BOILER_CONST as u32) << 24 |
                                   ((boiler_index as u32) << 16) |
                                   (BOILER_PID_I_TERM_CONST as u32);
                    let pid_d_key = (ENTITY_TYPE_BOILER_CONST as u32) << 24 |
                                   ((boiler_index as u32) << 16) |
                                   (BOILER_PID_D_TERM_CONST as u32);

                    send_if_valid!(pid_p_key, StateChange::SensorStateChange(
                        SensorState::new(pid_p_key, Some(pid_out.p))
                    ));
                    send_if_valid!(pid_i_key, StateChange::SensorStateChange(
                        SensorState::new(pid_i_key, Some(pid_out.i))
                    ));
                    send_if_valid!(pid_d_key, StateChange::SensorStateChange(
                        SensorState::new(pid_d_key, Some(pid_out.d))
                    ));
                }

                // Extract and publish temperature setpoint
                let temp_setpoint = boiler_status.control_state.values.target_temperature;
                send_if_valid!(temp_setpoint_key, StateChange::NumberStateChange(NumberState {
                    key: temp_setpoint_key,
                    state: temp_setpoint,
                    missing_state: false,
                }));

                // Extract and publish pressure setpoint (send_if_valid filters non-existent entities)
                let pressure_setpoint_key = (ENTITY_TYPE_BOILER_CONST as u32) << 24 |
                                           ((boiler_index as u32) << 16) |
                                           (BOILER_PRESSURE_SETPOINT_CONST as u32);

                send_if_valid!(pressure_setpoint_key, StateChange::NumberStateChange(NumberState {
                    key: pressure_setpoint_key,
                    state: boiler_status.control_state.values.target_pressure,
                    missing_state: false,
                }));

                debug!("Updated boiler {} status", boiler_index);
            }

            // Update all group statuses dynamically
            for (&group_index, group_status) in status.group_statuses.iter() {
                let input_flow_key = (ENTITY_TYPE_GROUP_CONST as u32) << 24 |
                                    ((group_index as u32) << 16) |
                                    (GROUP_INPUT_FLOW_RATE_CONST as u32);
                let output_flow_key = (ENTITY_TYPE_GROUP_CONST as u32) << 24 |
                                     ((group_index as u32) << 16) |
                                     (GROUP_OUTPUT_FLOW_RATE_CONST as u32);
                let weight_key = (ENTITY_TYPE_GROUP_CONST as u32) << 24 |
                                ((group_index as u32) << 16) |
                                (GROUP_WEIGHT_CONST as u32);
                let is_brewing_key = (ENTITY_TYPE_GROUP_CONST as u32) << 24 |
                                    ((group_index as u32) << 16) |
                                    (GROUP_IS_BREWING_CONST as u32);
                let brew_time_key = (ENTITY_TYPE_GROUP_CONST as u32) << 24 |
                                   ((group_index as u32) << 16) |
                                   (GROUP_BREW_TIME_CONST as u32);
                let pump_duty_key = (ENTITY_TYPE_GROUP_CONST as u32) << 24 |
                                   ((group_index as u32) << 16) |
                                   (GROUP_PUMP_DUTY_CYCLE_CONST as u32);

                send_if_valid!(input_flow_key, StateChange::SensorStateChange(
                    SensorState::new(input_flow_key, group_status.input_flow_rate)
                ));
                send_if_valid!(output_flow_key, StateChange::SensorStateChange(SensorState::new(
                    output_flow_key,
                    group_status.output_flow_rate
                )));
                send_if_valid!(weight_key, StateChange::SensorStateChange(SensorState::new(
                    weight_key,
                    group_status.output_weight
                )));
                send_if_valid!(is_brewing_key, StateChange::BinarySensorChange(BinarySensorState {
                    key: is_brewing_key,
                    state: group_status.is_brewing,
                    missing_state: false,
                }));
                send_if_valid!(brew_time_key, StateChange::SensorStateChange(SensorState::new(
                    brew_time_key,
                    group_status.current_brew.as_ref().map(|ref s| s.brew_time.as_secs_f32())
                )));
                send_if_valid!(pump_duty_key, StateChange::SensorStateChange(SensorState::new(
                    pump_duty_key,
                    Some(group_status.pump_output.duty_cycle() as f32)
                )));

                // Extract and publish group PID terms (only for PID output)
                if let variegated_controller_types::Output::PidOutput(pid_out) = &group_status.pump_output {
                    let pid_p_key = (ENTITY_TYPE_GROUP_CONST as u32) << 24 |
                                   ((group_index as u32) << 16) |
                                   (GROUP_PID_P_TERM_CONST as u32);
                    let pid_i_key = (ENTITY_TYPE_GROUP_CONST as u32) << 24 |
                                   ((group_index as u32) << 16) |
                                   (GROUP_PID_I_TERM_CONST as u32);
                    let pid_d_key = (ENTITY_TYPE_GROUP_CONST as u32) << 24 |
                                   ((group_index as u32) << 16) |
                                   (GROUP_PID_D_TERM_CONST as u32);

                    send_if_valid!(pid_p_key, StateChange::SensorStateChange(
                        SensorState::new(pid_p_key, Some(pid_out.p))
                    ));
                    send_if_valid!(pid_i_key, StateChange::SensorStateChange(
                        SensorState::new(pid_i_key, Some(pid_out.i))
                    ));
                    send_if_valid!(pid_d_key, StateChange::SensorStateChange(
                        SensorState::new(pid_d_key, Some(pid_out.d))
                    ));
                }

                debug!("Updated group {} status", group_index);
            }

            // Update all water tap statuses dynamically
            for (&water_tap_index, water_tap_status) in status.water_tap_statuses.iter() {
                let is_dispensing_key = (ENTITY_TYPE_WATER_TAP_CONST as u32) << 24 |
                                       ((water_tap_index as u32) << 16) |
                                       (WATER_TAP_IS_DISPENSING_CONST as u32);

                send_if_valid!(is_dispensing_key, StateChange::BinarySensorChange(BinarySensorState {
                    key: is_dispensing_key,
                    state: water_tap_status.is_dispensing,
                    missing_state: false,
                }));

                debug!("Updated water tap {} status", water_tap_index);
            }

            // Update all steam wand statuses dynamically
            for (&steam_wand_index, steam_wand_status) in status.steam_wand_statuses.iter() {
                let is_steaming_key = (ENTITY_TYPE_STEAM_WAND_CONST as u32) << 24 |
                                     ((steam_wand_index as u32) << 16) |
                                     (STEAM_WAND_IS_STEAMING_CONST as u32);
                let valve_openness_key = (ENTITY_TYPE_STEAM_WAND_CONST as u32) << 24 |
                                        ((steam_wand_index as u32) << 16) |
                                        (STEAM_WAND_VALVE_OPENNESS_CONST as u32);

                send_if_valid!(is_steaming_key, StateChange::BinarySensorChange(BinarySensorState {
                    key: is_steaming_key,
                    state: steam_wand_status.is_steaming,
                    missing_state: false,
                }));

                send_if_valid!(valve_openness_key, StateChange::SensorStateChange(
                    SensorState::new(valve_openness_key, Some(steam_wand_status.valve_openness as f32))
                ));

                debug!("Updated steam wand {} status", steam_wand_index);
            }

            // Update all tank statuses dynamically
            for (&tank_index, tank_status) in status.tank_statuses.iter() {
                let water_level_key = (ENTITY_TYPE_TANK_CONST as u32) << 24 |
                                     ((tank_index as u32) << 16) |
                                     (TANK_WATER_LEVEL_CONST as u32);

                send_if_valid!(water_level_key, StateChange::SensorStateChange(
                    SensorState::new(water_level_key, tank_status.water_level.map(|wl| wl as f32))
                ));

                debug!("Updated tank {} status", tank_index);
            }

            // Update machine mode status
            let machine_mode_key = (ENTITY_TYPE_MACHINE_CONST as u32) << 24 |
                                   (0 << 16) |
                                   (super::entity_builder::MACHINE_MODE_CONST as u32);

            send_if_valid!(machine_mode_key, StateChange::SelectStateChange(SelectState {
                key: machine_mode_key,
                state: super::entity_builder::machine_mode_to_string(status.mode),
                missing_state: false,
            }));

            i = 0;
        }
    }
}

fn update_machine_configuration(
    sender: &StateChangeSender,
    machine_config: &variegated_controller_types::MachineConfiguration,
) {
    // Heating element interlock
    let interlock_key = (ENTITY_TYPE_MACHINE_CONST as u32) << 24 |
                       (0 << 16) |
                       (MACHINE_HEATING_ELEMENT_INTERLOCK_CONST as u32);

    let _ = sender.try_send(StateChange::SwitchStateChange(SwitchState {
        key: interlock_key,
        state: machine_config.heating_element_interlock,
    }));

    info!("Updated machine configuration: heating_element_interlock={}", machine_config.heating_element_interlock);
}

fn update_boiler_configuration(
    sender: &StateChangeSender,
    boiler_index: u8,
    boiler_config: &variegated_controller_types::BoilerConfiguration,
    boiler_def: &variegated_controller_types::BoilerDefinition,
) {
    let has_temperature_sensor = boiler_def.sensors.contains(&SensorCapability::Temperature);
    let has_pressure_sensor = boiler_def.sensors.contains(&SensorCapability::Pressure);
    let has_temperature_control = boiler_def.control_modes.contains(&ControlModeCapability::TemperaturePid);
    let has_pressure_control = boiler_def.control_modes.contains(&ControlModeCapability::PressurePid);

    // Temperature PID parameters (only if temperature control is available)
    if has_temperature_control {
        let temp_pid = &boiler_config.temperature_pid_parameters;

        let temp_kp_key = (ENTITY_TYPE_BOILER_CONST as u32) << 24 |
                         ((boiler_index as u32) << 16) |
                         (BOILER_TEMP_KP_CONST as u32);
        let temp_ki_key = (ENTITY_TYPE_BOILER_CONST as u32) << 24 |
                         ((boiler_index as u32) << 16) |
                         (BOILER_TEMP_KI_CONST as u32);
        let temp_kd_key = (ENTITY_TYPE_BOILER_CONST as u32) << 24 |
                         ((boiler_index as u32) << 16) |
                         (BOILER_TEMP_KD_CONST as u32);

        let _ = sender.try_send(StateChange::NumberStateChange(NumberState {
            key: temp_kp_key,
            state: temp_pid.kp.positive_scale,
            missing_state: false,
        }));

        let _ = sender.try_send(StateChange::NumberStateChange(NumberState {
            key: temp_ki_key,
            state: temp_pid.ki.positive_scale,
            missing_state: false,
        }));

        let _ = sender.try_send(StateChange::NumberStateChange(NumberState {
            key: temp_kd_key,
            state: temp_pid.kd.positive_scale,
            missing_state: false,
        }));

        // Temperature PID Limits
        let temp_kp_upper_limit_key = (ENTITY_TYPE_BOILER_CONST as u32) << 24 |
                                     ((boiler_index as u32) << 16) |
                                     (BOILER_TEMP_KP_UPPER_LIMIT_CONST as u32);
        let temp_kp_lower_limit_key = (ENTITY_TYPE_BOILER_CONST as u32) << 24 |
                                     ((boiler_index as u32) << 16) |
                                     (BOILER_TEMP_KP_LOWER_LIMIT_CONST as u32);
        let temp_ki_upper_limit_key = (ENTITY_TYPE_BOILER_CONST as u32) << 24 |
                                     ((boiler_index as u32) << 16) |
                                     (BOILER_TEMP_KI_UPPER_LIMIT_CONST as u32);
        let temp_ki_lower_limit_key = (ENTITY_TYPE_BOILER_CONST as u32) << 24 |
                                     ((boiler_index as u32) << 16) |
                                     (BOILER_TEMP_KI_LOWER_LIMIT_CONST as u32);
        let temp_kd_upper_limit_key = (ENTITY_TYPE_BOILER_CONST as u32) << 24 |
                                     ((boiler_index as u32) << 16) |
                                     (BOILER_TEMP_KD_UPPER_LIMIT_CONST as u32);
        let temp_kd_lower_limit_key = (ENTITY_TYPE_BOILER_CONST as u32) << 24 |
                                     ((boiler_index as u32) << 16) |
                                     (BOILER_TEMP_KD_LOWER_LIMIT_CONST as u32);

        let _ = sender.try_send(StateChange::NumberStateChange(NumberState {
            key: temp_kp_upper_limit_key,
            state: temp_pid.kp.limits.upper(),
            missing_state: false,
        }));

        let _ = sender.try_send(StateChange::NumberStateChange(NumberState {
            key: temp_kp_lower_limit_key,
            state: temp_pid.kp.limits.lower(),
            missing_state: false,
        }));

        let _ = sender.try_send(StateChange::NumberStateChange(NumberState {
            key: temp_ki_upper_limit_key,
            state: temp_pid.ki.limits.upper(),
            missing_state: false,
        }));

        let _ = sender.try_send(StateChange::NumberStateChange(NumberState {
            key: temp_ki_lower_limit_key,
            state: temp_pid.ki.limits.lower(),
            missing_state: false,
        }));

        let _ = sender.try_send(StateChange::NumberStateChange(NumberState {
            key: temp_kd_upper_limit_key,
            state: temp_pid.kd.limits.upper(),
            missing_state: false,
        }));

        let _ = sender.try_send(StateChange::NumberStateChange(NumberState {
            key: temp_kd_lower_limit_key,
            state: temp_pid.kd.limits.lower(),
            missing_state: false,
        }));
    }

    // Pressure PID parameters (only if pressure control is available)
    if has_pressure_control {
        let pressure_pid = &boiler_config.pressure_pid_parameters;

        let pressure_kp_key = (ENTITY_TYPE_BOILER_CONST as u32) << 24 |
                             ((boiler_index as u32) << 16) |
                             (BOILER_PRESSURE_KP_CONST as u32);
        let pressure_ki_key = (ENTITY_TYPE_BOILER_CONST as u32) << 24 |
                             ((boiler_index as u32) << 16) |
                             (BOILER_PRESSURE_KI_CONST as u32);
        let pressure_kd_key = (ENTITY_TYPE_BOILER_CONST as u32) << 24 |
                             ((boiler_index as u32) << 16) |
                             (BOILER_PRESSURE_KD_CONST as u32);

        let _ = sender.try_send(StateChange::NumberStateChange(NumberState {
            key: pressure_kp_key,
            state: pressure_pid.kp.positive_scale,
            missing_state: false,
        }));

        let _ = sender.try_send(StateChange::NumberStateChange(NumberState {
            key: pressure_ki_key,
            state: pressure_pid.ki.positive_scale,
            missing_state: false,
        }));

        let _ = sender.try_send(StateChange::NumberStateChange(NumberState {
            key: pressure_kd_key,
            state: pressure_pid.kd.positive_scale,
            missing_state: false,
        }));
    }

    // Max temperature (only if temperature sensor is available)
    if has_temperature_sensor {
        if let Some(max_temp) = boiler_config.max_temperature {
            let max_temp_key = (ENTITY_TYPE_BOILER_CONST as u32) << 24 |
                              ((boiler_index as u32) << 16) |
                              (BOILER_MAX_TEMPERATURE_CONST as u32);

            let _ = sender.try_send(StateChange::NumberStateChange(NumberState {
                key: max_temp_key,
                state: max_temp,
                missing_state: false,
            }));
        }
    }

    // Max pressure (only if pressure sensor is available)
    if has_pressure_sensor {
        if let Some(max_pressure) = boiler_config.max_pressure {
            let max_pressure_key = (ENTITY_TYPE_BOILER_CONST as u32) << 24 |
                                   ((boiler_index as u32) << 16) |
                                   (BOILER_MAX_PRESSURE_CONST as u32);

            let _ = sender.try_send(StateChange::NumberStateChange(NumberState {
                key: max_pressure_key,
                state: max_pressure,
                missing_state: false,
            }));
        }
    }

    // Fill configuration (only if boiler has fill mechanism)
    if boiler_def.has_fill_mechanism {
        if let Some(fill_config) = &boiler_config.fill_config {
            if let Some(fill_threshold) = fill_config.fill_threshold {
                let fill_threshold_key = (ENTITY_TYPE_BOILER_CONST as u32) << 24 |
                                        ((boiler_index as u32) << 16) |
                                        (BOILER_FILL_THRESHOLD_CONST as u32);

                let _ = sender.try_send(StateChange::NumberStateChange(NumberState {
                    key: fill_threshold_key,
                    state: fill_threshold as f32,
                    missing_state: false,
                }));
            }
        }
    }

    // Control mode state (if select entity exists)
    let control_mode_key = (ENTITY_TYPE_BOILER_CONST as u32) << 24 |
                          ((boiler_index as u32) << 16) |
                          (BOILER_CONTROL_MODE_CONST as u32);

    let _ = sender.try_send(StateChange::SelectStateChange(SelectState {
        key: control_mode_key,
        state: boiler_control_mode_to_string(boiler_config.control_state.mode),
        missing_state: false,
    }));

    info!("Updated boiler {} configuration", boiler_index);
}

fn update_group_configuration(
    sender: &StateChangeSender,
    group_index: u8,
    group_config: &variegated_controller_types::GroupConfiguration,
    group_def: &variegated_controller_types::GroupDefinition,
) {
    let has_scale_tare = group_def.actuators.contains(&ActuatorCapability::ScaleTare);
    let has_flow_rate_control = group_def.control_modes.contains(&ControlModeCapability::FlowRatePid);
    let has_output_flow_rate_control = group_def.control_modes.contains(&ControlModeCapability::OutputFlowRatePid);

    // Flow rate setpoint (always publish if capability exists, regardless of current mode)
    if has_flow_rate_control {
        let flow_setpoint_key = (ENTITY_TYPE_GROUP_CONST as u32) << 24 |
                               ((group_index as u32) << 16) |
                               (GROUP_FLOW_RATE_SETPOINT_CONST as u32);

        let _ = sender.try_send(StateChange::NumberStateChange(NumberState {
            key: flow_setpoint_key,
            state: group_config.brew_control_state.values.flow_rate,
            missing_state: false,
        }));
    }

    // Output flow rate setpoint (always publish if capability exists, regardless of current mode)
    if has_output_flow_rate_control {
        let output_flow_setpoint_key = (ENTITY_TYPE_GROUP_CONST as u32) << 24 |
                                      ((group_index as u32) << 16) |
                                      (GROUP_OUTPUT_FLOW_RATE_SETPOINT_CONST as u32);

        let _ = sender.try_send(StateChange::NumberStateChange(NumberState {
            key: output_flow_setpoint_key,
            state: group_config.brew_control_state.values.output_flow_rate,
            missing_state: false,
        }));
    }

    // Max brew time
    if let Some(max_brew_time) = group_config.max_brew_time_seconds {
        let max_brew_time_key = (ENTITY_TYPE_GROUP_CONST as u32) << 24 |
                               ((group_index as u32) << 16) |
                               (GROUP_MAX_BREW_TIME_CONST as u32);

        let _ = sender.try_send(StateChange::NumberStateChange(NumberState {
            key: max_brew_time_key,
            state: max_brew_time as f32,
            missing_state: false,
        }));
    }

    // Auto tare enabled (only if scale tare capability is available)
    if has_scale_tare {
        let auto_tare_key = (ENTITY_TYPE_GROUP_CONST as u32) << 24 |
                           ((group_index as u32) << 16) |
                           (GROUP_AUTO_TARE_ENABLED_CONST as u32);

        let _ = sender.try_send(StateChange::SwitchStateChange(SwitchState {
            key: auto_tare_key,
            state: group_config.auto_tare_enabled,
        }));
    }

    info!("Updated group {} configuration", group_index);
}

fn update_water_tap_configuration(
    sender: &StateChangeSender,
    water_tap_index: u8,
    water_tap_config: &variegated_controller_types::WaterTapConfiguration,
    _water_tap_def: &variegated_controller_types::WaterTapDefinition,
) {
    // Temperature target
    if let Some(temp_target) = water_tap_config.temperature_target {
        let temp_target_key = (ENTITY_TYPE_WATER_TAP_CONST as u32) << 24 |
                             ((water_tap_index as u32) << 16) |
                             (WATER_TAP_TEMPERATURE_TARGET_CONST as u32);

        let _ = sender.try_send(StateChange::NumberStateChange(NumberState {
            key: temp_target_key,
            state: temp_target,
            missing_state: false,
        }));
    }

    // Max dispense time
    if let Some(max_dispense_time) = water_tap_config.max_dispense_time_seconds {
        let max_dispense_time_key = (ENTITY_TYPE_WATER_TAP_CONST as u32) << 24 |
                                   ((water_tap_index as u32) << 16) |
                                   (WATER_TAP_MAX_DISPENSE_TIME_CONST as u32);

        let _ = sender.try_send(StateChange::NumberStateChange(NumberState {
            key: max_dispense_time_key,
            state: max_dispense_time as f32,
            missing_state: false,
        }));
    }

    // Flow rate limit
    if let Some(flow_rate_limit) = water_tap_config.flow_rate_limit {
        let flow_rate_limit_key = (ENTITY_TYPE_WATER_TAP_CONST as u32) << 24 |
                                 ((water_tap_index as u32) << 16) |
                                 (WATER_TAP_FLOW_RATE_LIMIT_CONST as u32);

        let _ = sender.try_send(StateChange::NumberStateChange(NumberState {
            key: flow_rate_limit_key,
            state: flow_rate_limit,
            missing_state: false,
        }));
    }

    info!("Updated water tap {} configuration", water_tap_index);
}

fn update_steam_wand_configuration(
    sender: &StateChangeSender,
    steam_wand_index: u8,
    steam_wand_config: &variegated_controller_types::SteamWandConfiguration,
    _steam_wand_def: &variegated_controller_types::SteamWandDefinition,
) {
    // Temperature target
    if let Some(temp_target) = steam_wand_config.temperature_target {
        let temp_target_key = (ENTITY_TYPE_STEAM_WAND_CONST as u32) << 24 |
                             ((steam_wand_index as u32) << 16) |
                             (STEAM_WAND_TEMPERATURE_TARGET_CONST as u32);

        let _ = sender.try_send(StateChange::NumberStateChange(NumberState {
            key: temp_target_key,
            state: temp_target,
            missing_state: false,
        }));
    }

    // Openness
    if let Some(openness) = steam_wand_config.openness {
        let openness_key = (ENTITY_TYPE_STEAM_WAND_CONST as u32) << 24 |
                                 ((steam_wand_index as u32) << 16) |
                                 (STEAM_WAND_OPENNESS_CONST as u32);

        let _ = sender.try_send(StateChange::NumberStateChange(NumberState {
            key: openness_key,
            state: openness as f32,
            missing_state: false,
        }));
    }

    // Purge time
    if let Some(purge_time) = steam_wand_config.purge_time_seconds {
        let purge_time_key = (ENTITY_TYPE_STEAM_WAND_CONST as u32) << 24 |
                            ((steam_wand_index as u32) << 16) |
                            (STEAM_WAND_PURGE_TIME_CONST as u32);

        let _ = sender.try_send(StateChange::NumberStateChange(NumberState {
            key: purge_time_key,
            state: purge_time as f32,
            missing_state: false,
        }));
    }

    // Max steam time
    if let Some(max_steam_time) = steam_wand_config.max_steam_time_seconds {
        let max_steam_time_key = (ENTITY_TYPE_STEAM_WAND_CONST as u32) << 24 |
                                ((steam_wand_index as u32) << 16) |
                                (STEAM_WAND_MAX_STEAM_TIME_CONST as u32);

        let _ = sender.try_send(StateChange::NumberStateChange(NumberState {
            key: max_steam_time_key,
            state: max_steam_time as f32,
            missing_state: false,
        }));
    }

    // Auto purge enabled
    let auto_purge_key = (ENTITY_TYPE_STEAM_WAND_CONST as u32) << 24 |
                        ((steam_wand_index as u32) << 16) |
                        (STEAM_WAND_AUTO_PURGE_ENABLED_CONST as u32);

    let _ = sender.try_send(StateChange::SwitchStateChange(SwitchState {
        key: auto_purge_key,
        state: steam_wand_config.auto_purge_enabled,
    }));

    info!("Updated steam wand {} configuration", steam_wand_index);
}

fn update_tank_configuration(
    sender: &StateChangeSender,
    tank_index: u8,
    tank_config: &variegated_controller_types::TankConfiguration,
    _tank_def: &variegated_controller_types::TankDefinition,
) {
    // Low level warning threshold
    if let Some(threshold) = tank_config.low_level_warning_threshold {
        let threshold_key = (ENTITY_TYPE_TANK_CONST as u32) << 24 |
                           ((tank_index as u32) << 16) |
                           (TANK_LOW_LEVEL_WARNING_THRESHOLD_CONST as u32);

        let _ = sender.try_send(StateChange::NumberStateChange(NumberState {
            key: threshold_key,
            state: threshold as f32,
            missing_state: false,
        }));
    }

    info!("Updated tank {} configuration", tank_index);
}
