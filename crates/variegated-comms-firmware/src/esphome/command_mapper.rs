use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver as ChannelReceiver, Sender as ChannelSender};
use esphome_device::{ClientEvent, Command};
use defmt::info;
use variegated_controller_types::{BoilerControlTargetValuesUpdate, MachineCommand, PidParameterTarget};

use crate::channels::{CLIENT_EVENT_CAPACITY, MACHINE_COMMAND_CAPACITY, ApplicationConfigurationSubscriber};
use super::entity_builder::{*, string_to_boiler_control_mode, string_to_machine_mode};

pub async fn sensor_states_task(
    command_receiver: ChannelReceiver<'static, CriticalSectionRawMutex, ClientEvent, CLIENT_EVENT_CAPACITY>,
    machine_command_sender: ChannelSender<'static, CriticalSectionRawMutex, MachineCommand, MACHINE_COMMAND_CAPACITY>,
    mut configuration_subscriber: ApplicationConfigurationSubscriber
) {
    loop {
        // Wait for a command
        let command = command_receiver.receive().await;

        match command {
            ClientEvent::CommandReceived(Command::SwitchCommand(data)) => {
                info!("Switch command received: state={}", data.state);
            }
            ClientEvent::CommandReceived(Command::NumberCommand(data)) => {
                info!("Number command received: key={}, state={}", data.key, data.state);

                // Parse the key to determine entity type, device index, and property
                let (entity_type, device_index, property) = parse_key(data.key);

                if entity_type == ENTITY_TYPE_BOILER_CONST {
                    match property {
                        BOILER_TEMP_SETPOINT_CONST => {
                            // Send SetBoilerControlTargetValues command via UART
                            let machine_command = MachineCommand::SetBoilerControlTargetValues(
                                device_index,
                                BoilerControlTargetValuesUpdate {
                                    temperature: Some(data.state),
                                    pressure: None,
                                }
                            );
                            machine_command_sender.send(machine_command).await;
                            info!("Sent SetBoilerControlTargetValues command for boiler {} with temperature: {}", device_index, data.state);
                        }
                        BOILER_TEMP_KP_CONST | BOILER_TEMP_KI_CONST | BOILER_TEMP_KD_CONST => {
                            // Get current configuration to build modified PID parameters
                            let current_config = configuration_subscriber.next_message_pure().await;

                            if let Some(boiler_config) = current_config.get_boiler_configuration(device_index) {
                                let mut new_pid_params = boiler_config.temperature_pid_parameters.clone();

                                // Update the specific parameter (set both positive_scale and negative_scale)
                                match property {
                                    BOILER_TEMP_KP_CONST => {
                                        new_pid_params.kp.positive_scale = data.state;
                                        new_pid_params.kp.negative_scale = data.state;
                                        info!("Updated boiler {} temperature kP to {}", device_index, data.state);
                                    }
                                    BOILER_TEMP_KI_CONST => {
                                        new_pid_params.ki.positive_scale = data.state;
                                        new_pid_params.ki.negative_scale = data.state;
                                        info!("Updated boiler {} temperature kI to {}", device_index, data.state);
                                    }
                                    BOILER_TEMP_KD_CONST => {
                                        new_pid_params.kd.positive_scale = data.state;
                                        new_pid_params.kd.negative_scale = data.state;
                                        info!("Updated boiler {} temperature kD to {}", device_index, data.state);
                                    }
                                    _ => {}
                                }

                                // Send SetPidParameters command via UART
                                let machine_command = MachineCommand::SetPidParameters(
                                    PidParameterTarget::BoilerTemperature(device_index),
                                    new_pid_params
                                );

                                machine_command_sender.send(machine_command).await;
                                info!("Sent SetPidParameters command for boiler {}", device_index);
                            }
                        }
                        BOILER_TEMP_KP_UPPER_LIMIT_CONST | BOILER_TEMP_KP_LOWER_LIMIT_CONST |
                        BOILER_TEMP_KI_UPPER_LIMIT_CONST | BOILER_TEMP_KI_LOWER_LIMIT_CONST |
                        BOILER_TEMP_KD_UPPER_LIMIT_CONST | BOILER_TEMP_KD_LOWER_LIMIT_CONST => {
                            // Get current configuration to build modified PID parameters with updated limits
                            let current_config = configuration_subscriber.next_message_pure().await;

                            if let Some(boiler_config) = current_config.get_boiler_configuration(device_index) {
                                let mut new_pid_params = boiler_config.temperature_pid_parameters.clone();

                                // Update the specific limit by setting the entire limits structure
                                match property {
                                    BOILER_TEMP_KP_UPPER_LIMIT_CONST => {
                                        match new_pid_params.kp.limits.try_set_upper(data.state) {
                                            Ok(_) => info!("Updated boiler {} temperature kP upper limit to {}", device_index, data.state),
                                            Err(_e) => info!("Failed to update boiler {} temperature kP upper limit to {}", device_index, data.state),
                                        }
                                    }
                                    BOILER_TEMP_KP_LOWER_LIMIT_CONST => {
                                        match new_pid_params.kp.limits.try_set_lower(data.state) {
                                            Ok(_) => info!("Updated boiler {} temperature kP lower limit to {}", device_index, data.state),
                                            Err(_e) => info!("Failed to update boiler {} temperature kP lower limit to {}", device_index, data.state),
                                        }
                                    }
                                    BOILER_TEMP_KI_UPPER_LIMIT_CONST => {
                                        new_pid_params.ki.limits.try_set_upper(data.state).ok();
                                        info!("Updated boiler {} temperature kI upper limit to {}", device_index, data.state);
                                    }
                                    BOILER_TEMP_KI_LOWER_LIMIT_CONST => {
                                        new_pid_params.ki.limits.try_set_lower(data.state).ok();
                                        info!("Updated boiler {} temperature kI lower limit to {}", device_index, data.state);
                                    }
                                    BOILER_TEMP_KD_UPPER_LIMIT_CONST => {
                                        new_pid_params.kd.limits.try_set_upper(data.state).ok();
                                        info!("Updated boiler {} temperature kD upper limit to {}", device_index, data.state);
                                    }
                                    BOILER_TEMP_KD_LOWER_LIMIT_CONST => {
                                        new_pid_params.kd.limits.try_set_lower(data.state).ok();
                                        info!("Updated boiler {} temperature kD lower limit to {}", device_index, data.state);
                                    }
                                    _ => {}
                                }

                                // Send SetPidParameters command via UART
                                let machine_command = MachineCommand::SetPidParameters(
                                    PidParameterTarget::BoilerTemperature(device_index),
                                    new_pid_params
                                );

                                machine_command_sender.send(machine_command).await;
                                info!("Sent SetPidParameters command for boiler {} with updated limits", device_index);
                            }
                        }
                        _ => {
                            info!("Unhandled boiler property: 0x{:04X}", property);
                        }
                    }
                } else if entity_type == ENTITY_TYPE_GROUP_CONST {
                    // Handle group commands (future expansion)
                    info!("Group command not yet implemented: device={}, property=0x{:04X}", device_index, property);
                } else {
                    info!("Unknown entity type: {}", entity_type);
                }
            }
            ClientEvent::CommandReceived(Command::SelectCommand(data)) => {
                info!("Select command received: key={}", data.key);

                // Parse the key to determine entity type, device index, and property
                let (entity_type, device_index, property) = parse_key(data.key);

                if entity_type == ENTITY_TYPE_BOILER_CONST && property == BOILER_CONTROL_MODE_CONST {
                    if let Some(mode) = string_to_boiler_control_mode(&data.state) {
                        // Send SetBoilerControlTarget command via UART
                        let machine_command = MachineCommand::SetBoilerControlTarget(
                            device_index,
                            mode,
                            None  // Don't change target values, just mode
                        );
                        machine_command_sender.send(machine_command).await;
                        info!("Sent SetBoilerControlTarget command for boiler {}", device_index);
                    } else {
                        info!("Invalid control mode");
                    }
                } else if entity_type == ENTITY_TYPE_MACHINE_CONST && property == MACHINE_MODE_CONST {
                    if let Some(mode) = string_to_machine_mode(&data.state) {
                        // Send SetMachineMode command via UART
                        let machine_command = MachineCommand::SetMachineMode(mode);
                        machine_command_sender.send(machine_command).await;
                        info!("Sent SetMachineMode command");
                    } else {
                        info!("Invalid machine mode");
                    }
                } else {
                    info!("Unknown select entity: type={}, device={}, property=0x{:04X}", entity_type, device_index, property);
                }
            }
            _ => {}
        }
    }
}
