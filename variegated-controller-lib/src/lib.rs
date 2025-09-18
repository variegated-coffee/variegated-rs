#![no_std]

pub mod routine;
pub mod settings;

extern crate alloc;

use crc::{Crc, CRC_32_ISCSI};
use defmt::{error, info, warn, Format};
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, RawMutex};
use embassy_sync::channel::{Receiver};
use embassy_sync::mutex::Mutex;
use embassy_sync::pubsub::Publisher;
use embassy_sync::watch;
use embassy_time::{Instant, Timer};
use heapless::FnvIndexMap;
use movavg::MovAvg;
use postcard::{from_bytes, from_bytes_crc32, to_slice, to_slice_crc32};
use sequential_storage::map::{SerializationError, Value};
use variegated_control_algorithm::pid::{PidCtrl, PidIn, PidOut};
use variegated_hal::{Boiler, Group, PeripheralRegistry};
use variegated_controller_types::{BoilerConfiguration, BoilerControlTarget, BoilerIndex, BoilerStatus, CommsStatus, Configuration, GroupConfiguration, GroupIndex, PeripheralStatus, FlowRateType, GroupBrewControlTarget, GroupStatus, MachineCommand, Output, PidLimits, PidParameterTarget, PidParameters, PidTerm, PressureType, RoutineExecutionStatus, RoutineIndex, SingleBoilerSingleGroupControllerState, Status, KalmanParameters};
use crate::routine::{RoutineParameters, RoutineExecutionContext, InMemoryRoutineRepository};
use variegated_controller_types::SingleBoilerSingleGroupControllerBoilers::{BrewBoiler, VirtualSteamBoiler};
use variegated_controller_types::SingleGroupControllerGroups::SingleGroup;
use variegated_hal::scale::ScaleConfiguration;
use crate::settings::SettingsStorage;

fn limited_pid() -> PidCtrl<f32> {
    let mut pid = PidCtrl::default();
    pid.limits.try_set_lower(0.0).unwrap();
    pid.limits.try_set_upper(100.0).unwrap();
    pid
}

#[derive(Clone, Copy, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SingleBoilerSingleGroupPidParameters {
    pub boiler_pressure_params: PidParameters,
    pub boiler_temperature_params: PidParameters,
    pub pump_flow_rate_params: PidParameters,
    pub pump_pressure_params: PidParameters,
    pub pump_output_flow_rate_params: PidParameters,
}

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SingleBoilerSingleGroupPersistentConfiguration {
    pub brew_boiler_control_target: BoilerControlTarget,
    pub steam_boiler_control_target: BoilerControlTarget,
    pub pid_parameters: SingleBoilerSingleGroupPidParameters,
    pub temperature_sensor_kalman_parameters: Option<KalmanParameters>,
    pub pressure_sensor_kalman_parameters: Option<KalmanParameters>,
    pub pump_tacho_pulses_per_liter: Option<f32>,
    pub flow_sensor_pulses_per_liter: Option<f32>,
}

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SingleBoilerSingleGroupEphemeralConfiguration {
    pub group_brew_control_target: GroupBrewControlTarget,
}

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SingleBoilerSingleGroupConfiguration {
    pub brew_boiler_control_target: BoilerControlTarget,
    pub steam_boiler_control_target: BoilerControlTarget,
    pub group_brew_control_target: GroupBrewControlTarget,
    pub pid_parameters: SingleBoilerSingleGroupPidParameters,
    pub temperature_sensor_kalman_parameters: Option<KalmanParameters>,
    pub pressure_sensor_kalman_parameters: Option<KalmanParameters>,
    pub pump_tacho_pulses_per_liter: Option<f32>,
    pub flow_sensor_pulses_per_liter: Option<f32>,
}

impl<'a> Value<'a> for SingleBoilerSingleGroupPersistentConfiguration {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        let crc = Crc::<u32>::new(&CRC_32_ISCSI);

        info!("Serializing SingleBoilerSingleGroupConfiguration");

        let slice = match to_slice_crc32(self, buffer, crc.digest()) {
            Ok(bytes) => Ok(bytes.len()),
            Err(postcard::Error::SerializeBufferFull) => {
                warn!("Serialization buffer too small");

                Err(SerializationError::BufferTooSmall)
            },
            Err(_) => {
                warn!("Serialization error");

                Err(SerializationError::InvalidData)
            },
        };

        info!("Serialized SingleBoilerSingleGroupConfiguration, len = {}", slice.clone().unwrap_or(0));

        slice
    }

    fn deserialize_from(buffer: &'a [u8]) -> Result<Self, SerializationError>
    where
        Self: Sized
    {
        info!("Deserializing configuration");

        let crc = Crc::<u32>::new(&CRC_32_ISCSI);

        let v = match from_bytes_crc32(buffer, crc.digest()) {
            Ok(value) => Ok(value),
            Err(postcard::Error::DeserializeUnexpectedEnd) => {
                warn!("Deserialization buffer too small");

                Err(SerializationError::InvalidFormat)
            },
            Err(postcard::Error::DeserializeBadEnum) => {
                warn!("Deserialization bad enum");

                Err(SerializationError::InvalidFormat)
            },
            Err(_) => {
                warn!("Deserialization error");
                Err(SerializationError::InvalidFormat)
            },
        };

        let vc = v.clone();
        if vc.is_err() {
            warn!("Deserialization failed");
        }

        if let Ok(vc) = vc {
            info!("Deserialized configuration");
        }

        v
    }
}

impl Default for SingleBoilerSingleGroupEphemeralConfiguration {
    fn default() -> Self {
        Self {
            group_brew_control_target: GroupBrewControlTarget::FixedDutyCycle(100),
        }
    }
}

impl Default for SingleBoilerSingleGroupPersistentConfiguration {
    fn default() -> Self {
        let mut pid_parameters = SingleBoilerSingleGroupPidParameters::default();

        pid_parameters.boiler_temperature_params = PidParameters {
            kp: PidTerm::new(3.0, PidLimits::default()),
            ki: PidTerm::new(0.01, PidLimits::new_with_limits(-10.0, 10.0).unwrap()),
            kd: PidTerm::new(30.0, PidLimits::new_with_limits(-10.0, 10.0).unwrap())
        };
        pid_parameters.pump_flow_rate_params = PidParameters {
            kp: PidTerm::new(10.0, PidLimits::default() ),
            ki: PidTerm::new(0.01, PidLimits::new_with_limits(-50.0, 80.0).unwrap() ),
            kd: PidTerm::new(30.0, PidLimits::new_with_limits(-10.0, 10.0).unwrap() )
        };
        pid_parameters.pump_output_flow_rate_params = PidParameters {
            kp: PidTerm::new(10.0, PidLimits::default() ),
            ki: PidTerm::new(0.01, PidLimits::new_with_limits(-50.0, 80.0).unwrap() ),
            kd: PidTerm::new(30.0, PidLimits::new_with_limits(-10.0, 10.0).unwrap() )
        };
        pid_parameters.pump_pressure_params = PidParameters {
            kp: PidTerm::new( 10.0, PidLimits::default() ),
            ki: PidTerm::new( 0.01, PidLimits::new_with_limits(-50.0, 80.0).unwrap() ),
            kd: PidTerm::new( 30.0, PidLimits::new_with_limits(-10.0, 10.0).unwrap() )
        };

        SingleBoilerSingleGroupPersistentConfiguration {
            brew_boiler_control_target: BoilerControlTarget::Temperature(110.0),
            steam_boiler_control_target: BoilerControlTarget::Off,
            pid_parameters,
            temperature_sensor_kalman_parameters: None,
            pressure_sensor_kalman_parameters: None,
            pump_tacho_pulses_per_liter: None,
            flow_sensor_pulses_per_liter: None,
        }
    }
}

impl Default for SingleBoilerSingleGroupConfiguration {
    fn default() -> Self {
        let mut pid_parameters = SingleBoilerSingleGroupPidParameters::default();

        pid_parameters.boiler_temperature_params = PidParameters {
            kp: PidTerm::new(3.0, PidLimits::default()),
            ki: PidTerm::new(0.01, PidLimits::new_with_limits(-10.0, 10.0).unwrap()),
            kd: PidTerm::new(30.0, PidLimits::new_with_limits(-10.0, 10.0).unwrap())
        };
        pid_parameters.pump_flow_rate_params = PidParameters {
            kp: PidTerm::new(10.0, PidLimits::default() ),
            ki: PidTerm::new(0.01, PidLimits::new_with_limits(-50.0, 80.0).unwrap() ),
            kd: PidTerm::new(30.0, PidLimits::new_with_limits(-10.0, 10.0).unwrap() )
        };
        pid_parameters.pump_output_flow_rate_params = PidParameters {
            kp: PidTerm::new(10.0, PidLimits::default() ),
            ki: PidTerm::new(0.01, PidLimits::new_with_limits(-50.0, 80.0).unwrap() ),
            kd: PidTerm::new(30.0, PidLimits::new_with_limits(-10.0, 10.0).unwrap() )
        };
        pid_parameters.pump_pressure_params = PidParameters {
            kp: PidTerm::new( 10.0, PidLimits::default() ),
            ki: PidTerm::new( 0.01, PidLimits::new_with_limits(-50.0, 80.0).unwrap() ),
            kd: PidTerm::new( 30.0, PidLimits::new_with_limits(-10.0, 10.0).unwrap() )
        };

        SingleBoilerSingleGroupConfiguration {
            brew_boiler_control_target: BoilerControlTarget::Temperature(110.0),
            steam_boiler_control_target: BoilerControlTarget::Off,
            group_brew_control_target: GroupBrewControlTarget::FixedDutyCycle(100),
            pid_parameters,
            temperature_sensor_kalman_parameters: None,
            pressure_sensor_kalman_parameters: None,
            pump_tacho_pulses_per_liter: None,
            flow_sensor_pulses_per_liter: None,
        }
    }
}

pub struct SingleBoilerSingleGroupController<'a, ChannelM: RawMutex, M: RawMutex, SettingsStoreT: SettingsStorage<SingleBoilerSingleGroupPersistentConfiguration>, const N_CHANNEL: usize, const N_WATCH: usize, const N_SUBS: usize, const N_CONFIG_SUBS: usize> {
    command_channel_receiver: Receiver<'a, ChannelM, MachineCommand, N_CHANNEL>,
    status_channel_sender: Publisher<'a, ChannelM, Status, 1, N_SUBS, 1>,
    configuration_channel_sender: Publisher<'a, ChannelM, Configuration, 1, N_CONFIG_SUBS, 1>,
    boiler: Boiler<'a, M, N_WATCH>,
    group: Group<'a, M, N_WATCH>,
    state: SingleBoilerSingleGroupControllerState,
    boiler_pid: PidCtrl<f32>,
    pump_pid: PidCtrl<f32>,
    configuration_store: SettingsStoreT,
    persistent_configuration: SingleBoilerSingleGroupPersistentConfiguration,
    ephemeral_configuration: SingleBoilerSingleGroupEphemeralConfiguration,
    routine_repository: &'static Mutex<NoopRawMutex, InMemoryRoutineRepository>,
    current_routine: Option<RoutineExecutionContext<SingleBoilerSingleGroupControllerState, SingleBoilerSingleGroupConfiguration>>,
    previous_status: Option<Status>,
    temperature_movavg: MovAvg<f32, f32, 10>,
    brew_start_time: Option<Instant>,
    curve_start_time: Option<Instant>,
    comms_status: Option<CommsStatus>,
    comms_status_received_instant: Option<Instant>,
    peripheral_registry: &'a PeripheralRegistry<'a>,
}

impl <'a, ChannelM: RawMutex, M: RawMutex, SettingsStoreT: SettingsStorage<SingleBoilerSingleGroupPersistentConfiguration>,const N_CHANNEL: usize, const N_WATCH: usize, const N_SUBS: usize, const N_CONFIG_SUBS: usize> SingleBoilerSingleGroupController<'a, ChannelM, M, SettingsStoreT, N_CHANNEL, N_WATCH, N_SUBS, N_CONFIG_SUBS> {
    fn current_configuration(&self) -> SingleBoilerSingleGroupConfiguration {
        SingleBoilerSingleGroupConfiguration {
            brew_boiler_control_target: self.persistent_configuration.brew_boiler_control_target,
            steam_boiler_control_target: self.persistent_configuration.steam_boiler_control_target,
            group_brew_control_target: self.ephemeral_configuration.group_brew_control_target,
            pid_parameters: self.persistent_configuration.pid_parameters,
            temperature_sensor_kalman_parameters: self.persistent_configuration.temperature_sensor_kalman_parameters,
            pressure_sensor_kalman_parameters: self.persistent_configuration.pressure_sensor_kalman_parameters,
            pump_tacho_pulses_per_liter: self.persistent_configuration.pump_tacho_pulses_per_liter,
            flow_sensor_pulses_per_liter: self.persistent_configuration.flow_sensor_pulses_per_liter,
        }
    }
    pub fn new(
        command_channel_receiver: Receiver<'a, ChannelM, MachineCommand, N_CHANNEL>,
        status_channel_sender: Publisher<'a, ChannelM, Status, 1, N_SUBS, 1>,
        configuration_channel_sender: Publisher<'a, ChannelM, Configuration, 1, N_CONFIG_SUBS, 1>,
        boiler: Boiler<'a, M, N_WATCH>,
        group: Group<'a, M, N_WATCH>,
        mut settings_store: SettingsStoreT,
        routine_repository: &'static Mutex<NoopRawMutex, InMemoryRoutineRepository>,
        peripheral_registry: &'a PeripheralRegistry<'a>,
    ) -> Self {
        Self {
            command_channel_receiver,
            status_channel_sender,
            configuration_channel_sender,
            boiler,
            group,
            state: SingleBoilerSingleGroupControllerState::default(),
            boiler_pid: limited_pid(),
            pump_pid: limited_pid(),
            configuration_store: settings_store,
            persistent_configuration: SingleBoilerSingleGroupPersistentConfiguration::default(),
            ephemeral_configuration: SingleBoilerSingleGroupEphemeralConfiguration::default(),
            routine_repository,
            current_routine: None,
            previous_status: None,
            temperature_movavg: MovAvg::default(),
            brew_start_time: None,
            curve_start_time: None,
            comms_status: None,
            comms_status_received_instant: None,
            peripheral_registry,
        }
    }

    pub async fn task(&mut self) {
        let mut last_pid_update = Instant::now();
        let mut last_configuration = self.current_configuration();

        loop {
            self.persistent_configuration = self.configuration_store.load_settings().await.unwrap_or_default();
            
            // Check if configuration changed and publish if it did
            let current_config = self.current_configuration();
            if current_config != last_configuration {
                let config: Configuration = current_config.clone().into();
                self.configuration_channel_sender.publish_immediate(config);
                last_configuration = current_config;
            }

            while !self.command_channel_receiver.is_empty() {
                let command = self.command_channel_receiver.try_receive();
                if let Ok(command) = command {
                    self.handle_command(command).await;
                    
                    // Check if configuration changed after handling command
                    let current_config = self.current_configuration();
                    if current_config != last_configuration {
                        let config: Configuration = current_config.clone().into();
                        self.configuration_channel_sender.publish_immediate(config);
                        last_configuration = current_config;
                    }
                }
            }

            if let Some(routine) = &mut self.current_routine {
                if routine.finished_executing {
                    info!("Routine finished executing");
                    self.handle_routine_exit().await;
                } else if let Some(status) = self.previous_status.as_ref() {
                    if let Some(command) = routine.step(status, None) {
                        self.handle_command(command).await;
                    }
                }
            }

            let (actual_boiler_control_target, actual_pump_control_target) = self.get_control_targets();

            let next_pid_update = Instant::now();
            let delta_t = (next_pid_update - last_pid_update).as_millis() as f32;
            last_pid_update = next_pid_update;

            let boiler_pid_out = self.update_boiler(actual_boiler_control_target, delta_t).await;
            let pump_pid_out = self.update_pump(actual_pump_control_target, delta_t).await;

            self.send_status(boiler_pid_out, pump_pid_out).await;

            Timer::after_millis(100).await;
        }
    }

    async fn update_pump(&mut self, actual_pump_control_target: GroupBrewControlTarget, delta_t: f32) -> Output {
        // Calculate elapsed time for curve evaluation if needed
        let elapsed_seconds = self.curve_start_time
            .map(|start| {
                let duration = Instant::now().saturating_duration_since(start);
                duration.as_secs() as f32 + (duration.as_millis() % 1000) as f32 / 1000.0
            })
            .unwrap_or(0.0);
        
        let pump_pv = match actual_pump_control_target {
            GroupBrewControlTarget::GroupFlowRate(target) => {
                self.pump_pid.setpoint = target as f32;
                self.pump_pid.set_parameters(self.persistent_configuration.pid_parameters.pump_flow_rate_params);

                self.group.get_input_flow_rate().unwrap_or(0.0) as f32
            },
            GroupBrewControlTarget::GroupFlowRateCurve(curve) => {
                let target = curve.evaluate(elapsed_seconds);
                self.pump_pid.setpoint = target;
                self.pump_pid.set_parameters(self.persistent_configuration.pid_parameters.pump_flow_rate_params);
                
                self.group.get_input_flow_rate().unwrap_or(0.0) as f32
            },
            GroupBrewControlTarget::Pressure(target) => {
                self.pump_pid.setpoint = target as f32;
                self.pump_pid.set_parameters(self.persistent_configuration.pid_parameters.pump_pressure_params);

                self.group.get_pressure().unwrap_or(0.0) as f32
            },
            GroupBrewControlTarget::PressureCurve(curve) => {
                let target = curve.evaluate(elapsed_seconds);
                self.pump_pid.setpoint = target;
                self.pump_pid.set_parameters(self.persistent_configuration.pid_parameters.pump_pressure_params);
                
                self.group.get_pressure().unwrap_or(0.0) as f32
            },
            GroupBrewControlTarget::OutputFlowRate(target) => {
                self.pump_pid.setpoint = target as f32;
                self.pump_pid.set_parameters(self.persistent_configuration.pid_parameters.pump_output_flow_rate_params);

                self.group.get_output_flow_rate().unwrap_or(0.0) as f32
            },
            GroupBrewControlTarget::OutputFlowRateCurve(curve) => {
                let target = curve.evaluate(elapsed_seconds);
                self.pump_pid.setpoint = target;
                self.pump_pid.set_parameters(self.persistent_configuration.pid_parameters.pump_output_flow_rate_params);
                
                self.group.get_output_flow_rate().unwrap_or(0.0) as f32
            },
            _ => 0.0,
        };

        let pump_pid_out = self.pump_pid.step(PidIn::new(pump_pv, delta_t));

        match actual_pump_control_target {
            GroupBrewControlTarget::Off => {
                self.group.set_pump_duty_cycle(0).await;
                Output::Off
            },
            GroupBrewControlTarget::FullOn => {
                self.group.set_pump_duty_cycle(100).await;
                Output::FixedDutyCycle(100)
            },
            GroupBrewControlTarget::FixedDutyCycle(duty_cycle) => {
                self.group.set_pump_duty_cycle(duty_cycle).await;
                info!("Fixed duty cycle target: {}", duty_cycle);
                Output::FixedDutyCycle(duty_cycle)
            }
            GroupBrewControlTarget::FixedDutyCycleCurve(curve) => {
                let target_duty_cycle = curve.evaluate(elapsed_seconds).clamp(0.0, 100.0) as u8;
                info!("Fixed duty cycle curve target: {}", target_duty_cycle);
                info!("Curve start: {:?} elapsed time: {} seconds", self.curve_start_time, elapsed_seconds);
                self.group.set_pump_duty_cycle(target_duty_cycle).await;
                Output::FixedDutyCycle(target_duty_cycle)
            }
            _ => {
                info!("PID target: {}", pump_pid_out.out);
                self.group.set_pump_duty_cycle(pump_pid_out.out as u8).await;
                Output::PidOutput(pump_pid_out)
            },
        }
    }

    async fn update_boiler(&mut self, actual_boiler_control_target: BoilerControlTarget, delta_t: f32) -> Output {
        let mut boiler_pv = match actual_boiler_control_target {
            BoilerControlTarget::Temperature(target) => {
                self.boiler_pid.setpoint = target as f32;
                self.boiler_pid.set_parameters(self.persistent_configuration.pid_parameters.boiler_temperature_params);

                self.boiler.get_temperature().unwrap_or(0.0) as f32
            }
            BoilerControlTarget::Pressure(target) => {
                self.boiler_pid.setpoint = target as f32;
                self.boiler_pid.set_parameters(self.persistent_configuration.pid_parameters.boiler_pressure_params);

                self.boiler.get_pressure().unwrap_or(0.0) as f32
            }
            _ => 0.0,
        };
        
        if let Ok(pv) = self.temperature_movavg.try_feed(boiler_pv) {
            boiler_pv = pv;
        }

        let boiler_pid_out = self.boiler_pid.step(PidIn::new(boiler_pv, delta_t));

        match actual_boiler_control_target {
            BoilerControlTarget::Off => {
                self.boiler.set_heating_element_duty_cycle(0).await;
                Output::Off
            },
            _ => {
                self.boiler.set_heating_element_duty_cycle(boiler_pid_out.out as u8).await;
                Output::PidOutput(boiler_pid_out)
            },
        }
    }

    fn get_control_targets(&mut self) -> (BoilerControlTarget, GroupBrewControlTarget) {
        let (actual_boiler_control_target, actual_pump_control_target) = match self.state {
            SingleBoilerSingleGroupControllerState::Brewing => {
                (self.persistent_configuration.brew_boiler_control_target, self.ephemeral_configuration.group_brew_control_target)
            }
            SingleBoilerSingleGroupControllerState::PumpingToWaterTap => {
                (self.persistent_configuration.brew_boiler_control_target, GroupBrewControlTarget::FullOn)
            },
            SingleBoilerSingleGroupControllerState::BrewModeIdle => {
                (self.persistent_configuration.brew_boiler_control_target, GroupBrewControlTarget::Off)
            }
            SingleBoilerSingleGroupControllerState::SteamModeIdle => {
                (self.persistent_configuration.steam_boiler_control_target, GroupBrewControlTarget::Off)
            }
            SingleBoilerSingleGroupControllerState::PowerSave => {
                (BoilerControlTarget::Off, GroupBrewControlTarget::Off)
            }
        };
        (actual_boiler_control_target, actual_pump_control_target)
    }

    async fn send_status(&mut self, boiler_output: Output, pump_output: Output) {
        let (brew_boiler_output, steam_boiler_output) = match self.state {
            SingleBoilerSingleGroupControllerState::SteamModeIdle => (Output::Off, boiler_output.clone()),
            _ => (boiler_output.clone(), Output::Off),
        };
        
        let brew_boiler_status = BoilerStatus {
            temperature: self.boiler.get_temperature(),
            pressure: self.boiler.get_pressure(),
            output: brew_boiler_output,
            control_target: self.persistent_configuration.brew_boiler_control_target,
        };

        let virtual_steam_boiler_status = BoilerStatus {
            temperature: self.boiler.get_temperature(),
            pressure: self.boiler.get_pressure(),
            output: steam_boiler_output,
            control_target: self.persistent_configuration.steam_boiler_control_target,
        };

        let group_status = GroupStatus {
            is_brewing: self.state == SingleBoilerSingleGroupControllerState::Brewing,
            three_way_valve_open: self.group.get_three_way_valve_open(),
            brew_time: self.brew_start_time.map(|start| start.elapsed().into()),
            input_flow_rate: self.group.get_input_flow_rate(),
            output_flow_rate: self.group.get_output_flow_rate(),
            output_weight: self.group.get_output_weight(),
            pressure: self.group.get_pressure(),
            temperature: self.group.get_temperature(),
            pump_output: pump_output.clone(),
            control_target: self.ephemeral_configuration.group_brew_control_target,
        };

        // Calculate current timestamp if we have comms_status
        let comms_status = if let (Some(status), Some(received_instant)) = 
            (&self.comms_status, self.comms_status_received_instant) {
            
            // Calculate elapsed time since reception
            let elapsed = Instant::now().saturating_duration_since(received_instant);
            let current_timestamp = status.timestamp.map(|ts| ts + elapsed.as_secs());
            
            Some(CommsStatus {
                timestamp: current_timestamp,
                wifi_connected: status.wifi_connected,
            })
        } else {
            self.comms_status.clone()
        };

        let routine_execution = self.current_routine.as_ref().map(|rxc| {
            let step_elapsed_time = rxc.step_start_time.map(|start| {
                let elapsed = start.elapsed();
                core::time::Duration::from_secs(elapsed.as_secs())
            });
            let total_elapsed_time = rxc.execution_start_time.map(|start| {
                let elapsed = start.elapsed();
                core::time::Duration::from_secs(elapsed.as_secs())
            });
            RoutineExecutionStatus {
                routine_index: rxc.routine_index,
                current_step: rxc.current_step,
                step_elapsed_time,
                total_elapsed_time,
                resolved_parameters: rxc.parameters.clone(),
            }
        });

        let status = Status {
            boiler_statuses: FnvIndexMap::from_iter([(BrewBoiler.as_index(), brew_boiler_status), (VirtualSteamBoiler.as_index(), virtual_steam_boiler_status)]),
            group_statuses: FnvIndexMap::from_iter([(SingleGroup.as_index(), group_status)]),
            mode: Default::default(),
            routine_execution,
            comms_status,
            peripheral_status: self.peripheral_registry.get_peripheral_status(),
        };

        self.status_channel_sender.publish_immediate(status.clone());

        self.previous_status = Some(status);
    }

    async fn handle_command(&mut self, command: MachineCommand) {
        match command {
            MachineCommand::StartBrewing(_) => {
                self.transition_to_state(SingleBoilerSingleGroupControllerState::Brewing).await;
            }
            MachineCommand::StopBrewing(_) => {
                self.transition_to_state(SingleBoilerSingleGroupControllerState::BrewModeIdle).await;
            }
            MachineCommand::StartPumpingToWaterTap(_) => {
                self.transition_to_state(SingleBoilerSingleGroupControllerState::PumpingToWaterTap).await;
            }
            MachineCommand::StopPumpingToWaterTap(_) => {
                self.transition_to_state(SingleBoilerSingleGroupControllerState::BrewModeIdle).await;
            }
            MachineCommand::SetBoilerControlTarget(boiler_index, control_target) => {
                info!("Setting boiler control target for boiler {} to {:?}", boiler_index, control_target);
                match boiler_index {
                    0 => {
                        self.persistent_configuration.brew_boiler_control_target = control_target;
                        self.configuration_store.save_settings(&self.persistent_configuration).await.ok();
                    },
                    1 => {
                        self.persistent_configuration.steam_boiler_control_target = control_target;
                        self.configuration_store.save_settings(&self.persistent_configuration).await.ok();
                    },
                    _ => {
                        error!("Invalid boiler index: {}", boiler_index);
                    }
                }
            }
            MachineCommand::SetGroupBrewControlTarget(group_index, control_target) => {
                info!("Setting group brew control target for group {} to {:?}", group_index, control_target);
                if group_index == 0 {
                    // Check if this is a curve target and record start time
                    match control_target {
                        GroupBrewControlTarget::GroupFlowRateCurve(_) |
                        GroupBrewControlTarget::PressureCurve(_) |
                        GroupBrewControlTarget::OutputFlowRateCurve(_) |
                        GroupBrewControlTarget::FixedDutyCycleCurve(_) => {
                            self.curve_start_time = Some(Instant::now());
                            info!("Starting curve control");
                        }
                        _ => {
                            // Reset curve start time for non-curve targets
                            self.curve_start_time = None;
                        }
                    }
                    self.ephemeral_configuration.group_brew_control_target = control_target;
                } else {
                    error!("Invalid group index: {}", group_index);
                }
            }
            MachineCommand::SetPidParameters(target, params) => {
                match target {
                    PidParameterTarget::BoilerPressure(_) => {
                        self.persistent_configuration.pid_parameters.boiler_pressure_params = params;
                    }
                    PidParameterTarget::BoilerTemperature(_) => {
                        self.persistent_configuration.pid_parameters.boiler_temperature_params = params;
                    }
                    PidParameterTarget::GroupFlowRate(_) => {
                        self.persistent_configuration.pid_parameters.pump_flow_rate_params = params;
                    }
                    PidParameterTarget::GroupPressure(_) => {
                        self.persistent_configuration.pid_parameters.pump_pressure_params = params;
                    }
                    PidParameterTarget::GroupOutputFlowRate(_) => {
                        self.persistent_configuration.pid_parameters.pump_output_flow_rate_params = params;
                    }
                }
                // Save after updating PID parameters
                self.configuration_store.save_settings(&self.persistent_configuration).await.ok();
            }
            MachineCommand::RunRoutine(index, params) => {
                info!("Running routine {} with {} parameters", index, params.as_ref().map(|p| p.len()).unwrap_or(0));
                self.handle_routine_start(index, params).await;
            }
            MachineCommand::CancelRoutine => {
                info!("Cancelling routine");
                self.handle_routine_exit().await;
            }
            MachineCommand::EnableBoiler(boiler_index) => {
                if boiler_index == 0 && self.state == SingleBoilerSingleGroupControllerState::SteamModeIdle {
                    info!("Enabling brew boiler");
                    self.transition_to_state(SingleBoilerSingleGroupControllerState::BrewModeIdle).await;
                } else if boiler_index == 1 && self.state == SingleBoilerSingleGroupControllerState::BrewModeIdle {
                    info!("Enabling steam boiler");
                    self.transition_to_state(SingleBoilerSingleGroupControllerState::SteamModeIdle).await;
                } else {
                    warn!("Invalid boiler index or state for enabling boiler: {} Current state: {:?}", boiler_index, self.state);
                }
            }
            MachineCommand::DisableBoiler(boiler_index) => {
                if boiler_index == 1 && self.state == SingleBoilerSingleGroupControllerState::SteamModeIdle {
                    info!("Going back to brew mode");
                    self.transition_to_state(SingleBoilerSingleGroupControllerState::BrewModeIdle).await;
                } else if boiler_index == 0 && self.state == SingleBoilerSingleGroupControllerState::BrewModeIdle {
                    info!("Going in to power save mode");
                    self.transition_to_state(SingleBoilerSingleGroupControllerState::PowerSave).await;
                } else {
                    warn!("Invalid boiler index or state for disabling boiler: {} Current state: {:?}", boiler_index, self.state);
                }
            },
            MachineCommand::TareGroupScale(group_index) => {
                if group_index == 0 {
                    info!("Taring group scale");
                    let _ = self.group.scale_tare().await;
                } else {
                    error!("Invalid group index for taring scale: {}", group_index);
                }
            }
            MachineCommand::ZeroCalibrateGroupScale(group_index) => {
                if group_index == 0 {
                    info!("Zero calibrating group scale");
                    let _ = self.group.scale_zero_calibration().await;
                } else {
                    error!("Invalid group index for zero calibrating scale: {}", group_index);
                }
            }
            MachineCommand::CalibrateGroupScale100g(group_index) => {
                if group_index == 0 {
                    info!("Calibrating group scale with 100g");
                    let _ = self.group.scale_reference_weight_calibration(100).await;
                } else {
                    error!("Invalid group index for 100g calibrating scale: {}", group_index);
                }
            }
            MachineCommand::UpdateCommsStatus(status) => {
                info!("Updating comms status: wifi={}, timestamp={:?}", status.wifi_connected, status.timestamp);
                self.comms_status = Some(status);
                self.comms_status_received_instant = Some(Instant::now());
            }
        }
    }

    async fn transition_to_state(&mut self, new_state: SingleBoilerSingleGroupControllerState) {
        if self.state != new_state {
            info!("Transitioning from {:?} to {:?}", self.state, new_state);
            let old_state = self.state;
            self.state = new_state;

            match (old_state, new_state) {
                (SingleBoilerSingleGroupControllerState::BrewModeIdle, SingleBoilerSingleGroupControllerState::Brewing) => {
                    info!("Starting brewing");
                    self.group.set_brew_state(true).await;
                    self.started_brewing().await;
                }
                (SingleBoilerSingleGroupControllerState::Brewing, SingleBoilerSingleGroupControllerState::BrewModeIdle) => {
                    info!("Stopping brewing");
                    self.group.set_brew_state(false).await;
                    self.stopped_brewing().await;
                }
                _ => {}
            }
        }
    }
    
    async fn started_brewing(&mut self) {
        self.brew_start_time = Some(Instant::now());
        self.boiler_pid.ki.accumulate += 50.0; // Initial accumulation to compensate for initial temperature drop
        let _ = self.group.scale_set_configuration(ScaleConfiguration {
            zero_tracking: Some(false),
            smoothing: Some(true)
        }).await;
        let _ = self.group.scale_tare().await;
    }
    
    async fn stopped_brewing(&mut self) {
        self.brew_start_time = None;
        self.curve_start_time = None;  // Reset curve start time when brewing stops
        let _ = self.group.scale_set_configuration(ScaleConfiguration {
            zero_tracking: Some(true),
            smoothing: Some(false)
        }).await;
    }

    async fn handle_routine_start(&mut self, routine_index: RoutineIndex, runtime_params: Option<RoutineParameters>) {
        if self.current_routine.is_some() {
            //warn!("Cannot run routine, already executing a routine");
            return;
        }
        let repo = self.routine_repository.lock().await;
        let routine = repo.get_routine(routine_index);

        if let Some(routine) = routine {
            info!("Running routine");
            self.current_routine = Some(RoutineExecutionContext::new(routine_index, routine.clone(), self.state, self.current_configuration(), runtime_params));
            info!("Routine started");
        } else {
            error!("Routine not found: {}", routine_index);
        }
    }

    async fn handle_routine_exit(&mut self) {
        if let Some(routine) = self.current_routine.take() {
            info!("Routine execution finished, saving state and configuration");
            // Restore saved configuration by splitting into persistent and ephemeral parts
            let saved_config = &routine.saved_configuration;
            self.persistent_configuration = SingleBoilerSingleGroupPersistentConfiguration {
                brew_boiler_control_target: saved_config.brew_boiler_control_target,
                steam_boiler_control_target: saved_config.steam_boiler_control_target,
                pid_parameters: saved_config.pid_parameters,
                temperature_sensor_kalman_parameters: saved_config.temperature_sensor_kalman_parameters,
                pressure_sensor_kalman_parameters: saved_config.pressure_sensor_kalman_parameters,
                pump_tacho_pulses_per_liter: saved_config.pump_tacho_pulses_per_liter,
                flow_sensor_pulses_per_liter: saved_config.flow_sensor_pulses_per_liter,
            };
            self.ephemeral_configuration.group_brew_control_target = saved_config.group_brew_control_target;
            // Save the restored persistent configuration
            self.configuration_store.save_settings(&self.persistent_configuration).await.ok();
            self.curve_start_time = None;  // Reset curve start time when routine exits
            self.transition_to_state(routine.saved_state).await;
        } else {
            warn!("No routine to exit");
        }
    }
}

impl From<SingleBoilerSingleGroupConfiguration> for Configuration {
    fn from(config: SingleBoilerSingleGroupConfiguration) -> Self {
        let mut configuration = Configuration::default();
        
        // Add brew boiler configuration
        let brew_boiler_config = BoilerConfiguration {
            temperature_pid_parameters: config.pid_parameters.boiler_temperature_params.clone(),
            pressure_pid_parameters: config.pid_parameters.boiler_pressure_params.clone(),
            control_target: config.brew_boiler_control_target,
        };
        configuration.insert_boiler_configuration(BrewBoiler.as_index(), brew_boiler_config);
        
        // Add virtual steam boiler configuration
        let steam_boiler_config = BoilerConfiguration {
            temperature_pid_parameters: PidParameters::default(), // Virtual steam boiler doesn't have separate PID
            pressure_pid_parameters: PidParameters::default(),
            control_target: config.steam_boiler_control_target,
        };
        configuration.insert_boiler_configuration(VirtualSteamBoiler.as_index(), steam_boiler_config);
        
        // Add group configuration
        let group_config = GroupConfiguration {
            flow_rate_pid_parameters: config.pid_parameters.pump_flow_rate_params.clone(),
            output_flow_rate_pid_parameters: config.pid_parameters.pump_output_flow_rate_params.clone(),
            pressure_pid_parameters: config.pid_parameters.pump_pressure_params.clone(),
            brew_control_target: config.group_brew_control_target,
        };
        configuration.insert_group_configuration(SingleGroup.as_index(), group_config);
        
        configuration
    }
}