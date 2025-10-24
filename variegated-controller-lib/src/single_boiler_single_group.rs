#![no_std]

extern crate alloc;

use alloc::vec::Vec;
use core::ops::DerefMut;
use crc::{Crc, CRC_32_ISCSI};
use defmt::{error, info, warn, Format};
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, RawMutex};
use embassy_sync::channel::Receiver;
use embassy_sync::mutex::Mutex;
use embassy_sync::pubsub::Publisher;
use embassy_sync::watch;
use embassy_time::{Instant, Timer};
use heapless::FnvIndexMap;
use movavg::MovAvg;
use postcard::{from_bytes, from_bytes_crc32, to_slice, to_slice_crc32};
use sequential_storage::map::{SerializationError, Value};
use variegated_control_algorithm::pid::{PidCtrl, PidIn, PidOut};
use variegated_hal::{Boiler, Group, Tank, PeripheralRegistry};
use variegated_controller_types::{BoilerConfiguration, BoilerControlMode, BoilerControlState, BoilerControlTargetValues, BoilerControlTargetValuesUpdate, BoilerIndex, BoilerStatus, CommsStatus, Configuration, GroupConfiguration, GroupIndex, InputVolumeType, PeripheralStatus, FlowRateType, GroupBrewControlMode, GroupBrewControlState, GroupBrewControlTargetValues, GroupBrewControlTargetValuesUpdate, GroupStatus, MachineCommand, Output, PidLimits, PidParameterTarget, PidParameters, PidTerm, PressureType, RoutineExecutionStatus, RoutineIndex, SingleBoilerSingleGroupControllerState, Status, KalmanParameters, TankConfiguration, TankStatus, RoutineParameters};
use crate::routine::{RoutineExecutionContext, InMemoryRoutineRepository, RoutineRepository};
use variegated_controller_types::SingleBoilerSingleGroupControllerBoilers::{BrewBoiler, VirtualSteamBoiler};
use variegated_controller_types::SingleGroupControllerGroups::SingleGroup;
use variegated_hal::scale::ScaleConfiguration;
use variegated_timekeeping::TimeKeeper;
use crate::settings::SettingsStorage;

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
    pub brew_boiler_control_state: BoilerControlState,
    pub steam_boiler_control_state: BoilerControlState,
    pub pid_parameters: SingleBoilerSingleGroupPidParameters,
    pub temperature_sensor_kalman_parameters: Option<KalmanParameters>,
    pub pressure_sensor_kalman_parameters: Option<KalmanParameters>,
    pub pump_tacho_pulses_per_liter: Option<f32>,
    pub flow_sensor_pulses_per_liter: Option<f32>,
}

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SingleBoilerSingleGroupEphemeralConfiguration {
    pub group_brew_control_state: GroupBrewControlState,
}

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SingleBoilerSingleGroupConfiguration {
    pub persistent: SingleBoilerSingleGroupPersistentConfiguration,
    pub ephemeral: SingleBoilerSingleGroupEphemeralConfiguration,
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
            group_brew_control_state: GroupBrewControlState {
                mode: GroupBrewControlMode::FixedDutyCycle,
                values: GroupBrewControlTargetValues {
                    duty_cycle: 100,
                    ..GroupBrewControlTargetValues::default()
                },
            },
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
            brew_boiler_control_state: BoilerControlState {
                mode: BoilerControlMode::Temperature,
                values: BoilerControlTargetValues {
                    target_temperature: 110.0,
                    target_pressure: 1.0,
                },
            },
            steam_boiler_control_state: BoilerControlState {
                mode: BoilerControlMode::Off,
                values: BoilerControlTargetValues::default(),
            },
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
        SingleBoilerSingleGroupConfiguration {
            persistent: SingleBoilerSingleGroupPersistentConfiguration::default(),
            ephemeral: SingleBoilerSingleGroupEphemeralConfiguration::default(),
        }
    }
}

pub struct SingleBoilerSingleGroupController<
    'a,
    ChannelM: RawMutex,
    M: RawMutex,
    SettingsStoreT: SettingsStorage<SingleBoilerSingleGroupPersistentConfiguration>,
    const N_CHANNEL: usize,
    const N_WATCH: usize,
    const N_SUBS: usize,
    const N_CONFIG_SUBS: usize
> {
    command_channel_receiver: Receiver<'a, ChannelM, MachineCommand, N_CHANNEL>,
    status_channel_sender: Publisher<'a, ChannelM, Status, 1, N_SUBS, 1>,
    configuration_channel_sender: Publisher<'a, ChannelM, Configuration, 1, N_CONFIG_SUBS, 1>,
    boiler: Boiler<'a, M, N_WATCH>,
    group: Group<'a, M, N_WATCH>,
    tank: Option<Tank<'a, M, N_WATCH>>,
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
    brew_start_input_volume: Option<InputVolumeType>,
    previous_brew: Option<crate::PreviousBrewInfo>,
    curve_start_time: Option<Instant>,
    comms_status: Option<CommsStatus>,
    comms_status_received_instant: Option<Instant>,
    peripheral_registry: &'a PeripheralRegistry<'a>,
}

impl<
    'a,
    ChannelM: RawMutex,
    M: RawMutex,
    SettingsStoreT: SettingsStorage<SingleBoilerSingleGroupPersistentConfiguration>,
    const N_CHANNEL: usize,
    const N_WATCH: usize,
    const N_SUBS: usize,
    const N_CONFIG_SUBS: usize
> SingleBoilerSingleGroupController<'a, ChannelM, M, SettingsStoreT, N_CHANNEL, N_WATCH, N_SUBS, N_CONFIG_SUBS> {
    fn current_configuration(&self) -> SingleBoilerSingleGroupConfiguration {
        SingleBoilerSingleGroupConfiguration {
            persistent: self.persistent_configuration.clone(),
            ephemeral: self.ephemeral_configuration.clone(),
        }
    }
    pub fn new(
        command_channel_receiver: Receiver<'a, ChannelM, MachineCommand, N_CHANNEL>,
        status_channel_sender: Publisher<'a, ChannelM, Status, 1, N_SUBS, 1>,
        configuration_channel_sender: Publisher<'a, ChannelM, Configuration, 1, N_CONFIG_SUBS, 1>,
        boiler: Boiler<'a, M, N_WATCH>,
        group: Group<'a, M, N_WATCH>,
        tank: Option<Tank<'a, M, N_WATCH>>,
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
            tank,
            state: SingleBoilerSingleGroupControllerState::default(),
            boiler_pid: super::limited_pid(),
            pump_pid: super::limited_pid(),
            configuration_store: settings_store,
            persistent_configuration: SingleBoilerSingleGroupPersistentConfiguration::default(),
            ephemeral_configuration: SingleBoilerSingleGroupEphemeralConfiguration::default(),
            routine_repository,
            current_routine: None,
            previous_status: None,
            temperature_movavg: MovAvg::default(),
            brew_start_time: None,
            brew_start_input_volume: None,
            previous_brew: None,
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

    async fn update_pump(&mut self, actual_pump_control_state: GroupBrewControlState, delta_t: f32) -> Output {
        // Calculate elapsed time for curve evaluation if needed
        let elapsed_seconds = self.curve_start_time
            .map(|start| {
                let duration = Instant::now().saturating_duration_since(start);
                duration.as_secs() as f32 + (duration.as_millis() % 1000) as f32 / 1000.0
            })
            .unwrap_or(0.0);

        let pump_pv = match actual_pump_control_state.mode {
            GroupBrewControlMode::GroupFlowRate => {
                self.pump_pid.setpoint = actual_pump_control_state.values.flow_rate as f32;
                self.pump_pid.set_parameters(self.persistent_configuration.pid_parameters.pump_flow_rate_params);

                self.group.get_input_flow_rate().unwrap_or(0.0) as f32
            },
            GroupBrewControlMode::GroupFlowRateCurve => {
                let target = actual_pump_control_state.values.flow_rate_curve.evaluate(elapsed_seconds);
                self.pump_pid.setpoint = target;
                self.pump_pid.set_parameters(self.persistent_configuration.pid_parameters.pump_flow_rate_params);

                self.group.get_input_flow_rate().unwrap_or(0.0) as f32
            },
            GroupBrewControlMode::Pressure => {
                self.pump_pid.setpoint = actual_pump_control_state.values.pressure as f32;
                self.pump_pid.set_parameters(self.persistent_configuration.pid_parameters.pump_pressure_params);

                self.group.get_pressure().unwrap_or(0.0) as f32
            },
            GroupBrewControlMode::PressureCurve => {
                let target = actual_pump_control_state.values.pressure_curve.evaluate(elapsed_seconds);
                self.pump_pid.setpoint = target;
                self.pump_pid.set_parameters(self.persistent_configuration.pid_parameters.pump_pressure_params);

                self.group.get_pressure().unwrap_or(0.0) as f32
            },
            GroupBrewControlMode::OutputFlowRate => {
                self.pump_pid.setpoint = actual_pump_control_state.values.output_flow_rate as f32;
                self.pump_pid.set_parameters(self.persistent_configuration.pid_parameters.pump_output_flow_rate_params);

                self.group.get_output_flow_rate().unwrap_or(0.0) as f32
            },
            GroupBrewControlMode::OutputFlowRateCurve => {
                let target = actual_pump_control_state.values.output_flow_rate_curve.evaluate(elapsed_seconds);
                self.pump_pid.setpoint = target;
                self.pump_pid.set_parameters(self.persistent_configuration.pid_parameters.pump_output_flow_rate_params);

                self.group.get_output_flow_rate().unwrap_or(0.0) as f32
            },
            _ => 0.0,
        };

        let pump_pid_out = self.pump_pid.step(PidIn::new(pump_pv, delta_t));

        match actual_pump_control_state.mode {
            GroupBrewControlMode::Off => {
                self.group.set_brewing_state(false, 0).await;
                Output::Off
            },
            GroupBrewControlMode::FullOn => {
                self.group.set_brewing_state(true, 100).await;
                Output::FixedDutyCycle(100)
            },
            GroupBrewControlMode::FixedDutyCycle => {
                let duty_cycle = actual_pump_control_state.values.duty_cycle;
                self.group.set_brewing_state(true, duty_cycle).await;
                info!("Fixed duty cycle target: {}", duty_cycle);
                Output::FixedDutyCycle(duty_cycle)
            }
            GroupBrewControlMode::FixedDutyCycleCurve => {
                let target_duty_cycle = actual_pump_control_state.values.duty_cycle_curve.evaluate(elapsed_seconds).clamp(0.0, 100.0) as u8;
                info!("Fixed duty cycle curve target: {}", target_duty_cycle);
                info!("Curve start: {:?} elapsed time: {} seconds", self.curve_start_time, elapsed_seconds);
                self.group.set_brewing_state(true, target_duty_cycle).await;
                Output::FixedDutyCycle(target_duty_cycle)
            }
            _ => {
                info!("PID target: {}", pump_pid_out.out);
                self.group.set_brewing_state(true, pump_pid_out.out as u8).await;
                Output::PidOutput(pump_pid_out)
            },
        }
    }

    async fn update_boiler(&mut self, actual_boiler_control_state: BoilerControlState, delta_t: f32) -> Output {
        let mut boiler_pv = match actual_boiler_control_state.mode {
            BoilerControlMode::Temperature => {
                self.boiler_pid.setpoint = actual_boiler_control_state.values.target_temperature as f32;
                self.boiler_pid.set_parameters(self.persistent_configuration.pid_parameters.boiler_temperature_params);

                self.boiler.get_temperature().unwrap_or(0.0) as f32
            }
            BoilerControlMode::Pressure => {
                self.boiler_pid.setpoint = actual_boiler_control_state.values.target_pressure as f32;
                self.boiler_pid.set_parameters(self.persistent_configuration.pid_parameters.boiler_pressure_params);

                self.boiler.get_pressure().unwrap_or(0.0) as f32
            }
            _ => 0.0,
        };

        if let Ok(pv) = self.temperature_movavg.try_feed(boiler_pv) {
            boiler_pv = pv;
        }

        let boiler_pid_out = self.boiler_pid.step(PidIn::new(boiler_pv, delta_t));

        match actual_boiler_control_state.mode {
            BoilerControlMode::Off => {
                self.boiler.set_heating_element_duty_cycle(0).await;
                Output::Off
            },
            _ => {
                self.boiler.set_heating_element_duty_cycle(boiler_pid_out.out as u8).await;
                Output::PidOutput(boiler_pid_out)
            },
        }
    }

    fn get_control_targets(&mut self) -> (BoilerControlState, GroupBrewControlState) {
        let (actual_boiler_control_state, actual_pump_control_state) = match self.state {
            SingleBoilerSingleGroupControllerState::Brewing => {
                (self.persistent_configuration.brew_boiler_control_state, self.ephemeral_configuration.group_brew_control_state)
            }
            SingleBoilerSingleGroupControllerState::PumpingToWaterTap => {
                let mut pump_state = self.ephemeral_configuration.group_brew_control_state;
                pump_state.mode = GroupBrewControlMode::FullOn;
                (self.persistent_configuration.brew_boiler_control_state, pump_state)
            },
            SingleBoilerSingleGroupControllerState::BrewModeIdle => {
                let mut pump_state = self.ephemeral_configuration.group_brew_control_state;
                pump_state.mode = GroupBrewControlMode::Off;
                (self.persistent_configuration.brew_boiler_control_state, pump_state)
            }
            SingleBoilerSingleGroupControllerState::SteamModeIdle => {
                let mut pump_state = self.ephemeral_configuration.group_brew_control_state;
                pump_state.mode = GroupBrewControlMode::Off;
                (self.persistent_configuration.steam_boiler_control_state, pump_state)
            }
            SingleBoilerSingleGroupControllerState::PowerSave => {
                let mut boiler_state = BoilerControlState::default();
                boiler_state.mode = BoilerControlMode::Off;
                let mut pump_state = GroupBrewControlState::default();
                pump_state.mode = GroupBrewControlMode::Off;
                (boiler_state, pump_state)
            }
        };
        (actual_boiler_control_state, actual_pump_control_state)
    }

    async fn send_status(&mut self, boiler_output: Output, pump_output: Output) {
        let (brew_boiler_output, steam_boiler_output) = match self.state {
            SingleBoilerSingleGroupControllerState::SteamModeIdle => (Output::Off, boiler_output.clone()),
            _ => (boiler_output.clone(), Output::Off),
        };

        let brew_boiler_status = BoilerStatus {
            temperature: self.boiler.get_temperature(),
            pressure: self.boiler.get_pressure(),
            water_level: self.boiler.get_water_level(),
            output: brew_boiler_output,
            control_state: self.persistent_configuration.brew_boiler_control_state,
        };

        let virtual_steam_boiler_status = BoilerStatus {
            temperature: self.boiler.get_temperature(),
            pressure: self.boiler.get_pressure(),
            water_level: self.boiler.get_water_level(),
            output: steam_boiler_output,
            control_state: self.persistent_configuration.steam_boiler_control_state,
        };

        let brew_input_volume = match (self.brew_start_input_volume, self.group.get_input_volume()) {
            (Some(start_volume), Some(current_volume)) => Some(current_volume - start_volume),
            _ => None,
        };

        let group_status = GroupStatus {
            is_brewing: self.state == SingleBoilerSingleGroupControllerState::Brewing,
            three_way_valve_open: self.group.get_three_way_valve_open(),
            brew_time: self.brew_start_time.map(|start| start.elapsed().into()),
            brew_input_volume,
            input_flow_rate: self.group.get_input_flow_rate(),
            input_volume: self.group.get_input_volume(),
            output_flow_rate: self.group.get_output_flow_rate(),
            output_weight: self.group.get_output_weight(),
            pressure: self.group.get_pressure(),
            temperature: self.group.get_temperature(),
            pump_output: pump_output.clone(),
            control_state: self.ephemeral_configuration.group_brew_control_state,
            previous_brew: self.previous_brew.map(|info| info.into()),
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

        // Create tank statuses map - only include tank if it exists
        let tank_statuses = if let Some(ref mut tank) = self.tank {
            FnvIndexMap::from_iter([(0, TankStatus {
                water_level: tank.get_water_level(),
            })])
        } else {
            FnvIndexMap::new()
        };

        let status = Status {
            boiler_statuses: FnvIndexMap::from_iter([(BrewBoiler.as_index(), brew_boiler_status), (VirtualSteamBoiler.as_index(), virtual_steam_boiler_status)]),
            group_statuses: FnvIndexMap::from_iter([(SingleGroup.as_index(), group_status)]),
            water_tap_statuses: FnvIndexMap::new(),
            tank_statuses,
            mode: Default::default(),
            routine_execution,
            comms_status,
            peripheral_status: self.peripheral_registry.get_peripheral_status(),
            current_local_time: TimeKeeper::now_local().map(|t| t.naive_local()),
        };

        self.status_channel_sender.publish_immediate(status.clone());

        self.previous_status = Some(status);
    }

    async fn handle_command(&mut self, command: MachineCommand) {
        match command {
            MachineCommand::RunRoutine(index, params) => {
                info!("Running routine {} with {} parameters", index, params.as_ref().map(|p| p.len()).unwrap_or(0));
                self.handle_routine_start(index, params).await;
            }
            MachineCommand::CancelRoutine => {
                info!("Cancelling routine");
                self.handle_routine_exit().await;
            }
            _ => {
                // All other commands delegate to the finally handler
                self.handle_routine_finally_commands(command).await;
            }
        }
    }

    async fn handle_routine_finally_commands(&mut self, command: MachineCommand) {
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
            MachineCommand::SetBoilerControlTarget(boiler_index, mode, values_update) => {
                info!("Setting boiler control mode for boiler {} to {:?} with values {:?}", boiler_index, mode, values_update);
                match boiler_index {
                    0 => {
                        self.persistent_configuration.brew_boiler_control_state.mode = mode;
                        if let Some(update) = values_update {
                            if let Some(temp) = update.temperature {
                                self.persistent_configuration.brew_boiler_control_state.values.target_temperature = temp;
                            }
                            if let Some(pressure) = update.pressure {
                                self.persistent_configuration.brew_boiler_control_state.values.target_pressure = pressure;
                            }
                        }
                        self.configuration_store.save_settings(&self.persistent_configuration).await.ok();
                    },
                    1 => {
                        self.persistent_configuration.steam_boiler_control_state.mode = mode;
                        if let Some(update) = values_update {
                            if let Some(temp) = update.temperature {
                                self.persistent_configuration.steam_boiler_control_state.values.target_temperature = temp;
                            }
                            if let Some(pressure) = update.pressure {
                                self.persistent_configuration.steam_boiler_control_state.values.target_pressure = pressure;
                            }
                        }
                        self.configuration_store.save_settings(&self.persistent_configuration).await.ok();
                    },
                    _ => {
                        error!("Invalid boiler index: {}", boiler_index);
                    }
                }
            }
            MachineCommand::SetBoilerControlTargetValues(boiler_index, update) => {
                info!("Setting boiler control values for boiler {} to {:?}", boiler_index, update);
                match boiler_index {
                    0 => {
                        if let Some(temp) = update.temperature {
                            self.persistent_configuration.brew_boiler_control_state.values.target_temperature = temp;
                        }
                        if let Some(pressure) = update.pressure {
                            self.persistent_configuration.brew_boiler_control_state.values.target_pressure = pressure;
                        }
                        self.configuration_store.save_settings(&self.persistent_configuration).await.ok();
                    },
                    1 => {
                        if let Some(temp) = update.temperature {
                            self.persistent_configuration.steam_boiler_control_state.values.target_temperature = temp;
                        }
                        if let Some(pressure) = update.pressure {
                            self.persistent_configuration.steam_boiler_control_state.values.target_pressure = pressure;
                        }
                        self.configuration_store.save_settings(&self.persistent_configuration).await.ok();
                    },
                    _ => {
                        error!("Invalid boiler index: {}", boiler_index);
                    }
                }
            }
            MachineCommand::SetGroupBrewControlTarget(group_index, mode, values_update) => {
                info!("Setting group brew control mode for group {} to {:?} with values {:?}", group_index, mode, values_update);
                if group_index == 0 {
                    // Check if this is a curve mode and record start time
                    match mode {
                        GroupBrewControlMode::GroupFlowRateCurve |
                        GroupBrewControlMode::PressureCurve |
                        GroupBrewControlMode::OutputFlowRateCurve |
                        GroupBrewControlMode::FixedDutyCycleCurve => {
                            self.curve_start_time = Some(Instant::now());
                            info!("Starting curve control");
                        }
                        _ => {
                            // Reset curve start time for non-curve modes
                            self.curve_start_time = None;
                        }
                    }
                    self.ephemeral_configuration.group_brew_control_state.mode = mode;
                    if let Some(update) = values_update {
                        if let Some(flow_rate) = update.flow_rate {
                            self.ephemeral_configuration.group_brew_control_state.values.flow_rate = flow_rate;
                        }
                        if let Some(curve) = update.flow_rate_curve {
                            self.ephemeral_configuration.group_brew_control_state.values.flow_rate_curve = curve;
                        }
                        if let Some(pressure) = update.pressure {
                            self.ephemeral_configuration.group_brew_control_state.values.pressure = pressure;
                        }
                        if let Some(curve) = update.pressure_curve {
                            self.ephemeral_configuration.group_brew_control_state.values.pressure_curve = curve;
                        }
                        if let Some(output_flow) = update.output_flow_rate {
                            self.ephemeral_configuration.group_brew_control_state.values.output_flow_rate = output_flow;
                        }
                        if let Some(curve) = update.output_flow_rate_curve {
                            self.ephemeral_configuration.group_brew_control_state.values.output_flow_rate_curve = curve;
                        }
                        if let Some(duty) = update.duty_cycle {
                            self.ephemeral_configuration.group_brew_control_state.values.duty_cycle = duty;
                        }
                        if let Some(curve) = update.duty_cycle_curve {
                            self.ephemeral_configuration.group_brew_control_state.values.duty_cycle_curve = curve;
                        }
                    }
                } else {
                    error!("Invalid group index: {}", group_index);
                }
            }
            MachineCommand::SetGroupBrewControlTargetValues(group_index, update) => {
                info!("Setting group brew control values for group {} to {:?}", group_index, update);
                if group_index == 0 {
                    if let Some(flow_rate) = update.flow_rate {
                        self.ephemeral_configuration.group_brew_control_state.values.flow_rate = flow_rate;
                    }
                    if let Some(curve) = update.flow_rate_curve {
                        self.ephemeral_configuration.group_brew_control_state.values.flow_rate_curve = curve;
                    }
                    if let Some(pressure) = update.pressure {
                        self.ephemeral_configuration.group_brew_control_state.values.pressure = pressure;
                    }
                    if let Some(curve) = update.pressure_curve {
                        self.ephemeral_configuration.group_brew_control_state.values.pressure_curve = curve;
                    }
                    if let Some(output_flow) = update.output_flow_rate {
                        self.ephemeral_configuration.group_brew_control_state.values.output_flow_rate = output_flow;
                    }
                    if let Some(curve) = update.output_flow_rate_curve {
                        self.ephemeral_configuration.group_brew_control_state.values.output_flow_rate_curve = curve;
                    }
                    if let Some(duty) = update.duty_cycle {
                        self.ephemeral_configuration.group_brew_control_state.values.duty_cycle = duty;
                    }
                    if let Some(curve) = update.duty_cycle_curve {
                        self.ephemeral_configuration.group_brew_control_state.values.duty_cycle_curve = curve;
                    }
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
            MachineCommand::RunRoutine(_, _) | MachineCommand::CancelRoutine => {
                warn!("Ignoring unsupported command in finally block: {:?}", command);
            }
            MachineCommand::OptimizeConfigurationStorage => {
                info!("Optimizing configuration storage");
                if let Err(e) = self.configuration_store.optimize_storage().await {
                    warn!("Failed to optimize configuration storage: {}", e);
                }
            }
            MachineCommand::OptimizeRoutineStorage => {
                warn!("OptimizeRoutineStorage not supported for single boiler controller (no routine repository)");
            }
            MachineCommand::OptimizeScheduleStorage => {
                warn!("OptimizeScheduleStorage not supported for single boiler controller (no schedule store)");
            }
            _ => {}
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
                    self.group.set_brewing_state(true, 0).await;
                    self.started_brewing().await;
                }
                (SingleBoilerSingleGroupControllerState::Brewing, SingleBoilerSingleGroupControllerState::BrewModeIdle) => {
                    info!("Stopping brewing");
                    self.group.set_brewing_state(false, 0).await;
                    self.stopped_brewing().await;
                }
                _ => {}
            }
        }
    }

    async fn started_brewing(&mut self) {
        self.brew_start_time = Some(Instant::now());
        self.brew_start_input_volume = self.group.get_input_volume();
        self.boiler_pid.ki.accumulate += 50.0; // Initial accumulation to compensate for initial temperature drop
        let _ = self.group.scale_set_configuration(ScaleConfiguration {
            zero_tracking: Some(false),
            smoothing: Some(true)
        }).await;
        let _ = self.group.scale_tare().await;
    }

    async fn stopped_brewing(&mut self) {
        // Capture previous brew data before clearing
        if let Some(started_at) = self.brew_start_time {
            let stopped_at = Instant::now();
            let brew_time = started_at.elapsed().into();
            let brew_input_volume = self.brew_start_input_volume.and_then(|start_volume|
                self.group.get_input_volume().map(|current| current - start_volume)
            );
            let output_weight = self.group.get_output_weight();

            self.previous_brew = Some(crate::PreviousBrewInfo {
                brew_time,
                brew_input_volume,
                output_weight,
                started_at,
                stopped_at,
            });
        }

        self.brew_start_time = None;
        self.brew_start_input_volume = None;
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
        let mut repo = self.routine_repository.lock().await;
        let routine = repo.get_routine(routine_index).await;

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

            // Get finally commands
            let default_status = Status::default();
            let status = self.previous_status.as_ref().unwrap_or(&default_status);
            let finally_commands = routine.finally(status);

            // Restore saved configuration by splitting into persistent and ephemeral parts
            let saved_config = &routine.saved_configuration;
            self.persistent_configuration = saved_config.persistent;
            self.ephemeral_configuration = saved_config.ephemeral;
            // Save the restored persistent configuration
            self.configuration_store.save_settings(&self.persistent_configuration).await.ok();
            self.curve_start_time = None;  // Reset curve start time when routine exits
            self.transition_to_state(routine.saved_state).await;

            // Execute finally commands
            for cmd in finally_commands {
                self.handle_routine_finally_commands(cmd).await;
            }
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
            temperature_pid_parameters: config.persistent.pid_parameters.boiler_temperature_params.clone(),
            pressure_pid_parameters: config.persistent.pid_parameters.boiler_pressure_params.clone(),
            control_state: config.persistent.brew_boiler_control_state,
            max_temperature: Some(100.0),
            max_pressure: Some(15.0),
            // Embedded sensor configuration
            temperature_sensor_kalman_parameters: None,
            pressure_sensor_kalman_parameters: None,
            // No fill pump for single boiler
            fill_config: None,
        };
        configuration.insert_boiler_configuration(BrewBoiler.as_index(), brew_boiler_config);

        // Add virtual steam boiler configuration
        let steam_boiler_config = BoilerConfiguration {
            temperature_pid_parameters: PidParameters::default(), // Virtual steam boiler doesn't have separate PID
            pressure_pid_parameters: PidParameters::default(),
            control_state: config.persistent.steam_boiler_control_state,
            max_temperature: Some(150.0),
            max_pressure: Some(3.0),
            // Embedded sensor configuration
            temperature_sensor_kalman_parameters: None,
            pressure_sensor_kalman_parameters: None,
            // No fill pump for virtual steam boiler
            fill_config: None,
        };
        configuration.insert_boiler_configuration(VirtualSteamBoiler.as_index(), steam_boiler_config);

        // Add group configuration
        let group_config = GroupConfiguration {
            flow_rate_pid_parameters: config.persistent.pid_parameters.pump_flow_rate_params.clone(),
            output_flow_rate_pid_parameters: config.persistent.pid_parameters.pump_output_flow_rate_params.clone(),
            pressure_pid_parameters: config.persistent.pid_parameters.pump_pressure_params.clone(),
            brew_control_state: config.ephemeral.group_brew_control_state,
            max_brew_time_seconds: Some(300), // 5 minutes max brew time
            auto_tare_enabled: true,
            pump_configuration: None,
            pressure_sensor_kalman_parameters: None,
            flow_sensor_pulses_per_liter: None,
        };
        configuration.insert_group_configuration(SingleGroup.as_index(), group_config);

        // Add tank configuration
        let tank_config = TankConfiguration {
            low_level_warning_threshold: Some(20),
            water_level_sensor_kalman_parameters: None,
        };
        configuration.insert_tank_configuration(0, tank_config);

        configuration
    }
}