
extern crate alloc;

use crc::{Crc, CRC_32_ISCSI};
use defmt::{debug, error, info, warn, Format};
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
use variegated_hal::{Boiler, Group, WaterTap, Tank, PeripheralRegistry};
use variegated_hal::machine_mechanism::dual_boiler_mechanism::DualBoilerFillMechanism;
use variegated_controller_types::{BoilerConfiguration, BoilerControlTarget, BoilerIndex, BoilerStatus, BoilerType, CommsStatus, Configuration, FillConfiguration, GroupConfiguration, GroupIndex, PeripheralStatus, FlowRateType, GroupBrewControlTarget, GroupStatus, MachineCommand, Output, PidLimits, PidParameterTarget, PidParameters, PidTerm, PressureType, RoutineExecutionStatus, RoutineIndex, Status, KalmanParameters, WaterLevelType, WaterDispersalPumpStrategy, WaterTapStatus, WaterTapConfiguration, TankConfiguration, TankStatus};
use crate::routine::{RoutineParameters, RoutineExecutionContext, InMemoryRoutineRepository};
use variegated_controller_types::DualBoilerSingleGroupControllerBoilers::{BrewBoiler, SteamBoiler};
use variegated_controller_types::SingleGroupControllerGroups::SingleGroup;
use variegated_hal::scale::ScaleConfiguration;
use crate::settings::SettingsStorage;

#[derive(Clone, Copy, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DualBoilerSingleGroupPidParameters {
    pub brew_boiler_temperature_params: PidParameters,
    pub brew_boiler_pressure_params: PidParameters,
    pub steam_boiler_temperature_params: PidParameters,
    pub steam_boiler_pressure_params: PidParameters,
    pub pump_flow_rate_params: PidParameters,
    pub pump_pressure_params: PidParameters,
    pub pump_output_flow_rate_params: PidParameters,
}

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DualBoilerSingleGroupPersistentConfiguration {
    pub brew_boiler_control_target: BoilerControlTarget,
    pub steam_boiler_control_target: BoilerControlTarget,
    pub pid_parameters: DualBoilerSingleGroupPidParameters,
    pub heating_element_interlock: bool,
    pub allow_simultaneous_operations: bool,
    pub pump_tacho_pulses_per_liter: Option<f32>,
    pub flow_sensor_pulses_per_liter: Option<f32>,
    pub service_boiler_fill_threshold: Option<WaterLevelType>,
    pub water_dispersal_pump_strategy: WaterDispersalPumpStrategy,
}

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DualBoilerSingleGroupEphemeralConfiguration {
    pub group_brew_control_target: GroupBrewControlTarget,
}

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DualBoilerSingleGroupConfiguration {
    pub persistent: DualBoilerSingleGroupPersistentConfiguration,
    pub ephemeral: DualBoilerSingleGroupEphemeralConfiguration,
}

impl<'a> Value<'a> for DualBoilerSingleGroupPersistentConfiguration {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        let crc = Crc::<u32>::new(&CRC_32_ISCSI);

        info!("Serializing DualBoilerSingleGroupConfiguration");

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

        info!("Serialized DualBoilerSingleGroupConfiguration, len = {}", slice.clone().unwrap_or(0));

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

impl Default for DualBoilerSingleGroupEphemeralConfiguration {
    fn default() -> Self {
        Self {
            group_brew_control_target: GroupBrewControlTarget::FixedDutyCycle(100),
        }
    }
}

impl Default for DualBoilerSingleGroupPersistentConfiguration {
    fn default() -> Self {
        let mut pid_parameters = DualBoilerSingleGroupPidParameters::default();

        // Brew boiler PID parameters
        pid_parameters.brew_boiler_temperature_params = PidParameters {
            kp: PidTerm::new(3.0, PidLimits::default()),
            ki: PidTerm::new(0.01, PidLimits::new_with_limits(-10.0, 10.0).unwrap()),
            kd: PidTerm::new(30.0, PidLimits::new_with_limits(-10.0, 10.0).unwrap())
        };
        pid_parameters.brew_boiler_pressure_params = PidParameters {
            kp: PidTerm::new(3.0, PidLimits::default()),
            ki: PidTerm::new(0.01, PidLimits::new_with_limits(-10.0, 10.0).unwrap()),
            kd: PidTerm::new(30.0, PidLimits::new_with_limits(-10.0, 10.0).unwrap())
        };

        // Steam boiler PID parameters
        pid_parameters.steam_boiler_temperature_params = PidParameters {
            kp: PidTerm::new(5.0, PidLimits::default()),
            ki: PidTerm::new(0.01, PidLimits::new_with_limits(-10.0, 10.0).unwrap()),
            kd: PidTerm::new(30.0, PidLimits::new_with_limits(-10.0, 10.0).unwrap())
        };
        pid_parameters.steam_boiler_pressure_params = PidParameters {
            kp: PidTerm::new(3.0, PidLimits::default()),
            ki: PidTerm::new(0.01, PidLimits::new_with_limits(-10.0, 10.0).unwrap()),
            kd: PidTerm::new(30.0, PidLimits::new_with_limits(-10.0, 10.0).unwrap())
        };

        // Pump PID parameters
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

        DualBoilerSingleGroupPersistentConfiguration {
            brew_boiler_control_target: BoilerControlTarget::Temperature(93.0),
            steam_boiler_control_target: BoilerControlTarget::Temperature(120.0),
            pid_parameters,
            heating_element_interlock: false,
            allow_simultaneous_operations: true,
            pump_tacho_pulses_per_liter: None,
            flow_sensor_pulses_per_liter: None,
            service_boiler_fill_threshold: Some(20), // Fill when below 20%
            water_dispersal_pump_strategy: WaterDispersalPumpStrategy::AlwaysPump,
        }
    }
}

impl Default for DualBoilerSingleGroupConfiguration {
    fn default() -> Self {
        DualBoilerSingleGroupConfiguration {
            persistent: DualBoilerSingleGroupPersistentConfiguration::default(),
            ephemeral: DualBoilerSingleGroupEphemeralConfiguration::default(),
        }
    }
}

pub struct DualBoilerSingleGroupController<
    'a,
    ChannelM: RawMutex,
    M: RawMutex,
    SettingsStoreT: SettingsStorage<DualBoilerSingleGroupPersistentConfiguration>,
    const N_CHANNEL: usize,
    const N_WATCH: usize,
    const N_SUBS: usize,
    const N_CONFIG_SUBS: usize
> {
    command_channel_receiver: Receiver<'a, ChannelM, MachineCommand, N_CHANNEL>,
    status_channel_sender: Publisher<'a, ChannelM, Status, 1, N_SUBS, 1>,
    configuration_channel_sender: Publisher<'a, ChannelM, Configuration, 1, N_CONFIG_SUBS, 1>,

    // Hardware components
    brew_boiler: Boiler<'a, M, N_WATCH>,
    steam_boiler: Boiler<'a, M, N_WATCH>,
    group: Group<'a, M, N_WATCH>,
    water_tap: WaterTap<'a, M, N_WATCH>,
    tank: Option<Tank<'a, M, N_WATCH>>,
    fill_mechanism: Option<DualBoilerFillMechanism<'a>>,

    // Control systems
    brew_boiler_pid: PidCtrl<f32>,
    steam_boiler_pid: PidCtrl<f32>,
    pump_pid: PidCtrl<f32>,
    last_brew_boiler_output: f32,
    last_steam_boiler_output: f32,

    // Configuration and storage
    configuration_store: SettingsStoreT,
    persistent_configuration: DualBoilerSingleGroupPersistentConfiguration,
    ephemeral_configuration: DualBoilerSingleGroupEphemeralConfiguration,

    // Component-based state tracking
    brew_boiler_enabled: bool,
    steam_boiler_enabled: bool,
    group_brewing: bool,
    water_tap_dispensing: bool,

    // Routine execution
    routine_repository: &'static Mutex<NoopRawMutex, InMemoryRoutineRepository>,
    current_routine: Option<RoutineExecutionContext<u8, DualBoilerSingleGroupConfiguration>>,

    // Status tracking
    previous_status: Option<Status>,
    brew_temperature_movavg: MovAvg<f32, f32, 10>,
    steam_temperature_movavg: MovAvg<f32, f32, 10>,
    brew_start_time: Option<Instant>,
    curve_start_time: Option<Instant>,
    comms_status: Option<CommsStatus>,
    comms_status_received_instant: Option<Instant>,
    peripheral_registry: &'a PeripheralRegistry<'a>,
}

impl<
    'a,
    ChannelM: RawMutex,
    M: RawMutex,
    SettingsStoreT: SettingsStorage<DualBoilerSingleGroupPersistentConfiguration>,
    const N_CHANNEL: usize,
    const N_WATCH: usize,
    const N_SUBS: usize,
    const N_CONFIG_SUBS: usize
> DualBoilerSingleGroupController<'a, ChannelM, M, SettingsStoreT, N_CHANNEL, N_WATCH, N_SUBS, N_CONFIG_SUBS> {
    fn current_configuration(&self) -> DualBoilerSingleGroupConfiguration {
        DualBoilerSingleGroupConfiguration {
            persistent: self.persistent_configuration.clone(),
            ephemeral: self.ephemeral_configuration.clone(),
        }
    }

    pub fn new(
        command_channel_receiver: Receiver<'a, ChannelM, MachineCommand, N_CHANNEL>,
        status_channel_sender: Publisher<'a, ChannelM, Status, 1, N_SUBS, 1>,
        configuration_channel_sender: Publisher<'a, ChannelM, Configuration, 1, N_CONFIG_SUBS, 1>,
        brew_boiler: Boiler<'a, M, N_WATCH>,
        steam_boiler: Boiler<'a, M, N_WATCH>,
        group: Group<'a, M, N_WATCH>,
        water_tap: WaterTap<'a, M, N_WATCH>,
        tank: Option<Tank<'a, M, N_WATCH>>,
        fill_mechanism: Option<DualBoilerFillMechanism<'a>>,
        settings_store: SettingsStoreT,
        routine_repository: &'static Mutex<NoopRawMutex, InMemoryRoutineRepository>,
        peripheral_registry: &'a PeripheralRegistry<'a>,
    ) -> Self {
        Self {
            command_channel_receiver,
            status_channel_sender,
            configuration_channel_sender,
            brew_boiler,
            steam_boiler,
            group,
            water_tap,
            tank,
            fill_mechanism,
            brew_boiler_pid: super::limited_pid(),
            steam_boiler_pid: super::limited_pid(),
            pump_pid: super::limited_pid(),
            last_brew_boiler_output: 0.0,
            last_steam_boiler_output: 0.0,
            configuration_store: settings_store,
            persistent_configuration: DualBoilerSingleGroupPersistentConfiguration::default(),
            ephemeral_configuration: DualBoilerSingleGroupEphemeralConfiguration::default(),
            brew_boiler_enabled: true,
            steam_boiler_enabled: true,
            group_brewing: false,
            water_tap_dispensing: false,
            routine_repository,
            current_routine: None,
            previous_status: None,
            brew_temperature_movavg: MovAvg::default(),
            steam_temperature_movavg: MovAvg::default(),
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
        let mut last_debug_print = Instant::now();
        let mut last_configuration_publish = Instant::now();

        loop {
            self.persistent_configuration = self.configuration_store.load_settings().await.unwrap_or_default();

            // Check if configuration changed and publish if it did
            let current_config = self.current_configuration();
            if current_config != last_configuration {
                let config: Configuration = current_config.clone().into();
                self.configuration_channel_sender.publish_immediate(config);
                last_configuration = current_config;
            }

            // Handle incoming commands
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

            // Handle routine execution
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

            let next_pid_update = Instant::now();
            let delta_t = (next_pid_update - last_pid_update).as_millis() as f32;
            last_pid_update = next_pid_update;

            // Update control loops
            let brew_boiler_output = self.update_brew_boiler(delta_t).await;
            let steam_boiler_output = self.update_steam_boiler(delta_t).await;
            let pump_output = self.update_group_pump(delta_t).await;
            let _water_tap_output = self.update_water_tap(delta_t).await;

            // Update filling logic
            self.update_service_boiler_filling().await;

            self.send_status(brew_boiler_output, steam_boiler_output, pump_output).await;

            // Debug print status every 10 seconds
            let now = Instant::now();
            if now.saturating_duration_since(last_debug_print).as_secs() >= 10 {
                if let Some(ref status) = self.previous_status {
                    debug!("Status: {:?}", status);
                }
                last_debug_print = now;
            }

            // Publish configuration every 10 seconds regardless of changes
            if now.saturating_duration_since(last_configuration_publish).as_secs() >= 10 {
                let current_config = self.current_configuration();
                let config: Configuration = current_config.clone().into();
                self.configuration_channel_sender.publish_immediate(config);
                info!("Periodic configuration published");
                last_configuration_publish = now;
                last_configuration = current_config;
            }

            Timer::after_millis(100).await;
        }
    }

    // Safety interlock logic
    fn check_heating_element_interlock(&self, brew_demand: f32, steam_demand: f32) -> (f32, f32) {
        if !self.persistent_configuration.heating_element_interlock {
            return (brew_demand, steam_demand);
        }

        // If both boilers want to heat simultaneously, prioritize based on demand
        if brew_demand > 0.0 && steam_demand > 0.0 {
            if brew_demand >= steam_demand {
                (brew_demand, 0.0) // Priority to brew boiler
            } else {
                (0.0, steam_demand) // Priority to steam boiler
            }
        } else {
            (brew_demand, steam_demand)
        }
    }

    async fn update_brew_boiler(&mut self, delta_t: f32) -> Output {
        if !self.brew_boiler_enabled {
            self.brew_boiler.set_heating_element_duty_cycle(0).await;
            return Output::Off;
        }

        let control_target = self.persistent_configuration.brew_boiler_control_target;
        let mut brew_pv = match control_target {
            BoilerControlTarget::Temperature(target) => {
                self.brew_boiler_pid.setpoint = target as f32;
                self.brew_boiler_pid.set_parameters(self.persistent_configuration.pid_parameters.brew_boiler_temperature_params);
                self.brew_boiler.get_temperature().unwrap_or(0.0) as f32
            }
            BoilerControlTarget::Pressure(target) => {
                self.brew_boiler_pid.setpoint = target as f32;
                self.brew_boiler_pid.set_parameters(self.persistent_configuration.pid_parameters.brew_boiler_pressure_params);
                self.brew_boiler.get_pressure().unwrap_or(0.0) as f32
            }
            BoilerControlTarget::Off => {
                self.brew_boiler.set_heating_element_duty_cycle(0).await;
                return Output::Off;
            }
        };

        if let Ok(pv) = self.brew_temperature_movavg.try_feed(brew_pv) {
            brew_pv = pv;
        }

        let brew_pid_out = self.brew_boiler_pid.step(PidIn::new(brew_pv, delta_t));
        let brew_demand = brew_pid_out.out.max(0.0);
        self.last_brew_boiler_output = brew_demand;

        // Apply heating element interlock (will be checked in update_steam_boiler too)
        let steam_demand = if self.steam_boiler_enabled {
            match self.persistent_configuration.steam_boiler_control_target {
                BoilerControlTarget::Off => 0.0,
                _ => self.last_steam_boiler_output.max(0.0),
            }
        } else {
            0.0
        };

        let (final_brew_demand, _) = self.check_heating_element_interlock(brew_demand, steam_demand);

        self.brew_boiler.set_heating_element_duty_cycle(final_brew_demand as u8).await;

        if final_brew_demand != brew_demand {
            // Create modified PID output to reflect interlock adjustment
            Output::PidOutput(PidOut { out: final_brew_demand, ..brew_pid_out })
        } else {
            Output::PidOutput(brew_pid_out)
        }
    }

    async fn update_steam_boiler(&mut self, delta_t: f32) -> Output {
        if !self.steam_boiler_enabled {
            self.steam_boiler.set_heating_element_duty_cycle(0).await;
            return Output::Off;
        }

        let control_target = self.persistent_configuration.steam_boiler_control_target;
        let mut steam_pv = match control_target {
            BoilerControlTarget::Temperature(target) => {
                self.steam_boiler_pid.setpoint = target as f32;
                self.steam_boiler_pid.set_parameters(self.persistent_configuration.pid_parameters.steam_boiler_temperature_params);
                self.steam_boiler.get_temperature().unwrap_or(0.0) as f32
            }
            BoilerControlTarget::Pressure(target) => {
                self.steam_boiler_pid.setpoint = target as f32;
                self.steam_boiler_pid.set_parameters(self.persistent_configuration.pid_parameters.steam_boiler_pressure_params);
                self.steam_boiler.get_pressure().unwrap_or(0.0) as f32
            }
            BoilerControlTarget::Off => {
                self.steam_boiler.set_heating_element_duty_cycle(0).await;
                return Output::Off;
            }
        };

        if let Ok(pv) = self.steam_temperature_movavg.try_feed(steam_pv) {
            steam_pv = pv;
        }

        let steam_pid_out = self.steam_boiler_pid.step(PidIn::new(steam_pv, delta_t));
        let steam_demand = steam_pid_out.out.max(0.0);
        self.last_steam_boiler_output = steam_demand;

        // Apply heating element interlock
        let brew_demand = if self.brew_boiler_enabled {
            match self.persistent_configuration.brew_boiler_control_target {
                BoilerControlTarget::Off => 0.0,
                _ => self.last_brew_boiler_output.max(0.0),
            }
        } else {
            0.0
        };

        let (_, final_steam_demand) = self.check_heating_element_interlock(brew_demand, steam_demand);

        self.steam_boiler.set_heating_element_duty_cycle(final_steam_demand as u8).await;

        if final_steam_demand != steam_demand {
            // Create modified PID output to reflect interlock adjustment
            Output::PidOutput(PidOut { out: final_steam_demand, ..steam_pid_out })
        } else {
            Output::PidOutput(steam_pid_out)
        }
    }

    async fn update_group_pump(&mut self, delta_t: f32) -> Output {
        // Calculate elapsed time for curve evaluation if needed
        let elapsed_seconds = self.curve_start_time
            .map(|start| {
                let duration = Instant::now().saturating_duration_since(start);
                duration.as_secs() as f32 + (duration.as_millis() % 1000) as f32 / 1000.0
            })
            .unwrap_or(0.0);

        let control_target = if self.group_brewing {
            self.ephemeral_configuration.group_brew_control_target
        } else {
            GroupBrewControlTarget::Off
        };

        let pump_pv = match control_target {
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

        match control_target {
            GroupBrewControlTarget::Off => {
                self.group.set_brewing_state(false, 0).await;
                Output::Off
            },
            GroupBrewControlTarget::FullOn => {
                self.group.set_brewing_state(true, 100).await;
                Output::FixedDutyCycle(100)
            },
            GroupBrewControlTarget::FixedDutyCycle(duty_cycle) => {
                self.group.set_brewing_state(true, duty_cycle).await;
                info!("Fixed duty cycle target: {}", duty_cycle);
                Output::FixedDutyCycle(duty_cycle)
            }
            GroupBrewControlTarget::FixedDutyCycleCurve(curve) => {
                let target_duty_cycle = curve.evaluate(elapsed_seconds).clamp(0.0, 100.0) as u8;
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

    async fn update_water_tap(&mut self, _delta_t: f32) -> Output {
        if self.water_tap_dispensing {
            // Apply water dispersal pump strategy
            let duty_cycle = match self.persistent_configuration.water_dispersal_pump_strategy {
                WaterDispersalPumpStrategy::NoPump => 0, // Valve opens but no pump
                _ => 100, // Normal pumping
            };
            self.water_tap.set_water_dispensing_state(true, duty_cycle).await;
            Output::FixedDutyCycle(duty_cycle)
        } else {
            self.water_tap.set_water_dispensing_state(false, 0).await;
            Output::Off
        }
    }

    async fn update_service_boiler_filling(&mut self) {
        // Only attempt filling if automatic filling is enabled and we have a fill mechanism
/*        if !self.persistent_configuration.automatic_filling_enabled {
            return;
        }*/

        if let Some(fill_mechanism) = &mut self.fill_mechanism {
            if let Some(current_level) = self.steam_boiler.get_water_level() {
                //info!("Checking boiler fill: current level = {}, threshold = {:?}", current_level, self.persistent_configuration.service_boiler_fill_threshold);
                // Use the existing logic in DualBoilerFillMechanism which handles:
                // - Threshold checking
                // - Safety interlocks (won't fill if not idle)
                // - Automatic start/stop of filling
                fill_mechanism.check_and_fill_if_needed(current_level, self.persistent_configuration.service_boiler_fill_threshold).await;
            }
        }
    }

    async fn send_status(&mut self, brew_boiler_output: Output, steam_boiler_output: Output, pump_output: Output) {
        let brew_boiler_status = BoilerStatus {
            temperature: self.brew_boiler.get_temperature(),
            pressure: self.brew_boiler.get_pressure(),
            water_level: self.brew_boiler.get_water_level(),
            output: brew_boiler_output,
            control_target: self.persistent_configuration.brew_boiler_control_target,
        };

        let steam_boiler_status = BoilerStatus {
            temperature: self.steam_boiler.get_temperature(),
            pressure: self.steam_boiler.get_pressure(),
            water_level: self.steam_boiler.get_water_level(),
            output: steam_boiler_output,
            control_target: self.persistent_configuration.steam_boiler_control_target,
        };

        let group_status = GroupStatus {
            is_brewing: self.group_brewing,
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

        let water_tap_status = WaterTapStatus {
            is_dispensing: self.water_tap_dispensing,
        };

        // Create tank statuses map - only include tank if it exists
        let tank_statuses = if let Some(ref mut tank) = self.tank {
            FnvIndexMap::from_iter([(0, TankStatus {
                water_level: tank.get_water_level(),
            })])
        } else {
            FnvIndexMap::new()
        };

        let status = Status {
            boiler_statuses: FnvIndexMap::from_iter([(BrewBoiler.as_index(), brew_boiler_status), (SteamBoiler.as_index(), steam_boiler_status)]),
            group_statuses: FnvIndexMap::from_iter([(SingleGroup.as_index(), group_status)]),
            water_tap_statuses: FnvIndexMap::from_iter([(0, water_tap_status)]),
            tank_statuses,
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
                if self.persistent_configuration.allow_simultaneous_operations || !self.water_tap_dispensing {
                    self.start_brewing().await;
                } else {
                    warn!("Cannot start brewing while dispensing water (simultaneous operations disabled)");
                }
            }
            MachineCommand::StopBrewing(_) => {
                self.stop_brewing().await;
            }
            MachineCommand::StartPumpingToWaterTap(_) => {
                if self.persistent_configuration.allow_simultaneous_operations || !self.group_brewing {
                    self.start_water_tap_dispensing().await;
                } else {
                    warn!("Cannot start water tap while brewing (simultaneous operations disabled)");
                }
            }
            MachineCommand::StopPumpingToWaterTap(_) => {
                self.stop_water_tap_dispensing().await;
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
                    PidParameterTarget::BoilerPressure(boiler_index) => {
                        match boiler_index {
                            0 => self.persistent_configuration.pid_parameters.brew_boiler_pressure_params = params,
                            1 => self.persistent_configuration.pid_parameters.steam_boiler_pressure_params = params,
                            _ => error!("Invalid boiler index for PID parameters: {}", boiler_index),
                        }
                    }
                    PidParameterTarget::BoilerTemperature(boiler_index) => {
                        match boiler_index {
                            0 => self.persistent_configuration.pid_parameters.brew_boiler_temperature_params = params,
                            1 => self.persistent_configuration.pid_parameters.steam_boiler_temperature_params = params,
                            _ => error!("Invalid boiler index for PID parameters: {}", boiler_index),
                        }
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
                match boiler_index {
                    0 => {
                        info!("Enabling brew boiler");
                        self.brew_boiler_enabled = true;
                    }
                    1 => {
                        info!("Enabling steam boiler");
                        self.steam_boiler_enabled = true;
                    }
                    _ => {
                        warn!("Invalid boiler index: {}", boiler_index);
                    }
                }
            }
            MachineCommand::DisableBoiler(boiler_index) => {
                match boiler_index {
                    0 => {
                        info!("Disabling brew boiler");
                        self.brew_boiler_enabled = false;
                    }
                    1 => {
                        info!("Disabling steam boiler");
                        self.steam_boiler_enabled = false;
                    }
                    _ => {
                        warn!("Invalid boiler index: {}", boiler_index);
                    }
                }
            }
            MachineCommand::RunRoutine(index, params) => {
                info!("Running routine {} with {} parameters", index, params.as_ref().map(|p| p.len()).unwrap_or(0));
                self.handle_routine_start(index, params).await;
            }
            MachineCommand::CancelRoutine => {
                info!("Cancelling routine");
                self.handle_routine_exit().await;
            }
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

    async fn start_brewing(&mut self) {
        if !self.group_brewing {
            info!("Starting brewing");
            self.group_brewing = true;
            self.brew_start_time = Some(Instant::now());
            self.group.set_brewing_state(true, 0).await;

            // Give initial PID boost for temperature drop compensation
            self.brew_boiler_pid.ki.accumulate += 50.0;

            let _ = self.group.scale_set_configuration(ScaleConfiguration {
                zero_tracking: Some(false),
                smoothing: Some(true)
            }).await;
            let _ = self.group.scale_tare().await;
        }
    }

    async fn stop_brewing(&mut self) {
        if self.group_brewing {
            info!("Stopping brewing");
            self.group_brewing = false;
            self.brew_start_time = None;
            self.curve_start_time = None;
            self.group.set_brewing_state(false, 0).await;

            let _ = self.group.scale_set_configuration(ScaleConfiguration {
                zero_tracking: Some(true),
                smoothing: Some(false)
            }).await;
        }
    }

    async fn start_water_tap_dispensing(&mut self) {
        if !self.water_tap_dispensing {
            info!("Starting water tap dispensing");
            self.water_tap_dispensing = true;
            // Apply water dispersal pump strategy
            let duty_cycle = match self.persistent_configuration.water_dispersal_pump_strategy {
                WaterDispersalPumpStrategy::NoPump => 0, // Valve opens but no pump
                _ => 100, // Normal pumping
            };
            self.water_tap.set_water_dispensing_state(true, duty_cycle).await;
        }
    }

    async fn stop_water_tap_dispensing(&mut self) {
        if self.water_tap_dispensing {
            info!("Stopping water tap dispensing");
            self.water_tap_dispensing = false;
            self.water_tap.set_water_dispensing_state(false, 0).await;
        }
    }

    async fn handle_routine_start(&mut self, routine_index: RoutineIndex, runtime_params: Option<RoutineParameters>) {
        if self.current_routine.is_some() {
            return;
        }
        let repo = self.routine_repository.lock().await;
        let routine = repo.get_routine(routine_index);

        if let Some(routine) = routine {
            info!("Running routine");
            self.current_routine = Some(RoutineExecutionContext::new(routine_index, routine.clone(), 0u8, self.current_configuration(), runtime_params));
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
            self.persistent_configuration = saved_config.persistent;
            self.ephemeral_configuration = saved_config.ephemeral;
            // Save the restored persistent configuration
            self.configuration_store.save_settings(&self.persistent_configuration).await.ok();
            self.curve_start_time = None;
        } else {
            warn!("No routine to exit");
        }
    }
}

impl From<DualBoilerSingleGroupConfiguration> for Configuration {
    fn from(config: DualBoilerSingleGroupConfiguration) -> Self {
        let mut configuration = Configuration::default();

        // Add brew boiler configuration
        let brew_boiler_config = BoilerConfiguration {
            temperature_pid_parameters: config.persistent.pid_parameters.brew_boiler_temperature_params.clone(),
            pressure_pid_parameters: config.persistent.pid_parameters.brew_boiler_pressure_params.clone(),
            control_target: config.persistent.brew_boiler_control_target,
            max_temperature: None,
            max_pressure: None,
            temperature_sensor_kalman_parameters: None,
            pressure_sensor_kalman_parameters: None,
            fill_config: None,
        };
        configuration.insert_boiler_configuration(BrewBoiler.as_index(), brew_boiler_config);

        // Add steam boiler configuration
        let steam_boiler_config = BoilerConfiguration {
            temperature_pid_parameters: config.persistent.pid_parameters.steam_boiler_temperature_params.clone(),
            pressure_pid_parameters: config.persistent.pid_parameters.steam_boiler_pressure_params.clone(),
            control_target: config.persistent.steam_boiler_control_target,
            max_temperature: None,
            max_pressure: None,
            temperature_sensor_kalman_parameters: None,
            pressure_sensor_kalman_parameters: None,
            fill_config: Some(FillConfiguration {
                fill_threshold: config.persistent.service_boiler_fill_threshold,
                pump_configuration: None,
            }),
        };
        configuration.insert_boiler_configuration(SteamBoiler.as_index(), steam_boiler_config);

        // Add group configuration
        let group_config = GroupConfiguration {
            flow_rate_pid_parameters: config.persistent.pid_parameters.pump_flow_rate_params.clone(),
            output_flow_rate_pid_parameters: config.persistent.pid_parameters.pump_output_flow_rate_params.clone(),
            pressure_pid_parameters: config.persistent.pid_parameters.pump_pressure_params.clone(),
            brew_control_target: config.ephemeral.group_brew_control_target,
            max_brew_time_seconds: None,
            auto_tare_enabled: true,
            pump_configuration: None,
            pressure_sensor_kalman_parameters: None,
            flow_sensor_pulses_per_liter: None,
        };
        configuration.insert_group_configuration(SingleGroup.as_index(), group_config);

        let water_tap_config = WaterTapConfiguration {
            pump_strategy: config.persistent.water_dispersal_pump_strategy,
            temperature_target: None,
            max_dispense_time_seconds: None,
            flow_rate_limit: None,
            pump_configuration: None,
        };
        configuration.insert_water_tap_configuration(0, water_tap_config);

        // Add tank configuration if tank is present
        // Note: We can't access self.tank here since this is a From implementation
        // Tank configuration will need to be added separately when tank is detected
        let tank_config = TankConfiguration {
            low_level_warning_threshold: Some(20),
            water_level_sensor_kalman_parameters: None,
        };
        configuration.insert_tank_configuration(0, tank_config);

        configuration
    }
}