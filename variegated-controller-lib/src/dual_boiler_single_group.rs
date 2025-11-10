
extern crate alloc;

use alloc::vec::Vec;
use crc::{Crc, CRC_32_ISCSI};
use defmt::{debug, error, info, warn, Format};
use embassy_rp::adc::Config;
use embassy_rp::watchdog::Watchdog;
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, RawMutex};
use embassy_sync::channel::{Receiver, Sender};
use embassy_sync::mutex::Mutex;
use embassy_sync::pubsub::Publisher;
use embassy_sync::watch;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use heapless::FnvIndexMap;
use movavg::MovAvg;
use postcard::{from_bytes, from_bytes_crc32, to_slice, to_slice_crc32};
use sequential_storage::map::{SerializationError, Value};
use variegated_control_algorithm::pid::{PidCtrl, PidIn, PidOut};
use variegated_hal::{Boiler, Group, WaterTap, Tank, PeripheralRegistry};
use variegated_hal::machine_mechanism::dual_boiler_mechanism::DualBoilerFillMechanism;
use variegated_controller_types::{BoilerConfiguration, BoilerControlMode, BoilerControlState, BoilerControlTargetValues, BoilerControlTargetValuesUpdate, BoilerIndex, BoilerStatus, BoilerType, CommsStatus, Configuration, FillConfiguration, GroupConfiguration, GroupIndex, InputVolumeType, PeripheralStatus, FlowRateType, GroupBrewControlMode, GroupBrewControlState, GroupBrewControlTargetValues, GroupBrewControlTargetValuesUpdate, GroupStatus, MachineCommand, Output, PidLimits, PidParameterTarget, PidParameters, PidTerm, PressureType, RoutineExecutionStatus, RoutineIndex, Status, StorageCommand, KalmanParameters, WaterLevelType, WaterDispersalPumpStrategy, WaterTapStatus, WaterTapConfiguration, TankConfiguration, TankStatus, RoutineParameters, MachineMode};
use crate::routine::{RoutineExecutionContext, InMemoryRoutineRepository, RoutineRepository};
use variegated_controller_types::DualBoilerSingleGroupControllerBoilers::{BrewBoiler, SteamBoiler};
use variegated_controller_types::SingleGroupControllerGroups::SingleGroup;
use variegated_hal::scale::ScaleConfiguration;
use variegated_timekeeping::TimeKeeper;
use crate::schedule::{InMemoryScheduleStore, ScheduleStore};
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

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DualBoilerSingleGroupPersistentConfiguration {
    pub brew_boiler_control_state: BoilerControlState,
    pub steam_boiler_control_state: BoilerControlState,
    pub pid_parameters: DualBoilerSingleGroupPidParameters,
    pub heating_element_interlock: bool,
    pub allow_simultaneous_operations: bool,
    pub pump_tacho_pulses_per_liter: Option<f32>,
    pub flow_sensor_pulses_per_liter: Option<f32>,
    pub service_boiler_fill_threshold: Option<WaterLevelType>,
    pub water_dispersal_pump_strategy: WaterDispersalPumpStrategy,
    pub group_pump_configuration: Option<variegated_controller_types::PumpConfiguration>,
    pub water_tap_pump_configuration: Option<variegated_controller_types::PumpConfiguration>,
    pub fill_pump_configuration: Option<variegated_controller_types::PumpConfiguration>,
}

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DualBoilerSingleGroupEphemeralConfiguration {
    pub mode: MachineMode,
    pub group_brew_control_state: GroupBrewControlState,
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DualBoilerSingleGroupConfiguration {
    pub persistent: DualBoilerSingleGroupPersistentConfiguration,
    pub ephemeral: DualBoilerSingleGroupEphemeralConfiguration,
}

impl DualBoilerSingleGroupConfiguration {
    pub fn effective_brew_boiler_control_mode(&self) -> BoilerControlMode {
        if self.ephemeral.mode != MachineMode::On {
            BoilerControlMode::Off
        } else {
            self.persistent.brew_boiler_control_state.mode
        }
    }

    pub fn effective_steam_boiler_control_mode(&self) -> BoilerControlMode {
        if self.ephemeral.mode != MachineMode::On {
            BoilerControlMode::Off
        } else {
            self.persistent.steam_boiler_control_state.mode
        }
    }

    pub fn effective_group_brew_control_mode(&self) -> GroupBrewControlMode {
        if self.ephemeral.mode != MachineMode::On {
            GroupBrewControlMode::Off
        } else {
            self.ephemeral.group_brew_control_state.mode
        }
    }
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
            group_brew_control_state: GroupBrewControlState {
                mode: GroupBrewControlMode::FixedDutyCycle,
                values: GroupBrewControlTargetValues {
                    duty_cycle: 100,
                    ..GroupBrewControlTargetValues::default()
                },
            },
            mode: MachineMode::default(),
        }
    }
}

impl Default for DualBoilerSingleGroupPersistentConfiguration {
    fn default() -> Self {
        let mut pid_parameters = DualBoilerSingleGroupPidParameters::default();

        // Brew boiler PID parameters
        pid_parameters.brew_boiler_temperature_params = PidParameters {
            kp: PidTerm::new(12.0, PidLimits::default()),
            ki: PidTerm::new(0.0003, PidLimits::new_with_limits(0.0, 30.0).unwrap()),
            kd: PidTerm::new(0.0, PidLimits::new_with_limits(-10.0, 10.0).unwrap())
        };
        pid_parameters.brew_boiler_pressure_params = PidParameters {
            kp: PidTerm::new(3.0, PidLimits::default()),
            ki: PidTerm::new(0.01, PidLimits::new_with_limits(-10.0, 10.0).unwrap()),
            kd: PidTerm::new(30.0, PidLimits::new_with_limits(-10.0, 10.0).unwrap())
        };

        // Steam boiler PID parameters
        pid_parameters.steam_boiler_temperature_params = PidParameters {
            kp: PidTerm::new(12.0, PidLimits::default()),
            ki: PidTerm::new(0.0003, PidLimits::new_with_limits(0.0, 30.0).unwrap()),
            kd: PidTerm::new(0.0, PidLimits::new_with_limits(-10.0, 10.0).unwrap())
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
            brew_boiler_control_state: BoilerControlState {
                mode: BoilerControlMode::Temperature,
                values: BoilerControlTargetValues {
                    target_temperature: 93.0,
                    target_pressure: 1.0,
                },
            },
            steam_boiler_control_state: BoilerControlState {
                mode: BoilerControlMode::Temperature,
                values: BoilerControlTargetValues {
                    target_temperature: 120.0,
                    target_pressure: 1.5,
                },
            },
            pid_parameters,
            heating_element_interlock: false,
            allow_simultaneous_operations: true,
            pump_tacho_pulses_per_liter: None,
            flow_sensor_pulses_per_liter: None,
            service_boiler_fill_threshold: Some(20), // Fill when below 20%
            water_dispersal_pump_strategy: WaterDispersalPumpStrategy::AlwaysPump,
            group_pump_configuration: None,
            water_tap_pump_configuration: None,
            fill_pump_configuration: None,
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
    BoilerM: RawMutex,
    GroupM: RawMutex,
    WaterTapM: RawMutex,
    TankM: RawMutex,
    FillM: RawMutex,
    StorageM: RawMutex + 'static,
    SettingsStoreT: SettingsStorage<DualBoilerSingleGroupPersistentConfiguration> + 'static,
    RoutineRepoT: RoutineRepository + 'static,
    ScheduleStoreT: ScheduleStore + 'static,
    const N_CHANNEL: usize,
    const N_WATCH: usize,
    const N_SUBS: usize,
    const N_CONFIG_SUBS: usize
> {
    command_channel_receiver: Receiver<'a, ChannelM, MachineCommand, N_CHANNEL>,
    status_channel_sender: Publisher<'a, ChannelM, Status, 1, N_SUBS, 1>,
    configuration_channel_sender: Publisher<'a, ChannelM, Configuration, 1, N_CONFIG_SUBS, 1>,
    storage_command_sender: Sender<'a, ChannelM, StorageCommand, 4>,

    // Hardware components
    brew_boiler: Boiler<'a, BoilerM, N_WATCH>,
    steam_boiler: Boiler<'a, BoilerM, N_WATCH>,
    group: Group<'a, GroupM, N_WATCH>,
    water_tap: WaterTap<'a, WaterTapM, N_WATCH>,
    tank: Option<Tank<'a, TankM, N_WATCH>>,
    fill_mechanism: Option<DualBoilerFillMechanism<'a, FillM>>,

    // Control systems
    brew_boiler_pid: PidCtrl<f32>,
    steam_boiler_pid: PidCtrl<f32>,
    pump_pid: PidCtrl<f32>,
    last_brew_boiler_output: f32,
    last_steam_boiler_output: f32,

    // Configuration and storage
    configuration_store: &'static Mutex<StorageM, SettingsStoreT>,
    configuration: DualBoilerSingleGroupConfiguration,
//    persistent_configuration: DualBoilerSingleGroupPersistentConfiguration,
//    ephemeral_configuration: DualBoilerSingleGroupEphemeralConfiguration,

    // Component-based state tracking
    brew_boiler_enabled: bool,
    steam_boiler_enabled: bool,
    group_brewing: bool,
    water_tap_dispensing: bool,

    // Routine execution
    routine_repository: &'static Mutex<StorageM, RoutineRepoT>,
    current_routine: Option<RoutineExecutionContext<u8, DualBoilerSingleGroupConfiguration>>,

    // Shot logging
    shot_logger: crate::shot_log::ShotLogger,
    previous_routine_step: Option<usize>,

    // Schedule store
    schedule_store: &'static Mutex<StorageM, ScheduleStoreT>,

    // Status tracking
    previous_status: Option<Status>,
    brew_temperature_movavg: MovAvg<f32, f32, 10>,
    steam_temperature_movavg: MovAvg<f32, f32, 10>,
    brew_start_time: Option<Instant>,
    brew_start_input_volume: Option<InputVolumeType>,
    previous_brew: Option<crate::PreviousBrewInfo>,
    curve_start_time: Option<Instant>,
    comms_status: Option<CommsStatus>,
    comms_status_received_instant: Option<Instant>,
    peripheral_registry: &'a PeripheralRegistry<'a>,
    watchdog: Option<Watchdog>,

    // Shot state tracking
    current_shot_state: Option<variegated_controller_types::ShotState>,
    flow_rate_history: MovAvg<f32, f32, 10>,
    pressure_history: MovAvg<f32, f32, 10>,
    saturation_start_time: Option<Instant>,
    last_shot_state_sample_time: Option<Instant>,
}

impl<
    'a,
    ChannelM: RawMutex,
    BoilerM: RawMutex,
    GroupM: RawMutex,
    WaterTapM: RawMutex,
    TankM: RawMutex,
    FillM: RawMutex,
    StorageM: RawMutex + 'static,
    SettingsStoreT: SettingsStorage<DualBoilerSingleGroupPersistentConfiguration>,
    RoutineRepoT: RoutineRepository,
    ScheduleStoreT: ScheduleStore,
    const N_CHANNEL: usize,
    const N_WATCH: usize,
    const N_SUBS: usize,
    const N_CONFIG_SUBS: usize
> DualBoilerSingleGroupController<'a, ChannelM, BoilerM, GroupM, WaterTapM, TankM, FillM, StorageM, SettingsStoreT, RoutineRepoT, ScheduleStoreT, N_CHANNEL, N_WATCH, N_SUBS, N_CONFIG_SUBS> {
/*    fn current_configuration(&self) -> DualBoilerSingleGroupConfiguration {
        DualBoilerSingleGroupConfiguration {
            persistent: self.persistent_configuration.clone(),
            ephemeral: self.ephemeral_configuration.clone(),
        }
    }*/

    pub fn new(
        command_channel_receiver: Receiver<'a, ChannelM, MachineCommand, N_CHANNEL>,
        status_channel_sender: Publisher<'a, ChannelM, Status, 1, N_SUBS, 1>,
        configuration_channel_sender: Publisher<'a, ChannelM, Configuration, 1, N_CONFIG_SUBS, 1>,
        storage_command_sender: Sender<'a, ChannelM, StorageCommand, 4>,
        brew_boiler: Boiler<'a, BoilerM, N_WATCH>,
        steam_boiler: Boiler<'a, BoilerM, N_WATCH>,
        group: Group<'a, GroupM, N_WATCH>,
        water_tap: WaterTap<'a, WaterTapM, N_WATCH>,
        tank: Option<Tank<'a, TankM, N_WATCH>>,
        fill_mechanism: Option<DualBoilerFillMechanism<'a, FillM>>,
        settings_store: &'static Mutex<StorageM, SettingsStoreT>,
        routine_repository: &'static Mutex<StorageM, RoutineRepoT>,
        schedule_store: &'static Mutex<StorageM, ScheduleStoreT>,
        peripheral_registry: &'a PeripheralRegistry<'a>,
        watchdog: Option<Watchdog>,
    ) -> Self {
        Self {
            command_channel_receiver,
            status_channel_sender,
            configuration_channel_sender,
            storage_command_sender,
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
            configuration: DualBoilerSingleGroupConfiguration::default(),
//            persistent_configuration: DualBoilerSingleGroupPersistentConfiguration::default(),
//            ephemeral_configuration: DualBoilerSingleGroupEphemeralConfiguration::default(),
            brew_boiler_enabled: true,
            steam_boiler_enabled: true,
            group_brewing: false,
            water_tap_dispensing: false,
            routine_repository,
            schedule_store,
            current_routine: None,
            shot_logger: crate::shot_log::ShotLogger::new(),
            previous_routine_step: None,
            previous_status: None,
            brew_temperature_movavg: MovAvg::default(),
            steam_temperature_movavg: MovAvg::default(),
            brew_start_time: None,
            brew_start_input_volume: None,
            previous_brew: None,
            curve_start_time: None,
            comms_status: None,
            comms_status_received_instant: None,
            peripheral_registry,
            watchdog,

            // Shot state tracking initialization
            current_shot_state: None,
            flow_rate_history: MovAvg::default(),
            pressure_history: MovAvg::default(),
            saturation_start_time: None,
            last_shot_state_sample_time: None,
        }
    }

    async fn create_general_configuration(&mut self) -> Configuration {
        let mut configuration = Configuration::default();

        // Add brew boiler configuration
        let brew_boiler_config = BoilerConfiguration {
            temperature_pid_parameters: self.configuration.persistent.pid_parameters.brew_boiler_temperature_params.clone(),
            pressure_pid_parameters: self.configuration.persistent.pid_parameters.brew_boiler_pressure_params.clone(),
            control_state: self.configuration.persistent.brew_boiler_control_state,
            max_temperature: None,
            max_pressure: None,
            temperature_sensor_kalman_parameters: None,
            pressure_sensor_kalman_parameters: None,
            fill_config: None,
        };
        configuration.insert_boiler_configuration(BrewBoiler.as_index(), brew_boiler_config);

        // Add steam boiler configuration
        let steam_boiler_config = BoilerConfiguration {
            temperature_pid_parameters: self.configuration.persistent.pid_parameters.steam_boiler_temperature_params.clone(),
            pressure_pid_parameters: self.configuration.persistent.pid_parameters.steam_boiler_pressure_params.clone(),
            control_state: self.configuration.persistent.steam_boiler_control_state,
            max_temperature: None,
            max_pressure: None,
            temperature_sensor_kalman_parameters: None,
            pressure_sensor_kalman_parameters: None,
            fill_config: Some(FillConfiguration {
                fill_threshold: self.configuration.persistent.service_boiler_fill_threshold,
                pump_configuration: self.configuration.persistent.fill_pump_configuration.clone(),
            }),
        };
        configuration.insert_boiler_configuration(SteamBoiler.as_index(), steam_boiler_config);

        // Add group configuration
        let group_config = GroupConfiguration {
            flow_rate_pid_parameters: self.configuration.persistent.pid_parameters.pump_flow_rate_params.clone(),
            output_flow_rate_pid_parameters: self.configuration.persistent.pid_parameters.pump_output_flow_rate_params.clone(),
            pressure_pid_parameters: self.configuration.persistent.pid_parameters.pump_pressure_params.clone(),
            brew_control_state: self.configuration.ephemeral.group_brew_control_state,
            max_brew_time_seconds: None,
            auto_tare_enabled: true,
            pump_configuration: self.configuration.persistent.group_pump_configuration.clone(),
            pressure_sensor_kalman_parameters: None,
            flow_sensor_pulses_per_liter: None,
        };
        configuration.insert_group_configuration(SingleGroup.as_index(), group_config);

        let water_tap_config = WaterTapConfiguration {
            pump_strategy: self.configuration.persistent.water_dispersal_pump_strategy,
            temperature_target: None,
            max_dispense_time_seconds: None,
            flow_rate_limit: None,
            pump_configuration: self.configuration.persistent.water_tap_pump_configuration.clone(),
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

        // Try to get schedules with timeout to avoid blocking if optimization is running
        configuration.schedules = match with_timeout(Duration::from_millis(100), self.schedule_store.lock()).await {
            Ok(mut store) => store.get_schedules().await.cloned().collect(),
            Err(_) => {
                warn!("Failed to acquire schedule_store lock for configuration (timeout)");
                Vec::new()
            }
        };

        configuration
    }

    pub async fn task(&mut self) {
        let mut last_pid_update = Instant::now();
        let mut last_published_configuration = self.configuration.clone();
        let mut last_debug_print = Instant::now();
        let mut last_configuration_publish = Instant::now();

        loop {
            // Try to load settings with timeout to avoid blocking if optimization is running
            self.configuration.persistent = match with_timeout(Duration::from_millis(100), self.configuration_store.lock()).await {
                Ok(mut store) => store.load_settings().await.unwrap_or_default(),
                Err(_) => {
                    // If we can't acquire the lock, keep current configuration
                    self.configuration.persistent.clone()
                }
            };

            last_published_configuration = self.publish_configuration_if_changed(last_published_configuration).await;

            // Handle incoming commands
            while !self.command_channel_receiver.is_empty() {
                let command = self.command_channel_receiver.try_receive();
                if let Ok(command) = command {
                    self.handle_command(command).await;
                    last_published_configuration = self.publish_configuration_if_changed(last_published_configuration).await;
                }
            }

            // Handle routine execution
            if let Some(routine) = &mut self.current_routine {
                if routine.finished_executing {
                    info!("Routine finished executing");
                    self.handle_routine_exit().await;
                } else if let Some(status) = self.previous_status.as_ref() {
                    // Record shot log sample
                    self.shot_logger.record_sample(status);

                    // Detect and record step transitions
                    if routine.current_step != self.previous_routine_step {
                        if let Some(current_step) = routine.current_step {
                            use variegated_controller_types::RoutineEvent;
                            let event = RoutineEvent {
                                timestamp_millis: self.shot_logger.current_log()
                                    .and_then(|log| log.samples.last())
                                    .map(|s| s.timestamp_millis)
                                    .unwrap_or(0),
                                from_step: self.previous_routine_step,
                                to_step: current_step,
                                exit_condition_description: None,
                                step_description: routine.routine.steps.get(current_step)
                                    .and_then(|s| s.description.clone()),
                            };
                            self.shot_logger.record_routine_event(event);
                            self.previous_routine_step = Some(current_step);
                        }
                    }

                    // Execute routine step commands
                    for command in routine.step(status, None) {
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

            // Update shot state tracking
            self.update_shot_state();

            self.send_status(brew_boiler_output, steam_boiler_output, pump_output).await;

            // Debug print status every 10 seconds
            let now = Instant::now();
            if now.saturating_duration_since(last_debug_print).as_secs() >= 1 {
                if let Some(ref status) = self.previous_status {
                    debug!("Status: {:?}", status);
                }
                last_debug_print = now;
            }

            // Publish configuration every 10 seconds regardless of changes
            if now.saturating_duration_since(last_configuration_publish).as_secs() >= 10 {
                let current_config = self.configuration.clone();
                self.publish_general_configuration().await;
                info!("Periodic configuration published");
                last_configuration_publish = now;
                last_published_configuration = current_config;
            }

            // Feed the watchdog to prevent system reset
            if let Some(ref mut watchdog) = self.watchdog {
                watchdog.feed();
            }

            Timer::after_millis(100).await;
        }
    }

    async fn publish_configuration_if_changed(&mut self, previous_configuration: DualBoilerSingleGroupConfiguration) -> DualBoilerSingleGroupConfiguration {
        // Check if configuration changed and publish if it did
        if self.configuration != previous_configuration {
            self.publish_general_configuration().await;
            return self.configuration.clone();
        }

        previous_configuration
    }

    async fn publish_general_configuration(&mut self) {
        let config: Configuration = self.create_general_configuration().await;;
        self.configuration_channel_sender.publish_immediate(config);
    }

    // Safety interlock logic
    fn check_heating_element_interlock(&self, brew_demand: f32, steam_demand: f32) -> (f32, f32) {
        if !self.configuration.persistent.heating_element_interlock {
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

        let control_state = self.configuration.persistent.brew_boiler_control_state;
        let mut brew_pv = match self.configuration.effective_brew_boiler_control_mode() {
            BoilerControlMode::Temperature => {
                self.brew_boiler_pid.setpoint = control_state.values.target_temperature as f32;
                self.brew_boiler_pid.set_parameters(self.configuration.persistent.pid_parameters.brew_boiler_temperature_params);
                self.brew_boiler.get_temperature().unwrap_or(0.0) as f32
            }
            BoilerControlMode::Pressure => {
                self.brew_boiler_pid.setpoint = control_state.values.target_pressure as f32;
                self.brew_boiler_pid.set_parameters(self.configuration.persistent.pid_parameters.brew_boiler_pressure_params);
                self.brew_boiler.get_pressure().unwrap_or(0.0) as f32
            }
            BoilerControlMode::Off => {
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
            match self.configuration.effective_steam_boiler_control_mode() {
                BoilerControlMode::Off => 0.0,
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

        let control_state = self.configuration.persistent.steam_boiler_control_state;
        let mut steam_pv = match self.configuration.effective_steam_boiler_control_mode() {
            BoilerControlMode::Temperature => {
                self.steam_boiler_pid.setpoint = control_state.values.target_temperature as f32;
                self.steam_boiler_pid.set_parameters(self.configuration.persistent.pid_parameters.steam_boiler_temperature_params);
                self.steam_boiler.get_temperature().unwrap_or(0.0) as f32
            }
            BoilerControlMode::Pressure => {
                self.steam_boiler_pid.setpoint = control_state.values.target_pressure as f32;
                self.steam_boiler_pid.set_parameters(self.configuration.persistent.pid_parameters.steam_boiler_pressure_params);
                self.steam_boiler.get_pressure().unwrap_or(0.0) as f32
            }
            BoilerControlMode::Off => {
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
            match self.configuration.effective_brew_boiler_control_mode() {
                BoilerControlMode::Off => 0.0,
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

    fn apply_pump_configuration_limits(&self, duty_cycle: u8, is_off: bool) -> u8 {
        // If pump is off, always return 0 regardless of min_duty_cycle
        if is_off {
            return 0;
        }

        // Apply pump configuration limits if configured
        if let Some(ref config) = self.configuration.persistent.group_pump_configuration {
            let mut limited_duty = duty_cycle;

            // Apply minimum duty cycle limit
            if let Some(min_duty) = config.min_duty_cycle {
                limited_duty = limited_duty.max(min_duty);
            }

            // Apply maximum duty cycle limit
            if let Some(max_duty) = config.max_duty_cycle {
                limited_duty = limited_duty.min(max_duty);
            }

            limited_duty
        } else {
            duty_cycle
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

        let control_state = if self.group_brewing {
            self.configuration.ephemeral.group_brew_control_state
        } else {
            let mut off_state = self.configuration.ephemeral.group_brew_control_state;
            off_state.mode = GroupBrewControlMode::Off;
            off_state
        };

        let pump_pv = match control_state.mode {
            GroupBrewControlMode::GroupFlowRate => {
                self.pump_pid.setpoint = control_state.values.flow_rate as f32;
                self.pump_pid.set_parameters(self.configuration.persistent.pid_parameters.pump_flow_rate_params);
                self.group.get_input_flow_rate().unwrap_or(0.0) as f32
            },
            GroupBrewControlMode::GroupFlowRateCurve => {
                let target = control_state.values.flow_rate_curve.evaluate(elapsed_seconds);
                self.pump_pid.setpoint = target;
                self.pump_pid.set_parameters(self.configuration.persistent.pid_parameters.pump_flow_rate_params);
                self.group.get_input_flow_rate().unwrap_or(0.0) as f32
            },
            GroupBrewControlMode::Pressure => {
                self.pump_pid.setpoint = control_state.values.pressure as f32;
                self.pump_pid.set_parameters(self.configuration.persistent.pid_parameters.pump_pressure_params);
                self.group.get_pressure().unwrap_or(0.0) as f32
            },
            GroupBrewControlMode::PressureCurve => {
                let target = control_state.values.pressure_curve.evaluate(elapsed_seconds);
                self.pump_pid.setpoint = target;
                self.pump_pid.set_parameters(self.configuration.persistent.pid_parameters.pump_pressure_params);
                self.group.get_pressure().unwrap_or(0.0) as f32
            },
            GroupBrewControlMode::OutputFlowRate => {
                self.pump_pid.setpoint = control_state.values.output_flow_rate as f32;
                self.pump_pid.set_parameters(self.configuration.persistent.pid_parameters.pump_output_flow_rate_params);
                self.group.get_output_flow_rate().unwrap_or(0.0) as f32
            },
            GroupBrewControlMode::OutputFlowRateCurve => {
                let target = control_state.values.output_flow_rate_curve.evaluate(elapsed_seconds);
                self.pump_pid.setpoint = target;
                self.pump_pid.set_parameters(self.configuration.persistent.pid_parameters.pump_output_flow_rate_params);
                self.group.get_output_flow_rate().unwrap_or(0.0) as f32
            },
            _ => 0.0,
        };

        let pump_pid_out = self.pump_pid.step(PidIn::new(pump_pv, delta_t));

        match control_state.mode {
            GroupBrewControlMode::Off => {
                let duty_cycle = self.apply_pump_configuration_limits(0, true);
                self.group.set_brewing_state(false, duty_cycle).await;
                Output::Off
            },
            GroupBrewControlMode::FullOn => {
                let duty_cycle = self.apply_pump_configuration_limits(100, false);
                self.group.set_brewing_state(true, duty_cycle).await;
                Output::FixedDutyCycle(duty_cycle)
            },
            GroupBrewControlMode::FixedDutyCycle => {
                let duty_cycle = self.apply_pump_configuration_limits(control_state.values.duty_cycle, false);
                self.group.set_brewing_state(true, duty_cycle).await;
                //info!("Fixed duty cycle target: {}", duty_cycle);
                Output::FixedDutyCycle(duty_cycle)
            }
            GroupBrewControlMode::FixedDutyCycleCurve => {
                let target_duty_cycle = control_state.values.duty_cycle_curve.evaluate(elapsed_seconds).clamp(0.0, 100.0) as u8;
                let duty_cycle = self.apply_pump_configuration_limits(target_duty_cycle, false);
                //info!("Fixed duty cycle curve target: {}", duty_cycle);
                //info!("Curve start: {:?} elapsed time: {} seconds", self.curve_start_time, elapsed_seconds);
                self.group.set_brewing_state(true, duty_cycle).await;
                Output::FixedDutyCycle(duty_cycle)
            }
            _ => {
                let duty_cycle = self.apply_pump_configuration_limits(pump_pid_out.out as u8, false);
                info!("PID target: {}", duty_cycle);
                self.group.set_brewing_state(true, duty_cycle).await;
                Output::PidOutput(PidOut { out: duty_cycle as f32, ..pump_pid_out })
            },
        }
    }

    fn apply_water_tap_pump_configuration_limits(&self, duty_cycle: u8, is_off: bool) -> u8 {
        // If pump is off, always return 0 regardless of min_duty_cycle
        if is_off {
            return 0;
        }

        // Apply pump configuration limits if configured
        if let Some(ref config) = self.configuration.persistent.water_tap_pump_configuration {
            let mut limited_duty = duty_cycle;

            // Apply minimum duty cycle limit
            if let Some(min_duty) = config.min_duty_cycle {
                limited_duty = limited_duty.max(min_duty);
            }

            // Apply maximum duty cycle limit
            if let Some(max_duty) = config.max_duty_cycle {
                limited_duty = limited_duty.min(max_duty);
            }

            limited_duty
        } else {
            duty_cycle
        }
    }

    async fn update_water_tap(&mut self, _delta_t: f32) -> Output {
        if self.water_tap_dispensing {
            // Apply water dispersal pump strategy
            let base_duty_cycle = match self.configuration.persistent.water_dispersal_pump_strategy {
                WaterDispersalPumpStrategy::NoPump => 0, // Valve opens but no pump
                _ => 100, // Normal pumping
            };
            let duty_cycle = self.apply_water_tap_pump_configuration_limits(base_duty_cycle, false);
            self.water_tap.set_water_dispensing_state(true, duty_cycle).await;
            Output::FixedDutyCycle(duty_cycle)
        } else {
            let duty_cycle = self.apply_water_tap_pump_configuration_limits(0, true);
            self.water_tap.set_water_dispensing_state(false, duty_cycle).await;
            Output::Off
        }
    }

    async fn update_service_boiler_filling(&mut self) {
        if self.configuration.ephemeral.mode != MachineMode::On {
            return;
        }

        if let Some(fill_mechanism) = &mut self.fill_mechanism {
            if let Some(current_level) = self.steam_boiler.get_water_level() {
                fill_mechanism.check_and_fill_if_needed(current_level, self.configuration.persistent.service_boiler_fill_threshold).await;
            }
        }
    }

    async fn send_status(&mut self, brew_boiler_output: Output, steam_boiler_output: Output, pump_output: Output) {
        let brew_boiler_status = BoilerStatus {
            temperature: self.brew_boiler.get_temperature(),
            pressure: self.brew_boiler.get_pressure(),
            water_level: self.brew_boiler.get_water_level(),
            output: brew_boiler_output,
            control_state: self.configuration.persistent.brew_boiler_control_state,
        };

        let steam_boiler_status = BoilerStatus {
            temperature: self.steam_boiler.get_temperature(),
            pressure: self.steam_boiler.get_pressure(),
            water_level: self.steam_boiler.get_water_level(),
            output: steam_boiler_output,
            control_state: self.configuration.persistent.steam_boiler_control_state,
        };

        let brew_input_volume = match (self.brew_start_input_volume, self.group.get_input_volume()) {
            (Some(start_volume), Some(current_volume)) => Some(current_volume - start_volume),
            _ => None,
        };

        let group_status = GroupStatus {
            is_brewing: self.group_brewing,
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
            control_state: self.configuration.ephemeral.group_brew_control_state,
            previous_brew: self.previous_brew.map(|info| info.into()),
            shot_state: self.current_shot_state,
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
                wifi_rssi: status.wifi_rssi,
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
            mode: self.configuration.ephemeral.mode,
            routine_execution,
            comms_status,
            peripheral_status: self.peripheral_registry.get_peripheral_status(),
            current_local_time: TimeKeeper::now_local().map(|t| t.naive_local()),
        };

        self.status_channel_sender.publish_immediate(status.clone());

        self.previous_status = Some(status);
    }

    async fn handle_command(&mut self, command: MachineCommand) {
        info!("Received command: {:?}", command);

        match command {
            MachineCommand::RunRoutine(index, params) => {
                if self.configuration.ephemeral.mode != MachineMode::On {
                    warn!("Cannot start routine while not in On mode");
                    return;
                }

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
                if self.configuration.ephemeral.mode != MachineMode::On {
                    warn!("Cannot start brewing while not in On mode");
                    return;
                }

                if self.configuration.persistent.allow_simultaneous_operations || !self.water_tap_dispensing {
                    self.start_brewing().await;
                } else {
                    warn!("Cannot start brewing while dispensing water (simultaneous operations disabled)");
                }
            }
            MachineCommand::StopBrewing(_) => {
                self.stop_brewing().await;
            }
            MachineCommand::StartPumpingToWaterTap(_) => {
                if self.configuration.ephemeral.mode != MachineMode::On {
                    warn!("Cannot start pumping while not in On mode");
                    return;
                }

                if self.configuration.persistent.allow_simultaneous_operations || !self.group_brewing {
                    self.start_water_tap_dispensing().await;
                } else {
                    warn!("Cannot start water tap while brewing (simultaneous operations disabled)");
                }
            }
            MachineCommand::StopPumpingToWaterTap(_) => {
                self.stop_water_tap_dispensing().await;
            }
            MachineCommand::SetBoilerControlTarget(boiler_index, mode, values_update) => {
                info!("Setting boiler control mode for boiler {} to {:?} with values {:?}", boiler_index, mode, values_update);
                match boiler_index {
                    0 => {
                        self.configuration.persistent.brew_boiler_control_state.mode = mode;
                        if let Some(update) = values_update {
                            if let Some(temp) = update.temperature {
                                self.configuration.persistent.brew_boiler_control_state.values.target_temperature = temp;
                            }
                            if let Some(pressure) = update.pressure {
                                self.configuration.persistent.brew_boiler_control_state.values.target_pressure = pressure;
                            }
                        }
                        match with_timeout(Duration::from_millis(100), self.configuration_store.lock()).await {
                            Ok(mut store) => { store.save_settings(&self.configuration.persistent).await.ok(); }
                            Err(_) => warn!("Failed to acquire configuration_store lock for save (timeout)"),
                        }
                    },
                    1 => {
                        self.configuration.persistent.steam_boiler_control_state.mode = mode;
                        if let Some(update) = values_update {
                            if let Some(temp) = update.temperature {
                                self.configuration.persistent.steam_boiler_control_state.values.target_temperature = temp;
                            }
                            if let Some(pressure) = update.pressure {
                                self.configuration.persistent.steam_boiler_control_state.values.target_pressure = pressure;
                            }
                        }
                        match with_timeout(Duration::from_millis(100), self.configuration_store.lock()).await {
                            Ok(mut store) => { store.save_settings(&self.configuration.persistent).await.ok(); }
                            Err(_) => warn!("Failed to acquire configuration_store lock for save (timeout)"),
                        }
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
                            self.configuration.persistent.brew_boiler_control_state.values.target_temperature = temp;
                        }
                        if let Some(pressure) = update.pressure {
                            self.configuration.persistent.brew_boiler_control_state.values.target_pressure = pressure;
                        }
                        match with_timeout(Duration::from_millis(100), self.configuration_store.lock()).await {
                            Ok(mut store) => { store.save_settings(&self.configuration.persistent).await.ok(); }
                            Err(_) => warn!("Failed to acquire configuration_store lock for save (timeout)"),
                        }
                    },
                    1 => {
                        if let Some(temp) = update.temperature {
                            self.configuration.persistent.steam_boiler_control_state.values.target_temperature = temp;
                        }
                        if let Some(pressure) = update.pressure {
                            self.configuration.persistent.steam_boiler_control_state.values.target_pressure = pressure;
                        }
                        match with_timeout(Duration::from_millis(100), self.configuration_store.lock()).await {
                            Ok(mut store) => { store.save_settings(&self.configuration.persistent).await.ok(); }
                            Err(_) => warn!("Failed to acquire configuration_store lock for save (timeout)"),
                        }
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
                    self.configuration.ephemeral.group_brew_control_state.mode = mode;
                    if let Some(update) = values_update {
                        if let Some(flow_rate) = update.flow_rate {
                            self.configuration.ephemeral.group_brew_control_state.values.flow_rate = flow_rate;
                        }
                        if let Some(curve) = update.flow_rate_curve {
                            self.configuration.ephemeral.group_brew_control_state.values.flow_rate_curve = curve;
                        }
                        if let Some(pressure) = update.pressure {
                            self.configuration.ephemeral.group_brew_control_state.values.pressure = pressure;
                        }
                        if let Some(curve) = update.pressure_curve {
                            self.configuration.ephemeral.group_brew_control_state.values.pressure_curve = curve;
                        }
                        if let Some(output_flow) = update.output_flow_rate {
                            self.configuration.ephemeral.group_brew_control_state.values.output_flow_rate = output_flow;
                        }
                        if let Some(curve) = update.output_flow_rate_curve {
                            self.configuration.ephemeral.group_brew_control_state.values.output_flow_rate_curve = curve;
                        }
                        if let Some(duty) = update.duty_cycle {
                            self.configuration.ephemeral.group_brew_control_state.values.duty_cycle = duty;
                        }
                        if let Some(curve) = update.duty_cycle_curve {
                            self.configuration.ephemeral.group_brew_control_state.values.duty_cycle_curve = curve;
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
                        self.configuration.ephemeral.group_brew_control_state.values.flow_rate = flow_rate;
                    }
                    if let Some(curve) = update.flow_rate_curve {
                        self.configuration.ephemeral.group_brew_control_state.values.flow_rate_curve = curve;
                    }
                    if let Some(pressure) = update.pressure {
                        self.configuration.ephemeral.group_brew_control_state.values.pressure = pressure;
                    }
                    if let Some(curve) = update.pressure_curve {
                        self.configuration.ephemeral.group_brew_control_state.values.pressure_curve = curve;
                    }
                    if let Some(output_flow) = update.output_flow_rate {
                        self.configuration.ephemeral.group_brew_control_state.values.output_flow_rate = output_flow;
                    }
                    if let Some(curve) = update.output_flow_rate_curve {
                        self.configuration.ephemeral.group_brew_control_state.values.output_flow_rate_curve = curve;
                    }
                    if let Some(duty) = update.duty_cycle {
                        self.configuration.ephemeral.group_brew_control_state.values.duty_cycle = duty;
                    }
                    if let Some(curve) = update.duty_cycle_curve {
                        self.configuration.ephemeral.group_brew_control_state.values.duty_cycle_curve = curve;
                    }
                } else {
                    error!("Invalid group index: {}", group_index);
                }
            }
            MachineCommand::SetPidParameters(target, params) => {
                match target {
                    PidParameterTarget::BoilerPressure(boiler_index) => {
                        match boiler_index {
                            0 => self.configuration.persistent.pid_parameters.brew_boiler_pressure_params = params,
                            1 => self.configuration.persistent.pid_parameters.steam_boiler_pressure_params = params,
                            _ => error!("Invalid boiler index for PID parameters: {}", boiler_index),
                        }
                    }
                    PidParameterTarget::BoilerTemperature(boiler_index) => {
                        match boiler_index {
                            0 => self.configuration.persistent.pid_parameters.brew_boiler_temperature_params = params,
                            1 => self.configuration.persistent.pid_parameters.steam_boiler_temperature_params = params,
                            _ => error!("Invalid boiler index for PID parameters: {}", boiler_index),
                        }
                    }
                    PidParameterTarget::GroupFlowRate(_) => {
                        self.configuration.persistent.pid_parameters.pump_flow_rate_params = params;
                    }
                    PidParameterTarget::GroupPressure(_) => {
                        self.configuration.persistent.pid_parameters.pump_pressure_params = params;
                    }
                    PidParameterTarget::GroupOutputFlowRate(_) => {
                        self.configuration.persistent.pid_parameters.pump_output_flow_rate_params = params;
                    }
                }
                // Save after updating PID parameters
                match with_timeout(Duration::from_millis(100), self.configuration_store.lock()).await {
                    Ok(mut store) => { store.save_settings(&self.configuration.persistent).await.ok(); }
                    Err(_) => warn!("Failed to acquire configuration_store lock for save (timeout)"),
                }
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
            MachineCommand::RemoveScheduleItem(idx) => {
                info!("Removing schedule item at index {}", idx);
                match with_timeout(Duration::from_millis(100), self.schedule_store.lock()).await {
                    Ok(mut store) => {
                        let res = store.remove_schedule(idx).await;
                        if res.is_none() {
                            warn!("Failed to remove schedule at index {}: index out of bounds", idx);
                        }
                    }
                    Err(_) => warn!("Failed to acquire schedule_store lock (timeout)"),
                }
            }
            MachineCommand::AddScheduleItem(item) => {
                info!("Adding new schedule item");
                match with_timeout(Duration::from_millis(100), self.schedule_store.lock()).await {
                    Ok(mut store) => store.add_schedule(item).await,
                    Err(_) => warn!("Failed to acquire schedule_store lock (timeout)"),
                }
            }
            MachineCommand::UpdateScheduleItem(idx, item) => {
                info!("Updating schedule item at index {}", idx);
                match with_timeout(Duration::from_millis(100), self.schedule_store.lock()).await {
                    Ok(mut store) => {
                        let res = store.update_schedule(idx, item).await;
                        if res.is_err() {
                            warn!("Failed to update schedule at index {}: index out of bounds", idx);
                        }
                    }
                    Err(_) => warn!("Failed to acquire schedule_store lock (timeout)"),
                }
            }
            MachineCommand::AddRoutine(routine) => {
                info!("Adding new routine");
                match with_timeout(Duration::from_millis(100), self.routine_repository.lock()).await {
                    Ok(mut repo) => repo.add_routine(routine).await,
                    Err(_) => warn!("Failed to acquire routine_repository lock (timeout)"),
                }
            }
            MachineCommand::RemoveRoutine(idx) => {
                match with_timeout(Duration::from_millis(100), self.routine_repository.lock()).await {
                    Ok(mut repo) => {
                        let res = repo.remove_routine(idx).await;
                        if res.is_none() {
                            warn!("Failed to remove routine at index {}: index out of bounds", idx);
                        }
                    }
                    Err(_) => warn!("Failed to acquire routine_repository lock (timeout)"),
                }
            }
            MachineCommand::UpdateRoutine(idx, Routine) => {
                match with_timeout(Duration::from_millis(100), self.routine_repository.lock()).await {
                    Ok(mut repo) => {
                        let res = repo.update_routine(idx, Routine).await;
                        if res.is_err() {
                            warn!("Failed to update routine at index {}: index out of bounds", idx);
                        }
                    }
                    Err(_) => warn!("Failed to acquire routine_repository lock (timeout)"),
                }
            }
            MachineCommand::SetMachineMode(mode) => {
                info!("Setting machine mode to {:?}", mode);
                self.configuration.ephemeral.mode = mode;
                if mode != MachineMode::On {
                    // Stop brewing and water tap dispensing if not in On mode
                    if self.group_brewing {
                        self.stop_brewing().await;
                    }
                    if self.water_tap_dispensing {
                        self.stop_water_tap_dispensing().await;
                    }
                }
            }
            MachineCommand::OptimizeConfigurationStorage => {
                info!("Sending OptimizeConfiguration to storage task");
                if let Err(_) = self.storage_command_sender.try_send(StorageCommand::OptimizeConfiguration) {
                    warn!("Failed to send OptimizeConfiguration command: channel full");
                }
            }
            MachineCommand::OptimizeRoutineStorage => {
                info!("Sending OptimizeRoutines to storage task");
                if let Err(_) = self.storage_command_sender.try_send(StorageCommand::OptimizeRoutines) {
                    warn!("Failed to send OptimizeRoutines command: channel full");
                }
            }
            MachineCommand::OptimizeScheduleStorage => {
                info!("Sending OptimizeSchedules to storage task");
                if let Err(_) = self.storage_command_sender.try_send(StorageCommand::OptimizeSchedules) {
                    warn!("Failed to send OptimizeSchedules command: channel full");
                }
            }
            MachineCommand::SetGroupPumpConfiguration(group_index, config) => {
                if group_index == 0 {
                    info!("Setting group pump configuration: {:?}", config);
                    self.configuration.persistent.group_pump_configuration = Some(config);
                    match with_timeout(Duration::from_millis(100), self.configuration_store.lock()).await {
                        Ok(mut store) => { store.save_settings(&self.configuration.persistent).await.ok(); }
                        Err(_) => warn!("Failed to acquire configuration_store lock for save (timeout)"),
                    }
                } else {
                    error!("Invalid group index for pump configuration: {}", group_index);
                }
            }
            MachineCommand::SetWaterTapPumpConfiguration(water_tap_index, config) => {
                if water_tap_index == 0 {
                    info!("Setting water tap pump configuration: {:?}", config);
                    self.configuration.persistent.water_tap_pump_configuration = Some(config);
                    match with_timeout(Duration::from_millis(100), self.configuration_store.lock()).await {
                        Ok(mut store) => { store.save_settings(&self.configuration.persistent).await.ok(); }
                        Err(_) => warn!("Failed to acquire configuration_store lock for save (timeout)"),
                    }
                } else {
                    error!("Invalid water tap index for pump configuration: {}", water_tap_index);
                }
            }
            MachineCommand::SetFillPumpConfiguration(boiler_index, config) => {
                // For dual boiler, we only support fill pump configuration for the steam/service boiler (index 1)
                if boiler_index == 1 {
                    info!("Setting fill pump configuration: {:?}", config);
                    self.configuration.persistent.fill_pump_configuration = Some(config);
                    match with_timeout(Duration::from_millis(100), self.configuration_store.lock()).await {
                        Ok(mut store) => { store.save_settings(&self.configuration.persistent).await.ok(); }
                        Err(_) => warn!("Failed to acquire configuration_store lock for save (timeout)"),
                    }
                } else {
                    error!("Invalid boiler index for fill pump configuration: {} (only boiler 1 supports filling)", boiler_index);
                }
            }
            MachineCommand::InferGroupPressureIntegral(group_index, target_pressure) => {
                if group_index == 0 {
                    info!("Inferring group pressure integral for target pressure: {} bar", target_pressure);

                    // Get current duty cycle from the control state configuration
                    let current_duty_cycle = self.configuration.ephemeral.group_brew_control_state.values.duty_cycle;
                    let current_pressure = self.group.get_pressure().unwrap_or(0.0);

                    // Set up PID for pressure control
                    self.pump_pid.setpoint = target_pressure as f32;
                    self.pump_pid.set_parameters(self.configuration.persistent.pid_parameters.pump_pressure_params);

                    // Infer and set the integral
                    self.pump_pid.infer_and_set_integral(current_duty_cycle as f32, current_pressure as f32);

                    info!("Set pressure integral based on duty cycle {} and pressure {}", current_duty_cycle, current_pressure);
                } else {
                    error!("Invalid group index: {}", group_index);
                }
            }
            MachineCommand::InferGroupFlowRateIntegral(group_index, target_flow_rate) => {
                if group_index == 0 {
                    info!("Inferring group flow rate integral for target flow rate: {} ml/s", target_flow_rate);

                    // Get current duty cycle from the control state configuration
                    let current_duty_cycle = self.configuration.ephemeral.group_brew_control_state.values.duty_cycle;
                    let current_flow_rate = self.group.get_input_flow_rate().unwrap_or(0.0);

                    // Set up PID for flow rate control
                    self.pump_pid.setpoint = target_flow_rate as f32;
                    self.pump_pid.set_parameters(self.configuration.persistent.pid_parameters.pump_flow_rate_params);

                    // Infer and set the integral
                    self.pump_pid.infer_and_set_integral(current_duty_cycle as f32, current_flow_rate as f32);

                    info!("Set flow rate integral based on duty cycle {} and flow rate {}", current_duty_cycle, current_flow_rate);
                } else {
                    error!("Invalid group index: {}", group_index);
                }
            }
            MachineCommand::InferGroupOutputFlowRateIntegral(group_index, target_output_flow_rate) => {
                if group_index == 0 {
                    info!("Inferring group output flow rate integral for target: {} ml/s", target_output_flow_rate);

                    // Get current duty cycle from the control state configuration
                    let current_duty_cycle = self.configuration.ephemeral.group_brew_control_state.values.duty_cycle;
                    let current_output_flow_rate = self.group.get_output_flow_rate().unwrap_or(0.0);

                    // Set up PID for output flow rate control
                    self.pump_pid.setpoint = target_output_flow_rate as f32;
                    self.pump_pid.set_parameters(self.configuration.persistent.pid_parameters.pump_output_flow_rate_params);

                    // Infer and set the integral
                    self.pump_pid.infer_and_set_integral(current_duty_cycle as f32, current_output_flow_rate as f32);

                    info!("Set output flow rate integral based on duty cycle {} and output flow rate {}", current_duty_cycle, current_output_flow_rate);
                } else {
                    error!("Invalid group index: {}", group_index);
                }
            }
        }
    }

    async fn start_brewing(&mut self) {
        if !self.group_brewing {
            let current_volume = self.group.get_input_volume();
            info!("Starting brewing - capturing baseline volume: {:?}", current_volume);
            self.group_brewing = true;
            self.brew_start_time = Some(Instant::now());
            self.brew_start_input_volume = current_volume;
            info!("brew_start_input_volume set to: {:?}", self.brew_start_input_volume);

            // Initialize shot state tracking
            self.current_shot_state = Some(variegated_controller_types::ShotState::HeadspaceFill);
            self.flow_rate_history = MovAvg::default(); // Reset history
            self.pressure_history = MovAvg::default(); // Reset history
            self.saturation_start_time = None;
            self.last_shot_state_sample_time = None; // Reset sampling timer
            info!("Shot state initialized to HeadspaceFill");

            self.group.set_brewing_state(true, 0).await;

            // Give initial PID boost for temperature drop compensation
            self.brew_boiler_pid.ki.accumulate += 50.0;

            let _ = self.group.scale_set_configuration(ScaleConfiguration {
                zero_tracking: Some(false),
                smoothing: Some(true)
            }).await;
            let _ = self.group.scale_tare().await;
        } else {
            info!("start_brewing() called but group_brewing already true - skipping baseline capture");
        }
    }

    async fn stop_brewing(&mut self) {
        if self.group_brewing {
            let current_volume = self.group.get_input_volume();
            info!("Stopping brewing - current volume: {:?}, brew_start_input_volume: {:?}",
                  current_volume, self.brew_start_input_volume);

            // Capture previous brew data before clearing
            if let Some(started_at) = self.brew_start_time {
                let stopped_at = Instant::now();
                let brew_time = started_at.elapsed().into();
                let brew_input_volume = self.brew_start_input_volume.and_then(|start_volume|
                    self.group.get_input_volume().map(|current| current - start_volume)
                );
                let output_weight = self.group.get_output_weight();

                info!("Final brew_input_volume: {:?}", brew_input_volume);

                self.previous_brew = Some(crate::PreviousBrewInfo {
                    brew_time,
                    brew_input_volume,
                    output_weight,
                    started_at,
                    stopped_at,
                });
            }

            info!("Clearing brew_start_input_volume (was: {:?})", self.brew_start_input_volume);
            self.group_brewing = false;
            self.brew_start_time = None;
            self.brew_start_input_volume = None;
            self.curve_start_time = None;

            // Clear shot state tracking
            self.current_shot_state = None;
            self.saturation_start_time = None;
            info!("Shot state cleared");

            info!("brew_start_input_volume now: {:?}", self.brew_start_input_volume);
            self.group.set_brewing_state(false, 0).await;

            let _ = self.group.scale_set_configuration(ScaleConfiguration {
                zero_tracking: Some(true),
                smoothing: Some(false)
            }).await;
        }
    }

    fn update_shot_state(&mut self) {
        // Constants for shot state detection
        const FIRST_DROP_WEIGHT_THRESHOLD: f32 = 1.0; // grams
        const FLOW_DECREASE_THRESHOLD: f32 = 3.0; // ml/s below average (dramatic change at saturation)
        const PRESSURE_INCREASE_THRESHOLD: f32 = 3.0; // bar above average (dramatic change at saturation)
        const SAMPLE_INTERVAL_MS: u64 = 333; // Sample every 333ms (3 samples/sec, 10 samples = 3.33 seconds history)

        if !self.group_brewing {
            // Not brewing, no shot state
            return;
        }

        // Check if it's time to sample (every 500ms)
        let now = Instant::now();
        let should_sample = match self.last_shot_state_sample_time {
            None => true, // First sample
            Some(last_time) => now.saturating_duration_since(last_time).as_millis() >= SAMPLE_INTERVAL_MS,
        };

        if !should_sample {
            return; // Skip this update, not time to sample yet
        }

        // Update sample time
        self.last_shot_state_sample_time = Some(now);

        let current_flow = self.group.get_input_flow_rate().unwrap_or(0.0);
        let current_pressure = self.group.get_pressure().unwrap_or(0.0);

        // Update histories and get averaged values (sampling at 3 Hz for 3.33s history window)
        let flow_avg = self.flow_rate_history.try_feed(current_flow).unwrap_or(current_flow);
        let pressure_avg = self.pressure_history.try_feed(current_pressure).unwrap_or(current_pressure);

        match self.current_shot_state {
            Some(variegated_controller_types::ShotState::HeadspaceFill) => {
                // Check for transition to Saturation
                // We need to detect when flow is decreasing AND pressure is increasing

                // Simple slope approximation: compare current reading to average
                // If current < average, flow is decreasing
                // If current > average, pressure is increasing
                let flow_decreasing = current_flow < flow_avg && (flow_avg - current_flow) > FLOW_DECREASE_THRESHOLD;
                let pressure_increasing = current_pressure > pressure_avg && (current_pressure - pressure_avg) > PRESSURE_INCREASE_THRESHOLD;

                if flow_decreasing && pressure_increasing {
                    info!("Shot state transition: HeadspaceFill -> Saturation (flow: {}->{}, pressure: {}->{})",
                          flow_avg, current_flow, pressure_avg, current_pressure);
                    self.current_shot_state = Some(variegated_controller_types::ShotState::Saturation);
                    self.saturation_start_time = Some(Instant::now());
                }
            }
            Some(variegated_controller_types::ShotState::Saturation) => {
                // Check for transition to PostFirstDrop
                if let Some(output_weight) = self.group.get_output_weight() {
                    if output_weight > FIRST_DROP_WEIGHT_THRESHOLD {
                        info!("Shot state transition: Saturation -> PostFirstDrop (weight: {}g)", output_weight);
                        self.current_shot_state = Some(variegated_controller_types::ShotState::PostFirstDrop);
                    }
                }
            }
            Some(variegated_controller_types::ShotState::PostFirstDrop) => {
                // Final state, no more transitions
            }
            None => {
                // Should not happen during brewing, but handle gracefully
                warn!("Shot state is None while brewing - resetting to HeadspaceFill");
                self.current_shot_state = Some(variegated_controller_types::ShotState::HeadspaceFill);
            }
        }
    }

    async fn start_water_tap_dispensing(&mut self) {
        if !self.water_tap_dispensing {
            info!("Starting water tap dispensing");
            self.water_tap_dispensing = true;
            // Apply water dispersal pump strategy
            let base_duty_cycle = match self.configuration.persistent.water_dispersal_pump_strategy {
                WaterDispersalPumpStrategy::NoPump => 0, // Valve opens but no pump
                _ => 100, // Normal pumping
            };
            let duty_cycle = self.apply_water_tap_pump_configuration_limits(base_duty_cycle, false);
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
        let mut repo = self.routine_repository.lock().await;
        let routine = repo.get_routine(routine_index).await;

        if let Some(routine) = routine {
            info!("Running routine");

            // Create routine execution context
            let routine_execution_context = RoutineExecutionContext::new(
                routine_index,
                routine.clone(),
                0u8,
                self.configuration.clone(),
                runtime_params.clone()
            );

            // Start shot logging
            use variegated_controller_types::{ShotLogMetadata, ShotType, ShotStatus, RoutineExecutionMetadata};
            let metadata = ShotLogMetadata {
                shot_type: ShotType::Routine,
                group_index: SingleGroup.as_index(),
                routine_metadata: Some(RoutineExecutionMetadata {
                    routine_index,
                    routine_name: routine.name.clone(),
                    routine_type: routine.routine_type,
                    resolved_parameters: routine_execution_context.parameters.clone(),
                }),
                start_time_millis: embassy_time::Instant::now().as_millis(),
                end_time_millis: None,
                final_status: ShotStatus::Running,
            };
            self.shot_logger.start_shot(metadata);
            self.previous_routine_step = None;

            self.current_routine = Some(routine_execution_context);
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

            // In a single group setting, it always makes sense to stop brewing and water tap dispensing
            if self.group_brewing {
                self.stop_brewing().await;
            }
            if self.water_tap_dispensing {
                self.stop_water_tap_dispensing().await;
            }

            self.configuration = routine.saved_configuration.clone();

            // Save the restored persistent configuration
            match with_timeout(Duration::from_millis(100), self.configuration_store.lock()).await {
                Ok(mut store) => { store.save_settings(&self.configuration.persistent).await.ok(); }
                Err(_) => warn!("Failed to acquire configuration_store lock for save (timeout)"),
            }
            self.curve_start_time = None;

            // Execute finally commands
            for cmd in finally_commands {
                self.handle_routine_finally_commands(cmd).await;
            }

            // Finish shot logging
            use variegated_controller_types::ShotStatus;
            self.shot_logger.finish_shot(ShotStatus::Completed);
            self.previous_routine_step = None;
        } else {
            warn!("No routine to exit");
        }
    }
}