
extern crate alloc;

use alloc::vec::Vec;
use crc::{Crc, CRC_32_ISCSI};
use defmt::Format;
use variegated_log::{log_debug, log_error, log_info, log_warn};
use variegated_controller_types::debug::{name, DebugEvent};
use embassy_rp::adc::Config;
use embassy_rp::watchdog::Watchdog;
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, RawMutex};
use embassy_sync::channel::{Receiver, Sender};
use embassy_sync::mutex::Mutex;
use embassy_sync::pubsub::Publisher;
use embassy_sync::watch;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use heapless::index_map::FnvIndexMap;
use movavg::MovAvg;
use postcard::{from_bytes, from_bytes_crc32, to_slice, to_slice_crc32};
use sequential_storage::map::{SerializationError, Value};
use variegated_control_algorithm::pid::{PidCtrl, PidIn, PidOut};
use variegated_hal::{Boiler, Group, WaterTap, Tank, PeripheralRegistry};
#[cfg(feature = "pwm-steam-valve")]
use variegated_hal::SteamWand;
use variegated_hal::machine_mechanism::dual_boiler_mechanism::DualBoilerFillMechanism;
use variegated_controller_types::{BoilerConfiguration, BoilerControlMode, BoilerControlState, BoilerControlTargetValues, BoilerControlTargetValuesUpdate, BoilerIndex, BoilerStatus, BoilerType, BrewStatus, CommsStatus, Configuration, FillConfiguration, GroupConfiguration, GroupIndex, InputVolumeType, PeripheralStatus, FlowRateType, GroupBrewControlMode, GroupBrewControlState, GroupBrewControlTargetValues, GroupBrewControlTargetValuesUpdate, GroupStatus, MachineCommand, MachineConfiguration, Output, PidLimits, PidParameterTarget, PidParameters, PidTerm, PressureType, RoutineExecutionStatus, RoutineIndex, Status, StorageCommand, KalmanParameters, TemperatureType, WaterLevelType, WaterDispersalPumpStrategy, WaterTapStatus, WaterTapConfiguration, TankConfiguration, TankIndex, TankStatus, RoutineParameters, MachineMode, SteamWandControlState, SteamWandConfiguration, OutputVolumeType};
#[cfg(feature = "pwm-steam-valve")]
use variegated_controller_types::{SteamWandStatus, ValveOpenType};
use crate::routine::{RoutineExecutionContext, InMemoryRoutineRepository, RoutineRepository};
use variegated_controller_types::DualBoilerSingleGroupControllerBoilers::{BrewBoiler, SteamBoiler};
use variegated_controller_types::SingleGroupControllerGroups::SingleGroup;
use variegated_hal::scale::ScaleConfiguration;
use variegated_timekeeping::TimeKeeper;
use crate::schedule::{InMemoryScheduleStore, ScheduleStore};
use crate::settings::SettingsStorage;
use crate::{BLUETOOTH_SCAN_DURATION_MS, BLUETOOTH_SCAN_SLACK_MS};
use variegated_controller_types::bluetooth::{
    BluetoothAssociations, BluetoothScanStatus, BluetoothScanUpdate,
};
use variegated_controller_types::wifi::StoredWifiCredentials;

/// Persistent configuration for dual-boiler single-group machine
/// Uses nested leaf types from variegated-controller-types for clean structure
///
/// IMPORTANT: Structure must remain consistent regardless of feature flags for serialization compatibility
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DualBoilerSingleGroupPersistentConfiguration {
    // General configuration components (leaf types from variegated-controller-types)
    pub machine: MachineConfiguration,
    pub brew_boiler: BoilerConfiguration,
    pub steam_boiler: BoilerConfiguration,
    pub group: GroupConfiguration,
    pub water_tap: WaterTapConfiguration,
    pub tank: TankConfiguration,
    pub steam_wand: SteamWandConfiguration,

    // Default control states (what to reset to on restart)
    pub default_group_brew_control_state: GroupBrewControlState,
    pub default_steam_wand_control_state: SteamWandControlState,

    // Dual-boiler-specific fields (not in general Configuration)
    pub heating_element_contention_strategy: variegated_controller_types::HeatingElementContentionStrategy,
    pub allow_simultaneous_operations: bool,
    pub pump_tacho_pulses_per_liter: Option<f32>,
}

/// Ephemeral (runtime-only) configuration for dual-boiler single-group machine
/// Contains current state that resets to defaults on restart
///
/// IMPORTANT: Structure must remain consistent regardless of feature flags for serialization compatibility
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DualBoilerSingleGroupEphemeralConfiguration {
    pub mode: MachineMode,
    // Current runtime control states (reset to defaults on restart)
    pub group_brew_control_state: GroupBrewControlState,
    pub steam_wand_control_state: SteamWandControlState,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize, PartialEq)]
pub struct DualBoilerSingleGroupConfiguration {
    pub persistent: DualBoilerSingleGroupPersistentConfiguration,
    pub ephemeral: DualBoilerSingleGroupEphemeralConfiguration,
}

impl DualBoilerSingleGroupConfiguration {
    pub fn effective_brew_boiler_control_mode(&self) -> BoilerControlMode {
        if self.ephemeral.mode != MachineMode::On {
            BoilerControlMode::Off
        } else {
            self.persistent.brew_boiler.control_state.mode
        }
    }

    pub fn effective_steam_boiler_control_mode(&self) -> BoilerControlMode {
        if self.ephemeral.mode != MachineMode::On {
            BoilerControlMode::Off
        } else {
            self.persistent.steam_boiler.control_state.mode
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

        log_info!("Serializing DualBoilerSingleGroupConfiguration");

        let slice = match to_slice_crc32(self, buffer, crc.digest()) {
            Ok(bytes) => Ok(bytes.len()),
            Err(postcard::Error::SerializeBufferFull) => {
                log_warn!("Serialization buffer too small");

                Err(SerializationError::BufferTooSmall)
            },
            Err(_) => {
                log_warn!("Serialization error");

                Err(SerializationError::InvalidData)
            },
        };

        log_info!("Serialized DualBoilerSingleGroupConfiguration, len = {}", slice.clone().unwrap_or(0));

        slice
    }

    fn deserialize_from(buffer: &'a [u8]) -> Result<(Self, usize), SerializationError>
    where
        Self: Sized
    {
        log_info!("Deserializing configuration");

        let crc = Crc::<u32>::new(&CRC_32_ISCSI);

        let v = match from_bytes_crc32(buffer, crc.digest()) {
            Ok(value) => Ok(value),
            Err(postcard::Error::DeserializeUnexpectedEnd) => {
                log_warn!("Deserialization buffer too small");

                Err(SerializationError::InvalidFormat)
            },
            Err(postcard::Error::DeserializeBadEnum) => {
                log_warn!("Deserialization bad enum");

                Err(SerializationError::InvalidFormat)
            },
            Err(_) => {
                log_warn!("Deserialization error");
                Err(SerializationError::InvalidFormat)
            },
        };

        match v {
            Ok(value) => {
                log_info!("Deserialized configuration");
                // See `ScheduleItem`'s impl: the whole slice is consumed.
                Ok((value, buffer.len()))
            }
            Err(e) => {
                log_warn!("Deserialization failed");
                Err(e)
            }
        }
    }
}

/// Convert persistent configuration to general Configuration
/// This eliminates manual field mapping and ensures all fields are converted
impl From<DualBoilerSingleGroupPersistentConfiguration> for Configuration {
    fn from(persistent: DualBoilerSingleGroupPersistentConfiguration) -> Self {
        let mut configuration = Configuration::default();

        // Machine-wide config
        configuration.machine_config = persistent.machine;

        // Boiler configurations
        configuration.insert_boiler_configuration(
            BrewBoiler.as_index(),
            persistent.brew_boiler
        );
        configuration.insert_boiler_configuration(
            SteamBoiler.as_index(),
            persistent.steam_boiler
        );

        // Group configuration
        configuration.insert_group_configuration(
            SingleGroup.as_index(),
            persistent.group
        );

        // Water tap configuration
        configuration.insert_water_tap_configuration(0, persistent.water_tap);

        // Tank configuration
        configuration.insert_tank_configuration(0, persistent.tank);

        // Steam wand configuration - only insert if feature is enabled (behavior, not structure)
        #[cfg(feature = "pwm-steam-valve")]
        {
            configuration.insert_steam_wand_configuration(0, persistent.steam_wand);
        }
        // Note: steam_wand field exists in persistent config regardless of feature,
        // but we only use it if the feature is enabled

        configuration
    }
}

/// Also implement the reference version for efficiency
impl From<&DualBoilerSingleGroupPersistentConfiguration> for Configuration {
    fn from(persistent: &DualBoilerSingleGroupPersistentConfiguration) -> Self {
        persistent.clone().into()
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
            steam_wand_control_state: SteamWandControlState::default(),
        }
    }
}

impl Default for DualBoilerSingleGroupPersistentConfiguration {
    fn default() -> Self {
        // Create PID parameters with sensible defaults
        let brew_boiler_temperature_params = PidParameters {
            kp: PidTerm::new(12.0, PidLimits::default()),
            ki: PidTerm::new(0.0003, PidLimits::new_with_limits(0.0, 30.0).unwrap()),
            kd: PidTerm::new(0.0, PidLimits::new_with_limits(-10.0, 10.0).unwrap())
        };
        let brew_boiler_pressure_params = PidParameters {
            kp: PidTerm::new(3.0, PidLimits::default()),
            ki: PidTerm::new(0.01, PidLimits::new_with_limits(-10.0, 10.0).unwrap()),
            kd: PidTerm::new(30.0, PidLimits::new_with_limits(-10.0, 10.0).unwrap())
        };
        let steam_boiler_temperature_params = PidParameters {
            kp: PidTerm::new(12.0, PidLimits::default()),
            ki: PidTerm::new(0.0003, PidLimits::new_with_limits(0.0, 30.0).unwrap()),
            kd: PidTerm::new(0.0, PidLimits::new_with_limits(-10.0, 10.0).unwrap())
        };
        let steam_boiler_pressure_params = PidParameters {
            kp: PidTerm::new(3.0, PidLimits::default()),
            ki: PidTerm::new(0.01, PidLimits::new_with_limits(-10.0, 10.0).unwrap()),
            kd: PidTerm::new(30.0, PidLimits::new_with_limits(-10.0, 10.0).unwrap())
        };
        let pump_flow_rate_params = PidParameters {
            kp: PidTerm::new(10.0, PidLimits::default()),
            ki: PidTerm::new(0.01, PidLimits::new_with_limits(-50.0, 80.0).unwrap()),
            kd: PidTerm::new(30.0, PidLimits::new_with_limits(-10.0, 10.0).unwrap())
        };
        let pump_output_flow_rate_params = PidParameters {
            kp: PidTerm::new(10.0, PidLimits::default()),
            ki: PidTerm::new(0.01, PidLimits::new_with_limits(-50.0, 80.0).unwrap()),
            kd: PidTerm::new(30.0, PidLimits::new_with_limits(-10.0, 10.0).unwrap())
        };
        let pump_pressure_params = PidParameters {
            kp: PidTerm::new(10.0, PidLimits::default()),
            ki: PidTerm::new(0.01, PidLimits::new_with_limits(-50.0, 80.0).unwrap()),
            kd: PidTerm::new(30.0, PidLimits::new_with_limits(-10.0, 10.0).unwrap())
        };

        Self {
            machine: MachineConfiguration {
                heating_element_interlock: false,
                max_shot_logs: 100,
                log_sample_decimation: 1,
                prevent_start_on_empty_tank: false,
                allow_continue_on_empty_tank: true,
            },
            brew_boiler: BoilerConfiguration {
                temperature_pid_parameters: brew_boiler_temperature_params,
                pressure_pid_parameters: brew_boiler_pressure_params,
                control_state: BoilerControlState {
                    mode: BoilerControlMode::Temperature,
                    values: BoilerControlTargetValues {
                        target_temperature: 93.0,
                        target_pressure: 1.0,
                    },
                },
                max_temperature: Some(105.0),
                max_pressure: Some(15.0),
                temperature_sensor_kalman_parameters: None,
                pressure_sensor_kalman_parameters: None,
                fill_config: None, // Brew boiler typically doesn't auto-fill
                supply_tank_index: Some(0),
                minimum_safe_level: None,
            },
            steam_boiler: BoilerConfiguration {
                temperature_pid_parameters: steam_boiler_temperature_params,
                pressure_pid_parameters: steam_boiler_pressure_params,
                control_state: BoilerControlState {
                    mode: BoilerControlMode::Temperature,
                    values: BoilerControlTargetValues {
                        target_temperature: 120.0,
                        target_pressure: 1.5,
                    },
                },
                max_temperature: Some(130.0),
                max_pressure: Some(2.0),
                temperature_sensor_kalman_parameters: None,
                pressure_sensor_kalman_parameters: None,
                fill_config: Some(FillConfiguration {
                    fill_threshold: Some(20),
                    pump_configuration: None,
                }),
                supply_tank_index: Some(0),
                minimum_safe_level: None,
            },
            group: GroupConfiguration {
                flow_rate_pid_parameters: pump_flow_rate_params,
                output_flow_rate_pid_parameters: pump_output_flow_rate_params,
                pressure_pid_parameters: pump_pressure_params,
                brew_control_state: GroupBrewControlState::default(), // Not used - see default_group_brew_control_state
                max_brew_time_seconds: None,
                auto_tare_enabled: false,
                pump_configuration: None,
                pressure_sensor_kalman_parameters: None,
                flow_sensor_pulses_per_liter: None,
                supply_tank_index: Some(0),
            },
            water_tap: WaterTapConfiguration {
                pump_strategy: WaterDispersalPumpStrategy::AlwaysPump(100),
                temperature_target: None,
                max_dispense_time_seconds: None,
                flow_rate_limit: None,
                pump_configuration: None,
                supply_tank_index: Some(0),
            },
            tank: TankConfiguration {
                low_level_warning_threshold: None,
                water_level_sensor_kalman_parameters: None,
                empty_threshold: None,
            },
            // Steam wand config - always present, defaults change based on feature
            steam_wand: {
                #[cfg(feature = "pwm-steam-valve")]
                {
                    SteamWandConfiguration {
                        temperature_target: None,
                        openness: Some(100),
                        purge_time_seconds: None,
                        max_steam_time_seconds: None,
                        auto_purge_enabled: false,
                        supply_tank_index: Some(0),
                    }
                }
                #[cfg(not(feature = "pwm-steam-valve"))]
                {
                    SteamWandConfiguration::default()
                }
            },
            // Default control states (what to reset to on restart)
            default_group_brew_control_state: GroupBrewControlState {
                mode: GroupBrewControlMode::FixedDutyCycle,
                values: GroupBrewControlTargetValues {
                    duty_cycle: 100,
                    ..GroupBrewControlTargetValues::default()
                },
            },
            // Steam wand default control state - always present, defaults change based on feature
            default_steam_wand_control_state: {
                #[cfg(feature = "pwm-steam-valve")]
                {
                    SteamWandControlState::default()
                }
                #[cfg(not(feature = "pwm-steam-valve"))]
                {
                    SteamWandControlState::default()
                }
            },
            // Machine-specific
            heating_element_contention_strategy: variegated_controller_types::HeatingElementContentionStrategy::default(),
            allow_simultaneous_operations: true,
            pump_tacho_pulses_per_liter: None,
        }
    }
}

impl Default for DualBoilerSingleGroupConfiguration {
    fn default() -> Self {
        let persistent = DualBoilerSingleGroupPersistentConfiguration::default();
        DualBoilerSingleGroupConfiguration {
            ephemeral: DualBoilerSingleGroupEphemeralConfiguration {
                mode: MachineMode::Off,
                // Initialize ephemeral control states from persistent defaults
                group_brew_control_state: persistent.default_group_brew_control_state,
                steam_wand_control_state: persistent.default_steam_wand_control_state,
            },
            persistent,
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
    BluetoothStoreT: SettingsStorage<BluetoothAssociations> + 'static,
    WifiStoreT: SettingsStorage<StoredWifiCredentials> + 'static,
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
    #[cfg(feature = "pwm-steam-valve")]
    steam_wand: SteamWand,
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
    machine_config: MachineConfiguration,
    tank_config: TankConfiguration,
    group_config: GroupConfiguration,
    water_tap_config: WaterTapConfiguration,
    brew_boiler_config: BoilerConfiguration,
    steam_boiler_config: BoilerConfiguration,

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
    shot_log_sender: Option<Sender<'a, ChannelM, variegated_controller_types::ShotLog, 2>>,
    /// Annotations waiting to be stamped onto the next shot.
    ///
    /// RAM only, and cleared in full when a shot ends. Beans and grind carry over to the
    /// next shot no more than the dose does: leaving any of them set would silently label
    /// tomorrow's shots with today's coffee, and a wrong label is worse than none.
    pending_annotations: variegated_controller_types::ShotAnnotations,
    /// Where a request that has to touch the card goes.
    ///
    /// `None` on a machine without shot-log storage, in which case such commands are
    /// refused rather than dropped. The storage layer lives on core 1 behind a task --
    /// the card shares the display's SPI bus -- so the controller cannot service these
    /// itself and must hand them across.
    ///
    /// Routed through the controller rather than intercepted in the comms transceiver so
    /// that a `MachineCommand` has exactly one interpreter. Intercepting at the
    /// transceiver would give the same command two different paths depending on whether
    /// it arrived over HTTP or over the debug channel, and the debug one would go on
    /// being refused.
    shot_log_query_sender:
        Option<Sender<'a, ChannelM, crate::shot_log_query::ShotLogQuery, 1>>,
    /// Whether an SD card is inserted, or `None` when this build has no SD storage.
    ///
    /// An atomic rather than a channel: presence is known by the storage task on core 1,
    /// `Status` is built here on core 0, and this is read on the publish path, which
    /// cannot await. One writer, one reader, one bool -- and a lost update corrects
    /// itself on the next publish a few milliseconds later, so `Relaxed` is enough;
    /// nothing else is ordered against it.
    ///
    /// The `Option` is the same `Option` that reaches `Status`, mapped straight through
    /// rather than translated, so there is only one place that decides what "no SD
    /// storage" means.
    sd_card_present: Option<&'a core::sync::atomic::AtomicBool>,

    // Schedule store
    schedule_store: &'static Mutex<StorageM, ScheduleStoreT>,

    // Bluetooth peripheral associations.
    //
    // Its own store, at its own flash range, rather than a field on the persistent
    // configuration. These blobs are postcard with a CRC and no version, so appending a
    // field to the configuration would make every previously stored copy fail to
    // deserialize and silently fall back to `Default` -- a factory reset of every boiler
    // and PID setting on the first boot after the upgrade.
    bluetooth_store: &'static Mutex<StorageM, BluetoothStoreT>,
    // Kept in RAM because it is read on every configuration assembly and written only
    // when the user changes something. The store's own cache would serve, but taking its
    // lock on the publish path would make a configuration publish contend with a flash
    // write.
    bluetooth_associations: BluetoothAssociations,
    bluetooth_scan_sender: Option<Sender<'a, ChannelM, u16, 2>>,
    bluetooth_status: BluetoothScanStatus,
    bluetooth_publish_pending: bool,
    // When the current scan should be considered over even if the comms processor never
    // says so. Without it a comms reset mid-scan would leave `scanning` latched true and
    // the UI's scan button disabled until the next reboot.
    bluetooth_scan_deadline: Option<Instant>,

    // Wi-Fi credentials, at their own key in the settings flash range.
    //
    // A key rather than a field on the persistent configuration, for the reason given on
    // `bluetooth_store` above -- appending to that blob is a factory reset.
    wifi_store: &'static Mutex<StorageM, WifiStoreT>,
    // Kept in RAM for the same reason the association list is: this is the only reader and
    // writer, so re-reading the store could only ever return what is already in hand.
    wifi_credentials: StoredWifiCredentials,
    // Unlike the association list, credentials do *not* ride on the `Configuration`
    // publish -- `Configuration` is what the browser receives and a password has no
    // business on that path -- so a change here has to announce itself.
    wifi_publish_pending: bool,
    // Where an accepted provisioning-window request goes, carrying the duration in
    // milliseconds; zero means close. `None` on a machine whose comms processor is not
    // wired for it, in which case requests are refused rather than silently dropped.
    wifi_provisioning_sender: Option<Sender<'a, ChannelM, u32, 2>>,
    // Where `IdentifyMachine` goes, carrying the instant it was handled.
    //
    // A `Watch` rather than a channel, unlike `wifi_provisioning_sender` directly above: this
    // machine can have both display tasks built, only the latest request matters, and a second
    // Identify arriving mid-flash should extend it rather than queue behind it.
    //
    // An `Instant` rather than a unit so the flash is anchored to when the command was
    // *handled*, not to when a display noticed -- and so the value genuinely changes, which is
    // what `Receiver::try_changed` keys off.
    //
    // `None` on a machine with no display wired for it, in which case Identify does nothing,
    // which the Improv spec explicitly allows.
    identify_publisher: Option<watch::Sender<'a, ChannelM, Instant, 2>>,
    // Raised by `AppDebugOp::ClearWifiCredentials`, which reaches this processor's debug task
    // rather than the command channel -- it is not a machine command, so it has no route into
    // `handle_command` and is polled in the task loop instead.
    //
    // A `Signal` rather than a channel because the request carries no payload and has no
    // ordering to preserve: two clears in a row are one clear. Mirrors
    // `interlock_enabled_signal`'s shape, which is the other `&'static Signal` here.
    clear_wifi_credentials_signal: &'static embassy_sync::signal::Signal<
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        (),
    >,
    // Where credentials go for the transceiver to put on the link. A `Watch` rather than a
    // channel because only the latest value matters and a receiver that missed an
    // intermediate one has missed nothing.
    wifi_credentials_publisher: Option<embassy_sync::watch::Sender<'a, ChannelM, StoredWifiCredentials, 2>>,

    // Status tracking
    previous_status: Option<Status>,
    brew_temperature_movavg: MovAvg<f32, f32, 10>,
    steam_temperature_movavg: MovAvg<f32, f32, 10>,
    brew_start_time: Option<Instant>,
    brew_start_input_volume: Option<InputVolumeType>,
    accumulated_extracted_solids: Option<f32>,
    last_extraction_time: Option<Instant>,
    previous_brew: Option<crate::PreviousBrewInfo>,
    curve_start_time: Option<Instant>,
    comms_status: Option<CommsStatus>,
    comms_status_received_instant: Option<Instant>,
    peripheral_registry: &'a PeripheralRegistry<'a>,
    watchdog: Option<Watchdog>,

    // Heating element coordination signals
    interlock_enabled_signal: &'static embassy_sync::signal::Signal<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, bool>,
    contention_strategy_signal: &'static embassy_sync::signal::Signal<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, variegated_controller_types::HeatingElementContentionStrategy>,

    // Shot state tracking
    current_shot_state: Option<variegated_controller_types::ShotState>,
    flow_rate_history: MovAvg<f32, f32, 10>,
    pressure_history: MovAvg<f32, f32, 10>,
    saturation_start_time: Option<Instant>,
    last_shot_state_sample_time: Option<Instant>,
    input_volume_at_first_drop: Option<InputVolumeType>,
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
    BluetoothStoreT: SettingsStorage<BluetoothAssociations>,
    WifiStoreT: SettingsStorage<StoredWifiCredentials>,
    const N_CHANNEL: usize,
    const N_WATCH: usize,
    const N_SUBS: usize,
    const N_CONFIG_SUBS: usize
> DualBoilerSingleGroupController<'a, ChannelM, BoilerM, GroupM, WaterTapM, TankM, FillM, StorageM, SettingsStoreT, RoutineRepoT, ScheduleStoreT, BluetoothStoreT, WifiStoreT, N_CHANNEL, N_WATCH, N_SUBS, N_CONFIG_SUBS> {
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
        #[cfg(feature = "pwm-steam-valve")]
        mut steam_wand: SteamWand,
        tank: Option<Tank<'a, TankM, N_WATCH>>,
        fill_mechanism: Option<DualBoilerFillMechanism<'a, FillM>>,
        settings_store: &'static Mutex<StorageM, SettingsStoreT>,
        routine_repository: &'static Mutex<StorageM, RoutineRepoT>,
        schedule_store: &'static Mutex<StorageM, ScheduleStoreT>,
        bluetooth_store: &'static Mutex<StorageM, BluetoothStoreT>,
        // Where an accepted `ScanForBluetoothPeripherals` goes, carrying the duration in
        // milliseconds. `None` on a machine whose comms processor is not wired for it, in
        // which case scan requests are refused rather than silently dropped.
        bluetooth_scan_sender: Option<Sender<'a, ChannelM, u16, 2>>,
        wifi_store: &'static Mutex<StorageM, WifiStoreT>,
        // Where an accepted `OpenWifiProvisioningWindow` goes, carrying the duration in
        // milliseconds; zero means close. `None` on a machine whose comms processor is not
        // wired for it, in which case requests are refused rather than silently dropped.
        wifi_provisioning_sender: Option<Sender<'a, ChannelM, u32, 2>>,
        // Where credentials go for the transceiver to put on the link. `None` on a machine
        // with no comms processor.
        wifi_credentials_publisher: Option<embassy_sync::watch::Sender<'a, ChannelM, StoredWifiCredentials, 2>>,
        peripheral_registry: &'a PeripheralRegistry<'a>,
        watchdog: Option<Watchdog>,
        interlock_enabled_signal: &'static embassy_sync::signal::Signal<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, bool>,
        contention_strategy_signal: &'static embassy_sync::signal::Signal<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, variegated_controller_types::HeatingElementContentionStrategy>,
        shot_log_sender: Option<Sender<'a, ChannelM, variegated_controller_types::ShotLog, 2>>,
        // `None` on any build without SD storage, which is what makes
        // `Status::sd_card_present` report "not supported" rather than "no card". The
        // controller itself stays feature-agnostic: `variegated-controller-lib` has no
        // `sd-card-storage` feature on the status-building path, and giving it one would
        // put a `cfg` in the middle of `Status` assembly for a fact the example already
        // knows.
        sd_card_present: Option<&'a core::sync::atomic::AtomicBool>,
        // Where `SetShotAnnotations` goes. `None` on a machine without shot-log storage,
        // which refuses the command rather than accepting it into a void.
        shot_log_query_sender: Option<
            Sender<'a, ChannelM, crate::shot_log_query::ShotLogQuery, 1>,
        >,
        // Where `IdentifyMachine` goes. `None` on a machine with no display to flash.
        identify_publisher: Option<watch::Sender<'a, ChannelM, Instant, 2>>,
        // Raised by the debug op that forgets the stored network. Not an `Option`, unlike its
        // neighbours: every machine has a credential store, so there is no machine for which
        // this does not apply.
        clear_wifi_credentials_signal: &'static embassy_sync::signal::Signal<
            embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
            (),
        >,
    ) -> Self {
        // Create configuration objects from persistent config defaults
        // These will be overridden when the persistent config is loaded from flash
        let default_persistent = DualBoilerSingleGroupPersistentConfiguration::default();

        // Simply clone the nested configurations from the default persistent config
        let machine_config = default_persistent.machine.clone();
        let brew_boiler_config = default_persistent.brew_boiler.clone();
        let steam_boiler_config = default_persistent.steam_boiler.clone();
        let tank_config = default_persistent.tank.clone();
        let group_config = default_persistent.group.clone();
        let water_tap_config = default_persistent.water_tap.clone();

        // Initialize steam wand from ephemeral configuration
        #[cfg(feature = "pwm-steam-valve")]
        {
            let default_config = DualBoilerSingleGroupEphemeralConfiguration::default();
            let initial_valve_openness = default_config.steam_wand_control_state.valve_openness;
            let _ = steam_wand.set_steam_valve_openness(initial_valve_openness);
        }

        Self {
            command_channel_receiver,
            status_channel_sender,
            configuration_channel_sender,
            storage_command_sender,
            brew_boiler,
            steam_boiler,
            group,
            water_tap,
            #[cfg(feature = "pwm-steam-valve")]
            steam_wand,
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
            machine_config,
            tank_config,
            group_config,
            water_tap_config,
            brew_boiler_config,
            steam_boiler_config,
            brew_boiler_enabled: true,
            steam_boiler_enabled: true,
            group_brewing: false,
            water_tap_dispensing: false,
            routine_repository,
            schedule_store,
            bluetooth_store,
            bluetooth_associations: BluetoothAssociations::default(),
            bluetooth_scan_sender,
            bluetooth_status: BluetoothScanStatus::default(),
            bluetooth_publish_pending: false,
            bluetooth_scan_deadline: None,
            wifi_store,
            wifi_credentials: StoredWifiCredentials::default(),
            wifi_publish_pending: false,
            wifi_provisioning_sender,
            identify_publisher,
            clear_wifi_credentials_signal,
            wifi_credentials_publisher,
            current_routine: None,
            shot_logger: crate::shot_log::ShotLogger::new(),
            previous_routine_step: None,
            shot_log_sender,
            pending_annotations: variegated_controller_types::ShotAnnotations::new(),
            shot_log_query_sender,
            sd_card_present,
            previous_status: None,
            brew_temperature_movavg: MovAvg::default(),
            steam_temperature_movavg: MovAvg::default(),
            brew_start_time: None,
            brew_start_input_volume: None,
            accumulated_extracted_solids: None,
            last_extraction_time: None,
            previous_brew: None,
            curve_start_time: None,
            comms_status: None,
            comms_status_received_instant: None,
            peripheral_registry,
            watchdog,
            interlock_enabled_signal,
            contention_strategy_signal,

            // Shot state tracking initialization
            current_shot_state: None,
            flow_rate_history: MovAvg::default(),
            pressure_history: MovAvg::default(),
            saturation_start_time: None,
            last_shot_state_sample_time: None,
            input_volume_at_first_drop: None,
        }
    }

    /// Record what a scale currently reads as the dose for the next shot.
    ///
    /// Refuses rather than guesses when the named scale has nothing to say. A scale that
    /// has never reported and a scale that is disconnected both come back `None` here,
    /// and both mean the same thing to the user: press the button again once the scale
    /// is talking. Writing a `0.0` dose instead would be indistinguishable, in the stored
    /// record, from a shot genuinely pulled with an empty basket.
    ///
    /// Not async, and it does not tare. The weight is whatever the scale reads at the
    /// moment the button is pressed, which is what "put the basket on and tag it" means;
    /// a tare here would zero the scale the user just balanced.
    fn tag_dose_from_scale(&mut self, scale: variegated_controller_types::ScaleSelector) {
        use variegated_controller_types::{
            ScaleSelector, ShotAnnotationKey, ShotAnnotationValue,
        };

        let weight = match scale {
            ScaleSelector::GroupScale(index) if index == SingleGroup.as_index() => {
                self.group.get_output_weight()
            }
            // A single-group machine has exactly one group scale. An index for any other
            // group is a client bug, not a missing peripheral, so it is logged as such
            // rather than folded into "the scale is not reporting".
            ScaleSelector::GroupScale(index) => {
                log_warn!("TagDoseFromScale: no group {} on this machine", index);
                return;
            }
        };

        let Some(grams) = weight else {
            log_warn!("TagDoseFromScale: {:?} has no reading to take", scale);
            return;
        };

        match self.pending_annotations.set(
            ShotAnnotationKey::DoseWeight,
            ShotAnnotationValue::Number(grams),
        ) {
            Ok(()) => log_info!("Dose tagged from {:?}: {} g", scale, grams),
            // Only reachable with eight custom annotations already set and no dose among
            // them. Reported, because the alternative is a dose the user asked for and
            // did not get.
            Err(_) => log_warn!(
                "TagDoseFromScale: the annotation block is full ({} entries)",
                self.pending_annotations.len()
            ),
        }
    }

    async fn create_general_configuration(&mut self) -> Configuration {
        // Use the From trait to convert persistent config to Configuration
        // This eliminates ~80 lines of manual field mapping!
        let mut configuration: Configuration = (&self.configuration.persistent).into();

        // Override with runtime ephemeral state where needed
        // Group brew control state comes from ephemeral (current state)
        if let Some(group_config) = configuration.group_configurations.get_mut(&SingleGroup.as_index()) {
            group_config.brew_control_state = self.configuration.ephemeral.group_brew_control_state;
        }

        // Steam wand valve openness comes from ephemeral (current state)
        #[cfg(feature = "pwm-steam-valve")]
        {
            if let Some(steam_wand_config) = configuration.steam_wand_configurations.get_mut(&0) {
                steam_wand_config.openness = Some(self.configuration.ephemeral.steam_wand_control_state.valve_openness);
            }
        }

        // Load schedules with timeout to avoid blocking if optimization is running
        configuration.schedules = match with_timeout(Duration::from_millis(100), self.schedule_store.lock()).await {
            Ok(mut store) => store.get_schedules().await.cloned().collect(),
            Err(_) => {
                log_warn!("Failed to acquire schedule_store lock for configuration (timeout)");
                Vec::new()
            }
        };

        // From the RAM copy, not the store: this runs on the publish path, and taking the
        // store's lock here would put a configuration publish behind a flash write.
        configuration.bluetooth_peripherals = self.bluetooth_associations.0.clone();

        configuration
    }

    /// Persist the association list, and tell the comms processor it changed.
    ///
    /// The push is not a nicety. The comms processor holds no configuration of its own,
    /// so until it is told, an association the user just created does not exist as far as
    /// the radio is concerned. It rides on the `Configuration` publish rather than a
    /// message of its own -- see the transceiver, which compares the list it last sent.
    async fn save_bluetooth_associations(&mut self) {
        match with_timeout(Duration::from_millis(100), self.bluetooth_store.lock()).await {
            Ok(mut store) => {
                if store.save_settings(&self.bluetooth_associations).await.is_err() {
                    log_warn!("Failed to save Bluetooth associations");
                }
            }
            Err(_) => log_warn!("Failed to acquire bluetooth_store lock for save (timeout)"),
        }
        // `publish_configuration_if_changed` compares the *machine* configuration, which
        // these associations are deliberately not part of -- they have their own store.
        // So a change here is invisible to that comparison and needs to say so itself,
        // or an association would not reach the comms processor until something else
        // happened to dirty the configuration.
        self.bluetooth_publish_pending = true;
    }

    /// Persist the Wi-Fi credentials, and tell the comms processor they changed.
    ///
    /// The push is not a nicety: the comms processor holds no configuration of its own, so
    /// until it is told, credentials the user just provisioned do not exist as far as the
    /// radio is concerned.
    ///
    /// Unlike the association list this deliberately does **not** ride on the
    /// `Configuration` publish. That path ends at the browser, and a password has no
    /// business on it -- so this needs a flag of its own rather than reusing
    /// `bluetooth_publish_pending`'s trick of dirtying the configuration.
    /// Forget the stored network, persistently.
    ///
    /// Writing the cleared value is the whole point: the state this reproduces is a machine
    /// that has *never* been provisioned, and one that merely disconnected would come back
    /// knowing a network after the next reboot.
    ///
    /// `save_wifi_credentials` also sets `wifi_publish_pending`, so the comms processor is
    /// told on the next tick and parks waiting to be provisioned. That is what makes a reboot
    /// unnecessary to reach the state -- and still worth doing to prove it survives one.
    ///
    /// A no-op with a distinct log line when there was nothing stored, rather than a silent
    /// one: this is a debug affordance, and "already clear" is a different answer from
    /// "cleared" to whoever just typed it.
    async fn clear_wifi_credentials(&mut self) {
        if self.wifi_credentials.0.is_none() {
            log_info!("Wi-Fi credentials already cleared; nothing to forget");
            return;
        }
        // Never logs the SSID, here or anywhere on this path. The rule is the same one
        // `try_candidate` states on the comms side: a credential is not written to a log that
        // someone may be sharing a screen of while provisioning.
        self.wifi_credentials = StoredWifiCredentials(None);
        self.save_wifi_credentials().await;
        log_warn!("Wi-Fi credentials cleared; this machine is now unprovisioned");
    }

    async fn save_wifi_credentials(&mut self) {
        match with_timeout(Duration::from_millis(100), self.wifi_store.lock()).await {
            Ok(mut store) => {
                if store.save_settings(&self.wifi_credentials).await.is_err() {
                    log_warn!("Failed to save Wi-Fi credentials");
                }
            }
            Err(_) => log_warn!("Failed to acquire wifi_store lock for save (timeout)"),
        }
        self.wifi_publish_pending = true;
    }

    pub async fn task(&mut self) {
        let mut last_pid_update = Instant::now();
        let mut last_published_configuration = self.configuration.clone();
        let mut last_configuration_publish = Instant::now();

        // Once, before the loop. Unlike the machine configuration, which is re-read every
        // tick because it is cheap and the store owns the cache, this is the only reader
        // and writer of the association list -- so loading it repeatedly would be work
        // that could only ever return what is already in hand.
        self.bluetooth_associations = match self.bluetooth_store.lock().await.load_settings().await {
            Ok(associations) => associations,
            Err(_) => {
                log_warn!("Failed to load Bluetooth associations; starting with none");
                BluetoothAssociations::default()
            }
        };
        log_info!("Loaded {} Bluetooth associations", self.bluetooth_associations.0.len());
        // The comms processor asks for these itself at boot, but it has no way to know
        // whether this processor was simply slow to answer, so publish once regardless.
        self.bluetooth_publish_pending = true;

        // Once, before the loop, for the same reason as the association list above.
        self.wifi_credentials = match self.wifi_store.lock().await.load_settings().await {
            Ok(credentials) => credentials,
            Err(_) => {
                log_warn!("Failed to load Wi-Fi credentials; starting with none");
                StoredWifiCredentials::default()
            }
        };
        // Logged as configured-or-not, never as a value. The SSID alone would be harmless,
        // but a log line that prints half a credential is one edit away from printing all
        // of it.
        log_info!(
            "Wi-Fi credentials: {}",
            if self.wifi_credentials.0.is_some() { "configured" } else { "none stored" }
        );
        self.wifi_publish_pending = true;

        loop {
            // A scan the comms processor never reported the end of -- because it reset,
            // or the link dropped mid-scan. Latching `scanning` true would disable the
            // UI's scan button until the next reboot, so time it out here rather than
            // trusting a message that may not arrive.
            if let Some(deadline) = self.bluetooth_scan_deadline {
                if Instant::now() >= deadline {
                    log_warn!("Bluetooth scan timed out without a result from the comms processor");
                    self.bluetooth_scan_deadline = None;
                    self.bluetooth_status.scanning = false;
                }
            }

            // Try to load settings with timeout to avoid blocking if optimization is running
            self.configuration.persistent = match with_timeout(Duration::from_millis(100), self.configuration_store.lock()).await {
                Ok(mut store) => store.load_settings().await.unwrap_or_default(),
                Err(_) => {
                    // If we can't acquire the lock, keep current configuration
                    self.configuration.persistent.clone()
                }
            };

            // Initialize hardware signals with current configuration (only on first iteration)
            if last_pid_update == Instant::now() {
                self.interlock_enabled_signal.signal(self.configuration.persistent.machine.heating_element_interlock);
                self.contention_strategy_signal.signal(self.configuration.persistent.heating_element_contention_strategy);
            }

            last_published_configuration = self.publish_configuration_if_changed(last_published_configuration).await;

            // The debug op that forgets the stored network.
            //
            // Polled here rather than handled in `handle_command`, because it is deliberately
            // not a `MachineCommand`: forgetting a network is something a developer does to a
            // bench, not something a user does to a machine, so it has no business in the
            // palette every client shares.
            //
            // `try_take` rather than `wait`: this loop must keep running the boilers.
            if self.clear_wifi_credentials_signal.try_take().is_some() {
                self.clear_wifi_credentials().await;
            }

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
                    self.handle_routine_exit(false).await;
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

            // The 1 Hz `Status: {:?}` dump that used to live here is gone: status
            // now travels verbatim as `DebugPayload::Status`, also at 1 Hz, so the
            // text form carried nothing the structured payload does not -- and it
            // truncated to 96 characters on the bus, so it did not even carry that.
            let now = Instant::now();

            // Publish configuration every 10 seconds regardless of changes
            if now.saturating_duration_since(last_configuration_publish).as_secs() >= 10 {
                let current_config = self.configuration.clone();
                self.publish_general_configuration().await;
                log_info!("Periodic configuration published");
                last_configuration_publish = now;
                last_published_configuration = current_config;
            }

            // Feed the watchdog to prevent system reset
            if let Some(ref mut watchdog) = self.watchdog {
                watchdog.feed(crate::WATCHDOG_TIMEOUT);
            }

            Timer::after_millis(100).await;
        }
    }

    async fn publish_configuration_if_changed(&mut self, previous_configuration: DualBoilerSingleGroupConfiguration) -> DualBoilerSingleGroupConfiguration {
        // Credentials go out on their own channel, not inside `Configuration`, so this is
        // checked separately from the comparison below rather than folded into it. The
        // whole point is that this value never touches the path the browser reads.
        if self.wifi_publish_pending {
            self.wifi_publish_pending = false;
            if let Some(publisher) = self.wifi_credentials_publisher.as_ref() {
                publisher.send(self.wifi_credentials.clone());
            }
        }

        if self.configuration != previous_configuration || self.bluetooth_publish_pending {
            self.bluetooth_publish_pending = false;
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
    async fn update_brew_boiler(&mut self, delta_t: f32) -> Output {
        if !self.brew_boiler_enabled {
            self.brew_boiler.set_heating_element_duty_cycle(0).await;
            return Output::Off;
        }

        let control_state = self.configuration.persistent.brew_boiler.control_state;
        let mut brew_pv = match self.configuration.effective_brew_boiler_control_mode() {
            BoilerControlMode::Temperature => {
                self.brew_boiler_pid.setpoint = control_state.values.target_temperature as f32;
                self.brew_boiler_pid.set_parameters(self.configuration.persistent.brew_boiler.temperature_pid_parameters);
                self.brew_boiler.get_temperature().unwrap_or(0.0) as f32
            }
            BoilerControlMode::Pressure => {
                self.brew_boiler_pid.setpoint = control_state.values.target_pressure as f32;
                self.brew_boiler_pid.set_parameters(self.configuration.persistent.brew_boiler.pressure_pid_parameters);
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
        let mut brew_demand = brew_pid_out.out.max(0.0);
        self.last_brew_boiler_output = brew_demand;

        // Safety check: disable heating if temperature exceeds maximum
        if let Some(max_temp) = self.brew_boiler_config.max_temperature {
            if let Some(current_temp) = self.brew_boiler.get_temperature() {
                if current_temp >= max_temp {
                    // No interpolated reading: this fires on every iteration of a
                    // 10 Hz control loop for as long as the interlock holds, and an
                    // `f32` at full `Display` precision changes with ADC noise on
                    // essentially every one -- which would defeat the bus sink's
                    // duplicate suppression entirely. The temperature and the
                    // configured maximum both reach the host in `Status` and
                    // `Configuration`; what only this line can say is *which*
                    // interlock tripped.
                    log_warn!("Brew boiler heating disabled: temperature at or above configured maximum");
                    brew_demand = 0.0;
                }
            }
        }

        // Safety check: disable heating if pressure exceeds maximum
        if let Some(max_pressure) = self.brew_boiler_config.max_pressure {
            if let Some(current_pressure) = self.brew_boiler.get_pressure() {
                if current_pressure >= max_pressure {
                    // Constant text, for the reason given on the temperature
                    // interlock above.
                    log_warn!("Brew boiler heating disabled: pressure at or above configured maximum");
                    brew_demand = 0.0;
                }
            }
        }

        // Dry-run protection: disable heating if water level too low
        let brew_boiler_level = self.brew_boiler.get_water_level();
        if !Self::is_boiler_level_safe(brew_boiler_level, &self.brew_boiler_config) {
            log_warn!("Brew boiler heating disabled: water level below minimum safe level");
            brew_demand = 0.0;
        }

        // Set duty cycle directly - coordination handled by hardware device
        self.brew_boiler.set_heating_element_duty_cycle(brew_demand as u8).await;

        Output::PidOutput(brew_pid_out)
    }

    async fn update_steam_boiler(&mut self, delta_t: f32) -> Output {
        if !self.steam_boiler_enabled {
            self.steam_boiler.set_heating_element_duty_cycle(0).await;
            return Output::Off;
        }

        let control_state = self.configuration.persistent.steam_boiler.control_state;
        let mut steam_pv = match self.configuration.effective_steam_boiler_control_mode() {
            BoilerControlMode::Temperature => {
                self.steam_boiler_pid.setpoint = control_state.values.target_temperature as f32;
                self.steam_boiler_pid.set_parameters(self.configuration.persistent.steam_boiler.temperature_pid_parameters);
                self.steam_boiler.get_temperature().unwrap_or(0.0) as f32
            }
            BoilerControlMode::Pressure => {
                self.steam_boiler_pid.setpoint = control_state.values.target_pressure as f32;
                self.steam_boiler_pid.set_parameters(self.configuration.persistent.steam_boiler.pressure_pid_parameters);
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
        let mut steam_demand = steam_pid_out.out.max(0.0);
        self.last_steam_boiler_output = steam_demand;

        // Safety check: disable heating if temperature exceeds maximum
        if let Some(max_temp) = self.steam_boiler_config.max_temperature {
            if let Some(current_temp) = self.steam_boiler.get_temperature() {
                if current_temp >= max_temp {
                    // Constant text, for the reason given on the brew boiler's
                    // temperature interlock.
                    log_warn!("Steam boiler heating disabled: temperature at or above configured maximum");
                    steam_demand = 0.0;
                }
            }
        }

        // Safety check: disable heating if pressure exceeds maximum
        if let Some(max_pressure) = self.steam_boiler_config.max_pressure {
            if let Some(current_pressure) = self.steam_boiler.get_pressure() {
                if current_pressure >= max_pressure {
                    // Constant text, for the reason given on the brew boiler's
                    // temperature interlock.
                    log_warn!("Steam boiler heating disabled: pressure at or above configured maximum");
                    steam_demand = 0.0;
                }
            }
        }

        // Dry-run protection: disable heating if water level too low
        let steam_boiler_level = self.steam_boiler.get_water_level();
        if !Self::is_boiler_level_safe(steam_boiler_level, &self.steam_boiler_config) {
            log_warn!("Steam boiler heating disabled: water level below minimum safe level");
            steam_demand = 0.0;
        }

        // Set duty cycle directly - coordination handled by hardware device
        self.steam_boiler.set_heating_element_duty_cycle(steam_demand as u8).await;

        Output::PidOutput(steam_pid_out)
    }

    fn apply_pump_configuration_limits(&self, duty_cycle: u8, is_off: bool) -> u8 {
        // If pump is off, always return 0 regardless of min_duty_cycle
        if is_off {
            return 0;
        }

        // Apply pump configuration limits if configured
        if let Some(ref config) = self.configuration.persistent.group.pump_configuration {
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
                self.pump_pid.set_parameters(self.configuration.persistent.group.flow_rate_pid_parameters);
                self.group.get_input_flow_rate().unwrap_or(0.0) as f32
            },
            GroupBrewControlMode::GroupFlowRateCurve => {
                let target = control_state.values.flow_rate_curve.evaluate(elapsed_seconds);
                self.pump_pid.setpoint = target;
                self.pump_pid.set_parameters(self.configuration.persistent.group.flow_rate_pid_parameters);
                self.group.get_input_flow_rate().unwrap_or(0.0) as f32
            },
            GroupBrewControlMode::Pressure => {
                self.pump_pid.setpoint = control_state.values.pressure as f32;
                self.pump_pid.set_parameters(self.configuration.persistent.group.pressure_pid_parameters);
                self.group.get_pressure().unwrap_or(0.0) as f32
            },
            GroupBrewControlMode::PressureCurve => {
                let target = control_state.values.pressure_curve.evaluate(elapsed_seconds);
                self.pump_pid.setpoint = target;
                self.pump_pid.set_parameters(self.configuration.persistent.group.pressure_pid_parameters);
                self.group.get_pressure().unwrap_or(0.0) as f32
            },
            GroupBrewControlMode::OutputFlowRate => {
                self.pump_pid.setpoint = control_state.values.output_flow_rate as f32;
                self.pump_pid.set_parameters(self.configuration.persistent.group.output_flow_rate_pid_parameters);
                self.group.get_output_flow_rate().unwrap_or(0.0) as f32
            },
            GroupBrewControlMode::OutputFlowRateCurve => {
                let target = control_state.values.output_flow_rate_curve.evaluate(elapsed_seconds);
                self.pump_pid.setpoint = target;
                self.pump_pid.set_parameters(self.configuration.persistent.group.output_flow_rate_pid_parameters);
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
        if let Some(ref config) = self.configuration.persistent.water_tap.pump_configuration {
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
            let base_duty_cycle = match self.configuration.persistent.water_tap.pump_strategy {
                WaterDispersalPumpStrategy::NoPump => 0, // Valve opens but no pump
                WaterDispersalPumpStrategy::AlwaysPump(duty_cycle) => duty_cycle, // Pump at specified duty cycle
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

        // Check tank status before trying to fill
        let tank_empty = self.is_tank_empty();
        let prevent_on_empty = self.machine_config.prevent_start_on_empty_tank;
        let allow_continue = self.machine_config.allow_continue_on_empty_tank;
        let fill_threshold = self.configuration.persistent.steam_boiler.fill_config
            .as_ref()
            .and_then(|cfg| cfg.fill_threshold);

        if let Some(fill_mechanism) = &mut self.fill_mechanism {
            if let Some(current_level) = self.steam_boiler.get_water_level() {
                fill_mechanism.check_and_fill_if_needed(
                    current_level,
                    fill_threshold,
                    tank_empty,
                    prevent_on_empty,
                    allow_continue,
                ).await;
            }
        }
    }

    async fn send_status(&mut self, brew_boiler_output: Output, steam_boiler_output: Output, pump_output: Output) {
        let brew_boiler_status = BoilerStatus {
            temperature: self.brew_boiler.get_temperature(),
            pressure: self.brew_boiler.get_pressure(),
            water_level: self.brew_boiler.get_water_level(),
            output: brew_boiler_output,
            control_state: self.configuration.persistent.brew_boiler.control_state,
        };

        let steam_boiler_status = BoilerStatus {
            temperature: self.steam_boiler.get_temperature(),
            pressure: self.steam_boiler.get_pressure(),
            water_level: self.steam_boiler.get_water_level(),
            output: steam_boiler_output,
            control_state: self.configuration.persistent.steam_boiler.control_state,
        };

        // Calculate extraction_rate first (needed for both GroupStatus and extracted_solids accumulation)
        let extraction_rate = {
            let ec = self.group.get_output_electrical_conductivity();
            let flow = self.group.get_output_flow_rate()
                .or_else(|| self.group.get_input_flow_rate());
            match (ec, flow) {
                (Some(ec), Some(flow)) => Some(ec * flow),
                _ => None,
            }
        };

        // Accumulate extracted_solids during brew
        if let (Some(accumulated), Some(last_time), Some(rate)) =
            (self.accumulated_extracted_solids, self.last_extraction_time, extraction_rate) {
            let now = Instant::now();
            let delta_millis = now.saturating_duration_since(last_time).as_millis();
            let delta_secs = delta_millis as f32 / 1000.0;
            self.accumulated_extracted_solids = Some(accumulated + rate * delta_secs);
            self.last_extraction_time = Some(now);
        }

        let current_brew = self.brew_start_time.map(|start| {
            let brew_input_volume = match (self.brew_start_input_volume, self.group.get_input_volume()) {
                (Some(start_volume), Some(current_volume)) => Some(current_volume - start_volume),
                _ => None,
            };
            // Calculate output_volume:
            // 1. If output_weight exists, use it (assuming density ~1 g/ml)
            // 2. Else if input_volume_at_first_drop exists, use current_input_volume - first_drop_volume
            let output_volume = if let Some(weight) = self.group.get_output_weight() {
                Some(weight as OutputVolumeType)
            } else if let (Some(first_drop_vol), Some(current_vol)) = (self.input_volume_at_first_drop, self.group.get_input_volume()) {
                Some(current_vol - first_drop_vol)
            } else {
                None
            };
            BrewStatus {
                brew_time: start.elapsed().into(),
                brew_input_volume,
                shot_state: self.current_shot_state,
                extracted_solids: self.accumulated_extracted_solids,
                output_volume,
            }
        });

        let group_status = GroupStatus {
            is_brewing: self.group_brewing,
            three_way_valve_open: self.group.get_three_way_valve_open(),
            current_brew,
            input_flow_rate: self.group.get_input_flow_rate(),
            input_volume: self.group.get_input_volume(),
            output_flow_rate: self.group.get_output_flow_rate(),
            output_weight: self.group.get_output_weight(),
            pressure: self.group.get_pressure(),
            temperature: self.group.get_temperature(),
            output_temperature: self.group.get_output_temperature(),
            output_electrical_conductivity: self.group.get_output_electrical_conductivity(),
            extraction_rate,
            pump_output: pump_output.clone(),
            control_state: self.configuration.ephemeral.group_brew_control_state,
            previous_brew: self.previous_brew.map(|info| info.into()),
        };

        // Calculate current timestamp if we have comms_status
        let (comms_status, comms_status_age) = if let (Some(status), Some(received_instant)) =
            (&self.comms_status, self.comms_status_received_instant) {

            // Calculate elapsed time since reception
            let elapsed = Instant::now().saturating_duration_since(received_instant);
            let current_timestamp = status.timestamp.map(|ts| ts + elapsed.as_secs());

            (Some(CommsStatus {
                timestamp: current_timestamp,
                wifi_connected: status.wifi_connected,
                wifi_rssi: status.wifi_rssi,
                // Carried through unextrapolated, unlike the timestamp above. Provisioning
                // state is a fact about the other processor's radio at the moment it last
                // reported, and there is no way to advance it here -- a window this side
                // guessed had expired would clear the display's indicator while the radio
                // was still advertising.
                improv: status.improv,
                peripheral_connection_status: status.peripheral_connection_status.clone(),
            }),
            // Published alongside, because everything above is extrapolated: the
            // timestamp keeps advancing whether or not the comms processor is alive, so
            // the age is the only thing in `Status` that can say it is not.
            Some(core::time::Duration::from_millis(elapsed.as_millis())))
        } else {
            (self.comms_status.clone(), None)
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
                current_step: rxc.current_step.map(|s| s as u32),
                step_elapsed_time,
                total_elapsed_time,
                resolved_parameters: rxc.parameters.clone(),
            }
        });

        let water_tap_status = WaterTapStatus {
            is_dispensing: self.water_tap_dispensing,
        };

        #[cfg(feature = "pwm-steam-valve")]
        let steam_wand_status = SteamWandStatus {
            is_steaming: self.steam_wand.get_steaming_state(),
            valve_openness: self.steam_wand.get_steam_valve_openness(),
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
            #[cfg(feature = "pwm-steam-valve")]
            steam_wand_statuses: FnvIndexMap::from_iter([(0, steam_wand_status)]),
            #[cfg(not(feature = "pwm-steam-valve"))]
            steam_wand_statuses: FnvIndexMap::new(),
            tank_statuses,
            mode: self.configuration.ephemeral.mode,
            routine_execution,
            comms_status,
            comms_status_age,
            peripheral_status: self.peripheral_registry.get_peripheral_status(),
            current_local_time: TimeKeeper::now_local().map(|t| t.naive_local()),
            bluetooth: self.bluetooth_status.clone(),
            pending_shot_annotations: self.pending_annotations.clone(),
            sd_card_present: self
                .sd_card_present
                .map(|flag| flag.load(core::sync::atomic::Ordering::Relaxed)),
        };

        self.status_channel_sender.publish_immediate(status.clone());

        self.previous_status = Some(status);
    }

    /// Determines if tank is empty based on configuration.
    /// Returns false (not empty) if no tank, no sensor, no threshold, or level is above threshold.
    fn is_tank_empty(&mut self) -> bool {
        match (&mut self.tank, &self.tank_config.empty_threshold) {
            (Some(tank), Some(threshold)) => {
                match tank.get_water_level() {
                    Some(level) => level < *threshold,
                    None => false, // No sensor reading = assume OK
                }
            }
            _ => false, // No tank or no threshold = assume OK (mains water supply)
        }
    }

    /// Determines if we should block starting a new water operation.
    /// Logic:
    /// - If feature disabled: allow
    /// - If tank not empty: allow
    /// - If routine executing AND allow_continue=true: allow (treat as continuation)
    /// - If routine executing AND allow_continue=false: block (abort routine)
    /// - If no routine: block (standalone operation with empty tank)
    fn should_block_water_operation(&mut self) -> bool {
        // Feature disabled?
        if !self.machine_config.prevent_start_on_empty_tank {
            return false;
        }

        // Tank empty?
        let tank_empty = self.is_tank_empty();
        if !tank_empty {
            return false;
        }

        // Tank is empty - check if routine is executing
        if self.current_routine.is_some() {
            // Routine executing: respect allow_continue policy
            return !self.machine_config.allow_continue_on_empty_tank;
        } else {
            // No routine: always block standalone operations on empty tank
            return true;
        }
    }

    /// Determines if a boiler's water level is safe for heating.
    /// Returns true if heating is allowed, false if it should be blocked.
    /// Logic:
    /// - If no minimum_safe_level configured: allow (feature disabled)
    /// - If level sensor reading available: check level >= minimum
    /// - If no sensor reading (but feature enabled): block (assume empty for safety)
    fn is_boiler_level_safe(level: Option<WaterLevelType>, boiler_config: &BoilerConfiguration) -> bool {
        // If no minimum configured, feature is disabled (no level sensor needed)
        let Some(minimum) = boiler_config.minimum_safe_level else {
            return true;
        };

        // Feature enabled: check water level
        match level {
            Some(level) => level >= minimum,  // Have reading: check against threshold
            None => false, // No reading but sensor exists: assume empty (UNSAFE)
        }
    }

    async fn handle_command(&mut self, command: MachineCommand) {
        defmt::info!("Received command: {:?}", command);

        match command {
            MachineCommand::RunRoutine(index, params) => {
                if self.configuration.ephemeral.mode != MachineMode::On {
                    log_warn!("Cannot start routine while not in On mode");
                    return;
                }

                log_info!("Running routine {} with {} parameters", index, params.as_ref().map(|p| p.len()).unwrap_or(0));
                self.handle_routine_start(index, params).await;
            }
            MachineCommand::CancelRoutine => {
                self.handle_routine_exit(true).await;
            }
            _ => {
                // All other commands delegate to the finally handler
                self.handle_routine_finally_commands(command).await;
            }
        }
    }

    /// Handles commands eligible for routine "finally" blocks (cleanup commands).
    /// This is the main command executor for all non-routine-lifecycle commands,
    /// whether from external sources or routine steps.
    async fn handle_routine_finally_commands(&mut self, command: MachineCommand) {
        match command {
            MachineCommand::StartBrewing(_) => {
                if self.configuration.ephemeral.mode != MachineMode::On {
                    log_warn!("Cannot start brewing while not in On mode");
                    return;
                }

                // Validate tank status before starting brewing
                if self.should_block_water_operation() {
                    variegated_log::emit_event(DebugEvent::InterlockTripped { interlock: name("start_brewing_water_tank_low") });
                    return;
                }

                if self.configuration.persistent.allow_simultaneous_operations || !self.water_tap_dispensing {
                    self.start_brewing().await;
                } else {
                    log_warn!("Cannot start brewing while dispensing water (simultaneous operations disabled)");
                }
            }
            MachineCommand::StopBrewing(_) => {
                self.stop_brewing().await;
            }
            MachineCommand::StartPumpingToWaterTap(_) => {
                if self.configuration.ephemeral.mode != MachineMode::On {
                    log_warn!("Cannot start pumping while not in On mode");
                    return;
                }

                // Validate tank status before starting water dispensing
                if self.should_block_water_operation() {
                    variegated_log::emit_event(DebugEvent::InterlockTripped { interlock: name("water_tap_water_tank_low") });
                    return;
                }

                if self.configuration.persistent.allow_simultaneous_operations || !self.group_brewing {
                    self.start_water_tap_dispensing().await;
                } else {
                    log_warn!("Cannot start water tap while brewing (simultaneous operations disabled)");
                }
            }
            MachineCommand::StopPumpingToWaterTap(_) => {
                self.stop_water_tap_dispensing().await;
            }
            #[cfg(feature = "pwm-steam-valve")]
            MachineCommand::StartSteaming(_) => {
                if self.configuration.ephemeral.mode != MachineMode::On {
                    log_warn!("Cannot start steaming while not in On mode");
                    return;
                }

                self.start_steaming().await;
            }
            #[cfg(feature = "pwm-steam-valve")]
            MachineCommand::StopSteaming(_) => {
                self.stop_steaming().await;
            }
            #[cfg(feature = "pwm-steam-valve")]
            MachineCommand::SetSteamValveOpenness(_, openness) => {
                self.set_steam_valve_openness(openness).await;
            }
            MachineCommand::SetBoilerControlTarget(boiler_index, mode, values_update) => {
                log_info!("Setting boiler control mode for boiler {} to {:?} with values {:?}", boiler_index, mode, values_update);
                match boiler_index {
                    0 => {
                        self.configuration.persistent.brew_boiler.control_state.mode = mode;
                        if let Some(update) = values_update {
                            if let Some(temp) = update.temperature {
                                self.configuration.persistent.brew_boiler.control_state.values.target_temperature = temp;
                            }
                            if let Some(pressure) = update.pressure {
                                self.configuration.persistent.brew_boiler.control_state.values.target_pressure = pressure;
                            }
                        }
                        match with_timeout(Duration::from_millis(100), self.configuration_store.lock()).await {
                            Ok(mut store) => { store.save_settings(&self.configuration.persistent).await.ok(); }
                            Err(_) => log_warn!("Failed to acquire configuration_store lock for save (timeout)"),
                        }
                    },
                    1 => {
                        self.configuration.persistent.steam_boiler.control_state.mode = mode;
                        if let Some(update) = values_update {
                            if let Some(temp) = update.temperature {
                                self.configuration.persistent.steam_boiler.control_state.values.target_temperature = temp;
                            }
                            if let Some(pressure) = update.pressure {
                                self.configuration.persistent.steam_boiler.control_state.values.target_pressure = pressure;
                            }
                        }
                        match with_timeout(Duration::from_millis(100), self.configuration_store.lock()).await {
                            Ok(mut store) => { store.save_settings(&self.configuration.persistent).await.ok(); }
                            Err(_) => log_warn!("Failed to acquire configuration_store lock for save (timeout)"),
                        }
                    },
                    _ => {
                        log_error!("Invalid boiler index: {}", boiler_index);
                    }
                }
            }
            MachineCommand::SetBoilerControlTargetValues(boiler_index, update) => {
                log_info!("Setting boiler control values for boiler {} to {:?}", boiler_index, update);
                match boiler_index {
                    0 => {
                        if let Some(temp) = update.temperature {
                            self.configuration.persistent.brew_boiler.control_state.values.target_temperature = temp;
                        }
                        if let Some(pressure) = update.pressure {
                            self.configuration.persistent.brew_boiler.control_state.values.target_pressure = pressure;
                        }
                        match with_timeout(Duration::from_millis(100), self.configuration_store.lock()).await {
                            Ok(mut store) => { store.save_settings(&self.configuration.persistent).await.ok(); }
                            Err(_) => log_warn!("Failed to acquire configuration_store lock for save (timeout)"),
                        }
                    },
                    1 => {
                        if let Some(temp) = update.temperature {
                            self.configuration.persistent.steam_boiler.control_state.values.target_temperature = temp;
                        }
                        if let Some(pressure) = update.pressure {
                            self.configuration.persistent.steam_boiler.control_state.values.target_pressure = pressure;
                        }
                        match with_timeout(Duration::from_millis(100), self.configuration_store.lock()).await {
                            Ok(mut store) => { store.save_settings(&self.configuration.persistent).await.ok(); }
                            Err(_) => log_warn!("Failed to acquire configuration_store lock for save (timeout)"),
                        }
                    },
                    _ => {
                        log_error!("Invalid boiler index: {}", boiler_index);
                    }
                }
            }
            MachineCommand::SetGroupBrewControlTarget(group_index, mode, values_update) => {
                log_info!("Setting group brew control mode for group {} to {:?} with values {:?}", group_index, mode, values_update);
                if group_index == 0 {
                    // Check if this is a curve mode and record start time
                    match mode {
                        GroupBrewControlMode::GroupFlowRateCurve |
                        GroupBrewControlMode::PressureCurve |
                        GroupBrewControlMode::OutputFlowRateCurve |
                        GroupBrewControlMode::FixedDutyCycleCurve => {
                            self.curve_start_time = Some(Instant::now());
                            log_info!("Starting curve control");
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
                    log_error!("Invalid group index: {}", group_index);
                }
            }
            MachineCommand::SetGroupBrewControlTargetValues(group_index, update) => {
                log_info!("Setting group brew control values for group {} to {:?}", group_index, update);
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
                    log_error!("Invalid group index: {}", group_index);
                }
            }
            MachineCommand::SetPidParameters(target, params) => {
                match target {
                    PidParameterTarget::BoilerPressure(boiler_index) => {
                        match boiler_index {
                            0 => self.configuration.persistent.brew_boiler.pressure_pid_parameters = params,
                            1 => self.configuration.persistent.steam_boiler.pressure_pid_parameters = params,
                            _ => log_error!("Invalid boiler index for PID parameters: {}", boiler_index),
                        }
                    }
                    PidParameterTarget::BoilerTemperature(boiler_index) => {
                        match boiler_index {
                            0 => self.configuration.persistent.brew_boiler.temperature_pid_parameters = params,
                            1 => self.configuration.persistent.steam_boiler.temperature_pid_parameters = params,
                            _ => log_error!("Invalid boiler index for PID parameters: {}", boiler_index),
                        }
                    }
                    PidParameterTarget::GroupFlowRate(_) => {
                        self.configuration.persistent.group.flow_rate_pid_parameters = params;
                    }
                    PidParameterTarget::GroupPressure(_) => {
                        self.configuration.persistent.group.pressure_pid_parameters = params;
                    }
                    PidParameterTarget::GroupOutputFlowRate(_) => {
                        self.configuration.persistent.group.output_flow_rate_pid_parameters = params;
                    }
                }
                // Save after updating PID parameters
                match with_timeout(Duration::from_millis(100), self.configuration_store.lock()).await {
                    Ok(mut store) => { store.save_settings(&self.configuration.persistent).await.ok(); }
                    Err(_) => log_warn!("Failed to acquire configuration_store lock for save (timeout)"),
                }
            }
            MachineCommand::EnableBoiler(boiler_index) => {
                match boiler_index {
                    0 => {
                        log_info!("Enabling brew boiler");
                        self.brew_boiler_enabled = true;
                    }
                    1 => {
                        log_info!("Enabling steam boiler");
                        self.steam_boiler_enabled = true;
                    }
                    _ => {
                        log_warn!("Invalid boiler index: {}", boiler_index);
                    }
                }
            }
            MachineCommand::DisableBoiler(boiler_index) => {
                match boiler_index {
                    0 => {
                        log_info!("Disabling brew boiler");
                        self.brew_boiler_enabled = false;
                    }
                    1 => {
                        log_info!("Disabling steam boiler");
                        self.steam_boiler_enabled = false;
                    }
                    _ => {
                        log_warn!("Invalid boiler index: {}", boiler_index);
                    }
                }
            }
            MachineCommand::TareGroupScale(group_index) => {
                if group_index == 0 {
                    log_info!("Taring group scale");
                    let _ = self.group.scale_tare().await;
                } else {
                    log_error!("Invalid group index for taring scale: {}", group_index);
                }
            }
            MachineCommand::ZeroCalibrateGroupScale(group_index) => {
                if group_index == 0 {
                    log_info!("Zero calibrating group scale");
                    let _ = self.group.scale_zero_calibration().await;
                } else {
                    log_error!("Invalid group index for zero calibrating scale: {}", group_index);
                }
            }
            MachineCommand::CalibrateGroupScale100g(group_index) => {
                if group_index == 0 {
                    log_info!("Calibrating group scale with 100g");
                    let _ = self.group.scale_reference_weight_calibration(100).await;
                } else {
                    log_error!("Invalid group index for 100g calibrating scale: {}", group_index);
                }
            }
            MachineCommand::UpdateCommsStatus(status) => {
                log_info!("Updating comms status: wifi={}, timestamp={:?}", status.wifi_connected, status.timestamp);
                self.comms_status = Some(status);
                self.comms_status_received_instant = Some(Instant::now());
            }
            MachineCommand::RunRoutine(_, _) | MachineCommand::CancelRoutine => {
                defmt::warn!("Ignoring unsupported command in finally block: {:?}", command);
            }
            MachineCommand::RemoveScheduleItem(idx) => {
                log_info!("Removing schedule item at index {}", idx);
                match with_timeout(Duration::from_millis(100), self.schedule_store.lock()).await {
                    Ok(mut store) => {
                        let res = store.remove_schedule(idx as usize).await;
                        if res.is_none() {
                            log_warn!("Failed to remove schedule at index {}: index out of bounds", idx);
                        }
                    }
                    Err(_) => log_warn!("Failed to acquire schedule_store lock (timeout)"),
                }
            }
            MachineCommand::AddScheduleItem(item) => {
                log_info!("Adding new schedule item");
                match with_timeout(Duration::from_millis(100), self.schedule_store.lock()).await {
                    Ok(mut store) => store.add_schedule(item).await,
                    Err(_) => log_warn!("Failed to acquire schedule_store lock (timeout)"),
                }
            }
            MachineCommand::UpdateScheduleItem(idx, item) => {
                log_info!("Updating schedule item at index {}", idx);
                match with_timeout(Duration::from_millis(100), self.schedule_store.lock()).await {
                    Ok(mut store) => {
                        let res = store.update_schedule(idx as usize, item).await;
                        if res.is_err() {
                            log_warn!("Failed to update schedule at index {}: index out of bounds", idx);
                        }
                    }
                    Err(_) => log_warn!("Failed to acquire schedule_store lock (timeout)"),
                }
            }
            MachineCommand::AddRoutine(routine) => {
                log_info!("Adding new routine");
                match with_timeout(Duration::from_millis(100), self.routine_repository.lock()).await {
                    Ok(mut repo) => repo.add_routine(routine).await,
                    Err(_) => log_warn!("Failed to acquire routine_repository lock (timeout)"),
                }
            }
            MachineCommand::RemoveRoutine(idx) => {
                match with_timeout(Duration::from_millis(100), self.routine_repository.lock()).await {
                    Ok(mut repo) => {
                        let res = repo.remove_routine(idx).await;
                        if res.is_none() {
                            log_warn!("Failed to remove routine at index {}: index out of bounds", idx);
                        }
                    }
                    Err(_) => log_warn!("Failed to acquire routine_repository lock (timeout)"),
                }
            }
            MachineCommand::UpdateRoutine(idx, Routine) => {
                match with_timeout(Duration::from_millis(100), self.routine_repository.lock()).await {
                    Ok(mut repo) => {
                        let res = repo.update_routine(idx, Routine).await;
                        if res.is_err() {
                            log_warn!("Failed to update routine at index {}: index out of bounds", idx);
                        }
                    }
                    Err(_) => log_warn!("Failed to acquire routine_repository lock (timeout)"),
                }
            }
            MachineCommand::SetMachineMode(mode) => {
                log_info!("Setting machine mode to {:?}", mode);
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
                log_info!("Sending OptimizeConfiguration to storage task");
                if let Err(_) = self.storage_command_sender.try_send(StorageCommand::OptimizeConfiguration) {
                    log_warn!("Failed to send OptimizeConfiguration command: channel full");
                }
            }
            MachineCommand::OptimizeRoutineStorage => {
                log_info!("Sending OptimizeRoutines to storage task");
                if let Err(_) = self.storage_command_sender.try_send(StorageCommand::OptimizeRoutines) {
                    log_warn!("Failed to send OptimizeRoutines command: channel full");
                }
            }
            MachineCommand::OptimizeScheduleStorage => {
                log_info!("Sending OptimizeSchedules to storage task");
                if let Err(_) = self.storage_command_sender.try_send(StorageCommand::OptimizeSchedules) {
                    log_warn!("Failed to send OptimizeSchedules command: channel full");
                }
            }
            MachineCommand::SetGroupPumpConfiguration(group_index, config) => {
                if group_index == 0 {
                    log_info!("Setting group pump configuration: {:?}", config);
                    self.configuration.persistent.group.pump_configuration = Some(config);
                    match with_timeout(Duration::from_millis(100), self.configuration_store.lock()).await {
                        Ok(mut store) => { store.save_settings(&self.configuration.persistent).await.ok(); }
                        Err(_) => log_warn!("Failed to acquire configuration_store lock for save (timeout)"),
                    }
                } else {
                    log_error!("Invalid group index for pump configuration: {}", group_index);
                }
            }
            MachineCommand::SetWaterTapPumpConfiguration(water_tap_index, config) => {
                if water_tap_index == 0 {
                    log_info!("Setting water tap pump configuration: {:?}", config);
                    self.configuration.persistent.water_tap.pump_configuration = Some(config);
                    match with_timeout(Duration::from_millis(100), self.configuration_store.lock()).await {
                        Ok(mut store) => { store.save_settings(&self.configuration.persistent).await.ok(); }
                        Err(_) => log_warn!("Failed to acquire configuration_store lock for save (timeout)"),
                    }
                } else {
                    log_error!("Invalid water tap index for pump configuration: {}", water_tap_index);
                }
            }
            MachineCommand::SetFillPumpConfiguration(boiler_index, config) => {
                // For dual boiler, we only support fill pump configuration for the steam/service boiler (index 1)
                if boiler_index == 1 {
                    log_info!("Setting fill pump configuration: {:?}", config);
                    // Update the pump_configuration within the FillConfiguration
                    if let Some(ref mut fill_config) = self.configuration.persistent.steam_boiler.fill_config {
                        fill_config.pump_configuration = Some(config);
                    } else {
                        // Create new FillConfiguration with the pump config
                        self.configuration.persistent.steam_boiler.fill_config = Some(FillConfiguration {
                            fill_threshold: None, // Keep existing or use None
                            pump_configuration: Some(config),
                        });
                    }
                    match with_timeout(Duration::from_millis(100), self.configuration_store.lock()).await {
                        Ok(mut store) => { store.save_settings(&self.configuration.persistent).await.ok(); }
                        Err(_) => log_warn!("Failed to acquire configuration_store lock for save (timeout)"),
                    }
                } else {
                    log_error!("Invalid boiler index for fill pump configuration: {} (only boiler 1 supports filling)", boiler_index);
                }
            }
            MachineCommand::InferGroupPressureIntegral(group_index, target_pressure) => {
                if group_index == 0 {
                    log_info!("Inferring group pressure integral for target pressure: {} bar", target_pressure);

                    // Get current duty cycle from the control state configuration
                    let current_duty_cycle = self.configuration.ephemeral.group_brew_control_state.values.duty_cycle;
                    let current_pressure = self.group.get_pressure().unwrap_or(0.0);

                    // Set up PID for pressure control
                    self.pump_pid.setpoint = target_pressure as f32;
                    self.pump_pid.set_parameters(self.configuration.persistent.group.pressure_pid_parameters);

                    // Infer and set the integral
                    self.pump_pid.infer_and_set_integral(current_duty_cycle as f32, current_pressure as f32);

                    log_info!("Set pressure integral based on duty cycle {} and pressure {}", current_duty_cycle, current_pressure);
                } else {
                    log_error!("Invalid group index: {}", group_index);
                }
            }
            MachineCommand::InferGroupFlowRateIntegral(group_index, target_flow_rate) => {
                if group_index == 0 {
                    log_info!("Inferring group flow rate integral for target flow rate: {} ml/s", target_flow_rate);

                    // Get current duty cycle from the control state configuration
                    let current_duty_cycle = self.configuration.ephemeral.group_brew_control_state.values.duty_cycle;
                    let current_flow_rate = self.group.get_input_flow_rate().unwrap_or(0.0);

                    // Set up PID for flow rate control
                    self.pump_pid.setpoint = target_flow_rate as f32;
                    self.pump_pid.set_parameters(self.configuration.persistent.group.flow_rate_pid_parameters);

                    // Infer and set the integral
                    self.pump_pid.infer_and_set_integral(current_duty_cycle as f32, current_flow_rate as f32);

                    log_info!("Set flow rate integral based on duty cycle {} and flow rate {}", current_duty_cycle, current_flow_rate);
                } else {
                    log_error!("Invalid group index: {}", group_index);
                }
            }
            MachineCommand::InferGroupOutputFlowRateIntegral(group_index, target_output_flow_rate) => {
                if group_index == 0 {
                    log_info!("Inferring group output flow rate integral for target: {} ml/s", target_output_flow_rate);

                    // Get current duty cycle from the control state configuration
                    let current_duty_cycle = self.configuration.ephemeral.group_brew_control_state.values.duty_cycle;
                    let current_output_flow_rate = self.group.get_output_flow_rate().unwrap_or(0.0);

                    // Set up PID for output flow rate control
                    self.pump_pid.setpoint = target_output_flow_rate as f32;
                    self.pump_pid.set_parameters(self.configuration.persistent.group.output_flow_rate_pid_parameters);

                    // Infer and set the integral
                    self.pump_pid.infer_and_set_integral(current_duty_cycle as f32, current_output_flow_rate as f32);

                    log_info!("Set output flow rate integral based on duty cycle {} and output flow rate {}", current_duty_cycle, current_output_flow_rate);
                } else {
                    log_error!("Invalid group index: {}", group_index);
                }
            }
            MachineCommand::SetHeatingElementInterlock(enabled) => {
                log_info!("Setting heating element interlock: {}", enabled);
                self.configuration.persistent.machine.heating_element_interlock = enabled;
                self.interlock_enabled_signal.signal(enabled);
                match with_timeout(Duration::from_millis(100), self.configuration_store.lock()).await {
                    Ok(mut store) => { store.save_settings(&self.configuration.persistent).await.ok(); }
                    Err(_) => log_warn!("Failed to acquire configuration_store lock for save (timeout)"),
                }
            }
            MachineCommand::SetHeatingElementContentionStrategy(strategy) => {
                log_info!("Setting heating element contention strategy: {:?}", strategy);
                self.configuration.persistent.heating_element_contention_strategy = strategy;
                self.contention_strategy_signal.signal(strategy);
                match with_timeout(Duration::from_millis(100), self.configuration_store.lock()).await {
                    Ok(mut store) => { store.save_settings(&self.configuration.persistent).await.ok(); }
                    Err(_) => log_warn!("Failed to acquire configuration_store lock for save (timeout)"),
                }
            }
            MachineCommand::SetWaterDispersalPumpStrategy(index, strategy) => {
                log_info!("Setting water dispersal pump strategy for water tap {}: {:?}", index, strategy);
                self.configuration.persistent.water_tap.pump_strategy = strategy;
                match with_timeout(Duration::from_millis(100), self.configuration_store.lock()).await {
                    Ok(mut store) => { store.save_settings(&self.configuration.persistent).await.ok(); }
                    Err(_) => log_warn!("Failed to acquire configuration_store lock for save (timeout)"),
                }
            }
            MachineCommand::AssociateBluetoothPeripheral(association) => {
                let id = association.id;
                if self.bluetooth_associations.upsert(association) {
                    log_info!("Associated Bluetooth peripheral 0x{:04X}", id);
                    self.save_bluetooth_associations().await;
                } else {
                    // Only reachable when the list is full *and* the id is new, since an
                    // existing id replaces in place.
                    log_warn!("Cannot associate 0x{:04X}: no free Bluetooth peripheral slots", id);
                }
            }
            MachineCommand::RemoveBluetoothPeripheral(id) => {
                if self.bluetooth_associations.remove(id) {
                    log_info!("Removed Bluetooth association 0x{:04X}", id);
                    self.save_bluetooth_associations().await;
                } else {
                    log_warn!("No Bluetooth association for 0x{:04X} to remove", id);
                }
            }
            MachineCommand::SetBluetoothPeripheralEnabled(id, enabled) => {
                if self.bluetooth_associations.set_enabled(id, enabled) {
                    log_info!("Bluetooth association 0x{:04X} enabled={}", id, enabled);
                    self.save_bluetooth_associations().await;
                } else {
                    log_warn!("No Bluetooth association for 0x{:04X} to enable/disable", id);
                }
            }
            MachineCommand::ScanForBluetoothPeripherals => {
                // A discovery scan monopolises a radio that is shared with Wi-Fi and with
                // the live links to the peripherals themselves. The ACAIA driver has to
                // heartbeat every couple of seconds or the scale drops the connection, so
                // a scan started mid-shot can cost brew-by-weight the shot it is
                // weighing. The comms processor cannot see any of that -- this is the
                // only processor that knows coffee is being made.
                let busy = self.group_brewing
                    || self.water_tap_dispensing
                    || self.current_routine.is_some();
                #[cfg(feature = "pwm-steam-valve")]
                let busy = busy || self.steam_wand.get_steaming_state();

                if busy {
                    log_warn!("Refusing Bluetooth scan: machine is busy");
                    self.bluetooth_status.blocked = true;
                } else if let Some(sender) = self.bluetooth_scan_sender {
                    match sender.try_send(BLUETOOTH_SCAN_DURATION_MS) {
                        Ok(()) => {
                            log_info!("Starting Bluetooth scan");
                            self.bluetooth_status.blocked = false;
                            self.bluetooth_status.scanning = true;
                            self.bluetooth_status.reports_dropped = 0;
                            // Cleared on *start*, not on finish. The user is about to
                            // pick from this list, and leaving the previous scan's
                            // results visible underneath the new ones would offer them
                            // devices that may no longer be there.
                            self.bluetooth_status.discovered.clear();
                            self.bluetooth_scan_deadline = Some(
                                Instant::now()
                                    + Duration::from_millis(
                                        BLUETOOTH_SCAN_DURATION_MS as u64 + BLUETOOTH_SCAN_SLACK_MS,
                                    ),
                            );
                        }
                        Err(_) => log_warn!("Failed to start Bluetooth scan: channel full"),
                    }
                } else {
                    log_warn!("Refusing Bluetooth scan: no comms processor wired for it");
                    self.bluetooth_status.blocked = true;
                }
            }
            MachineCommand::UpdateBluetoothScan(update) => match update {
                BluetoothScanUpdate::Discovered(device) => {
                    // **Merged, not replaced.** The comms processor reports a device more
                    // than once on purpose: a name and a set of service UUIDs usually
                    // arrive in different advertising reports -- the name in the scan
                    // response, the UUIDs in the advertisement -- and each is forwarded
                    // when it adds something. Overwriting would keep whichever came last
                    // and throw away the other half.
                    match self
                        .bluetooth_status
                        .discovered
                        .iter_mut()
                        .find(|d| d.address == device.address)
                    {
                        Some(existing) => {
                            if !device.name.is_empty() {
                                existing.name = device.name;
                            }
                            if device.suggested_driver.is_some() {
                                existing.suggested_driver = device.suggested_driver;
                            }
                            // `rssi` is deliberately left at the first sighting, matching
                            // what the field claims. It ranks the list; it is not a
                            // measurement, and re-reading it per report would make the
                            // order jump around while the user is reading it.
                        }
                        None => {
                            if self.bluetooth_status.discovered.push(device).is_err() {
                                // Counted where the UI already looks for "results were
                                // lost", rather than in a log nobody reads mid-scan.
                                self.bluetooth_status.reports_dropped =
                                    self.bluetooth_status.reports_dropped.saturating_add(1);
                            }
                        }
                    }
                }
                BluetoothScanUpdate::Finished { reports_dropped } => {
                    log_info!(
                        "Bluetooth scan finished: {} found, {} dropped by the comms processor",
                        self.bluetooth_status.discovered.len(),
                        reports_dropped
                    );
                    self.bluetooth_scan_deadline = None;
                    self.bluetooth_status.scanning = false;
                    // Added to, not overwritten: this processor drops reports of its own
                    // when the list is full, and both losses are the same fact to a user
                    // wondering where their scale went.
                    self.bluetooth_status.reports_dropped =
                        self.bluetooth_status.reports_dropped.saturating_add(reports_dropped);
                }
            },
            MachineCommand::SetPendingShotAnnotations(annotations) => {
                self.pending_annotations = annotations;
                log_debug!(
                    "Pending shot annotations set ({} entries)",
                    self.pending_annotations.len()
                );
            }
            MachineCommand::TagDoseFromScale(scale) => {
                self.tag_dose_from_scale(scale);
            }
            MachineCommand::SetWifiCredentials(credentials) => {
                // Persisted without validation, and that is correct rather than lax: these
                // arrive only from `WifiCredentialsProvisioned`, which the comms processor
                // sends *after* its radio has associated using them. This processor has no
                // radio and could not check anything anyway.
                let stored = StoredWifiCredentials(Some(credentials));
                if self.wifi_credentials != stored {
                    self.wifi_credentials = stored;
                    self.save_wifi_credentials().await;
                    log_info!("Stored new Wi-Fi credentials");
                }
            }
            MachineCommand::OpenWifiProvisioningWindow { duration_ms } => {
                // Refused while the machine is busy, on the same grounds as a discovery
                // scan and using the same predicate. Minutes of connectable advertising
                // share one antenna with Wi-Fi and with the live links to the scales, and
                // the ACAIA drops its connection if its heartbeat misses by a couple of
                // seconds. This is the only processor that knows coffee is being made.
                let busy = self.group_brewing
                    || self.water_tap_dispensing
                    || self.current_routine.is_some();
                #[cfg(feature = "pwm-steam-valve")]
                let busy = busy || self.steam_wand.get_steaming_state();

                if busy {
                    log_warn!("Refusing to open the Wi-Fi provisioning window: machine is busy");
                } else if let Some(sender) = self.wifi_provisioning_sender.as_ref() {
                    if sender.try_send(duration_ms).is_err() {
                        log_warn!("Failed to forward the provisioning window request: channel full");
                    }
                } else {
                    // Refused out loud rather than dropped, so a machine that is not wired
                    // for this says so instead of appearing to accept and then advertising
                    // nothing.
                    log_warn!("This machine has no Wi-Fi provisioning path");
                }
            }
            MachineCommand::CloseWifiProvisioningWindow => {
                if let Some(sender) = self.wifi_provisioning_sender.as_ref() {
                    // Zero means close. One channel rather than two because the two
                    // requests are mutually exclusive and ordering between them matters:
                    // separate channels could deliver a close before the open it was meant
                    // to cancel.
                    let _ = sender.try_send(0);
                }
            }
            MachineCommand::IdentifyMachine => {
                // Still logged as well as published: this is the far end of a round trip that
                // starts in a browser, and the log is the only place both ends are visible at
                // once.
                log_info!("Identify requested");
                if let Some(publisher) = self.identify_publisher.as_ref() {
                    publisher.send(Instant::now());
                }
            }
            MachineCommand::SetShotAnnotations(id, annotations) => {
                // Editing a *stored* shot is a whole-file rewrite on the card, which
                // happens on core 1. Handed across rather than performed here.
                match self.shot_log_query_sender {
                    // `try_send`, not `await`. This runs on the command-handling path of
                    // the control loop, which must not block on a storage task that may
                    // be mid-write; a depth-1 channel that is already full means a
                    // request is in flight, and the right answer is to refuse this one
                    // rather than to stall the machine behind it.
                    Some(ref sender) => {
                        let query = crate::shot_log_query::ShotLogQuery::SetAnnotations {
                            id,
                            annotations,
                        };
                        if sender.try_send(query).is_err() {
                            log_warn!(
                                "SetShotAnnotations({:?}) refused: a shot-log request is already in flight",
                                id
                            );
                        }
                    }
                    None => log_warn!(
                        "SetShotAnnotations({:?}) ignored: this machine has no shot-log storage",
                        id
                    ),
                }
            }
            #[cfg(not(feature = "pwm-steam-valve"))]
            MachineCommand::StartSteaming(_) | MachineCommand::StopSteaming(_) | MachineCommand::SetSteamValveOpenness(_, _) => {
                log_warn!("Steam wand commands are not supported without the pwm-steam-valve feature");
            }
        }
    }

    async fn start_brewing(&mut self) {
        if !self.group_brewing {
            let current_volume = self.group.get_input_volume();
            variegated_log::emit_event(DebugEvent::BrewStarted { group: SingleGroup.as_index() });
            self.group_brewing = true;
            self.brew_start_time = Some(Instant::now());
            self.brew_start_input_volume = current_volume;
            self.accumulated_extracted_solids = Some(0.0);
            self.last_extraction_time = Some(Instant::now());

            // Initialize shot state tracking
            self.current_shot_state = Some(variegated_controller_types::ShotState::HeadspaceFill);
            self.flow_rate_history = MovAvg::default(); // Reset history
            self.pressure_history = MovAvg::default(); // Reset history
            self.saturation_start_time = None;
            self.last_shot_state_sample_time = None; // Reset sampling timer
            self.input_volume_at_first_drop = None; // Will be set when transitioning to PostFirstDrop

            self.group.set_brewing_state(true, 0).await;

            // Give initial PID boost for temperature drop compensation
            self.brew_boiler_pid.ki.accumulate += 50.0;

            let _ = self.group.scale_set_configuration(ScaleConfiguration {
                zero_tracking: Some(false),
                smoothing: Some(true)
            }).await;
            let _ = self.group.scale_tare().await;
        } else {
            log_info!("start_brewing() called but group_brewing already true - skipping baseline capture");
        }
    }

    async fn stop_brewing(&mut self) {
        if self.group_brewing {
            // `current_volume` was read only to log it; the promoted event carries
            // the group instead, and the volume is already in `Status`.
            variegated_log::emit_event(DebugEvent::BrewStopped { group: SingleGroup.as_index() });

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

            self.group_brewing = false;
            self.brew_start_time = None;
            self.brew_start_input_volume = None;
            self.accumulated_extracted_solids = None;
            self.last_extraction_time = None;
            self.curve_start_time = None;

            // Clear shot state tracking
            self.current_shot_state = None;
            self.saturation_start_time = None;
            self.input_volume_at_first_drop = None;
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
        const EC_POST_FIRST_DROP_THRESHOLD: f32 = 1.0; // EC > 1.0 definitively indicates coffee is flowing
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

        // Early transition to PostFirstDrop if EC > 1.0 (definitive first-drop detection)
        if let Some(ec) = self.group.get_output_electrical_conductivity() {
            if ec > EC_POST_FIRST_DROP_THRESHOLD && self.current_shot_state != Some(variegated_controller_types::ShotState::PostFirstDrop) {
                // Capture input volume at first drop for output volume calculation
                self.input_volume_at_first_drop = self.group.get_input_volume();
                log_info!("Shot state transition: {:?} -> PostFirstDrop (EC: {} > {}, input_vol: {:?})",
                      self.current_shot_state, ec, EC_POST_FIRST_DROP_THRESHOLD, self.input_volume_at_first_drop);
                self.current_shot_state = Some(variegated_controller_types::ShotState::PostFirstDrop);
                return;
            }
        }

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
                    log_info!("Shot state transition: HeadspaceFill -> Saturation (flow: {}->{}, pressure: {}->{})",
                          flow_avg, current_flow, pressure_avg, current_pressure);
                    self.current_shot_state = Some(variegated_controller_types::ShotState::Saturation);
                    self.saturation_start_time = Some(Instant::now());
                }
            }
            Some(variegated_controller_types::ShotState::Saturation) => {
                // Check for transition to PostFirstDrop
                if let Some(output_weight) = self.group.get_output_weight() {
                    if output_weight > FIRST_DROP_WEIGHT_THRESHOLD {
                        // Capture input volume at first drop for output volume calculation
                        self.input_volume_at_first_drop = self.group.get_input_volume();
                        log_info!("Shot state transition: Saturation -> PostFirstDrop (weight: {}g, input_vol: {:?})",
                              output_weight, self.input_volume_at_first_drop);
                        self.current_shot_state = Some(variegated_controller_types::ShotState::PostFirstDrop);
                    }
                }
            }
            Some(variegated_controller_types::ShotState::PostFirstDrop) => {
                // Final state, no more transitions
            }
            None => {
                // Should not happen during brewing, but handle gracefully
                log_warn!("Shot state is None while brewing - resetting to HeadspaceFill");
                self.current_shot_state = Some(variegated_controller_types::ShotState::HeadspaceFill);
            }
        }
    }

    async fn start_water_tap_dispensing(&mut self) {
        if !self.water_tap_dispensing {
            log_info!("Starting water tap dispensing");
            self.water_tap_dispensing = true;
            // Apply water dispersal pump strategy
            let base_duty_cycle = match self.configuration.persistent.water_tap.pump_strategy {
                WaterDispersalPumpStrategy::NoPump => 0, // Valve opens but no pump
                WaterDispersalPumpStrategy::AlwaysPump(duty_cycle) => duty_cycle, // Pump at specified duty cycle
            };
            let duty_cycle = self.apply_water_tap_pump_configuration_limits(base_duty_cycle, false);
            self.water_tap.set_water_dispensing_state(true, duty_cycle).await;
        }
    }

    async fn stop_water_tap_dispensing(&mut self) {
        if self.water_tap_dispensing {
            log_info!("Stopping water tap dispensing");
            self.water_tap_dispensing = false;
            self.water_tap.set_water_dispensing_state(false, 0).await;
        }
    }

    #[cfg(feature = "pwm-steam-valve")]
    async fn start_steaming(&mut self) {
        variegated_log::emit_event(DebugEvent::SteamStarted);
        if let Err(e) = self.steam_wand.set_steaming_state(true) {
            log_error!("Failed to start steaming: {:?}", e);
        }
    }

    #[cfg(feature = "pwm-steam-valve")]
    async fn stop_steaming(&mut self) {
        variegated_log::emit_event(DebugEvent::SteamStopped);
        if let Err(e) = self.steam_wand.set_steaming_state(false) {
            log_error!("Failed to stop steaming: {:?}", e);
        }
    }

    #[cfg(feature = "pwm-steam-valve")]
    async fn set_steam_valve_openness(&mut self, openness: ValveOpenType) {
        log_info!("Setting steam valve openness to {}%", openness);

        // Update configuration (ephemeral)
        self.configuration.ephemeral.steam_wand_control_state.valve_openness = openness;

        // Update HAL
        if let Err(e) = self.steam_wand.set_steam_valve_openness(openness) {
            log_error!("Failed to set steam valve openness: {:?}", e);
        }
    }

    async fn handle_routine_start(&mut self, routine_index: RoutineIndex, runtime_params: Option<RoutineParameters>) {
        if self.current_routine.is_some() {
            return;
        }

        // Validate tank status before starting routine
        if self.should_block_water_operation() {
            variegated_log::emit_event(DebugEvent::InterlockTripped { interlock: name("run_routine_water_tank_low") });
            return;
        }
        let mut repo = self.routine_repository.lock().await;
        let routine = repo.get_routine(routine_index).await;

        if let Some(routine) = routine {
            log_info!("Running routine");

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
                // Copied, not moved: the pending block is only cleared when the shot
                // ends, so that a shot aborted before it produces a log does not silently
                // lose the beans and grind the user had already entered.
                //
                // Only the user's own annotations. The routine is recorded by
                // `routine_metadata` just below, in the same block -- duplicating it here
                // would put a machine-derived fact somewhere a `SetShotAnnotations` could
                // delete it.
                annotations: self.pending_annotations.clone(),
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
            variegated_log::emit_event(DebugEvent::RoutineStarted { index: routine_index.to_storage_index() });
        } else {
            log_error!("Routine not found: {}", routine_index);
        }
    }

    /// `cancelled` distinguishes the two ways a routine can end. The event is
    /// emitted here rather than at the call sites so the two stay mutually
    /// exclusive: a cancel is not a completion, and a host counting completions
    /// must not see both for one routine.
    async fn handle_routine_exit(&mut self, cancelled: bool) {
        if let Some(routine) = self.current_routine.take() {
            variegated_log::emit_event(if cancelled {
                DebugEvent::RoutineCancelled
            } else {
                DebugEvent::RoutineCompleted
            });

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
                Err(_) => log_warn!("Failed to acquire configuration_store lock for save (timeout)"),
            }
            self.curve_start_time = None;

            // Execute finally commands
            for cmd in finally_commands {
                self.handle_routine_finally_commands(cmd).await;
            }

            // Finish shot logging
            use variegated_controller_types::ShotStatus;
            self.shot_logger.finish_shot(ShotStatus::Completed);

            // Send completed shot log for storage (if sender configured)
            if let Some(ref sender) = self.shot_log_sender {
                if let Some(shot_log) = self.shot_logger.latest_log() {
                    if let Err(_) = sender.try_send(shot_log.clone()) {
                        log_warn!("Failed to send shot log for storage (channel full)");
                    } else {
                        log_debug!("Shot log sent for storage");
                    }
                }
            }

            // Cleared in full, including beans and grind. Carrying any of them forward
            // would label the next shot with this one's coffee whether or not the user
            // changed it -- and an annotation nobody entered is indistinguishable from
            // one they did.
            self.pending_annotations.clear();

            self.previous_routine_step = None;
        } else {
            log_warn!("No routine to exit");
        }
    }
}