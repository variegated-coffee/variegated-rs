
extern crate alloc;

use alloc::vec::Vec;
use crc::{Crc, CRC_32_ISCSI};
use variegated_log::{log_debug, log_error, log_info, log_warn};
use variegated_controller_types::debug::{name, CheckinDetail, CheckinStatus, DebugEvent};
use embassy_rp::watchdog::Watchdog;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_sync::mutex::Mutex;
use embassy_sync::pubsub::Publisher;
use embassy_sync::watch;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use heapless::index_map::FnvIndexMap;
use movavg::MovAvg;
use postcard::{from_bytes_crc32, to_slice_crc32};
use sequential_storage::map::{SerializationError, Value};
use variegated_control_algorithm::pid::{PidCtrl, PidIn, PidOut};
use variegated_hal::{Boiler, Group, WaterTap, Tank, PeripheralRegistry};
#[cfg(feature = "pwm-steam-valve")]
use variegated_hal::SteamWand;
use variegated_hal::machine_mechanism::dual_boiler_mechanism::DualBoilerFillMechanism;
use variegated_controller_types::{BoilerConfiguration, BoilerControlMode, BoilerControlState, BoilerControlTargetValues, BoilerStatus, BrewStatus, CommsStatus, Configuration, DutyCycleType, FillConfiguration, GroupConfiguration, HexadecimalDutyCycleType, InputVolumeType, GroupBrewControlMode, GroupBrewControlState, GroupBrewControlTargetValues, GroupBrewLimitMode, GroupStatus, MachineCommand, MachineConfiguration, Output, PidLimits, PidParameterTarget, PidParameters, PidTerm, PumpOutput, RoutineExecutionStatus, RoutineIndex, Status, StorageCommand, WaterLevelType, WaterDispersalPumpStrategy, WaterTapStatus, WaterTapConfiguration, TankConfiguration, TankStatus, RoutineParameters, MachineMode, SteamWandControlState, SteamWandConfiguration, OutputVolumeType};
use variegated_controller_types::MachineDefinition;
#[cfg(feature = "pwm-steam-valve")]
use variegated_controller_types::{SteamWandStatus, ValveOpenType};
use crate::routine::{RoutineExecutionContext, RoutineRepository};
use variegated_controller_types::DualBoilerSingleGroupControllerBoilers::{BrewBoiler, SteamBoiler};
use variegated_controller_types::SingleGroupControllerGroups::SingleGroup;
use variegated_hal::scale::ScaleConfiguration;
use variegated_timekeeping::TimeKeeper;
use crate::schedule::ScheduleStore;
use crate::settings::SettingsStorage;
use crate::{BLUETOOTH_SCAN_DURATION_MS, BLUETOOTH_SCAN_SLACK_MS};
use variegated_controller_types::bluetooth::{
    BluetoothAssociations, BluetoothScanStatus, BluetoothScanUpdate,
};
use variegated_controller_types::shot_upload::ShotUploadConfig;
use variegated_controller_types::timezone::TimezoneSetting;
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
                limit: GroupBrewLimitMode::Unlimited,
                values: GroupBrewControlTargetValues {
                    duty_cycle: DutyCycleType::FULL,
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
                pump_strategy: WaterDispersalPumpStrategy::AlwaysPump(DutyCycleType::FULL),
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
                limit: GroupBrewLimitMode::Unlimited,
                values: GroupBrewControlTargetValues {
                    duty_cycle: DutyCycleType::FULL,
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
    UploadStoreT: SettingsStorage<ShotUploadConfig> + 'static,
    TimezoneStoreT: SettingsStorage<TimezoneSetting> + 'static,
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
    machine_config: MachineConfiguration,
    tank_config: TankConfiguration,
    // No `group_config` or `water_tap_config` field. The live values are in
    // `self.configuration` and reach the rest of the system through
    // `insert_group_configuration` / `insert_water_tap_configuration`. A copy cloned out at
    // construction would be a snapshot frozen at boot, and a stale copy of a configuration
    // is worse than no copy -- it reads like the real thing.
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
    /// Set when something folded into `Configuration` *after* the comparison in
    /// [`Self::publish_configuration_if_changed`] has changed.
    ///
    /// That comparison is on `DualBoilerSingleGroupConfiguration`, which holds the persistent
    /// and ephemeral machine configuration and nothing else.
    /// [`Self::create_general_configuration`] then folds in three more things that live in
    /// stores of their own -- the schedules, the Bluetooth associations and the shot-upload
    /// view -- and a change to any of them is therefore invisible to that comparison. Without
    /// this flag such a change reached the browser only when the ten-second periodic publish
    /// came round.
    ///
    /// **One flag for all of them rather than one each.** The condition they share is "the
    /// published configuration is stale for a reason the comparison cannot see", and a flag
    /// per field is one more chance to add a fourth field and forget. It was
    /// `bluetooth_publish_pending` when the associations were the only such field.
    configuration_publish_pending: bool,
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

    // Where finished shot logs are uploaded, at its own key in the settings flash range.
    //
    // Same shape as the Wi-Fi trio directly above and for the same three reasons: a key
    // rather than a field on the persistent configuration (appending to that blob is a
    // factory reset); held in RAM because this is its only reader and writer; and published
    // on its own rather than riding the `Configuration` push, because the token is a secret
    // and `Configuration` is what the browser receives.
    //
    // The comms processor has no flash, so this is the only copy on the machine.
    shot_upload_store: &'static Mutex<StorageM, UploadStoreT>,
    shot_upload_config: ShotUploadConfig,
    timezone_store: &'static Mutex<StorageM, TimezoneStoreT>,
    /// The machine's timezone, as stored. Applied to the `TimeKeeper` at boot and on change.
    ///
    /// Kept in RAM beside the store for `bluetooth_associations`' reason: it is read on every
    /// configuration assembly and written only when the user changes it, and taking the
    /// store's lock on the publish path would put a configuration publish behind a flash write.
    timezone: TimezoneSetting,
    shot_upload_publish_pending: bool,
    // Where the config goes for the transceiver to put on the link. A `Watch` for the same
    // reason as `wifi_credentials_publisher`: only the latest value matters.
    shot_upload_config_publisher: Option<embassy_sync::watch::Sender<'a, ChannelM, ShotUploadConfig, 2>>,

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
    /// What this machine declares it can sense.
    ///
    /// Needed here, and not only by the transceiver that ships it to clients, because a
    /// routine's prerequisites are capabilities and this is the only place that says which
    /// peripheral provides which. `peripheral_registry` answers *is it connected*;
    /// this answers *what is it for*, and a prerequisite check needs both.
    machine_definition: &'a MachineDefinition,
    /// Since when a running routine's prerequisites have been unmet, if they are.
    ///
    /// `None` while everything the routine needs is present. See
    /// [`PREREQUISITE_LOSS_GRACE`] for why a running routine is not abandoned the instant
    /// this becomes `Some`.
    prerequisite_lost_since: Option<Instant>,
    watchdog: Option<Watchdog>,

    // Heating element coordination signals
    interlock_enabled_signal: &'static embassy_sync::signal::Signal<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, bool>,
    contention_strategy_signal: &'static embassy_sync::signal::Signal<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, variegated_controller_types::HeatingElementContentionStrategy>,

    // Shot state tracking. The detection itself lives in `variegated-controller-types`
    // rather than here, because this crate cannot be built for the host and the thresholds
    // it uses are only defensible when they can be replayed against recorded shots.
    shot_state: crate::ShotStateTracker,
    input_volume_at_first_drop: Option<InputVolumeType>,

    /// Whether the currently open shot log was opened by `start_brewing` rather than by a
    /// routine. Only that kind is closed by `stop_brewing`; see `finish_manual_shot_log`.
    manual_shot_active: bool,

    /// Where this loop reports its own health. See [`Self::with_checkin`].
    checkin: variegated_checkin::CheckinHandle,
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
    UploadStoreT: SettingsStorage<ShotUploadConfig>,
    TimezoneStoreT: SettingsStorage<TimezoneSetting>,
    const N_CHANNEL: usize,
    const N_WATCH: usize,
    const N_SUBS: usize,
    const N_CONFIG_SUBS: usize
> DualBoilerSingleGroupController<'a, ChannelM, BoilerM, GroupM, WaterTapM, TankM, FillM, StorageM, SettingsStoreT, RoutineRepoT, ScheduleStoreT, BluetoothStoreT, WifiStoreT, UploadStoreT, TimezoneStoreT, N_CHANNEL, N_WATCH, N_SUBS, N_CONFIG_SUBS> {
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
        shot_upload_store: &'static Mutex<StorageM, UploadStoreT>,
        timezone_store: &'static Mutex<StorageM, TimezoneStoreT>,
        // Where the shot-log upload config goes for the transceiver to put on the link.
        // `None` on a machine with no comms processor -- which is also a machine that
        // cannot upload anything, so the config is stored and simply never acted on.
        shot_upload_config_publisher: Option<embassy_sync::watch::Sender<'a, ChannelM, ShotUploadConfig, 2>>,
        peripheral_registry: &'a PeripheralRegistry<'a>,
        machine_definition: &'a MachineDefinition,
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
            // 0-255, not 0-100: the pump PID computes on the pump's own scale.
            pump_pid: super::hexadecimal_limited_pid(),
            last_brew_boiler_output: 0.0,
            last_steam_boiler_output: 0.0,
            configuration_store: settings_store,
            configuration: DualBoilerSingleGroupConfiguration::default(),
//            persistent_configuration: DualBoilerSingleGroupPersistentConfiguration::default(),
//            ephemeral_configuration: DualBoilerSingleGroupEphemeralConfiguration::default(),
            machine_config,
            tank_config,
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
            configuration_publish_pending: false,
            bluetooth_scan_deadline: None,
            wifi_store,
            wifi_credentials: StoredWifiCredentials::default(),
            wifi_publish_pending: false,
            wifi_provisioning_sender,
            identify_publisher,
            clear_wifi_credentials_signal,
            wifi_credentials_publisher,
            shot_upload_store,
            shot_upload_config: ShotUploadConfig::default(),
            shot_upload_publish_pending: false,
            timezone_store,
            timezone: TimezoneSetting::default(),
            shot_upload_config_publisher,
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
            machine_definition,
            prerequisite_lost_since: None,
            watchdog,
            interlock_enabled_signal,
            contention_strategy_signal,

            // Shot state tracking initialization
            shot_state: crate::ShotStateTracker::new(),
            input_volume_at_first_drop: None,
            manual_shot_active: false,
            checkin: variegated_checkin::CheckinHandle::none(),
        }
    }

    /// Report this loop's health into a check-in slot.
    ///
    /// Not an `Option`: [`variegated_checkin::CheckinHandle::none`] points at a slot nothing
    /// reads, so an unwired controller runs the same code with no branch. A caller that sets
    /// this must **not** also wrap `task()` in `variegated_checkin::watch` -- one writer per
    /// slot, and this reports strictly more than the wrapper would.
    pub fn with_checkin(mut self, checkin: variegated_checkin::CheckinHandle) -> Self {
        self.checkin = checkin;
        self
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
        // Same, and note the type conversion is where the token gets dropped -- `From`
        // reduces it to `token_set: bool`, so there is no path from here to the browser
        // carrying the secret even if someone later assigns the whole config by mistake.
        configuration.shot_upload = (&self.shot_upload_config).into();
        // From the RAM copy for the same reason as the two above.
        configuration.timezone = self.timezone.clone();

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
        self.configuration_publish_pending = true;
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
    /// `configuration_publish_pending`'s trick of dirtying the configuration.
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

    /// Mirrors `save_wifi_credentials` exactly, including the 100 ms lock timeout that is
    /// the house style for every store lock taken from the control loop.
    async fn save_shot_upload_config(&mut self) {
        match with_timeout(Duration::from_millis(100), self.shot_upload_store.lock()).await {
            Ok(mut store) => {
                if store.save_settings(&self.shot_upload_config).await.is_err() {
                    log_warn!("Failed to save shot upload config");
                }
            }
            Err(_) => log_warn!("Failed to acquire shot_upload_store lock for save (timeout)"),
        }
        // Two flags, two destinations, and they are not interchangeable. This one sends the
        // full config -- token included -- to the comms processor on its own watch.
        self.shot_upload_publish_pending = true;
        // And this one republishes `Configuration`, which carries the redacted `ShotUploadView`
        // the browser reads. Without it the settings panel showed a stale endpoint for up to
        // ten seconds after an edit, exactly as the schedule list did.
        self.configuration_publish_pending = true;
    }

    /// Persist the timezone, and republish the configuration that carries it.
    ///
    /// No watch of its own, unlike the shot-upload config: the comms processor has no use for
    /// the zone -- it keeps time in UTC and sends UTC seconds — and the browser reads it from
    /// `Configuration`.
    async fn save_timezone(&mut self) {
        match with_timeout(Duration::from_millis(100), self.timezone_store.lock()).await {
            Ok(mut store) => {
                if store.save_settings(&self.timezone).await.is_err() {
                    log_warn!("Failed to save timezone");
                }
            }
            Err(_) => log_warn!("Failed to acquire timezone_store lock for save (timeout)"),
        }
        self.configuration_publish_pending = true;
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
        self.configuration_publish_pending = true;

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

        // Once, before the loop, for the same reason as the two stores above.
        self.shot_upload_config = match self.shot_upload_store.lock().await.load_settings().await {
            Ok(config) => config,
            Err(_) => {
                log_warn!("Failed to load shot upload config; uploads disabled");
                ShotUploadConfig::default()
            }
        };
        // The endpoint is safe to log and is the field you need when uploads go somewhere
        // unexpected; the token is reported only as present-or-not. `ShotUploadConfig`'s
        // own `Debug` elides it, but this path formats the fields itself, so it has to
        // make the same choice explicitly.
        log_info!(
            "Shot upload: endpoint {}, token {}",
            if self.shot_upload_config.endpoint.is_some() { "configured" } else { "none stored" },
            if self.shot_upload_config.token.is_some() { "configured" } else { "none stored" }
        );
        self.shot_upload_publish_pending = true;
        // Explicitly, rather than relying on the association load above having already set it.
        // That is true today -- this whole prologue is straight-line before the loop -- but it
        // is not a property either block states, and the failure it would produce is a stored
        // endpoint the settings panel never shows.
        self.configuration_publish_pending = true;

        // The stored zone, into RAM only. The `TimeKeeper` was already told in `main`, before
        // any task was spawned, because the scheduler and this controller start together and a
        // scheduler tick taken before this point would be a tick in the wrong zone.
        self.timezone = match self.timezone_store.lock().await.load_settings().await {
            Ok(setting) => setting,
            Err(_) => {
                log_warn!("Failed to load timezone; assuming UTC");
                TimezoneSetting::default()
            }
        };

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

            // What this pass will report, downgraded as it goes. Starts clean and only ever
            // gets worse within a pass, so the order of the checks below does not matter --
            // and it is recomputed from scratch each pass, so a condition that clears is
            // reported as cleared on the next tick rather than latching.
            let mut health = CheckinStatus::Good;

            // Try to load settings with timeout to avoid blocking if optimization is running
            self.configuration.persistent = match with_timeout(Duration::from_millis(100), self.configuration_store.lock()).await {
                Ok(mut store) => match store.load_settings().await {
                    Ok(settings) => settings,
                    Err(_) => {
                        // Substituting a `Default` is this loop continuing to run on a
                        // configuration the operator did not choose. It is survivable --
                        // which is why it is a warning and not an error -- but it is the
                        // difference between a machine that is set up and one that looks
                        // set up, and until now it said so only in a log line.
                        health = CheckinStatus::Warning(CheckinDetail::Degraded);
                        Default::default()
                    }
                },
                Err(_) => {
                    // The store is busy -- `optimize_storage` holds this lock across a
                    // flash erase. Reported rather than silently kept, because the machine
                    // is now running on a snapshot rather than on what is stored, and if
                    // the lock is *never* released this is the only place that would say so.
                    health = CheckinStatus::Warning(CheckinDetail::ResourceUnavailable);
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

            // Record a shot log sample.
            //
            // Outside the routine block below, and that placement is the whole of what made
            // manual brews unlogged: `record_sample` needs nothing but a `Status`, and
            // no-ops when no log is open, but sitting inside `if let Some(routine)` it could
            // only ever run for a routine. Routine *events* stay inside, because those
            // genuinely need the routine.
            if let Some(status) = self.previous_status.as_ref() {
                self.shot_logger.record_sample(status);
            }

            // A running routine whose prerequisites have gone away.
            //
            // Debounced rather than immediate: BLE re-association is owned by the comms
            // processor and a link that merely blips must not cost a shot. See
            // `PREREQUISITE_LOSS_GRACE`. Checked before the step below, so a routine that has
            // lost what it needs does not take one more step on stale readings.
            if self.current_routine.is_some() {
                let peripherals = self.peripheral_registry.get_peripheral_status();
                let missing = self
                    .current_routine
                    .as_ref()
                    .and_then(|routine| {
                        crate::routine_prerequisites::unmet_prerequisites(
                            &routine.routine.prerequisites,
                            self.machine_definition,
                            &peripherals,
                        )
                        .next()
                        .copied()
                    });

                match (missing, self.prerequisite_lost_since) {
                    (None, _) => self.prerequisite_lost_since = None,
                    (Some(_), None) => self.prerequisite_lost_since = Some(Instant::now()),
                    (Some(missing), Some(since)) => {
                        if since.elapsed() >= crate::routine_prerequisites::PREREQUISITE_LOSS_GRACE
                        {
                            log_warn!(
                                "Abandoning routine: {:?} has been unavailable for {} ms",
                                missing.capability,
                                since.elapsed().as_millis()
                            );
                            // The cancel path, not the finished one: it runs `finally`, stops
                            // brewing and the tap, and restores the saved configuration.
                            self.handle_routine_exit(true).await;
                            self.prerequisite_lost_since = None;
                        }
                    }
                }
            }

            // Handle routine execution
            if let Some(routine) = &mut self.current_routine {
                if routine.finished_executing {
                    self.handle_routine_exit(false).await;
                } else if let Some(status) = self.previous_status.as_ref() {
                    // Detect and record step transitions
                    if routine.current_step != self.previous_routine_step {
                        if let Some(current_step) = routine.current_step {
                            use variegated_controller_types::RoutineEvent;
                            let event = RoutineEvent {
                                timestamp_millis: self.shot_logger.current_log()
                                    .and_then(|log| log.samples.last())
                                    .map(|s| s.timestamp_millis)
                                    .unwrap_or(0),
                                // Narrowed at the wire boundary. `current_step` stays a
                                // `usize` because it indexes `steps` below; the log
                                // carries a `u32` because `usize` cannot be described to
                                // the schema exporter unambiguously.
                                from_step: self.previous_routine_step.map(|step| step as u32),
                                to_step: current_step as u32,
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

            // No text dump of the status here: it travels verbatim as
            // `DebugPayload::Status` at 1 Hz, and a formatted copy would only truncate
            // to 96 characters on the bus.
            let now = Instant::now();

            // Publish configuration every 10 seconds regardless of changes
            if now.saturating_duration_since(last_configuration_publish).as_secs() >= 10 {
                let current_config = self.configuration.clone();
                self.publish_general_configuration().await;
                //log_info!("Periodic configuration published");
                last_configuration_publish = now;
                last_published_configuration = current_config;
            }

            // Feed the watchdog to prevent system reset
            if let Some(ref mut watchdog) = self.watchdog {
                watchdog.feed(crate::WATCHDOG_TIMEOUT);
            }

            // Recorded beside the watchdog feed, and that adjacency is the point: these are
            // the two things this loop says about its own liveness, and the check-in is the
            // one that can distinguish *this* loop running from the executor running. It
            // goes after the feed so a pass that reached the feed is a pass that reported.
            self.checkin.record(health);

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

        // Separately again, and for the same reason: the upload token is a secret, so it
        // rides its own channel rather than `Configuration`.
        if self.shot_upload_publish_pending {
            self.shot_upload_publish_pending = false;
            if let Some(publisher) = self.shot_upload_config_publisher.as_ref() {
                publisher.send(self.shot_upload_config.clone());
            }
        }

        if self.configuration != previous_configuration || self.configuration_publish_pending {
            self.configuration_publish_pending = false;
            self.publish_general_configuration().await;

            return self.configuration.clone();
        }

        previous_configuration
    }

    async fn publish_general_configuration(&mut self) {
        let config: Configuration = self.create_general_configuration().await;
        self.configuration_channel_sender.publish_immediate(config);
    }

    // Safety interlock logic
    async fn update_brew_boiler(&mut self, delta_t: f32) -> Output {
        if !self.brew_boiler_enabled {
            self.brew_boiler.set_heating_element_duty_cycle(DutyCycleType::OFF).await;
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
                self.brew_boiler.set_heating_element_duty_cycle(DutyCycleType::OFF).await;
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
                    //log_warn!("Brew boiler heating disabled: temperature at or above configured maximum");
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
                    //log_warn!("Brew boiler heating disabled: pressure at or above configured maximum");
                    brew_demand = 0.0;
                }
            }
        }

        // Dry-run protection: disable heating if water level too low
        let brew_boiler_level = self.brew_boiler.get_water_level();
        if !Self::is_boiler_level_safe(brew_boiler_level, &self.brew_boiler_config) {
            //log_warn!("Brew boiler heating disabled: water level below minimum safe level");
            brew_demand = 0.0;
        }

        // Set duty cycle directly - coordination handled by hardware device
        self.brew_boiler.set_heating_element_duty_cycle(DutyCycleType::from_f32(brew_demand)).await;

        Output::PidOutput(brew_pid_out)
    }

    async fn update_steam_boiler(&mut self, delta_t: f32) -> Output {
        if !self.steam_boiler_enabled {
            self.steam_boiler.set_heating_element_duty_cycle(DutyCycleType::OFF).await;
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
                self.steam_boiler.set_heating_element_duty_cycle(DutyCycleType::OFF).await;
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
                    //log_warn!("Steam boiler heating disabled: temperature at or above configured maximum");
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
                    //log_warn!("Steam boiler heating disabled: pressure at or above configured maximum");
                    steam_demand = 0.0;
                }
            }
        }

        // Dry-run protection: disable heating if water level too low
        let steam_boiler_level = self.steam_boiler.get_water_level();
        if !Self::is_boiler_level_safe(steam_boiler_level, &self.steam_boiler_config) {
            //log_warn!("Steam boiler heating disabled: water level below minimum safe level");
            steam_demand = 0.0;
        }

        // Set duty cycle directly - coordination handled by hardware device
        self.steam_boiler.set_heating_element_duty_cycle(DutyCycleType::from_f32(steam_demand)).await;

        Output::PidOutput(steam_pid_out)
    }

    /// Clamp a commanded duty cycle to the operator's configured pump limits.
    ///
    /// The limits are stored as percentages -- they are an operator setting, and they stay
    /// on the scale the editor shows -- so they are converted to the pump's scale here
    /// rather than compared against it directly.
    fn apply_pump_configuration_limits(&self, duty_cycle: HexadecimalDutyCycleType, is_off: bool) -> HexadecimalDutyCycleType {
        // If pump is off, always return 0 regardless of min_duty_cycle
        if is_off {
            return HexadecimalDutyCycleType::OFF;
        }

        // Apply pump configuration limits if configured
        if let Some(ref config) = self.configuration.persistent.group.pump_configuration {
            let mut limited_duty = duty_cycle;

            // Apply minimum duty cycle limit
            if let Some(min_duty) = config.min_duty_cycle {
                limited_duty = limited_duty.max(min_duty.into());
            }

            // Apply maximum duty cycle limit
            if let Some(max_duty) = config.max_duty_cycle {
                limited_duty = limited_duty.min(max_duty.into());
            }

            limited_duty
        } else {
            duty_cycle
        }
    }

    async fn update_group_pump(&mut self, delta_t: f32) -> PumpOutput {
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

        // Everything below the controller is on the pump's 0-255 scale; the operator's
        // percentages are converted here, once, on the way in.
        match control_state.mode {
            GroupBrewControlMode::Off => {
                let duty_cycle = self.apply_pump_configuration_limits(HexadecimalDutyCycleType::OFF, true);
                self.group.set_brewing_state(false, duty_cycle).await;
                PumpOutput::Off
            },
            GroupBrewControlMode::FullOn => {
                let duty_cycle = self.apply_pump_configuration_limits(HexadecimalDutyCycleType::FULL, false);
                self.group.set_brewing_state(true, duty_cycle).await;
                PumpOutput::FixedDutyCycle(duty_cycle)
            },
            GroupBrewControlMode::FixedDutyCycle => {
                let duty_cycle = self.apply_pump_configuration_limits(control_state.values.duty_cycle.into(), false);
                self.group.set_brewing_state(true, duty_cycle).await;
                PumpOutput::FixedDutyCycle(duty_cycle)
            }
            GroupBrewControlMode::FixedDutyCycleCurve => {
                // The curve is authored in percent and evaluates to an `f32`, so going
                // through `DutyCycle` costs no resolution -- the narrowing to a byte happens
                // once, on the far side of the conversion.
                let target_percent = DutyCycleType::from_f32(
                    control_state.values.duty_cycle_curve.evaluate(elapsed_seconds),
                );
                let duty_cycle = self.apply_pump_configuration_limits(target_percent.into(), false);
                self.group.set_brewing_state(true, duty_cycle).await;
                PumpOutput::FixedDutyCycle(duty_cycle)
            }
            _ => {
                // The PID computes natively in 0-255, so this narrows but does not rescale.
                let duty_cycle = self.apply_pump_configuration_limits(
                    HexadecimalDutyCycleType::from_f32(pump_pid_out.out),
                    false,
                );
                self.group.set_brewing_state(true, duty_cycle).await;
                // `out` is republished as the *limited* duty so the status reports what the
                // pump was actually given rather than what the PID asked for.
                PumpOutput::PidOutput(PidOut { out: duty_cycle.value() as f32, ..pump_pid_out })
            },
        }
    }

    /// The water tap's copy of [`Self::apply_pump_configuration_limits`], reading the tap's
    /// own `pump_configuration`. Same percentage-to-raw conversion, same reason.
    fn apply_water_tap_pump_configuration_limits(&self, duty_cycle: HexadecimalDutyCycleType, is_off: bool) -> HexadecimalDutyCycleType {
        // If pump is off, always return 0 regardless of min_duty_cycle
        if is_off {
            return HexadecimalDutyCycleType::OFF;
        }

        // Apply pump configuration limits if configured
        if let Some(ref config) = self.configuration.persistent.water_tap.pump_configuration {
            let mut limited_duty = duty_cycle;

            // Apply minimum duty cycle limit
            if let Some(min_duty) = config.min_duty_cycle {
                limited_duty = limited_duty.max(min_duty.into());
            }

            // Apply maximum duty cycle limit
            if let Some(max_duty) = config.max_duty_cycle {
                limited_duty = limited_duty.min(max_duty.into());
            }

            limited_duty
        } else {
            duty_cycle
        }
    }

    async fn update_water_tap(&mut self, _delta_t: f32) -> PumpOutput {
        if self.water_tap_dispensing {
            // Apply water dispersal pump strategy. The strategy is an operator setting and
            // so is in percent; the pump is not.
            let base_duty_cycle: HexadecimalDutyCycleType = match self.configuration.persistent.water_tap.pump_strategy {
                WaterDispersalPumpStrategy::NoPump => HexadecimalDutyCycleType::OFF, // Valve opens but no pump
                WaterDispersalPumpStrategy::AlwaysPump(duty_cycle) => duty_cycle.into(), // Pump at specified duty cycle
            };
            let duty_cycle = self.apply_water_tap_pump_configuration_limits(base_duty_cycle, false);
            self.water_tap.set_water_dispensing_state(true, duty_cycle).await;
            PumpOutput::FixedDutyCycle(duty_cycle)
        } else {
            let duty_cycle = self.apply_water_tap_pump_configuration_limits(HexadecimalDutyCycleType::OFF, true);
            self.water_tap.set_water_dispensing_state(false, duty_cycle).await;
            PumpOutput::Off
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

    async fn send_status(&mut self, brew_boiler_output: Output, steam_boiler_output: Output, pump_output: PumpOutput) {
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
                shot_state: self.shot_state.state(),
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
            pump_rpm: self.group.get_pump_rpm(),
            // The **resolved** setpoint, taken from the PID rather than from
            // `control_state.values` above. Under a curve those two disagree completely:
            // the stored value is whatever was last written by a non-curve command -- a
            // shot spent entirely in `PressureCurve` leaves it at whatever preinfusion set,
            // for the whole shot -- while this is where the ramp has actually got to.
            //
            // Gated the same way `update_group_pump` gates its own dispatch, so a machine
            // that is not brewing reports no target rather than a stale one.
            brew_control_target: {
                let mode = self.configuration.ephemeral.group_brew_control_state.mode;
                if self.group_brewing && mode != GroupBrewControlMode::Off {
                    Some(variegated_controller_types::BrewControlTarget {
                        mode,
                        value: self.pump_pid.setpoint,
                    })
                } else {
                    None
                }
            },
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
                // Carried through unchanged, like `improv` and unlike the timestamp: it
                // counts syncs that happened on the other processor, and extrapolating a
                // count would be inventing one.
                sntp_sync_seq: status.sntp_sync_seq,
                // Both carried through unchanged, for `improv`'s reason. Neither can be
                // advanced from this side, and the staleness that makes a latched network
                // name misleading is already reported as `comms_status_age`.
                wifi_ssid: status.wifi_ssid.clone(),
                wifi_ip: status.wifi_ip,
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

                // Unconditional, unlike the steam exclusion below. `allow_simultaneous_operations`
                // is about running the two boilers at once -- brewing while steaming -- and the
                // tap is not a boiler: it shares the pump and the same body of brew water with
                // the group. There is no configuration under which starting a shot into a
                // running tap is what the user meant.
                if self.water_tap_dispensing {
                    log_warn!("Cannot start brewing while dispensing water");
                    variegated_log::emit_event(DebugEvent::InterlockTripped { interlock: name("start_brewing_water_tap_active") });
                    return;
                }

                self.start_brewing().await;
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

                // The other half of the brew/tap exclusion above, and unconditional for the same
                // reason.
                if self.group_brewing {
                    log_warn!("Cannot start water tap while brewing");
                    variegated_log::emit_event(DebugEvent::InterlockTripped { interlock: name("water_tap_brewing_active") });
                    return;
                }

                // Steam blocks the tap but the tap does not block steam, and the asymmetry is
                // deliberate. Steam is a valve the user is holding a jug under; refusing to open
                // it leaves the panel button doing nothing, and the GS3's steam button carries
                // its own three-position `steam_valve_state` (gs3-firmware/src/buttons.rs) which
                // is not derived from `Status` and would silently desync from the machine if a
                // `StartSteaming` were dropped here. Refusing the tap has neither problem: its
                // button reads `is_dispensing` back out of `Status` every cycle.
                #[cfg(feature = "pwm-steam-valve")]
                if self.steam_wand.get_steaming_state() {
                    log_warn!("Cannot start water tap while steaming");
                    variegated_log::emit_event(DebugEvent::InterlockTripped { interlock: name("water_tap_steaming_active") });
                    return;
                }

                self.start_water_tap_dispensing().await;
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

                // The one thing `allow_simultaneous_operations` actually names: both boilers
                // working at once. Inert at the default, which is `true` on every configuration
                // in this tree -- brewing and steaming together is the point of a dual boiler.
                if !self.configuration.persistent.allow_simultaneous_operations && self.group_brewing {
                    log_warn!("Cannot start steaming while brewing (simultaneous operations disabled)");
                    variegated_log::emit_event(DebugEvent::InterlockTripped { interlock: name("steam_brewing_active") });
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
                //log_info!("Updating comms status: wifi={}, timestamp={:?}", status.wifi_connected, status.timestamp);
                self.comms_status = Some(status);
                self.comms_status_received_instant = Some(Instant::now());
            }
            MachineCommand::RunRoutine(_, _) | MachineCommand::CancelRoutine => {
                defmt::warn!("Ignoring unsupported command in finally block: {:?}", command);
            }
            MachineCommand::RemoveScheduleItem(idx) => {
                log_info!("Removing schedule item at index {}", idx);
                match with_timeout(Duration::from_millis(100), self.schedule_store.lock()).await {
                    Ok(mut store) => match store.remove_schedule(idx as usize).await {
                        Ok(Some(_)) => self.configuration_publish_pending = true,
                        // Distinguished from the arm below, because they are different
                        // problems: this one is a client naming an index that is not there,
                        // that one is a flash write that failed.
                        Ok(None) => log_warn!("No schedule at index {} to remove", idx),
                        Err(e) => log_warn!("Failed to remove schedule at index {}: {}", idx, e),
                    },
                    Err(_) => log_warn!("Failed to acquire schedule_store lock (timeout)"),
                }
            }
            MachineCommand::AddScheduleItem(item) => {
                log_info!("Adding new schedule item");
                match with_timeout(Duration::from_millis(100), self.schedule_store.lock()).await {
                    Ok(mut store) => match store.add_schedule(item).await {
                        Ok(index) => {
                            log_info!("Added schedule at index {}", index);
                            self.configuration_publish_pending = true;
                        }
                        Err(e) => log_warn!("Failed to add schedule: {}", e),
                    },
                    Err(_) => log_warn!("Failed to acquire schedule_store lock (timeout)"),
                }
            }
            MachineCommand::UpdateScheduleItem(idx, item) => {
                log_info!("Updating schedule item at index {}", idx);
                match with_timeout(Duration::from_millis(100), self.schedule_store.lock()).await {
                    Ok(mut store) => match store.update_schedule(idx as usize, item).await {
                        Ok(()) => self.configuration_publish_pending = true,
                        Err(e) => log_warn!("Failed to update schedule at index {}: {}", idx, e),
                    },
                    Err(_) => log_warn!("Failed to acquire schedule_store lock (timeout)"),
                }
            }
            MachineCommand::AddRoutine(routine) => {
                log_info!("Adding new routine");
                match with_timeout(Duration::from_millis(100), self.routine_repository.lock()).await {
                    Ok(mut repo) => match repo.add_routine(routine).await {
                        Ok(index) => log_info!("Added routine at index {:?}", index),
                        Err(e) => log_warn!("Failed to add routine: {}", e),
                    },
                    Err(_) => log_warn!("Failed to acquire routine_repository lock (timeout)"),
                }
            }
            MachineCommand::RemoveRoutine(idx) => {
                match with_timeout(Duration::from_millis(100), self.routine_repository.lock()).await {
                    Ok(mut repo) => {
                        match repo.remove_routine(idx).await {
                            Ok(Some(_)) => {}
                            // Two different problems, and they used to be the same answer:
                            // a client naming an index that is not there, versus a flash
                            // write that failed.
                            Ok(None) => log_warn!("No routine at index {} to remove", idx),
                            Err(e) => log_warn!("Failed to remove routine at index {}: {}", idx, e),
                        }
                    }
                    Err(_) => log_warn!("Failed to acquire routine_repository lock (timeout)"),
                }
            }
            MachineCommand::UpdateRoutine(idx, routine) => {
                match with_timeout(Duration::from_millis(100), self.routine_repository.lock()).await {
                    Ok(mut repo) => {
                        let res = repo.update_routine(idx, routine).await;
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
            // **No `configuration_publish_pending`, deliberately.** The compaction happens
            // later and in `storage_task`, not here, so a flag set now would advertise a
            // change that has not happened yet. And it would advertise nothing anyway:
            // `Configuration.schedules` is a `Vec<ScheduleItem>` carrying no indices, so
            // renumbering is invisible to a browser.
            //
            // @todo It is *not* invisible to the GS3 panel, whose menu resolves a schedule by
            // storage index. Optimizing while a schedule screen is open can leave that screen
            // pointing at a different schedule. Pre-existing, and out of scope here.
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

                    // Get current duty cycle from the control state configuration. This is a
                    // percentage, and the PID's integral is in the pump's 0-255 units, so it
                    // has to be converted rather than passed through -- seeding a 0-255 loop
                    // with a percentage would start the pump at 1/2.55 of the intended output.
                    //
                    // Note this reads the `FixedDutyCycle` *target* rather than the duty the
                    // pump is actually running at, which `pump_transfer::last_commanded_duty`
                    // explains is only right when the transfer comes from duty-cycle mode.
                    // Pre-existing, and the single-boiler controller no longer does it.
                    let current_duty_cycle: HexadecimalDutyCycleType =
                        self.configuration.ephemeral.group_brew_control_state.values.duty_cycle.into();
                    let current_pressure = self.group.get_pressure().unwrap_or(0.0);

                    // Set up PID for pressure control
                    self.pump_pid.setpoint = target_pressure as f32;
                    self.pump_pid.set_parameters(self.configuration.persistent.group.pressure_pid_parameters);

                    // Infer and set the integral
                    self.pump_pid.infer_and_set_integral(current_duty_cycle.value() as f32, current_pressure as f32);

                    log_info!("Set pressure integral based on duty cycle {}/255 and pressure {}", current_duty_cycle.value(), current_pressure);
                } else {
                    log_error!("Invalid group index: {}", group_index);
                }
            }
            MachineCommand::InferGroupFlowRateIntegral(group_index, target_flow_rate) => {
                if group_index == 0 {
                    log_info!("Inferring group flow rate integral for target flow rate: {} ml/s", target_flow_rate);

                    // A percentage, converted to the PID's 0-255 units. See the pressure
                    // case above for both this and the target-versus-actual caveat.
                    let current_duty_cycle: HexadecimalDutyCycleType =
                        self.configuration.ephemeral.group_brew_control_state.values.duty_cycle.into();
                    let current_flow_rate = self.group.get_input_flow_rate().unwrap_or(0.0);

                    // Set up PID for flow rate control
                    self.pump_pid.setpoint = target_flow_rate as f32;
                    self.pump_pid.set_parameters(self.configuration.persistent.group.flow_rate_pid_parameters);

                    // Infer and set the integral
                    self.pump_pid.infer_and_set_integral(current_duty_cycle.value() as f32, current_flow_rate as f32);

                    log_info!("Set flow rate integral based on duty cycle {}/255 and flow rate {}", current_duty_cycle.value(), current_flow_rate);
                } else {
                    log_error!("Invalid group index: {}", group_index);
                }
            }
            MachineCommand::InferGroupOutputFlowRateIntegral(group_index, target_output_flow_rate) => {
                if group_index == 0 {
                    log_info!("Inferring group output flow rate integral for target: {} ml/s", target_output_flow_rate);

                    // A percentage, converted to the PID's 0-255 units. See the pressure
                    // case above for both this and the target-versus-actual caveat.
                    let current_duty_cycle: HexadecimalDutyCycleType =
                        self.configuration.ephemeral.group_brew_control_state.values.duty_cycle.into();
                    let current_output_flow_rate = self.group.get_output_flow_rate().unwrap_or(0.0);

                    // Set up PID for output flow rate control
                    self.pump_pid.setpoint = target_output_flow_rate as f32;
                    self.pump_pid.set_parameters(self.configuration.persistent.group.output_flow_rate_pid_parameters);

                    // Infer and set the integral
                    self.pump_pid.infer_and_set_integral(current_duty_cycle.value() as f32, current_output_flow_rate as f32);

                    log_info!("Set output flow rate integral based on duty cycle {}/255 and output flow rate {}", current_duty_cycle.value(), current_output_flow_rate);
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
            MachineCommand::SetShotUploadSettings(settings) => {
                // The settings UI's edit. Merged rather than replacing, because the browser
                // is never sent the token and so cannot send it back -- see
                // `ShotUploadTokenUpdate`.
                //
                // The compare-before-save is doing real work here: the panel posts on every
                // save, and a save that changed nothing would otherwise take the store lock
                // and republish.
                let mut updated = self.shot_upload_config.clone();
                updated.apply(settings);
                if self.shot_upload_config != updated {
                    self.shot_upload_config = updated;
                    self.save_shot_upload_config().await;
                    log_info!(
                        "Shot upload settings updated: endpoint {}, token {}, uploads {}",
                        if self.shot_upload_config.endpoint.is_some() { "set" } else { "cleared" },
                        if self.shot_upload_config.token.is_some() { "set" } else { "cleared" },
                        if self.shot_upload_config.enabled { "enabled" } else { "disabled" }
                    );
                }
            }
            MachineCommand::SetShotUploadConfig(config) => {
                // Persisted without validation, for a different reason than the Wi-Fi arm
                // above: this processor *could* parse the URL, but it is not the one that
                // uses it. The comms processor parses it at upload time and reports what it
                // found, and duplicating that here would put two parsers on one string with
                // no mechanism keeping them in agreement.
                //
                // The compare-before-save matters more than usual: `save_settings` short-
                // circuits on an equal cached value, but reaching it at all takes the store
                // lock, and this command can arrive on every reconnect.
                if self.shot_upload_config != config {
                    self.shot_upload_config = config;
                    self.save_shot_upload_config().await;
                    // Never the token, and not even its length -- see the type's `Debug`.
                    log_info!(
                        "Stored shot upload config: endpoint {}, token {}",
                        if self.shot_upload_config.endpoint.is_some() { "set" } else { "cleared" },
                        if self.shot_upload_config.token.is_some() { "set" } else { "cleared" }
                    );
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
            MachineCommand::SetTimezone(setting) => {
                match variegated_timekeeping::TimeZoneWrapper::from_iana_name(setting.as_str()) {
                    Some(zone) => {
                        // Applied before it is stored, so a `TimeKeeper` that refuses it does
                        // not leave flash claiming a zone the scheduler is not using.
                        if let Err(e) = TimeKeeper::set_timezone(zone) {
                            log_warn!("Failed to apply timezone: {:?}", e);
                        } else {
                            log_info!("Timezone set to {}", setting.as_str());
                            self.timezone = setting;
                            self.save_timezone().await;
                        }
                    }
                    // Refused, not stored. This firmware's database is trimmed at build time,
                    // so "unknown" here usually means "outside the region this build carries"
                    // rather than "misspelt" -- and either way the honest outcome is that the
                    // machine keeps the zone it had, and says so.
                    None => log_warn!("Refusing unknown timezone: {}", setting.as_str()),
                }
            }
            MachineCommand::RequestConfiguration => {
                // This board already republishes every 10 seconds, so the command is not
                // load-bearing here the way it is on a single-boiler machine -- but it is
                // implemented all the same, because a consumer that has just come up
                // should not have to wait out someone else's timer, and because a command
                // that works on one controller and is dropped by the other is exactly the
                // sort of divergence that makes the two firmwares need separate handling.
                log_info!("Configuration republish requested");
                self.publish_general_configuration().await;
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
            MachineCommand::DeleteShotLog(id) => {
                // Handed to core 1 like an annotation edit, and for the same reason: the
                // card is not reachable from the control loop.
                match self.shot_log_query_sender {
                    Some(ref sender) => {
                        let query = crate::shot_log_query::ShotLogQuery::Delete { id };
                        if sender.try_send(query).is_err() {
                            log_warn!(
                                "DeleteShotLog({:?}) refused: a shot-log request is already in flight",
                                id
                            );
                        }
                    }
                    None => log_warn!(
                        "DeleteShotLog({:?}) ignored: this machine has no shot-log storage",
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
            self.shot_state.start();
            self.input_volume_at_first_drop = None; // Will be set when transitioning to PostFirstDrop

            self.start_manual_shot_log();

            self.group.set_brewing_state(true, HexadecimalDutyCycleType::OFF).await;

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
            self.shot_state.stop();
            self.input_volume_at_first_drop = None;

            self.finish_manual_shot_log();

            self.group.set_brewing_state(false, HexadecimalDutyCycleType::OFF).await;

            let _ = self.group.scale_set_configuration(ScaleConfiguration {
                zero_tracking: Some(true),
                smoothing: Some(false)
            }).await;
        }
    }

    /// Feed the shot-state tracker this tick's sensor readings.
    ///
    /// The decision itself is [`crate::ShotStateTracker`]; everything
    /// here is gathering inputs and reacting to a transition. `update` does its own rate
    /// limiting, so this can be called on every 100 ms tick.
    fn update_shot_state(&mut self) {
        if !self.group_brewing {
            // Not brewing, no shot state. `stop_brewing` is what clears the tracker.
            return;
        }

        if self.shot_state.state().is_none() {
            // Should not happen during brewing, but handle gracefully.
            log_warn!("Shot state is None while brewing - restarting shot state tracking");
            self.shot_state.start();
        }

        let inputs = crate::ShotStateInputs {
            input_flow_rate: self.group.get_input_flow_rate(),
            pressure: self.group.get_pressure(),
            output_weight: self.group.get_output_weight(),
            output_electrical_conductivity: self.group.get_output_electrical_conductivity(),
        };

        let Some(new_state) = self.shot_state.update(Instant::now().as_millis(), inputs) else {
            return;
        };

        if new_state == variegated_controller_types::ShotState::PostFirstDrop {
            // Captured here rather than inside the tracker: it is the *input* volume, which
            // is the group's to report, and it is only wanted as the baseline for the
            // output-volume derivation below.
            self.input_volume_at_first_drop = self.group.get_input_volume();
        }

        log_info!(
            "Shot state transition: -> {:?} (flow: {:?}, pressure: {:?}, weight: {:?}, ec: {:?}, input_vol: {:?})",
            new_state,
            inputs.input_flow_rate,
            inputs.pressure,
            inputs.output_weight,
            inputs.output_electrical_conductivity,
            self.input_volume_at_first_drop
        );
    }

    /// Open a shot log for a brew nobody scripted.
    ///
    /// A brew started from the panel -- the brew button, or a bare `StartBrewing` -- is as
    /// much a shot as one a routine pulled, and until now only the routine path logged
    /// anything at all. The logger never needed a routine to work: `record_sample` takes
    /// only a `Status`, and `routine_metadata` has always been an `Option`. The three calls
    /// simply lived on the routine path and nowhere else.
    ///
    /// No-ops if a routine is running, or if some other log is already open. Between them
    /// those cover both orders: a routine that issues `StartBrewing` as a step must not
    /// open a second log over its own, and a manual brew that a routine then interrupts is
    /// handed over by `handle_routine_start` rather than closed twice.
    fn start_manual_shot_log(&mut self) {
        use variegated_controller_types::{ShotLogMetadata, ShotStatus, ShotType};

        if self.current_routine.is_some() || self.shot_logger.is_logging() {
            return;
        }

        self.shot_logger.start_shot(ShotLogMetadata {
            // Copied, not moved, and the user's annotations only -- see the equivalent
            // block in `handle_routine_start`.
            annotations: self.pending_annotations.clone(),
            shot_type: ShotType::Manual,
            group_index: SingleGroup.as_index(),
            // No routine, and that is the fact being recorded rather than a gap in one.
            routine_metadata: None,
            start_time_millis: Instant::now().as_millis(),
            end_time_millis: None,
            final_status: ShotStatus::Running,
            recorded_at_unix_millis: None,
        });
        self.manual_shot_active = true;
        log_debug!("Started a manual shot log");
    }

    /// Close a manual shot log and hand it to storage.
    ///
    /// Guarded on `manual_shot_active` rather than on "is a log open", because
    /// `handle_routine_exit` stops brewing *before* it finishes its own log -- so an
    /// unguarded version here would close the routine's log early, from the wrong place,
    /// and the routine path would then find nothing to send.
    fn finish_manual_shot_log(&mut self) {
        use variegated_controller_types::ShotStatus;

        if !self.manual_shot_active {
            return;
        }
        self.manual_shot_active = false;

        self.shot_logger.finish_shot(ShotStatus::Completed);
        self.send_latest_shot_log();

        // Cleared for the same reason the routine path clears them: an annotation carried
        // into the next shot is indistinguishable from one the user entered for it.
        self.pending_annotations.clear();
    }

    /// Hand the most recently finished shot to the storage task, if there is one listening.
    ///
    /// `try_send` rather than `send`: this runs inside the control loop, and a storage task
    /// that has fallen behind must cost a shot log rather than a boiler update.
    fn send_latest_shot_log(&mut self) {
        if let Some(ref sender) = self.shot_log_sender {
            if let Some(shot_log) = self.shot_logger.latest_log() {
                if let Err(_) = sender.try_send(shot_log.clone()) {
                    log_warn!("Failed to send shot log for storage (channel full)");
                } else {
                    log_debug!("Shot log sent for storage");
                }
            }
        }
    }

    async fn start_water_tap_dispensing(&mut self) {
        if !self.water_tap_dispensing {
            log_info!("Starting water tap dispensing");
            self.water_tap_dispensing = true;
            // Apply water dispersal pump strategy
            let base_duty_cycle: HexadecimalDutyCycleType = match self.configuration.persistent.water_tap.pump_strategy {
                WaterDispersalPumpStrategy::NoPump => HexadecimalDutyCycleType::OFF, // Valve opens but no pump
                WaterDispersalPumpStrategy::AlwaysPump(duty_cycle) => duty_cycle.into(), // Pump at specified duty cycle
            };
            let duty_cycle = self.apply_water_tap_pump_configuration_limits(base_duty_cycle, false);
            self.water_tap.set_water_dispensing_state(true, duty_cycle).await;
        }
    }

    async fn stop_water_tap_dispensing(&mut self) {
        if self.water_tap_dispensing {
            log_info!("Stopping water tap dispensing");
            self.water_tap_dispensing = false;
            self.water_tap.set_water_dispensing_state(false, HexadecimalDutyCycleType::OFF).await;
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
            // The backstop, not the gate. Every surface that can start a routine greys out
            // the ones it cannot run, so in normal use this never fires; it catches the
            // debug channel, a scheduled run, and the race where the scale drops between a
            // menu being drawn and the button being pressed.
            let peripherals = self.peripheral_registry.get_peripheral_status();
            if let Some(missing) = crate::routine_prerequisites::unmet_prerequisites(
                &routine.prerequisites,
                self.machine_definition,
                &peripherals,
            )
            .next()
            {
                log_warn!(
                    "Cannot start routine {}: needs {:?}",
                    routine_index,
                    missing.capability
                );
                variegated_log::emit_event(DebugEvent::RoutineRefused {
                    index: routine_index.to_storage_index(),
                    capability: missing.capability,
                });
                return;
            }

            log_info!("Running routine");

            // Shot attributes and linked parameters, in the order `routine_annotations`
            // documents: the routine's standing attributes fill blanks only, then any linked
            // parameter the caller did not supply is seeded from the pending annotations,
            // then the values actually being run with are written back. All of it before the
            // metadata below clones `pending_annotations` into the shot log.
            crate::routine_annotations::apply_static_shot_annotations(
                &routine,
                &mut self.pending_annotations,
            );
            let runtime_params = crate::routine_annotations::seed_linked_parameters(
                &routine,
                runtime_params.clone(),
                &self.pending_annotations,
            );

            // Recorded from `runtime_params`, before the context merges the routine's
            // defaults in -- see `record_linked_parameters` for why the merged map is the
            // wrong input.
            crate::routine_annotations::record_linked_parameters(
                &routine,
                runtime_params.as_ref(),
                &mut self.pending_annotations,
            );

            // Create routine execution context
            let routine_execution_context = RoutineExecutionContext::new(
                routine_index,
                routine.clone(),
                0u8,
                self.configuration.clone(),
                runtime_params
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
                // Filled in by `finish_shot`, from this shot's start `Instant` -- so a
                // shot that begins before the clock syncs still gets a real start time if
                // the clock arrives before the shot ends.
                recorded_at_unix_millis: None,
            };
            // A manual brew already in progress hands its log over here rather than
            // keeping it: `start_shot` closes the open one as `Aborted`, and clearing the
            // flag is what stops the eventual `stop_brewing` from closing *this* log in its
            // place. The routine owns the shot from now on.
            self.manual_shot_active = false;
            self.shot_logger.start_shot(metadata);
            self.previous_routine_step = None;

            // Cleared here rather than only where a routine ends, so a routine started while
            // a previous one's loss timer was still armed does not inherit it.
            self.prerequisite_lost_since = None;
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
            self.send_latest_shot_log();

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