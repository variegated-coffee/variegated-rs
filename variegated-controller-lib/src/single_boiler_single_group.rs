#![no_std]

extern crate alloc;

use alloc::vec::Vec;
use core::ops::DerefMut;
use crc::{Crc, CRC_32_ISCSI};
use defmt::Format;
use variegated_log::{log_debug, log_error, log_info, log_warn};
use variegated_controller_types::debug::{name, DebugEvent};
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, RawMutex};
use embassy_sync::channel::{Receiver, Sender};
use embassy_sync::mutex::Mutex;
use embassy_sync::pubsub::Publisher;
use embassy_sync::watch;
use embassy_rp::watchdog::Watchdog;
use embassy_time::{Instant, Timer};
use heapless::index_map::FnvIndexMap;
use movavg::MovAvg;
use postcard::{from_bytes, from_bytes_crc32, to_slice, to_slice_crc32};
use sequential_storage::map::{SerializationError, Value};
use variegated_control_algorithm::pid::{PidCtrl, PidIn, PidOut};
use variegated_hal::{Boiler, Group, Tank, PeripheralRegistry};
use variegated_controller_types::{BoilerConfiguration, BoilerControlMode, BoilerControlState, BoilerControlTargetValues, BoilerControlTargetValuesUpdate, BoilerIndex, BoilerStatus, BrewStatus, CommsStatus, Configuration, GroupConfiguration, GroupIndex, InputVolumeType, PeripheralStatus, FlowRateType, GroupBrewControlMode, GroupBrewControlState, GroupBrewControlTargetValues, GroupBrewControlTargetValuesUpdate, GroupStatus, MachineCommand, MachineConfiguration, Output, PidLimits, PidParameterTarget, PidParameters, PidTerm, PressureType, RoutineExecutionStatus, RoutineIndex, SingleBoilerSingleGroupControllerState, Status, KalmanParameters, TankConfiguration, TankIndex, TankStatus, WaterLevelType, RoutineParameters, OutputVolumeType};
use crate::routine::{RoutineExecutionContext, InMemoryRoutineRepository, RoutineRepository};
use variegated_controller_types::SingleBoilerSingleGroupControllerBoilers::{BrewBoiler, VirtualSteamBoiler};
use variegated_controller_types::SingleGroupControllerGroups::SingleGroup;
use variegated_hal::scale::ScaleConfiguration;
use variegated_timekeeping::TimeKeeper;
use crate::settings::SettingsStorage;
use variegated_controller_types::bluetooth::{
    BluetoothAssociations, BluetoothScanStatus, BluetoothScanUpdate,
};
use variegated_controller_types::wifi::StoredWifiCredentials;

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

        log_info!("Serializing SingleBoilerSingleGroupConfiguration");

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

        log_info!("Serialized SingleBoilerSingleGroupConfiguration, len = {}", slice.clone().unwrap_or(0));

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
    BluetoothStoreT: SettingsStorage<BluetoothAssociations>,
    WifiStoreT: SettingsStorage<StoredWifiCredentials>,
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
    machine_config: MachineConfiguration,
    tank_config: TankConfiguration,
    group_config: GroupConfiguration,
    boiler_config: BoilerConfiguration,
    routine_repository: &'static Mutex<NoopRawMutex, InMemoryRoutineRepository>,
    current_routine: Option<RoutineExecutionContext<SingleBoilerSingleGroupControllerState, SingleBoilerSingleGroupConfiguration>>,
    shot_logger: crate::shot_log::ShotLogger,
    previous_routine_step: Option<usize>,
    shot_log_sender: Option<Sender<'a, ChannelM, variegated_controller_types::ShotLog, 2>>,
    /// Annotations waiting to be stamped onto the next shot. See the equivalent field in
    /// `dual_boiler_single_group` for why this is RAM-only and cleared in full.
    pending_annotations: variegated_controller_types::ShotAnnotations,
    /// Where a request that has to touch the card goes -- `None` on every single-boiler
    /// build today, since none has shot-log storage. Carried anyway so the two
    /// controllers interpret a `MachineCommand` the same way; see the equivalent field in
    /// `dual_boiler_single_group`.
    shot_log_query_sender:
        Option<Sender<'a, ChannelM, crate::shot_log_query::ShotLogQuery, 1>>,
    /// Whether an SD card is inserted, or `None` when this build has no SD storage --
    /// which is every single-boiler build today. See the equivalent field in
    /// `dual_boiler_single_group` for why this is an atomic.
    ///
    /// Carried even though no caller supplies it, so the two controllers assemble
    /// `Status` the same way. A field present in one and absent from the other is how
    /// they drift into needing separate handling for the same wire type.
    sd_card_present: Option<&'a core::sync::atomic::AtomicBool>,
    previous_status: Option<Status>,
    temperature_movavg: MovAvg<f32, f32, 10>,
    brew_start_time: Option<Instant>,
    brew_start_input_volume: Option<InputVolumeType>,
    accumulated_extracted_solids: Option<f32>,
    last_extraction_time: Option<Instant>,
    previous_brew: Option<crate::PreviousBrewInfo>,
    curve_start_time: Option<Instant>,
    comms_status: Option<CommsStatus>,
    comms_status_received_instant: Option<Instant>,
    peripheral_registry: &'a PeripheralRegistry<'a>,
    /// `Option` because the caller decides whether this board's WATCHDOG peripheral is
    /// available to claim, not this controller -- same shape as the dual-boiler one.
    watchdog: Option<Watchdog>,

    // Bluetooth peripheral associations. See the equivalent block in
    // `dual_boiler_single_group` for why these get a store of their own rather than a
    // field on the persistent configuration.
    //
    // A single-boiler machine has a comms processor like any other, so it can carry a
    // Bluetooth scale or a water sensor; nothing about this is dual-boiler-specific.
    bluetooth_store: BluetoothStoreT,
    bluetooth_associations: BluetoothAssociations,
    bluetooth_associations_loaded: bool,
    bluetooth_scan_sender: Option<Sender<'a, ChannelM, u16, 2>>,
    bluetooth_status: BluetoothScanStatus,
    bluetooth_publish_pending: bool,
    bluetooth_scan_deadline: Option<Instant>,

    // Wi-Fi credentials, at their own key in the settings flash range. Same reasoning as
    // the Bluetooth store above; only the payload type and the key differ.
    wifi_store: WifiStoreT,
    wifi_credentials: StoredWifiCredentials,
    wifi_credentials_loaded: bool,
    // Credentials do *not* ride on the `Configuration` publish -- that path ends at the
    // browser and a password has no business on it -- so a change announces itself.
    wifi_publish_pending: bool,
    wifi_provisioning_sender: Option<Sender<'a, ChannelM, u32, 2>>,
    // Where `IdentifyMachine` goes, carrying the instant it was handled. A `Watch` rather than
    // a channel: only the latest request matters, and a second Identify arriving mid-flash
    // should extend it rather than queue behind it. `None` on a machine with no display to
    // flash, in which case Identify does nothing, which the Improv spec explicitly allows.
    identify_publisher: Option<embassy_sync::watch::Sender<'a, ChannelM, Instant, 2>>,
    // Raised by `AppDebugOp::ClearWifiCredentials`. Not a `MachineCommand`, so it has no route
    // into `handle_command` and is polled in the task loop instead. A `Signal` because the
    // request carries no payload and two clears in a row are one clear.
    clear_wifi_credentials_signal: &'static embassy_sync::signal::Signal<
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        (),
    >,
    // Where credentials go for the transceiver to put on the link. A `Watch` rather than a
    // channel because only the latest value matters.
    wifi_credentials_publisher: Option<embassy_sync::watch::Sender<'a, ChannelM, StoredWifiCredentials, 2>>,
}

impl<
    'a,
    ChannelM: RawMutex,
    M: RawMutex,
    SettingsStoreT: SettingsStorage<SingleBoilerSingleGroupPersistentConfiguration>,
    BluetoothStoreT: SettingsStorage<BluetoothAssociations>,
    WifiStoreT: SettingsStorage<StoredWifiCredentials>,
    const N_CHANNEL: usize,
    const N_WATCH: usize,
    const N_SUBS: usize,
    const N_CONFIG_SUBS: usize
> SingleBoilerSingleGroupController<'a, ChannelM, M, SettingsStoreT, BluetoothStoreT, WifiStoreT, N_CHANNEL, N_WATCH, N_SUBS, N_CONFIG_SUBS> {
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
        machine_config: MachineConfiguration,
        tank_config: TankConfiguration,
        group_config: GroupConfiguration,
        boiler_config: BoilerConfiguration,
        routine_repository: &'static Mutex<NoopRawMutex, InMemoryRoutineRepository>,
        peripheral_registry: &'a PeripheralRegistry<'a>,
        bluetooth_store: BluetoothStoreT,
        // Where an accepted `ScanForBluetoothPeripherals` goes, carrying the duration in
        // milliseconds. `None` on a machine whose comms processor is not wired for it, in
        // which case scan requests are refused rather than silently dropped.
        bluetooth_scan_sender: Option<Sender<'a, ChannelM, u16, 2>>,
        wifi_store: WifiStoreT,
        // Where an accepted `OpenWifiProvisioningWindow` goes, carrying the duration in
        // milliseconds; zero means close. `None` on a machine whose comms processor is not
        // wired for it, in which case requests are refused rather than silently dropped.
        wifi_provisioning_sender: Option<Sender<'a, ChannelM, u32, 2>>,
        // Where credentials go for the transceiver to put on the link. `None` on a machine
        // with no comms processor.
        wifi_credentials_publisher: Option<embassy_sync::watch::Sender<'a, ChannelM, StoredWifiCredentials, 2>>,
        // Already `start`ed by the caller, with `crate::WATCHDOG_TIMEOUT` -- the same
        // constant `task()` feeds it with. embassy-rp 0.10's `feed` sets the new timeout
        // rather than merely refreshing the old one, so the two values have to agree --
        // and a mismatch would compile cleanly while silently changing the window.
        watchdog: Option<Watchdog>,
        shot_log_sender: Option<Sender<'a, ChannelM, variegated_controller_types::ShotLog, 2>>,
        // `None` on any build without SD storage, which is what makes
        // `Status::sd_card_present` report "not supported" rather than "no card".
        sd_card_present: Option<&'a core::sync::atomic::AtomicBool>,
        // `None` on every build today -- no single-boiler machine has shot-log storage.
        shot_log_query_sender: Option<
            Sender<'a, ChannelM, crate::shot_log_query::ShotLogQuery, 1>,
        >,
        // Where `IdentifyMachine` goes. `None` on a machine with no display to flash.
        identify_publisher: Option<embassy_sync::watch::Sender<'a, ChannelM, Instant, 2>>,
        // Raised by the debug op that forgets the stored network. Not an `Option`: every
        // machine has a credential store.
        clear_wifi_credentials_signal: &'static embassy_sync::signal::Signal<
            embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
            (),
        >,
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
            machine_config,
            tank_config,
            group_config,
            boiler_config,
            routine_repository,
            current_routine: None,
            shot_logger: crate::shot_log::ShotLogger::new(),
            previous_routine_step: None,
            shot_log_sender,
            pending_annotations: variegated_controller_types::ShotAnnotations::new(),
            shot_log_query_sender,
            sd_card_present,
            previous_status: None,
            temperature_movavg: MovAvg::default(),
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
            bluetooth_store,
            bluetooth_associations: BluetoothAssociations::default(),
            bluetooth_associations_loaded: false,
            bluetooth_scan_sender,
            bluetooth_status: BluetoothScanStatus::default(),
            bluetooth_publish_pending: false,
            bluetooth_scan_deadline: None,
            wifi_store,
            wifi_credentials: StoredWifiCredentials::default(),
            wifi_credentials_loaded: false,
            wifi_publish_pending: false,
            wifi_provisioning_sender,
            identify_publisher,
            clear_wifi_credentials_signal,
            wifi_credentials_publisher,
        }
    }

    /// The published view of the configuration.
    ///
    /// Exists so the association list is folded in at every publish site rather than at
    /// the two that happened to be written first -- the browser reads its list out of
    /// `Configuration`, and a publish that omitted it would blank the Bluetooth page.
    fn general_configuration(&self, current: SingleBoilerSingleGroupConfiguration) -> Configuration {
        let mut configuration: Configuration = current.into();
        configuration.bluetooth_peripherals = self.bluetooth_associations.0.clone();
        configuration
    }

    /// Persist the association list and arrange for the comms processor to hear about it.
    ///
    /// See the equivalent in `dual_boiler_single_group`: the change is invisible to the
    /// machine-configuration comparison, so it has to announce itself.
    async fn save_bluetooth_associations(&mut self) {
        if self.bluetooth_store.save_settings(&self.bluetooth_associations).await.is_err() {
            log_warn!("Failed to save Bluetooth associations");
        }
        self.bluetooth_publish_pending = true;
    }

    /// Persist the Wi-Fi credentials and arrange for the comms processor to hear about it.
    ///
    /// Unlike the association list this does **not** ride on the `Configuration` publish:
    /// that path ends at the browser, and a password has no business on it.
    async fn save_wifi_credentials(&mut self) {
        if self.wifi_store.save_settings(&self.wifi_credentials).await.is_err() {
            log_warn!("Failed to save Wi-Fi credentials");
        }
        self.wifi_publish_pending = true;
    }

    /// Forget the stored network, persistently.
    ///
    /// Identical to the dual boiler's, and identical for a reason: this reproduces the state a
    /// machine is in before it has ever been provisioned, and that state is not
    /// machine-specific. Writing the cleared value is the point -- one that merely
    /// disconnected would come back knowing a network after the next reboot. Saving also sets
    /// `wifi_publish_pending`, so the comms processor is told and parks.
    async fn clear_wifi_credentials(&mut self) {
        if self.wifi_credentials.0.is_none() {
            log_info!("Wi-Fi credentials already cleared; nothing to forget");
            return;
        }
        // Never logs the SSID. Same rule as everywhere else on this path.
        self.wifi_credentials = StoredWifiCredentials(None);
        self.save_wifi_credentials().await;
        log_warn!("Wi-Fi credentials cleared; this machine is now unprovisioned");
    }

    pub async fn task(&mut self) {
        let mut last_pid_update = Instant::now();
        let mut last_configuration = self.current_configuration();

        loop {
            self.persistent_configuration = self.configuration_store.load_settings().await.unwrap_or_default();

            // Loaded on the first pass rather than in `new`, which is not async. Guarded
            // by a flag rather than reloaded each tick: this is the only reader and
            // writer of the list, so a second load could only return what is in hand.
            if !self.bluetooth_associations_loaded {
                self.bluetooth_associations_loaded = true;
                self.bluetooth_associations =
                    self.bluetooth_store.load_settings().await.unwrap_or_default();
                log_info!("Loaded {} Bluetooth associations", self.bluetooth_associations.0.len());
                // The comms processor asks at boot, but cannot tell a slow answer from no
                // answer, so publish once regardless.
                self.bluetooth_publish_pending = true;
            }

            // Same lazy load, same reasoning, for the credentials.
            if !self.wifi_credentials_loaded {
                self.wifi_credentials_loaded = true;
                self.wifi_credentials = self.wifi_store.load_settings().await.unwrap_or_default();
                // Logged as configured-or-not, never as a value. The SSID alone would be
                // harmless, but a log line that prints half a credential is one edit away
                // from printing all of it.
                log_info!(
                    "Wi-Fi credentials: {}",
                    if self.wifi_credentials.0.is_some() { "configured" } else { "none stored" }
                );
                self.wifi_publish_pending = true;
            }

            // Credentials go out on their own channel, never inside `Configuration` --
            // that path ends at the browser. Checked here rather than folded into the
            // configuration comparison below for the same reason.
            if self.wifi_publish_pending {
                self.wifi_publish_pending = false;
                if let Some(publisher) = self.wifi_credentials_publisher.as_ref() {
                    publisher.send(self.wifi_credentials.clone());
                }
            }

            // A scan whose end was never reported -- a comms reset, or a link that dropped
            // mid-scan. Left alone it would latch `scanning` true and keep the UI's scan
            // button disabled until the next reboot.
            if let Some(deadline) = self.bluetooth_scan_deadline {
                if Instant::now() >= deadline {
                    log_warn!("Bluetooth scan timed out without a result from the comms processor");
                    self.bluetooth_scan_deadline = None;
                    self.bluetooth_status.scanning = false;
                }
            }

            // Check if configuration changed and publish if it did
            let current_config = self.current_configuration();
            if current_config != last_configuration || self.bluetooth_publish_pending {
                self.bluetooth_publish_pending = false;
                let config = self.general_configuration(current_config.clone());
                self.configuration_channel_sender.publish_immediate(config);
                last_configuration = current_config;
            }

            // The debug op that forgets the stored network. Polled here rather than handled in
            // `handle_command`, because it is deliberately not a `MachineCommand` -- see the
            // note on the field. `try_take` so this loop keeps running the boiler.
            if self.clear_wifi_credentials_signal.try_take().is_some() {
                self.clear_wifi_credentials().await;
            }

            while !self.command_channel_receiver.is_empty() {
                let command = self.command_channel_receiver.try_receive();
                if let Ok(command) = command {
                    self.handle_command(command).await;

                    // Check if configuration changed after handling command
                    let current_config = self.current_configuration();
                    if current_config != last_configuration || self.bluetooth_publish_pending {
                        self.bluetooth_publish_pending = false;
                        let config = self.general_configuration(current_config.clone());
                        self.configuration_channel_sender.publish_immediate(config);
                        last_configuration = current_config;
                    }
                }
            }

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

            let (actual_boiler_control_target, actual_pump_control_target) = self.get_control_targets();

            let next_pid_update = Instant::now();
            let delta_t = (next_pid_update - last_pid_update).as_millis() as f32;
            last_pid_update = next_pid_update;

            let boiler_pid_out = self.update_boiler(actual_boiler_control_target, delta_t).await;
            let pump_pid_out = self.update_pump(actual_pump_control_target, delta_t).await;

            self.send_status(boiler_pid_out, pump_pid_out).await;

            // Feed the watchdog to prevent system reset
            if let Some(ref mut watchdog) = self.watchdog {
                watchdog.feed(crate::WATCHDOG_TIMEOUT);
            }

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
                Output::FixedDutyCycle(duty_cycle)
            }
            GroupBrewControlMode::FixedDutyCycleCurve => {
                let target_duty_cycle = actual_pump_control_state.values.duty_cycle_curve.evaluate(elapsed_seconds).clamp(0.0, 100.0) as u8;
                self.group.set_brewing_state(true, target_duty_cycle).await;
                Output::FixedDutyCycle(target_duty_cycle)
            }
            _ => {
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
                // Dry-run protection: disable heating if water level too low
                let boiler_level = self.boiler.get_water_level();
                let duty_cycle = if !Self::is_boiler_level_safe(boiler_level, &self.boiler_config) {
                    log_warn!("Boiler heating disabled: water level below minimum safe level");
                    0
                } else {
                    boiler_pid_out.out as u8
                };

                self.boiler.set_heating_element_duty_cycle(duty_cycle).await;

                if duty_cycle == 0 && boiler_pid_out.out > 0.0 {
                    // Level check blocked heating
                    Output::PidOutput(PidOut { out: 0.0, ..boiler_pid_out })
                } else {
                    Output::PidOutput(boiler_pid_out)
                }
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
            // Calculate output_volume from output_weight (assuming density ~1 g/ml)
            // Shot state tracking not implemented for single boiler, so no input-volume-based fallback
            let output_volume = self.group.get_output_weight().map(|w| w as OutputVolumeType);
            BrewStatus {
                brew_time: start.elapsed().into(),
                brew_input_volume,
                shot_state: None, // Not yet implemented for single boiler controller
                extracted_solids: self.accumulated_extracted_solids,
                output_volume,
            }
        });

        let group_status = GroupStatus {
            is_brewing: self.state == SingleBoilerSingleGroupControllerState::Brewing,
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
            control_state: self.ephemeral_configuration.group_brew_control_state,
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
                // Carried through unextrapolated, unlike the timestamp above -- see the
                // note on the dual-boiler controller's copy of this.
                improv: status.improv,
                peripheral_connection_status: FnvIndexMap::default(),
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
            steam_wand_statuses: FnvIndexMap::new(),
            tank_statuses,
            mode: Default::default(),
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

    async fn handle_command(&mut self, command: MachineCommand) {
        match command {
            MachineCommand::RunRoutine(index, params) => {
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
                // Validate tank status before starting brewing
                if self.should_block_water_operation() {
                    variegated_log::emit_event(DebugEvent::InterlockTripped { interlock: name("start_brewing_water_tank_low") });
                    return;
                }
                self.transition_to_state(SingleBoilerSingleGroupControllerState::Brewing).await;
            }
            MachineCommand::StopBrewing(_) => {
                self.transition_to_state(SingleBoilerSingleGroupControllerState::BrewModeIdle).await;
            }
            MachineCommand::StartPumpingToWaterTap(_) => {
                // Validate tank status before starting water dispensing
                if self.should_block_water_operation() {
                    variegated_log::emit_event(DebugEvent::InterlockTripped { interlock: name("water_tap_water_tank_low") });
                    return;
                }
                self.transition_to_state(SingleBoilerSingleGroupControllerState::PumpingToWaterTap).await;
            }
            MachineCommand::StopPumpingToWaterTap(_) => {
                self.transition_to_state(SingleBoilerSingleGroupControllerState::BrewModeIdle).await;
            }
            MachineCommand::SetBoilerControlTarget(boiler_index, mode, values_update) => {
                log_info!("Setting boiler control mode for boiler {} to {:?} with values {:?}", boiler_index, mode, values_update);
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
                        log_error!("Invalid boiler index: {}", boiler_index);
                    }
                }
            }
            MachineCommand::SetBoilerControlTargetValues(boiler_index, update) => {
                log_info!("Setting boiler control values for boiler {} to {:?}", boiler_index, update);
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
                    log_error!("Invalid group index: {}", group_index);
                }
            }
            MachineCommand::SetGroupBrewControlTargetValues(group_index, update) => {
                log_info!("Setting group brew control values for group {} to {:?}", group_index, update);
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
                    log_error!("Invalid group index: {}", group_index);
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
                    log_info!("Enabling brew boiler");
                    self.transition_to_state(SingleBoilerSingleGroupControllerState::BrewModeIdle).await;
                } else if boiler_index == 1 && self.state == SingleBoilerSingleGroupControllerState::BrewModeIdle {
                    log_info!("Enabling steam boiler");
                    self.transition_to_state(SingleBoilerSingleGroupControllerState::SteamModeIdle).await;
                } else {
                    log_warn!("Invalid boiler index or state for enabling boiler: {} Current state: {:?}", boiler_index, self.state);
                }
            }
            MachineCommand::DisableBoiler(boiler_index) => {
                if boiler_index == 1 && self.state == SingleBoilerSingleGroupControllerState::SteamModeIdle {
                    log_info!("Going back to brew mode");
                    self.transition_to_state(SingleBoilerSingleGroupControllerState::BrewModeIdle).await;
                } else if boiler_index == 0 && self.state == SingleBoilerSingleGroupControllerState::BrewModeIdle {
                    log_info!("Going in to power save mode");
                    self.transition_to_state(SingleBoilerSingleGroupControllerState::PowerSave).await;
                } else {
                    log_warn!("Invalid boiler index or state for disabling boiler: {} Current state: {:?}", boiler_index, self.state);
                }
            },
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
            MachineCommand::OptimizeConfigurationStorage => {
                log_info!("Optimizing configuration storage");
                if let Err(e) = self.configuration_store.optimize_storage().await {
                    log_warn!("Failed to optimize configuration storage: {}", e);
                }
            }
            MachineCommand::OptimizeRoutineStorage => {
                log_warn!("OptimizeRoutineStorage not supported for single boiler controller (no routine repository)");
            }
            MachineCommand::OptimizeScheduleStorage => {
                log_warn!("OptimizeScheduleStorage not supported for single boiler controller (no schedule store)");
            }
            MachineCommand::InferGroupPressureIntegral(group_index, target_pressure) => {
                if group_index == 0 {
                    log_info!("Inferring group pressure integral for target pressure: {} bar", target_pressure);

                    // Get current duty cycle from the control state configuration
                    let current_duty_cycle = self.ephemeral_configuration.group_brew_control_state.values.duty_cycle;
                    let current_pressure = self.group.get_pressure().unwrap_or(0.0);

                    // Set up PID for pressure control
                    self.pump_pid.setpoint = target_pressure as f32;
                    self.pump_pid.set_parameters(self.persistent_configuration.pid_parameters.pump_pressure_params);

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
                    let current_duty_cycle = self.ephemeral_configuration.group_brew_control_state.values.duty_cycle;
                    let current_flow_rate = self.group.get_input_flow_rate().unwrap_or(0.0);

                    // Set up PID for flow rate control
                    self.pump_pid.setpoint = target_flow_rate as f32;
                    self.pump_pid.set_parameters(self.persistent_configuration.pid_parameters.pump_flow_rate_params);

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
                    let current_duty_cycle = self.ephemeral_configuration.group_brew_control_state.values.duty_cycle;
                    let current_output_flow_rate = self.group.get_output_flow_rate().unwrap_or(0.0);

                    // Set up PID for output flow rate control
                    self.pump_pid.setpoint = target_output_flow_rate as f32;
                    self.pump_pid.set_parameters(self.persistent_configuration.pid_parameters.pump_output_flow_rate_params);

                    // Infer and set the integral
                    self.pump_pid.infer_and_set_integral(current_duty_cycle as f32, current_output_flow_rate as f32);

                    log_info!("Set output flow rate integral based on duty cycle {} and output flow rate {}", current_duty_cycle, current_output_flow_rate);
                } else {
                    log_error!("Invalid group index: {}", group_index);
                }
            }
            MachineCommand::AssociateBluetoothPeripheral(association) => {
                let id = association.id;
                if self.bluetooth_associations.upsert(association) {
                    log_info!("Associated Bluetooth peripheral 0x{:04X}", id);
                    self.save_bluetooth_associations().await;
                } else {
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
                // A discovery scan monopolises a radio shared with Wi-Fi and with the live
                // links to the peripherals themselves, and the ACAIA driver drops its
                // connection if it misses a couple of heartbeats. Only this processor
                // knows whether coffee is being made, so only it can refuse.
                //
                // `SteamModeIdle` is not busy -- despite the boiler being hot, nothing is
                // flowing and no shot is at stake.
                let busy = self.current_routine.is_some()
                    || matches!(
                        self.state,
                        SingleBoilerSingleGroupControllerState::Brewing
                            | SingleBoilerSingleGroupControllerState::PumpingToWaterTap
                    );

                if busy {
                    log_warn!("Refusing Bluetooth scan: machine is busy");
                    self.bluetooth_status.blocked = true;
                } else if let Some(sender) = self.bluetooth_scan_sender {
                    match sender.try_send(crate::BLUETOOTH_SCAN_DURATION_MS) {
                        Ok(()) => {
                            log_info!("Starting Bluetooth scan");
                            self.bluetooth_status.blocked = false;
                            self.bluetooth_status.scanning = true;
                            self.bluetooth_status.reports_dropped = 0;
                            // Cleared on start, not on finish: the user is about to pick
                            // from this list, and last scan's devices may be gone.
                            self.bluetooth_status.discovered.clear();
                            self.bluetooth_scan_deadline = Some(
                                Instant::now()
                                    + embassy_time::Duration::from_millis(
                                        crate::BLUETOOTH_SCAN_DURATION_MS as u64
                                            + crate::BLUETOOTH_SCAN_SLACK_MS,
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
                    // Merged, not replaced -- a device's name and its service UUIDs
                    // usually arrive in different advertising reports, and overwriting
                    // would keep whichever came last. See the equivalent in
                    // `dual_boiler_single_group`.
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
                        }
                        None => {
                            if self.bluetooth_status.discovered.push(device).is_err() {
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
                // sends *after* its radio has associated using them.
                let stored = StoredWifiCredentials(Some(credentials));
                if self.wifi_credentials != stored {
                    self.wifi_credentials = stored;
                    self.save_wifi_credentials().await;
                    log_info!("Stored new Wi-Fi credentials");
                }
            }
            MachineCommand::OpenWifiProvisioningWindow { duration_ms } => {
                // The same busy predicate a discovery scan uses, for the same reason:
                // minutes of connectable advertising share one antenna with Wi-Fi and the
                // live links to the scales, and only this processor knows a shot is in
                // progress.
                let busy = self.current_routine.is_some()
                    || matches!(
                        self.state,
                        SingleBoilerSingleGroupControllerState::Brewing
                            | SingleBoilerSingleGroupControllerState::PumpingToWaterTap
                    );

                if busy {
                    log_warn!("Refusing to open the Wi-Fi provisioning window: machine is busy");
                } else if let Some(sender) = self.wifi_provisioning_sender {
                    if sender.try_send(duration_ms).is_err() {
                        log_warn!("Failed to forward the provisioning window request: channel full");
                    }
                } else {
                    log_warn!("This machine has no Wi-Fi provisioning path");
                }
            }
            MachineCommand::CloseWifiProvisioningWindow => {
                if let Some(sender) = self.wifi_provisioning_sender {
                    // Zero means close -- one channel for both, so a close cannot overtake
                    // the open it was meant to cancel.
                    let _ = sender.try_send(0);
                }
            }
            MachineCommand::IdentifyMachine => {
                // Logged as well as published: this is the far end of a round trip that starts
                // in a browser, and the log is the only place both ends are visible at once.
                log_info!("Identify requested");
                if let Some(publisher) = self.identify_publisher.as_ref() {
                    publisher.send(Instant::now());
                }
            }
            MachineCommand::SetShotAnnotations(id, annotations) => {
                // Identical to the dual-boiler arm, and identical for a reason: the
                // sender is `None` on every build today, so this always refuses -- but a
                // single-boiler machine that gained a card reader should not also need
                // this command re-implemented. See that arm for why `try_send`.
                match self.shot_log_query_sender {
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
            _ => {}
        }
    }

    async fn transition_to_state(&mut self, new_state: SingleBoilerSingleGroupControllerState) {
        if self.state != new_state {
            log_info!("Transitioning from {:?} to {:?}", self.state, new_state);
            let old_state = self.state;
            self.state = new_state;

            match (old_state, new_state) {
                (SingleBoilerSingleGroupControllerState::BrewModeIdle, SingleBoilerSingleGroupControllerState::Brewing) => {
                    variegated_log::emit_event(DebugEvent::BrewStarted { group: SingleGroup.as_index() });
                    self.group.set_brewing_state(true, 0).await;
                    self.started_brewing().await;
                }
                (SingleBoilerSingleGroupControllerState::Brewing, SingleBoilerSingleGroupControllerState::BrewModeIdle) => {
                    variegated_log::emit_event(DebugEvent::BrewStopped { group: SingleGroup.as_index() });
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
        self.accumulated_extracted_solids = Some(0.0);
        self.last_extraction_time = Some(Instant::now());
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
        self.accumulated_extracted_solids = None;
        self.last_extraction_time = None;
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
                self.state,
                self.current_configuration(),
                runtime_params.clone()
            );

            // Start shot logging
            use variegated_controller_types::{ShotLogMetadata, ShotType, ShotStatus, RoutineExecutionMetadata};
            let metadata = ShotLogMetadata {
                // Copied, not moved, and the user's annotations only -- see the
                // equivalent block in `dual_boiler_single_group`.
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

    /// Record what a scale currently reads as the dose for the next shot.
    ///
    /// See the equivalent method in `dual_boiler_single_group` for why this refuses
    /// rather than substituting a zero, and why it does not tare.
    fn tag_dose_from_scale(&mut self, scale: variegated_controller_types::ScaleSelector) {
        use variegated_controller_types::{
            ScaleSelector, ShotAnnotationKey, ShotAnnotationValue,
        };

        let weight = match scale {
            ScaleSelector::GroupScale(index) if index == SingleGroup.as_index() => {
                self.group.get_output_weight()
            }
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
            Err(_) => log_warn!(
                "TagDoseFromScale: the annotation block is full ({} entries)",
                self.pending_annotations.len()
            ),
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
            supply_tank_index: None,
            minimum_safe_level: None, // Not stored in persistent config
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
            supply_tank_index: None,
            minimum_safe_level: None, // Virtual steam boiler shares the same physical boiler
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
            supply_tank_index: None,
        };
        configuration.insert_group_configuration(SingleGroup.as_index(), group_config);

        // Add tank configuration
        let tank_config = TankConfiguration {
            low_level_warning_threshold: Some(20),
            water_level_sensor_kalman_parameters: None,
            empty_threshold: None,
        };
        configuration.insert_tank_configuration(0, tank_config);

        configuration
    }
}