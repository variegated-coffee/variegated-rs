extern crate alloc;

use variegated_log::{log_debug, log_error, log_info, log_warn};
use variegated_controller_types::debug::{name, CheckinDetail, CheckinStatus, DebugEvent};
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_sync::mutex::Mutex;
use embassy_sync::pubsub::Publisher;
use embassy_rp::watchdog::Watchdog;
// No `with_timeout` here any more: every timed store access this controller made now lives
// in `crate::command::stores`, which owns the lock and the timeout together.
use embassy_time::{Instant, Timer};
use heapless::index_map::FnvIndexMap;
use movavg::MovAvg;
use variegated_control_algorithm::pid::{PidCtrl, PidIn, PidOut};
use variegated_hal::{Boiler, Group, Tank, PeripheralRegistry};
use variegated_controller_types::{BoilerConfiguration, BoilerControlMode, BoilerControlState, BoilerIndex, BoilerStatus, BrewStatus, CommsStatus, Configuration, DutyCycleType, HexadecimalDutyCycleType, InputVolumeType, GroupBrewControlMode, GroupBrewControlState, GroupBrewLimitMode, BrewLimitStatus, GroupStatus, MachineCommand, MachineConfiguration, MachineMode, Output, PumpOutput, RoutineExecutionStatus, RoutineIndex, SingleBoilerSingleGroupControllerState, Status, MachineDefinition, TankConfiguration, TankStatus, WaterLevelType, RoutineParameters, OutputVolumeType, StorageCommand};
use crate::command;
use crate::routine::{RoutineExecutionContext, RoutineRepository};
use variegated_controller_types::SingleBoilerSingleGroupControllerBoilers::{BrewBoiler, VirtualSteamBoiler};
use variegated_controller_types::SingleGroupControllerGroups::SingleGroup;
use variegated_hal::scale::ScaleConfiguration;
use variegated_timekeeping::TimeKeeper;
use crate::settings::SettingsStorage;
use crate::pump_transfer::{PumpPidEngagement, PumpPidTransfer};
use crate::pump_limit::{self, LimitEngagement, LimitTransfer};
use variegated_controller_types::bluetooth::{
    BluetoothAssociations, BluetoothScanStatus,
};
use variegated_controller_types::shot_upload::ShotUploadConfig;
use variegated_controller_types::timezone::TimezoneSetting;
use variegated_controller_types::wifi::StoredWifiCredentials;

// The configuration types, their stored representation and their projection onto
// `Configuration` live in `crate::single_boiler_config`, which is ungated where this module
// is not. Re-exported rather than moved-and-repointed so that every caller -- the firmware,
// the menu, the debug bridge -- keeps naming them here. See that module's docs for why the
// split exists.
pub use crate::single_boiler_config::{
    SingleBoilerSingleGroupConfiguration, SingleBoilerSingleGroupEphemeralConfiguration,
    SingleBoilerSingleGroupPersistentConfiguration, SingleBoilerSingleGroupPidParameters,
};

pub struct SingleBoilerSingleGroupController<
    'a,
    ChannelM: RawMutex,
    M: RawMutex,
    StorageM: RawMutex + 'static,
    SettingsStoreT: SettingsStorage<SingleBoilerSingleGroupPersistentConfiguration>,
    RoutineRepoT: RoutineRepository + 'static,
    BluetoothStoreT: SettingsStorage<BluetoothAssociations>,
    WifiStoreT: SettingsStorage<StoredWifiCredentials>,
    UploadStoreT: SettingsStorage<ShotUploadConfig>,
    TimezoneStoreT: SettingsStorage<TimezoneSetting>,
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
    /// Whether `pump_pid` currently owns the pump, and what the pump is running at. See
    /// [`crate::pump_transfer`] — without it the PID wound up while open-loop control held
    /// the output, and took over from a standing start when it got it back.
    pump_pid_engagement: PumpPidEngagement,
    /// The limit loop, when one is armed. A second controller against the same actuator,
    /// selected against `pump_pid` by taking the lower output — see [`crate::pump_limit`].
    limit_pid: PidCtrl<f32>,
    limit_engagement: LimitEngagement,
    /// What the last `update_pump` decided about the limit, for the status publisher — which
    /// runs on its own cadence and cannot recompute it. See [`GroupStatus::brew_limit`].
    last_brew_limit: Option<BrewLimitStatus>,
    configuration_store: SettingsStoreT,
    /// The machine's configuration, persistent and ephemeral halves together.
    ///
    /// One field rather than two, matching the dual-boiler. They were split here, and
    /// `current_configuration` rebuilt the pair into a `SingleBoilerSingleGroupConfiguration`
    /// -- cloning both halves -- every time the publish path wanted to compare against the
    /// last one. Keeping the whole value means the command handlers in [`crate::command`]
    /// can take it through one [`crate::command::ConfigurationAccess`] impl, which is what
    /// lets the two machines share those handlers at all.
    configuration: SingleBoilerSingleGroupConfiguration,
    machine_config: MachineConfiguration,
    tank_config: TankConfiguration,
    // No `group_config`. It was accepted as a constructor argument, stored, and never read:
    // the group configuration this controller publishes is built from
    // `config.persistent.pid_parameters` and `config.ephemeral` further down. So the
    // argument was ignored, and the field was a copy of something that never applied. The
    // parameter is gone too -- the Silvia firmware passed `GroupConfiguration::default()`,
    // so nothing observable changes.
    boiler_config: BoilerConfiguration,
    routine_repository: &'static Mutex<StorageM, RoutineRepoT>,
    /// Where an optimization request goes, rather than being run here.
    ///
    /// See the `OptimizeRoutineStorage` arm: on a flash-backed repository that call erases
    /// and rewrites the whole range, and this loop is the one holding the boiler.
    storage_command_sender: Sender<'a, ChannelM, StorageCommand, 4>,
    current_routine: Option<RoutineExecutionContext<SingleBoilerSingleGroupControllerState, SingleBoilerSingleGroupConfiguration>>,
    shot_logger: crate::shot_log::ShotLogger,
    previous_routine_step: Option<usize>,
    shot_log_sender: Option<Sender<'a, ChannelM, variegated_controller_types::ShotLog, 2>>,
    /// Whether the currently open shot log was opened by `started_brewing` rather than by a
    /// routine. Only that kind is closed by `stopped_brewing`; see `finish_manual_shot_log`.
    manual_shot_active: bool,
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
    /// Which phase of the shot the machine is in. Driven by `update_shot_state` below.
    ///
    /// This machine has no conductivity probe, so the first drop can only be seen by weight
    /// and only once the puck has saturated; without a scale paired the state legitimately
    /// stops at `Saturation`. See `crate::shot_state` for why a missing signal removes a
    /// transition rather than defaulting to a number.
    shot_state: crate::ShotStateTracker,
    comms_status: Option<CommsStatus>,
    comms_status_received_instant: Option<Instant>,
    peripheral_registry: &'a PeripheralRegistry<'a>,
    /// What this machine declares it can sense -- the capability half of a prerequisite
    /// check, which `peripheral_registry` cannot answer. See the dual-boiler twin.
    machine_definition: &'a MachineDefinition,
    /// Since when a running routine's prerequisites have been unmet, if they are.
    prerequisite_lost_since: Option<Instant>,
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
    configuration_publish_pending: bool,
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

    // Where finished shot logs are uploaded, at its own key in the settings flash range.
    // Same shape and same reasoning as the Wi-Fi trio above: its own key rather than a
    // field on the persistent configuration, and its own publish rather than riding
    // `Configuration`, because the token is a secret and that path ends at the browser.
    shot_upload_store: UploadStoreT,
    shot_upload_config: ShotUploadConfig,
    timezone_store: TimezoneStoreT,
    /// The machine's timezone, as stored. Applied to the `TimeKeeper` at boot and on change.
    ///
    /// This board has no scheduler, so the zone reaches only `Status::current_local_time` and
    /// the browser -- but it is the same setting on the same key, and a machine whose clock
    /// reads UTC while the dual-boiler's reads local would be a difference with no reason
    /// behind it.
    timezone: TimezoneSetting,
    /// Whether the lazy load below has run. An *empty* zone is a legitimate answer (UTC), so
    /// this cannot be inferred from the value.
    timezone_loaded: bool,
    shot_upload_config_loaded: bool,
    shot_upload_publish_pending: bool,
    shot_upload_config_publisher: Option<embassy_sync::watch::Sender<'a, ChannelM, ShotUploadConfig, 2>>,

    /// Where this loop reports its own health. See `with_checkin`.
    checkin: variegated_checkin::CheckinHandle,
}

impl<
    'a,
    ChannelM: RawMutex,
    M: RawMutex,
    StorageM: RawMutex + 'static,
    SettingsStoreT: SettingsStorage<SingleBoilerSingleGroupPersistentConfiguration>,
    RoutineRepoT: RoutineRepository + 'static,
    BluetoothStoreT: SettingsStorage<BluetoothAssociations>,
    WifiStoreT: SettingsStorage<StoredWifiCredentials>,
    UploadStoreT: SettingsStorage<ShotUploadConfig>,
    TimezoneStoreT: SettingsStorage<TimezoneSetting>,
    const N_CHANNEL: usize,
    const N_WATCH: usize,
    const N_SUBS: usize,
    const N_CONFIG_SUBS: usize
> SingleBoilerSingleGroupController<'a, ChannelM, M, StorageM, SettingsStoreT, RoutineRepoT, BluetoothStoreT, WifiStoreT, UploadStoreT, TimezoneStoreT, N_CHANNEL, N_WATCH, N_SUBS, N_CONFIG_SUBS> {
    fn current_configuration(&self) -> SingleBoilerSingleGroupConfiguration {
        self.configuration.clone()
    }
    pub fn new(
        command_channel_receiver: Receiver<'a, ChannelM, MachineCommand, N_CHANNEL>,
        status_channel_sender: Publisher<'a, ChannelM, Status, 1, N_SUBS, 1>,
        configuration_channel_sender: Publisher<'a, ChannelM, Configuration, 1, N_CONFIG_SUBS, 1>,
        boiler: Boiler<'a, M, N_WATCH>,
        group: Group<'a, M, N_WATCH>,
        tank: Option<Tank<'a, M, N_WATCH>>,
        settings_store: SettingsStoreT,
        machine_config: MachineConfiguration,
        tank_config: TankConfiguration,
        boiler_config: BoilerConfiguration,
        routine_repository: &'static Mutex<StorageM, RoutineRepoT>,
        storage_command_sender: Sender<'a, ChannelM, StorageCommand, 4>,
        peripheral_registry: &'a PeripheralRegistry<'a>,
        machine_definition: &'a MachineDefinition,
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
        shot_upload_store: UploadStoreT,
        timezone_store: TimezoneStoreT,
        // Where the shot-log upload config goes for the transceiver to put on the link.
        // `None` on a machine with no comms processor -- which is also a machine that
        // cannot upload anything, so the config is stored and simply never acted on.
        shot_upload_config_publisher: Option<embassy_sync::watch::Sender<'a, ChannelM, ShotUploadConfig, 2>>,
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
            // 0-255, not 0-100: the pump PID computes on the pump's own scale.
            pump_pid: super::hexadecimal_limited_pid(),
            pump_pid_engagement: PumpPidEngagement::new(),
            // Same clamps as the main loop: it drives the same actuator on the same scale,
            // and is only ever holding a different quantity.
            limit_pid: super::hexadecimal_limited_pid(),
            limit_engagement: LimitEngagement::new(),
            last_brew_limit: None,
            configuration_store: settings_store,
            configuration: SingleBoilerSingleGroupConfiguration::default(),
            machine_config,
            tank_config,
            boiler_config,
            routine_repository,
            storage_command_sender,
            current_routine: None,
            shot_logger: crate::shot_log::ShotLogger::new(),
            previous_routine_step: None,
            shot_log_sender,
            manual_shot_active: false,
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
            shot_state: crate::ShotStateTracker::new(),
            comms_status: None,
            comms_status_received_instant: None,
            peripheral_registry,
            machine_definition,
            prerequisite_lost_since: None,
            watchdog,
            bluetooth_store,
            bluetooth_associations: BluetoothAssociations::default(),
            bluetooth_associations_loaded: false,
            bluetooth_scan_sender,
            bluetooth_status: BluetoothScanStatus::default(),
            configuration_publish_pending: false,
            bluetooth_scan_deadline: None,
            wifi_store,
            wifi_credentials: StoredWifiCredentials::default(),
            wifi_credentials_loaded: false,
            wifi_publish_pending: false,
            wifi_provisioning_sender,
            identify_publisher,
            clear_wifi_credentials_signal,
            wifi_credentials_publisher,
            shot_upload_store,
            timezone_store,
            timezone: TimezoneSetting::default(),
            timezone_loaded: false,
            shot_upload_config: ShotUploadConfig::default(),
            shot_upload_config_loaded: false,
            shot_upload_publish_pending: false,
            shot_upload_config_publisher,
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

    /// The published view of the configuration.
    ///
    /// Exists so the association list is folded in at every publish site rather than at
    /// the two that happened to be written first -- the browser reads its list out of
    /// `Configuration`, and a publish that omitted it would blank the Bluetooth page.
    fn general_configuration(&self, current: SingleBoilerSingleGroupConfiguration) -> Configuration {
        let mut configuration: Configuration = current.into();
        configuration.bluetooth_peripherals = self.bluetooth_associations.0.clone();
        // The `From` is where the token gets dropped -- it reduces to `token_set: bool`, so
        // no path from here to the browser carries the secret.
        configuration.shot_upload = (&self.shot_upload_config).into();
        configuration.timezone = self.timezone.clone();
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
        self.configuration_publish_pending = true;
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

    /// Persist the shot-log upload config and arrange for the comms processor to hear
    /// about it. Same reasoning as `save_wifi_credentials` directly above, and for a
    /// sharper reason: the token grants write access to an account on a public service.
    async fn save_shot_upload_config(&mut self) {
        if self.shot_upload_store.save_settings(&self.shot_upload_config).await.is_err() {
            log_warn!("Failed to save shot upload config");
        }
        // Two flags, two destinations. This one sends the full config -- token included -- to
        // the comms processor on its own watch.
        self.shot_upload_publish_pending = true;
        // And this one republishes `Configuration`, which carries the redacted
        // `ShotUploadView` the browser reads. Without it the settings panel showed a stale
        // endpoint for up to ten seconds after an edit.
        self.configuration_publish_pending = true;
    }

    /// Persist the timezone, and republish the configuration that carries it.
    ///
    /// No watch of its own: the comms processor keeps time in UTC and has no use for the zone,
    /// and the browser reads it from `Configuration`.
    async fn save_timezone(&mut self) {
        if self.timezone_store.save_settings(&self.timezone).await.is_err() {
            log_warn!("Failed to save timezone");
        }
        self.configuration_publish_pending = true;
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
        // `Instant::MIN` is tick zero, so this reads as "last published at boot" and the
        // first republish falls due ten seconds after boot rather than ten seconds after
        // this loop starts. The two are not the same on this board: `task()` runs after
        // the settings flash load, the ADC bring-up and the display reset, so if that
        // preamble ever takes more than ten seconds the republish is due on the first
        // tick instead of a further ten seconds later. Using `Instant::now()` here would
        // silently push the first retry out by however long boot happened to take.
        let mut last_configuration_publish = Instant::MIN;

        loop {
            // Recomputed from scratch each pass, so a condition that clears is reported as
            // cleared on the next tick rather than latching.
            let mut health = CheckinStatus::Good;

            self.configuration.persistent = match self.configuration_store.load_settings().await {
                Ok(settings) => settings,
                Err(_) => {
                    // Substituting a `Default` is this loop continuing to run on a
                    // configuration the operator did not choose. Survivable -- hence a
                    // warning -- but it is the difference between a machine that is set up
                    // and one that looks set up, and until now it said so nowhere at all:
                    // the `unwrap_or_default` this replaces discarded the error silently.
                    health = CheckinStatus::Warning(CheckinDetail::Degraded);
                    Default::default()
                }
            };

            // Repairs the stored `Off` that made the steam switch change the mode and then
            // stop the heating. Applied to the loaded value rather than written back: it is
            // idempotent, and a machine whose owner sets a steam temperature stores
            // `Temperature` and stops needing it.
            self.configuration.persistent.steam_boiler_control_state =
                crate::single_boiler_state::steam_boiler_state_or_default(
                    self.configuration.persistent.steam_boiler_control_state,
                );

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
                self.configuration_publish_pending = true;
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

            // Same lazy load again, for the upload config.
            if !self.shot_upload_config_loaded {
                self.shot_upload_config_loaded = true;
                self.shot_upload_config =
                    self.shot_upload_store.load_settings().await.unwrap_or_default();
                // The endpoint is not a secret and is the field you need when uploads go
                // somewhere unexpected; the token is reported only as present-or-not.
                log_info!(
                    "Shot upload: endpoint {}, token {}",
                    if self.shot_upload_config.endpoint.is_some() { "configured" } else { "none stored" },
                    if self.shot_upload_config.token.is_some() { "configured" } else { "none stored" }
                );
                self.shot_upload_publish_pending = true;
                // Explicitly, rather than relying on the Bluetooth block above having already
                // set it this iteration. That happens to be true today and is not a property
                // either block states; reordering or removing that one would leave the stored
                // upload settings unpublished until something else dirtied the configuration.
                self.configuration_publish_pending = true;
            }

            // Same lazy load, same reasoning, for the timezone. Into RAM only: the
            // `TimeKeeper` was already told in `main`, before any task was spawned.
            if !self.timezone_loaded {
                self.timezone_loaded = true;
                self.timezone = self.timezone_store.load_settings().await.unwrap_or_default();
                self.configuration_publish_pending = true;
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

            // Separately again, and for the same reason: the upload token is a secret.
            if self.shot_upload_publish_pending {
                self.shot_upload_publish_pending = false;
                if let Some(publisher) = self.shot_upload_config_publisher.as_ref() {
                    publisher.send(self.shot_upload_config.clone());
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
            if current_config != last_configuration || self.configuration_publish_pending {
                self.configuration_publish_pending = false;
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
                    if current_config != last_configuration || self.configuration_publish_pending {
                        self.configuration_publish_pending = false;
                        let config = self.general_configuration(current_config.clone());
                        self.configuration_channel_sender.publish_immediate(config);
                        last_configuration = current_config;
                    }
                }
            }

            // Record a shot log sample. Outside the routine block, so a manual brew is
            // logged too -- see the equivalent note in `dual_boiler_single_group`.
            if let Some(status) = self.previous_status.as_ref() {
                self.shot_logger.record_sample(status);
            }

            // A running routine whose prerequisites have gone away; debounced, and checked
            // before the step so it does not advance on stale readings. See the equivalent
            // block in `dual_boiler_single_group`.
            if self.current_routine.is_some() {
                let peripherals = self.peripheral_registry.get_peripheral_status();
                let missing = self.current_routine.as_ref().and_then(|routine| {
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
                            self.handle_routine_exit(true).await;
                            self.prerequisite_lost_since = None;
                        }
                    }
                }
            }

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

            let (actual_boiler_control_target, actual_pump_control_target) = self.get_control_targets();

            let next_pid_update = Instant::now();
            let delta_t = (next_pid_update - last_pid_update).as_millis() as f32;
            last_pid_update = next_pid_update;

            let boiler_pid_out = self.update_boiler(actual_boiler_control_target, delta_t).await;
            let pump_pid_out = self.update_pump(actual_pump_control_target, delta_t).await;

            self.update_shot_state();

            self.send_status(boiler_pid_out, pump_pid_out).await;

            // Republish the configuration every 10 seconds whether or not it changed,
            // matching `dual_boiler_single_group`.
            //
            // This is what makes the payload survivable, and it is worth being explicit
            // about why it is needed at all. `Configuration` is published through an
            // embassy-sync pub-sub channel, which retains nothing: `try_publish` succeeds
            // with no subscribers and simply drops the value, and `subscriber()` starts a
            // reader at the *current* message id, so a consumer created afterwards never
            // sees what came before it. On this board the one guaranteed publish is the
            // boot one -- forced by the Bluetooth lazy load a few milliseconds in -- and
            // the comms processor's HTTP cache does not subscribe until Wi-Fi association
            // and DHCP have finished, tens of seconds later. Publish-on-change alone
            // therefore left `/configuration` answering 503 for the entire life of a boot,
            // until the first setting was edited.
            //
            // Every other payload on the link already survives this: `MachineDefinition`
            // and `RoutineSummaries` are written into caches directly by their reader
            // arms, and `Status` is republished at 1 Hz. This closes the gap for the one
            // that was neither.
            //
            // `last_configuration` is advanced with it so the change comparison at the top
            // of the loop does not immediately publish a second copy.
            let now = Instant::now();
            if now.saturating_duration_since(last_configuration_publish).as_secs() >= 10 {
                let current_config = self.current_configuration();
                let config = self.general_configuration(current_config.clone());
                self.configuration_channel_sender.publish_immediate(config);
                last_configuration_publish = now;
                last_configuration = current_config;
            }

            // Feed the watchdog to prevent system reset
            if let Some(ref mut watchdog) = self.watchdog {
                watchdog.feed(crate::WATCHDOG_TIMEOUT);
            }

            // Beside the watchdog feed, and after it, so a pass that reached the feed is a
            // pass that reported. The check-in is the one of the two that can distinguish
            // *this* loop running from the executor running.
            self.checkin.record(health);

            Timer::after_millis(100).await;
        }
    }

    /// Step the limit loop, or `None` if no limit is running this iteration.
    ///
    /// `commanded` is what the pump is being driven at right now; an engaging loop inherits
    /// it, without which its first output is `kp * error` and wins the selector by accident.
    /// See [`crate::pump_limit::LimitTransfer`].
    ///
    /// The gains are the ones already tuned for controlling that quantity, because that is
    /// what this is: the same physical loop, holding a cap instead of a setpoint.
    fn step_limit_loop(
        &mut self,
        state: &GroupBrewControlState,
        commanded: f32,
        delta_t: f32,
    ) -> Option<PidOut<f32>> {
        let transfer = self.limit_engagement.transfer_for(state.mode, state.limit);
        if transfer == LimitTransfer::Hold {
            return None;
        }

        let setpoint = pump_limit::limit_setpoint(state.limit, &state.values)?;
        let (pv, params) = match state.limit {
            // `limit_setpoint` already returned `None` for this.
            GroupBrewLimitMode::Unlimited => return None,
            GroupBrewLimitMode::MaxPressure => (
                self.group.get_pressure().unwrap_or(0.0) as f32,
                self.configuration.persistent.pid_parameters.pump_pressure_params,
            ),
            GroupBrewLimitMode::MaxGroupFlowRate => (
                self.group.get_input_flow_rate().unwrap_or(0.0) as f32,
                self.configuration.persistent.pid_parameters.pump_flow_rate_params,
            ),
            // An absent scale reads as 0.0, permanently below any cap. Safe, but only
            // because of the seeding below and the tracking in `update_pump` -- see
            // `GroupBrewLimitMode::MaxOutputFlowRate`.
            GroupBrewLimitMode::MaxOutputFlowRate => (
                self.group.get_output_flow_rate().unwrap_or(0.0) as f32,
                self.configuration.persistent.pid_parameters.pump_output_flow_rate_params,
            ),
        };

        self.limit_pid.setpoint = setpoint;
        self.limit_pid.set_parameters(params);
        if transfer == LimitTransfer::Engage {
            self.limit_pid.infer_and_set_integral(commanded, pv);
        }

        Some(self.limit_pid.step(PidIn::new(pv, delta_t)))
    }

    async fn update_pump(&mut self, actual_pump_control_state: GroupBrewControlState, delta_t: f32) -> PumpOutput {
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
                self.pump_pid.set_parameters(self.configuration.persistent.pid_parameters.pump_flow_rate_params);

                self.group.get_input_flow_rate().unwrap_or(0.0) as f32
            },
            GroupBrewControlMode::GroupFlowRateCurve => {
                let target = actual_pump_control_state.values.flow_rate_curve.evaluate(elapsed_seconds);
                self.pump_pid.setpoint = target;
                self.pump_pid.set_parameters(self.configuration.persistent.pid_parameters.pump_flow_rate_params);

                self.group.get_input_flow_rate().unwrap_or(0.0) as f32
            },
            GroupBrewControlMode::Pressure => {
                self.pump_pid.setpoint = actual_pump_control_state.values.pressure as f32;
                self.pump_pid.set_parameters(self.configuration.persistent.pid_parameters.pump_pressure_params);

                self.group.get_pressure().unwrap_or(0.0) as f32
            },
            GroupBrewControlMode::PressureCurve => {
                let target = actual_pump_control_state.values.pressure_curve.evaluate(elapsed_seconds);
                self.pump_pid.setpoint = target;
                self.pump_pid.set_parameters(self.configuration.persistent.pid_parameters.pump_pressure_params);

                self.group.get_pressure().unwrap_or(0.0) as f32
            },
            GroupBrewControlMode::OutputFlowRate => {
                self.pump_pid.setpoint = actual_pump_control_state.values.output_flow_rate as f32;
                self.pump_pid.set_parameters(self.configuration.persistent.pid_parameters.pump_output_flow_rate_params);

                self.group.get_output_flow_rate().unwrap_or(0.0) as f32
            },
            GroupBrewControlMode::OutputFlowRateCurve => {
                let target = actual_pump_control_state.values.output_flow_rate_curve.evaluate(elapsed_seconds);
                self.pump_pid.setpoint = target;
                self.pump_pid.set_parameters(self.configuration.persistent.pid_parameters.pump_output_flow_rate_params);

                self.group.get_output_flow_rate().unwrap_or(0.0) as f32
            },
            _ => 0.0,
        };

        // The PID steps only while it owns the output, and inherits the duty cycle already
        // being commanded on the way in. Stepping it unconditionally -- which is what this
        // did -- wound the integral up against the hardcoded `0.0` process value of the
        // arm above every time the pump was under open-loop control, and left it at zero
        // for a PID taking over a running pump. See `crate::pump_transfer`.
        let pump_pid_out = match self.pump_pid_engagement.transfer_for(actual_pump_control_state.mode) {
            PumpPidTransfer::Hold => None,
            PumpPidTransfer::Engage { seed_from_duty } => {
                // Setpoint and gains are already set for this mode by the match above, and
                // both matter: the seed is `target_output - kp * error`.
                self.pump_pid.infer_and_set_integral(seed_from_duty.value() as f32, pump_pv);
                Some(self.pump_pid.step(PidIn::new(pump_pv, delta_t)))
            }
            PumpPidTransfer::Continue => Some(self.pump_pid.step(PidIn::new(pump_pv, delta_t))),
        };

        // What the mode alone asks for, before any limit. `pump_pid_out` is `Some` exactly
        // when the mode is closed-loop, so it -- rather than a second list of modes that
        // could drift from `is_closed_loop` -- picks the branch.
        //
        // Everything below the controller is on the pump's 0-255 scale; the operator's
        // percentages are converted here, once, on the way in. `None` is `Off`: the pump is
        // not driven at all, and a limit must not change that.
        let (brewing, main_output) = if let Some(pump_pid_out) = pump_pid_out {
            // The PID computes natively in 0-255, so its output needs narrowing but not
            // rescaling.
            (true, Some(pump_pid_out.out))
        } else {
            match actual_pump_control_state.mode {
                GroupBrewControlMode::FullOn => {
                    (true, Some(HexadecimalDutyCycleType::FULL.value() as f32))
                },
                GroupBrewControlMode::FixedDutyCycle => {
                    let duty_cycle: HexadecimalDutyCycleType =
                        actual_pump_control_state.values.duty_cycle.into();
                    (true, Some(duty_cycle.value() as f32))
                }
                GroupBrewControlMode::FixedDutyCycleCurve => {
                    // The curve is authored in percent and evaluates to an `f32`, so going
                    // through `DutyCycle` costs no resolution -- the narrowing to a byte
                    // happens once, on the far side of the conversion.
                    let target_percent = DutyCycleType::from_f32(
                        actual_pump_control_state.values.duty_cycle_curve.evaluate(elapsed_seconds),
                    );
                    let target_duty_cycle: HexadecimalDutyCycleType = target_percent.into();
                    (true, Some(target_duty_cycle.value() as f32))
                }
                // `Off`, and anything else `is_closed_loop` declines.
                _ => (false, None),
            }
        };

        // The limit loop, and the selector. See `crate::pump_limit` for why this is a
        // min-select rather than a switch, and for what the two loops owe each other.
        let commanded = main_output.unwrap_or(0.0);
        let limit_pid_out = self.step_limit_loop(&actual_pump_control_state, commanded, delta_t);
        let selection = pump_limit::select(commanded, limit_pid_out.map(|out| out.out));

        // Remembered rather than recomputed: the status publisher runs on its own cadence and
        // has no way to know whether the limit was the loop that won.
        self.last_brew_limit = pump_limit::limit_setpoint(
            actual_pump_control_state.limit,
            &actual_pump_control_state.values,
        )
        // `None` while the loop is not running, which is what `Off` and unarmed both mean.
        .filter(|_| limit_pid_out.is_some())
        .map(|value| BrewLimitStatus {
            mode: actual_pump_control_state.limit,
            value,
            binding: selection.binding,
        });

        // External reset feedback. Whichever loop did *not* get the output is forced to the
        // one that did, or it integrates against an error it is not driving, winds up, and
        // takes over with a step the next time it wins. An open-loop main mode has no
        // integral to hold, so only the limit is tracked there.
        if let Some(limit_out) = limit_pid_out {
            if selection.binding {
                if let Some(main_out) = pump_pid_out {
                    self.pump_pid.track_to(selection.output, &main_out);
                }
            } else {
                self.limit_pid.track_to(selection.output, &limit_out);
            }
        }

        let output = if selection.binding {
            // The limit loop is what is actually driving the pump, so report its terms
            // rather than the main loop's. `brew_limit.binding` in the status and the shot
            // log is what says which quantity they belong to.
            PumpOutput::PidOutput(limit_pid_out.expect("binding implies a limit output"))
        } else if let Some(pump_pid_out) = pump_pid_out {
            PumpOutput::PidOutput(pump_pid_out)
        } else if brewing {
            // `from_f32` saturates, where the bare `as u8` this replaced did not.
            PumpOutput::FixedDutyCycle(HexadecimalDutyCycleType::from_f32(selection.output))
        } else {
            PumpOutput::Off
        };

        // One place drives the pump, from one number, whichever loop produced it.
        self.group.set_brewing_state(brewing, output.hexadecimal_duty_cycle()).await;

        // What the next `Engage` inherits, which is why it is recorded in every mode.
        self.pump_pid_engagement.record_commanded_duty(output.hexadecimal_duty_cycle());
        output
    }

    async fn update_boiler(&mut self, actual_boiler_control_state: BoilerControlState, delta_t: f32) -> Output {
        let mut boiler_pv = match actual_boiler_control_state.mode {
            BoilerControlMode::Temperature => {
                self.boiler_pid.setpoint = actual_boiler_control_state.values.target_temperature as f32;
                self.boiler_pid.set_parameters(self.configuration.persistent.pid_parameters.boiler_temperature_params);

                self.boiler.get_temperature().unwrap_or(0.0) as f32
            }
            BoilerControlMode::Pressure => {
                self.boiler_pid.setpoint = actual_boiler_control_state.values.target_pressure as f32;
                self.boiler_pid.set_parameters(self.configuration.persistent.pid_parameters.boiler_pressure_params);

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
                self.boiler.set_heating_element_duty_cycle(DutyCycleType::OFF).await;
                Output::Off
            },
            _ => {
                // Over-temperature interlock. The dual-boiler has had one on each boiler
                // (`dual_boiler_single_group.rs:1184`, `:1259`); this controller published a
                // `max_temperature` for both of its virtual boilers and enforced neither, so
                // the only thing bounding the element was the setpoint itself. One element
                // serves both roles, so the ceiling comes from the controller state rather
                // than from a single configured maximum.
                let max_temperature = crate::single_boiler_state::max_temperature_for(self.state);
                let too_hot = self.boiler.get_temperature()
                    .map_or(false, |current| current as f32 >= max_temperature);

                // Dry-run protection: disable heating if water level too low
                let boiler_level = self.boiler.get_water_level();
                let duty_cycle = if too_hot {
                    // Constant text, for the reason the dual-boiler gives at the same spot:
                    // an interpolated reading would change with ADC noise on every iteration
                    // of a 10 Hz loop and defeat the bus sink's duplicate suppression. The
                    // temperature and the maximum both reach the host in `Status` and
                    // `Configuration` already.
                    log_warn!("Boiler heating disabled: temperature at or above configured maximum");
                    DutyCycleType::OFF
                } else if !Self::is_boiler_level_safe(boiler_level, &self.boiler_config) {
                    log_warn!("Boiler heating disabled: water level below minimum safe level");
                    DutyCycleType::OFF
                } else {
                    // The boiler PID is clamped to 0-100 by `limited_pid`, so this stays a
                    // percentage -- unlike the pump's, which moved to 0-255.
                    DutyCycleType::from_f32(boiler_pid_out.out)
                };

                self.boiler.set_heating_element_duty_cycle(duty_cycle).await;

                if duty_cycle == DutyCycleType::OFF && boiler_pid_out.out > 0.0 {
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
                (self.configuration.persistent.brew_boiler_control_state, self.configuration.ephemeral.group_brew_control_state)
            }
            SingleBoilerSingleGroupControllerState::PumpingToWaterTap => {
                let mut pump_state = self.configuration.ephemeral.group_brew_control_state;
                pump_state.mode = GroupBrewControlMode::FullOn;
                (self.configuration.persistent.brew_boiler_control_state, pump_state)
            },
            SingleBoilerSingleGroupControllerState::BrewModeIdle => {
                let mut pump_state = self.configuration.ephemeral.group_brew_control_state;
                pump_state.mode = GroupBrewControlMode::Off;
                (self.configuration.persistent.brew_boiler_control_state, pump_state)
            }
            SingleBoilerSingleGroupControllerState::SteamModeIdle => {
                let mut pump_state = self.configuration.ephemeral.group_brew_control_state;
                pump_state.mode = GroupBrewControlMode::Off;
                (self.configuration.persistent.steam_boiler_control_state, pump_state)
            }
            SingleBoilerSingleGroupControllerState::PowerSave => {
                let mut boiler_state = BoilerControlState::default();
                boiler_state.mode = BoilerControlMode::Off;
                let mut pump_state = GroupBrewControlState::default();
                pump_state.mode = GroupBrewControlMode::Off;
                (boiler_state, pump_state)
            }
        };

        // The mode gates everything above, applied last so there is one place it can be
        // forgotten rather than five. A machine that is off heats nothing and pumps nothing,
        // whatever state the controller happens to be in.
        //
        // The dual-boiler spells this as three `effective_*_control_mode` helpers on its
        // configuration, because there the caller has to choose between two real boilers.
        // Here the match above has already chosen, so the same rule is one branch rather
        // than three methods.
        //
        // Only the *modes* are forced to `Off`; the values are left alone. The targets a
        // user configured are what the machine returns to when it is switched back on, and
        // what the interface goes on showing as the setpoint in the meantime.
        let (mut actual_boiler_control_state, mut actual_pump_control_state) =
            (actual_boiler_control_state, actual_pump_control_state);

        if self.configuration.ephemeral.mode != MachineMode::On {
            actual_boiler_control_state.mode = BoilerControlMode::Off;
            actual_pump_control_state.mode = GroupBrewControlMode::Off;
        }

        (actual_boiler_control_state, actual_pump_control_state)
    }

    async fn send_status(&mut self, boiler_output: Output, pump_output: PumpOutput) {
        // One element, two published slots: the inactive one reports `Off` so that an
        // interface can tell which of them the element is actually under. The split lives in
        // `single_boiler_state` beside its inverse, `active_boiler_index`, because the two
        // are one convention and a test there holds them together.
        let (brew_boiler_output, steam_boiler_output) =
            crate::single_boiler_state::element_outputs_for(self.state, boiler_output);

        let brew_boiler_status = BoilerStatus {
            temperature: self.boiler.get_temperature(),
            pressure: self.boiler.get_pressure(),
            water_level: self.boiler.get_water_level(),
            output: brew_boiler_output,
            control_state: self.configuration.persistent.brew_boiler_control_state,
        };

        let virtual_steam_boiler_status = BoilerStatus {
            temperature: self.boiler.get_temperature(),
            pressure: self.boiler.get_pressure(),
            water_level: self.boiler.get_water_level(),
            output: steam_boiler_output,
            control_state: self.configuration.persistent.steam_boiler_control_state,
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
            // Calculate output_volume from output_weight (assuming density ~1 g/ml).
            //
            // Deliberately scale-weight only, so this stays a *measured* quantity and is
            // `None` without a scale. `dual_boiler_single_group` falls back to the input
            // volume accumulated since the first drop; that is an estimate, and this machine
            // does not make it.
            let output_volume = self.group.get_output_weight().map(|w| w as OutputVolumeType);
            BrewStatus {
                brew_time: start.elapsed().into(),
                brew_input_volume,
                shot_state: self.shot_state.state(),
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
            control_state: self.configuration.ephemeral.group_brew_control_state,
            previous_brew: self.previous_brew.map(|info| info.into()),
            // `None` on every machine this controller currently runs, since no
            // single-boiler firmware passes a tacho receiver. Routed through the getter
            // rather than hardcoded so wiring one is a firmware-only change.
            pump_rpm: self.group.get_pump_rpm(),
            // The resolved setpoint -- see the dual-boiler controller's copy for why this
            // is taken from the PID rather than from `control_state.values` above.
            brew_control_target: {
                let mode = self.configuration.ephemeral.group_brew_control_state.mode;
                // Same condition as `is_brewing` above -- this controller tracks brewing as a
                // state rather than as a flag.
                let brewing = self.state == SingleBoilerSingleGroupControllerState::Brewing;
                if brewing && mode != GroupBrewControlMode::Off {
                    Some(variegated_controller_types::BrewControlTarget {
                        mode,
                        value: self.pump_pid.setpoint,
                    })
                } else {
                    None
                }
            },
            // Set by `update_pump`, which is the only place that knows whether the limit won.
            brew_limit: self.last_brew_limit,
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
                // Carried through unchanged -- see the note on the dual-boiler copy.
                sntp_sync_seq: status.sntp_sync_seq,
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
            // The machine's own mode, not a placeholder. This was `Default::default()` --
            // which is `MachineMode::Off` -- so the machine reported itself off no matter
            // what, and there was no field to report from anyway.
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

    /// Write the persistent half of the configuration to its store.
    ///
    /// One method, because six commands end in it. The dual-boiler's twin takes a mutex
    /// under a timeout where this one owns its store outright, which is the whole reason
    /// persistence stayed on the controllers when the commands themselves were shared.
    async fn save_persistent_configuration(&mut self) {
        self.configuration_store.save_settings(&self.configuration.persistent).await.ok();
    }

    /// Carry out what a shared target command decided.
    ///
    /// The decision is `crate::command::targets`, which is pure and host-tested; this is the
    /// half that needs a clock and a store. `CurveAction::Leave` is deliberately not
    /// `Clear` — see [`crate::command::CurveAction`].
    async fn apply_target_outcome(&mut self, outcome: command::TargetOutcome) {
        match outcome.curve {
            command::CurveAction::Start => self.curve_start_time = Some(Instant::now()),
            command::CurveAction::Clear => self.curve_start_time = None,
            command::CurveAction::Leave => {}
        }
        if outcome.persist {
            self.save_persistent_configuration().await;
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
            // The six target commands are `crate::command::targets`, shared with the
            // dual-boiler controller and host-tested there. What is left here is the half
            // that genuinely differs between the machines: which store the result goes to,
            // and the curve clock it is measured against.
            MachineCommand::SetBoilerControlTarget(boiler_index, mode, values_update) => {
                let outcome = command::targets::set_boiler_control_target(
                    &mut self.configuration, boiler_index, mode, values_update);
                self.apply_target_outcome(outcome).await;
            }
            MachineCommand::SetBoilerControlTargetValues(boiler_index, update) => {
                let outcome = command::targets::set_boiler_control_target_values(
                    &mut self.configuration, boiler_index, update);
                self.apply_target_outcome(outcome).await;
            }
            MachineCommand::SetGroupBrewControlTarget(group_index, mode, values_update) => {
                let outcome = command::targets::set_group_brew_control_target(
                    &mut self.configuration, group_index, mode, values_update);
                self.apply_target_outcome(outcome).await;
            }
            MachineCommand::SetGroupBrewControlTargetValues(group_index, update) => {
                let outcome = command::targets::set_group_brew_control_target_values(
                    &mut self.configuration, group_index, update);
                self.apply_target_outcome(outcome).await;
            }
            MachineCommand::SetGroupBrewLimit(group_index, limit, values_update) => {
                let outcome = command::targets::set_group_brew_limit(
                    &mut self.configuration, group_index, limit, values_update);
                self.apply_target_outcome(outcome).await;
            }
            MachineCommand::SetPidParameters(target, params) => {
                let outcome = command::targets::set_pid_parameters(
                    &mut self.configuration, target, params);
                self.apply_target_outcome(outcome).await;
            }
            // The mode table is `crate::single_boiler_state`, which is host-tested. It used
            // to be two chains of `if`/`else if` here, and `PowerSave` had no arm returning
            // from it -- a machine that entered power save stayed there until reboot, and
            // the steam switch, which requires `BrewModeIdle`, could never work again. That
            // went unnoticed because no UI on either board sends these commands.
            //
            // Refusals are deliberate and stay refusals: the machine will not change mode
            // while it is brewing.
            MachineCommand::EnableBoiler(boiler_index) => {
                self.apply_boiler_mode_command(true, boiler_index).await;
            }
            MachineCommand::DisableBoiler(boiler_index) => {
                self.apply_boiler_mode_command(false, boiler_index).await;
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
                //log_info!("Updating comms status: wifi={}, timestamp={:?}", status.wifi_connected, status.timestamp);
                self.comms_status = Some(status);
                self.comms_status_received_instant = Some(Instant::now());
            }
            MachineCommand::RunRoutine(_, _) | MachineCommand::CancelRoutine => {
                defmt::warn!("Ignoring unsupported command in finally block: {:?}", command);
            }
            // Turning the machine on and off. This had no arm at all, so it fell into the
            // catch-all below and vanished -- which is why it failed identically from the
            // UI, the web interface and the debug link, with nothing anywhere saying so.
            MachineCommand::SetMachineMode(mode) => {
                log_info!("Setting machine mode to {:?}", mode);
                self.configuration.ephemeral.mode = mode;

                // Anything in flight stops with it. Leaving a brew running on a machine the
                // user has just switched off would be the surprising reading of "off", and
                // the gate in `get_control_targets` would cut the pump underneath it
                // anyway -- this way the state machine agrees, and the shot log is closed.
                if mode != MachineMode::On {
                    match self.state {
                        SingleBoilerSingleGroupControllerState::Brewing
                        | SingleBoilerSingleGroupControllerState::PumpingToWaterTap => {
                            self.transition_to_state(
                                SingleBoilerSingleGroupControllerState::BrewModeIdle,
                            )
                            .await;
                        }
                        _ => {}
                    }
                }
            }
            MachineCommand::OptimizeConfigurationStorage => {
                log_info!("Optimizing configuration storage");
                if let Err(e) = self.configuration_store.optimize_storage().await {
                    log_warn!("Failed to optimize configuration storage: {}", e);
                }
            }
            // The three routine mutations. These had no arms at all, so a routine deleted
            // from the web interface fell into the catch-all below and the machine went on
            // serving it -- the same failure `SetMachineMode` had above, and just as
            // invisible, since the comms processor's `DELETE` returns before the command
            // has been anywhere near a repository.
            //
            // No "the list changed" notification here. `add_routine`, `remove_routine` and
            // `update_routine` raise `ROUTINES_CHANGED` themselves, and the transceiver
            // pushes a fresh summary list off the back of it.
            MachineCommand::AddRoutine(routine) => {
                command::stores::add_routine(self.routine_repository, routine).await;
            }
            MachineCommand::RemoveRoutine(idx) => {
                command::stores::remove_routine(self.routine_repository, idx).await;
            }
            MachineCommand::UpdateRoutine(idx, routine) => {
                command::stores::update_routine(self.routine_repository, idx, routine).await;
            }
            // Handed off rather than run here, which it used to be. On a flash-backed
            // repository `optimize_storage` erases the whole range and rewrites every
            // routine, sleeping a millisecond between each so it does not trip the
            // watchdog -- hundreds of milliseconds to seconds, and every one of them spent
            // inside this loop, which is the loop that runs the PID and the interlocks. The
            // element would hold whatever it was last commanded to for the duration.
            //
            // It was safe inline for exactly as long as the only implementation was the
            // in-memory one, whose `optimize_storage` returns `Ok(())` without doing
            // anything. That stopped being true when this controller became generic.
            //
            // `try_send` on a depth-4 channel: an optimization already queued is the same
            // request, so dropping the second is right, and a full channel must not park
            // the control loop.
            MachineCommand::OptimizeRoutineStorage => {
                log_info!("Queueing routine storage optimization");
                if self.storage_command_sender.try_send(StorageCommand::OptimizeRoutines).is_err() {
                    log_warn!("Storage command channel full, dropping OptimizeRoutines");
                }
            }
            MachineCommand::OptimizeScheduleStorage => {
                log_warn!("OptimizeScheduleStorage not supported for single boiler controller (no schedule store)");
            }
            MachineCommand::InferGroupPressureIntegral(group_index, target_pressure) => {
                if group_index == 0 {
                    log_info!("Inferring group pressure integral for target pressure: {} bar", target_pressure);

                    // The duty cycle the pump is actually running at, not the
                    // `FixedDutyCycle` target -- see `last_commanded_duty` for why those
                    // are not interchangeable.
                    let current_duty_cycle = self.pump_pid_engagement.last_commanded_duty();
                    let current_pressure = self.group.get_pressure().unwrap_or(0.0);

                    // Set up PID for pressure control
                    self.pump_pid.setpoint = target_pressure as f32;
                    self.pump_pid.set_parameters(self.configuration.persistent.pid_parameters.pump_pressure_params);

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

                    // The duty cycle the pump is actually running at, not the
                    // `FixedDutyCycle` target -- see `last_commanded_duty` for why those
                    // are not interchangeable.
                    let current_duty_cycle = self.pump_pid_engagement.last_commanded_duty();
                    let current_flow_rate = self.group.get_input_flow_rate().unwrap_or(0.0);

                    // Set up PID for flow rate control
                    self.pump_pid.setpoint = target_flow_rate as f32;
                    self.pump_pid.set_parameters(self.configuration.persistent.pid_parameters.pump_flow_rate_params);

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

                    // The duty cycle the pump is actually running at, not the
                    // `FixedDutyCycle` target -- see `last_commanded_duty` for why those
                    // are not interchangeable.
                    let current_duty_cycle = self.pump_pid_engagement.last_commanded_duty();
                    let current_output_flow_rate = self.group.get_output_flow_rate().unwrap_or(0.0);

                    // Set up PID for output flow rate control
                    self.pump_pid.setpoint = target_output_flow_rate as f32;
                    self.pump_pid.set_parameters(self.configuration.persistent.pid_parameters.pump_output_flow_rate_params);

                    // Infer and set the integral
                    self.pump_pid.infer_and_set_integral(current_duty_cycle.value() as f32, current_output_flow_rate as f32);

                    log_info!("Set output flow rate integral based on duty cycle {}/255 and output flow rate {}", current_duty_cycle.value(), current_output_flow_rate);
                } else {
                    log_error!("Invalid group index: {}", group_index);
                }
            }
            MachineCommand::AssociateBluetoothPeripheral(association) => {
                if command::bluetooth::associate(&mut self.bluetooth_associations, association)
                    .wanted()
                {
                    self.save_bluetooth_associations().await;
                }
            }
            MachineCommand::RemoveBluetoothPeripheral(id) => {
                if command::bluetooth::remove(&mut self.bluetooth_associations, id).wanted() {
                    self.save_bluetooth_associations().await;
                }
            }
            MachineCommand::SetBluetoothPeripheralEnabled(id, enabled) => {
                if command::bluetooth::set_enabled(&mut self.bluetooth_associations, id, enabled)
                    .wanted()
                {
                    self.save_bluetooth_associations().await;
                }
            }
            MachineCommand::ScanForBluetoothPeripherals => {
                // What counts as busy is this machine's to decide, and is the only part of
                // starting a scan that differs between the two controllers.
                //
                // `SteamModeIdle` is not busy -- despite the boiler being hot, nothing is
                // flowing and no shot is at stake.
                let busy = self.current_routine.is_some()
                    || matches!(
                        self.state,
                        SingleBoilerSingleGroupControllerState::Brewing
                            | SingleBoilerSingleGroupControllerState::PumpingToWaterTap
                    );
                command::bluetooth::start_scan(
                    &mut self.bluetooth_status,
                    &mut self.bluetooth_scan_deadline,
                    self.bluetooth_scan_sender,
                    busy,
                );
            }
            MachineCommand::UpdateBluetoothScan(update) => {
                command::bluetooth::apply_scan_update(
                    &mut self.bluetooth_status,
                    &mut self.bluetooth_scan_deadline,
                    update,
                );
            }
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
            MachineCommand::SetShotUploadSettings(settings) => {
                // The settings UI's edit. Merged rather than replacing, because the browser
                // is never sent the token and so cannot send it back -- see
                // `ShotUploadTokenUpdate`. The compare-before-save matters: the panel posts
                // on every save, changed or not.
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
                // uses it. The comms processor parses it at upload time and reports what
                // it found; two parsers on one string with nothing keeping them in
                // agreement is worse than one.
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
            MachineCommand::SetTimezone(setting) => {
                match variegated_timekeeping::TimeZoneWrapper::from_iana_name(setting.as_str()) {
                    Some(zone) => {
                        // Applied before it is stored, so a `TimeKeeper` that refuses it does
                        // not leave flash claiming a zone the machine is not keeping time in.
                        if let Err(e) = variegated_timekeeping::TimeKeeper::set_timezone(zone) {
                            log_warn!("Failed to apply timezone: {:?}", e);
                        } else {
                            log_info!("Timezone set to {}", setting.as_str());
                            self.timezone = setting;
                            self.save_timezone().await;
                        }
                    }
                    // Refused, not stored. See the dual-boiler's arm.
                    None => log_warn!("Refusing unknown timezone: {}", setting.as_str()),
                }
            }
            MachineCommand::RequestConfiguration => {
                // Published here rather than by setting a pending flag, so the answer is on
                // the channel before this function returns. The caller is a consumer that
                // has just discovered it has no configuration at all; making it wait for
                // the next comparison tick would be an odd way to answer "send it now".
                //
                // Deliberately does not touch the caller's `last_configuration`. Publishing
                // a value equal to it is harmless -- the change comparison in `task` sees
                // no change and does not publish again -- whereas resetting it would make
                // the *next* genuine change invisible.
                log_info!("Configuration republish requested");
                let config = self.general_configuration(self.current_configuration());
                self.configuration_channel_sender.publish_immediate(config);
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
            MachineCommand::DeleteShotLog(id) => {
                // Identical to the dual-boiler arm, for the reason the arm above gives:
                // the sender is `None` on every build today, so this always refuses, but
                // a single-boiler machine that gained a card reader should not need
                // deletion re-implemented from scratch.
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
            // Everything this controller does not implement, named rather than dropped.
            //
            // This arm was `_ => {}`. Sixteen of the fifty-one `MachineCommand` variants
            // land here, and while they were silent a command that did nothing was
            // indistinguishable from one that worked -- which is exactly how
            // `SetMachineMode` came to be reported as "the machine is always Off and cannot
            // be turned on, from the UI, the web interface *or* the debug link". All three
            // were accepting the command and throwing it away.
            //
            // `label()` rather than `{:?}`: `MachineCommand` has no `Debug`, and this needs
            // to reach the `log` half of `log_warn!` -- and so the debug bus and the host's
            // Events pane -- not only a probe.
            //
            // Some of these are genuinely inapplicable to a one-boiler, one-element machine
            // and always will be; others are unimplemented and tracked in
            // `SINGLE_BOILER_GAPS.md`. From here the two look the same, which is why this
            // says "does not handle" rather than guessing at a reason.
            other => log_warn!("Unhandled MachineCommand: {}", other.label()),
        }
    }

    /// Apply an `EnableBoiler`/`DisableBoiler` against the mode table.
    ///
    /// The decision is `single_boiler_state::boiler_mode_transition`, which is pure and
    /// host-tested; this only carries it out and says what happened. A `None` is a refusal
    /// rather than an error -- the machine declining to change mode mid-brew is the wanted
    /// behaviour -- but it is logged, because it is also what a caller sees when it names a
    /// boiler that does not exist.
    async fn apply_boiler_mode_command(&mut self, enable: bool, boiler_index: BoilerIndex) {
        match crate::single_boiler_state::boiler_mode_transition(self.state, enable, boiler_index) {
            Some(next) => {
                log_info!("Boiler mode: {:?} -> {:?}", self.state, next);
                self.transition_to_state(next).await;
            }
            None => log_warn!(
                "Refusing to {} boiler {} in state {:?}",
                if enable { "enable" } else { "disable" },
                boiler_index,
                self.state
            ),
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
                    self.group.set_brewing_state(true, HexadecimalDutyCycleType::OFF).await;
                    self.started_brewing().await;
                }
                (SingleBoilerSingleGroupControllerState::Brewing, SingleBoilerSingleGroupControllerState::BrewModeIdle) => {
                    variegated_log::emit_event(DebugEvent::BrewStopped { group: SingleGroup.as_index() });
                    self.group.set_brewing_state(false, HexadecimalDutyCycleType::OFF).await;
                    self.stopped_brewing().await;
                }
                _ => {}
            }
        }
    }

    /// Advance the shot-phase state machine from the current sensor readings.
    ///
    /// Called every control-loop tick; the tracker rate-limits itself to its own sample
    /// interval, so calling it more often than that costs nothing.
    fn update_shot_state(&mut self) {
        if self.state != SingleBoilerSingleGroupControllerState::Brewing {
            return;
        }

        if self.shot_state.state().is_none() {
            log_warn!("Shot state is None while brewing - restarting shot state tracking");
            self.shot_state.start();
        }

        let inputs = crate::ShotStateInputs {
            input_flow_rate: self.group.get_input_flow_rate(),
            pressure: self.group.get_pressure(),
            output_weight: self.group.get_output_weight(),
            // Always `None` on this machine -- there is no conductivity probe -- which
            // leaves the weight path, and that one only counts once the puck has saturated.
            output_electrical_conductivity: self.group.get_output_electrical_conductivity(),
        };

        let Some(new_state) = self.shot_state.update(Instant::now().as_millis(), inputs) else {
            return;
        };

        // The only externally visible sign that detection fired: this machine's display
        // carries no shot-phase readout, so a tuning pass would start from these lines.
        log_info!(
            "Shot state transition: -> {:?} (flow: {:?}, pressure: {:?}, weight: {:?})",
            new_state,
            inputs.input_flow_rate,
            inputs.pressure,
            inputs.output_weight
        );
    }

    async fn started_brewing(&mut self) {
        self.start_manual_shot_log();
        self.brew_start_time = Some(Instant::now());
        self.brew_start_input_volume = self.group.get_input_volume();
        self.accumulated_extracted_solids = Some(0.0);
        self.last_extraction_time = Some(Instant::now());
        // Belongs with the resets rather than beside the tare below: `start` discards
        // everything from the previous shot, including its peak flow and pressure trough.
        self.shot_state.start();
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
        self.shot_state.stop();

        self.finish_manual_shot_log();

        let _ = self.group.scale_set_configuration(ScaleConfiguration {
            zero_tracking: Some(true),
            smoothing: Some(false)
        }).await;
    }

    /// Open a shot log for a brew nobody scripted.
    ///
    /// See the equivalent pair in `dual_boiler_single_group` for why the logger needs
    /// nothing a manual brew lacks, and why both guards are here.
    fn start_manual_shot_log(&mut self) {
        use variegated_controller_types::{ShotLogMetadata, ShotStatus, ShotType};

        if self.current_routine.is_some() || self.shot_logger.is_logging() {
            return;
        }

        self.shot_logger.start_shot(ShotLogMetadata {
            annotations: self.pending_annotations.clone(),
            shot_type: ShotType::Manual,
            group_index: SingleGroup.as_index(),
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
    /// Guarded on `manual_shot_active` because `handle_routine_exit` stops brewing before
    /// it finishes its own log -- see the note on the dual-boiler version.
    fn finish_manual_shot_log(&mut self) {
        use variegated_controller_types::ShotStatus;

        if !self.manual_shot_active {
            return;
        }
        self.manual_shot_active = false;

        self.shot_logger.finish_shot(ShotStatus::Completed);
        self.send_latest_shot_log();
        self.pending_annotations.clear();
    }

    /// Hand the most recently finished shot to the storage task, if there is one listening.
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
        // Cloned rather than borrowed so the CRC lookup below can take `repo` again. The
        // routine is cloned into the execution context a few lines down regardless, so this
        // moves an allocation rather than adding one.
        let routine = repo.get_routine(routine_index).await.cloned();
        // Zero means the repository had no CRC for this index, which can only happen for a
        // routine it could not encode -- the same condition that makes one unstorable. A
        // reader matching zero against a library finds nothing, which is the right answer.
        let routine_crc = repo.get_routine_crc(routine_index).await.unwrap_or(0);

        if let Some(routine) = routine {
            // The backstop; see the equivalent block in `dual_boiler_single_group`.
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
            // documents, and all of it before the metadata below clones the pending block.
            crate::routine_annotations::apply_static_shot_annotations(
                &routine,
                &mut self.pending_annotations,
            );
            let runtime_params = crate::routine_annotations::seed_linked_parameters(
                &routine,
                runtime_params.clone(),
                &self.pending_annotations,
            );

            // Before the context merges defaults in; see the dual-boiler twin.
            crate::routine_annotations::record_linked_parameters(
                &routine,
                runtime_params.as_ref(),
                &mut self.pending_annotations,
            );

            // Create routine execution context
            let routine_execution_context = RoutineExecutionContext::new(
                routine_index,
                routine.clone(),
                self.state,
                self.current_configuration(),
                runtime_params
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
                    routine_crc,
                }),
                start_time_millis: embassy_time::Instant::now().as_millis(),
                end_time_millis: None,
                final_status: ShotStatus::Running,
                // Filled in by `finish_shot` -- see the equivalent block in
                // `dual_boiler_single_group`.
                recorded_at_unix_millis: None,
            };
            // A manual brew already in progress hands its log over here -- see the note on
            // the equivalent line in `dual_boiler_single_group`.
            self.manual_shot_active = false;
            self.shot_logger.start_shot(metadata);
            self.previous_routine_step = None;

            // Cleared here so a new routine does not inherit the previous one's loss timer.
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

            // `finally` runs *before* the restore, so that nothing a routine sets outlives
            // it. These commands are for actions -- stopping a brew, taring a scale -- and
            // any configuration one of them touches is undone by the restore below. That is
            // the intended reading rather than a side effect: a routine's effects end with
            // the routine.
            //
            // It used to run last, which made `finally` the one hole in that rule: a
            // `SetGroupPressure` there survived the routine and nothing said so.
            for cmd in finally_commands {
                self.handle_routine_finally_commands(cmd).await;
            }

            // Restore saved configuration by splitting into persistent and ephemeral parts
            let saved_config = &routine.saved_configuration;
            self.configuration.persistent = saved_config.persistent;
            self.configuration.ephemeral = saved_config.ephemeral;
            // Save the restored persistent configuration
            self.save_persistent_configuration().await;
            self.curve_start_time = None;  // Reset curve start time when routine exits

            // **Never resume an active state.** The old ordering prevented this by accident:
            // `finally`'s `StopBrewing` ran last and won. With `finally` moved ahead of the
            // restore, handing `saved_state` straight back would resume a brew the routine
            // had just stopped -- so an active state falls back to idle, and only an idle
            // one is restored as-is.
            let resume_state = match routine.saved_state {
                SingleBoilerSingleGroupControllerState::Brewing
                | SingleBoilerSingleGroupControllerState::PumpingToWaterTap => {
                    SingleBoilerSingleGroupControllerState::BrewModeIdle
                }
                other => other,
            };
            self.transition_to_state(resume_state).await;

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

// `impl From<SingleBoilerSingleGroupConfiguration> for Configuration` now lives beside the
// type it converts, in `crate::single_boiler_config`.