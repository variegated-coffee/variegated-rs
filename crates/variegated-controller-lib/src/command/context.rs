//! The one dispatcher, and the machine's side of it.
//!
//! # Why this is a trait
//!
//! Sharing the command *bodies* left the dispatch duplicated: roughly two thirds of the arms
//! in the two controllers were byte-identical one-liners, and three identical copies of
//! "call the shared handler, then note the publish" is still three copies. Removing that means
//! exactly one `match` over `MachineCommand` can exist, which in turn means the machine-specific
//! arms have to be reachable from shared code. That is what [`MachineCommandContext`] is.
//!
//! # What it buys, and what it costs
//!
//! [`MachineCommandContext::handle_machine_command`] is the only place a `MachineCommand` is
//! consumed anywhere in this crate. A variant added to the enum is a compile error *there*, in
//! one place; if the answer is "the machines do this differently", adding a required method
//! below is then a compile error in both implementations. The guarantee the two exhaustive
//! matches used to give is kept, and the duplication they cost is not.
//!
//! The cost is the accessor surface below. It is deliberately accessors rather than behaviour:
//! everything a shared arm does lives in the sibling modules and is host-tested there, and the
//! methods here only hand it the field it works on.

use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::channel::Sender;
use embassy_sync::mutex::Mutex;
use embassy_time::Instant;
use variegated_controller_types::bluetooth::{BluetoothAssociations, BluetoothScanStatus};
use variegated_controller_types::shot_log::ShotAnnotations;
use variegated_controller_types::shot_upload::ShotUploadConfig;
use variegated_controller_types::timezone::TimezoneSetting;
use variegated_controller_types::wifi::StoredWifiCredentials;
use variegated_controller_types::{
    BoilerIndex, CommsStatus, GroupIndex, HeatingElementContentionStrategy, MachineCommand,
    MachineMode, PumpConfiguration, ScaleSelector, SteamWandIndex, StorageCommand, ValveOpenType,
    WaterDispersalPumpStrategy, WaterTapIndex,
};
use variegated_log::{log_error, log_info, log_warn};

use crate::routine::RoutineRepository;
use crate::schedule::ScheduleStore;
use crate::settings::SettingsSink;
use crate::shot_log_query::ShotLogQuery;

use super::access::{ConfigurationAccess, TargetOutcome};
use super::group::GroupAccess;
use super::pump::PumpQuantity;
use super::{bluetooth, connectivity, shot, stores, ScaleAction};

/// Everything the shared dispatcher needs from a machine.
///
/// Implemented by both controllers. The required items split into three groups: the associated
/// types that name a machine's storage and channel generics, the accessors that hand a shared
/// handler the field it edits, and the behaviours the two machines genuinely do differently.
/// The lifetime parameter is the controller's own, and it is load-bearing rather than
/// decoration: the sender accessors below hand back `Sender<'a, ..>` rather than something
/// borrowed from `&self`, so a sender can be taken *and then* a field mutably borrowed. Tying
/// them to `&self` makes `ScanForBluetoothPeripherals` -- which needs the sender and the status
/// at once -- fail to borrow-check.
///
/// `Self: 'a` because the sender accessors return values carrying that lifetime, which only
/// makes sense for a controller that lives at least as long as the channels it holds -- which
/// both do, since the senders are its own fields.
/// `PumpLoopContext` is a supertrait rather than a bound on the one method that needs it: a
/// machine that handles commands has a pump, and the seeding command is one of them.
#[allow(async_fn_in_trait)]
pub trait MachineCommandContext<'a>: 'a + crate::command::pump::PumpLoopContext {
    /// This machine's configuration, reachable through [`ConfigurationAccess`].
    type Configuration: ConfigurationAccess;
    /// The mutex kind guarding this machine's stores.
    type StorageM: RawMutex + 'static;
    /// The mutex kind behind this machine's channels.
    type ChannelM: RawMutex;
    /// This machine's routine repository.
    type RoutineRepo: RoutineRepository + 'static;
    /// This machine's schedule store.
    type ScheduleStoreT: ScheduleStore + 'static;

    // ---- Shared state ----------------------------------------------------------------

    /// The configuration the target commands edit.
    fn configuration_mut(&mut self) -> &mut Self::Configuration;

    /// Carry out what a target command decided: the curve clock, and the store if asked.
    async fn apply_target_outcome(&mut self, outcome: TargetOutcome);

    /// Where routines live.
    fn routine_repository(&self) -> &'static Mutex<Self::StorageM, Self::RoutineRepo>;

    /// Where schedules live.
    fn schedule_store(&self) -> &'static Mutex<Self::StorageM, Self::ScheduleStoreT>;

    /// Remember that `Configuration` should go out again.
    fn note_configuration_publish(&mut self, publish: stores::Publish);

    /// Where a storage optimization request is queued, rather than run in the control loop.
    fn storage_command_sender(&self) -> Sender<'a, Self::ChannelM, StorageCommand, 4>;

    /// The three Bluetooth fields, together.
    ///
    /// One accessor rather than three because a scan update touches the status and the
    /// deadline at once, and three separate `&mut self` borrows would not coexist.
    fn bluetooth(
        &mut self,
    ) -> (&mut BluetoothAssociations, &mut BluetoothScanStatus, &mut Option<Instant>);

    /// Where an accepted discovery scan is sent, if this machine is wired for one.
    fn bluetooth_scan_sender(&self) -> Option<Sender<'a, Self::ChannelM, u16, 2>>;

    // ---- The four peripheral stores ---------------------------------------------------
    //
    // Each is a `SettingsSink` and its value, returned together so the write below can take
    // both out of one borrow. How the sink reaches its store -- owned, or through a mutex
    // under a timeout -- is the sink's business and the only thing the two machines disagree
    // about here. See `crate::settings::SettingsSink`.

    /// Where the association list is written, and the list.
    type BluetoothSink: SettingsSink<BluetoothAssociations>;
    /// Where the Wi-Fi credentials are written.
    type WifiSink: SettingsSink<StoredWifiCredentials>;
    /// Where the shot-upload configuration is written.
    type UploadSink: SettingsSink<ShotUploadConfig>;
    /// Where the timezone is written.
    type TimezoneSink: SettingsSink<TimezoneSetting>;

    /// The association store, and the list to write.
    fn bluetooth_store(&mut self) -> (&mut Self::BluetoothSink, &BluetoothAssociations);
    /// The credentials store, and the credentials.
    fn wifi_store(&mut self) -> (&mut Self::WifiSink, &StoredWifiCredentials);
    /// The upload store, and the configuration.
    fn upload_store(&mut self) -> (&mut Self::UploadSink, &ShotUploadConfig);
    /// The timezone store, and the zone.
    fn timezone_store(&mut self) -> (&mut Self::TimezoneSink, &TimezoneSetting);

    /// Remember that `Configuration` should be republished.
    fn mark_configuration_publish(&mut self);
    /// Remember that the comms processor should be sent the credentials.
    fn mark_wifi_publish(&mut self);
    /// Remember that the comms processor should be sent the upload configuration.
    fn mark_shot_upload_publish(&mut self);

    /// The stored Wi-Fi credentials.
    fn wifi_credentials_mut(&mut self) -> &mut StoredWifiCredentials;

    /// Where a provisioning-window request is sent, if this machine is wired for one.
    fn wifi_provisioning_sender(&self) -> Option<Sender<'a, Self::ChannelM, u32, 2>>;

    /// The stored shot-upload configuration.
    fn shot_upload_config_mut(&mut self) -> &mut ShotUploadConfig;

    /// The stored timezone.
    fn timezone_mut(&mut self) -> &mut TimezoneSetting;

    // ---- Persisting them, which is the same on both machines --------------------------

    /// Persist the association list, and tell the comms processor it changed.
    ///
    /// The push is not a nicety. The comms processor holds no configuration of its own, so
    /// until it is told, an association the user just created does not exist as far as the
    /// radio is concerned. It rides on the `Configuration` publish, which compares the
    /// *machine* configuration -- and these associations are deliberately not part of that,
    /// since they have a store of their own. So a change here is invisible to that comparison
    /// and has to announce itself.
    async fn save_bluetooth_associations(&mut self) {
        let (store, value) = self.bluetooth_store();
        store.store(value, "Bluetooth associations").await;
        self.mark_configuration_publish();
    }

    /// Persist the Wi-Fi credentials, and tell the comms processor they changed.
    ///
    /// Unlike the association list this deliberately does **not** ride on the `Configuration`
    /// publish: that path ends at the browser, and a password has no business on it. Hence a
    /// flag of its own rather than dirtying the configuration.
    async fn save_wifi_credentials(&mut self) {
        let (store, value) = self.wifi_store();
        store.store(value, "Wi-Fi credentials").await;
        self.mark_wifi_publish();
    }

    /// Persist the shot-upload configuration, and announce it twice.
    ///
    /// **Two flags, two destinations, and they are not interchangeable.** One sends the full
    /// configuration -- token included -- to the comms processor on its own watch. The other
    /// republishes `Configuration`, which carries the redacted view the browser reads; without
    /// it the settings panel showed a stale endpoint for up to ten seconds after an edit.
    async fn save_shot_upload_config(&mut self) {
        let (store, value) = self.upload_store();
        store.store(value, "shot upload config").await;
        self.mark_shot_upload_publish();
        self.mark_configuration_publish();
    }

    /// Persist the timezone, and republish the configuration that carries it.
    ///
    /// No watch of its own, unlike the upload configuration: the comms processor keeps time in
    /// UTC and has no use for the zone, and the browser reads it from `Configuration`.
    async fn save_timezone(&mut self) {
        let (store, value) = self.timezone_store();
        store.store(value, "timezone").await;
        self.mark_configuration_publish();
    }

    /// Forget the stored network, persistently.
    ///
    /// **Writing the cleared value is the whole point.** The state this reproduces is a machine
    /// that has *never* been provisioned; one that merely disconnected would come back knowing
    /// a network after the next reboot. Saving also marks the Wi-Fi publish, so the comms
    /// processor is told on the next tick and parks waiting to be provisioned -- which is what
    /// makes a reboot unnecessary to reach the state, and still worth doing to prove it
    /// survives one.
    ///
    /// A no-op with a distinct log line when there was nothing stored, rather than a silent
    /// one: this is a debug affordance, and "already clear" is a different answer from
    /// "cleared" to whoever just typed it.
    ///
    /// Never logs the SSID, here or anywhere on this path: a credential is not written to a log
    /// someone may be sharing a screen of while provisioning.
    async fn clear_wifi_credentials(&mut self) {
        if self.wifi_credentials_mut().0.is_none() {
            log_info!("Wi-Fi credentials already cleared; nothing to forget");
            return;
        }
        *self.wifi_credentials_mut() = StoredWifiCredentials(None);
        self.save_wifi_credentials().await;
        log_warn!("Wi-Fi credentials cleared; this machine is now unprovisioned");
    }

    /// The annotations waiting to be stamped onto the next shot.
    fn pending_annotations_mut(&mut self) -> &mut ShotAnnotations;

    /// Where a request that has to touch the card goes, if this machine has one.
    fn shot_log_query_sender(&self) -> Option<Sender<'a, Self::ChannelM, ShotLogQuery, 1>>;

    /// Record what the comms processor last reported.
    fn set_comms_status(&mut self, status: CommsStatus);

    // ---- Water interlocks -------------------------------------------------------------

    /// Whether the tank is below its empty threshold, on a machine that has one.
    ///
    /// Machine-specific only because the tank is a `variegated_hal::Tank` parameterised
    /// differently on each; the *policy* built on it is shared below.
    fn is_tank_empty(&mut self) -> bool;

    /// The machine-wide configuration the policy reads.
    fn machine_config(&self) -> &variegated_controller_types::MachineConfiguration;

    /// Whether a routine is executing.
    fn routine_running(&self) -> bool;

    /// Should a new water operation be refused?
    ///
    /// The decision is [`super::interlocks::should_block_water_operation`], which is pure and
    /// host-tested; this only reads the tank for it.
    fn should_block_water_operation(&mut self) -> bool {
        let tank_empty = self.is_tank_empty();
        let routine_running = self.routine_running();
        super::interlocks::should_block_water_operation(
            self.machine_config(),
            tank_empty,
            routine_running,
        )
    }

    // ---- Manual shot logging ----------------------------------------------------------

    /// The shot logger, the pending annotations and the manual-shot flag, together.
    ///
    /// One accessor because closing a manual log touches all three at once.
    fn shot_logging(
        &mut self,
    ) -> (&mut crate::ShotLogger, &mut ShotAnnotations, &mut bool);

    /// Where a finished shot is handed for storage, if this machine has somewhere to put it.
    fn shot_log_sender(
        &self,
    ) -> Option<Sender<'a, Self::ChannelM, variegated_controller_types::ShotLog, 2>>;

    /// Open a log for a shot the user started by hand.
    fn start_manual_shot_log(&mut self) {
        let routine_running = self.routine_running();
        let group_index = Self::GROUP_INDEX;
        let (logger, pending, manual_active) = self.shot_logging();
        shot::start_manual_shot_log(logger, pending, manual_active, routine_running, group_index);
    }

    /// Close it and hand it to storage.
    fn finish_manual_shot_log(&mut self) {
        let sender = self.shot_log_sender();
        let (logger, pending, manual_active) = self.shot_logging();
        shot::finish_manual_shot_log(logger, pending, manual_active, sender.as_ref());
    }

    /// Close a *routine's* log and clear what it carried.
    fn finish_routine_shot_log(&mut self) {
        let sender = self.shot_log_sender();
        let (logger, pending, _) = self.shot_logging();
        shot::finish_routine_shot_log(logger, pending, sender.as_ref());
    }

    /// Whether the machine is doing something a radio-heavy operation must not interrupt.
    ///
    /// **The one predicate the two machines genuinely disagree about.** One reads three flags
    /// and a steam wand; the other matches on a state enum. Both a discovery scan and a
    /// provisioning window are refused on it, for the same reason: minutes of connectable
    /// advertising share one antenna with Wi-Fi and with the live links to the scales, and
    /// this is the only processor that knows coffee is being made.
    fn is_busy(&self) -> bool;

    // ---- Machine-specific behaviour --------------------------------------------------

    /// `StartBrewing`. Interlocks are this machine's business.
    async fn start_brewing(&mut self);
    /// `StopBrewing`.
    async fn stop_brewing(&mut self);
    /// `StartPumpingToWaterTap`.
    async fn start_water_tap(&mut self);
    /// `StopPumpingToWaterTap`.
    async fn stop_water_tap(&mut self);

    /// `StartSteaming`. A machine with no steam wand refuses.
    async fn start_steaming(&mut self, index: SteamWandIndex);
    /// `StopSteaming`.
    async fn stop_steaming(&mut self, index: SteamWandIndex);
    /// `SetSteamValveOpenness`.
    async fn set_steam_valve_openness(&mut self, index: SteamWandIndex, openness: ValveOpenType);

    /// `EnableBoiler` and `DisableBoiler`. A mode table on one machine, two flags on the other.
    async fn set_boiler_enabled(&mut self, enable: bool, index: BoilerIndex);

    /// `SetMachineMode`, including whatever has to stop when the machine goes off.
    async fn set_machine_mode(&mut self, mode: MachineMode);

    /// This machine's brew group.
    ///
    /// Both machines have exactly one, which is what lets the group commands below be shared
    /// rather than reimplemented per controller.
    type Group: GroupAccess;

    /// Access it.
    fn group(&mut self) -> &mut Self::Group;

    /// The index that names this machine's only group.
    ///
    /// A constant rather than a hard-coded `0` in the shared handlers, so a machine that grew a
    /// second group would have somewhere to say so rather than silently answering for the
    /// wrong one.
    const GROUP_INDEX: GroupIndex = 0;

    /// The three scale commands.
    ///
    /// The `Result` is discarded as it always was: a scale that is not attached, or that
    /// refuses, is a condition the user can see on the display rather than something this loop
    /// can act on.
    async fn scale_action(&mut self, index: GroupIndex, action: ScaleAction) {
        if index != Self::GROUP_INDEX {
            log_error!("Invalid group index for {} scale: {}", action.label(), index);
            return;
        }
        log_info!("{} group scale", action.label());
        match action {
            ScaleAction::Tare => self.group().tare_scale().await,
            ScaleAction::ZeroCalibrate => self.group().zero_calibrate_scale().await,
            ScaleAction::CalibrateWith100g => self.group().calibrate_scale_with_100g().await,
        }
    }

    /// The three `InferGroup*Integral` commands.
    ///
    /// The index check is here; the seeding itself is
    /// [`crate::command::pump::PumpLoopContext::seed_pump_integral`], which is where the two
    /// machines' one genuine disagreement about it lives.
    fn seed_pump_integral(&mut self, index: GroupIndex, quantity: PumpQuantity, target: f32) {
        if index != Self::GROUP_INDEX {
            log_error!("Invalid group index: {}", index);
            return;
        }
        crate::command::pump::PumpLoopContext::seed_integral(self, quantity, target);
    }

    /// `OptimizeConfigurationStorage`. Run inline where the store is owned, queued where it is
    /// not.
    async fn optimize_configuration_storage(&mut self);

    /// `RequestConfiguration`.
    async fn request_configuration(&mut self);

    /// `TagDoseFromScale` -- read the scale now and record it as the next shot's dose.
    fn tag_dose_from_scale(&mut self, scale: ScaleSelector) {
        let group_index = Self::GROUP_INDEX;
        let weight = self.group().output_weight();
        shot::tag_dose_from_scale(self.pending_annotations_mut(), scale, group_index, weight);
    }

    /// `IdentifyMachine`. What identifying means is the machine's to decide, and a machine
    /// with nothing to flash may do nothing at all.
    fn identify(&mut self);

    /// `SetGroupPumpConfiguration`.
    async fn set_group_pump_configuration(&mut self, index: GroupIndex, config: PumpConfiguration);
    /// `SetWaterTapPumpConfiguration`.
    async fn set_water_tap_pump_configuration(&mut self, index: WaterTapIndex, config: PumpConfiguration);
    /// `SetFillPumpConfiguration`.
    async fn set_fill_pump_configuration(&mut self, index: BoilerIndex, config: PumpConfiguration);
    /// `SetHeatingElementInterlock`. Meaningless with one element.
    async fn set_heating_element_interlock(&mut self, enabled: bool);
    /// `SetHeatingElementContentionStrategy`. Likewise.
    async fn set_heating_element_contention_strategy(&mut self, strategy: HeatingElementContentionStrategy);
    /// `SetWaterDispersalPumpStrategy`.
    async fn set_water_dispersal_pump_strategy(&mut self, index: WaterTapIndex, strategy: WaterDispersalPumpStrategy);

    // ---- The dispatcher ---------------------------------------------------------------

    /// Carry out a command.
    ///
    /// **The only `match` over `MachineCommand` in this crate, and exhaustive.** A variant
    /// added to the enum fails to compile here; if the machines handle it differently, the
    /// required method it needs then fails to compile in both implementations.
    ///
    /// Routine lifecycle -- `RunRoutine` and `CancelRoutine` -- is deliberately *not* here.
    /// Those are handled by the controllers before this is reached, because a routine's own
    /// `finally` block dispatches through this function and must not be able to start another
    /// routine.
    async fn handle_machine_command(&mut self, command: MachineCommand) {
        match command {
            // ---- Brewing and dispensing ----
            MachineCommand::StartBrewing(_) => self.start_brewing().await,
            MachineCommand::StopBrewing(_) => self.stop_brewing().await,
            MachineCommand::StartPumpingToWaterTap(_) => self.start_water_tap().await,
            MachineCommand::StopPumpingToWaterTap(_) => self.stop_water_tap().await,
            MachineCommand::StartSteaming(index) => self.start_steaming(index).await,
            MachineCommand::StopSteaming(index) => self.stop_steaming(index).await,
            MachineCommand::SetSteamValveOpenness(index, openness) => {
                self.set_steam_valve_openness(index, openness).await
            }

            // ---- Targets: `super::targets`, host-tested against both configurations ----
            MachineCommand::SetBoilerControlTarget(index, mode, update) => {
                let outcome = super::targets::set_boiler_control_target(
                    self.configuration_mut(), index, mode, update);
                self.apply_target_outcome(outcome).await;
            }
            MachineCommand::SetBoilerControlTargetValues(index, update) => {
                let outcome = super::targets::set_boiler_control_target_values(
                    self.configuration_mut(), index, update);
                self.apply_target_outcome(outcome).await;
            }
            MachineCommand::SetGroupBrewControlTarget(index, mode, update) => {
                let outcome = super::targets::set_group_brew_control_target(
                    self.configuration_mut(), index, mode, update);
                self.apply_target_outcome(outcome).await;
            }
            MachineCommand::SetGroupBrewControlTargetValues(index, update) => {
                let outcome = super::targets::set_group_brew_control_target_values(
                    self.configuration_mut(), index, update);
                self.apply_target_outcome(outcome).await;
            }
            MachineCommand::SetGroupBrewLimit(index, limit, update) => {
                let outcome = super::targets::set_group_brew_limit(
                    self.configuration_mut(), index, limit, update);
                self.apply_target_outcome(outcome).await;
            }
            MachineCommand::SetPidParameters(target, params) => {
                let outcome = super::targets::set_pid_parameters(
                    self.configuration_mut(), target, params);
                self.apply_target_outcome(outcome).await;
            }

            // ---- Boilers, mode, scale ----
            MachineCommand::EnableBoiler(index) => self.set_boiler_enabled(true, index).await,
            MachineCommand::DisableBoiler(index) => self.set_boiler_enabled(false, index).await,
            MachineCommand::SetMachineMode(mode) => self.set_machine_mode(mode).await,
            MachineCommand::TareGroupScale(index) => {
                self.scale_action(index, ScaleAction::Tare).await
            }
            MachineCommand::ZeroCalibrateGroupScale(index) => {
                self.scale_action(index, ScaleAction::ZeroCalibrate).await
            }
            MachineCommand::CalibrateGroupScale100g(index) => {
                self.scale_action(index, ScaleAction::CalibrateWith100g).await
            }
            MachineCommand::InferGroupPressureIntegral(index, target) => {
                self.seed_pump_integral(index, PumpQuantity::Pressure, target as f32)
            }
            MachineCommand::InferGroupFlowRateIntegral(index, target) => {
                self.seed_pump_integral(index, PumpQuantity::GroupFlowRate, target as f32)
            }
            MachineCommand::InferGroupOutputFlowRateIntegral(index, target) => {
                self.seed_pump_integral(index, PumpQuantity::OutputFlowRate, target as f32)
            }

            // ---- Routines and schedules: `super::stores` ----
            MachineCommand::AddRoutine(routine) => {
                stores::add_routine(self.routine_repository(), routine).await
            }
            MachineCommand::RemoveRoutine(index) => {
                stores::remove_routine(self.routine_repository(), index).await
            }
            MachineCommand::UpdateRoutine(index, routine) => {
                stores::update_routine(self.routine_repository(), index, routine).await
            }
            MachineCommand::AddScheduleItem(item) => {
                let publish = stores::add_schedule(self.schedule_store(), item).await;
                self.note_configuration_publish(publish);
            }
            MachineCommand::RemoveScheduleItem(index) => {
                let publish = stores::remove_schedule(self.schedule_store(), index).await;
                self.note_configuration_publish(publish);
            }
            MachineCommand::UpdateScheduleItem(index, item) => {
                let publish = stores::update_schedule(self.schedule_store(), index, item).await;
                self.note_configuration_publish(publish);
            }

            // ---- Storage optimization ----
            //
            // Both range-rewriting optimizations are queued rather than run here: on a
            // flash-backed store each erases a whole range and rewrites it, and this is the
            // loop that runs the PID and the interlocks. `try_send` on a depth-4 channel,
            // because a second request is the same request and a full channel must not park
            // the control loop.
            //
            // `OptimizeConfigurationStorage` is the machine's own, because one owns its
            // configuration store by value and the other reaches it through a mutex.
            MachineCommand::OptimizeRoutineStorage => {
                queue_storage_command(self.storage_command_sender(), StorageCommand::OptimizeRoutines)
            }
            MachineCommand::OptimizeScheduleStorage => {
                queue_storage_command(self.storage_command_sender(), StorageCommand::OptimizeSchedules)
            }
            MachineCommand::OptimizeConfigurationStorage => {
                self.optimize_configuration_storage().await
            }

            // ---- Bluetooth: `super::bluetooth` ----
            MachineCommand::AssociateBluetoothPeripheral(association) => {
                let (associations, _, _) = self.bluetooth();
                if bluetooth::associate(associations, association).wanted() {
                    self.save_bluetooth_associations().await;
                }
            }
            MachineCommand::RemoveBluetoothPeripheral(id) => {
                let (associations, _, _) = self.bluetooth();
                if bluetooth::remove(associations, id).wanted() {
                    self.save_bluetooth_associations().await;
                }
            }
            MachineCommand::SetBluetoothPeripheralEnabled(id, enabled) => {
                let (associations, _, _) = self.bluetooth();
                if bluetooth::set_enabled(associations, id, enabled).wanted() {
                    self.save_bluetooth_associations().await;
                }
            }
            MachineCommand::ScanForBluetoothPeripherals => {
                let busy = self.is_busy();
                let sender = self.bluetooth_scan_sender();
                let (_, status, deadline) = self.bluetooth();
                bluetooth::start_scan(status, deadline, sender, busy);
            }
            MachineCommand::UpdateBluetoothScan(update) => {
                let (_, status, deadline) = self.bluetooth();
                bluetooth::apply_scan_update(status, deadline, update);
            }

            // ---- Connectivity: `super::connectivity` ----
            MachineCommand::SetWifiCredentials(credentials) => {
                if connectivity::set_wifi_credentials(self.wifi_credentials_mut(), credentials)
                    .wanted()
                {
                    self.save_wifi_credentials().await;
                }
            }
            MachineCommand::SetShotUploadSettings(settings) => {
                if connectivity::apply_shot_upload_settings(
                    self.shot_upload_config_mut(), settings).wanted()
                {
                    self.save_shot_upload_config().await;
                }
            }
            MachineCommand::SetShotUploadConfig(config) => {
                if connectivity::set_shot_upload_config(self.shot_upload_config_mut(), config)
                    .wanted()
                {
                    self.save_shot_upload_config().await;
                }
            }
            MachineCommand::SetTimezone(setting) => {
                if connectivity::set_timezone(self.timezone_mut(), setting).wanted() {
                    self.save_timezone().await;
                }
            }
            MachineCommand::OpenWifiProvisioningWindow { duration_ms } => {
                let busy = self.is_busy();
                connectivity::open_provisioning_window(
                    self.wifi_provisioning_sender(), duration_ms, busy);
            }
            MachineCommand::CloseWifiProvisioningWindow => {
                connectivity::close_provisioning_window(self.wifi_provisioning_sender())
            }

            // ---- Shot log: `super::shot` ----
            MachineCommand::SetPendingShotAnnotations(annotations) => {
                shot::set_pending_annotations(self.pending_annotations_mut(), annotations)
            }
            MachineCommand::SetShotAnnotations(id, annotations) => {
                shot::set_shot_annotations(self.shot_log_query_sender().as_ref(), id, annotations)
            }
            MachineCommand::DeleteShotLog(id) => {
                shot::delete_shot_log(self.shot_log_query_sender().as_ref(), id)
            }
            MachineCommand::TagDoseFromScale(scale) => self.tag_dose_from_scale(scale),

            // ---- Pump and element configuration ----
            MachineCommand::SetGroupPumpConfiguration(index, config) => {
                self.set_group_pump_configuration(index, config).await
            }
            MachineCommand::SetWaterTapPumpConfiguration(index, config) => {
                self.set_water_tap_pump_configuration(index, config).await
            }
            MachineCommand::SetFillPumpConfiguration(index, config) => {
                self.set_fill_pump_configuration(index, config).await
            }
            MachineCommand::SetHeatingElementInterlock(enabled) => {
                self.set_heating_element_interlock(enabled).await
            }
            MachineCommand::SetHeatingElementContentionStrategy(strategy) => {
                self.set_heating_element_contention_strategy(strategy).await
            }
            MachineCommand::SetWaterDispersalPumpStrategy(index, strategy) => {
                self.set_water_dispersal_pump_strategy(index, strategy).await
            }

            // ---- The rest ----
            MachineCommand::UpdateCommsStatus(status) => self.set_comms_status(status),
            MachineCommand::RequestConfiguration => self.request_configuration().await,
            MachineCommand::IdentifyMachine => self.identify(),

            // Handled by the controllers before this is reached; see the note on this method.
            // Reaching here means a routine's `finally` block asked to start or stop a
            // routine, which it must not be able to do.
            MachineCommand::RunRoutine(_, _) | MachineCommand::CancelRoutine => {
                log_warn!(
                    "Ignoring routine lifecycle command in finally block: {}",
                    command.label()
                );
            }
        }
    }
}

/// Queue a storage optimization, or say why it was dropped.
fn queue_storage_command<M: RawMutex>(
    sender: Sender<'_, M, StorageCommand, 4>,
    command: StorageCommand,
) {
    if sender.try_send(command).is_err() {
        log_warn!("Storage command channel full, dropping {:?}", command);
    }
}
