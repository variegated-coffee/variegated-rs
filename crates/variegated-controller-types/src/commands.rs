use crate::*;

/// Commands for storage operations that may take a long time
/// These are handled by a separate task to avoid blocking the main control loop
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug)]
pub enum StorageCommand {
    OptimizeRoutines,
    OptimizeSchedules,
    OptimizeConfiguration,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone)]
pub enum MachineCommand {
    StartBrewing(GroupIndex),
    StopBrewing(GroupIndex),
    StartPumpingToWaterTap(WaterTapIndex),
    StopPumpingToWaterTap(WaterTapIndex),
    StartSteaming(SteamWandIndex),
    StopSteaming(SteamWandIndex),
    SetSteamValveOpenness(SteamWandIndex, ValveOpenType),

    /// Set boiler control mode, optionally updating target values
    /// Examples:
    /// - SetBoilerControlTarget(0, Temperature, None) - Switch to temp mode with remembered target
    /// - SetBoilerControlTarget(0, Temperature, Some({target_temperature: Some(95.0), ..})) - Switch to temp mode at 95°C
    /// - SetBoilerControlTarget(0, Off, None) - Turn off, preserving all target values
    SetBoilerControlTarget(BoilerIndex, BoilerControlMode, Option<BoilerControlTargetValuesUpdate>),

    /// Update boiler target values without changing mode
    /// Examples:
    /// - SetBoilerControlTargetValues(0, {target_temperature: Some(95.0), ..}) - Change temp to 95°C
    /// - SetBoilerControlTargetValues(0, {target_pressure: Some(9.0), ..}) - Change pressure to 9 bar
    SetBoilerControlTargetValues(BoilerIndex, BoilerControlTargetValuesUpdate),

    /// Set group brew control mode, optionally updating target values
    /// Examples:
    /// - SetGroupBrewControlTarget(0, Pressure, None) - Switch to pressure mode with remembered target
    /// - SetGroupBrewControlTarget(0, FixedDutyCycle, Some({duty_cycle: Some(60), ..})) - 60% duty cycle
    SetGroupBrewControlTarget(GroupIndex, GroupBrewControlMode, Option<GroupBrewControlTargetValuesUpdate>),

    /// Update group brew target values without changing mode
    /// Example:
    /// - SetGroupBrewControlTargetValues(0, {pressure: Some(9.0), ..}) - Adjust pressure to 9 bar
    SetGroupBrewControlTargetValues(GroupIndex, GroupBrewControlTargetValuesUpdate),

    SetPidParameters(PidParameterTarget, PidParameters),
    RunRoutine(RoutineIndex, Option<RoutineParameters>),
    CancelRoutine,
    EnableBoiler(BoilerIndex),
    DisableBoiler(BoilerIndex),
    TareGroupScale(GroupIndex),
    ZeroCalibrateGroupScale(GroupIndex),
    CalibrateGroupScale100g(GroupIndex),
    UpdateCommsStatus(CommsStatus),
    AddScheduleItem(ScheduleItem),
    RemoveScheduleItem(u32),
    UpdateScheduleItem(u32, ScheduleItem),
    AddRoutine(Routine),
    RemoveRoutine(RoutineIndex),
    UpdateRoutine(RoutineIndex, Routine),
    SetMachineMode(MachineMode),
    OptimizeConfigurationStorage,
    OptimizeRoutineStorage,
    OptimizeScheduleStorage,
    SetGroupPumpConfiguration(GroupIndex, PumpConfiguration),
    SetWaterTapPumpConfiguration(WaterTapIndex, PumpConfiguration),
    SetFillPumpConfiguration(BoilerIndex, PumpConfiguration),

    /// Infer and set the group pressure PID integral term for bumpless transfer
    /// Takes target pressure and calculates integral based on current duty cycle and measurement
    InferGroupPressureIntegral(GroupIndex, PressureType),

    /// Infer and set the group flow rate PID integral term for bumpless transfer
    /// Takes target flow rate and calculates integral based on current duty cycle and measurement
    InferGroupFlowRateIntegral(GroupIndex, FlowRateType),

    /// Infer and set the group output flow rate PID integral term for bumpless transfer
    /// Takes target output flow rate and calculates integral based on current duty cycle and measurement
    InferGroupOutputFlowRateIntegral(GroupIndex, FlowRateType),

    /// Enable or disable heating element interlock (prevents simultaneous heating)
    SetHeatingElementInterlock(bool),

    /// Set the strategy for resolving heating element contention when demand exceeds capacity
    SetHeatingElementContentionStrategy(HeatingElementContentionStrategy),

    /// Set the water dispersal pump strategy for a specific water tap
    SetWaterDispersalPumpStrategy(WaterTapIndex, WaterDispersalPumpStrategy),

    /// Bind a Bluetooth address and driver to a peripheral role.
    ///
    /// An upsert, keyed on the association's [`PeripheralId`]. A role cannot be filled
    /// twice, so associating a device with a peripheral that already has one replaces
    /// it rather than failing or duplicating.
    AssociateBluetoothPeripheral(crate::bluetooth::BluetoothPeripheralAssociation),

    /// Forget the association for a peripheral role.
    RemoveBluetoothPeripheral(PeripheralId),

    /// Stop or resume connecting to an associated peripheral, keeping the association.
    ///
    /// Separate from [`Self::RemoveBluetoothPeripheral`] so that switching a scale off
    /// for a while does not cost the user a discovery scan and a re-pairing to get it
    /// back.
    SetBluetoothPeripheralEnabled(PeripheralId, bool),

    /// Run a Bluetooth discovery scan.
    ///
    /// May be refused: see
    /// [`crate::bluetooth::BluetoothScanStatus::blocked`]. The duration is not a
    /// parameter because it is a radio-coexistence decision rather than a user
    /// preference.
    ScanForBluetoothPeripherals,

    /// Progress of a discovery scan, relayed from the comms processor.
    ///
    /// Not a user command, despite sitting here. The comms processor's only route into
    /// the controller is this channel, and the controller is what assembles `Status` --
    /// the same reasoning that puts [`Self::UpdateCommsStatus`] in this enum.
    UpdateBluetoothScan(crate::bluetooth::BluetoothScanUpdate),

    /// Replace the annotations on a shot already stored on the card.
    ///
    /// Appended, not inserted -- see the note on
    /// [`crate::CommsProcessorToApplicationProcessorMessage::DebugCommand`]; the same
    /// discriminant rule governs this enum.
    ///
    /// A whole block rather than a single key, because the on-card record is rewritten
    /// entirely for any edit: a per-key command would cost one whole-file rewrite per
    /// field, and a user correcting three fields would pay three.
    SetShotAnnotations(crate::shot_log::ShotLogId, crate::shot_log::ShotAnnotations),

    /// Replace the annotations that will be stamped onto the next shot.
    ///
    /// Appended, not inserted.
    ///
    /// Separate from [`Self::SetShotAnnotations`] rather than an `Option<ShotLogId>` on
    /// it: the two touch different things -- one rewrites a file on the card, the other
    /// writes a field in RAM -- and they fail in different ways. Sharing a variant would
    /// let a client that meant "next shot" address a stored one by supplying an id it
    /// thought was ignored.
    SetPendingShotAnnotations(crate::shot_log::ShotAnnotations),

    /// Read a scale's current weight and record it as the dose for the next shot.
    ///
    /// Appended, not inserted.
    ///
    /// Takes a [`ScaleSelector`] because a machine can carry several scales and "the
    /// scale" is not well defined. Refused rather than guessed if that scale has no
    /// weight to report: a dose silently recorded from the wrong place is worse than no
    /// dose at all, because nothing downstream can tell it was wrong.
    TagDoseFromScale(ScaleSelector),

    /// Open the Improv provisioning window for `duration_ms`.
    ///
    /// Appended, not inserted.
    ///
    /// Refused while the machine is busy, on the same grounds as a discovery scan: minutes
    /// of connectable advertising share one antenna with Wi-Fi and with the live links to
    /// the scales. This is the only processor that knows coffee is being made.
    OpenWifiProvisioningWindow { duration_ms: u32 },

    /// Close the Improv provisioning window.
    ///
    /// Appended, not inserted.
    CloseWifiProvisioningWindow,

    /// Persist credentials the comms processor has already proven work.
    ///
    /// Appended, not inserted.
    ///
    /// Not a user command despite sitting here, like [`Self::UpdateCommsStatus`]: it
    /// arrives from the comms processor, whose only route into the controller is the
    /// command channel. It carries no validation burden because the sender has already
    /// associated with these credentials -- Improv requires the device to verify before
    /// reporting success, so a typo never reaches this point.
    SetWifiCredentials(crate::wifi::WifiCredentials),

    /// Make the machine identify itself -- Improv's Identify RPC.
    ///
    /// Appended, not inserted.
    ///
    /// What identifying means is the machine's to decide: flash the display, blink an LED,
    /// or on a machine with neither, nothing at all.
    IdentifyMachine,

    /// Republish the current [`crate::Configuration`], unchanged or not.
    ///
    /// Appended, not inserted.
    ///
    /// Not a user command, like [`Self::UpdateCommsStatus`]: it exists because
    /// `Configuration` is the one payload on the link that is *published once* and read
    /// only through a pub-sub channel, and embassy-sync's channels have no retained
    /// value. A subscriber created after the publish never sees it -- `subscriber()`
    /// starts a reader at the current message id -- so a consumer that comes up late
    /// (the comms processor's HTTP cache waits on Wi-Fi association and DHCP, tens of
    /// seconds after boot) is left with nothing and no way to ask.
    ///
    /// Every other payload survives that race by accident: `MachineDefinition` and
    /// `RoutineSummaries` are written into caches directly by their reader arms, and
    /// `Status` is republished at 1 Hz forever. This is the missing "ask again" for the
    /// one that is not.
    ///
    /// Distinct from
    /// [`crate::CommsProcessorToApplicationProcessorMessage::RequestConfiguration`],
    /// which the *transceiver* answers out of its own cache of the last configuration it
    /// forwarded. That reply is only as good as the cache, and the cache is empty until a
    /// publish has been seen -- so it cannot recover a boot where the first publish went
    /// nowhere. This one reaches the controller, which always has the real value.
    RequestConfiguration,

    /// Delete a shot from the card.
    ///
    /// Appended, not inserted.
    ///
    /// A `MachineCommand` rather than a shot-log query, matching
    /// [`Self::SetShotAnnotations`]: the controller is the single interpreter of
    /// commands, and routing a write around it would give the same operation two
    /// different behaviours depending on whether it arrived over HTTP or over the debug
    /// link.
    ///
    /// **Fire and forget, and irreversible.** Nothing acknowledges it. Success is
    /// reported by a `ShotLogEvent::Deleted` push; a failure is logged on the application
    /// processor and the shot simply stays where it was.
    DeleteShotLog(crate::shot_log::ShotLogId),
}

impl MachineCommand {
    /// The variant's name, and nothing else.
    ///
    /// Mirrors [`crate::debug_command::DebugCommand::label`], and exists for the same
    /// reason: a `&'static str` can go through *both* halves of `variegated_log`'s
    /// `log_*!` macros, where the hand-written `defmt::Format` below reaches only a probe.
    /// A controller that drops a command it does not implement can therefore say which one
    /// on the debug bus, which is where anyone diagnosing the machine is actually looking.
    ///
    /// Deliberately no payload. `Format` already renders those where it is useful, and
    /// several variants carry a whole `Routine` -- which is why `AddRoutine` prints as
    /// `AddRoutine()` there.
    ///
    /// Exhaustive rather than defaulted: adding a variant should be a compile error here,
    /// not a command that silently reports as "unknown".
    pub fn label(&self) -> &'static str {
        match self {
            MachineCommand::StartBrewing(_) => "StartBrewing",
            MachineCommand::StopBrewing(_) => "StopBrewing",
            MachineCommand::StartSteaming(_) => "StartSteaming",
            MachineCommand::StopSteaming(_) => "StopSteaming",
            MachineCommand::StartPumpingToWaterTap(_) => "StartPumpingToWaterTap",
            MachineCommand::StopPumpingToWaterTap(_) => "StopPumpingToWaterTap",
            MachineCommand::SetSteamValveOpenness(_, _) => "SetSteamValveOpenness",
            MachineCommand::SetBoilerControlTarget(_, _, _) => "SetBoilerControlTarget",
            MachineCommand::SetBoilerControlTargetValues(_, _) => "SetBoilerControlTargetValues",
            MachineCommand::SetGroupBrewControlTarget(_, _, _) => "SetGroupBrewControlTarget",
            MachineCommand::SetGroupBrewControlTargetValues(_, _) => "SetGroupBrewControlTargetValues",
            MachineCommand::SetPidParameters(_, _) => "SetPidParameters",
            MachineCommand::RunRoutine(_, _) => "RunRoutine",
            MachineCommand::CancelRoutine => "CancelRoutine",
            MachineCommand::EnableBoiler(_) => "EnableBoiler",
            MachineCommand::DisableBoiler(_) => "DisableBoiler",
            MachineCommand::TareGroupScale(_) => "TareGroupScale",
            MachineCommand::ZeroCalibrateGroupScale(_) => "ZeroCalibrateGroupScale",
            MachineCommand::CalibrateGroupScale100g(_) => "CalibrateGroupScale100g",
            MachineCommand::UpdateCommsStatus(_) => "UpdateCommsStatus",
            MachineCommand::AddScheduleItem(_) => "AddScheduleItem",
            MachineCommand::RemoveScheduleItem(_) => "RemoveScheduleItem",
            MachineCommand::UpdateScheduleItem(_, _) => "UpdateScheduleItem",
            MachineCommand::AddRoutine(_) => "AddRoutine",
            MachineCommand::RemoveRoutine(_) => "RemoveRoutine",
            MachineCommand::UpdateRoutine(_, _) => "UpdateRoutine",
            MachineCommand::SetMachineMode(_) => "SetMachineMode",
            MachineCommand::OptimizeConfigurationStorage => "OptimizeConfigurationStorage",
            MachineCommand::OptimizeRoutineStorage => "OptimizeRoutineStorage",
            MachineCommand::OptimizeScheduleStorage => "OptimizeScheduleStorage",
            MachineCommand::SetGroupPumpConfiguration(_, _) => "SetGroupPumpConfiguration",
            MachineCommand::SetWaterTapPumpConfiguration(_, _) => "SetWaterTapPumpConfiguration",
            MachineCommand::SetFillPumpConfiguration(_, _) => "SetFillPumpConfiguration",
            MachineCommand::InferGroupPressureIntegral(_, _) => "InferGroupPressureIntegral",
            MachineCommand::InferGroupFlowRateIntegral(_, _) => "InferGroupFlowRateIntegral",
            MachineCommand::InferGroupOutputFlowRateIntegral(_, _) => "InferGroupOutputFlowRateIntegral",
            MachineCommand::SetHeatingElementInterlock(_) => "SetHeatingElementInterlock",
            MachineCommand::SetHeatingElementContentionStrategy(_) => "SetHeatingElementContentionStrategy",
            MachineCommand::SetWaterDispersalPumpStrategy(_, _) => "SetWaterDispersalPumpStrategy",
            MachineCommand::AssociateBluetoothPeripheral(_) => "AssociateBluetoothPeripheral",
            MachineCommand::RemoveBluetoothPeripheral(_) => "RemoveBluetoothPeripheral",
            MachineCommand::SetBluetoothPeripheralEnabled(_, _) => "SetBluetoothPeripheralEnabled",
            MachineCommand::ScanForBluetoothPeripherals => "ScanForBluetoothPeripherals",
            MachineCommand::UpdateBluetoothScan(_) => "UpdateBluetoothScan",
            MachineCommand::SetWifiCredentials(_) => "SetWifiCredentials",
            MachineCommand::OpenWifiProvisioningWindow { .. } => "OpenWifiProvisioningWindow",
            MachineCommand::CloseWifiProvisioningWindow => "CloseWifiProvisioningWindow",
            MachineCommand::IdentifyMachine => "IdentifyMachine",
            MachineCommand::RequestConfiguration => "RequestConfiguration",
            MachineCommand::SetShotAnnotations(_, _) => "SetShotAnnotations",
            MachineCommand::SetPendingShotAnnotations(_) => "SetPendingShotAnnotations",
            MachineCommand::TagDoseFromScale(_) => "TagDoseFromScale",
            MachineCommand::DeleteShotLog(_) => "DeleteShotLog",
        }
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for MachineCommand {
    fn format(&self, f: defmt::Formatter) {
        match self {
            MachineCommand::StartBrewing(idx) => defmt::write!(f, "StartBrewing({})", idx),
            MachineCommand::StopBrewing(idx) => defmt::write!(f, "StopBrewing({})", idx),
            MachineCommand::StartPumpingToWaterTap(idx) => defmt::write!(f, "StartPumpingToWaterTap({})", idx),
            MachineCommand::StopPumpingToWaterTap(idx) => defmt::write!(f, "StopPumpingToWaterTap({})", idx),
            MachineCommand::StartSteaming(idx) => defmt::write!(f, "StartSteaming({})", idx),
            MachineCommand::StopSteaming(idx) => defmt::write!(f, "StopSteaming({})", idx),
            MachineCommand::SetSteamValveOpenness(idx, openness) => defmt::write!(f, "SetSteamValveOpenness({}, {})", idx, openness),
            MachineCommand::SetBoilerControlTarget(idx, mode, values) => defmt::write!(f, "SetBoilerControlTarget({}, {:?}, {:?})", idx, mode, values),
            MachineCommand::SetBoilerControlTargetValues(idx, values) => defmt::write!(f, "SetBoilerControlTargetValues({}, {:?})", idx, values),
            MachineCommand::SetGroupBrewControlTarget(idx, mode, values) => defmt::write!(f, "SetGroupBrewControlTarget({}, {:?}, {:?})", idx, mode, values),
            MachineCommand::SetGroupBrewControlTargetValues(idx, values) => defmt::write!(f, "SetGroupBrewControlTargetValues({}, {:?})", idx, values),
            MachineCommand::SetPidParameters(target, params) => defmt::write!(f, "SetPidParameters({:?}, {:?})", target, params),
            MachineCommand::RunRoutine(idx, params) => defmt::write!(f, "RunRoutine({}, {} params)", idx, params.as_ref().map(|p| p.len()).unwrap_or(0)),
            MachineCommand::CancelRoutine => defmt::write!(f, "CancelRoutine"),
            MachineCommand::EnableBoiler(idx) => defmt::write!(f, "EnableBoiler({})", idx),
            MachineCommand::DisableBoiler(idx) => defmt::write!(f, "DisableBoiler({})", idx),
            MachineCommand::TareGroupScale(idx) => defmt::write!(f, "TareGroupScale({})", idx),
            MachineCommand::ZeroCalibrateGroupScale(idx) => defmt::write!(f, "ZeroCalibrateGroupScale({})", idx),
            MachineCommand::CalibrateGroupScale100g(idx) => defmt::write!(f, "CalibrateGroupScale100g({})", idx),
            MachineCommand::UpdateCommsStatus(status) => defmt::write!(f, "UpdateCommsStatus({:?})", status),
            MachineCommand::AddScheduleItem(item) => defmt::write!(f, "AddScheduleItem({})", item),
            MachineCommand::RemoveScheduleItem(idx) => defmt::write!(f, "RemoveScheduleItem({})", idx),
            MachineCommand::UpdateScheduleItem(idx, item) => defmt::write!(f, "UpdateScheduleItem({}, {})", idx, item),
            MachineCommand::AddRoutine(_routine) => defmt::write!(f, "AddRoutine()"),
            MachineCommand::RemoveRoutine(idx) => defmt::write!(f, "RemoveRoutine({})", idx),
            MachineCommand::UpdateRoutine(idx, _routine) => defmt::write!(f, "UpdateRoutine({})", idx),
            MachineCommand::SetMachineMode(mode) => defmt::write!(f, "SetMachineMode({:?})", mode),
            MachineCommand::OptimizeConfigurationStorage => defmt::write!(f, "OptimizeConfigurationStorage"),
            MachineCommand::OptimizeRoutineStorage => defmt::write!(f, "OptimizeRoutineStorage"),
            MachineCommand::OptimizeScheduleStorage => defmt::write!(f, "OptimizeScheduleStorage"),
            MachineCommand::SetGroupPumpConfiguration(idx, config) => defmt::write!(f, "SetGroupPumpConfiguration({}, {:?})", idx, config),
            MachineCommand::SetWaterTapPumpConfiguration(idx, config) => defmt::write!(f, "SetWaterTapPumpConfiguration({}, {:?})", idx, config),
            MachineCommand::SetFillPumpConfiguration(idx, config) => defmt::write!(f, "SetFillPumpConfiguration({}, {:?})", idx, config),
            MachineCommand::InferGroupPressureIntegral(idx, pressure) => defmt::write!(f, "InferGroupPressureIntegral({}, {})", idx, pressure),
            MachineCommand::InferGroupFlowRateIntegral(idx, flow_rate) => defmt::write!(f, "InferGroupFlowRateIntegral({}, {})", idx, flow_rate),
            MachineCommand::InferGroupOutputFlowRateIntegral(idx, flow_rate) => defmt::write!(f, "InferGroupOutputFlowRateIntegral({}, {})", idx, flow_rate),
            MachineCommand::SetHeatingElementInterlock(enabled) => defmt::write!(f, "SetHeatingElementInterlock({})", enabled),
            MachineCommand::SetHeatingElementContentionStrategy(strategy) => defmt::write!(f, "SetHeatingElementContentionStrategy({:?})", strategy),
            MachineCommand::SetWaterDispersalPumpStrategy(idx, strategy) => defmt::write!(f, "SetWaterDispersalPumpStrategy({}, {:?})", idx, strategy),
            MachineCommand::AssociateBluetoothPeripheral(a) => defmt::write!(f, "AssociateBluetoothPeripheral(0x{:04X}, {:?})", a.id, a.driver),
            MachineCommand::RemoveBluetoothPeripheral(id) => defmt::write!(f, "RemoveBluetoothPeripheral(0x{:04X})", id),
            MachineCommand::SetBluetoothPeripheralEnabled(id, enabled) => defmt::write!(f, "SetBluetoothPeripheralEnabled(0x{:04X}, {})", id, enabled),
            MachineCommand::ScanForBluetoothPeripherals => defmt::write!(f, "ScanForBluetoothPeripherals"),
            MachineCommand::UpdateBluetoothScan(update) => defmt::write!(f, "UpdateBluetoothScan({:?})", update),
            // The annotation values are not printed. They are user text of up to 48
            // bytes per entry, eight entries deep, and this formats on every accepted
            // command -- the count is what tells you the command arrived and roughly
            // what it carried.
            MachineCommand::SetShotAnnotations(id, annotations) => defmt::write!(f, "SetShotAnnotations({:?}, {} entries)", id, annotations.len()),
            MachineCommand::SetPendingShotAnnotations(annotations) => defmt::write!(f, "SetPendingShotAnnotations({} entries)", annotations.len()),
            MachineCommand::TagDoseFromScale(scale) => defmt::write!(f, "TagDoseFromScale({:?})", scale),
            MachineCommand::DeleteShotLog(id) => defmt::write!(f, "DeleteShotLog({:?})", id),
            MachineCommand::OpenWifiProvisioningWindow { duration_ms } => defmt::write!(f, "OpenWifiProvisioningWindow({})", duration_ms),
            MachineCommand::CloseWifiProvisioningWindow => defmt::write!(f, "CloseWifiProvisioningWindow"),
            // `{}` on the credentials, not their fields: the type's own `Format` elides the
            // password, and formatting the fields here would bypass that. This command
            // reaches the debug wire and the TCP debug server.
            MachineCommand::SetWifiCredentials(c) => defmt::write!(f, "SetWifiCredentials({})", c),
            MachineCommand::IdentifyMachine => defmt::write!(f, "IdentifyMachine"),
            MachineCommand::RequestConfiguration => defmt::write!(f, "RequestConfiguration"),
        }
    }
}

/// Actions that can be scheduled to run at specific times.
/// This is a subset of MachineCommand that excludes meta-commands like
/// adding/removing schedules or routines, which don't make sense in a schedule.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
// `Debug` for the same reason as `ScheduleItem`: the schedule task logs each dispatched
// action through `log_*!`, and the `log` half needs it.
#[derive(Clone, Debug)]
pub enum ScheduleAction {
    /// Run a routine with optional parameters
    RunRoutine(RoutineIndex, Option<RoutineParameters>),
    /// Cancel the currently running routine
    CancelRoutine,
    /// Set the machine mode (On, Off, PowerSave)
    SetMachineMode(MachineMode),
    /// Set boiler control mode and optionally update target values
    SetBoilerControlTarget(BoilerIndex, BoilerControlMode, Option<BoilerControlTargetValuesUpdate>),
    /// Update boiler target values without changing mode
    SetBoilerControlTargetValues(BoilerIndex, BoilerControlTargetValuesUpdate),
}

#[cfg(feature = "defmt")]
impl defmt::Format for ScheduleAction {
    fn format(&self, f: defmt::Formatter) {
        match self {
            ScheduleAction::RunRoutine(idx, params) => defmt::write!(f, "RunRoutine({}, {} params)", idx, params.as_ref().map(|p| p.len()).unwrap_or(0)),
            ScheduleAction::CancelRoutine => defmt::write!(f, "CancelRoutine"),
            ScheduleAction::SetMachineMode(mode) => defmt::write!(f, "SetMachineMode({:?})", mode),
            ScheduleAction::SetBoilerControlTarget(idx, mode, values) => defmt::write!(f, "SetBoilerControlTarget({}, {:?}, {:?})", idx, mode, values),
            ScheduleAction::SetBoilerControlTargetValues(idx, values) => defmt::write!(f, "SetBoilerControlTargetValues({}, {:?})", idx, values),
        }
    }
}

impl ScheduleAction {
    /// Convert a ScheduleAction to the corresponding MachineCommand
    pub fn to_machine_command(&self) -> MachineCommand {
        match self {
            ScheduleAction::RunRoutine(idx, params) => MachineCommand::RunRoutine(*idx, params.clone()),
            ScheduleAction::CancelRoutine => MachineCommand::CancelRoutine,
            ScheduleAction::SetMachineMode(mode) => MachineCommand::SetMachineMode(*mode),
            ScheduleAction::SetBoilerControlTarget(idx, mode, values) => MachineCommand::SetBoilerControlTarget(*idx, *mode, *values),
            ScheduleAction::SetBoilerControlTargetValues(idx, values) => MachineCommand::SetBoilerControlTargetValues(*idx, *values),
        }
    }
}

#[cfg(all(test, feature = "serde"))]
mod delete_shot_log_tests {
    use super::*;
    use crate::shot_log::ShotLogId;

    /// The command round-trips and names itself.
    ///
    /// `label()` is exhaustive by design, so this test's real value is that the *file*
    /// stops compiling if a future variant skips it -- but the round trip is worth
    /// pinning too: this command reaches the debug wire, where a wrong discriminant
    /// deletes the wrong shot rather than failing.
    #[test]
    fn delete_shot_log_round_trips() {
        let command = MachineCommand::DeleteShotLog(ShotLogId {
            day: Some(20_260_809),
            time: 16_423_349,
        });
        assert_eq!(command.label(), "DeleteShotLog");

        let encoded = postcard::to_allocvec(&command).unwrap();
        let decoded: MachineCommand = postcard::from_bytes(&encoded).unwrap();
        assert_eq!(decoded.label(), "DeleteShotLog");
    }
}
