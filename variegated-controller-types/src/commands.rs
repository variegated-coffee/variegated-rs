use crate::*;
use heapless::index_map::FnvIndexMap;

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
            MachineCommand::AddRoutine(routine) => defmt::write!(f, "AddRoutine()"),
            MachineCommand::RemoveRoutine(idx) => defmt::write!(f, "RemoveRoutine({})", idx),
            MachineCommand::UpdateRoutine(idx, routine) => defmt::write!(f, "UpdateRoutine({})", idx),
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
        }
    }
}

/// Actions that can be scheduled to run at specific times.
/// This is a subset of MachineCommand that excludes meta-commands like
/// adding/removing schedules or routines, which don't make sense in a schedule.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone)]
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
