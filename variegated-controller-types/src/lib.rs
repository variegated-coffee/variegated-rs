#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

use alloc::string::String;
use alloc::vec;
use alloc::vec::Vec;
use core::fmt;
use core::time::Duration;
use chrono::{Datelike, NaiveDate, NaiveDateTime, Weekday};
use heapless::{FnvIndexMap};
use heapless::FnvIndexSet;
use variegated_control_algorithm::pid::PidOut;

pub const MAX_BOILERS: usize = 8;
pub const MAX_GROUPS: usize = 4;
pub const MAX_WATER_TAPS: usize = 4;
pub const MAX_STEAM_WANDS: usize = 4;
pub const MAX_ENVIRONMENTAL_TEMPERATURE_SENSORS: usize = 2;
pub const MAX_TANKS: usize = 2;
pub const MAX_PERIPHERALS: usize = 16;

pub type TemperatureType = f32; // Celsius
pub type PressureType = f32; // Bar
pub type WaterLevelType = u8; // Percent
pub type FlowRateType = f32; // ml/s
pub type InputVolumeType = f64; // ml
pub type WeightType = f32; // g
pub type WeightChangeType  = f32; // g/s
pub type FrequencyType = f32; // Hz
pub type RPMType = f32; // RPM
pub type DutyCycleType = u8; // Percent
pub type ValveOpenType = u8; // Percent
pub type MixingProportionType = u8; // Percent

pub type BoilerIndex = u8;
pub type GroupIndex = u8;
pub type WaterTapIndex = u8;
pub type TankIndex = u8;
pub type SteamWandIndex = u8;

pub type RoutineIndex = usize;

pub type PidParameters = variegated_control_algorithm::pid::PidParameters<f32>;
pub type PidTerm = variegated_control_algorithm::pid::PidTerm<f32>;
pub type PidLimits = variegated_control_algorithm::pid::Limits<f32>;

pub type ExternalSensorId = u8; // Unique identifier for external sensors

pub type EnvironmentalSensorId = u8; // Unique identifier for environmental sensors

pub type PeripheralId = u16; // Unique identifier for peripherals

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug)]
pub struct ProtocolVersion {
    /// Major version of the protocol. Incremented for breaking changes.
    pub major: u8,
    /// Minor version of the protocol. Incremented for non-breaking changes.
    pub minor: u8,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug)]
pub struct ProtocolConfig {
    /// The protocol version.
    pub protocol_version: ProtocolVersion,
    /// The maximum number of boilers supported by this compile of the protocol.
    pub max_boilers: usize,
    /// The maximum number of groups supported by this compile of the protocol.
    pub max_groups: usize,
    /// The maximum number of water taps supported by this compile of the protocol.
    pub max_water_taps: usize,
    /// The maximum number of tanks supported by this compile of the protocol.
    pub max_tanks: usize,
    /// The maximum number of environmental temperature sensors supported by this compile of the protocol.
    pub max_environmental_temperature_sensors: usize,
}

pub const PROTOCOL_VERSION: ProtocolVersion = ProtocolVersion { major: 1, minor: 0 };

pub const PROTOCOL_CONFIG: ProtocolConfig = ProtocolConfig {
    protocol_version: PROTOCOL_VERSION,
    max_boilers: MAX_BOILERS,
    max_groups: MAX_GROUPS,
    max_water_taps: MAX_WATER_TAPS,
    max_tanks: MAX_TANKS,
    max_environmental_temperature_sensors: MAX_ENVIRONMENTAL_TEMPERATURE_SENSORS,
};

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug)]
pub enum PidParameterTarget {
    BoilerTemperature(BoilerIndex),
    BoilerPressure(BoilerIndex),
    GroupFlowRate(GroupIndex),
    GroupOutputFlowRate(GroupIndex),
    GroupPressure(GroupIndex),
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone)]
pub enum MachineCommand {
    StartBrewing(GroupIndex),
    StopBrewing(GroupIndex),
    StartPumpingToWaterTap(WaterTapIndex),
    StopPumpingToWaterTap(WaterTapIndex),

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
    RunRoutine(RoutineIndex, #[cfg_attr(feature = "schemars", schemars(with = "Option<std::collections::HashMap<u8, f32>>"))] Option<FnvIndexMap<u8, f32, 8>>),
    CancelRoutine,
    EnableBoiler(BoilerIndex),
    DisableBoiler(BoilerIndex),
    TareGroupScale(GroupIndex),
    ZeroCalibrateGroupScale(GroupIndex),
    CalibrateGroupScale100g(GroupIndex),
    UpdateCommsStatus(CommsStatus),
    AddScheduleItem(ScheduleItem),
    RemoveScheduleItem(usize),
    UpdateScheduleItem(usize, ScheduleItem),
    AddRoutine(Routine),
    RemoveRoutine(usize),
    UpdateRoutine(usize, Routine),
    SetMachineMode(MachineMode),
}

#[cfg(feature = "defmt")]
impl defmt::Format for MachineCommand {
    fn format(&self, f: defmt::Formatter) {
        match self {
            MachineCommand::StartBrewing(idx) => defmt::write!(f, "StartBrewing({})", idx),
            MachineCommand::StopBrewing(idx) => defmt::write!(f, "StopBrewing({})", idx),
            MachineCommand::StartPumpingToWaterTap(idx) => defmt::write!(f, "StartPumpingToWaterTap({})", idx),
            MachineCommand::StopPumpingToWaterTap(idx) => defmt::write!(f, "StopPumpingToWaterTap({})", idx),
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
        }
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ControlCurve {
    pub a: f32,  // coefficient for T^2
    pub b: f32,  // coefficient for T
    pub c: f32,  // constant term
    pub min: f32, // minimum allowed value
    pub max: f32, // maximum allowed value
}

impl ControlCurve {
    pub fn evaluate(&self, time_seconds: f32) -> f32 {
        let value = self.a * time_seconds * time_seconds + self.b * time_seconds + self.c;
        value.clamp(self.min, self.max)
    }
}

// ===== BOILER CONTROL TYPES =====

/// The control mode for a boiler - what type of control is active
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum BoilerControlMode {
    Temperature,  // Control based on temperature
    Pressure,     // Control based on pressure
    #[default]
    Off          // No control - heater off
}

/// All stored target values for boiler control
/// These values persist regardless of current mode
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoilerControlTargetValues {
    pub target_temperature: TemperatureType,
    pub target_pressure: PressureType,
}

impl Default for BoilerControlTargetValues {
    fn default() -> Self {
        Self {
            target_temperature: 93.0,  // Default brew temperature
            target_pressure: 1.0,       // Default pressure in bar
        }
    }
}

/// Update structure for changing boiler target values
/// Only specified fields will be updated
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BoilerControlTargetValuesUpdate {
    pub temperature: Option<TemperatureType>,
    pub pressure: Option<PressureType>,
}

/// Complete boiler control state
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoilerControlState {
    pub mode: BoilerControlMode,
    pub values: BoilerControlTargetValues,
}

impl Default for BoilerControlState {
    fn default() -> Self {
        Self {
            mode: BoilerControlMode::Off,
            values: BoilerControlTargetValues::default(),
        }
    }
}


#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum WaterDispersalPumpStrategy {
    AlwaysPump,
    NoPump,
}

// ===== GROUP BREW CONTROL TYPES =====

/// The control mode for group brewing
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum GroupBrewControlMode {
    GroupFlowRate,        // Control pump to achieve flow rate at group
    GroupFlowRateCurve,   // Follow a flow rate curve over time
    Pressure,             // Control pump to achieve pressure
    PressureCurve,        // Follow a pressure curve over time
    OutputFlowRate,       // Control based on output (scale) flow rate
    OutputFlowRateCurve,  // Follow output flow curve over time
    FixedDutyCycle,       // Fixed pump duty cycle
    FixedDutyCycleCurve,  // Follow duty cycle curve over time
    FullOn,               // Pump at 100%
    #[default]
    Off                   // Pump off
}

/// All stored target values for group brew control
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GroupBrewControlTargetValues {
    pub flow_rate: FlowRateType,
    pub flow_rate_curve: ControlCurve,
    pub pressure: PressureType,
    pub pressure_curve: ControlCurve,
    pub output_flow_rate: FlowRateType,
    pub output_flow_rate_curve: ControlCurve,
    pub duty_cycle: u8,
    pub duty_cycle_curve: ControlCurve,
}

impl Default for GroupBrewControlTargetValues {
    fn default() -> Self {
        Self {
            flow_rate: 2.5,  // Default 2.5 ml/s
            flow_rate_curve: ControlCurve { a: 0.0, b: 2.5, c: 0.0, min: 0.0, max: 10.0 },
            pressure: 9.0,   // Default 9 bar
            pressure_curve: ControlCurve { a: 0.0, b: 0.0, c: 9.0, min: 0.0, max: 15.0 },
            output_flow_rate: 2.0,  // Default 2.0 ml/s output
            output_flow_rate_curve: ControlCurve { a: 0.0, b: 2.0, c: 0.0, min: 0.0, max: 10.0 },
            duty_cycle: 100,  // Default 100%
            duty_cycle_curve: ControlCurve { a: 0.0, b: 0.0, c: 100.0, min: 0.0, max: 100.0 },
        }
    }
}

/// Update structure for changing group brew target values
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GroupBrewControlTargetValuesUpdate {
    pub flow_rate: Option<FlowRateType>,
    pub flow_rate_curve: Option<ControlCurve>,
    pub pressure: Option<PressureType>,
    pub pressure_curve: Option<ControlCurve>,
    pub output_flow_rate: Option<FlowRateType>,
    pub output_flow_rate_curve: Option<ControlCurve>,
    pub duty_cycle: Option<u8>,
    pub duty_cycle_curve: Option<ControlCurve>,
}

/// Complete group brew control state
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GroupBrewControlState {
    pub mode: GroupBrewControlMode,
    pub values: GroupBrewControlTargetValues,
}

impl Default for GroupBrewControlState {
    fn default() -> Self {
        Self {
            mode: GroupBrewControlMode::Off,
            values: GroupBrewControlTargetValues::default(),
        }
    }
}


#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Copy, Default, Debug, PartialEq)]
pub enum SingleBoilerSingleGroupControllerState {
    #[default]
    BrewModeIdle,
    SteamModeIdle,
    Brewing,
    PumpingToWaterTap,
    PowerSave,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Copy, Debug)]
#[repr(u8)]
pub enum SingleBoilerSingleGroupControllerBoilers {
    BrewBoiler = 0,
    VirtualSteamBoiler = 1,
}

impl SingleBoilerSingleGroupControllerBoilers {
    pub fn is_brew_boiler(self) -> bool {
        matches!(self, SingleBoilerSingleGroupControllerBoilers::BrewBoiler)
    }

    pub fn is_virtual_steam_boiler(self) -> bool {
        matches!(self, SingleBoilerSingleGroupControllerBoilers::VirtualSteamBoiler)
    }

    pub fn as_index(&self) -> BoilerIndex {
        match self {
            SingleBoilerSingleGroupControllerBoilers::BrewBoiler => 0,
            SingleBoilerSingleGroupControllerBoilers::VirtualSteamBoiler => 1,
        }
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Copy, Debug)]
#[repr(u8)]
pub enum DualBoilerSingleGroupControllerBoilers {
    BrewBoiler = 0,
    SteamBoiler = 1,
}

impl DualBoilerSingleGroupControllerBoilers {
    pub fn is_brew_boiler(self) -> bool {
        matches!(self, DualBoilerSingleGroupControllerBoilers::BrewBoiler)
    }

    pub fn is_steam_boiler(self) -> bool {
        matches!(self, DualBoilerSingleGroupControllerBoilers::SteamBoiler)
    }

    pub fn as_index(&self) -> BoilerIndex {
        match self {
            DualBoilerSingleGroupControllerBoilers::BrewBoiler => 0,
            DualBoilerSingleGroupControllerBoilers::SteamBoiler => 1,
        }
    }
}

pub enum SingleGroupControllerGroups {
    SingleGroup = 0,
}

impl SingleGroupControllerGroups {
    pub fn as_index(&self) -> GroupIndex {
        match self {
            SingleGroupControllerGroups::SingleGroup => 0,
        }
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Copy, PartialEq, Debug, Default)]
pub enum MachineMode {
    On,
    #[default]
    Off,
    PowerSaveStandby,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Copy, Debug, Default)]
pub struct PeripheralInfo {
    pub peripheral_type: PeripheralType,
    pub is_available: bool,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Debug, Default)]
pub struct PeripheralStatus {
    #[cfg_attr(feature = "schemars", schemars(with = "std::collections::HashMap<PeripheralId, PeripheralInfo>"))]
    pub peripherals: FnvIndexMap<PeripheralId, PeripheralInfo, MAX_PERIPHERALS>,
}

#[cfg(feature = "defmt")]
impl defmt::Format for PeripheralStatus {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "PeripheralStatus {{ peripherals: [");
        for (id, info) in &self.peripherals {
            defmt::write!(f, "({}, {:?}, {}), ", id, info.peripheral_type, info.is_available);
        }
        defmt::write!(f, "] }}");
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub enum PeripheralType {
    #[default]
    Scale,
    PressureSensor,
    FlowMeter,
    LevelSensor,
}

pub trait PeripheralStatusProvider {
    fn get_peripheral_id(&self) -> PeripheralId;
    fn get_peripheral_type(&self) -> PeripheralType;
    fn is_available(&self) -> bool;
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Debug, Default)]
pub struct RoutineExecutionStatus {
    pub routine_index: RoutineIndex,
    pub current_step: Option<usize>,
    pub step_elapsed_time: Option<Duration>,
    pub total_elapsed_time: Option<Duration>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::collections::HashMap<u8, f32>"))]
    pub resolved_parameters: FnvIndexMap<u8, f32, 8>, // resolved parameter values for display
}

#[cfg(feature = "defmt")]
impl defmt::Format for RoutineExecutionStatus {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "RoutineExecutionStatus {{ routine_index: {}, current_step: {:?}, step_elapsed: {:?}, total_elapsed: {:?}, params_count: {} }}", 
            self.routine_index, 
            self.current_step, 
            self.step_elapsed_time, 
            self.total_elapsed_time,
            self.resolved_parameters.len()
        );
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Debug, Default)]
pub struct Status {
    #[cfg_attr(feature = "schemars", schemars(with = "std::collections::HashMap<BoilerIndex, BoilerStatus>"))]
    pub boiler_statuses: FnvIndexMap<BoilerIndex, BoilerStatus, MAX_BOILERS>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::collections::HashMap<GroupIndex, GroupStatus>"))]
    pub group_statuses: FnvIndexMap<GroupIndex, GroupStatus, MAX_GROUPS>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::collections::HashMap<WaterTapIndex, WaterTapStatus>"))]
    pub water_tap_statuses: FnvIndexMap<WaterTapIndex, WaterTapStatus, MAX_WATER_TAPS>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::collections::HashMap<TankIndex, TankStatus>"))]
    pub tank_statuses: FnvIndexMap<TankIndex, TankStatus, MAX_TANKS>,
    pub mode: MachineMode,
    pub routine_execution: Option<RoutineExecutionStatus>,
    pub comms_status: Option<CommsStatus>,
    pub peripheral_status: PeripheralStatus,
    #[cfg_attr(feature = "schemars", schemars(with = "Option<String>"))]
    pub current_local_time: Option<NaiveDateTime>
//    pub environmental_temperature_sensors: FnvIndexMap<EnvironmentalSensorId, TemperatureType, MAX_ENVIRONMENTAL_TEMPERATURE_SENSORS>, // Up to 8 external sensors
}

impl Status {
    pub fn new() -> Self {
        Status {
            boiler_statuses: FnvIndexMap::new(),
            group_statuses: FnvIndexMap::new(),
            water_tap_statuses: FnvIndexMap::new(),
            tank_statuses: FnvIndexMap::new(),
            mode: MachineMode::Off,
            routine_execution: None,
            comms_status: None,
            peripheral_status: PeripheralStatus::default(),
            current_local_time: None,
//            environmental_temperature_sensors: FnvIndexMap::new(),
        }
    }

    pub fn get_boiler_status(&self, boiler_index: BoilerIndex) -> Option<&BoilerStatus> {
        self.boiler_statuses.get(&boiler_index)
    }

    pub fn get_group_status(&self, group_index: GroupIndex) -> Option<&GroupStatus> {
        self.group_statuses.get(&group_index)
    }

    pub fn get_water_tap_status(&self, water_tap_index: WaterTapIndex) -> Option<&WaterTapStatus> {
        self.water_tap_statuses.get(&water_tap_index)
    }

    pub fn get_tank_status(&self, tank_index: TankIndex) -> Option<&TankStatus> {
        self.tank_statuses.get(&tank_index)
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Status {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "Status {{");

        // Machine mode and routine status
        defmt::write!(f, " mode: {:?}", self.mode);
        if let Some(ref routine) = self.routine_execution {
            defmt::write!(f, ", routine: {} step:{}", routine.routine_index, routine.current_step);
        }

        // Boiler statuses
        defmt::write!(f, ", boilers: [");
        for (index, boiler_status) in self.boiler_statuses.iter() {
            defmt::write!(f, " B{}(", index);
            if let Some(temp) = boiler_status.temperature {
                defmt::write!(f, "T:{}°C", temp);
            } else {
                defmt::write!(f, "T:None");
            }
            if let Some(pressure) = boiler_status.pressure {
                defmt::write!(f, " P:{}bar", pressure);
            } else {
                defmt::write!(f, " P:None");
            }
            if let Some(water_level) = boiler_status.water_level {
                defmt::write!(f, " WL:{}%", water_level);
            } else {
                defmt::write!(f, " WL:None");
            }
            match boiler_status.output {
                Output::Off => defmt::write!(f, " OUT:Off"),
                Output::FixedDutyCycle(dc) => defmt::write!(f, " OUT:{}%", dc),
                Output::PidOutput(pid_out) => defmt::write!(f, " OUT:PID{}%", pid_out.out),
            }
            defmt::write!(f, " MODE:{:?}", boiler_status.control_state.mode);
            defmt::write!(f, " VALUES:T{}°C/P{}bar", boiler_status.control_state.values.target_temperature, boiler_status.control_state.values.target_pressure);
            defmt::write!(f, ")");
        }
        defmt::write!(f, " ]");

        // Group statuses
        defmt::write!(f, ", groups: [");
        for (index, group_status) in self.group_statuses.iter() {
            defmt::write!(f, " G{}(", index);
            defmt::write!(f, "brewing:{}", group_status.is_brewing);
            if let Some(brew_time) = group_status.brew_time {
                defmt::write!(f, " time:{}s", brew_time.as_secs());
            }
            if let Some(in_flow) = group_status.input_flow_rate {
                defmt::write!(f, " in_flow:{}", in_flow);
            }
            if let Some(volume) = group_status.input_volume {
                defmt::write!(f, " volume:{}ml", volume);
            }
            if let Some(out_flow) = group_status.output_flow_rate {
                defmt::write!(f, " out_flow:{}", out_flow);
            }
            if let Some(weight) = group_status.output_weight {
                defmt::write!(f, " weight:{}g", weight);
            }
            if let Some(pressure) = group_status.pressure {
                defmt::write!(f, " P:{}bar", pressure);
            }
            if let Some(temp) = group_status.temperature {
                defmt::write!(f, " T:{}°C", temp);
            }
            match group_status.pump_output {
                Output::Off => defmt::write!(f, " PUMP:Off"),
                Output::FixedDutyCycle(dc) => defmt::write!(f, " PUMP:{}%", dc),
                Output::PidOutput(pid_out) => defmt::write!(f, " PUMP:PID{}%", pid_out.out),
            }
            defmt::write!(f, ")");
        }
        defmt::write!(f, " ]");

        // Water tap statuses
        defmt::write!(f, ", water_taps: [");
        for (index, water_tap_status) in self.water_tap_statuses.iter() {
            defmt::write!(f, " WT{}(", index);
            defmt::write!(f, "dispensing:{}", water_tap_status.is_dispensing);
            defmt::write!(f, ")");
        }
        defmt::write!(f, " ]");

        // Tank statuses
        defmt::write!(f, ", tanks: [");
        for (index, tank_status) in self.tank_statuses.iter() {
            defmt::write!(f, " T{}(", index);
            if let Some(water_level) = tank_status.water_level {
                defmt::write!(f, "WL:{}%", water_level);
            } else {
                defmt::write!(f, "WL:None");
            }
            defmt::write!(f, ")");
        }
        defmt::write!(f, " ]");

        // Communication status
        if let Some(ref comms) = self.comms_status {
            defmt::write!(f, ", wifi:{}", comms.wifi_connected);
            if let Some(timestamp) = comms.timestamp {
                defmt::write!(f, " ts:{}", timestamp);
            }
        }

        defmt::write!(f, " }}");
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default)]
pub enum Output {
    #[default]
    Off,
    FixedDutyCycle(DutyCycleType),
    PidOutput(PidOut<f32>),
}

impl Output {
    pub fn duty_cycle(&self) -> DutyCycleType {
        match self {
            Output::Off => 0,
            Output::FixedDutyCycle(duty_cycle) => *duty_cycle as DutyCycleType,
            Output::PidOutput(pid_out) => pid_out.out as DutyCycleType,
        }
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default)]
pub struct BoilerStatus {
    pub temperature: Option<TemperatureType>,
    pub pressure: Option<PressureType>,
    pub water_level: Option<WaterLevelType>,
    pub output: Output,
    pub control_state: BoilerControlState,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PreviousBrewInfo {
    pub brew_time: Duration,
    pub brew_input_volume: Option<InputVolumeType>,
    pub output_weight: Option<WeightType>,
    pub started_at_millis: u64,  // Milliseconds since system start
    pub stopped_at_millis: u64,  // Milliseconds since system start
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default)]
pub struct GroupStatus {
    pub is_brewing: bool,
    pub three_way_valve_open: Option<bool>,
    pub brew_time: Option<Duration>,
    pub brew_input_volume: Option<InputVolumeType>,
    pub input_flow_rate: Option<FlowRateType>,
    pub input_volume: Option<InputVolumeType>,
    pub output_flow_rate: Option<FlowRateType>,
    pub output_weight: Option<WeightType>,
    pub pressure: Option<PressureType>,
    pub temperature: Option<TemperatureType>,
    pub pump_output: Output,
    pub control_state: GroupBrewControlState,
    pub previous_brew: Option<PreviousBrewInfo>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default)]
pub struct WaterTapStatus {
    pub is_dispensing: bool,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Debug, Default)]
pub struct TankStatus {
    pub water_level: Option<WaterLevelType>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Debug, Default)]
pub struct ScheduleTrigger {
    pub on_minute: u8,
    pub on_hour: u8,
    #[cfg_attr(feature = "schemars", schemars(with = "Option<std::vec::Vec<String>>"))]
    pub on_days: Option<FnvIndexSet<Weekday, 8>>, // If None, trigger every day
    #[cfg_attr(feature = "schemars", schemars(with = "Option<String>"))]
    pub on_date: Option<NaiveDate>,
    pub enabled: bool,
    pub once: bool, // If true, remove schedule item after triggering
}

#[cfg(feature = "defmt")]
impl defmt::Format for ScheduleTrigger {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "ScheduleTrigger {{ {:02}:{:02}", self.on_hour, self.on_minute);

        if let Some(ref days) = self.on_days {
            defmt::write!(f, " days:[");
            for day in days {
                let day_str = match day {
                    Weekday::Mon => "Mon",
                    Weekday::Tue => "Tue",
                    Weekday::Wed => "Wed",
                    Weekday::Thu => "Thu",
                    Weekday::Fri => "Fri",
                    Weekday::Sat => "Sat",
                    Weekday::Sun => "Sun",
                };
                defmt::write!(f, "{},", day_str);
            }
            defmt::write!(f, "]");
        }

        if let Some(ref date) = self.on_date {
            defmt::write!(f, " date:{}-{:02}-{:02}", date.year(), date.month(), date.day());
        }

        if !self.enabled {
            defmt::write!(f, " DISABLED");
        }

        if self.once {
            defmt::write!(f, " once");
        }

        defmt::write!(f, " }}");
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Default)]
pub struct ScheduleItem {
    pub trigger_at: ScheduleTrigger,
    pub commands: Vec<MachineCommand>,
}

#[cfg(feature = "defmt")]
impl defmt::Format for ScheduleItem {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "ScheduleItem {{ trigger_at: {}, commands: [", self.trigger_at);
        for (i, cmd) in self.commands.iter().enumerate() {
            if i > 0 {
                defmt::write!(f, ", ");
            }
            defmt::write!(f, "{}", cmd);
        }
        defmt::write!(f, "] }}");
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Default)]
pub struct Configuration {
    pub machine_config: MachineConfiguration,
    #[cfg_attr(feature = "schemars", schemars(with = "std::collections::HashMap<BoilerIndex, BoilerConfiguration>"))]
    pub boiler_configurations: FnvIndexMap<BoilerIndex, BoilerConfiguration, MAX_BOILERS>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::collections::HashMap<GroupIndex, GroupConfiguration>"))]
    pub group_configurations: FnvIndexMap<GroupIndex, GroupConfiguration, MAX_GROUPS>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::collections::HashMap<WaterTapIndex, WaterTapConfiguration>"))]
    pub water_tap_configurations: FnvIndexMap<WaterTapIndex, WaterTapConfiguration, MAX_WATER_TAPS>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::collections::HashMap<TankIndex, TankConfiguration>"))]
    pub tank_configurations: FnvIndexMap<TankIndex, TankConfiguration, MAX_TANKS>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::collections::HashMap<SteamWandIndex, SteamWandConfiguration>"))]
    pub steam_wand_configurations: FnvIndexMap<SteamWandIndex, SteamWandConfiguration, MAX_STEAM_WANDS>,
    pub schedules: Vec<ScheduleItem>,
}

impl Configuration {
    pub fn new() -> Self {
        Configuration {
            machine_config: MachineConfiguration::default(),
            boiler_configurations: FnvIndexMap::new(),
            group_configurations: FnvIndexMap::new(),
            water_tap_configurations: FnvIndexMap::new(),
            tank_configurations: FnvIndexMap::new(),
            steam_wand_configurations: FnvIndexMap::new(),
            schedules: vec![],
        }
    }

    // Boiler configuration methods
    pub fn get_boiler_configuration(&self, index: BoilerIndex) -> Option<&BoilerConfiguration> {
        self.boiler_configurations.get(&index)
    }

    pub fn insert_boiler_configuration(&mut self, index: BoilerIndex, config: BoilerConfiguration) {
        self.boiler_configurations.insert(index, config).ok();
    }

    pub fn iter_boilers(&self) -> impl Iterator<Item = (&BoilerIndex, &BoilerConfiguration)> {
        self.boiler_configurations.iter()
    }

    // Group configuration methods
    pub fn get_group_configuration(&self, index: GroupIndex) -> Option<&GroupConfiguration> {
        self.group_configurations.get(&index)
    }

    pub fn insert_group_configuration(&mut self, index: GroupIndex, config: GroupConfiguration) {
        self.group_configurations.insert(index, config).ok();
    }

    pub fn iter_groups(&self) -> impl Iterator<Item = (&GroupIndex, &GroupConfiguration)> {
        self.group_configurations.iter()
    }

    // Water tap configuration methods
    pub fn get_water_tap_configuration(&self, index: WaterTapIndex) -> Option<&WaterTapConfiguration> {
        self.water_tap_configurations.get(&index)
    }

    pub fn insert_water_tap_configuration(&mut self, index: WaterTapIndex, config: WaterTapConfiguration) {
        self.water_tap_configurations.insert(index, config).ok();
    }

    pub fn iter_water_taps(&self) -> impl Iterator<Item = (&WaterTapIndex, &WaterTapConfiguration)> {
        self.water_tap_configurations.iter()
    }

    // Tank configuration methods
    pub fn get_tank_configuration(&self, index: TankIndex) -> Option<&TankConfiguration> {
        self.tank_configurations.get(&index)
    }

    pub fn insert_tank_configuration(&mut self, index: TankIndex, config: TankConfiguration) {
        self.tank_configurations.insert(index, config).ok();
    }

    pub fn iter_tanks(&self) -> impl Iterator<Item = (&TankIndex, &TankConfiguration)> {
        self.tank_configurations.iter()
    }

    // Steam wand configuration methods
    pub fn get_steam_wand_configuration(&self, index: SteamWandIndex) -> Option<&SteamWandConfiguration> {
        self.steam_wand_configurations.get(&index)
    }

    pub fn insert_steam_wand_configuration(&mut self, index: SteamWandIndex, config: SteamWandConfiguration) {
        self.steam_wand_configurations.insert(index, config).ok();
    }

    pub fn iter_steam_wands(&self) -> impl Iterator<Item = (&SteamWandIndex, &SteamWandConfiguration)> {
        self.steam_wand_configurations.iter()
    }

}

#[cfg(feature = "defmt")]
impl defmt::Format for Configuration {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "Configuration {{ }}",
            /*self.boiler_configuration, self.group_configuration*/);
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default)]
pub struct PumpConfiguration {
    pub tacho_pulses_per_liter: Option<f32>,
    pub max_duty_cycle: Option<DutyCycleType>,
    pub min_duty_cycle: Option<DutyCycleType>,
    pub ramp_up_time_ms: Option<u32>,
    pub ramp_down_time_ms: Option<u32>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default)]
pub struct FillConfiguration {
    pub fill_threshold: Option<WaterLevelType>, // If none, filling is disabled
    pub pump_configuration: Option<PumpConfiguration>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default)]
pub struct BoilerConfiguration {
    pub temperature_pid_parameters: PidParameters,
    pub pressure_pid_parameters: PidParameters,
    pub control_state: BoilerControlState,
    pub max_temperature: Option<TemperatureType>,
    pub max_pressure: Option<PressureType>,
    pub temperature_sensor_kalman_parameters: Option<KalmanParameters>,
    pub pressure_sensor_kalman_parameters: Option<KalmanParameters>,
    pub fill_config: Option<FillConfiguration>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct KalmanParameters {
    pub process_noise: f32,
    pub measurement_noise: f32,
    pub estimated_error: f32,
    pub posterior_estimate: f32,
}



#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default)]
pub struct GroupConfiguration {
    pub flow_rate_pid_parameters: PidParameters,
    pub output_flow_rate_pid_parameters: PidParameters,
    pub pressure_pid_parameters: PidParameters,
    pub brew_control_state: GroupBrewControlState,
    pub max_brew_time_seconds: Option<u32>,
    pub auto_tare_enabled: bool,
    pub pump_configuration: Option<PumpConfiguration>,
    pub pressure_sensor_kalman_parameters: Option<KalmanParameters>,
    pub flow_sensor_pulses_per_liter: Option<f32>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MachineType {
    SingleBoilerSingleGroup,
    DualBoilerSingleGroup,
    DualBoilerDualGroup,
}

impl Default for MachineType {
    fn default() -> Self {
        MachineType::SingleBoilerSingleGroup
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Debug, Default)]
pub struct MachineConfiguration {
    pub heating_element_interlock: bool,
}


#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default)]
pub struct WaterTapConfiguration {
    pub pump_strategy: WaterDispersalPumpStrategy,
    pub temperature_target: Option<TemperatureType>,
    pub max_dispense_time_seconds: Option<u32>,
    pub flow_rate_limit: Option<FlowRateType>,
    pub pump_configuration: Option<PumpConfiguration>,
}

impl Default for WaterDispersalPumpStrategy {
    fn default() -> Self {
        WaterDispersalPumpStrategy::AlwaysPump
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Debug, Default)]
pub struct SteamWandConfiguration {
    pub temperature_target: Option<TemperatureType>,
    pub pressure_target: Option<PressureType>,
    pub purge_time_seconds: Option<u32>,
    pub max_steam_time_seconds: Option<u32>,
    pub auto_purge_enabled: bool,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Debug, Default)]
pub struct TankConfiguration {
    pub low_level_warning_threshold: Option<WaterLevelType>,
    pub water_level_sensor_kalman_parameters: Option<KalmanParameters>,
}


#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone,  Debug)]
pub struct ExternalSensorData {
    pub id: EnvironmentalSensorId,
    pub value: f32,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Copy, Debug)]
pub struct CommsStatus {
    pub timestamp: Option<u64>, // Unix timestamp in seconds
    pub wifi_connected: bool,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone)]
pub enum CommsProcessorToApplicationProcessorMessage {
    Command(MachineCommand),
    CommsStatus(CommsStatus),
    RequestStatus,
    RequestMachineDefinition,
    RequestConfiguration,
    RequestRoutines,
    ExternalSensorUpdate(ExternalSensorData),
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SensorCapability {
    Temperature,
    Pressure,
    WaterLevel,
    InputFlowRate,
    OutputFlowRate,
    Weight,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ActuatorCapability {
    HeatingElement,
    Pump,
    SolenoidValve,
    ThreeWayValve,
    WaterMixer,
    ScaleTare,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ControlModeCapability {
    TemperaturePid,
    PressurePid,
    FlowRatePid,
    OutputFlowRatePid,
    FixedDutyCycle,
    FullOn,
    Off,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum BoilerType {
    BrewBoiler,
    SteamBoiler,
    VirtualSteamBoiler,
}

impl Default for BoilerType {
    fn default() -> Self {
        BoilerType::BrewBoiler
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Debug)]
pub struct BoilerDefinition {
    #[cfg_attr(feature = "schemars", schemars(with = "String"))]
    pub name: heapless::String<32>,
    pub boiler_type: BoilerType,
    #[cfg_attr(feature = "schemars", schemars(with = "std::vec::Vec<SensorCapability>"))]
    pub sensors: heapless::Vec<SensorCapability, 8>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::vec::Vec<ActuatorCapability>"))]
    pub actuators: heapless::Vec<ActuatorCapability, 8>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::vec::Vec<ControlModeCapability>"))]
    pub control_modes: heapless::Vec<ControlModeCapability, 8>,
    pub has_fill_mechanism: bool,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Debug)]
pub struct GroupDefinition {
    #[cfg_attr(feature = "schemars", schemars(with = "String"))]
    pub name: heapless::String<32>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::vec::Vec<SensorCapability>"))]
    pub sensors: heapless::Vec<SensorCapability, 8>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::vec::Vec<ActuatorCapability>"))]
    pub actuators: heapless::Vec<ActuatorCapability, 8>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::vec::Vec<ControlModeCapability>"))]
    pub control_modes: heapless::Vec<ControlModeCapability, 8>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Debug)]
pub struct WaterTapDefinition {
    #[cfg_attr(feature = "schemars", schemars(with = "String"))]
    pub name: heapless::String<32>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::vec::Vec<SensorCapability>"))]
    pub sensors: heapless::Vec<SensorCapability, 8>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::vec::Vec<ActuatorCapability>"))]
    pub actuators: heapless::Vec<ActuatorCapability, 8>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::vec::Vec<ControlModeCapability>"))]
    pub control_modes: heapless::Vec<ControlModeCapability, 8>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Debug)]
pub struct SteamWandDefinition {
    #[cfg_attr(feature = "schemars", schemars(with = "String"))]
    pub name: heapless::String<32>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::vec::Vec<SensorCapability>"))]
    pub sensors: heapless::Vec<SensorCapability, 8>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::vec::Vec<ActuatorCapability>"))]
    pub actuators: heapless::Vec<ActuatorCapability, 8>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::vec::Vec<ControlModeCapability>"))]
    pub control_modes: heapless::Vec<ControlModeCapability, 8>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Debug)]
pub struct TankDefinition {
    #[cfg_attr(feature = "schemars", schemars(with = "String"))]
    pub name: heapless::String<32>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::vec::Vec<SensorCapability>"))]
    pub sensors: heapless::Vec<SensorCapability, 8>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Debug)]
pub struct PeripheralDefinition {
    pub peripheral_type: PeripheralType,
    #[cfg_attr(feature = "schemars", schemars(with = "String"))]
    pub location: heapless::String<32>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::vec::Vec<SensorCapability>"))]
    pub capabilities: heapless::Vec<SensorCapability, 8>,
    pub support_calibration: bool,
    pub via_comms_mcu: bool,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EnvironmentalSensorType {
    AmbientTemperature,
    CaseTemperature,
    ExternalTemperature,
    Humidity,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Debug)]
pub struct EnvironmentalSensorDefinition {
    #[cfg_attr(feature = "schemars", schemars(with = "String"))]
    pub name: heapless::String<32>,
    pub sensor_type: EnvironmentalSensorType,
    pub measurement_range: Option<(f32, f32)>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Debug)]
pub struct MachineDefinition {
    #[cfg_attr(feature = "schemars", schemars(with = "String"))]
    pub name: heapless::String<32>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::collections::HashMap<BoilerIndex, BoilerDefinition>"))]
    pub boilers: FnvIndexMap<BoilerIndex, BoilerDefinition, MAX_BOILERS>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::collections::HashMap<GroupIndex, GroupDefinition>"))]
    pub groups: FnvIndexMap<GroupIndex, GroupDefinition, MAX_GROUPS>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::collections::HashMap<WaterTapIndex, WaterTapDefinition>"))]
    pub water_taps: FnvIndexMap<WaterTapIndex, WaterTapDefinition, MAX_WATER_TAPS>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::collections::HashMap<TankIndex, TankDefinition>"))]
    pub tanks: FnvIndexMap<TankIndex, TankDefinition, MAX_TANKS>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::collections::HashMap<SteamWandIndex, SteamWandDefinition>"))]
    pub steam_wands: FnvIndexMap<SteamWandIndex, SteamWandDefinition, 4>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::collections::HashMap<EnvironmentalSensorId, EnvironmentalSensorDefinition>"))]
    pub environmental_sensors: FnvIndexMap<EnvironmentalSensorId, EnvironmentalSensorDefinition, MAX_ENVIRONMENTAL_TEMPERATURE_SENSORS>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::collections::HashMap<PeripheralId, PeripheralDefinition>"))]
    pub peripherals: FnvIndexMap<PeripheralId, PeripheralDefinition, MAX_PERIPHERALS>,
}

 impl MachineDefinition {
    pub fn add_boiler(&mut self, index: BoilerIndex, definition: BoilerDefinition) -> Result<(), ()> {
        self.boilers.insert(index, definition).map(|_| ()).map_err(|_| ())
    }

    pub fn add_group(&mut self, index: GroupIndex, definition: GroupDefinition) -> Result<(), ()> {
        self.groups.insert(index, definition).map(|_| ()).map_err(|_| ())
    }

    pub fn add_water_tap(&mut self, index: WaterTapIndex, definition: WaterTapDefinition) -> Result<(), ()> {
        self.water_taps.insert(index, definition).map(|_| ()).map_err(|_| ())
    }

    pub fn add_tank(&mut self, index: TankIndex, definition: TankDefinition) -> Result<(), ()> {
        self.tanks.insert(index, definition).map(|_| ()).map_err(|_| ())
    }

    pub fn add_peripheral(&mut self, id: PeripheralId, definition: PeripheralDefinition) -> Result<(), ()> {
        self.peripherals.insert(id, definition).map(|_| ()).map_err(|_| ())
    }

    pub fn add_steam_wand(&mut self, index: u8, definition: SteamWandDefinition) -> Result<(), ()> {
        self.steam_wands.insert(index, definition).map(|_| ()).map_err(|_| ())
    }

    pub fn add_environmental_sensor(&mut self, id: EnvironmentalSensorId, definition: EnvironmentalSensorDefinition) -> Result<(), ()> {
        self.environmental_sensors.insert(id, definition).map(|_| ()).map_err(|_| ())
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for MachineDefinition {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "MachineDefinition {{ boilers: {} boilers, groups: {} groups, water_taps: {} water_taps, tanks: {} tanks, steam_wands: {} wands, env_sensors: {} sensors, peripherals: {} peripherals }}",
            self.boilers.len(),
            self.groups.len(),
            self.water_taps.len(),
            self.tanks.len(),
            self.steam_wands.len(),
            self.environmental_sensors.len(),
            self.peripherals.len(),
        );
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone)]
pub struct RoutineList {
    pub routines: Vec<Routine>,
}

#[cfg(feature = "defmt")]
impl defmt::Format for RoutineList {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "RoutineList {{ routines: [");
        for (i, routine) in self.routines.iter().enumerate() {
            if i > 0 {
                defmt::write!(f, ", ");
            }
            defmt::write!(f, "{}", routine.name.as_str());
        }
        defmt::write!(f, "] }}");
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone)]
pub enum ApplicationProcessorToCommsProcessorMessage {
    Hello(ProtocolConfig),
    Status(Status),
    MachineDefinition(MachineDefinition),
    Configuration(Configuration),
    Routines(RoutineList),
}


pub type UserActionIndex = u8;
pub type RoutineParameters = FnvIndexMap<u8, f32, 8>;

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Copy, Debug)]
pub enum ParameterValue {
    Static(f32),
    Parameter(u8), // index into parameter map
    DerivedParameter(u8), // index into derived parameter list (separate namespace)
}

impl fmt::Display for ParameterValue {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ParameterValue::Static(value) => write!(f, "{:.1}", value),
            ParameterValue::Parameter(index) => write!(f, "P{}", index),
            ParameterValue::DerivedParameter(index) => write!(f, "D{}", index),
        }
    }
}

impl ParameterValue {
    // For display purposes, treat Parameter as a placeholder value
    pub fn as_secs(&self) -> u64 {
        match self {
            ParameterValue::Static(value) => *value as u64,
            ParameterValue::Parameter(_) => 0, // Placeholder - should be resolved
            ParameterValue::DerivedParameter(_) => 0, // Placeholder - should be resolved
        }
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ParameterUnit {
    Seconds,
    Celsius,
    Bar,
    MillilitersPerSecond,
    Grams,
    Percent,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Debug)]
pub struct RoutineParameter {
    pub index: u8,
    pub name: String,  // User-facing, e.g. "Preinfusion Time", "Target Pressure"
    pub default: f32,
    pub unit: Option<ParameterUnit>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Debug)]
pub struct DerivedParameter {
    pub index: u8,  // Separate index space from regular parameters
    pub name: String,
    pub unit: Option<ParameterUnit>,
    pub formula: DerivedFormula,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Debug)]
pub enum DerivedFormula {
    Linear {
        base_param: u8,      // Index of base parameter
        multiplier: f32,
        offset: f32,
    },
    Sum {
        params: Vec<u8>,     // Indices of parameters to sum
    },
    Difference {
        param_a: u8,
        param_b: u8,         // a - b
    },
    Product {
        params: Vec<u8>,     // Indices of parameters to multiply
    },
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Copy, Debug)]
pub enum StateCondition {
    Brewing(GroupIndex),
    NotBrewing(GroupIndex),
    BoilerTemperatureAbove(BoilerIndex, ParameterValue),
    BoilerTemperatureBelow(BoilerIndex, ParameterValue),
    BoilerPressureAbove(BoilerIndex, ParameterValue),
    BoilerPressureBelow(BoilerIndex, ParameterValue),
    GroupInputFlowRateAbove(GroupIndex, ParameterValue),
    GroupInputFlowRateBelow(GroupIndex, ParameterValue),
    GroupPressureAbove(GroupIndex, ParameterValue),
    GroupPressureBelow(GroupIndex, ParameterValue),
    WaterTapFlowRateAbove(WaterTapIndex, ParameterValue),
    WaterTapFlowRateBelow(WaterTapIndex, ParameterValue),
    OutputWeightAbove(GroupIndex, ParameterValue),
    OutputWeightBelow(GroupIndex, ParameterValue),
    InputVolumeAboveRelativeToStart(GroupIndex, ParameterValue),
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Copy, Debug)]
pub enum RoutineExitCondition {
    Always,
    Never,
    After(ParameterValue), // seconds as f32, converted to Duration at runtime
    AfterDurationRelativeToStart(ParameterValue),
    StateConditionMet(StateCondition),
    UserAction(UserActionIndex),
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Copy, Debug)]
pub enum RoutineStepExitType {
    NextStep,
    JumpToStep(usize),
    Finished,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Debug)]
pub enum RoutineCommand {
    // Direct pass-through for non-parameterizable commands
    StartBrewing(GroupIndex),
    StopBrewing(GroupIndex),
    TareGroupScale(GroupIndex),

    // Parameterizable commands
    SetBoilerTemperature(BoilerIndex, ParameterValue),
    SetBoilerPressure(BoilerIndex, ParameterValue),
    SetGroupFlowRate(GroupIndex, ParameterValue),
    SetGroupPressure(GroupIndex, ParameterValue),
    SetGroupOutputFlowRate(GroupIndex, ParameterValue),
    SetGroupFixedDutyCycle(GroupIndex, ParameterValue),
    SetGroupFullOn(GroupIndex),
    SetGroupOff(GroupIndex),
    SetBoilerOff(BoilerIndex),

    // Transition-enabled commands (only for groups since only they support curves)
    SetGroupFlowRateWithTransition(GroupIndex, ParameterValue, ParameterValue), // target, transition_time
    SetGroupPressureWithTransition(GroupIndex, ParameterValue, ParameterValue), // target, transition_time
    SetGroupOutputFlowRateWithTransition(GroupIndex, ParameterValue, ParameterValue), // target, transition_time
    SetGroupFixedDutyCycleWithTransition(GroupIndex, ParameterValue, ParameterValue), // target, transition_time
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Debug)]
pub struct RoutineExit {
    pub condition: RoutineExitCondition,
    pub then: RoutineStepExitType,
    pub description: Option<String>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Debug)]
pub struct RoutineStep {
    pub entry_command: Option<RoutineCommand>,
    pub exits: Vec<RoutineExit>,
    pub description: Option<String>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Copy, Debug)]
pub enum RoutineType {
    HeatUp,
    UserDefined,
    Cleaning,
    HardwareButtonMapped,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone)]
pub struct Routine {
    pub routine_type: RoutineType,
    pub name: String,
    pub parameters: Vec<RoutineParameter>, // max 8
    pub derived_parameters: Vec<DerivedParameter>, // max 16
    pub steps: Vec<RoutineStep>,
    pub finally: Vec<RoutineCommand>,
}

#[cfg(feature = "defmt")]
impl defmt::Format for Routine {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "Routine {{ name: {}, type: {:?}, parameters: {}, derived_parameters: {}, steps: {} }}",
            self.name.as_str(),
            self.routine_type,
            self.parameters.len(),
            self.derived_parameters.len(),
            self.steps.len(),
        );
    }
}

impl Routine {
    pub fn new(routine_type: RoutineType, name: String, parameters: Vec<RoutineParameter>, derived_parameters: Vec<DerivedParameter>, steps: Vec<RoutineStep>) -> Self {
        // Validate limits
        assert!(parameters.len() <= 8, "Maximum 8 regular parameters allowed");
        assert!(derived_parameters.len() <= 16, "Maximum 16 derived parameters allowed");

        Self {
            routine_type,
            name,
            parameters,
            derived_parameters,
            steps,
            finally: vec![],
        }
    }

    pub fn routine_type(&self) -> RoutineType {
        self.routine_type
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn steps(&self) -> &[RoutineStep] {
        &self.steps
    }

    pub fn parameters(&self) -> &[RoutineParameter] {
        &self.parameters
    }

    pub fn derived_parameters(&self) -> &[DerivedParameter] {
        &self.derived_parameters
    }
}

impl RoutineExit {
    pub fn new(condition: RoutineExitCondition, then: RoutineStepExitType) -> Self {
        Self {
            condition,
            then,
            description: None,
        }
    }

    pub fn with_description(condition: RoutineExitCondition, then: RoutineStepExitType, description: String) -> Self {
        Self {
            condition,
            then,
            description: Some(description),
        }
    }

    pub fn description(&self) -> Option<&str> {
        self.description.as_deref()
    }
}

impl RoutineStep {
    pub fn description(&self) -> Option<&str> {
        self.description.as_deref()
    }

    pub fn exits(&self) -> &[RoutineExit] {
        &self.exits
    }
}

// Sequential storage implementation for Routine
#[cfg(feature = "sequential-storage")]
use sequential_storage::map::{SerializationError, Value};
#[cfg(feature = "sequential-storage")]
use postcard::{to_slice_crc32, from_bytes_crc32};
#[cfg(feature = "sequential-storage")]
use crc::{Crc, CRC_32_ISCSI};

#[cfg(feature = "sequential-storage")]
impl<'a> Value<'a> for Routine {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        let crc = Crc::<u32>::new(&CRC_32_ISCSI);

        let slice = match to_slice_crc32(self, buffer, crc.digest()) {
            Ok(bytes) => Ok(bytes.len()),
            Err(postcard::Error::SerializeBufferFull) => {
                Err(SerializationError::BufferTooSmall)
            },
            Err(_) => {
                Err(SerializationError::InvalidData)
            },
        };

        slice
    }

    fn deserialize_from(buffer: &'a [u8]) -> Result<Self, SerializationError>
    where
        Self: Sized
    {
        let crc = Crc::<u32>::new(&CRC_32_ISCSI);

        let v = match from_bytes_crc32(buffer, crc.digest()) {
            Ok(value) => Ok(value),
            Err(postcard::Error::DeserializeUnexpectedEnd) => {
                Err(SerializationError::InvalidFormat)
            },
            Err(postcard::Error::DeserializeBadEnum) => {
                Err(SerializationError::InvalidFormat)
            },
            Err(_) => {
                Err(SerializationError::InvalidFormat)
            },
        };

        v
    }
}

#[cfg(feature = "sequential-storage")]
impl<'a> Value<'a> for ScheduleItem {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        let crc = Crc::<u32>::new(&CRC_32_ISCSI);

        let slice = match to_slice_crc32(self, buffer, crc.digest()) {
            Ok(bytes) => Ok(bytes.len()),
            Err(postcard::Error::SerializeBufferFull) => {
                Err(SerializationError::BufferTooSmall)
            },
            Err(_) => {
                Err(SerializationError::InvalidData)
            },
        };

        slice
    }

    fn deserialize_from(buffer: &'a [u8]) -> Result<Self, SerializationError>
    where
        Self: Sized
    {
        let crc = Crc::<u32>::new(&CRC_32_ISCSI);

        let v = match from_bytes_crc32(buffer, crc.digest()) {
            Ok(value) => Ok(value),
            Err(postcard::Error::DeserializeUnexpectedEnd) => {
                Err(SerializationError::InvalidFormat)
            },
            Err(postcard::Error::DeserializeBadEnum) => {
                Err(SerializationError::InvalidFormat)
            },
            Err(_) => {
                Err(SerializationError::InvalidFormat)
            },
        };

        v
    }
}