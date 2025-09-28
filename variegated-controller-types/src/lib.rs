#![no_std]
extern crate alloc;

use core::time::Duration;
use heapless::FnvIndexMap;
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
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug)]
pub struct ProtocolVersion {
    /// Major version of the protocol. Incremented for breaking changes.
    pub major: u8,
    /// Minor version of the protocol. Incremented for non-breaking changes.
    pub minor: u8,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug)]
pub struct ProtocolConfig {
    /// The protocol version used by the machine.
    pub protocol_version: ProtocolVersion,
    /// The maximum number of boilers supported by the machine.
    pub max_boilers: usize,
    /// The maximum number of groups supported by the machine.
    pub max_groups: usize,
    /// The maximum number of water taps supported by the machine.
    pub max_water_taps: usize,
    /// The maximum number of tanks supported by the machine.
    pub max_tanks: usize,
    /// The maximum number of environmental temperature sensors supported by the machine.
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
#[derive(Clone, Debug)]
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
    RunRoutine(RoutineIndex, Option<FnvIndexMap<u8, f32, 8>>),
    CancelRoutine,
    EnableBoiler(BoilerIndex),
    DisableBoiler(BoilerIndex),
    TareGroupScale(GroupIndex),
    ZeroCalibrateGroupScale(GroupIndex),
    CalibrateGroupScale100g(GroupIndex),
    UpdateCommsStatus(CommsStatus),
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
        }
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BoilerControlTargetValuesUpdate {
    pub temperature: Option<TemperatureType>,
    pub pressure: Option<PressureType>,
}

/// Complete boiler control state
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum WaterDispersalPumpStrategy {
    AlwaysPump,
    NoPump,
}

// ===== GROUP BREW CONTROL TYPES =====

/// The control mode for group brewing
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
#[derive(Clone, Debug, Default)]
pub enum MachineMode {
    On,
    #[default]
    Off,
    PowerSaveStandby,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default)]
pub struct PeripheralInfo {
    pub peripheral_type: PeripheralType,
    pub is_available: bool,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug, Default)]
pub struct PeripheralStatus {
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
#[derive(Clone, Debug, Default)]
pub struct RoutineExecutionStatus {
    pub routine_index: RoutineIndex,
    pub current_step: Option<usize>,
    pub step_elapsed_time: Option<Duration>,
    pub total_elapsed_time: Option<Duration>,
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
#[derive(Clone, Debug, Default)]
pub struct Status {
    pub boiler_statuses: FnvIndexMap<BoilerIndex, BoilerStatus, MAX_BOILERS>,
    pub group_statuses: FnvIndexMap<GroupIndex, GroupStatus, MAX_GROUPS>,
    pub water_tap_statuses: FnvIndexMap<WaterTapIndex, WaterTapStatus, MAX_WATER_TAPS>,
    pub tank_statuses: FnvIndexMap<TankIndex, TankStatus, MAX_TANKS>,
    pub mode: MachineMode,
    pub routine_execution: Option<RoutineExecutionStatus>,
    pub comms_status: Option<CommsStatus>,
    pub peripheral_status: PeripheralStatus,
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
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default)]
pub struct BoilerStatus {
    pub temperature: Option<TemperatureType>,
    pub pressure: Option<PressureType>,
    pub water_level: Option<WaterLevelType>,
    pub output: Output,
    pub control_state: BoilerControlState,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default)]
pub struct GroupStatus {
    pub is_brewing: bool,
    pub three_way_valve_open: Option<bool>,
    pub brew_time: Option<Duration>,
    pub input_flow_rate: Option<FlowRateType>,
    pub input_volume: Option<InputVolumeType>,
    pub output_flow_rate: Option<FlowRateType>,
    pub output_weight: Option<WeightType>,
    pub pressure: Option<PressureType>,
    pub temperature: Option<TemperatureType>,
    pub pump_output: Output,
    pub control_state: GroupBrewControlState,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default)]
pub struct WaterTapStatus {
    pub is_dispensing: bool,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default)]
pub struct TankStatus {
    pub water_level: Option<WaterLevelType>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug, Default)]
pub struct Configuration {
    pub machine_config: MachineConfiguration,
    pub boiler_configurations: FnvIndexMap<BoilerIndex, BoilerConfiguration, MAX_BOILERS>,
    pub group_configurations: FnvIndexMap<GroupIndex, GroupConfiguration, MAX_GROUPS>,
    pub water_tap_configurations: FnvIndexMap<WaterTapIndex, WaterTapConfiguration, MAX_WATER_TAPS>,
    pub tank_configurations: FnvIndexMap<TankIndex, TankConfiguration, MAX_TANKS>,
    pub steam_wand_configurations: FnvIndexMap<SteamWandIndex, SteamWandConfiguration, MAX_STEAM_WANDS>,
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
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default)]
pub struct FillConfiguration {
    pub fill_threshold: Option<WaterLevelType>, // If none, filling is disabled
    pub pump_configuration: Option<PumpConfiguration>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct KalmanParameters {
    pub process_noise: f32,
    pub measurement_noise: f32,
    pub estimated_error: f32,
    pub posterior_estimate: f32,
}



#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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
#[derive(Clone, Debug, Default)]
pub struct MachineConfiguration {
    pub heating_element_interlock: bool,
}


#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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
#[derive(Clone, Debug, Default)]
pub struct TankConfiguration {
    pub low_level_warning_threshold: Option<WaterLevelType>,
    pub water_level_sensor_kalman_parameters: Option<KalmanParameters>,
}


#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone,  Debug)]
pub struct ExternalSensorData {
    pub id: EnvironmentalSensorId,
    pub value: f32,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug)]
pub struct CommsStatus {
    pub timestamp: Option<u64>, // Unix timestamp in seconds
    pub wifi_connected: bool,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug)]
pub enum CommsProcessorToApplicationProcessorMessage {
    Command(MachineCommand),
    CommsStatus(CommsStatus),
    RequestStatus,
    RequestMachineDefinition,
    RequestConfiguration,
    ExternalSensorUpdate(ExternalSensorData),
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
#[derive(Clone, Debug)]
pub struct BoilerDefinition {
    pub name: heapless::String<32>,
    pub boiler_type: BoilerType,
    pub sensors: heapless::Vec<SensorCapability, 8>,
    pub actuators: heapless::Vec<ActuatorCapability, 8>,
    pub control_modes: heapless::Vec<ControlModeCapability, 8>,
    pub has_fill_mechanism: bool,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug)]
pub struct GroupDefinition {
    pub name: heapless::String<32>,
    pub sensors: heapless::Vec<SensorCapability, 8>,
    pub actuators: heapless::Vec<ActuatorCapability, 8>,
    pub control_modes: heapless::Vec<ControlModeCapability, 8>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug)]
pub struct WaterTapDefinition {
    pub name: heapless::String<32>,
    pub sensors: heapless::Vec<SensorCapability, 8>,
    pub actuators: heapless::Vec<ActuatorCapability, 8>,
    pub control_modes: heapless::Vec<ControlModeCapability, 8>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug)]
pub struct SteamWandDefinition {
    pub name: heapless::String<32>,
    pub sensors: heapless::Vec<SensorCapability, 8>,
    pub actuators: heapless::Vec<ActuatorCapability, 8>,
    pub control_modes: heapless::Vec<ControlModeCapability, 8>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug)]
pub struct TankDefinition {
    pub name: heapless::String<32>,
    pub sensors: heapless::Vec<SensorCapability, 8>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug)]
pub struct PeripheralDefinition {
    pub peripheral_type: PeripheralType,
    pub location: heapless::String<32>,
    pub capabilities: heapless::Vec<SensorCapability, 8>,
    pub support_calibration: bool,
    pub via_comms_mcu: bool,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EnvironmentalSensorType {
    AmbientTemperature,
    CaseTemperature,
    ExternalTemperature,
    Humidity,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug)]
pub struct EnvironmentalSensorDefinition {
    pub name: heapless::String<32>,
    pub sensor_type: EnvironmentalSensorType,
    pub measurement_range: Option<(f32, f32)>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug)]
pub struct MachineDefinition {
    pub name: heapless::String<32>,
    pub boilers: FnvIndexMap<BoilerIndex, BoilerDefinition, MAX_BOILERS>,
    pub groups: FnvIndexMap<GroupIndex, GroupDefinition, MAX_GROUPS>,
    pub water_taps: FnvIndexMap<WaterTapIndex, WaterTapDefinition, MAX_WATER_TAPS>,
    pub tanks: FnvIndexMap<TankIndex, TankDefinition, MAX_TANKS>,
    pub steam_wands: FnvIndexMap<SteamWandIndex, SteamWandDefinition, 4>,
    pub environmental_sensors: FnvIndexMap<EnvironmentalSensorId, EnvironmentalSensorDefinition, MAX_ENVIRONMENTAL_TEMPERATURE_SENSORS>,
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
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug)]
pub enum ApplicationProcessorToCommsProcessorMessage {
    Hello(ProtocolConfig),
    Status(Status),
    MachineDefinition(MachineDefinition),
    Configuration(Configuration),
}