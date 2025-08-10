#![no_std]
extern crate alloc;

use core::time::Duration;
use heapless::FnvIndexMap;
use variegated_control_algorithm::pid::PidOut;

pub const MAX_BOILERS: usize = 8;
pub const MAX_GROUPS: usize = 4;
pub const MAX_WATER_TAPS: usize = 4;
pub const MAX_ENVIRONMENTAL_TEMPERATURE_SENSORS: usize = 2;
pub const MAX_TANKS: usize = 1;
pub const MAX_PERIPHERALS: usize = 16;


pub type TemperatureType = f32; // Celsius
pub type PressureType = f32; // Bar
pub type WaterLevelType = u8; // Percent
pub type FlowRateType = f32; // ml/s
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
    SetBoilerControlTarget(BoilerIndex, BoilerControlTarget),
    SetGroupBrewControlTarget(GroupIndex, GroupBrewControlTarget),
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
            MachineCommand::SetBoilerControlTarget(idx, target) => defmt::write!(f, "SetBoilerControlTarget({}, {:?})", idx, target),
            MachineCommand::SetGroupBrewControlTarget(idx, target) => defmt::write!(f, "SetGroupBrewControlTarget({}, {:?})", idx, target),
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
#[derive(Clone, Copy, Debug, Default)]
pub enum BoilerControlTarget {
    Temperature(TemperatureType),
    Pressure(PressureType),
    #[default]
    Off
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default)]
pub enum GroupBrewControlTarget {
    GroupFlowRate(FlowRateType),
    Pressure(PressureType),
    OutputFlowRate(FlowRateType),
    FixedDutyCycle(u8),
    FullOn,
    #[default]
    Off
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
}

#[cfg(feature = "defmt")]
impl defmt::Format for Status {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "NewStatus {{ mode: {:#?}, routine_running: {} }}",
            self.mode, self.routine_execution.is_some());
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
    pub output: Output,
    pub control_target: BoilerControlTarget,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default)]
pub struct GroupStatus {
    pub is_brewing: bool,
    pub three_way_valve_open: Option<bool>,
    pub brew_time: Option<Duration>,
    pub input_flow_rate: Option<FlowRateType>,
    pub output_flow_rate: Option<FlowRateType>,
    pub output_weight: Option<WeightType>,
    pub pressure: Option<PressureType>,
    pub temperature: Option<TemperatureType>,
    pub pump_output: Output,
    pub control_target: GroupBrewControlTarget,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug, Default)]
pub struct Configuration {
    boiler_configuration: FnvIndexMap<BoilerIndex, BoilerConfiguration, MAX_BOILERS>,
    group_configuration: FnvIndexMap<GroupIndex, GroupConfiguration, MAX_GROUPS>,
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
pub struct BoilerConfiguration {
    pub temperature_pid_parameters: PidParameters,
    pub pressure_pid_parameters: PidParameters,
    pub control_target: BoilerControlTarget,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default)]
pub struct GroupConfiguration {
    pub flow_rate_pid_parameters: PidParameters,
    pub output_flow_rate_pid_parameters: PidParameters,
    pub pressure_pid_parameters: PidParameters,
    pub brew_control_target: GroupBrewControlTarget,
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
#[derive(Clone, Debug)]
pub enum ApplicationProcessorToCommsProcessorMessage {
    Hello(ProtocolConfig),
    Status(Status),
    MachineDefinition,
    Configuration(Configuration),
}