#![no_std]
extern crate alloc;

use core::time::Duration;
use heapless::FnvIndexMap;
use variegated_control_algorithm::pid::PidOut;

const MAX_BOILERS: usize = 8;
const MAX_GROUPS: usize = 4;
const MAX_WATER_TAPS: usize = 4;

pub type TemperatureType = f32; // Celsius
pub type PressureType = f32; // Bar
pub type WaterLevelType = u8; // Percent
pub type FlowRateType = f32; // ml/s
pub type WeightType = f32; // g
pub type FrequencyType = f32; // Hz
pub type RPMType = f32; // RPM
pub type DutyCycleType = u8; // Percent
pub type ValveOpenType = u8; // Percent
pub type MixingProportionType = u8; // Percent

pub type BoilerIndex = u8;
pub type GroupIndex = u8;
pub type WaterTapIndex = u8;

pub type RoutineIndex = usize;

pub type PidParameters = variegated_control_algorithm::pid::PidParameters<f32>;
pub type PidTerm = variegated_control_algorithm::pid::PidTerm<f32>;
pub type PidLimits = variegated_control_algorithm::pid::Limits<f32>;

pub type ExternalSensorId = u8; // Unique identifier for external sensors

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
}

const PROTOCOL_VERSION: ProtocolVersion = ProtocolVersion { major: 1, minor: 0 };

const PROTOCOL_CONFIG: ProtocolConfig = ProtocolConfig {
    protocol_version: PROTOCOL_VERSION,
    max_boilers: MAX_BOILERS,
    max_groups: MAX_GROUPS,
    max_water_taps: MAX_WATER_TAPS,
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
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug)]
pub enum MachineCommand {
    StartBrewing(GroupIndex),
    StopBrewing(GroupIndex),
    StartPumpingToWaterTap(WaterTapIndex),
    StopPumpingToWaterTap(WaterTapIndex),
    SetBoilerControlTarget(BoilerIndex, BoilerControlTarget),
    SetGroupBrewControlTarget(GroupIndex, GroupBrewControlTarget),
    SetPidParameters(PidParameterTarget, PidParameters),
    RunRoutine(RoutineIndex),
    CancelRoutine,
    EnableBoiler(BoilerIndex),
    DisableBoiler(BoilerIndex),
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

    pub fn as_index(self) -> BoilerIndex {
        match self {
            SingleBoilerSingleGroupControllerBoilers::BrewBoiler => 0,
            SingleBoilerSingleGroupControllerBoilers::VirtualSteamBoiler => 1,
        }
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default)]
pub struct Status {
    pub boiler_temp: TemperatureType,
    pub boiler_pressure: Option<PressureType>,
    pub brew_boiler_duty_cycle: u8,
    pub pump_duty_cycle: u8,
    pub is_brewing: bool,
    pub group_flow_rate: Option<FlowRateType>,
    pub boiler_pid_output: Option<PidOut<f32>>,
    pub pump_pid_output: Option<PidOut<f32>>,
    pub config_brew_boiler_control_target: BoilerControlTarget,
    pub config_steam_boiler_control_target: BoilerControlTarget,
    pub config_group_brew_control_target: GroupBrewControlTarget,
    pub routine_running: bool,
    pub routine_step: Option<usize>,
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
#[derive(Clone, Debug, Default)]
pub struct NewStatus {
    pub boiler_statuses: FnvIndexMap<BoilerIndex, BoilerStatus, MAX_BOILERS>,
    pub group_statuses: FnvIndexMap<GroupIndex, GroupStatus, MAX_GROUPS>,
    pub mode: MachineMode,
    pub current_routine: Option<RoutineIndex>,
    pub routine_step: Option<usize>,
}

#[cfg(feature = "defmt")]
impl defmt::Format for NewStatus {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "NewStatus {{ mode: {:#?}, current_routine: {:#?}, routine_step: {:#?} }}",
            /*self.boiler_statuses, self.group_statuses, */self.mode, self.current_routine, self.routine_step);
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
    pub id: ExternalSensorId,
    pub value: f32,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone,  Debug)]
pub struct CommsStatus {
    pub timestamp: Option<u64>, // Unix timestamp in seconds
    pub wifi_connected: bool,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone,  Debug)]
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
    Status(NewStatus),
    MachineDefinition,
    Configuration(Configuration),
}