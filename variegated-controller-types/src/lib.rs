#![no_std]
extern crate alloc;

use alloc::vec::Vec;
use defmt::Format;
use variegated_control_algorithm::pid::PidOut;

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

pub type PidParameters = variegated_control_algorithm::pid::PidParameters<f32>;
pub type PidTerm = variegated_control_algorithm::pid::PidTerm<f32>;
pub type PidLimits = variegated_control_algorithm::pid::Limits<f32>;

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Copy, Debug, Format)]
pub enum PidParameterTarget {
    BoilerTemperature(BoilerIndex),
    BoilerPressure(BoilerIndex),
    GroupFlowRate(GroupIndex),
    GroupOutputFlowRate(GroupIndex),
    GroupPressure(GroupIndex),
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Copy, Debug, Format)]
pub enum MachineCommand {
    StartBrewing(GroupIndex),
    StopBrewing(GroupIndex),
    StartPumpingToWaterTap(WaterTapIndex),
    StopPumpingToWaterTap(WaterTapIndex),
    SetBoilerControlTarget(BoilerIndex, BoilerControlTarget),
    SetGroupBrewControlTarget(GroupIndex, GroupBrewControlTarget),
    SetPidParameters(PidParameterTarget, PidParameters),
    RunRoutine,
    CancelRoutine,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Copy, Debug, Format, Default)]
pub enum BoilerControlTarget {
    Temperature(TemperatureType),
    Pressure(PressureType),
    #[default]
    Off
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Copy, Debug, Format, Default)]
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
#[derive(Clone, Copy, Default, Debug, Format, PartialEq)]
pub enum SingleBoilerSingleGroupControllerState {
    #[default]
    BrewModeIdle,
    SteamModeIdle,
    Brewing,
    PumpingToWaterTap,
    PowerSave,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Copy, Debug, Format)]
#[repr(u8)]
pub enum SingleBoilerSingleGroupControllerBoilers {
    BrewBoiler = 0,
    VirtualSteamBoiler = 1,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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
}

pub struct Configuration {
    boiler_configuration: Vec<BoilerConfiguration>,
    group_configuration: Vec<GroupConfiguration>,
}

pub struct BoilerConfiguration {
    pub index: BoilerIndex,
    pub temperature_pid_parameters: PidParameters,
    pub pressure_pid_parameters: PidParameters,
    pub control_target: BoilerControlTarget,
}

pub struct GroupConfiguration {
    pub index: GroupIndex,
    pub flow_rate_pid_parameters: PidParameters,
    pub output_flow_rate_pid_parameters: PidParameters,
    pub pressure_pid_parameters: PidParameters,
    pub brew_control_target: GroupBrewControlTarget,
}

