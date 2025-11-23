//! API request/response types for HTTP endpoints

use alloc::collections::BTreeMap;
use alloc::string::String;
use serde::{Serialize, Deserialize};
use variegated_controller_types::{
    Routine, BoilerControlMode, GroupBrewControlMode,
    ControlCurve, PidParameters, PumpConfiguration
};

/// Response structure for categorized routines
#[derive(Serialize, Deserialize)]
pub struct RoutineStorage {
    pub internal: BTreeMap<u32, Routine>,
    pub function: BTreeMap<u32, Routine>,
    pub custom: BTreeMap<u32, Routine>,
}

/// Request to set boiler control mode and targets
#[derive(Serialize, Deserialize)]
pub struct SetBoilerControlRequest {
    pub boiler_index: u8,
    pub mode: BoilerControlMode,
    pub target_temperature: Option<f32>,
    pub target_pressure: Option<f32>,
}

/// Request to set group brew control mode and targets
#[derive(Serialize, Deserialize)]
pub struct SetGroupControlRequest {
    pub group_index: u8,
    pub mode: GroupBrewControlMode,
    pub duty_cycle: Option<u8>,
    pub flow_rate: Option<f32>,
    pub pressure: Option<f32>,
    pub output_flow_rate: Option<f32>,
    pub duty_cycle_curve: Option<ControlCurve>,
    pub flow_rate_curve: Option<ControlCurve>,
    pub pressure_curve: Option<ControlCurve>,
    pub output_flow_rate_curve: Option<ControlCurve>,
}

/// Request to set PID parameters for a controller
#[derive(Serialize, Deserialize)]
pub struct SetPidParametersRequest {
    pub target_type: String,
    pub index: u32,
    pub pid_parameters: PidParameters,
}

/// Request to set group pump configuration
#[derive(Serialize, Deserialize)]
pub struct SetGroupPumpConfigurationRequest {
    pub group_index: u8,
    pub pump_configuration: PumpConfiguration,
}

/// Request to set water tap pump configuration
#[derive(Serialize, Deserialize)]
pub struct SetWaterTapPumpConfigurationRequest {
    pub water_tap_index: u8,
    pub pump_configuration: PumpConfiguration,
}

/// Request to set boiler fill pump configuration
#[derive(Serialize, Deserialize)]
pub struct SetFillPumpConfigurationRequest {
    pub boiler_index: u8,
    pub pump_configuration: PumpConfiguration,
}

/// Request to set steam valve openness
#[derive(Serialize, Deserialize)]
pub struct SetSteamValveOpennessRequest {
    pub steam_wand_index: u8,
    pub openness: u8,
}
