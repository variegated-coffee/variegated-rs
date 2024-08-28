use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug)]
pub struct UpdateSettings {
    pub boiler_setpoint: f32,
    pub pump_target: f32,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct PertinentData {
    pub boiler_temp: f32,
    pub boiler_pressure: f32,
    pub is_brewing: bool,
    pub boiler_duty_cycle: f32,
    pub pump_duty_cycle: f32,
}