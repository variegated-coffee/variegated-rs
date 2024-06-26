use defmt::Format;
use variegated_controller_lib::{SystemActuatorState, SystemSensorState};
use variegated_soft_pwm::SoftPwm;

#[derive(Default, Copy, Clone, Debug, Format)]
pub struct SilviaSystemSensorState {
    pub brew_switch: bool,
    pub water_switch: bool,
    pub steam_switch: bool,
    pub boiler_temp_c: f32,
    pub boiler_pressure_bar: f32,
}

impl SystemSensorState for SilviaSystemSensorState {}

#[derive(Default, Copy, Clone, Debug, Format)]
pub struct SilviaSystemActuatorState {
    pub brew_boiler_heating_element: SoftPwm,
    pub brew_solenoid: bool,
    pub pump_duty_cycle: f32,
}

impl SystemActuatorState for SilviaSystemActuatorState {}

#[derive(Default, Copy, Clone, Debug, Format)]
pub struct SilviaSystemGeneralState {
    pub is_brewing: bool,
    pub steam_mode: bool,
    pub water_dispenser_mode: bool,
}

#[derive(Default, Copy, Clone, Debug, Format)]
pub struct SilviaSystemConfiguration {
    pub boiler_temp_setpoint_c: f32,
    pub pump_pressure_setpoint_bar: f32,
}
