use defmt::Format;
use pid_ctrl::PidOut;
use variegated_controller_lib::{SystemActuatorState, SystemSensorState};
use variegated_soft_pwm::SoftPwm;

#[derive(Default, Copy, Clone, Debug, Format)]
pub struct SilviaSystemSensorState {
    pub brew_switch: bool,
    pub water_switch: bool,
    pub steam_switch: bool,
    pub boiler_temp_c: f32,
    pub boiler_pressure_bar: f32,
    pub pump_rpm: f32,
    pub flow_meter_rpm: f32,
    
    pub scale_untared_g: f32,
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
pub enum SilviaExtractionPhase {
    #[default]
    Preinfusion,
    RampUp,
    Extraction,
    DecliningProfile
}

impl SilviaExtractionPhase {
    pub fn next(&self) -> SilviaExtractionPhase {
        match self {
            SilviaExtractionPhase::Preinfusion => SilviaExtractionPhase::RampUp,
            SilviaExtractionPhase::RampUp => SilviaExtractionPhase::Extraction,
            SilviaExtractionPhase::Extraction => SilviaExtractionPhase::DecliningProfile,
            SilviaExtractionPhase::DecliningProfile => SilviaExtractionPhase::Preinfusion,
        }
    }
}

#[derive(Default, Copy, Clone, Debug)]
pub struct SilviaSystemGeneralState {
    pub is_brewing: bool,
    pub steam_mode: bool,
    pub water_dispenser_mode: bool,
    pub extraction_phase: SilviaExtractionPhase,
    pub brew_time_s: Option<f32>,
    pub current_pressure_set_point: f32,
    
    pub brew_boiler_pid: PidOut<f32>,
    pub brew_pressure_pid: PidOut<f32>,
    
    pub scale_tared_g: f32,
}

#[derive(Default, Copy, Clone, Debug, Format)]
pub struct SilviaSystemConfiguration {
    pub boiler_temp_setpoint_c: f32,
    pub pump_pressure_setpoint_preinfusion_bar: f32,
    pub pump_pressure_setpoint_ramp_up_bar: f32,
    pub pump_pressure_setpoint_extraction_bar: f32,
}

impl SilviaSystemConfiguration {
    pub fn pressure_set_point_for_phase(&self, phase: SilviaExtractionPhase, brew_time: f32) -> f32 {
        match phase {
            SilviaExtractionPhase::Preinfusion => self.pump_pressure_setpoint_preinfusion_bar,
            SilviaExtractionPhase::RampUp => self.pump_pressure_setpoint_ramp_up_bar,
            SilviaExtractionPhase::Extraction => self.pump_pressure_setpoint_extraction_bar,
            SilviaExtractionPhase::DecliningProfile => self.declining_profile_pressure_at_time(brew_time),
        }
    }
    
    fn declining_profile_pressure_at_time(&self, time_s: f32) -> f32 {
        if time_s < 10.0 {
            return 2.0;
        }
        
        (-0.25 * time_s + 13.0).clamp(0.0, 9.0)
    }
}