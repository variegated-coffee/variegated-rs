use embassy_rp::peripherals::PWM_SLICE4;
use embassy_rp::pwm::{Slice, Config, Pwm};
use fixed::traits::ToFixed;
use crate::state::SilviaSystemActuatorState;

pub struct SilviaPwmOutputCluster<'a> {
    pwm: PwmSlice<'a>
}

impl<'a> SilviaPwmOutputCluster<'a> {
    pub fn new(pwm: Pwm<'a>) -> Self {
        let slice = PwmSlice::new(10_000, 0.0, 0.0, pwm);
        
        Self { 
            pwm: slice
        }
    }
    
    pub(crate) async fn update_from_actuator_state(&mut self, actuator_state: &SilviaSystemActuatorState) {
        //self.pwm.set_duty_cycle_a(actuator_state.pump_duty_cycle);
        self.pwm.set_duty_cycle_b(actuator_state.pump_duty_cycle);
    }
}

pub(crate) struct PwmSlice<'a> {
    frequency: u32,
    duty_cycle_a: PwmDutyCyclePercent,
    duty_cycle_b: PwmDutyCyclePercent,
    pwm: Pwm<'a>
}

type PwmDutyCyclePercent = f32;

impl<'a> PwmSlice<'a> {
    pub(crate) fn new(frequency: u32, duty_cycle_a: PwmDutyCyclePercent, duty_cycle_b: PwmDutyCyclePercent, pwm: Pwm<'a>) -> Self {
        let mut slice = PwmSlice {
            frequency,
            duty_cycle_a,
            duty_cycle_b,
            pwm
        };

        slice.update_pwm_config();

        slice
    }

    pub(crate) fn set_frequency(&mut self, frequency: u32) {
        self.frequency = frequency;
        self.update_pwm_config();
    }

    pub(crate) fn set_duty_cycle_a(&mut self, duty_cycle_a: PwmDutyCyclePercent) {
        self.duty_cycle_a = duty_cycle_a;
        self.update_pwm_config();
    }

    pub(crate) fn set_duty_cycle_b(&mut self, duty_cycle_b: PwmDutyCyclePercent) {
        self.duty_cycle_b = duty_cycle_b;
        self.update_pwm_config();
    }

    fn update_pwm_config(&mut self){
        let mut c: Config = Default::default();

        let (top, div) = calculate_top_div(self.frequency, false);
        c.top = top;
        c.divider = div.to_fixed();

        let duty_a = (self.duty_cycle_a / 100.0 * top as f32) as u16;
        let duty_b = (self.duty_cycle_b / 100.0 * top as f32) as u16;

        c.compare_a = duty_a;
        c.compare_b = duty_b;

        self.pwm.set_config(&c);
    }
}

fn calculate_top_div(freq: u32, phase_correct: bool) -> (u16, fixed::FixedU16<fixed::types::extra::U4>)
{
    let freq_cpu: u32 = 125_000_000;

    let div = match freq {
        2000.. => 1,
        200..=2000 => 10,
        20..=200 => 100,
        10..=20 => 200,
        _ => 255
    };

    let div_fixed: fixed::FixedU16<fixed::types::extra::U4> = div.to_fixed();

    let mut top = (freq_cpu / freq / div) - 1;
    let _actual_frequency = freq_cpu / ((top + 1) * div);

    if phase_correct {
        top /= 2;
    }

    (top as u16, div_fixed)
}
