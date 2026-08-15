use embassy_rp::pwm::PwmOutput;
use embedded_hal::pwm::SetDutyCycle;
use crate::DutyCycleType;
use crate::pump::{Pump, PumpError};

pub struct GpioPwmPump<'a> {
    pwm_output: PwmOutput<'a>,
    current_duty_cycle: DutyCycleType,
}

impl<'a> GpioPwmPump<'a> {
    pub fn new(pwm_output: PwmOutput<'a>) -> Self {
        GpioPwmPump {
            pwm_output,
            current_duty_cycle: 0
        }
    }
}

impl<'a> Pump for GpioPwmPump<'a> {
    fn set_duty_cycle(&mut self, duty_cycle: DutyCycleType) -> Result<(), PumpError> {
        if duty_cycle > 100 {
            return Err(PumpError::DutyCycleOutOfRange);
        }

        self.current_duty_cycle = duty_cycle;
        self.pwm_output.set_duty_cycle_percent(duty_cycle).map_err(|_| PumpError::PwmError)
    }

    fn get_duty_cycle(&self) -> DutyCycleType {
        self.current_duty_cycle
    }
}