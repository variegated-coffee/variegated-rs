use defmt::Format;
use embassy_rp::pwm::PwmOutput;
use embedded_hal::pwm::SetDutyCycle;
use crate::DutyCycleType;

#[derive(Debug, Format, Clone, Copy)]
pub enum GpioPwmPumpError {
    DutyCycleOutOfRange,
    PwmError
}

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

    pub fn set_duty_cycle(&mut self, duty_cycle: DutyCycleType) -> Result<(), GpioPwmPumpError> {
        self.current_duty_cycle = duty_cycle;
        self.pwm_output.set_duty_cycle_percent(duty_cycle).map_err(|_| GpioPwmPumpError::PwmError)
    }
    
    pub fn get_duty_cycle(&self) -> DutyCycleType {
        self.current_duty_cycle
    }
}