use embassy_rp::pwm::PwmOutput;
use embedded_hal::pwm::SetDutyCycle;
use crate::HexadecimalDutyCycleType;
use crate::pump::{Pump, PumpError};

pub struct GpioPwmPump<'a> {
    pwm_output: PwmOutput<'a>,
    current_duty_cycle: HexadecimalDutyCycleType,
}

impl<'a> GpioPwmPump<'a> {
    pub fn new(pwm_output: PwmOutput<'a>) -> Self {
        GpioPwmPump {
            pwm_output,
            current_duty_cycle: HexadecimalDutyCycleType::OFF
        }
    }
}

impl<'a> Pump for GpioPwmPump<'a> {
    fn set_duty_cycle(&mut self, duty_cycle: HexadecimalDutyCycleType) -> Result<(), PumpError> {
        self.current_duty_cycle = duty_cycle;
        // `255`, not `256`: full scale has to mean fully on, and the PWM slice's own top is
        // 14999, so `v * 14999 / 255` is exact enough that no resolution is lost here.
        self.pwm_output
            .set_duty_cycle_fraction(duty_cycle.value() as u16, 255)
            .map_err(|_| PumpError::PwmError)
    }

    fn get_duty_cycle(&self) -> HexadecimalDutyCycleType {
        self.current_duty_cycle
    }
}
