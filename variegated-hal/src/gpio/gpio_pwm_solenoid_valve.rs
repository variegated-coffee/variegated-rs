use embassy_rp::pwm::PwmOutput;
use embedded_hal::pwm::SetDutyCycle;
use crate::{ValveMechanism, ValveMechanismError, ValveOpenType};

pub struct GpioPwmSolenoidValve<'a> {
    pwm_output: PwmOutput<'a>,
    current_valve_state: ValveOpenType,
}

impl<'a> GpioPwmSolenoidValve<'a> {
    pub fn new(mut pwm_output: PwmOutput<'a>) -> Self {
        // Initialize to closed (0% duty cycle) for safety
        let _ = pwm_output.set_duty_cycle_percent(0);

        GpioPwmSolenoidValve {
            pwm_output,
            current_valve_state: 0,
        }
    }
}

impl<'a> ValveMechanism for GpioPwmSolenoidValve<'a> {
    fn set_valve_state(&mut self, state: ValveOpenType) -> Result<(), ValveMechanismError> {
        if state > 100 {
            return Err(ValveMechanismError::UnknownError);
        }

        self.current_valve_state = state;
        self.pwm_output
            .set_duty_cycle_percent(state)
            .map_err(|_| ValveMechanismError::UnknownError)
    }

    fn get_valve_state(&self) -> ValveOpenType {
        self.current_valve_state
    }

    fn get_binary_state(&self) -> bool {
        self.current_valve_state > 0
    }
}
