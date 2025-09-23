use defmt::info;
use embassy_rp::gpio::Output;
use crate::{ValveMechanism, ValveMechanismError, ValveOpenType};

pub struct GpioBinarySolenoidValve<'a> {
    output: Output<'a>
}

impl<'a> GpioBinarySolenoidValve<'a> {
    pub fn new(output: Output<'a>) -> Self {
        GpioBinarySolenoidValve {
            output
        }
    }

    pub fn get_state(&self) -> ValveOpenType {
        if self.output.is_set_high() {
            100
        } else {
            0
        }
    }
}

impl<'a> ValveMechanism for GpioBinarySolenoidValve<'a> {
    fn set_valve_state(&mut self, state: ValveOpenType) -> Result<(), ValveMechanismError> {
        if state > 20 {
            info!("Opening solenoid");
            self.output.set_high();
        } else {
            info!("Closing solenoid");
            self.output.set_low();
        }
        Ok(())
    }

    fn get_valve_state(&self) -> ValveOpenType {
        self.get_state()
    }

    fn get_binary_state(&self) -> bool {
        self.get_valve_state() > 20
    }
}