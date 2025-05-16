use defmt::info;
use embassy_rp::gpio::Output;

pub struct GpioThreeWaySolenoid<'a> {
    output: Output<'a>
}

impl<'a> GpioThreeWaySolenoid<'a> {
    pub fn new(output: Output<'a>) -> Self {
        GpioThreeWaySolenoid {
            output
        }
    }

    pub fn set_state(&mut self, state: bool) {
        if state {
            info!("Opening solenoid");
            self.output.set_high();
        } else {
            info!("Closing solenoid");
            self.output.set_low();
        }
    }
}