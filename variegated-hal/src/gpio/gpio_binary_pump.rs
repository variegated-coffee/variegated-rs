use embassy_rp::gpio::Output;
use crate::DutyCycleType;
use crate::pump::{Pump, PumpError};
use variegated_log::log_info;

pub struct GpioBinaryPump<'a> {
    output: Output<'a>,
    current_duty_cycle: DutyCycleType,
    threshold: DutyCycleType,
}

impl<'a> GpioBinaryPump<'a> {
    pub fn new(output: Output<'a>) -> Self {
        Self::new_with_threshold(output, 20)
    }

    pub fn new_with_threshold(output: Output<'a>, threshold: DutyCycleType) -> Self {
        GpioBinaryPump {
            output,
            current_duty_cycle: 0,
            threshold,
        }
    }

    pub fn set_threshold(&mut self, threshold: DutyCycleType) {
        self.threshold = threshold;
        // Re-apply the current duty cycle with new threshold
        let _ = self.set_duty_cycle(self.current_duty_cycle);
    }
}

impl<'a> Pump for GpioBinaryPump<'a> {
    fn set_duty_cycle(&mut self, duty_cycle: DutyCycleType) -> Result<(), PumpError> {
        if duty_cycle > 100 {
            return Err(PumpError::DutyCycleOutOfRange);
        }

        // These guards must use `self.threshold`, the same value the actuation
        // below switches on. They previously hardcoded `20` while the message
        // printed `self.threshold`, so with any non-default threshold the line
        // both fired at the wrong moment and misreported what had been crossed.
        if self.current_duty_cycle <= self.threshold && duty_cycle > self.threshold {
            log_info!("Binary pump ON (duty cycle {} > threshold {})", duty_cycle, self.threshold);
        } else if self.current_duty_cycle > self.threshold && duty_cycle <= self.threshold {
            log_info!("Binary pump OFF (duty cycle {} <= threshold {})", duty_cycle, self.threshold);
        }

        self.current_duty_cycle = duty_cycle;

        if duty_cycle > self.threshold {
            self.output.set_high();
        } else {
            self.output.set_low();
        }

        Ok(())
    }

    fn get_duty_cycle(&self) -> DutyCycleType {
        self.current_duty_cycle
    }
}