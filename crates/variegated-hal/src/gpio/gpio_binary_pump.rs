use embassy_rp::gpio::Output;
use crate::{DutyCycleType, HexadecimalDutyCycleType};
use crate::pump::{Pump, PumpError};
use variegated_log::log_info;

/// The duty cycle above which the relay closes, as a percentage.
///
/// Expressed in percent and converted, rather than written as the raw `51`, because 20% is
/// the number that means something: it is the point below which a rotary pump is being
/// asked for so little that switching it on is worse than leaving it off. Writing the raw
/// value here is how this constant silently became 7.8% when the scale changed.
const DEFAULT_THRESHOLD_PERCENT: DutyCycleType = DutyCycleType::new(20);

pub struct GpioBinaryPump<'a> {
    output: Output<'a>,
    current_duty_cycle: HexadecimalDutyCycleType,
    threshold: HexadecimalDutyCycleType,
}

impl<'a> GpioBinaryPump<'a> {
    pub fn new(output: Output<'a>) -> Self {
        Self::new_with_threshold(output, DEFAULT_THRESHOLD_PERCENT.into())
    }

    pub fn new_with_threshold(output: Output<'a>, threshold: HexadecimalDutyCycleType) -> Self {
        GpioBinaryPump {
            output,
            current_duty_cycle: HexadecimalDutyCycleType::OFF,
            threshold,
        }
    }

    pub fn set_threshold(&mut self, threshold: HexadecimalDutyCycleType) {
        self.threshold = threshold;
        // Re-apply the current duty cycle with new threshold
        let _ = self.set_duty_cycle(self.current_duty_cycle);
    }
}

impl<'a> Pump for GpioBinaryPump<'a> {
    fn set_duty_cycle(&mut self, duty_cycle: HexadecimalDutyCycleType) -> Result<(), PumpError> {
        // These guards must use `self.threshold`, the same value the actuation
        // below switches on. They previously hardcoded `20` while the message
        // printed `self.threshold`, so with any non-default threshold the line
        // both fired at the wrong moment and misreported what had been crossed.
        if self.current_duty_cycle <= self.threshold && duty_cycle > self.threshold {
            log_info!("Binary pump ON (duty cycle {} > threshold {})", duty_cycle.value(), self.threshold.value());
        } else if self.current_duty_cycle > self.threshold && duty_cycle <= self.threshold {
            log_info!("Binary pump OFF (duty cycle {} <= threshold {})", duty_cycle.value(), self.threshold.value());
        }

        self.current_duty_cycle = duty_cycle;

        if duty_cycle > self.threshold {
            self.output.set_high();
        } else {
            self.output.set_low();
        }

        Ok(())
    }

    fn get_duty_cycle(&self) -> HexadecimalDutyCycleType {
        self.current_duty_cycle
    }
}
