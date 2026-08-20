use crate::HexadecimalDutyCycleType;
use defmt::Format;

#[derive(Debug, Format, Clone, Copy)]
pub enum PumpError {
    PwmError,
    GpioError,
}

/// A pump, driven on the 0-255 scale rather than as a percentage.
///
/// The scale is not cosmetic. The RP2350 PWM slice behind a gear pump runs with `top =
/// 14999`, so a percent is ~150 counts and there is resolution going spare; 100 steps is
/// too coarse for the pump PID, whose smallest possible correction would otherwise be a
/// full percent of pump output. Everything an operator authors is still a percentage --
/// see `variegated_controller_types::duty_cycle` -- and the conversion happens once, in the
/// controller, on the way in.
///
/// There is deliberately no out-of-range error: every `u8` is a valid
/// [`HexadecimalDutyCycleType`], so the range check the percentage version needed has no
/// counterpart here.
pub trait Pump {
    fn set_duty_cycle(&mut self, duty_cycle: HexadecimalDutyCycleType) -> Result<(), PumpError>;
    fn get_duty_cycle(&self) -> HexadecimalDutyCycleType;
}
