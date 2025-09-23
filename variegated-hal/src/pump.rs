use crate::DutyCycleType;
use defmt::Format;

#[derive(Debug, Format, Clone, Copy)]
pub enum PumpError {
    DutyCycleOutOfRange,
    PwmError,
    GpioError,
}

pub trait Pump {
    fn set_duty_cycle(&mut self, duty_cycle: DutyCycleType) -> Result<(), PumpError>;
    fn get_duty_cycle(&self) -> DutyCycleType;
}