#![no_std]

use core::fmt::{Debug, Formatter};
use defmt::Format;
use embassy_time::Duration;

#[derive(Copy, Clone, Debug, Format)]
pub struct Cycle {
    pub on_duration: Duration,
    pub off_duration: Duration,
}

/// A duty cycle as a percentage, 0-100.
///
/// Deliberately this crate's own type rather than the espresso tree's `DutyCycle`: software
/// PWM at 0.1-1 Hz is a general-purpose thing, and it should not need to know what a boiler
/// is to be used. The caller converts at the seam.
///
/// The reason it is a newtype at all is that duty cycle now exists at two scales in the
/// consuming firmware -- percent for heating elements, 0-255 for the pump -- and the two are
/// otherwise both bare `u8`. Nothing here is on the 0-255 scale, and this type is how that
/// stays true.
#[derive(Copy, Clone, Debug, Format, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct Percent(u8);

impl Percent {
    /// Fully off.
    pub const OFF: Self = Self(0);

    /// A percentage, clamped to 0-100.
    pub const fn new(percent: u8) -> Self {
        Self(if percent > 100 { 100 } else { percent })
    }

    /// The percentage, 0-100.
    pub const fn value(self) -> u8 {
        self.0
    }
}

#[derive(Copy, Clone)]
pub struct SoftPwm {
    cycle_duration: Duration,
    pub duty_cycle_percent: Percent,
}

impl Debug for SoftPwm {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(f, "SoftPwm {{ duty_cycle: {} }}",
            self.duty_cycle_percent.value(),
        )
    }
}

impl Format for SoftPwm {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "SoftPwm {{ duty_cycle_percent: {} }}",
            self.duty_cycle_percent.value(),
        );
    }
}

impl Default for SoftPwm {
    fn default() -> Self {
        SoftPwm::new(Duration::from_secs(2), Percent::OFF)
    }
}

impl SoftPwm {
    pub fn new(cycle_duration: Duration, duty_cycle_percent: Percent) -> Self {
        SoftPwm {
            cycle_duration,
            duty_cycle_percent
        }
    }

    pub fn set_duty_cycle(&mut self, duty_cycle_percent: Percent) {
        self.duty_cycle_percent = duty_cycle_percent;
    }

    pub fn get_duty_cycle(&self) -> Percent {
        self.duty_cycle_percent
    }

    pub fn get_cycle(&self) -> Cycle {
        let cycle_duration = self.cycle_duration.as_millis() as f32 / 1000.0;
        let duty = self.duty_cycle_percent.value() as f32 / 100.0;
        let on_duration = cycle_duration * duty;
        let off_duration = cycle_duration * (1.0 - duty);
        Cycle {
            on_duration: Duration::from_millis((on_duration * 1000.0) as u64),
            off_duration: Duration::from_millis((off_duration * 1000.0) as u64),
        }
    }
}
