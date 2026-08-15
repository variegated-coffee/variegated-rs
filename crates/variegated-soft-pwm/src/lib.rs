#![no_std]

use core::fmt::{Debug, Formatter};
use defmt::Format;
use embassy_time::Duration;

#[derive(Copy, Clone, Debug, Format)]
pub struct Cycle {
    pub on_duration: Duration,
    pub off_duration: Duration,
}

#[derive(Copy, Clone)]
pub struct SoftPwm {
    cycle_duration: Duration,
    pub duty_cycle_percent: u8,
}

impl Debug for SoftPwm {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(f, "SoftPwm {{ duty_cycle: {} }}",
            self.duty_cycle_percent,
        )
    }
}

impl Format for SoftPwm {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "SoftPwm {{ duty_cycle_percent: {} }}",
            self.duty_cycle_percent,
        );
    }
}

impl Default for SoftPwm {
    fn default() -> Self {
        SoftPwm::new(Duration::from_secs(2), 0)
    }
}

impl SoftPwm {
    pub fn new(cycle_duration: Duration, duty_cycle_percent: u8) -> Self {
        SoftPwm {
            cycle_duration,
            duty_cycle_percent
        }
    }

    pub fn set_duty_cycle(&mut self, duty_cycle_percent: u8) {
        self.duty_cycle_percent = duty_cycle_percent;
    }
    
    pub fn get_duty_cycle(&self) -> u8 {
        self.duty_cycle_percent
    }
    
    pub fn get_cycle(&self) -> Cycle {
        let cycle_duration = self.cycle_duration.as_millis() as f32 / 1000.0;
        let on_duration = cycle_duration * (self.duty_cycle_percent as f32 / 100.0);
        let off_duration = cycle_duration * (1.0 - (self.duty_cycle_percent as f32 / 100.0));
        Cycle {
            on_duration: Duration::from_millis((on_duration * 1000.0) as u64),
            off_duration: Duration::from_millis((off_duration * 1000.0) as u64),
        }
    }
}

