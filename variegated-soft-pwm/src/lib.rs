#![no_std]

use core::fmt::{Debug, Formatter};
use defmt::Format;
use embassy_time::{Duration, Instant};

#[derive(Copy, Clone)]
pub struct SoftPwm {
    cycle_duration: Duration,
    pub current_duty_cycle_percent: u8,
    next_duty_cycle_percent: u8,
    current_cycle_start: Instant,
    update_time: Instant,
}

impl Debug for SoftPwm {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(f, "SoftPwm {{ current_duty_cycle_percent: {}, next_duty_cycle_percent: {}, current_output: {} }}",
            self.current_duty_cycle_percent,
            self.next_duty_cycle_percent,
            self.get_output(),
        )
    }

}

impl Format for SoftPwm {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "SoftPwm {{ current_duty_cycle_percent: {}, next_duty_cycle_percent: {}, current_output: {} }}",
            self.current_duty_cycle_percent,
            self.next_duty_cycle_percent,
            self.get_output(),
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
            cycle_duration: cycle_duration,
            current_duty_cycle_percent: duty_cycle_percent,
            next_duty_cycle_percent: duty_cycle_percent,
            current_cycle_start: Instant::now(),
            update_time: Instant::now(),
        }
    }

    pub fn set_duty_cycle(&mut self, duty_cycle_percent: u8) {
        self.next_duty_cycle_percent = duty_cycle_percent;
    }

    pub fn update(&mut self, now: Instant) {
        let cycle_elapsed = now - self.current_cycle_start;
        if cycle_elapsed >= self.cycle_duration {
            self.current_cycle_start = now;
            self.current_duty_cycle_percent = self.next_duty_cycle_percent;
        }
        self.update_time = now;
    }

    pub fn get_output(&self) -> bool {
        let cycle_elapsed = self.update_time - self.current_cycle_start;
        let cycle_elapsed_s = cycle_elapsed.as_millis() as f32 / 1000.0;
        let cycle_duration_s = self.cycle_duration.as_millis() as f32 / 1000.0;
        let cycle_position = cycle_elapsed_s / cycle_duration_s;
        cycle_position < self.current_duty_cycle_percent as f32 / 100.0
    }
}

