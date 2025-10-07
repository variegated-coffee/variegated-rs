#![no_std]

pub mod routine;
pub mod settings;
pub mod single_boiler_single_group;
pub mod dual_boiler_single_group;
pub mod schedule;
mod shot_log;

extern crate alloc;

use core::time::Duration;
use embassy_time::Instant;
use variegated_control_algorithm::pid::PidCtrl;
use variegated_controller_types::{InputVolumeType, WeightType};

/// Internal PreviousBrewInfo with Instant timestamps for controller use
#[derive(Clone, Copy, Debug)]
pub struct PreviousBrewInfo {
    pub brew_time: Duration,
    pub brew_input_volume: Option<InputVolumeType>,
    pub output_weight: Option<WeightType>,
    pub started_at: Instant,
    pub stopped_at: Instant,
}

impl From<PreviousBrewInfo> for variegated_controller_types::PreviousBrewInfo {
    fn from(info: PreviousBrewInfo) -> Self {
        variegated_controller_types::PreviousBrewInfo {
            brew_time: info.brew_time,
            brew_input_volume: info.brew_input_volume,
            output_weight: info.output_weight,
            started_at_millis: info.started_at.as_millis(),
            stopped_at_millis: info.stopped_at.as_millis(),
        }
    }
}

pub fn limited_pid() -> PidCtrl<f32> {
    let mut pid = PidCtrl::default();
    pid.limits.try_set_lower(0.0).unwrap();
    pid.limits.try_set_upper(100.0).unwrap();
    pid
}

