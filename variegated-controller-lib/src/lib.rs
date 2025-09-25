#![no_std]

pub mod routine;
pub mod settings;
pub mod single_boiler_single_group;
pub mod dual_boiler_single_group;

extern crate alloc;

use variegated_control_algorithm::pid::PidCtrl;

pub fn limited_pid() -> PidCtrl<f32> {
    let mut pid = PidCtrl::default();
    pid.limits.try_set_lower(0.0).unwrap();
    pid.limits.try_set_upper(100.0).unwrap();
    pid
}

