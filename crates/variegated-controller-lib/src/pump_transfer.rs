//! When the pump PID is in control, and what its integral should be when it takes over.
//!
//! The pump can be driven two ways. In *open-loop* modes — [`Off`], [`FullOn`],
//! [`FixedDutyCycle`], [`FixedDutyCycleCurve`] — the duty cycle comes straight from the
//! control target and the PID has no say. In the remaining *closed-loop* modes the PID's
//! output **is** the duty cycle.
//!
//! Moving between the two is where bumpless transfer lives, and both controllers got it
//! wrong in the same two ways:
//!
//! 1. **The PID stepped in every mode**, including the open-loop ones, where `update_pump`
//!    fed it a hardcoded process value of `0.0` against whatever setpoint the last
//!    closed-loop mode had left behind. The error was therefore the entire stale setpoint,
//!    every iteration, and `delta_t` is in *milliseconds* — so with the stock `ki` of 0.01
//!    the integral gained `1.0` per bar of stale setpoint per 100 ms tick and pinned itself
//!    to the `80` clamp inside a second. Re-entering a closed-loop mode then slammed the
//!    pump to 80%.
//!
//! 2. **Nothing seeded the integral on the way in.** `MachineCommand::InferGroup*Integral`
//!    exists precisely for this, but only dual-boiler *routines* ever send it. Anything
//!    driving the modes by hand — single-boiler's manual-brew screen, the web interface,
//!    the debug link — entered a closed-loop mode with the integral at zero. Since the UI
//!    syncs the setpoint to the current measurement (so the error starts at ~0, so the P
//!    term starts at ~0) the output was `0 + 0 + 0`: the pump dropped dead the instant you
//!    left duty-cycle mode, then crawled back up over minutes.
//!
//! Both are the same missing idea — the PID has to *inherit* the output it is taking over
//! from, and must not accumulate while it does not have the output. That is what this
//! module decides. It is pure, and lives here rather than in the controller because the
//! controllers are behind the `hardware` feature and cannot be host-tested.
//!
//! # What this does not fix
//!
//! The inherited duty cycle lands in the integral accumulator, which the *next* step clamps
//! to `ki.limits` — stock `-50..80`. A transfer out of 100% duty therefore opens at 80%, not
//! 100%. `InferGroup*Integral` has always behaved this way, so dual-boiler routines that
//! pre-infuse at full duty and hand over to pressure already do it; this is not a regression
//! and the remedy is a tuning decision about `ki.limits`, not a code change.
//!
//! [`Off`]: GroupBrewControlMode::Off
//! [`FullOn`]: GroupBrewControlMode::FullOn
//! [`FixedDutyCycle`]: GroupBrewControlMode::FixedDutyCycle
//! [`FixedDutyCycleCurve`]: GroupBrewControlMode::FixedDutyCycleCurve

use variegated_controller_types::{DutyCycleType, GroupBrewControlMode};

/// Whether `mode` puts the pump PID in control of the duty cycle.
///
/// This is the *only* place the partition is written down. `update_pump` derives its output
/// branch from the same answer rather than re-listing the modes, so the two cannot drift.
pub fn is_closed_loop(mode: GroupBrewControlMode) -> bool {
    match mode {
        GroupBrewControlMode::Off
        | GroupBrewControlMode::FullOn
        | GroupBrewControlMode::FixedDutyCycle
        | GroupBrewControlMode::FixedDutyCycleCurve => false,
        GroupBrewControlMode::GroupFlowRate
        | GroupBrewControlMode::GroupFlowRateCurve
        | GroupBrewControlMode::Pressure
        | GroupBrewControlMode::PressureCurve
        | GroupBrewControlMode::OutputFlowRate
        | GroupBrewControlMode::OutputFlowRateCurve => true,
    }
}

/// What to do with the pump PID this iteration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PumpPidTransfer {
    /// The PID is taking over from open-loop control. Seed its integral so its first output
    /// matches the duty cycle already being commanded, then step it.
    Engage {
        /// The duty cycle the pump is running at right now, to be inherited.
        seed_from_duty: DutyCycleType,
    },
    /// The PID already had the output and keeps it. Step it.
    ///
    /// This covers closed-loop *to* closed-loop moves — pressure to flow rate, say. The
    /// integral is carried across deliberately: it is denominated in duty cycle, not in the
    /// process variable, so keeping it is exactly what holds the output continuous while
    /// the setpoint and the units under it change.
    Continue,
    /// The PID is not in control. Do not step it — holding the integral is what keeps it
    /// from winding up against a process value it is not driving.
    Hold,
}

/// Tracks whether the pump PID currently owns the output, and what the pump is doing.
///
/// One per group. [`record_commanded_duty`] must be called with the duty cycle actually
/// commanded each iteration, including in open-loop modes — that value is what the PID
/// inherits on the next [`Engage`].
///
/// [`record_commanded_duty`]: PumpPidEngagement::record_commanded_duty
/// [`Engage`]: PumpPidTransfer::Engage
#[derive(Debug, Clone, Copy, Default)]
pub struct PumpPidEngagement {
    engaged: bool,
    last_commanded_duty: DutyCycleType,
}

impl PumpPidEngagement {
    pub fn new() -> Self {
        Self::default()
    }

    /// Decide what happens to the PID for `mode`, and record the resulting engagement.
    pub fn transfer_for(&mut self, mode: GroupBrewControlMode) -> PumpPidTransfer {
        if !is_closed_loop(mode) {
            self.engaged = false;
            return PumpPidTransfer::Hold;
        }

        if self.engaged {
            PumpPidTransfer::Continue
        } else {
            self.engaged = true;
            PumpPidTransfer::Engage {
                seed_from_duty: self.last_commanded_duty,
            }
        }
    }

    /// Record the duty cycle actually sent to the pump this iteration.
    pub fn record_commanded_duty(&mut self, duty: DutyCycleType) {
        self.last_commanded_duty = duty;
    }

    /// The duty cycle last commanded — what an `InferGroup*Integral` should transfer from.
    ///
    /// Both controllers used to read `ephemeral.group_brew_control_state.values.duty_cycle`
    /// for this, which is the [`FixedDutyCycle`] *target* rather than the current output. It
    /// happens to be right when the transfer is from duty-cycle mode and is stale in every
    /// other case — including closed-loop to closed-loop, where it holds whatever the user
    /// last dialled into the duty-cycle screen.
    ///
    /// [`FixedDutyCycle`]: GroupBrewControlMode::FixedDutyCycle
    pub fn last_commanded_duty(&self) -> DutyCycleType {
        self.last_commanded_duty
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ALL_MODES: [GroupBrewControlMode; 10] = [
        GroupBrewControlMode::Off,
        GroupBrewControlMode::FullOn,
        GroupBrewControlMode::FixedDutyCycle,
        GroupBrewControlMode::FixedDutyCycleCurve,
        GroupBrewControlMode::GroupFlowRate,
        GroupBrewControlMode::GroupFlowRateCurve,
        GroupBrewControlMode::Pressure,
        GroupBrewControlMode::PressureCurve,
        GroupBrewControlMode::OutputFlowRate,
        GroupBrewControlMode::OutputFlowRateCurve,
    ];

    /// `is_closed_loop` is exhaustive over the enum, so a mode added later fails to compile
    /// rather than silently defaulting to one side. This holds `ALL_MODES` to the same
    /// standard, since the tests below are only as good as their coverage.
    #[test]
    fn all_modes_are_covered() {
        for mode in ALL_MODES {
            let _ = is_closed_loop(mode);
        }
        assert_eq!(ALL_MODES.len(), 10);
    }

    /// The transfer the manual-brew screen makes, and the one that was broken: duty cycle to
    /// a closed-loop mode. The PID must inherit the 60% the pump is already running at.
    #[test]
    fn engaging_from_open_loop_inherits_the_commanded_duty() {
        let mut engagement = PumpPidEngagement::new();

        assert_eq!(
            engagement.transfer_for(GroupBrewControlMode::FixedDutyCycle),
            PumpPidTransfer::Hold
        );
        engagement.record_commanded_duty(60);

        assert_eq!(
            engagement.transfer_for(GroupBrewControlMode::Pressure),
            PumpPidTransfer::Engage { seed_from_duty: 60 }
        );
    }

    /// Engaging happens once. A second iteration in the same mode must not re-seed, or the
    /// integral would be pinned to the last output forever and the PID would never correct.
    #[test]
    fn staying_in_a_closed_loop_mode_engages_only_once() {
        let mut engagement = PumpPidEngagement::new();
        engagement.record_commanded_duty(60);

        assert_eq!(
            engagement.transfer_for(GroupBrewControlMode::Pressure),
            PumpPidTransfer::Engage { seed_from_duty: 60 }
        );
        engagement.record_commanded_duty(58);

        for _ in 0..10 {
            assert_eq!(
                engagement.transfer_for(GroupBrewControlMode::Pressure),
                PumpPidTransfer::Continue
            );
        }
    }

    /// Closed loop to closed loop keeps the integral. The output is continuous because the
    /// accumulator is in duty cycle, so it survives the change of process variable.
    #[test]
    fn closed_loop_to_closed_loop_continues() {
        let mut engagement = PumpPidEngagement::new();
        engagement.record_commanded_duty(60);

        assert!(matches!(
            engagement.transfer_for(GroupBrewControlMode::Pressure),
            PumpPidTransfer::Engage { .. }
        ));
        assert_eq!(
            engagement.transfer_for(GroupBrewControlMode::GroupFlowRate),
            PumpPidTransfer::Continue
        );
        assert_eq!(
            engagement.transfer_for(GroupBrewControlMode::OutputFlowRateCurve),
            PumpPidTransfer::Continue
        );
    }

    /// The windup bug, stated as a property: the PID is never stepped in a mode that does
    /// not give it the output. Before this, `update_pump` stepped it unconditionally.
    #[test]
    fn open_loop_modes_never_step_the_pid() {
        for mode in ALL_MODES.iter().copied().filter(|m| !is_closed_loop(*m)) {
            let mut engagement = PumpPidEngagement::new();
            // Reached both from cold and from an engaged PID.
            assert_eq!(engagement.transfer_for(mode), PumpPidTransfer::Hold);
            engagement.transfer_for(GroupBrewControlMode::Pressure);
            assert_eq!(engagement.transfer_for(mode), PumpPidTransfer::Hold);
        }
    }

    /// Leaving a closed-loop mode and coming back re-seeds rather than resuming the old
    /// integral, because the pump may have been doing anything in between.
    #[test]
    fn leaving_and_returning_re_engages() {
        let mut engagement = PumpPidEngagement::new();
        engagement.record_commanded_duty(45);

        assert!(matches!(
            engagement.transfer_for(GroupBrewControlMode::Pressure),
            PumpPidTransfer::Engage { .. }
        ));
        engagement.record_commanded_duty(45);

        assert_eq!(
            engagement.transfer_for(GroupBrewControlMode::Off),
            PumpPidTransfer::Hold
        );
        engagement.record_commanded_duty(0);

        // Off commanded 0, so there is nothing to inherit -- which is right. Coming out of
        // Off is a start, not a transfer.
        assert_eq!(
            engagement.transfer_for(GroupBrewControlMode::Pressure),
            PumpPidTransfer::Engage { seed_from_duty: 0 }
        );
    }

    /// A cold controller has commanded nothing, so the first closed-loop mode inherits 0 and
    /// the PID starts from rest. Same as today's behaviour, and the only case where it was
    /// correct by accident.
    #[test]
    fn a_cold_controller_engages_from_zero() {
        let mut engagement = PumpPidEngagement::new();
        assert_eq!(
            engagement.transfer_for(GroupBrewControlMode::GroupFlowRate),
            PumpPidTransfer::Engage { seed_from_duty: 0 }
        );
    }
}
