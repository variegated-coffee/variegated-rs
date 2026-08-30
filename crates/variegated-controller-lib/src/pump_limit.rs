//! Which controller owns the pump when a limit is armed, and what it is held to.
//!
//! A limit caps one quantity while a *different* one is being controlled — "hold 9 bar, but
//! never exceed 2.5 ml/s". The mechanism is **min-select override**: the main loop and the
//! limit loop both run every iteration against their own setpoints, and the pump gets the
//! lower of the two outputs. When the puck is loose enough that holding 9 bar would need
//! more than 2.5 ml/s, the limit loop's output falls below the main loop's and takes over;
//! as the puck tightens, the main loop comes back down and wins again.
//!
//! # Why min-select rather than a switch
//!
//! Both reference implementations treat a limit as a change of *control authority* rather
//! than a clamp on the output, and so does this — but they get there by switching
//! controllers on a threshold, and this does not switch at all. Three things follow:
//!
//! - **The handover is continuous.** There is no threshold to cross and nothing to chatter
//!   against. How sharply the limit takes over is set by the limit loop's `kp`: a high gain
//!   is a hard clamp, a low one is a wide, soft region of the sort Decent exposes as its
//!   limiter's "range of action". We get that behaviour without carrying the parameter.
//! - **A limit can never *raise* the output**, only lower it, because `min` is `min`. In
//!   particular a limit armed while the mode is [`Off`] cannot start the pump: the main
//!   output is zero and zero wins.
//! - **Open-loop modes compose for free.** "Preinfuse at fixed duty, but do not exceed
//!   4 bar" is `min(fixed_duty, limit_out)` and needs no special case, which matters
//!   because it is the commonest use of a limit there is.
//!
//! # What the caller still has to do
//!
//! The deselected loop **must be tracked** toward the selected output every iteration — see
//! [`variegated_control_algorithm::pid::PidCtrl::track_to`]. Without it the loser integrates
//! against an error it is not driving, winds up, and takes over with a step. That is the
//! same failure [`crate::pump_transfer`] describes for open-to-closed-loop transfer, except
//! it recurs every iteration instead of once.
//!
//! **Tracking anchors the loser; it must not silence it.** What is held is the *integral*,
//! which relaxes toward the selected output over the loop's own `Ti`. The loser's `kp * error`
//! stays on top of that, so its proposal sits `kp * error` above the output that won, and
//! that offset is the whole input to the next iteration's selection: it is how a loop says
//! how much room it still has. Forcing the loser's *total output* onto the selected value
//! instead — which is what `track_to` used to do — erases the offset, and the first bullet
//! above stops being true. Both loops are then pinned to whatever the pair last agreed on,
//! neither can advance, and `min` of two mutually-slaved proposals ratchets downward while
//! the limited quantity sits well short of its cap. That is not hypothetical; it is what
//! shot `v2hfh1e26wta5jvehdwfaxp8g8` did for 38 seconds, and
//! `a_limit_with_headroom_does_not_hold_the_main_loop_down` below is that shot in miniature.
//!
//! This module is pure, and lives here rather than in either controller for the reason
//! [`crate::pump_transfer`] gives: the controllers are behind the `hardware` feature and
//! cannot be host-tested, so the decisions live somewhere that can.
//!
//! [`Off`]: GroupBrewControlMode::Off

use variegated_controller_types::{
    GroupBrewControlMode, GroupBrewControlTargetValues, GroupBrewLimitMode,
};

/// Whether the limit loop should run this iteration.
///
/// **Deliberately not [`crate::pump_transfer::is_closed_loop`].** That partition asks who
/// owns the output; this asks whether a cap is meaningful, and the answers differ exactly
/// where it matters. A limit applies under the open-loop duty-cycle modes — capping a fixed
/// duty by pressure is the preinfusion case — so those are `true` here and `false` there.
///
/// [`Off`] is the one mode that is never limited. Not because `min` would do the wrong thing
/// (it would return zero, correctly), but because a limit loop stepped against a pump nobody
/// is driving accumulates error for no reason, which is what `pump_transfer` exists to stop.
///
/// Written as an exhaustive match rather than `mode != Off`, so a mode added later fails to
/// compile instead of silently picking a side.
///
/// [`Off`]: GroupBrewControlMode::Off
pub fn limit_is_active(mode: GroupBrewControlMode, limit: GroupBrewLimitMode) -> bool {
    if !limit.is_armed() {
        return false;
    }

    match mode {
        GroupBrewControlMode::Off => false,
        GroupBrewControlMode::FullOn
        | GroupBrewControlMode::FixedDutyCycle
        | GroupBrewControlMode::FixedDutyCycleCurve
        | GroupBrewControlMode::GroupFlowRate
        | GroupBrewControlMode::GroupFlowRateCurve
        | GroupBrewControlMode::Pressure
        | GroupBrewControlMode::PressureCurve
        | GroupBrewControlMode::OutputFlowRate
        | GroupBrewControlMode::OutputFlowRateCurve => true,
    }
}

/// What to do with the limit loop this iteration.
///
/// The same discipline [`crate::pump_transfer::PumpPidTransfer`] applies to the main loop,
/// and for a sharper reason. A PID that has just started has no integral, so its first
/// output is `kp * error` and nothing else — a number related to how far the quantity is
/// from its cap, and *unrelated to what the pump is currently doing*. Feed that to a
/// selector whose other input is a real duty cycle and it will usually win by accident: a
/// 1.5 ml/s cap against a group reading 0.0 produces an output of 15 on a 0-255 scale, which
/// beats anything the main loop is asking for and slams the pump shut.
///
/// Seeding it from the output already being commanded is what makes the first iteration
/// mean the same thing as the hundredth.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LimitTransfer {
    /// The limit loop is starting. Seed its integral from the output currently commanded —
    /// `infer_and_set_integral(main_output, pv)` — and then step it.
    Engage,
    /// It was already running. Step it; tracking has kept it where it belongs.
    Continue,
    /// Not limited. Do not step it, so it cannot accumulate against a quantity nobody is
    /// capping.
    Hold,
}

/// Tracks whether the limit loop is currently running. One per group.
#[derive(Debug, Clone, Copy, Default)]
pub struct LimitEngagement {
    engaged: Option<GroupBrewLimitMode>,
}

impl LimitEngagement {
    pub fn new() -> Self {
        Self::default()
    }

    /// Decide what happens to the limit loop for this mode and limit, and record it.
    ///
    /// Re-engages when the limit comes on, and **also when the armed quantity changes while
    /// it stays on**. That second case is why this remembers *which* limit rather than a
    /// bare flag: the integral is denominated in duty cycle, but the loop's setpoint, gains
    /// and process variable are not, so a controller carried straight from a pressure cap
    /// into a flow cap would be holding a number it computed for a different quantity. It is
    /// re-seeded from the commanded output instead, which means the same thing in both.
    pub fn transfer_for(
        &mut self,
        mode: GroupBrewControlMode,
        limit: GroupBrewLimitMode,
    ) -> LimitTransfer {
        if !limit_is_active(mode, limit) {
            self.engaged = None;
            return LimitTransfer::Hold;
        }

        if self.engaged == Some(limit) {
            LimitTransfer::Continue
        } else {
            self.engaged = Some(limit);
            LimitTransfer::Engage
        }
    }
}

/// The cap the armed limit is holding to, in that quantity's own unit.
///
/// `None` when nothing is armed. The values are carried for every limit at once and this
/// selects one, exactly as `mode` selects among the setpoints beside them.
pub fn limit_setpoint(
    limit: GroupBrewLimitMode,
    values: &GroupBrewControlTargetValues,
) -> Option<f32> {
    match limit {
        GroupBrewLimitMode::Unlimited => None,
        GroupBrewLimitMode::MaxPressure => Some(values.max_pressure),
        GroupBrewLimitMode::MaxGroupFlowRate => Some(values.max_group_flow_rate),
        GroupBrewLimitMode::MaxOutputFlowRate => Some(values.max_output_flow_rate),
    }
}

/// What the selector decided.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Selection {
    /// What to drive the pump at, on the pump's 0-255 scale.
    pub output: f32,
    /// Whether the *limit* loop won — that is, whether the limit is actually holding the
    /// pump back right now, as opposed to merely being armed.
    ///
    /// This is what a UI should show and what the shot log should record. "Armed" is a
    /// setting; "binding" is a thing that is happening.
    pub binding: bool,
}

/// Take the lower of the two outputs, and say which one won.
///
/// `limit` is `None` when no limit is armed or the limit loop is not running, in which case
/// the main loop's output passes through untouched.
///
/// The comparison is strict, so a limit that exactly ties the main output does **not** count
/// as binding. That is the right way round for a tie: reporting "the limit is holding you
/// back" when it is holding you to precisely where you already were would put a spurious
/// engagement in every shot log of a profile whose limit sits at its setpoint.
pub fn select(main: f32, limit: Option<f32>) -> Selection {
    match limit {
        Some(limit) if limit < main => Selection { output: limit, binding: true },
        _ => Selection { output: main, binding: false },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use variegated_control_algorithm::pid::{PidCtrl, PidIn};

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

    const ALL_LIMITS: [GroupBrewLimitMode; 4] = [
        GroupBrewLimitMode::Unlimited,
        GroupBrewLimitMode::MaxPressure,
        GroupBrewLimitMode::MaxGroupFlowRate,
        GroupBrewLimitMode::MaxOutputFlowRate,
    ];

    /// The predicates are exhaustive over their enums, so a variant added later fails to
    /// compile. This holds the fixtures to the same standard -- the tests below are only as
    /// good as their coverage.
    #[test]
    fn all_variants_are_covered() {
        assert_eq!(ALL_MODES.len(), 10);
        assert_eq!(ALL_LIMITS.len(), 4);
    }

    #[test]
    fn nothing_is_limited_while_the_pump_is_off() {
        for limit in ALL_LIMITS {
            assert!(
                !limit_is_active(GroupBrewControlMode::Off, limit),
                "{limit:?} must not run a limit loop against a pump nobody is driving"
            );
        }
    }

    #[test]
    fn an_unarmed_limit_never_runs() {
        for mode in ALL_MODES {
            assert!(!limit_is_active(mode, GroupBrewLimitMode::Unlimited));
        }
    }

    /// The preinfusion case, and the reason this predicate is not `is_closed_loop`: capping a
    /// fixed duty cycle by pressure is the commonest limit there is.
    #[test]
    fn open_loop_modes_still_take_a_limit() {
        for mode in [
            GroupBrewControlMode::FullOn,
            GroupBrewControlMode::FixedDutyCycle,
            GroupBrewControlMode::FixedDutyCycleCurve,
        ] {
            assert!(limit_is_active(mode, GroupBrewLimitMode::MaxPressure));
            // The contrast that makes the point: `pump_transfer` says the PID does *not*
            // own the output in these modes, and a limit applies anyway.
            assert!(!crate::pump_transfer::is_closed_loop(mode));
        }
    }

    #[test]
    fn each_limit_reads_its_own_value() {
        let values = GroupBrewControlTargetValues {
            max_pressure: 9.5,
            max_group_flow_rate: 2.5,
            max_output_flow_rate: 1.5,
            ..GroupBrewControlTargetValues::default()
        };

        assert_eq!(limit_setpoint(GroupBrewLimitMode::Unlimited, &values), None);
        assert_eq!(limit_setpoint(GroupBrewLimitMode::MaxPressure, &values), Some(9.5));
        assert_eq!(limit_setpoint(GroupBrewLimitMode::MaxGroupFlowRate, &values), Some(2.5));
        assert_eq!(limit_setpoint(GroupBrewLimitMode::MaxOutputFlowRate, &values), Some(1.5));
    }

    /// The default must be permissive: arming a limit nobody set a value for has to do
    /// nothing, not strangle the shot. A `0.0` default would make `MaxPressure` mean "never
    /// build pressure".
    #[test]
    fn unset_limits_default_out_of_the_way() {
        let values = GroupBrewControlTargetValues::default();

        for limit in [
            GroupBrewLimitMode::MaxPressure,
            GroupBrewLimitMode::MaxGroupFlowRate,
            GroupBrewLimitMode::MaxOutputFlowRate,
        ] {
            let setpoint = limit_setpoint(limit, &values).expect("armed");
            assert!(setpoint > 0.0, "{limit:?} defaults to {setpoint}, which would strangle the shot");
        }
    }

    /// Engaging happens once. A second iteration on the same limit must not re-seed, or the
    /// integral would be pinned to the commanded output forever and the loop would never
    /// actually limit anything.
    #[test]
    fn a_limit_engages_once_and_then_continues() {
        let mut engagement = LimitEngagement::new();
        let mode = GroupBrewControlMode::Pressure;

        assert_eq!(
            engagement.transfer_for(mode, GroupBrewLimitMode::MaxGroupFlowRate),
            LimitTransfer::Engage
        );
        assert_eq!(
            engagement.transfer_for(mode, GroupBrewLimitMode::MaxGroupFlowRate),
            LimitTransfer::Continue
        );
    }

    /// Swapping the armed quantity has to re-seed. The integral is in duty cycle, but the
    /// setpoint, gains and process variable are not -- continuing here would hold a number
    /// computed against bar while now controlling ml/s.
    #[test]
    fn changing_the_armed_quantity_re_engages() {
        let mut engagement = LimitEngagement::new();
        let mode = GroupBrewControlMode::Pressure;

        assert_eq!(
            engagement.transfer_for(mode, GroupBrewLimitMode::MaxGroupFlowRate),
            LimitTransfer::Engage
        );
        assert_eq!(
            engagement.transfer_for(mode, GroupBrewLimitMode::MaxOutputFlowRate),
            LimitTransfer::Engage
        );
    }

    /// Going through `Off` must drop the engagement, so coming back seeds from whatever the
    /// pump is doing then rather than from what it was doing before it stopped.
    #[test]
    fn stopping_the_pump_drops_the_engagement() {
        let mut engagement = LimitEngagement::new();
        let limit = GroupBrewLimitMode::MaxPressure;

        assert_eq!(engagement.transfer_for(GroupBrewControlMode::Pressure, limit), LimitTransfer::Engage);
        assert_eq!(engagement.transfer_for(GroupBrewControlMode::Off, limit), LimitTransfer::Hold);
        assert_eq!(engagement.transfer_for(GroupBrewControlMode::Pressure, limit), LimitTransfer::Engage);
    }

    #[test]
    fn the_lower_output_wins() {
        assert_eq!(select(200.0, Some(120.0)), Selection { output: 120.0, binding: true });
        assert_eq!(select(120.0, Some(200.0)), Selection { output: 120.0, binding: false });
        assert_eq!(select(120.0, None), Selection { output: 120.0, binding: false });
    }

    /// A tie is not an engagement -- see [`select`].
    #[test]
    fn a_tie_does_not_count_as_binding() {
        assert_eq!(select(120.0, Some(120.0)), Selection { output: 120.0, binding: false });
    }

    /// The safety property that falls out of `min`: a limit cannot start a stopped pump.
    #[test]
    fn a_limit_cannot_raise_the_output() {
        assert_eq!(select(0.0, Some(200.0)).output, 0.0);
    }

    /// A machine with no scale reports no output flow, so the controller feeds the limit loop
    /// a process value of zero -- permanently below the cap. It must never bind.
    ///
    /// **This does not hold for free, and the naive version of it is wrong.** A limit loop's
    /// first output is `kp * error` with no integral behind it: for a 1.5 ml/s cap against a
    /// reading of 0.0 that is 15 on a 0-255 scale, which beats a main loop asking for 120 and
    /// shuts the pump. What actually makes it safe is the pair of disciplines this module
    /// exists to state -- seed on [`LimitTransfer::Engage`], track every iteration after --
    /// so the loop sits at the commanded output and only falls below it when its own error
    /// goes negative. This test runs the whole loop rather than one step, because one step is
    /// exactly the case that misled.
    #[test]
    fn a_limit_on_an_absent_sensor_does_not_bind() {
        const MAIN: f32 = 120.0;
        let mut limit_pid = PidCtrl::<f32>::new_with_pid(10.0, 0.01, 0.0);
        limit_pid.setpoint = 1.5; // ml/s out of the group
        let pv = 0.0; // no scale

        let mut engagement = LimitEngagement::new();
        for iteration in 0..50 {
            let transfer = engagement
                .transfer_for(GroupBrewControlMode::Pressure, GroupBrewLimitMode::MaxOutputFlowRate);
            if transfer == LimitTransfer::Engage {
                limit_pid.infer_and_set_integral(MAIN, pv);
            }

            let out = limit_pid.step(PidIn::new(pv, 100.0));
            let selection = select(MAIN, Some(out.out));

            assert!(
                !selection.binding,
                "iteration {iteration}: an absent sensor made the limit bind at {}",
                out.out
            );
            assert_eq!(selection.output, MAIN);

            limit_pid.track_to(selection.output, &out, 100.0);
        }
    }

    /// The first iteration is the one that goes wrong without seeding, so state it on its own
    /// rather than trusting the loop above to have covered it.
    #[test]
    fn an_unseeded_limit_loop_would_win_by_accident() {
        let mut limit_pid = PidCtrl::<f32>::new_with_pid(10.0, 0.01, 0.0);
        limit_pid.setpoint = 1.5;

        let unseeded = limit_pid.step(PidIn::new(0.0, 100.0));
        assert!(
            select(120.0, Some(unseeded.out)).binding,
            "if this stops being true the seeding requirement has gone away and \
             `LimitTransfer::Engage` needs revisiting"
        );
    }

    /// External reset feedback, which is the half of min-select that is easy to leave out.
    /// The deselected loop is anchored to the selected output, so it neither winds up nor
    /// takes over with a step.
    ///
    /// The fixture is pure-integral, so anything the integral does shows up undiluted — and
    /// so `Ti` is zero and the anchoring is a single assignment. A loop that *has* a
    /// proportional term settles `kp * error` above the selected output instead; see
    /// `a_limit_with_headroom_does_not_hold_the_main_loop_down` for why that matters.
    #[test]
    fn tracking_holds_the_deselected_loop_at_the_selected_output() {
        let mut main = PidCtrl::<f32>::new_with_pid(0.0, 1.0, 0.0);
        main.setpoint = 9.0;

        // Ten iterations wound up against an error it is not driving.
        let mut last = main.step(PidIn::new(0.0, 1.0));
        for _ in 0..9 {
            last = main.step(PidIn::new(0.0, 1.0));
        }
        assert!(last.out > 9.0, "the fixture must actually wind up, or this proves nothing");

        // The limit has been winning all along at 40.
        main.track_to(40.0, &last, 1.0);
        let after = main.step(PidIn::new(0.0, 1.0));

        // Back at the selected output, plus the one iteration of integration it just did --
        // not the runaway value it had accumulated.
        assert!(
            (after.out - 49.0).abs() < 0.001,
            "expected ~49 (40 tracked + 9 error x 1s), got {}",
            after.out
        );
    }

    /// `track_to` keeps the D contribution out of the value it drives the integral toward,
    /// where `infer_and_set_integral` assumes it is zero. With a live derivative the two
    /// disagree by exactly that term, and only one of them leaves the controller where it was
    /// asked to be.
    ///
    /// Stated as the invariant at the *instant* of tracking -- `p + i + d == selected` --
    /// rather than by stepping again afterwards. Stepping moves both the integral and the
    /// derivative, so an assertion on the next output is really an assertion about those
    /// movements and says very little about the thing under test.
    ///
    /// The gains are chosen so that `ki * tdelta / kp` is at least 1, which puts the tracking
    /// in its snap case and lets the integral reach that target in this one call. That is
    /// deliberate: this test is about *which* value is tracked toward, and the rate at which
    /// a longer `Ti` approaches it is a separate question, covered by
    /// `a_limit_with_headroom_does_not_hold_the_main_loop_down` and by the pid crate's
    /// `a_tracked_loop_still_proposes_its_own_proportional_offset`.
    #[test]
    fn tracking_accounts_for_the_derivative() {
        let mut pid = PidCtrl::<f32>::new_with_pid(1.0, 2.0, 1.0);
        pid.setpoint = 10.0;

        // Move the measurement between steps so the D term is live.
        pid.step(PidIn::new(0.0, 1.0));
        let last = pid.step(PidIn::new(4.0, 1.0));
        assert!(last.d != 0.0, "the fixture needs a live derivative to be about anything");

        pid.track_to(50.0, &last, 1.0);
        let reconstructed = last.p + pid.ki.accumulate + last.d;
        assert!(
            (reconstructed - 50.0).abs() < 0.001,
            "track_to left the controller at {reconstructed}, not 50"
        );

        // What the one-shot transfer primitive would have left instead: short by exactly the
        // D term it assumed away.
        pid.infer_and_set_integral(50.0, 4.0);
        let naive = last.p + pid.ki.accumulate + last.d;
        assert!(
            (naive - (50.0 + last.d)).abs() < 0.001,
            "expected infer_and_set_integral to be off by the D term ({}), got {naive}",
            last.d
        );
    }

    /// The whole selector, both loops live, over a shot's worth of iterations.
    ///
    /// **This is the one test here that runs a real main loop against a real limit loop.**
    /// Everything above either pins one of them to a constant or exercises the pieces
    /// separately, and that is exactly how the defect this reproduces survived: every part
    /// behaves correctly on its own.
    ///
    /// The fixture is shot `v2hfh1e26wta5jvehdwfaxp8g8` in miniature — a profile asking for
    /// 1.6 ml/s under a 7 bar cap, on a puck too tight to give that flow at that pressure.
    /// The correct outcome is therefore *the limit doing its job*: pressure pinned at the cap,
    /// flow wherever the puck leaves it. What happened instead was pressure 0.6 bar under the
    /// cap and a pump that never moved, because tracking had flattened both loops onto one
    /// output and `min` of two mutually-slaved proposals only ever ratchets down.
    ///
    /// Three details are load-bearing:
    ///
    /// - **The disturbance.** Without variation on the main loop's measurement both the
    ///   correct and the broken rule converge on the cap, and this test passes either way.
    ///   Scale-derived flow is the noisy, stale signal in this system, so a ripple on it is
    ///   what the real machine has. It is a square wave rather than anything sampled so the
    ///   test stays deterministic and needs no float trig.
    /// - **The lag.** Pressure responds to duty through a first-order lag. With an
    ///   instantaneous plant the limit loop's gain of 20.4 against 0.0655 bar/count is a loop
    ///   gain of 1.34, which oscillates on any one-tick delay — an artifact of the fixture,
    ///   not of the machine, and one that would make this test about the wrong thing.
    /// - **The falling permeability.** The puck opens up across the shot, as measured. It is
    ///   what makes the flow setpoint reachable by the end and unreachable at the start,
    ///   which is the situation a limit exists for.
    #[test]
    fn a_limit_with_headroom_does_not_hold_the_main_loop_down() {
        // Plant, from the shot: bar per duty count, and the pressure lag.
        const DUTY_TO_PRESSURE: f32 = 0.0655;
        const TAU_MS: f32 = 400.0;
        const TDELTA: f32 = 107.0;
        const CAP: f32 = 7.0;

        let mut main = PidCtrl::<f32>::new_with_pid(12.0, 0.003, 0.0);
        main.setpoint = 1.6; // ml/s out of the group

        let mut limit = PidCtrl::<f32>::new_with_pid(20.4, 0.0102, 0.0);
        limit.kp.set_asymmetric_scale(20.4, 30.6);
        limit.setpoint = CAP;

        let mut duty = 89.0f32;
        let mut pressure = DUTY_TO_PRESSURE * duty;
        main.infer_and_set_integral(duty, 0.17 * pressure);
        limit.infer_and_set_integral(duty, pressure);

        let mut settled_peak = f32::MIN;
        for n in 0..358 {
            // The puck loosens through the shot: 1.6 ml/s needs 9.4 bar at the start and
            // 7.0 by the end.
            let permeability = 0.17 + 0.057 * (n as f32) / 358.0;
            pressure += (DUTY_TO_PRESSURE * duty - pressure) * TDELTA / TAU_MS;
            let flow = permeability * pressure;

            // What the scale reports, which is not quite what the group is doing.
            let ripple = if n % 9 < 5 { 0.18 } else { -0.18 };

            let main_out = main.step(PidIn::new(flow + ripple, TDELTA));
            let limit_out = limit.step(PidIn::new(pressure, TDELTA));

            let selection = select(main_out.out, Some(limit_out.out));
            duty = selection.output.clamp(0.0, 255.0);

            if selection.binding {
                main.track_to(selection.output, &main_out, TDELTA);
            } else {
                limit.track_to(selection.output, &limit_out, TDELTA);
            }

            if n >= 308 {
                settled_peak = settled_peak.max(pressure);
            }
        }

        // The limit has to actually be reached. The broken rule ends this run at 4.56 bar
        // with the pump backed down to 72 counts, having given up on both setpoints at once.
        assert!(
            pressure > 6.5,
            "the pump settled at {duty} counts and {pressure} bar, well under its {CAP} bar \
             cap, with the flow loop still asking for more -- neither loop was in control"
        );

        // And not blown through: a limit that overshoots is not a limit.
        assert!(
            settled_peak < CAP + 0.2,
            "pressure reached {settled_peak} bar against a {CAP} bar cap"
        );
    }

    /// The companion to the test above, and the reason it needs its disturbance: on a clean
    /// signal the selector converges on the cap regardless, so this pins the quiet case
    /// rather than proving anything about tracking. Its value is as a tripwire — if this ever
    /// starts failing, the fix above has broken the ordinary path.
    #[test]
    fn a_limit_settles_on_its_cap_when_the_signal_is_quiet() {
        const DUTY_TO_PRESSURE: f32 = 0.0655;
        const TAU_MS: f32 = 400.0;
        const TDELTA: f32 = 107.0;
        const CAP: f32 = 7.0;

        let mut main = PidCtrl::<f32>::new_with_pid(12.0, 0.003, 0.0);
        main.setpoint = 1.6;

        let mut limit = PidCtrl::<f32>::new_with_pid(20.4, 0.0102, 0.0);
        limit.kp.set_asymmetric_scale(20.4, 30.6);
        limit.setpoint = CAP;

        let mut duty = 89.0f32;
        let mut pressure = DUTY_TO_PRESSURE * duty;
        main.infer_and_set_integral(duty, 0.17 * pressure);
        limit.infer_and_set_integral(duty, pressure);

        for _ in 0..1500 {
            pressure += (DUTY_TO_PRESSURE * duty - pressure) * TDELTA / TAU_MS;
            let flow = 0.17 * pressure;

            let main_out = main.step(PidIn::new(flow, TDELTA));
            let limit_out = limit.step(PidIn::new(pressure, TDELTA));

            let selection = select(main_out.out, Some(limit_out.out));
            duty = selection.output.clamp(0.0, 255.0);

            if selection.binding {
                main.track_to(selection.output, &main_out, TDELTA);
            } else {
                limit.track_to(selection.output, &limit_out, TDELTA);
            }
        }

        assert!(
            (pressure - CAP).abs() < 0.01,
            "a puck this tight cannot reach 1.6 ml/s, so the limit should own the pump and \
             hold {CAP} bar exactly; it held {pressure}"
        );
    }
}
