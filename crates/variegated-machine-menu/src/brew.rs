//! Choosing and editing the group's brew target from a menu.
//!
//! A panel that can set the brew control mode has to answer three questions, and they are the
//! same three on any machine: which modes are worth offering, what the mode after this one is,
//! and -- once a mode is chosen -- *which quantity* the user is now editing and what to call
//! it. None of that is per-machine, so it is here rather than in a firmware.
//!
//! # Why the target is one row rather than three
//!
//! [`GroupBrewControlTargetValues`] carries a pressure, a flow rate, a duty cycle and their
//! four curves at once, but only one of them is in use at a time -- the mode decides which.
//! Three fixed rows would be two rows of a four-row panel spent on quantities that do not
//! apply. [`brew_target`] is what lets a single row rename itself instead.

use variegated_controller_types::routines::parameters::ParameterUnit;
use variegated_controller_types::{
    DutyCycleType, GroupBrewControlMode, GroupBrewControlTargetValues,
};

/// The brew control modes a button panel offers, in cycle order.
///
/// A curated four of [`GroupBrewControlMode`]'s ten. The four `*Curve` variants each need a
/// `ControlCurve` -- five numbers -- which is not something four buttons or an encoder can
/// enter, and `FullOn` and `OutputFlowRate` have no target this layer knows how to bound.
/// Offering a mode whose target the panel then cannot set would be worse than not offering
/// it at all.
///
/// `Off` is last so that cycling forward from a working mode does not pass through it.
pub const OFFERED_BREW_MODES: [GroupBrewControlMode; 4] = [
    GroupBrewControlMode::Pressure,
    GroupBrewControlMode::GroupFlowRate,
    GroupBrewControlMode::FixedDutyCycle,
    GroupBrewControlMode::Off,
];

/// The mode after this one, wrapping.
///
/// A mode this layer does not offer -- set from the web, or the GS3's `FixedDutyCycle`
/// default arriving before anything has been chosen -- lands on the first offered mode
/// rather than sticking. A row that cannot be moved reads as a broken button.
pub fn next_brew_mode(current: GroupBrewControlMode) -> GroupBrewControlMode {
    match OFFERED_BREW_MODES.iter().position(|mode| *mode == current) {
        Some(index) => OFFERED_BREW_MODES[(index + 1) % OFFERED_BREW_MODES.len()],
        None => OFFERED_BREW_MODES[0],
    }
}

/// What a mode is called in a narrow value column.
///
/// Four characters at most, which is the GS3 character LCD's value field. Abbreviated for
/// that reason and not because the modes have short names.
///
/// A mode outside [`OFFERED_BREW_MODES`] reads `othr` rather than being mapped onto one of
/// the four: the machine really is doing something else, and naming it `Prs` would claim
/// otherwise on the one screen a user would check.
pub fn brew_mode_label(mode: GroupBrewControlMode) -> &'static str {
    match mode {
        GroupBrewControlMode::Pressure => "Prs",
        GroupBrewControlMode::GroupFlowRate => "Flow",
        GroupBrewControlMode::FixedDutyCycle => "Duty",
        GroupBrewControlMode::Off => "Off",
        _ => "othr",
    }
}

/// The quantity a given mode's target row edits: its name, its unit and its current value.
///
/// `None` when the mode has no target this layer can edit, which is what greys the row.
/// That covers `Off`, every curve mode and `FullOn` -- see [`OFFERED_BREW_MODES`].
///
/// The label is at most ten characters, so it fits the twelve a `{:<12}{:>4}` split leaves.
pub fn brew_target(
    mode: GroupBrewControlMode,
    values: &GroupBrewControlTargetValues,
) -> Option<(&'static str, ParameterUnit, f32)> {
    match mode {
        GroupBrewControlMode::Pressure => {
            Some(("Brew press", ParameterUnit::Bar, values.pressure))
        }
        GroupBrewControlMode::GroupFlowRate => Some((
            "Brew flow",
            ParameterUnit::MillilitersPerSecond,
            values.flow_rate,
        )),
        GroupBrewControlMode::FixedDutyCycle => Some((
            "Brew duty",
            ParameterUnit::Percent,
            values.duty_cycle.value() as f32,
        )),
        _ => None,
    }
}

/// A duty cycle as a percentage, rounded and clamped into a [`DutyCycleType`].
///
/// The rounding rule this used to spell out now lives on the type itself, in
/// [`DutyCycleType::from_f32`], because it is the same rule everywhere a duty cycle is
/// narrowed from a float -- an editor, an evaluated curve, a PID output -- and having one
/// copy per call site is how two of them ended up disagreeing.
///
/// Kept as a named function rather than inlined at the call sites because the editors read
/// better for it, and because it is the documented entry point the GS3's menu uses.
pub fn duty_cycle_from_editor(value: f32) -> DutyCycleType {
    DutyCycleType::from_f32(value)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn values() -> GroupBrewControlTargetValues {
        GroupBrewControlTargetValues {
            pressure: 9.0,
            flow_rate: 2.5,
            duty_cycle: DutyCycleType::new(80),
            ..Default::default()
        }
    }

    #[test]
    fn each_mode_names_its_own_quantity() {
        // The whole point of the renaming row: label, unit and value must move together, or
        // it shows one mode's number under another mode's name.
        let values = values();
        let cases = [
            (
                GroupBrewControlMode::Pressure,
                ("Brew press", ParameterUnit::Bar, 9.0),
            ),
            (
                GroupBrewControlMode::GroupFlowRate,
                ("Brew flow", ParameterUnit::MillilitersPerSecond, 2.5),
            ),
            (
                GroupBrewControlMode::FixedDutyCycle,
                ("Brew duty", ParameterUnit::Percent, 80.0),
            ),
        ];
        for (mode, expected) in cases {
            assert_eq!(brew_target(mode, &values), Some(expected), "for {mode:?}");
        }
    }

    #[test]
    fn a_mode_with_no_editable_target_greys_the_row() {
        let values = values();
        for mode in [
            GroupBrewControlMode::Off,
            GroupBrewControlMode::PressureCurve,
            GroupBrewControlMode::GroupFlowRateCurve,
            GroupBrewControlMode::OutputFlowRateCurve,
            GroupBrewControlMode::FixedDutyCycleCurve,
            GroupBrewControlMode::FullOn,
            GroupBrewControlMode::OutputFlowRate,
        ] {
            assert_eq!(brew_target(mode, &values), None, "for {mode:?}");
        }
    }

    #[test]
    fn cycling_visits_every_offered_mode_and_returns() {
        let mut mode = OFFERED_BREW_MODES[0];
        let mut seen = alloc::vec::Vec::new();
        for _ in 0..OFFERED_BREW_MODES.len() {
            seen.push(mode);
            mode = next_brew_mode(mode);
        }
        assert_eq!(seen, OFFERED_BREW_MODES);
        // Back where it started, so the row never dead-ends.
        assert_eq!(mode, OFFERED_BREW_MODES[0]);
    }

    #[test]
    fn off_is_last_so_cycling_forward_does_not_pass_through_it() {
        // Cycling from a working mode should reach the other working modes first. Landing on
        // Off in the middle would switch the group off on the way past.
        assert_eq!(next_brew_mode(GroupBrewControlMode::Pressure), GroupBrewControlMode::GroupFlowRate);
        assert_eq!(
            next_brew_mode(GroupBrewControlMode::GroupFlowRate),
            GroupBrewControlMode::FixedDutyCycle
        );
        assert_eq!(next_brew_mode(GroupBrewControlMode::FixedDutyCycle), GroupBrewControlMode::Off);
    }

    #[test]
    fn an_unoffered_mode_still_cycles_rather_than_sticking() {
        // The GS3's stored default is `FixedDutyCycle`, but a curve mode set from the web
        // would otherwise leave `position` as `None` and the row inert.
        for mode in [
            GroupBrewControlMode::PressureCurve,
            GroupBrewControlMode::FullOn,
            GroupBrewControlMode::OutputFlowRate,
        ] {
            assert_eq!(next_brew_mode(mode), OFFERED_BREW_MODES[0], "for {mode:?}");
        }
    }

    #[test]
    fn every_mode_label_fits_a_four_column_value_field() {
        // The GS3 character LCD truncates its row from the right, silently, so an over-wide
        // value eats nothing visible -- it just disappears.
        for mode in [
            GroupBrewControlMode::GroupFlowRate,
            GroupBrewControlMode::GroupFlowRateCurve,
            GroupBrewControlMode::Pressure,
            GroupBrewControlMode::PressureCurve,
            GroupBrewControlMode::OutputFlowRate,
            GroupBrewControlMode::OutputFlowRateCurve,
            GroupBrewControlMode::FixedDutyCycle,
            GroupBrewControlMode::FixedDutyCycleCurve,
            GroupBrewControlMode::FullOn,
            GroupBrewControlMode::Off,
        ] {
            let label = brew_mode_label(mode);
            assert!(label.len() <= 4, "{mode:?} renders as {label:?}");
        }
    }

    #[test]
    fn every_target_label_fits_the_twelve_column_label_field() {
        let values = values();
        for mode in OFFERED_BREW_MODES {
            if let Some((label, _, _)) = brew_target(mode, &values) {
                assert!(label.len() <= 12, "{mode:?} renders as {label:?}");
            }
        }
    }

    // The rounding and saturation rules themselves are tested on the type, in
    // `variegated_controller_types::duty_cycle`. These cover the editor's own contract:
    // the values an `Adjustable` bounded at 0..=100 can actually hand over.

    #[test]
    fn a_duty_cycle_one_float_step_short_of_full_still_rounds_to_full() {
        // The failure this function exists for: truncation would make the row stick at 99.
        assert_eq!(duty_cycle_from_editor(99.999_99).value(), 100);
        assert_eq!(duty_cycle_from_editor(79.999_99).value(), 80);
        assert_eq!(duty_cycle_from_editor(0.000_01).value(), 0);
    }

    #[test]
    fn duty_cycles_round_to_nearest_rather_than_down() {
        assert_eq!(duty_cycle_from_editor(50.0).value(), 50);
        assert_eq!(duty_cycle_from_editor(50.4).value(), 50);
        assert_eq!(duty_cycle_from_editor(50.5).value(), 51);
        assert_eq!(duty_cycle_from_editor(50.6).value(), 51);
    }

    #[test]
    fn duty_cycles_outside_the_range_saturate() {
        assert_eq!(duty_cycle_from_editor(-5.0).value(), 0);
        assert_eq!(duty_cycle_from_editor(0.0).value(), 0);
        assert_eq!(duty_cycle_from_editor(100.0).value(), 100);
        assert_eq!(duty_cycle_from_editor(1000.0).value(), 100);
        // `Adjustable` maps NaN to its minimum, so this should not arrive -- but a
        // saturating cast on NaN is 0, and 0% is the safe way to be wrong about a pump.
        assert_eq!(duty_cycle_from_editor(f32::NAN).value(), 0);
    }

    #[test]
    fn the_editor_bounds_for_every_target_unit_are_sane() {
        // `brew_target`'s units are fed straight to `parameter_bounds`. A unit with an
        // infinite ceiling there would give the panel an editor that never stops.
        let values = values();
        for mode in OFFERED_BREW_MODES {
            let Some((_, unit, _)) = brew_target(mode, &values) else { continue };
            let (min, max, step) = crate::parameter_bounds(Some(unit));
            assert!(min.is_finite() && max.is_finite(), "{unit:?} has an unbounded editor");
            assert!(step > 0.0, "{unit:?} has a zero step");
            assert!(max > min, "{unit:?} has an empty range");
        }
    }
}
