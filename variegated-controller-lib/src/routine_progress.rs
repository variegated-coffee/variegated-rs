//! How far a routine step is from its exit condition.
//!
//! A step ends when its [`RoutineExitCondition`] is met, and every display shows the user
//! how close that is: "9.2 > 9.0 bar", "23 > 30 g". Answering that needs the live value
//! from [`Status`] and the target from a [`ParameterValue`] -- and those two lookups are
//! logic, not rendering, which is why they are here and only the formatting is left to
//! each screen.
//!
//! # Why this exists
//!
//! Three renderers computed it independently -- the single-boiler's OLED, the dual-boiler's
//! character LCD, and its TFT -- and the three had diverged in three ways:
//!
//! * They resolved `ParameterValue::DerivedParameter` differently. Two returned `0.0`; the
//!   third looked the derived index up in the *base* parameter map, which is a separate
//!   index space, so it returned zero or an unrelated parameter's value. It is now
//!   [`crate::routine::resolve_parameter_value`], which evaluates the formula.
//! * One of them **ignored the boiler and group index carried by the condition** and always
//!   read the brew boiler and the single group. Harmless on a one-boiler machine, wrong the
//!   moment a condition names anything else.
//! * They covered 25, 13 and 8 `StateCondition` arms respectively, so the three screens
//!   disagreed about which conditions could show progress at all. The `match` below is
//!   exhaustive, so a new variant is a compile error in one place rather than silence on
//!   two displays.

use variegated_controller_types::{
    BoilerIndex, GroupIndex, ParameterUnit, ParameterValue, Routine, RoutineExecutionStatus,
    RoutineExitCondition, StateCondition, Status, WaterTapIndex,
};

use crate::routine::resolve_parameter_value;

/// Where a step stands against its exit condition.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ExitProgress {
    /// The live process value, or `None` when the machine is not reporting one -- an
    /// unconnected scale, a sensor a board does not have. Distinct from zero, which is a
    /// measurement.
    pub current: Option<f32>,
    /// What the condition is waiting for.
    pub target: f32,
    /// What the two numbers are measured in, so a renderer can label them without a second
    /// `match` on the condition.
    pub unit: ParameterUnit,
}

/// The current value and target for a step's exit condition, or `None` when the condition
/// has no numeric progress to show.
///
/// `routine` supplies the derived-parameter formulas, which [`Status`] does not carry.
/// Pass `None` when the definition is not to hand: a derived target then resolves to
/// `0.0` rather than to a wrong number, and the caller can tell the user the target is
/// unknown. Every other kind of target is unaffected.
pub fn exit_condition_progress(
    condition: &RoutineExitCondition,
    status: &Status,
    routine: Option<&Routine>,
) -> Option<ExitProgress> {
    let execution = status.routine_execution.as_ref();

    match condition {
        // Neither of these is a measurement against a threshold -- they fire immediately or
        // never -- so there is no progress to render.
        RoutineExitCondition::Always | RoutineExitCondition::Never => None,

        // A step timer. The elapsed side comes from the execution status rather than from a
        // sensor.
        RoutineExitCondition::After(target) => Some(ExitProgress {
            current: execution
                .and_then(|e| e.step_elapsed_time)
                .map(|d| d.as_secs_f32()),
            target: resolve(target, status, routine),
            unit: ParameterUnit::Seconds,
        }),

        // A brew timer, measured from when the group started rather than from the step.
        RoutineExitCondition::AfterDurationRelativeToStart(target) => Some(ExitProgress {
            current: status
                .get_group_status(default_group(execution))
                .and_then(|g| g.current_brew.as_ref())
                .map(|b| b.brew_time.as_secs_f32()),
            target: resolve(target, status, routine),
            unit: ParameterUnit::Seconds,
        }),

        // Waiting for a person. Nothing to measure.
        RoutineExitCondition::UserAction(_) => None,

        RoutineExitCondition::StateConditionMet(state) => state_progress(state, status, routine),
    }
}

fn state_progress(
    state: &StateCondition,
    status: &Status,
    routine: Option<&Routine>,
) -> Option<ExitProgress> {
    // Every arm reads the index *from the condition*. That is the fix for the renderer
    // which discarded it and always read the brew boiler and the single group.
    match state {
        // Boolean, not a threshold.
        StateCondition::Brewing(_) | StateCondition::NotBrewing(_) => None,

        StateCondition::BoilerTemperatureAbove(boiler, target)
        | StateCondition::BoilerTemperatureBelow(boiler, target) => Some(ExitProgress {
            current: boiler_status(status, *boiler).and_then(|b| b.temperature),
            target: resolve(target, status, routine),
            unit: ParameterUnit::Celsius,
        }),

        StateCondition::BoilerPressureAbove(boiler, target)
        | StateCondition::BoilerPressureBelow(boiler, target) => Some(ExitProgress {
            current: boiler_status(status, *boiler).and_then(|b| b.pressure),
            target: resolve(target, status, routine),
            unit: ParameterUnit::Bar,
        }),

        StateCondition::GroupInputFlowRateAbove(group, target)
        | StateCondition::GroupInputFlowRateBelow(group, target) => Some(ExitProgress {
            current: status.get_group_status(*group).and_then(|g| g.input_flow_rate),
            target: resolve(target, status, routine),
            unit: ParameterUnit::MillilitersPerSecond,
        }),

        StateCondition::GroupPressureAbove(group, target)
        | StateCondition::GroupPressureBelow(group, target) => Some(ExitProgress {
            current: status.get_group_status(*group).and_then(|g| g.pressure),
            target: resolve(target, status, routine),
            unit: ParameterUnit::Bar,
        }),

        // `current` is always `None`: `WaterTapStatus` carries only `is_dispensing`, so the
        // machine does not report a tap's flow rate at all. The target is still worth
        // returning -- a display can say what the step is waiting for even when it cannot
        // say how close it is -- and this is the honest way to express that, where a `0.0`
        // would read as a measurement.
        StateCondition::WaterTapFlowRateAbove(_tap, target)
        | StateCondition::WaterTapFlowRateBelow(_tap, target) => Some(ExitProgress {
            current: None,
            target: resolve(target, status, routine),
            unit: ParameterUnit::MillilitersPerSecond,
        }),

        StateCondition::OutputWeightAbove(group, target)
        | StateCondition::OutputWeightBelow(group, target) => Some(ExitProgress {
            current: status.get_group_status(*group).and_then(|g| g.output_weight),
            target: resolve(target, status, routine),
            unit: ParameterUnit::Grams,
        }),

        // Volume *since the brew started*, which the group status already reports
        // relative to that point -- see `current_brew.brew_input_volume`.
        StateCondition::InputVolumeAboveRelativeToStart(group, target) => Some(ExitProgress {
            current: status
                .get_group_status(*group)
                .and_then(|g| g.current_brew.as_ref())
                .and_then(|b| b.brew_input_volume),
            target: resolve(target, status, routine),
            unit: ParameterUnit::Milliliters,
        }),
    }
}

/// The target, resolved against the running routine's parameters.
///
/// Falls back to the base map alone when there is no `Routine` to hand, which affects only
/// `DerivedParameter` targets.
fn resolve(pv: &ParameterValue, status: &Status, routine: Option<&Routine>) -> f32 {
    let empty = variegated_controller_types::RoutineParameters::new();
    let parameters = status
        .routine_execution
        .as_ref()
        .map(|e| &e.resolved_parameters)
        .unwrap_or(&empty);

    let derived = routine.map(|r| r.derived_parameters.as_slice()).unwrap_or(&[]);

    resolve_parameter_value(pv, parameters, derived)
}

fn boiler_status(
    status: &Status,
    boiler: BoilerIndex,
) -> Option<&variegated_controller_types::BoilerStatus> {
    status.get_boiler_status(boiler)
}

/// The group a brew-relative timer is measured against.
///
/// `AfterDurationRelativeToStart` names no group, unlike every `StateCondition`, so this is
/// the one place a default is unavoidable. Group 0 is the only group on both machines this
/// firmware supports.
fn default_group(_execution: Option<&RoutineExecutionStatus>) -> GroupIndex {
    0
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::string::ToString;
    use alloc::vec;
    use variegated_controller_types::{
        BoilerStatus, DerivedFormula, DerivedParameter, GroupStatus, RoutineIndex,
        RoutineParameters, RoutineType,
    };

    /// A routine whose only interesting feature is its derived parameters.
    ///
    /// Base parameter 0 is 18.0 (a dose); the three derived parameters are the ratios a
    /// real routine would express against it.
    fn routine_with_derived() -> Routine {
        Routine {
            routine_type: RoutineType::UserDefined,
            name: "Ratio".to_string(),
            parameters: vec![],
            derived_parameters: vec![
                DerivedParameter {
                    index: 0,
                    name: "Yield".to_string(),
                    unit: None,
                    formula: DerivedFormula::Linear { base_param: 0, multiplier: 2.0, offset: 1.0 },
                },
                DerivedParameter {
                    index: 1,
                    name: "Total".to_string(),
                    unit: None,
                    formula: DerivedFormula::Sum { params: vec![0, 1] },
                },
                DerivedParameter {
                    index: 2,
                    name: "Headroom".to_string(),
                    unit: None,
                    formula: DerivedFormula::Difference { param_a: 1, param_b: 0 },
                },
            ],
            steps: vec![],
            finally: vec![],
        }
    }

    fn parameters() -> RoutineParameters {
        let mut p = RoutineParameters::new();
        let _ = p.insert(0, 18.0);
        let _ = p.insert(1, 40.0);
        p
    }

    fn status_running() -> Status {
        let mut status = Status::default();
        status.routine_execution = Some(RoutineExecutionStatus {
            routine_index: RoutineIndex::Custom(0),
            current_step: Some(0),
            step_elapsed_time: Some(core::time::Duration::from_secs(4)),
            total_elapsed_time: Some(core::time::Duration::from_secs(9)),
            resolved_parameters: parameters(),
        });
        status
    }

    #[test]
    fn a_derived_target_is_computed_from_its_formula() {
        // The bug this replaced: one renderer looked a *derived* index up in the *base*
        // parameter map. The two are separate index spaces, so derived 0 would have
        // returned base 0 -- 18.0, an unrelated parameter's value -- rather than 37.0.
        let routine = routine_with_derived();
        let mut status = status_running();
        let _ = status.boiler_statuses.insert(
            0,
            BoilerStatus { temperature: Some(93.0), ..Default::default() },
        );

        let progress = exit_condition_progress(
            &RoutineExitCondition::StateConditionMet(StateCondition::BoilerTemperatureAbove(
                0,
                ParameterValue::DerivedParameter(0),
            )),
            &status,
            Some(&routine),
        )
        .expect("temperature conditions report progress");

        assert_eq!(progress.current, Some(93.0));
        assert_eq!(progress.target, 37.0, "18.0 * 2.0 + 1.0");
        assert_ne!(progress.target, 18.0, "that would be base parameter 0, the old bug");
    }

    #[test]
    fn every_derived_formula_agrees_with_the_controller() {
        // The display path and the controller path are the same function now. This asserts
        // the arithmetic of each formula against the parameters above, so a change to one
        // cannot silently move the other.
        let routine = routine_with_derived();
        let params = parameters();
        let derived = &routine.derived_parameters;

        assert_eq!(resolve_parameter_value(&ParameterValue::DerivedParameter(0), &params, derived), 37.0);
        assert_eq!(resolve_parameter_value(&ParameterValue::DerivedParameter(1), &params, derived), 58.0);
        assert_eq!(resolve_parameter_value(&ParameterValue::DerivedParameter(2), &params, derived), 22.0);
        // An index with no formula. Not an error -- a malformed routine, and nothing better
        // to return from a pure function.
        assert_eq!(resolve_parameter_value(&ParameterValue::DerivedParameter(9), &params, derived), 0.0);
    }

    #[test]
    fn a_derived_target_without_the_routine_does_not_invent_a_number() {
        // `Status` does not carry the formulas, so a caller that has not fetched the routine
        // gets 0.0 rather than a plausible wrong target read out of the base map.
        let status = status_running();

        let progress = exit_condition_progress(
            &RoutineExitCondition::StateConditionMet(StateCondition::BoilerTemperatureAbove(
                0,
                ParameterValue::DerivedParameter(0),
            )),
            &status,
            None,
        )
        .expect("temperature conditions report progress");

        assert_eq!(progress.target, 0.0);
    }

    #[test]
    fn the_condition_names_which_boiler_to_read() {
        // The single-boiler's renderer discarded the index and always read the brew boiler.
        // Harmless on a one-boiler machine; wrong the moment a condition names another.
        let mut status = status_running();
        let _ = status.boiler_statuses.insert(
            0,
            BoilerStatus { temperature: Some(93.0), ..Default::default() },
        );
        let _ = status.boiler_statuses.insert(
            1,
            BoilerStatus { temperature: Some(128.0), ..Default::default() },
        );

        let steam = exit_condition_progress(
            &RoutineExitCondition::StateConditionMet(StateCondition::BoilerTemperatureAbove(
                1,
                ParameterValue::Static(125.0),
            )),
            &status,
            None,
        )
        .expect("temperature conditions report progress");

        assert_eq!(steam.current, Some(128.0), "boiler 1, not boiler 0");
    }

    #[test]
    fn an_unreported_sensor_is_absent_rather_than_zero() {
        // A scale that is not connected reports nothing. Zero is a weight; `None` is not,
        // and a display must be able to tell them apart.
        let status = status_running();

        let progress = exit_condition_progress(
            &RoutineExitCondition::StateConditionMet(StateCondition::OutputWeightAbove(
                0,
                ParameterValue::Static(36.0),
            )),
            &status,
            None,
        )
        .expect("weight conditions report progress");

        assert_eq!(progress.current, None);
        assert_eq!(progress.target, 36.0);
        assert_eq!(progress.unit, ParameterUnit::Grams);
    }

    #[test]
    fn timers_read_their_elapsed_time_from_the_right_clock() {
        let mut status = status_running();
        let mut group = GroupStatus::default();
        group.current_brew = Some(variegated_controller_types::BrewStatus {
            brew_time: core::time::Duration::from_secs(27),
            brew_input_volume: None,
            shot_state: None,
            extracted_solids: None,
            output_volume: None,
        });
        let _ = status.group_statuses.insert(0, group);

        let step = exit_condition_progress(
            &RoutineExitCondition::After(ParameterValue::Static(10.0)),
            &status,
            None,
        )
        .expect("timers report progress");
        assert_eq!(step.current, Some(4.0), "elapsed within the step");

        let brew = exit_condition_progress(
            &RoutineExitCondition::AfterDurationRelativeToStart(ParameterValue::Static(30.0)),
            &status,
            None,
        )
        .expect("timers report progress");
        assert_eq!(brew.current, Some(27.0), "elapsed since the brew started");
    }

    #[test]
    fn conditions_with_nothing_to_measure_report_nothing() {
        // `Always`/`Never` fire without a threshold, `UserAction` waits for a person, and
        // `Brewing` is a boolean. Returning `Some` with a fabricated target would put a
        // meaningless progress bar on three screens.
        let status = status_running();

        for condition in [
            RoutineExitCondition::Always,
            RoutineExitCondition::Never,
            RoutineExitCondition::UserAction(0),
            RoutineExitCondition::StateConditionMet(StateCondition::Brewing(0)),
            RoutineExitCondition::StateConditionMet(StateCondition::NotBrewing(0)),
        ] {
            assert_eq!(exit_condition_progress(&condition, &status, None), None);
        }
    }

    #[test]
    fn a_water_tap_reports_its_target_but_not_a_current_value() {
        // `WaterTapStatus` carries only `is_dispensing`, so there is no flow rate to show.
        // The target is still worth returning; `None` is what says the rest is unknown.
        let status = status_running();

        let progress = exit_condition_progress(
            &RoutineExitCondition::StateConditionMet(StateCondition::WaterTapFlowRateAbove(
                0,
                ParameterValue::Static(3.0),
            )),
            &status,
            None,
        )
        .expect("the target is still reportable");

        assert_eq!(progress.current, None);
        assert_eq!(progress.target, 3.0);
    }
}
