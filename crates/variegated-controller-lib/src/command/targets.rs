//! The six commands that retarget a boiler or the group.
//!
//! One implementation for both machines. Each function takes the configuration through
//! [`ConfigurationAccess`] and the command's payload already destructured -- never a
//! `MachineCommand`, for the reason given in [`super`] -- and returns a [`TargetOutcome`]
//! saying what the controller must do next. No store, no `self`, no `await`: persistence is
//! the controllers' business, because one owns its settings store by value and the other
//! reaches it through a mutex behind a timeout.

use variegated_controller_types::{
    BoilerControlMode, BoilerControlState, BoilerControlTargetValuesUpdate, BoilerIndex,
    GroupBrewControlMode, GroupBrewControlTargetValues, GroupBrewControlTargetValuesUpdate,
    GroupBrewLimitMode, GroupIndex, PidParameterTarget, PidParameters,
};
use variegated_log::{log_error, log_info};

use super::access::{ConfigurationAccess, CurveAction, TargetOutcome};

/// Does this mode run a ramp?
///
/// The list is exhaustive on purpose rather than a `matches!` with a `_` arm: a mode added
/// later is either a curve or it is not, and that decision should be made here rather than
/// defaulting to "not" and quietly leaving the clock stopped.
fn is_curve_mode(mode: GroupBrewControlMode) -> bool {
    match mode {
        GroupBrewControlMode::GroupFlowRateCurve
        | GroupBrewControlMode::PressureCurve
        | GroupBrewControlMode::OutputFlowRateCurve
        | GroupBrewControlMode::FixedDutyCycleCurve => true,
        GroupBrewControlMode::Off
        | GroupBrewControlMode::GroupFlowRate
        | GroupBrewControlMode::Pressure
        | GroupBrewControlMode::OutputFlowRate
        | GroupBrewControlMode::FixedDutyCycle
        // Open-loop at 100%, and a constant is not a ramp.
        | GroupBrewControlMode::FullOn => false,
    }
}

/// Apply a boiler values update, field by field.
///
/// Both fields are optional and an absent one means "leave it alone", which is what lets a
/// mode change carry a new temperature without restating the pressure.
pub fn apply_boiler_values(
    state: &mut BoilerControlState,
    update: BoilerControlTargetValuesUpdate,
) {
    if let Some(temperature) = update.temperature {
        state.values.target_temperature = temperature;
    }
    if let Some(pressure) = update.pressure {
        state.values.target_pressure = pressure;
    }
}

/// Apply a group brew values update, field by field.
///
/// **Eleven fields, and this is the only copy.** It was two identical copies, one per
/// controller, each carrying the same comment predicting that three copies of eleven
/// `if let`s is how one of them ends up silently missing a field. Nothing checked either.
pub fn apply_group_brew_values(
    values: &mut GroupBrewControlTargetValues,
    update: GroupBrewControlTargetValuesUpdate,
) {
    if let Some(flow_rate) = update.flow_rate { values.flow_rate = flow_rate; }
    if let Some(curve) = update.flow_rate_curve { values.flow_rate_curve = curve; }
    if let Some(pressure) = update.pressure { values.pressure = pressure; }
    if let Some(curve) = update.pressure_curve { values.pressure_curve = curve; }
    if let Some(output_flow) = update.output_flow_rate { values.output_flow_rate = output_flow; }
    if let Some(curve) = update.output_flow_rate_curve { values.output_flow_rate_curve = curve; }
    if let Some(duty) = update.duty_cycle { values.duty_cycle = duty; }
    if let Some(curve) = update.duty_cycle_curve { values.duty_cycle_curve = curve; }
    if let Some(max) = update.max_pressure { values.max_pressure = max; }
    if let Some(max) = update.max_group_flow_rate { values.max_group_flow_rate = max; }
    if let Some(max) = update.max_output_flow_rate { values.max_output_flow_rate = max; }
}

/// `SetBoilerControlTarget`: change the mode, optionally the setpoints with it.
///
/// A `None` update keeps both stored setpoints, which is what makes "switch to temperature
/// control at the temperature I already set" expressible without restating it.
pub fn set_boiler_control_target<C: ConfigurationAccess>(
    config: &mut C,
    index: BoilerIndex,
    mode: BoilerControlMode,
    update: Option<BoilerControlTargetValuesUpdate>,
) -> TargetOutcome {
    log_info!(
        "Setting boiler control mode for boiler {} to {:?} with values {:?}",
        index, mode, update
    );
    let Some(state) = config.boiler_control_state_mut(index) else {
        log_error!("Invalid boiler index: {}", index);
        return TargetOutcome::refused();
    };
    state.mode = mode;
    if let Some(update) = update {
        apply_boiler_values(state, update);
    }
    TargetOutcome::persistent()
}

/// `SetBoilerControlTargetValues`: change the setpoints, leaving the mode alone.
pub fn set_boiler_control_target_values<C: ConfigurationAccess>(
    config: &mut C,
    index: BoilerIndex,
    update: BoilerControlTargetValuesUpdate,
) -> TargetOutcome {
    log_info!("Setting boiler control values for boiler {} to {:?}", index, update);
    let Some(state) = config.boiler_control_state_mut(index) else {
        log_error!("Invalid boiler index: {}", index);
        return TargetOutcome::refused();
    };
    apply_boiler_values(state, update);
    TargetOutcome::persistent()
}

/// `SetGroupBrewControlTarget`: change what the pump is controlling, and restart the ramp.
///
/// Selecting a curve mode restarts the ramp from now; selecting any other mode stops it.
/// Both are [`CurveAction`]s rather than something done here -- see its docs.
pub fn set_group_brew_control_target<C: ConfigurationAccess>(
    config: &mut C,
    index: GroupIndex,
    mode: GroupBrewControlMode,
    update: Option<GroupBrewControlTargetValuesUpdate>,
) -> TargetOutcome {
    log_info!(
        "Setting group brew control mode for group {} to {:?} with values {:?}",
        index, mode, update
    );
    let Some(state) = config.group_brew_control_state_mut(index) else {
        log_error!("Invalid group index: {}", index);
        return TargetOutcome::refused();
    };
    state.mode = mode;
    if let Some(update) = update {
        apply_group_brew_values(&mut state.values, update);
    }

    if is_curve_mode(mode) {
        log_info!("Starting curve control");
        TargetOutcome::ephemeral(CurveAction::Start)
    } else {
        TargetOutcome::ephemeral(CurveAction::Clear)
    }
}

/// `SetGroupBrewControlTargetValues`: change the setpoints, leaving the mode alone.
pub fn set_group_brew_control_target_values<C: ConfigurationAccess>(
    config: &mut C,
    index: GroupIndex,
    update: GroupBrewControlTargetValuesUpdate,
) -> TargetOutcome {
    log_info!("Setting group brew control values for group {} to {:?}", index, update);
    let Some(state) = config.group_brew_control_state_mut(index) else {
        log_error!("Invalid group index: {}", index);
        return TargetOutcome::refused();
    };
    apply_group_brew_values(&mut state.values, update);
    TargetOutcome::ephemeral(CurveAction::Leave)
}

/// `SetGroupBrewLimit`: arm, change or disarm the cap on the pump.
///
/// [`GroupBrewLimitMode::Unlimited`] disarms. The curve clock is deliberately untouched: a
/// limit is a constant, and arming one must not restart the ramp a curve mode is partway
/// through.
pub fn set_group_brew_limit<C: ConfigurationAccess>(
    config: &mut C,
    index: GroupIndex,
    limit: GroupBrewLimitMode,
    update: Option<GroupBrewControlTargetValuesUpdate>,
) -> TargetOutcome {
    log_info!(
        "Setting group brew limit for group {} to {:?} with values {:?}",
        index, limit, update
    );
    let Some(state) = config.group_brew_control_state_mut(index) else {
        log_error!("Invalid group index: {}", index);
        return TargetOutcome::refused();
    };
    state.limit = limit;
    if let Some(update) = update {
        apply_group_brew_values(&mut state.values, update);
    }
    TargetOutcome::ephemeral(CurveAction::Leave)
}

/// `SetPidParameters`: replace one loop's gains.
///
/// **Persists even when the target resolved to nothing.** That matches what both controllers
/// did before this was shared, and it is harmless rather than merely tolerated: the settings
/// store compares against its cached value and skips a write that would change no bytes.
/// Returning `persist: false` here would be a behaviour change dressed as a refactor.
pub fn set_pid_parameters<C: ConfigurationAccess>(
    config: &mut C,
    target: PidParameterTarget,
    params: PidParameters,
) -> TargetOutcome {
    match config.pid_parameters_mut(target) {
        Some(slot) => *slot = params,
        // The index, not the whole target: it is the half that can be wrong, and this
        // matches the message the dual-boiler controller logged before.
        None => log_error!("Invalid boiler index for PID parameters: {:?}", target),
    }
    TargetOutcome::persistent()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dual_boiler_config::DualBoilerSingleGroupConfiguration;
    use crate::single_boiler_config::SingleBoilerSingleGroupConfiguration;
    use variegated_controller_types::{ControlCurve, DutyCycleType, PidLimits, PidTerm};

    /// A tuning distinguishable from every default in either machine.
    fn marker_params() -> PidParameters {
        PidParameters {
            kp: PidTerm::new(1.25, PidLimits::default()),
            ki: PidTerm::new(2.5, PidLimits::default()),
            kd: PidTerm::new(3.75, PidLimits::default()),
        }
    }

    // ---- the eleven-field update -------------------------------------------------------

    /// Every field of the update lands, and lands in its own slot.
    ///
    /// The property the two hand-maintained copies of this function never had checked. It is
    /// written as eleven separate one-field updates rather than one full update precisely
    /// because a copy that wrote the *wrong* slot would still pass the latter.
    #[test]
    fn each_group_brew_field_applies_independently() {
        // Distinguishable from the zeroed curve every default carries, so that "the field
        // was written" and "the field happens to equal the default" cannot be confused.
        let curve = ControlCurve { a: 1.0, b: 2.0, c: 3.0, min: 4.0, max: 5.0 };

        macro_rules! check {
            ($field:ident, $value:expr) => {{
                let mut values = GroupBrewControlTargetValues::default();
                let update = GroupBrewControlTargetValuesUpdate {
                    $field: Some($value),
                    ..GroupBrewControlTargetValuesUpdate::default()
                };
                apply_group_brew_values(&mut values, update);
                assert_eq!(values.$field, $value, concat!(stringify!($field), " did not apply"));

                // and nothing else moved
                let mut expected = GroupBrewControlTargetValues::default();
                expected.$field = $value;
                assert_eq!(values, expected, concat!(stringify!($field), " disturbed another field"));
            }};
        }

        check!(flow_rate, 3.5);
        check!(flow_rate_curve, curve.clone());
        check!(pressure, 8.5);
        check!(pressure_curve, curve.clone());
        check!(output_flow_rate, 1.75);
        check!(output_flow_rate_curve, curve.clone());
        check!(duty_cycle, DutyCycleType::new(42));
        check!(duty_cycle_curve, curve.clone());
        check!(max_pressure, 11.0);
        check!(max_group_flow_rate, 6.25);
        check!(max_output_flow_rate, 2.25);
    }

    /// An empty update changes nothing at all.
    #[test]
    fn an_empty_group_brew_update_is_a_no_op() {
        let mut values = GroupBrewControlTargetValues::default();
        apply_group_brew_values(&mut values, GroupBrewControlTargetValuesUpdate::default());
        assert_eq!(values, GroupBrewControlTargetValues::default());
    }

    // ---- boiler targets -----------------------------------------------------------------

    /// A mode change with no update keeps both stored setpoints.
    ///
    /// Stated in `MachineCommand`'s own docs and tested nowhere until now.
    #[test]
    fn a_mode_change_without_values_preserves_both_setpoints() {
        let mut config = SingleBoilerSingleGroupConfiguration::default();
        let before = *config.boiler_control_state_mut(0).unwrap();

        let outcome = set_boiler_control_target(&mut config, 0, BoilerControlMode::Pressure, None);

        let after = *config.boiler_control_state_mut(0).unwrap();
        assert_eq!(after.mode, BoilerControlMode::Pressure);
        assert_eq!(after.values, before.values, "the stored setpoints moved");
        assert!(outcome.persist);
    }

    /// A partial update leaves the field it does not name alone.
    #[test]
    fn a_partial_boiler_update_leaves_the_other_setpoint() {
        let mut config = SingleBoilerSingleGroupConfiguration::default();
        let before = *config.boiler_control_state_mut(0).unwrap();

        let outcome = set_boiler_control_target_values(
            &mut config,
            0,
            BoilerControlTargetValuesUpdate { temperature: Some(96.5), pressure: None },
        );

        let after = *config.boiler_control_state_mut(0).unwrap();
        assert_eq!(after.values.target_temperature, 96.5);
        assert_eq!(after.values.target_pressure, before.values.target_pressure);
        assert!(outcome.persist);
    }

    /// A boiler this machine does not have changes nothing and does not reach flash.
    #[test]
    fn an_unknown_boiler_index_is_refused() {
        let mut config = SingleBoilerSingleGroupConfiguration::default();
        let before = config.clone();

        let outcome = set_boiler_control_target(&mut config, 2, BoilerControlMode::Pressure, None);

        assert_eq!(outcome, TargetOutcome::refused());
        assert!(!outcome.persist, "a refused command must not write to flash");
        assert_eq!(config, before, "a refused command mutated the configuration");
    }

    /// Same for the values-only form, on the dual-boiler machine.
    #[test]
    fn an_unknown_boiler_index_is_refused_on_the_dual_boiler_too() {
        let mut config = DualBoilerSingleGroupConfiguration::default();
        let before = config.clone();

        let outcome = set_boiler_control_target_values(
            &mut config,
            7,
            BoilerControlTargetValuesUpdate { temperature: Some(96.5), pressure: None },
        );

        assert_eq!(outcome, TargetOutcome::refused());
        assert_eq!(config, before);
    }

    // ---- the curve clock ----------------------------------------------------------------

    /// Selecting a curve mode restarts the ramp; selecting a fixed mode stops it.
    #[test]
    fn curve_modes_start_the_ramp_and_others_clear_it() {
        let mut config = SingleBoilerSingleGroupConfiguration::default();

        for mode in [
            GroupBrewControlMode::GroupFlowRateCurve,
            GroupBrewControlMode::PressureCurve,
            GroupBrewControlMode::OutputFlowRateCurve,
            GroupBrewControlMode::FixedDutyCycleCurve,
        ] {
            let outcome = set_group_brew_control_target(&mut config, 0, mode, None);
            assert_eq!(outcome.curve, CurveAction::Start, "{mode:?} should start the ramp");
            assert!(!outcome.persist, "group brew state is ephemeral");
        }

        for mode in [
            GroupBrewControlMode::Off,
            GroupBrewControlMode::GroupFlowRate,
            GroupBrewControlMode::Pressure,
            GroupBrewControlMode::OutputFlowRate,
            GroupBrewControlMode::FixedDutyCycle,
        ] {
            let outcome = set_group_brew_control_target(&mut config, 0, mode, None);
            assert_eq!(outcome.curve, CurveAction::Clear, "{mode:?} should stop the ramp");
        }
    }

    /// Arming a limit must not disturb a ramp that is partway through.
    ///
    /// The one property in this file that is a safety argument rather than bookkeeping: the
    /// limit and the profile are independent, and restarting the ramp mid-shot would change
    /// the shot. Both controllers carried it as a comment.
    #[test]
    fn arming_a_limit_leaves_the_ramp_alone() {
        let mut config = SingleBoilerSingleGroupConfiguration::default();

        let outcome = set_group_brew_limit(
            &mut config,
            0,
            GroupBrewLimitMode::MaxPressure,
            Some(GroupBrewControlTargetValuesUpdate {
                max_pressure: Some(9.0),
                ..GroupBrewControlTargetValuesUpdate::default()
            }),
        );

        assert_eq!(outcome.curve, CurveAction::Leave);
        let state = config.group_brew_control_state_mut(0).unwrap();
        assert_eq!(state.limit, GroupBrewLimitMode::MaxPressure);
        assert_eq!(state.values.max_pressure, 9.0);
    }

    /// Disarming is a mode like any other, and equally leaves the ramp alone.
    #[test]
    fn unlimited_disarms_the_limit() {
        let mut config = SingleBoilerSingleGroupConfiguration::default();
        let _ = set_group_brew_limit(&mut config, 0, GroupBrewLimitMode::MaxGroupFlowRate, None);

        let outcome = set_group_brew_limit(&mut config, 0, GroupBrewLimitMode::Unlimited, None);

        assert_eq!(outcome.curve, CurveAction::Leave);
        assert_eq!(
            config.group_brew_control_state_mut(0).unwrap().limit,
            GroupBrewLimitMode::Unlimited
        );
    }

    /// A group this machine does not have changes nothing.
    #[test]
    fn an_unknown_group_index_is_refused() {
        let mut config = SingleBoilerSingleGroupConfiguration::default();
        let before = config.clone();

        let outcome = set_group_brew_control_target(
            &mut config,
            1,
            GroupBrewControlMode::PressureCurve,
            None,
        );

        assert_eq!(outcome, TargetOutcome::refused());
        assert_eq!(
            outcome.curve,
            CurveAction::Leave,
            "a refused command must not touch the ramp either"
        );
        assert_eq!(config, before);
    }

    // ---- PID routing --------------------------------------------------------------------

    /// On the dual-boiler machine the boiler index selects which tuning is written.
    #[test]
    fn dual_boiler_pid_parameters_are_per_boiler() {
        let mut config = DualBoilerSingleGroupConfiguration::default();
        let steam_before = config.persistent.steam_boiler.temperature_pid_parameters;

        let outcome = set_pid_parameters(
            &mut config,
            PidParameterTarget::BoilerTemperature(1),
            marker_params(),
        );
        assert!(outcome.persist);

        assert_eq!(config.persistent.steam_boiler.temperature_pid_parameters, marker_params());
        assert_ne!(
            config.persistent.brew_boiler.temperature_pid_parameters, marker_params(),
            "writing the steam boiler's tuning disturbed the brew boiler's"
        );
        assert_ne!(steam_before, marker_params(), "the test's marker matched the default");
    }

    /// On the single-boiler machine both boiler indices name the same tuning.
    ///
    /// **This pins deliberate behaviour, not an oversight.** There is one heating element and
    /// one tuning; the "virtual steam boiler" is a mode of it. The ESPHome bridge reads back
    /// the published parameters, edits one term and writes the whole struct, so a machine that
    /// answered index 1 with a *separate* default tuning is how a tuned PID gets replaced by
    /// an all-zero one. See the comment in `single_boiler_config`'s `From` impl.
    #[test]
    fn single_boiler_pid_parameters_are_shared_across_both_indices() {
        let mut config = SingleBoilerSingleGroupConfiguration::default();

        let outcome = set_pid_parameters(
            &mut config,
            PidParameterTarget::BoilerTemperature(1),
            marker_params(),
        );
        assert!(outcome.persist);

        assert_eq!(
            config.persistent.pid_parameters.boiler_temperature_params, marker_params(),
            "the steam index should write the one shared tuning"
        );
        assert_eq!(
            *config.pid_parameters_mut(PidParameterTarget::BoilerTemperature(0)).unwrap(),
            marker_params(),
            "both indices must resolve to the same tuning"
        );
    }

    /// Each of the five targets writes its own slot, on both machines.
    #[test]
    fn every_pid_target_writes_its_own_slot() {
        for target in [
            PidParameterTarget::BoilerTemperature(0),
            PidParameterTarget::BoilerPressure(0),
            PidParameterTarget::GroupFlowRate(0),
            PidParameterTarget::GroupPressure(0),
            PidParameterTarget::GroupOutputFlowRate(0),
        ] {
            let mut dual = DualBoilerSingleGroupConfiguration::default();
            let outcome = set_pid_parameters(&mut dual, target, marker_params());
            assert!(outcome.persist, "{target:?} must reach flash");
            assert_eq!(*dual.pid_parameters_mut(target).unwrap(), marker_params());

            let mut single = SingleBoilerSingleGroupConfiguration::default();
            let outcome = set_pid_parameters(&mut single, target, marker_params());
            assert!(outcome.persist);
            assert_eq!(*single.pid_parameters_mut(target).unwrap(), marker_params());
        }
    }

    /// An unresolvable PID target still persists, matching what both controllers did.
    ///
    /// Documented in `set_pid_parameters`: the store skips a write that changes no bytes, so
    /// this costs nothing, and returning `false` would be a behaviour change dressed as a
    /// refactor.
    #[test]
    fn an_unknown_pid_boiler_index_still_reports_persist() {
        let mut config = DualBoilerSingleGroupConfiguration::default();
        let before = config.clone();

        let outcome = set_pid_parameters(
            &mut config,
            PidParameterTarget::BoilerTemperature(9),
            marker_params(),
        );

        assert!(outcome.persist);
        assert_eq!(config, before, "nothing should have been written");
    }
}
