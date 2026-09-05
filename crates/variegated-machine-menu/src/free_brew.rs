//! Driving the pump by hand during a free brew.
//!
//! A free brew is the shot nobody wrote a routine for: the operator picks how the pump is
//! controlled and dials the setpoint while it runs. Both machines offer it, from opposite
//! kinds of control -- the Silvia's rotary encoder, the GS3's four buttons and a Bluetooth
//! dial -- and neither difference reaches this far. What is here is the model: which modes
//! are cycled, in what order, what each one's target may be, and what has to happen to the
//! targets at the moment the mode changes.
//!
//! # This is not [`crate::OFFERED_BREW_MODES`]
//!
//! That list is the *menu's*, and it ends in [`GroupBrewControlMode::Off`] so that a
//! settings row can turn the group off. Cycling onto `Off` mid-shot would shut the pump and
//! close the three-way valve, which is not a step on the way to the next mode. So this is its
//! own list of three, and it is deliberately a different list rather than a filtered view of
//! that one -- the two answer different questions and there is no reason they should move
//! together.
//!
//! # Half of bumpless transfer lives here
//!
//! [`FreeBrewState::sync_from`] is the UI half: before the mode changes, every target is
//! snapped to the measurement the incoming PID will actually read, so the error -- and
//! therefore the proportional term -- starts at ~0.
//!
//! **That half alone is a bug.** With P at 0 and the integral at 0 the output is `0 + 0 + 0`
//! and the pump drops dead the instant it leaves duty-cycle mode. The other half is the
//! controller's: it seeds the integral from the duty already being commanded, and holds the
//! PID while it does not own the output. See `variegated_controller_lib::pump_transfer`,
//! whose module doc is the full account. Neither half is any use without the other.

use variegated_controller_types::routines::parameters::ParameterUnit;
use variegated_controller_types::{
    DutyCycleType, GroupBrewControlMode, GroupBrewControlState,
    GroupBrewControlTargetValuesUpdate, GroupStatus,
};
use variegated_menu::Adjustable;

/// The modes a free brew cycles through, in cycle order.
///
/// Three of [`GroupBrewControlMode`]'s ten. The four `*Curve` variants each need five numbers
/// that no knob can enter, `FullOn` has no target to dial, `OutputFlowRate` measures across
/// the puck and is a routine's tool rather than a hand control, and `Off` is not a step on
/// the way anywhere -- see the module docs.
///
/// Duty cycle is first because it is where a free brew starts: open loop, no PID engaged, and
/// the one mode that is always safe to be in.
pub const FREE_BREW_MODES: [GroupBrewControlMode; 3] = [
    GroupBrewControlMode::FixedDutyCycle,
    GroupBrewControlMode::Pressure,
    GroupBrewControlMode::GroupFlowRate,
];

/// The mode after this one, wrapping.
///
/// A mode outside [`FREE_BREW_MODES`] -- set from the web, or `Off` after a target was dialled
/// to zero -- lands on the first rather than sticking. A control that cannot be moved reads as
/// a broken one.
pub fn next_free_brew_mode(current: GroupBrewControlMode) -> GroupBrewControlMode {
    match FREE_BREW_MODES.iter().position(|mode| *mode == current) {
        Some(index) => FREE_BREW_MODES[(index + 1) % FREE_BREW_MODES.len()],
        None => FREE_BREW_MODES[0],
    }
}

/// The range and step of a mode's target: `(min, max, step)`.
///
/// **Stated once**, and read by both [`FreeBrewState::adjust`] and
/// [`FreeBrewState::sync_from`] -- so a synced value lands on the same grid the knob then
/// walks along. Keeping two copies is how the Silvia's version came to round to one grid and
/// step along another.
///
/// Deliberately coarser than [`crate::parameter_bounds`], which is the menu editor's table.
/// This is a control operated while hot water is running through coffee: the GS3's panel has
/// no auto-repeat, so one press is one step, and a 0.1 bar step would make 9 bar ninety
/// presses. A dial batches a turn into a count and could afford finer, but one grid that both
/// can use beats two that disagree.
///
/// `None` for a mode with no dialable target, which is everything outside [`FREE_BREW_MODES`].
pub fn free_brew_bounds(mode: GroupBrewControlMode) -> Option<(f32, f32, f32)> {
    match mode {
        //                                       min    max   step
        GroupBrewControlMode::FixedDutyCycle => Some((0.0, 100.0, 5.0)),
        GroupBrewControlMode::Pressure => Some((0.0, 15.0, 0.5)),
        // 10 ml/s is already more than a group draws. This used to read 50 on the Silvia,
        // which put nine tenths of the knob's travel past anything reachable.
        GroupBrewControlMode::GroupFlowRate => Some((0.0, 10.0, 0.2)),
        _ => None,
    }
}

/// The measured value each free-brew mode's loop actually reads.
///
/// A projection of [`GroupStatus`] rather than the whole thing, because the GS3's button task
/// keeps one of these between presses and has no reason to hold a group status to do it.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FreeBrewMeasurements {
    /// What the pump is running at, as a percentage.
    pub duty_cycle: DutyCycleType,
    /// Group pressure in bar, or `None` when nothing is reporting it.
    pub pressure: Option<f32>,
    /// **Input** flow rate in ml/s. See [`FreeBrewState::sync_from`] for why it must be the
    /// input one.
    pub input_flow_rate: Option<f32>,
}

impl FreeBrewMeasurements {
    /// Read the three values out of a group's status.
    pub fn from_group_status(group: &GroupStatus) -> Self {
        Self {
            // `pump_output` is on the pump's 0-255 scale and this model is not; `duty_cycle`
            // is the derived percentage.
            duty_cycle: group.pump_output.duty_cycle(),
            pressure: group.pressure,
            input_flow_rate: group.input_flow_rate,
        }
    }
}

/// Which mode a free brew is in, and the target dialled into each.
///
/// All three targets are kept at once, so cycling away from a mode and back returns to the
/// number you left there rather than to a default.
///
/// **`mode` is the operator's, not the wire's.** [`Self::to_command`] collapses a zero target
/// to [`GroupBrewControlMode::Off`], so the machine can be in `Off` while this still says
/// `FixedDutyCycle` -- which is what lets one press of `+` lift the pump back off zero into
/// the mode it was already in.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FreeBrewState {
    mode: GroupBrewControlMode,
    /// A percentage. This is an operator control, so it stays on the operator's scale; the
    /// conversion to the pump's 0-255 scale happens in the controller.
    duty_cycle: DutyCycleType,
    flow_rate: f32,
    pressure: f32,
}

impl Default for FreeBrewState {
    /// Duty cycle at zero, and the two closed-loop targets at ordinary shot values.
    ///
    /// Zero duty rather than full: this is what a machine that has never been told otherwise
    /// opens with, and a pump that does nothing until asked is the right way to be wrong.
    fn default() -> Self {
        Self {
            mode: FREE_BREW_MODES[0],
            duty_cycle: DutyCycleType::OFF,
            flow_rate: 5.0,
            pressure: 9.0,
        }
    }
}

impl FreeBrewState {
    /// Seed from what the controller is currently set to.
    ///
    /// Used when a free brew starts, so the control opens on the machine's actual state
    /// rather than on a local copy left over from the last shot. A mode outside
    /// [`FREE_BREW_MODES`] -- including the `Off` a zero target collapses to -- normalises to
    /// the first, keeping every other target.
    pub fn from_control_state(state: &GroupBrewControlState) -> Self {
        Self {
            mode: if FREE_BREW_MODES.contains(&state.mode) {
                state.mode
            } else {
                FREE_BREW_MODES[0]
            },
            duty_cycle: state.values.duty_cycle,
            flow_rate: state.values.flow_rate,
            pressure: state.values.pressure,
        }
    }

    /// The mode being dialled.
    pub fn mode(&self) -> GroupBrewControlMode {
        self.mode
    }

    /// The target for the current mode, on that mode's own scale.
    pub fn value(&self) -> f32 {
        match self.mode {
            GroupBrewControlMode::Pressure => self.pressure,
            GroupBrewControlMode::GroupFlowRate => self.flow_rate,
            _ => self.duty_cycle.value() as f32,
        }
    }

    /// What the current mode is called, spelled out.
    ///
    /// The long form, for a screen with room for it. [`crate::brew_mode_label`] is the
    /// four-column abbreviation a menu value field needs.
    pub fn label(&self) -> &'static str {
        match self.mode {
            GroupBrewControlMode::Pressure => "Pressure",
            GroupBrewControlMode::GroupFlowRate => "Flow Rate",
            _ => "Duty Cycle",
        }
    }

    /// The unit the current mode's target is in.
    pub fn unit(&self) -> ParameterUnit {
        match self.mode {
            GroupBrewControlMode::Pressure => ParameterUnit::Bar,
            GroupBrewControlMode::GroupFlowRate => ParameterUnit::MillilitersPerSecond,
            _ => ParameterUnit::Percent,
        }
    }

    /// Step the current mode's target one step up or down, stopping at its bounds.
    pub fn adjust(&mut self, increment: bool) {
        let Some((min, max, step)) = free_brew_bounds(self.mode) else {
            return;
        };

        let mut adjustable = Adjustable::new(self.value(), min, max, step);
        if increment {
            adjustable.increase();
        } else {
            adjustable.decrease();
        }
        self.set_value(adjustable.value());
    }

    /// Advance to the next mode. Targets are left alone -- call [`Self::sync_from`] first.
    pub fn advance_mode(&mut self) {
        self.mode = next_free_brew_mode(self.mode);
    }

    /// Snap every target to the measurement its loop reads, on that target's own step grid.
    ///
    /// The UI half of bumpless transfer, and it belongs *before* [`Self::advance_mode`]: the
    /// incoming mode's target has to already match reality when the PID engages, or the
    /// transfer starts with a real error however well the integral was seeded.
    ///
    /// **The flow rate must come from the input.** [`GroupBrewControlMode::GroupFlowRate`]'s
    /// PID reads the group's input flow, so syncing from the *output* flow -- which the
    /// Silvia's version did until it was fixed -- hands the loop a setpoint measured on the
    /// far side of the puck. They are not the same number and the difference is the shot.
    ///
    /// A measurement that is `None` leaves its target alone rather than substituting a value
    /// the loop cannot see. Nothing is reporting that quantity, so the last deliberate
    /// setting is a better answer than zero.
    pub fn sync_from(&mut self, measured: &FreeBrewMeasurements) {
        if let Some((min, max, step)) = free_brew_bounds(GroupBrewControlMode::FixedDutyCycle) {
            self.duty_cycle =
                DutyCycleType::from_f32(snap(measured.duty_cycle.value() as f32, min, max, step));
        }

        if let Some(flow) = measured.input_flow_rate {
            if let Some((min, max, step)) = free_brew_bounds(GroupBrewControlMode::GroupFlowRate) {
                self.flow_rate = snap(flow, min, max, step);
            }
        }

        if let Some(pressure) = measured.pressure {
            if let Some((min, max, step)) = free_brew_bounds(GroupBrewControlMode::Pressure) {
                self.pressure = snap(pressure, min, max, step);
            }
        }
    }

    /// The mode and target update to send as
    /// `MachineCommand::SetGroupBrewControlTarget`.
    ///
    /// **A zero target becomes [`GroupBrewControlMode::Off`]**, which stops the pump and
    /// closes the three-way valve -- so dialling all the way down ends the shot. That is the
    /// behaviour the Silvia's encoder has always had and it is worth keeping: the control
    /// that started the pump is the one that stops it, without reaching for another button.
    pub fn to_command(
        &self,
    ) -> (
        GroupBrewControlMode,
        Option<GroupBrewControlTargetValuesUpdate>,
    ) {
        if self.is_at_zero() {
            return (GroupBrewControlMode::Off, None);
        }

        let update = match self.mode {
            GroupBrewControlMode::Pressure => GroupBrewControlTargetValuesUpdate {
                pressure: Some(self.pressure),
                ..Default::default()
            },
            GroupBrewControlMode::GroupFlowRate => GroupBrewControlTargetValuesUpdate {
                flow_rate: Some(self.flow_rate),
                ..Default::default()
            },
            _ => GroupBrewControlTargetValuesUpdate {
                duty_cycle: Some(self.duty_cycle),
                ..Default::default()
            },
        };

        (self.mode, Some(update))
    }

    /// Whether the current mode's target is zero, i.e. whether the pump is being asked for
    /// nothing.
    pub fn is_at_zero(&self) -> bool {
        match self.mode {
            GroupBrewControlMode::Pressure => self.pressure <= 0.0,
            GroupBrewControlMode::GroupFlowRate => self.flow_rate <= 0.0,
            _ => self.duty_cycle == DutyCycleType::OFF,
        }
    }

    fn set_value(&mut self, value: f32) {
        match self.mode {
            GroupBrewControlMode::Pressure => self.pressure = value,
            GroupBrewControlMode::GroupFlowRate => self.flow_rate = value,
            // `Adjustable` has already clamped to 0..=100; `from_f32` clamps again and rounds
            // rather than truncating, which is the rule every duty cycle narrows by.
            _ => self.duty_cycle = DutyCycleType::from_f32(value),
        }
    }
}

/// Clamp into range, then round to the nearest multiple of `step`.
///
/// The rounding is what puts a synced measurement onto the same grid the knob steps along, so
/// the first press after a sync moves by exactly one step rather than by whatever remainder
/// the measurement happened to carry.
fn snap(value: f32, min: f32, max: f32, step: f32) -> f32 {
    let clamped = if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    };

    // `as u32` truncates, so the half-step is what makes this round to nearest. The input is
    // already clamped to a non-negative range, so the cast cannot saturate from below.
    let snapped = ((clamped + step / 2.0) / step) as u32 as f32 * step;
    if snapped > max { max } else { snapped }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn measurements(duty: u8, pressure: Option<f32>, flow: Option<f32>) -> FreeBrewMeasurements {
        FreeBrewMeasurements {
            duty_cycle: DutyCycleType::new(duty),
            pressure,
            input_flow_rate: flow,
        }
    }

    #[test]
    fn cycling_visits_every_mode_and_returns() {
        let mut mode = FREE_BREW_MODES[0];
        let mut seen = alloc::vec::Vec::new();
        for _ in 0..FREE_BREW_MODES.len() {
            seen.push(mode);
            mode = next_free_brew_mode(mode);
        }
        assert_eq!(seen, FREE_BREW_MODES);
        assert_eq!(mode, FREE_BREW_MODES[0]);
    }

    #[test]
    fn the_cycle_is_duty_then_pressure_then_flow() {
        // The agreed order. Duty first because it is where a free brew opens, and pressure
        // before flow because that is the transfer people actually make.
        assert_eq!(
            next_free_brew_mode(GroupBrewControlMode::FixedDutyCycle),
            GroupBrewControlMode::Pressure
        );
        assert_eq!(
            next_free_brew_mode(GroupBrewControlMode::Pressure),
            GroupBrewControlMode::GroupFlowRate
        );
        assert_eq!(
            next_free_brew_mode(GroupBrewControlMode::GroupFlowRate),
            GroupBrewControlMode::FixedDutyCycle
        );
    }

    #[test]
    fn off_is_never_reached_by_cycling() {
        // Cycling onto `Off` mid-shot would shut the pump and close the valve. The only route
        // to `Off` is dialling a target to zero.
        let mut mode = FREE_BREW_MODES[0];
        for _ in 0..12 {
            mode = next_free_brew_mode(mode);
            assert_ne!(mode, GroupBrewControlMode::Off);
        }
    }

    #[test]
    fn a_mode_outside_the_cycle_lands_on_the_first_rather_than_sticking() {
        for mode in [
            GroupBrewControlMode::Off,
            GroupBrewControlMode::FullOn,
            GroupBrewControlMode::OutputFlowRate,
            GroupBrewControlMode::PressureCurve,
        ] {
            assert_eq!(next_free_brew_mode(mode), FREE_BREW_MODES[0], "for {mode:?}");
        }
    }

    #[test]
    fn each_mode_names_its_own_quantity_and_value() {
        // Label, unit and value have to move together, or the screen shows one mode's number
        // under another mode's name.
        let mut state = FreeBrewState::default();
        state.duty_cycle = DutyCycleType::new(60);
        state.pressure = 9.0;
        state.flow_rate = 4.0;

        let cases = [
            (
                GroupBrewControlMode::FixedDutyCycle,
                ("Duty Cycle", ParameterUnit::Percent, 60.0),
            ),
            (
                GroupBrewControlMode::Pressure,
                ("Pressure", ParameterUnit::Bar, 9.0),
            ),
            (
                GroupBrewControlMode::GroupFlowRate,
                ("Flow Rate", ParameterUnit::MillilitersPerSecond, 4.0),
            ),
        ];
        for (mode, (label, unit, value)) in cases {
            state.mode = mode;
            assert_eq!(state.label(), label, "for {mode:?}");
            assert_eq!(state.unit(), unit, "for {mode:?}");
            assert_eq!(state.value(), value, "for {mode:?}");
        }
    }

    #[test]
    fn every_mode_in_the_cycle_has_bounds() {
        // A mode that is offered but has no bounds would be a control that cannot be dialled.
        for mode in FREE_BREW_MODES {
            let bounds = free_brew_bounds(mode);
            assert!(bounds.is_some(), "{mode:?} has no bounds");
            let (min, max, step) = bounds.unwrap();
            assert!(max > min, "{mode:?} has an empty range");
            assert!(step > 0.0, "{mode:?} has a zero step");
        }
    }

    #[test]
    fn adjusting_steps_by_the_modes_own_step() {
        let mut state = FreeBrewState::default();

        state.mode = GroupBrewControlMode::Pressure;
        state.pressure = 9.0;
        state.adjust(true);
        assert_eq!(state.value(), 9.5);
        state.adjust(false);
        state.adjust(false);
        assert_eq!(state.value(), 8.5);

        state.mode = GroupBrewControlMode::GroupFlowRate;
        state.flow_rate = 4.0;
        state.adjust(true);
        assert!((state.value() - 4.2).abs() < 1e-5, "{}", state.value());

        state.mode = GroupBrewControlMode::FixedDutyCycle;
        state.duty_cycle = DutyCycleType::new(60);
        state.adjust(true);
        assert_eq!(state.value(), 65.0);
    }

    #[test]
    fn adjusting_saturates_at_both_bounds() {
        let mut state = FreeBrewState::default();

        state.mode = GroupBrewControlMode::GroupFlowRate;
        state.flow_rate = 0.0;
        state.adjust(false);
        assert_eq!(state.value(), 0.0);

        state.flow_rate = 10.0;
        state.adjust(true);
        assert_eq!(state.value(), 10.0);

        state.mode = GroupBrewControlMode::FixedDutyCycle;
        state.duty_cycle = DutyCycleType::new(100);
        state.adjust(true);
        assert_eq!(state.value(), 100.0);
    }

    #[test]
    fn adjusting_one_mode_leaves_the_others_alone() {
        // Cycling away and back has to return to the number you left, not to a default.
        let mut state = FreeBrewState::default();
        state.pressure = 9.0;
        state.flow_rate = 4.0;

        state.mode = GroupBrewControlMode::Pressure;
        state.adjust(true);

        state.mode = GroupBrewControlMode::GroupFlowRate;
        assert_eq!(state.value(), 4.0);
        state.mode = GroupBrewControlMode::Pressure;
        assert_eq!(state.value(), 9.5);
    }

    #[test]
    fn syncing_snaps_each_target_to_its_own_grid() {
        // The point of snapping: the press after a sync moves by exactly one step, rather
        // than by whatever remainder the measurement carried.
        let mut state = FreeBrewState::default();
        state.sync_from(&measurements(63, Some(8.37), Some(3.11)));

        assert_eq!(state.duty_cycle.value(), 65);
        assert_eq!(state.pressure, 8.5);
        assert!((state.flow_rate - 3.2).abs() < 1e-5, "{}", state.flow_rate);

        state.mode = GroupBrewControlMode::Pressure;
        state.adjust(true);
        assert_eq!(state.value(), 9.0);
    }

    #[test]
    fn a_missing_measurement_leaves_its_target_alone() {
        // Nothing is reporting that quantity, so the last deliberate setting beats zero.
        let mut state = FreeBrewState::default();
        state.pressure = 9.0;
        state.flow_rate = 4.0;

        state.sync_from(&measurements(0, None, None));

        assert_eq!(state.pressure, 9.0);
        assert_eq!(state.flow_rate, 4.0);
    }

    #[test]
    fn syncing_clamps_a_measurement_past_the_ceiling() {
        let mut state = FreeBrewState::default();
        state.sync_from(&measurements(100, Some(99.0), Some(99.0)));

        assert_eq!(state.pressure, 15.0);
        assert_eq!(state.flow_rate, 10.0);
        assert_eq!(state.duty_cycle.value(), 100);
    }

    #[test]
    fn a_command_carries_only_the_current_modes_target() {
        // Sending all three would overwrite targets the operator set from somewhere else.
        let mut state = FreeBrewState::default();
        state.pressure = 9.0;
        state.flow_rate = 4.0;
        state.duty_cycle = DutyCycleType::new(60);

        state.mode = GroupBrewControlMode::Pressure;
        let (mode, update) = state.to_command();
        assert_eq!(mode, GroupBrewControlMode::Pressure);
        let update = update.expect("a non-zero target sends an update");
        assert_eq!(update.pressure, Some(9.0));
        assert_eq!(update.flow_rate, None);
        assert_eq!(update.duty_cycle, None);
    }

    #[test]
    fn a_zero_target_collapses_to_off() {
        // Dialling to zero ends the shot: `Off` stops the pump and closes the three-way
        // valve. Every mode has to agree about that, or one of them becomes a dead end.
        let mut state = FreeBrewState::default();

        state.mode = GroupBrewControlMode::FixedDutyCycle;
        state.duty_cycle = DutyCycleType::OFF;
        assert_eq!(state.to_command(), (GroupBrewControlMode::Off, None));

        state.mode = GroupBrewControlMode::Pressure;
        state.pressure = 0.0;
        assert_eq!(state.to_command(), (GroupBrewControlMode::Off, None));

        state.mode = GroupBrewControlMode::GroupFlowRate;
        state.flow_rate = 0.0;
        assert_eq!(state.to_command(), (GroupBrewControlMode::Off, None));
    }

    #[test]
    fn lifting_off_zero_returns_to_the_mode_rather_than_staying_off() {
        // The mode is the operator's, not the wire's. One press of `+` has to put the pump
        // back into the mode the screen still says it is in.
        let mut state = FreeBrewState::default();
        state.mode = GroupBrewControlMode::Pressure;
        state.pressure = 0.0;
        assert_eq!(state.to_command().0, GroupBrewControlMode::Off);

        state.adjust(true);
        assert_eq!(state.to_command().0, GroupBrewControlMode::Pressure);
    }

    #[test]
    fn seeding_from_the_controller_keeps_every_target() {
        let mut control = GroupBrewControlState::default();
        control.mode = GroupBrewControlMode::Pressure;
        control.values.pressure = 7.5;
        control.values.flow_rate = 3.0;
        control.values.duty_cycle = DutyCycleType::new(45);

        let state = FreeBrewState::from_control_state(&control);
        assert_eq!(state.mode(), GroupBrewControlMode::Pressure);
        assert_eq!(state.pressure, 7.5);
        assert_eq!(state.flow_rate, 3.0);
        assert_eq!(state.duty_cycle.value(), 45);
    }

    #[test]
    fn seeding_from_an_unoffered_mode_normalises_without_losing_targets() {
        // `Off` is what a target dialled to zero leaves behind, so this is the ordinary case
        // of starting a second free brew, not an exotic one.
        let mut control = GroupBrewControlState::default();
        control.mode = GroupBrewControlMode::Off;
        control.values.pressure = 7.5;

        let state = FreeBrewState::from_control_state(&control);
        assert_eq!(state.mode(), FREE_BREW_MODES[0]);
        assert_eq!(state.pressure, 7.5);
    }
}
