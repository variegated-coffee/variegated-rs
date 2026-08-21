use crate::*;
use variegated_control_algorithm::pid::PidOut;

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ControlCurve {
    pub a: f32,  // coefficient for T^2
    pub b: f32,  // coefficient for T
    pub c: f32,  // constant term
    pub min: f32, // minimum allowed value
    pub max: f32, // maximum allowed value
}

impl ControlCurve {
    pub fn evaluate(&self, time_seconds: f32) -> f32 {
        let value = self.a * time_seconds * time_seconds + self.b * time_seconds + self.c;
        value.clamp(self.min, self.max)
    }
}

/// The control mode for group brewing
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum GroupBrewControlMode {
    GroupFlowRate,        // Control pump to achieve flow rate at group
    GroupFlowRateCurve,   // Follow a flow rate curve over time
    Pressure,             // Control pump to achieve pressure
    PressureCurve,        // Follow a pressure curve over time
    OutputFlowRate,       // Control based on output (scale) flow rate
    OutputFlowRateCurve,  // Follow output flow curve over time
    FixedDutyCycle,       // Fixed pump duty cycle
    FixedDutyCycleCurve,  // Follow duty cycle curve over time
    FullOn,               // Pump at 100%
    #[default]
    Off                   // Pump off
}

/// Which quantity, if any, caps the pump while a *different* one is being controlled.
///
/// Orthogonal to [`GroupBrewControlMode`] rather than a variant of it: any brew mode can
/// carry a limit, including the open-loop ones -- "preinfuse at fixed duty but do not exceed
/// 4 bar" is the commonest use of one.
///
/// # What a limit is, and is not
///
/// It is **not** a clamp on the pump's output. When a limit binds, control authority moves:
/// a pressure-controlled step that hits its flow limit stops controlling pressure and starts
/// controlling *flow*, at the limit, until the puck lets pressure recover. Both of the
/// formats this was built to accept work this way -- Meticulous compiles a limit into a
/// sibling controller node, Decent carries a per-frame `MaxFlowOrPressure` -- and the
/// mechanism here is the third form of the same idea, a min-select override. See
/// `variegated-controller-lib`'s `pump_limit`.
///
/// Unit variants, like [`GroupBrewControlMode`]'s: the values live in
/// [`GroupBrewControlTargetValues`] and this selects which one is armed. The names mirror
/// that enum's deliberately, because each one resolves to the PID parameter set already
/// tuned for controlling that quantity -- a limit loop is the same physical loop as the
/// control loop, with a different setpoint.
///
/// **One at a time.** Not an oversight: no shared profile in either ecosystem arms more than
/// one limit on a step, and Decent's format cannot express more than one at all.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum GroupBrewLimitMode {
    /// The pump answers to [`GroupBrewControlMode`] alone.
    ///
    /// A variant rather than wrapping the field in `Option`, so that the selector's match has
    /// to name this case instead of letting it disappear into a `map`. Called `Unlimited` and
    /// not `None` so `state.limit` never reads ambiguously against `Option::None`.
    #[default]
    Unlimited,
    /// Cap pressure at the group. Pairs with a flow-controlled mode.
    MaxPressure,
    /// Cap flow *into* the group, as the pump measures it. Pairs with a pressure-controlled
    /// mode, which is the commonest limit in shared profiles by a wide margin.
    MaxGroupFlowRate,
    /// Cap flow *out of* the group, as the scale measures it.
    ///
    /// Ours alone -- neither reference format has an equivalent, since both mean pump-side
    /// flow by "flow".
    ///
    /// On a machine with no scale the reading is absent and the controller substitutes zero,
    /// permanently below any cap, so this never binds. **That is a property of the seeding
    /// and tracking in `variegated-controller-lib`'s `pump_limit`, not of the arithmetic.**
    /// An unseeded limit loop's first output is `kp * error` and nothing else -- for a
    /// 1.5 ml/s cap against a reading of zero, about 15 on the pump's 0-255 scale, which
    /// beats whatever the main loop is asking for and shuts the pump. It is safe because the
    /// loop is seeded from the commanded output when it engages and tracked to the selected
    /// output thereafter, so it only falls below when its own error goes negative.
    ///
    /// Declare [`crate::SensorCapability::OutputFlowRate`] as a routine prerequisite anyway,
    /// so a routine that depends on this refuses rather than quietly running unlimited.
    MaxOutputFlowRate,
}

impl GroupBrewLimitMode {
    /// Whether anything is armed.
    pub fn is_armed(self) -> bool {
        !matches!(self, GroupBrewLimitMode::Unlimited)
    }
}

/// All stored target values for group brew control
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GroupBrewControlTargetValues {
    pub flow_rate: FlowRateType,
    pub flow_rate_curve: ControlCurve,
    pub pressure: PressureType,
    pub pressure_curve: ControlCurve,
    pub output_flow_rate: FlowRateType,
    pub output_flow_rate_curve: ControlCurve,
    /// A percentage. This is an operator-authored setpoint, so it stays on the scale an
    /// operator reasons in; the conversion to the pump's 0-255 scale happens once, in the
    /// controller, at the point the mechanism is driven.
    pub duty_cycle: DutyCycleType,
    /// Also authored in percent, and evaluated as an `f32`, so nothing is lost crossing to
    /// the pump's scale.
    pub duty_cycle_curve: ControlCurve,

    // The limit setpoints. Appended, and carried all at once the way every other setpoint
    // here is: `GroupBrewLimitMode` selects which is live, exactly as `mode` does above.
    //
    // **Their defaults are deliberately permissive** -- the top of each quantity's range,
    // which is the same number the sibling `*_curve` defaults already use as their `max`.
    // Arming a limit whose value nobody set must do nothing. A default of `0.0` would make
    // `MaxPressure` mean "never build pressure", which is the worst possible reading of a
    // field left untouched.
    /// Cap for [`GroupBrewLimitMode::MaxPressure`], in bar.
    pub max_pressure: PressureType,
    /// Cap for [`GroupBrewLimitMode::MaxGroupFlowRate`], in ml/s.
    pub max_group_flow_rate: FlowRateType,
    /// Cap for [`GroupBrewLimitMode::MaxOutputFlowRate`], in ml/s.
    pub max_output_flow_rate: FlowRateType,
}

impl Default for GroupBrewControlTargetValues {
    fn default() -> Self {
        Self {
            flow_rate: 2.5,  // Default 2.5 ml/s
            flow_rate_curve: ControlCurve { a: 0.0, b: 2.5, c: 0.0, min: 0.0, max: 10.0 },
            pressure: 9.0,   // Default 9 bar
            pressure_curve: ControlCurve { a: 0.0, b: 0.0, c: 9.0, min: 0.0, max: 15.0 },
            output_flow_rate: 2.0,  // Default 2.0 ml/s output
            output_flow_rate_curve: ControlCurve { a: 0.0, b: 2.0, c: 0.0, min: 0.0, max: 10.0 },
            duty_cycle: DutyCycle::FULL,  // Default 100%
            duty_cycle_curve: ControlCurve { a: 0.0, b: 0.0, c: 100.0, min: 0.0, max: 100.0 },
            // Permissive, and matching the `max` of the curve defaults above for the same
            // quantities. See the field comments.
            max_pressure: 15.0,
            max_group_flow_rate: 10.0,
            max_output_flow_rate: 10.0,
        }
    }
}

/// Update structure for changing group brew target values
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GroupBrewControlTargetValuesUpdate {
    pub flow_rate: Option<FlowRateType>,
    pub flow_rate_curve: Option<ControlCurve>,
    pub pressure: Option<PressureType>,
    pub pressure_curve: Option<ControlCurve>,
    pub output_flow_rate: Option<FlowRateType>,
    pub output_flow_rate_curve: Option<ControlCurve>,
    /// A percentage -- see [`GroupBrewControlTargetValues::duty_cycle`].
    pub duty_cycle: Option<DutyCycleType>,
    pub duty_cycle_curve: Option<ControlCurve>,
    pub max_pressure: Option<PressureType>,
    pub max_group_flow_rate: Option<FlowRateType>,
    pub max_output_flow_rate: Option<FlowRateType>,
}

/// Complete group brew control state
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GroupBrewControlState {
    pub mode: GroupBrewControlMode,
    /// Which quantity, if any, caps the pump while `mode`'s quantity is being controlled.
    ///
    /// Appended after `values` would have been the tidier place topically, but this belongs
    /// next to `mode` because that is what it is: a second selector over the same `values`.
    pub limit: GroupBrewLimitMode,
    pub values: GroupBrewControlTargetValues,
}

impl Default for GroupBrewControlState {
    fn default() -> Self {
        Self {
            mode: GroupBrewControlMode::Off,
            limit: GroupBrewLimitMode::Unlimited,
            values: GroupBrewControlTargetValues::default(),
        }
    }
}

/// What a **heating element** is being driven at, as a percentage.
///
/// The pump has its own carrier, [`PumpOutput`], because the two run on different scales --
/// see [`crate::duty_cycle`]. Splitting them is what stops a 0-255 pump value being read as
/// a percentage, which is a difference of 2.55x that looks entirely plausible in a log.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum Output {
    #[default]
    Off,
    FixedDutyCycle(DutyCycleType),
    PidOutput(PidOut<f32>),
}

impl Output {
    pub fn duty_cycle(&self) -> DutyCycleType {
        match self {
            Output::Off => DutyCycle::OFF,
            Output::FixedDutyCycle(duty_cycle) => *duty_cycle,
            Output::PidOutput(pid_out) => DutyCycle::from_f32(pid_out.out),
        }
    }
}

/// What the **pump** is being driven at, on the pump's own 0-255 scale.
///
/// `PidOutput`'s `out` is denominated in that scale too: the pump PID computes natively in
/// 0-255, so its integrator, clamps and gains are all in those units.
///
/// Both scales are readable from here -- [`PumpOutput::hexadecimal_duty_cycle`] is what was
/// commanded, and [`PumpOutput::duty_cycle`] derives the percentage for the surfaces that
/// display one. Only the raw value travels on the wire; the percentage is computed, so the
/// two can never disagree.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum PumpOutput {
    #[default]
    Off,
    FixedDutyCycle(HexadecimalDutyCycleType),
    PidOutput(PidOut<f32>),
}

impl PumpOutput {
    /// What was commanded, 0-255.
    pub fn hexadecimal_duty_cycle(&self) -> HexadecimalDutyCycleType {
        match self {
            PumpOutput::Off => HexadecimalDutyCycle::OFF,
            PumpOutput::FixedDutyCycle(duty_cycle) => *duty_cycle,
            // Saturating rather than a bare cast: an unclamped PID output used to be able
            // to produce a "percentage" of 254, which then overflowed arithmetic downstream
            // that trusted the 0-100 bound.
            PumpOutput::PidOutput(pid_out) => HexadecimalDutyCycle::from_f32(pid_out.out),
        }
    }

    /// The same value as a percentage, for display. Lossy, and derived rather than stored.
    pub fn duty_cycle(&self) -> DutyCycleType {
        self.hexadecimal_duty_cycle().into()
    }
}

/// Shot state during espresso extraction
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ShotState {
    /// Initial phase: filling the headspace and wetting the puck
    HeadspaceFill,
    /// Middle phase: puck is saturating, flow decreases and pressure builds
    Saturation,
    /// Final phase: first drops detected on scale, extraction underway
    PostFirstDrop,
}

impl ShotState {
    /// How far through a shot this phase is.
    ///
    /// Declaration order is the physical order, and nothing skips backwards within a shot.
    pub fn rank(self) -> u8 {
        match self {
            ShotState::HeadspaceFill => 0,
            ShotState::Saturation => 1,
            ShotState::PostFirstDrop => 2,
        }
    }

    /// Whether the shot has got at least as far as `phase`.
    ///
    /// **At least as far, not equal to**, and that is the whole reason this exists rather
    /// than callers writing `==`. A routine step waiting for saturation is asking "has the
    /// puck wetted through", not "is the machine in that exact phase this instant" -- and
    /// the phases are transient. `PostFirstDrop` can arrive in the same 400 ms evaluation
    /// window that `Saturation` did on a fast, coarse shot, and a step comparing for
    /// equality would then wait forever for a phase the machine has already left.
    pub fn reached(self, phase: ShotState) -> bool {
        self.rank() >= phase.rank()
    }
}

#[cfg(test)]
mod shot_state_tests {
    use super::*;

    #[test]
    fn a_later_phase_has_reached_an_earlier_one() {
        // The property a routine step depends on. Once the first drop has landed the puck is
        // certainly saturated, so a step waiting on saturation must fire even though the
        // machine has already moved past it -- both phases can be crossed inside one 400 ms
        // evaluation window on a fast shot, and `==` would hang there forever.
        assert!(ShotState::PostFirstDrop.reached(ShotState::Saturation));
        assert!(ShotState::PostFirstDrop.reached(ShotState::HeadspaceFill));
        assert!(ShotState::Saturation.reached(ShotState::HeadspaceFill));
    }

    #[test]
    fn an_earlier_phase_has_not_reached_a_later_one() {
        assert!(!ShotState::HeadspaceFill.reached(ShotState::Saturation));
        assert!(!ShotState::Saturation.reached(ShotState::PostFirstDrop));
    }

    #[test]
    fn a_phase_has_reached_itself() {
        for phase in [ShotState::HeadspaceFill, ShotState::Saturation, ShotState::PostFirstDrop] {
            assert!(phase.reached(phase));
        }
    }

    #[test]
    fn rank_follows_declaration_order() {
        // The ranks are what `reached` compares, and declaration order is the physical order
        // of a shot. A variant inserted rather than appended would renumber the postcard
        // discriminants *and* silently reorder these, so this pins both at once.
        assert_eq!(ShotState::HeadspaceFill.rank(), 0);
        assert_eq!(ShotState::Saturation.rank(), 1);
        assert_eq!(ShotState::PostFirstDrop.rank(), 2);
    }
}
