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
    pub duty_cycle: u8,
    pub duty_cycle_curve: ControlCurve,
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
            duty_cycle: 100,  // Default 100%
            duty_cycle_curve: ControlCurve { a: 0.0, b: 0.0, c: 100.0, min: 0.0, max: 100.0 },
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
    pub duty_cycle: Option<u8>,
    pub duty_cycle_curve: Option<ControlCurve>,
}

/// Complete group brew control state
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GroupBrewControlState {
    pub mode: GroupBrewControlMode,
    pub values: GroupBrewControlTargetValues,
}

impl Default for GroupBrewControlState {
    fn default() -> Self {
        Self {
            mode: GroupBrewControlMode::Off,
            values: GroupBrewControlTargetValues::default(),
        }
    }
}

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
            Output::Off => 0,
            Output::FixedDutyCycle(duty_cycle) => *duty_cycle as DutyCycleType,
            Output::PidOutput(pid_out) => pid_out.out as DutyCycleType,
        }
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
