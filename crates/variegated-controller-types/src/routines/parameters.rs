use crate::*;
use alloc::string::String;
use alloc::vec::Vec;
use core::fmt;
use heapless::index_map::FnvIndexMap;

pub type UserActionIndex = u8;
pub type RoutineParameters = FnvIndexMap<u8, f32, 8>;

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum RoutineIndex {
    Internal(u32),
    Function(u32),
    Custom(u32)
}

impl RoutineIndex {
    /// Convert RoutineIndex to a bit-packed u16 storage index
    /// Bits 15-14: Type (00=Internal, 01=Function, 10=Custom)
    /// Bits 13-0: Index (0-16383)
    pub fn to_storage_index(&self) -> u16 {
        match self {
            RoutineIndex::Internal(n) => {
                debug_assert!(*n < 0x4000, "Internal routine index too large");
                (*n as u16) & 0x3FFF
            }
            RoutineIndex::Function(n) => {
                debug_assert!(*n < 0x4000, "Function routine index too large");
                0x4000 | ((*n as u16) & 0x3FFF)
            }
            RoutineIndex::Custom(n) => {
                debug_assert!(*n < 0x4000, "Custom routine index too large");
                0x8000 | ((*n as u16) & 0x3FFF)
            }
        }
    }

    /// Convert a bit-packed u16 storage index to RoutineIndex
    pub fn from_storage_index(storage_index: u16) -> Option<Self> {
        let type_bits = (storage_index >> 14) & 0x03;
        let index = (storage_index & 0x3FFF) as u32;

        match type_bits {
            0b00 => Some(RoutineIndex::Internal(index)),
            0b01 => Some(RoutineIndex::Function(index)),
            0b10 => Some(RoutineIndex::Custom(index)),
            _ => None, // 0b11 is reserved
        }
    }

    /// Get the inner index value
    pub fn inner(&self) -> u32 {
        match self {
            RoutineIndex::Internal(n) | RoutineIndex::Function(n) | RoutineIndex::Custom(n) => *n,
        }
    }
}

impl core::fmt::Display for RoutineIndex {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            RoutineIndex::Internal(n) => write!(f, "Internal({})", n),
            RoutineIndex::Function(n) => write!(f, "Function({})", n),
            RoutineIndex::Custom(n) => write!(f, "Custom({})", n),
        }
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
// `PartialEq` and not `Eq`: `Static` holds an `f32`. Derives do not touch the encoding, so
// this is not a format change -- it is what lets `TransitionOrigin` below compare, and what
// lets a test assert on a command rather than on its `Debug` rendering.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ParameterValue {
    Static(f32),
    Parameter(u8), // index into parameter map
    DerivedParameter(u8), // index into derived parameter list (separate namespace)
}

impl fmt::Display for ParameterValue {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ParameterValue::Static(value) => write!(f, "{:.1}", value),
            ParameterValue::Parameter(index) => write!(f, "P{}", index),
            ParameterValue::DerivedParameter(index) => write!(f, "D{}", index),
        }
    }
}

/// Where a transition's ramp begins.
///
/// # Why this exists
///
/// It used to be implicit, and it was wrong: every `*WithTransition` command anchored its
/// ramp to the quantity's **measurement**. When the pump is tracking well, measurement and
/// setpoint agree and nobody notices. When it is not -- which is most of a real shot, since
/// a puck is a load the pump cannot instantly overcome -- the difference is the whole
/// behaviour of the step.
///
/// Two shots made the case. A routine asking for "9 bar over 1 s" then "decline to 4 bar
/// over 30 s" produced a *flat line at 4 bar*: real pressure had only reached 3.995 bar when
/// the second command ran, so the ramp was built from 3.995 to 4.0 and did nothing. Rewritten
/// as a staircase -- 9, 8, 7, 6, 5, 4 -- every "decline" instead ramped *upward*, because
/// each step re-anchored to a measurement that was still climbing.
///
/// Naming it makes the choice the routine author's, and makes a step's behaviour readable
/// from the step alone rather than from everything that ran before it -- which also makes it
/// well-defined when a `JumpToStep` arrives from somewhere unexpected.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TransitionOrigin {
    /// A value the routine names: static, a parameter, or a derived parameter.
    ///
    /// Composed rather than repeating [`ParameterValue`]'s three variants, so the two cannot
    /// drift apart and `resolve_parameter_value` serves both.
    Value(ParameterValue),
    /// What the quantity is currently *commanded* to.
    ///
    /// **The one to reach for**, and what a reader means by "transition" nearly always:
    /// continue from wherever the last ramp left the setpoint. Expressed once, it cannot
    /// disagree with the step before it the way a written-out `Value` can.
    ///
    /// Read from [`crate::GroupStatus::brew_control_target`], which is published for this.
    /// If the group is not currently being commanded in this quantity -- a pressure
    /// transition while the group is in flow mode -- there is no such target, and the
    /// command falls back to starting at its own destination. See the resolution in
    /// `variegated-controller-lib`.
    CurrentTarget,
    /// What the quantity currently *measures*.
    ///
    /// The old behaviour, kept because it is occasionally what you want: a deliberate
    /// resynchronisation to what the machine actually did, rather than to what it was asked
    /// to do. Choosing it is now a decision rather than an accident.
    CurrentValue,
}

impl TransitionOrigin {
    /// The default a routine editor should offer. See [`Self::CurrentTarget`].
    pub const fn default_for_editor() -> Self {
        Self::CurrentTarget
    }
}

impl fmt::Display for TransitionOrigin {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TransitionOrigin::Value(value) => write!(f, "{}", value),
            TransitionOrigin::CurrentTarget => write!(f, "current target"),
            TransitionOrigin::CurrentValue => write!(f, "current value"),
        }
    }
}

impl ParameterValue {
    // For display purposes, treat Parameter as a placeholder value
    pub fn as_secs(&self) -> u64 {
        match self {
            ParameterValue::Static(value) => *value as u64,
            ParameterValue::Parameter(_) => 0, // Placeholder - should be resolved
            ParameterValue::DerivedParameter(_) => 0, // Placeholder - should be resolved
        }
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ParameterUnit {
    Seconds,
    Celsius,
    Bar,
    MillilitersPerSecond,
    Grams,
    Percent,
    /// Volume, as `StateCondition::InputVolumeAboveRelativeToStart` measures it.
    ///
    /// Appended, not inserted: postcard encodes an enum as its declaration-order
    /// discriminant, so anything but the end would renumber the variants after it and
    /// silently mis-decode every stored routine that names a unit.
    ///
    /// It was missing because no display could render that condition -- the three renderers
    /// covered 25, 13 and 8 of the sixteen `StateCondition` variants between them, and this
    /// was in none of them. The exhaustive match in `routine_progress` is what surfaced it.
    Milliliters,
    /// Electrical conductivity, mS/cm. See [`crate::ECType`].
    ///
    /// Millisiemens, not micro. Three consumers disagreed about this -- the firmware printed
    /// it bare, the web status card said uS/cm and the companion app said nothing -- and
    /// naming the unit here is what settles it.
    MillisiemensPerCentimeter,
    /// Extraction rate, mS*ml/(cm*s). See [`crate::ExtractionRateType`].
    ///
    /// A composite rather than a named SI unit, because that is what the quantity is:
    /// conductivity times output flow. Spelling it out beats the "%" the web UI used to
    /// show, which was not a percentage of anything.
    ExtractionRate,
    /// Extracted solids, mS*ml/cm. See [`crate::ExtractedSolidsType`].
    ///
    /// The time integral of [`Self::ExtractionRate`].
    ExtractedSolids,
}

/// How many static shot attributes a routine may carry.
///
/// Four, not [`crate::MAX_SHOT_ANNOTATIONS`] (8). A maximal annotation block is 804 bytes
/// -- see [`crate::ShotAnnotations`] for where that number comes from -- against a
/// [`crate::ROUTINE_MAX_ENCODED_LEN`] of 2048, so allowing eight would let a routine spend
/// 40% of its entire encoding budget on metadata and leave too little for the steps that
/// are the point of it. Four covers "this routine always runs with this coffee at this
/// grind" with room to spare.
pub const MAX_ROUTINE_SHOT_ANNOTATIONS: usize = 4;

/// How many prerequisites a routine may declare.
///
/// There are only seven [`SensorCapability`] variants and a routine that needed all of them
/// would be a routine no machine can run, so this is a bound on nonsense rather than on
/// anything real. It is worth having because prerequisites are the one part of a routine
/// copied wholesale into [`RoutineSummary`], which crosses the link in a list of *every*
/// routine -- so an unbounded list is paid for by every client on every poll, not only by
/// whoever opens the routine.
pub const MAX_ROUTINE_PREREQUISITES: usize = 8;

/// A sensing capability a routine cannot run without.
///
/// Declared on the routine and checked before it starts, because the alternative is what
/// used to happen: a brew-by-weight routine on a machine with no scale started happily,
/// `StateCondition::OutputWeightAbove` read a missing sensor as `false`, and the exit
/// simply never fired. Nothing refused and nothing warned.
///
/// A [`SensorCapability`] rather than a [`crate::PeripheralId`]. Ids are hard-coded per
/// firmware -- `GRAVITY_PERIPHERAL_ID`, `BELKA_PERIPHERAL_ID` and friends are written out
/// separately in each `main.rs` -- so a routine authored on a GS3 would name a peripheral a
/// Silvia has never heard of. Capability is what `MachineDefinition.peripherals` already
/// declares, and it is the same word on both machines.
///
/// **Not group-scoped**, deliberately. `PeripheralDefinition` carries a freeform `location`
/// string and no group index, so "the scale under group 1" is not expressible against the
/// data that exists; adding it means adding a group to `PeripheralDefinition` first. Both
/// machines this firmware supports are single-group, so the distinction is currently inert.
///
/// `Copy` and `Eq` so that `RoutineSummary` stays cheaply comparable -- see
/// [`SensorCapability`].
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RoutinePrerequisite {
    pub capability: SensorCapability,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone, Debug)]
pub struct RoutineParameter {
    pub index: u8,
    pub name: String,  // User-facing, e.g. "Preinfusion Time", "Target Pressure"
    pub default: f32,
    pub unit: Option<ParameterUnit>,
    /// The numeric shot attribute this parameter mirrors, if any.
    ///
    /// A dose is one number that a user currently types twice: once as a routine parameter
    /// with `ParameterUnit::Grams`, and once as a `ShotAnnotationKey::DoseWeight`. Linking
    /// them makes the parameter screen seed itself from a captured dose, and makes the value
    /// the routine actually ran with the value recorded against the shot.
    ///
    /// **Numeric keys only.** A parameter is an `f32`, so only keys whose value is a
    /// [`crate::ShotAnnotationValue::Number`] round-trip: `DoseWeight`, and `Other(..)` when
    /// what is stored there happens to be numeric. `Beans` and `GrindSize` are `Text` on
    /// purpose -- grinders number their settings incompatibly and some do not number them at
    /// all -- and are not linkable. A `Text` value found on a linked key is ignored rather
    /// than parsed, for the reason `ShotAnnotations::dose_weight` gives: a value that arrived
    /// as text was not measured, and guessing at it is worse than having none.
    ///
    /// Appended, so a routine that names no link encodes one extra byte.
    pub linked_attribute: Option<ShotAnnotationKey>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone, Debug)]
pub struct DerivedParameter {
    pub index: u8,  // Separate index space from regular parameters
    pub name: String,
    pub unit: Option<ParameterUnit>,
    pub formula: DerivedFormula,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone, Debug)]
pub enum DerivedFormula {
    Linear {
        base_param: u8,      // Index of base parameter
        multiplier: f32,
        offset: f32,
    },
    Sum {
        params: Vec<u8>,     // Indices of parameters to sum
    },
    Difference {
        param_a: u8,
        param_b: u8,         // a - b
    },
    Product {
        params: Vec<u8>,     // Indices of parameters to multiply
    },
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug)]
pub enum StateCondition {
    Brewing(GroupIndex),
    NotBrewing(GroupIndex),
    BoilerTemperatureAbove(BoilerIndex, ParameterValue),
    BoilerTemperatureBelow(BoilerIndex, ParameterValue),
    BoilerPressureAbove(BoilerIndex, ParameterValue),
    BoilerPressureBelow(BoilerIndex, ParameterValue),
    GroupInputFlowRateAbove(GroupIndex, ParameterValue),
    GroupInputFlowRateBelow(GroupIndex, ParameterValue),
    GroupPressureAbove(GroupIndex, ParameterValue),
    GroupPressureBelow(GroupIndex, ParameterValue),
    WaterTapFlowRateAbove(WaterTapIndex, ParameterValue),
    WaterTapFlowRateBelow(WaterTapIndex, ParameterValue),
    OutputWeightAbove(GroupIndex, ParameterValue),
    OutputWeightBelow(GroupIndex, ParameterValue),
    InputVolumeAboveRelativeToStart(GroupIndex, ParameterValue),

    // The extraction conditions. Appended, not inserted -- postcard encodes an enum as its
    // declaration-order discriminant, so anything but the end renumbers every variant after
    // it and silently mis-decodes every stored routine.
    //
    // All three quantities were already computed, logged and displayed; nothing could end a
    // step on them.
    /// Conductivity leaving the group, mS/cm. Reads `GroupStatus.output_electrical_conductivity`.
    GroupOutputConductivityAbove(GroupIndex, ParameterValue),
    GroupOutputConductivityBelow(GroupIndex, ParameterValue),
    /// Conductivity times output flow. Reads `GroupStatus.extraction_rate`.
    ///
    /// Note what the controller does when a group reports no output flow: it substitutes
    /// *input* flow, which is a different quantity. A condition on this is only as
    /// trustworthy as the group's flow sensing.
    GroupExtractionRateAbove(GroupIndex, ParameterValue),
    GroupExtractionRateBelow(GroupIndex, ParameterValue),
    /// Solids in the cup so far. Reads `BrewStatus.extracted_solids`, so it is measurable
    /// **only during a brew** -- outside one the value is absent and the condition is not
    /// met, in either direction.
    ///
    /// Beware `...Below` on a machine with no conductivity probe: the accumulator is
    /// initialised to `Some(0.0)` on brew start regardless of whether a probe exists, so the
    /// condition is true immediately and forever.
    ExtractedSolidsAbove(GroupIndex, ParameterValue),
    ExtractedSolidsBelow(GroupIndex, ParameterValue),

    /// The shot has reached a phase of extraction: the puck saturating, or the first drop
    /// landing.
    ///
    /// Reads `BrewStatus.shot_state`, so it is measurable **only during a brew** -- outside
    /// one there is no shot to have a phase, and the condition is not met.
    ///
    /// This is the only condition whose operand is not a [`ParameterValue`]. A phase is not
    /// a threshold: there is nothing to compare and nothing a routine parameter could
    /// usefully vary. What "the puck is saturated" means is decided by
    /// `variegated-controller-lib`'s shot-state tracker, from the crossover between falling
    /// flow and rising pressure -- read that module before assuming this is a simple
    /// threshold on either.
    ///
    /// **Reached, not equal to** -- see [`ShotState::reached`]. Waiting for exactly a phase
    /// would hang when the machine passes through it between evaluations.
    ShotStateReached(GroupIndex, ShotState),
}
