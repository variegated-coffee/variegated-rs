//! Editing a routine's parameters before running it.

use variegated_controller_types::routines::core::Routine;
use variegated_controller_types::routines::parameters::{
    ParameterUnit, RoutineParameter, RoutineParameters,
};
use variegated_menu::{Adjustable, ListGeometry};

/// The most parameters a routine can have.
///
/// Not a new limit: `Routine::new` asserts it, and `RoutineParameters` is an
/// `FnvIndexMap<u8, f32, 8>`.
pub const MAX_ROUTINE_PARAMETERS: usize = 8;

/// The values a user has dialled in for one routine, before it runs.
///
/// **Positional** -- indexed the way `routine.parameters()` is, not by
/// `RoutineParameter::index`. Two reasons, and the first is the one that decided it:
///
/// - It is `Copy`, which is what lets the GS3 carry an in-progress edit in a `Watch`
///   payload. `RoutineParameters` is a `heapless::IndexMap` and is not.
/// - Writing a value cannot fail. The Silvia's version inserted into an `FnvIndexMap` keyed
///   on `RoutineParameter::index`, which returns an error on a routine with more than eight
///   parameters -- a case its call site logged a warning for and then dropped the edit.
///
/// The index-keyed form is still what `MachineCommand::RunRoutine` wants, so
/// [`ParameterValues::to_runtime`] converts at the last moment, where the `Routine` that
/// defines the mapping is in hand.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ParameterValues {
    values: [f32; MAX_ROUTINE_PARAMETERS],
    count: u8,
}

impl Default for ParameterValues {
    fn default() -> Self {
        Self { values: [0.0; MAX_ROUTINE_PARAMETERS], count: 0 }
    }
}

impl ParameterValues {
    /// Seed every parameter from `RoutineParameter::default`.
    ///
    /// A routine declaring more than [`MAX_ROUTINE_PARAMETERS`] is truncated rather than
    /// rejected. `Routine::new` asserts the limit, but a routine deserialized out of flash
    /// or off the wire never went through it.
    pub fn from_defaults(routine: &Routine) -> Self {
        let mut out = Self::default();
        for (position, param) in routine.parameters().iter().take(MAX_ROUTINE_PARAMETERS).enumerate()
        {
            out.values[position] = param.default;
            out.count = position as u8 + 1;
        }
        out
    }

    /// How many parameters are being edited.
    pub const fn len(&self) -> usize {
        self.count as usize
    }

    /// Whether the routine has no parameters at all.
    pub const fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// The value at a position, or `None` past the end.
    pub fn get(&self, position: usize) -> Option<f32> {
        if position < self.len() { Some(self.values[position]) } else { None }
    }

    /// Write a value. Out of range is a no-op, not a panic -- a menu is not worth taking the
    /// machine down for.
    pub fn set(&mut self, position: usize, value: f32) {
        if position < self.len() {
            self.values[position] = value;
        }
    }

    /// The form `MachineCommand::RunRoutine` takes, keyed by `RoutineParameter::index`.
    ///
    /// `None` when the routine has no parameters, which is what both firmwares send today.
    /// It is not merely cosmetic: it says "this run adds nothing to the defaults" rather
    /// than "this run overrides with an empty set", and it is what the call sites already
    /// distinguished by hand.
    ///
    /// Pass the same `Routine` these values were seeded from. Positions are resolved against
    /// its parameter list, so a different routine would silently key the values wrongly.
    pub fn to_runtime(&self, routine: &Routine) -> Option<RoutineParameters> {
        if self.is_empty() {
            return None;
        }
        let mut map = RoutineParameters::new();
        for (position, param) in routine.parameters().iter().take(self.len()).enumerate() {
            // Cannot fail: `len()` is capped at 8 and so is the map.
            let _ = map.insert(param.index, self.values[position]);
        }
        Some(map)
    }
}

/// What a parameter screen puts around the parameters themselves.
///
/// Chrome is a per-machine choice, the same way `ListGeometry::wrap` already is. The Silvia
/// draws a `<-` row because a rotary encoder has no dedicated back control; the GS3 has
/// button 4 and a permanent hint row saying so, and on a four-row panel a row spent on
/// "Back" is a routine you cannot see.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ParameterListChrome {
    /// Whether row 0 is a back row.
    pub back_row: bool,
    /// How many rows are on screen at once.
    pub visible_rows: usize,
    /// Whether moving past an end comes back at the other.
    pub wrap: bool,
}

/// What a row of a parameter screen is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ParameterRow {
    /// Leave without running.
    Back,
    /// The parameter at this position in `routine.parameters()`.
    Parameter(usize),
    /// Run the routine.
    Execute,
}

/// The geometry of a parameter screen: optional back row, the parameters, then Execute.
///
/// `param_count` is passed in rather than read off a [`ParameterValues`], because the
/// renderer draws `routine.parameters()` and a row count taken from anywhere else is a second
/// source of truth for the same number.
pub fn parameter_geometry(param_count: usize, chrome: ParameterListChrome) -> ListGeometry {
    ListGeometry {
        total_rows: chrome.back_row as usize + param_count + 1,
        visible_rows: chrome.visible_rows,
        wrap: chrome.wrap,
    }
}

/// What the row at `row` is.
///
/// Anything past the last parameter is [`ParameterRow::Execute`], so a selection left behind
/// by a list that shrank lands on Execute rather than on a parameter that no longer exists.
pub fn parameter_row(row: usize, param_count: usize, chrome: ParameterListChrome) -> ParameterRow {
    if chrome.back_row && row == 0 {
        return ParameterRow::Back;
    }
    let position = row - chrome.back_row as usize;
    if position < param_count { ParameterRow::Parameter(position) } else { ParameterRow::Execute }
}

/// The range and step for a parameter, derived from its unit.
///
/// `RoutineParameter` carries `index`, `name`, `default` and `unit` and **no min, max or
/// step**, and it must not gain them: its postcard encoding is positional and unversioned, so
/// every routine already in a machine's flash and every schema the frontend generates would
/// ride along on that change.
///
/// So the bounds come from the unit instead. Before this existed the Silvia used
/// `(0.0, f32::INFINITY, 0.5)` for every parameter of every routine -- a comment at that site
/// asked for exactly this and called it "a separate change".
///
/// `None` keeps that range: a parameter with no declared unit is a quantity this crate
/// genuinely knows nothing about, and inventing a ceiling for it would be worse than having
/// none. Its **step** is a choice rather than an inheritance, and it is 0.1 rather than the
/// historical 0.5. `ParameterUnit` has no ratio, so a brew ratio is authored unitless and
/// lands here -- and a ratio lives between about 1.5 and 3.0, which 0.5 crosses in three
/// presses. Small dimensionless numbers are what reaches this arm in practice; anything
/// coarser than they need has a unit to say so.
pub const fn parameter_bounds(unit: Option<ParameterUnit>) -> (f32, f32, f32) {
    match unit {
        Some(ParameterUnit::Seconds) => (0.0, 600.0, 0.5),
        Some(ParameterUnit::Celsius) => (0.0, 150.0, 0.5),
        Some(ParameterUnit::Bar) => (0.0, 15.0, 0.1),
        Some(ParameterUnit::MillilitersPerSecond) => (0.0, 20.0, 0.1),
        Some(ParameterUnit::Grams) => (0.0, 500.0, 0.5),
        Some(ParameterUnit::Percent) => (0.0, 100.0, 1.0),
        Some(ParameterUnit::Milliliters) => (0.0, 2000.0, 5.0),
        // Espresso runs roughly 1-3 mS/cm at the spout; brewed coffee is lower. 10 is well
        // clear of anything real without being a number a mis-set parameter can hide in.
        Some(ParameterUnit::MillisiemensPerCentimeter) => (0.0, 10.0, 0.1),
        // Conductivity times output flow, so of the order of a few mS·ml/cm·s at a typical
        // 1-2 ml/s. The ceilings here are generous rather than measured: unlike seconds or
        // bar, nobody has yet dialled these in on a real machine, and a ceiling that is too
        // low silently clamps a parameter where one that is too high merely allows a value
        // the routine will never reach.
        Some(ParameterUnit::ExtractionRate) => (0.0, 50.0, 0.1),
        // The integral of the above over a shot, so larger again.
        Some(ParameterUnit::ExtractedSolids) => (0.0, 500.0, 0.5),
        None => (0.0, f32::INFINITY, 0.1),
    }
}

/// An editor for one parameter, seeded at `current`.
pub fn parameter_adjustable(param: &RoutineParameter, current: f32) -> Adjustable {
    let (min, max, step) = parameter_bounds(param.unit);
    Adjustable::new(current, min, max, step)
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::string::ToString;
    use alloc::vec;
    use alloc::vec::Vec;
    use variegated_controller_types::routines::core::RoutineType;

    fn param(index: u8, default: f32, unit: Option<ParameterUnit>) -> RoutineParameter {
        RoutineParameter { index, name: "p".to_string(), default, unit, linked_attribute: None }
    }

    fn routine_with(parameters: Vec<RoutineParameter>) -> Routine {
        Routine::new(RoutineType::UserDefined, "r".to_string(), parameters, vec![], vec![])
    }

    const GS3: ParameterListChrome =
        ParameterListChrome { back_row: false, visible_rows: 4, wrap: true };
    const SILVIA: ParameterListChrome =
        ParameterListChrome { back_row: true, visible_rows: 5, wrap: false };

    /// `ParameterValues` must stay `Copy`.
    ///
    /// Not a style preference: it is the payload of the GS3's `MenuSnapshot` `Watch`, and
    /// losing `Copy` breaks that at a call site in another crate with an error that names
    /// neither this type nor the reason. Asserted at compile time so the failure lands here.
    const _: () = {
        const fn assert_copy<T: Copy>() {}
        assert_copy::<ParameterValues>();
    };

    #[test]
    fn values_are_seeded_from_the_defaults() {
        let routine = routine_with(vec![param(0, 25.0, None), param(1, 93.0, None)]);
        let values = ParameterValues::from_defaults(&routine);
        assert_eq!(values.len(), 2);
        assert_eq!(values.get(0), Some(25.0));
        assert_eq!(values.get(1), Some(93.0));
        assert_eq!(values.get(2), None);
    }

    #[test]
    fn to_runtime_keys_by_parameter_index_not_by_position() {
        // The mapping is the whole reason this conversion exists. A routine's parameter
        // indices are not required to be contiguous or to start at zero.
        let routine = routine_with(vec![param(3, 1.0, None), param(7, 2.0, None)]);
        let mut values = ParameterValues::from_defaults(&routine);
        values.set(0, 11.0);
        values.set(1, 22.0);

        let map = values.to_runtime(&routine).expect("two parameters");
        assert_eq!(map.get(&3), Some(&11.0));
        assert_eq!(map.get(&7), Some(&22.0));
        assert_eq!(map.get(&0), None, "position 0 is parameter 3, not parameter 0");
    }

    #[test]
    fn a_routine_with_no_parameters_sends_none() {
        let routine = routine_with(vec![]);
        let values = ParameterValues::from_defaults(&routine);
        assert!(values.is_empty());
        assert!(values.to_runtime(&routine).is_none());
    }

    #[test]
    fn setting_past_the_end_changes_nothing() {
        let routine = routine_with(vec![param(0, 1.0, None)]);
        let mut values = ParameterValues::from_defaults(&routine);
        let before = values;
        values.set(4, 99.0);
        assert_eq!(values, before);
    }

    #[test]
    fn more_parameters_than_the_cap_are_truncated_rather_than_panicking() {
        // `Routine::new` asserts eight, but a routine deserialized from flash or off the
        // wire never went through it.
        let routine = Routine {
            version: variegated_controller_types::ROUTINE_FORMAT_VERSION,
            routine_type: RoutineType::UserDefined,
            name: "r".to_string(),
            parameters: (0..12).map(|i| param(i, i as f32, None)).collect(),
            derived_parameters: vec![],
            steps: vec![],
            finally: vec![],
            prerequisites: vec![],
            shot_annotations: vec![],
        };
        let values = ParameterValues::from_defaults(&routine);
        assert_eq!(values.len(), MAX_ROUTINE_PARAMETERS);
        assert_eq!(values.to_runtime(&routine).unwrap().len(), MAX_ROUTINE_PARAMETERS);
    }

    #[test]
    fn the_gs3_screen_is_parameters_then_execute() {
        assert_eq!(parameter_geometry(2, GS3).total_rows, 3);
        assert_eq!(parameter_row(0, 2, GS3), ParameterRow::Parameter(0));
        assert_eq!(parameter_row(1, 2, GS3), ParameterRow::Parameter(1));
        assert_eq!(parameter_row(2, 2, GS3), ParameterRow::Execute);
    }

    #[test]
    fn the_silvia_screen_keeps_its_back_row() {
        assert_eq!(parameter_geometry(2, SILVIA).total_rows, 4);
        assert_eq!(parameter_row(0, 2, SILVIA), ParameterRow::Back);
        assert_eq!(parameter_row(1, 2, SILVIA), ParameterRow::Parameter(0));
        assert_eq!(parameter_row(2, 2, SILVIA), ParameterRow::Parameter(1));
        assert_eq!(parameter_row(3, 2, SILVIA), ParameterRow::Execute);
    }

    #[test]
    fn a_routine_with_no_parameters_still_has_an_execute_row() {
        // The GS3 shows the parameter screen even for a zero-parameter routine, deliberately:
        // selecting a row must never be the thing that starts hot water flowing.
        assert_eq!(parameter_geometry(0, GS3).total_rows, 1);
        assert_eq!(parameter_row(0, 0, GS3), ParameterRow::Execute);

        assert_eq!(parameter_geometry(0, SILVIA).total_rows, 2);
        assert_eq!(parameter_row(0, 0, SILVIA), ParameterRow::Back);
        assert_eq!(parameter_row(1, 0, SILVIA), ParameterRow::Execute);
    }

    #[test]
    fn a_row_past_the_end_is_execute_rather_than_a_missing_parameter() {
        assert_eq!(parameter_row(9, 2, GS3), ParameterRow::Execute);
        assert_eq!(parameter_row(9, 2, SILVIA), ParameterRow::Execute);
    }

    #[test]
    fn an_unspecified_unit_is_unbounded_and_steps_finely() {
        // Two different decisions, and only the first is inherited. The Silvia used
        // `(0.0, f32::INFINITY, 0.5)` for every parameter of every routine; the *range* is
        // kept because this crate knows nothing about the quantity, but the step is 0.1
        // because a brew ratio has no unit to be declared with and 0.5 spans its whole
        // useful width. Restoring the 0.5 would make ratios unadjustable again.
        assert_eq!(parameter_bounds(None), (0.0, f32::INFINITY, 0.1));
    }

    #[test]
    fn every_unit_has_a_usable_range() {
        for unit in [
            ParameterUnit::Seconds,
            ParameterUnit::Celsius,
            ParameterUnit::Bar,
            ParameterUnit::MillilitersPerSecond,
            ParameterUnit::Grams,
            ParameterUnit::Percent,
            ParameterUnit::Milliliters,
        ] {
            let (min, max, step) = parameter_bounds(Some(unit));
            assert!(min < max, "{unit:?} has an empty range");
            assert!(step > 0.0, "{unit:?} has a zero step");
            assert!(max.is_finite(), "{unit:?} has no ceiling");
            assert!(step <= max - min, "{unit:?} cannot be moved within its range");
        }
    }

    #[test]
    fn an_editor_clamps_to_its_units_ceiling() {
        let p = param(0, 9.0, Some(ParameterUnit::Bar));
        let mut a = parameter_adjustable(&p, 9.0);
        for _ in 0..1000 {
            a.increase();
        }
        assert_eq!(a.value(), 15.0);
    }

    #[test]
    fn an_out_of_range_stored_value_is_pulled_back_in() {
        // How a parameter edited before this crate existed arrives: the Silvia had no
        // ceiling at all, so a stored routine can carry any number.
        let p = param(0, 0.0, Some(ParameterUnit::Percent));
        assert_eq!(parameter_adjustable(&p, 4000.0).value(), 100.0);
    }
}
