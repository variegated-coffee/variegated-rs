//! The parts of a machine menu that are about *routines and values* rather than about
//! navigation or pixels.
//!
//! There are three layers, and keeping them apart is what this crate is for:
//!
//! | layer | lives in | knows about |
//! |---|---|---|
//! | navigation arithmetic | `variegated-menu` | rows, nothing else |
//! | **routine and value model** | **here** | `Routine`, `RoutineParameter`, `ParameterUnit` |
//! | content and pixels | each firmware | this machine's menus, this machine's display |
//!
//! `variegated-menu` deliberately refuses to learn what a row *means*, so a routine list
//! cannot live there. But the layer above it is not per-machine either: the Silvia and the
//! GS3 both list routines out of a repository, both seed parameter values from
//! `RoutineParameter::default`, both edit one with an [`variegated_menu::Adjustable`], and
//! both format a number against a [`ParameterUnit`]. Before this crate the Silvia had the
//! only implementation of each and the GS3 was about to grow a second.
//!
//! What stays in the firmwares is what is genuinely per-machine: the menu taxonomy, the item
//! tables, the button or encoder mapping, and the drawing. What is here is everything that
//! was the same question asked twice.
//!
//! Nothing in this crate allocates or touches hardware, so all of it is host-testable --
//! which matters because neither firmware crate can host a test binary at all. Both set
//! `test = false` on their only target and depend on `embassy-rp`.

#![no_std]
#![warn(missing_docs)]

extern crate alloc;

mod brew;
mod format;
mod listing;
mod parameters;
mod rows;
mod schedule;
mod temperature;

pub use brew::{
    brew_mode_label, brew_target, duty_cycle_from_editor, next_brew_mode, OFFERED_BREW_MODES,
};
pub use format::{format_value, unit_suffix, UnitStyle, VALUE_TEXT_LEN};
pub use listing::{RoutineRow, RoutineRows, MAX_MENU_ROUTINES, ROUTINE_NAME_LEN, routine_rows};
pub use parameters::{
    parameter_adjustable, parameter_bounds, parameter_geometry, parameter_row,
    ParameterListChrome, ParameterRow, ParameterValues, MAX_ROUTINE_PARAMETERS,
};
pub use rows::{
    drawable, editor_rows, info_rows, list_rows, push_drawable, time_editor_rows, COLUMNS,
};
pub use schedule::{
    apply_schedule_change, schedule_action_summary, schedule_label, schedule_recurrence,
    schedule_rows, DaySet, ScheduleActionKind, ScheduleChange, ScheduleRow, ScheduleRows,
    TimeEdit, TimeField, MAX_MENU_SCHEDULES, MINUTE_STEP, SCHEDULE_LABEL_LEN,
    SCHEDULE_RECURRENCE_LEN, TIME_TEXT_LEN,
};
pub use temperature::{boiler_temperature_adjustable, BOILER_TEMPERATURE_STEP};

