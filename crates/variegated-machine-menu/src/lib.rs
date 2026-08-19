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

mod format;
mod listing;
mod parameters;
mod temperature;

pub use format::{format_value, UnitStyle, VALUE_TEXT_LEN};
pub use listing::{RoutineRow, RoutineRows, MAX_MENU_ROUTINES, ROUTINE_NAME_LEN, routine_rows};
pub use parameters::{
    parameter_adjustable, parameter_bounds, parameter_geometry, parameter_row,
    ParameterListChrome, ParameterRow, ParameterValues, MAX_ROUTINE_PARAMETERS,
};
pub use temperature::{boiler_temperature_adjustable, BOILER_TEMPERATURE_STEP};
