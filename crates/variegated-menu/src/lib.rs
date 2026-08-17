//! Navigation and value editing for machine menus, with no opinion about pixels or content.
//!
//! Two firmwares in this workspace drive a menu: the GS3 with four of its six panel buttons,
//! the Silvia with a rotary encoder. They share no display stack -- 428x168 RGB565 against
//! 128x64 mono -- and no menu content. What they do share is the arithmetic, and that
//! arithmetic is where the bugs were: a selection index that counted one thing and a scroll
//! offset that counted another, and three implementations of "adjust a value with clamping"
//! that agreed with each other nowhere.
//!
//! So: **mechanism here, content and pixels in the firmware.** Callers say how many rows
//! there are; nothing in this crate ever learns what a row means.
//!
//! Neither firmware crate can host a test binary -- both set `test = false` on their only
//! target and depend on `embassy-rp`. This crate can, and that is most of the reason it
//! exists.

#![no_std]
#![warn(missing_docs)]

mod adjustable;
mod list;
mod stack;

pub use adjustable::Adjustable;
pub use list::{ListGeometry, ListNav};
pub use stack::{MenuFrame, MenuStack};
