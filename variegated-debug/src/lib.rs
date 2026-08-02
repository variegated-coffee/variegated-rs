#![no_std]
//! Device-side structured debug emission, shared by both firmwares.
//!
//! Both workspaces declare `embassy-sync = "0.8.0"` and `embassy-time = "0.5.1"`, so
//! the channel types are the same crate on the RP2350 and the ESP32-C6. The emitting
//! processor is selected at compile time by the `source-application` /
//! `source-comms` feature; enabling both, or neither, is a `compile_error!`.
//!
//! Sharing this rather than duplicating it is what keeps sequence numbering, drop
//! accounting and the non-blocking publish contract identical on both sides of the
//! link -- the host's gap detection depends on those matching.

// The test harness needs std even though the crate itself is no_std.
#[cfg(test)]
extern crate std;

pub mod bus;
pub mod rate;
pub mod sampler;
pub mod suppress;

#[cfg(feature = "usb-cdc-rp")]
pub mod usb_cdc;

pub use bus::{emit_event, emit_text, publish, publish_with};
