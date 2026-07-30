#![no_std]
//! Device-side structured debug emission for the RP2350 application processor.
//!
//! Not shared with the comms firmware: that workspace resolves embassy-sync 0.8
//! while this one resolves 0.7, so the channel types are different crates. The
//! comms firmware has its own equivalent bus.

// The test harness needs std even though the crate itself is no_std.
#[cfg(test)]
extern crate std;

pub mod bus;
pub mod rate;
pub mod sampler;

#[cfg(feature = "usb-cdc-rp")]
pub mod usb_cdc;

pub use bus::{emit_event, emit_text, publish, publish_with};
