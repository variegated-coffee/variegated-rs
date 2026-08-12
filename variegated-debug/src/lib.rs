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

// `crate::status` carries a `Box<Status>`; the allocator itself comes from the
// binary, as it does for every other crate in this tree.
extern crate alloc;
// The test harness needs std even though the crate itself is no_std.
#[cfg(test)]
extern crate std;

pub mod bus;
pub mod commands;
pub mod rate;
pub mod relay;
pub mod sampler;
pub mod status;
pub mod suppress;

// The linker symbols are `cortex-m-rt`'s, so this is meaningless anywhere else -- and the
// comms processor is a riscv32 with a wholly different memory layout. Gated on the chip
// features rather than on `usb-cdc-rp`, because reporting a stack is not a transport
// concern.
#[cfg(any(feature = "rp2040", feature = "rp235xa", feature = "rp235xb"))]
pub mod stack;

// Reports core 0's stack, so it inherits `stack`'s gate, and it describes the application
// processor specifically -- `SourceState::Application`.
#[cfg(all(
    feature = "source-application",
    any(feature = "rp2040", feature = "rp235xa", feature = "rp235xb")
))]
pub mod snapshot;

#[cfg(feature = "usb-cdc-rp")]
pub mod usb_cdc;

pub use bus::{emit_event, emit_text, publish, publish_with};
