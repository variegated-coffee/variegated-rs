//! This processor's end of the structured debug stream.
//!
//! There is deliberately **no `bus.rs` here**. The bus is the one in
//! `variegated-debug`, shared with the application processor: both workspaces
//! resolve `embassy-sync = "0.8.0"` and `embassy-time = "0.5.1"`, so
//! `variegated_debug::bus::BUS` is the same type on the RP2350 and the ESP32-C6.
//! A second, parallel implementation here would have to re-derive the sequence
//! numbering, the drop accounting and the never-block-on-a-host contract, and the
//! host's gap detection depends on those being identical on both sides of the link.
//! The only thing that distinguishes this build is the `source-comms` feature,
//! which stamps every frame with `DebugSource::Comms`.
//!
//! `BUS_SUBSCRIBERS` is 2 in the shared crate and 2 is right here too: the
//! USB-Serial-JTAG writer below, and Task 11's TCP server. Publishers need no slot
//! (`bus::publish` goes through `immediate_publisher`), so relaying the application
//! processor's frames onto this bus does not consume one.
//!
//! `variegated_debug::status` is *not* touched here. It is a `Signal`, which holds
//! exactly one waker, and a second concurrent `wait()` makes two consumers ping-pong
//! wakeups and busy-loop. Task 11's TCP server is the intended single consumer on
//! this processor, so this transport reads the bus only.

pub mod panic_console;
pub mod snapshot;
pub mod usb;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Sender;
use embassy_sync::pubsub::Subscriber;
use variegated_controller_types::debug::DebugFrame;
use variegated_controller_types::debug_command::DebugCommand;

use crate::channels::DEBUG_COMMAND_CAPACITY;

/// The shared bus, re-exported so call sites in this crate stay short and so that
/// `crate::debug::bus` means the same thing here as it does in the firmware on the
/// other end of the UART.
pub use variegated_debug::bus;

/// Where decoded commands are handed off, shared by the USB reader and Task 11's
/// TCP reader.
pub type CommandSink = Sender<'static, CriticalSectionRawMutex, DebugCommand, DEBUG_COMMAND_CAPACITY>;

/// A reader of the shared debug bus.
///
/// Nameable, rather than an `impl Trait` buried in `bus::subscriber`'s return type,
/// because **the slot has to be claimed before the task that uses it runs**, and
/// that means passing one of these across a task boundary.
///
/// `embassy_sync`'s pubsub discards a published message outright when
/// `subscriber_count == 0` -- `try_publish` returns `Ok(())` without touching the
/// queue (`embassy-sync-0.8.0/src/pubsub/mod.rs:332`) -- and a subscriber created
/// later starts at `next_message_id` (`:100`), so it cannot recover what it missed.
/// Between those two facts, nothing published before the first `subscriber()` call
/// exists at all: not lost in the ring, not counted as dropped, simply never
/// queued. Claiming the slot synchronously in `main`, before the first publish and
/// with no `.await` in between, is what makes the boot frames -- `Boot`, the early
/// `SpawnFailed`s and every `log_*!` up to the first yield -- reach a reader.
pub type BusSubscriber = Subscriber<
    'static,
    CriticalSectionRawMutex,
    DebugFrame,
    { bus::BUS_CAPACITY },
    { bus::BUS_SUBSCRIBERS },
    1,
>;

/// Number of connected TCP debug clients.
///
/// Lives here rather than in a future `tcp.rs` so Task 10's snapshot task can read
/// it whether or not the TCP server is compiled in -- and here rather than on
/// `bus`, which is a re-export of the shared crate and cannot gain members that
/// only one of the two processors has.
pub static TCP_DEBUG_CLIENTS: portable_atomic::AtomicU8 = portable_atomic::AtomicU8::new(0);
