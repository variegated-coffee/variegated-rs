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
//! `BUS_SUBSCRIBERS` is 2 in the shared crate and 2 is right here too, and both
//! slots are now spoken for: the USB-Serial-JTAG writer below, and [`tcp`]'s debug
//! server. Publishers need no slot (`bus::publish` goes through
//! `immediate_publisher`), so republishing the application processor's relayed
//! frames onto this bus does not consume one. **There is no third slot**, which is
//! what caps the TCP server at one concurrent client; see [`tcp`]'s module docs for
//! the second, independent reason.
//!
//! `variegated_debug::status` is *not* touched by the USB transport. It is a
//! `Signal`, which holds exactly one waker, and a second concurrent `wait()` makes
//! two consumers ping-pong wakeups and busy-loop rather than merely miss values.
//! the TCP server's serve loop is the single consumer on this processor; the producer is the
//! `Status` arm of `crate::application_processor`'s reader, which offers the copy
//! this processor receives over the inter-processor link. Nothing else may `wait()`
//! on it.

pub mod commands;
pub mod panic_console;
pub mod snapshot;
pub mod tcp;
pub mod uart;

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

/// Where decoded commands are handed off, shared by the USB reader and -- when
/// `config::TCP_COMMANDS_ENABLED` -- the TCP reader. Drained by the
/// application-processor sender, which runs them through [`commands::dispatch`].
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
/// Lives here rather than in [`tcp`] so Task 10's snapshot task can read it whether
/// or not the TCP server is compiled in -- and here rather than on `bus`, which is a
/// re-export of the shared crate and cannot gain members that only one of the two
/// processors has.
///
/// An `AtomicU8`, but [`tcp`] only ever stores `0` or `1`: two hard limits cap it at
/// one concurrent client (one free bus subscriber slot, and `status`'s
/// single-consumer `Signal`), and the wider type is what lets that change without a
/// type change. It is also read by the `Status` arm of
/// `crate::application_processor`, which only pays for a `Status` clone into the
/// debug path while somebody is there to receive it.
pub static TCP_DEBUG_CLIENTS: portable_atomic::AtomicU8 = portable_atomic::AtomicU8::new(0);
