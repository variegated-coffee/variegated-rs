#![no_std]
#![warn(missing_docs)]

//! Per-task check-in table for liveness monitoring.
//!
//! A fed hardware watchdog proves one thing: the executor is still scheduling. It says
//! nothing about a single task deadlocked on an `await` while the others keep running, and
//! on the RP2350 it says less than that -- the watchdog is fed from inside the controller's
//! own loop, so every other task on the board is outside its reach. This crate is the other
//! half: each monitored task or future records `(uptime, status)` into a static slot, and
//! `variegated_debug::checkin` ships the whole table to a host once a second.
//!
//! # Observe only
//!
//! Nothing here reboots anything, and nothing here reads the per-slot period. The period is
//! declared, shipped on the wire, and compared *by the host*. That split is deliberate: a
//! check-in deadline set wrong turns a working board into a reboot loop, and the way not to
//! ship that is to have field data on real periods before anything acts on them. The hook
//! for enforcement is already in place -- it would read the same `PERIODS` table
//! `define_checkins!` already generates, with no change to the wire.
//!
//! # One writer per slot, enforced
//!
//! A slot has exactly one writer. Two writers do not corrupt the table -- the stores are
//! atomic -- they corrupt its *meaning*: [`watch`] stamping `Good` on every poll would erase
//! a `Warning` the inner code had just published, and the row would read healthy for as long
//! as it was being polled. That is a silently wrong row, which is worse than a missing one.
//!
//! `variegated-instrumentation` states the same rule for a counter and leaves it at that,
//! because a counter written twice is merely double-counted and the number still looks like
//! a number. Here the failure is a *lie*, so the rule is carried by the type system instead
//! of by this paragraph:
//!
//! * [`Monitor::claim`] hands out a given slot's handle **once**. A second claim panics at
//!   boot, deterministically, naming the mistake.
//! * [`CheckinHandle`] is **not `Copy`**. After the claim it lives in exactly one place, and
//!   moving it into a second one is a move out of the first.
//!
//! Together those make the interesting mistake -- wrapping a future in [`watch`] *and*
//! handing the code inside it a handle to the same slot -- either a compile error (one
//! handle, moved twice) or a boot panic (two claims). Neither is a comment nobody reads.
//!
//! [`CheckinHandle::none`] is the deliberate exception: unclaimed, freely constructible, and
//! pointing at a slot nothing reads. Many writers, no meaning to corrupt.
//!
//! So the two mechanisms here are alternatives for a given slot, not layers:
//!
//! * [`watch`] wraps a future you do not own and reports **poll-liveness** -- it catches an
//!   arm of a `join` that stopped being woken, and it catches a future that returned when it
//!   should have run forever. It cannot say why, and it cannot tell healthy-idle from wedged.
//! * A [`CheckinHandle`] held by the code itself reports **what the loop body actually did**,
//!   which is strictly more than poll-liveness: it proves the body ran, not merely that the
//!   future was polled, and it can name the trouble.
//!
//! Moving a slot from the first to the second is an upgrade, and it means removing the
//! [`watch`] wrapper, not nesting one inside the other.
//!
//! # No feature gate
//!
//! Unlike `variegated-instrumentation`, there is no cargo feature compiling this out. The
//! cost is two relaxed 32-bit stores per loop iteration at a few hundred hertz at most --
//! not the per-packet profile that gating was for -- and a build with monitoring compiled
//! out is a build where the one question you need in the field cannot be asked. All three
//! firmwares enable `instrumentation` anyway, so "zero-cost when off" is a configuration
//! nobody builds. The noop split is also what forced two const parameters onto
//! `Ads124S08Sensor` that exist only to name handle types; [`CheckinHandle`] is a bare
//! pointer for that reason.
//!
//! # Example
//!
//! ```ignore
//! variegated_checkin::define_checkins! {
//!     pub enum CheckinId {
//!         /// The 100 ms control loop.
//!         Controller = 0 => 100,
//!         /// Drains a command channel; no period, it is event-driven.
//!         Storage = 1 => _,
//!     }
//! }
//!
//! static MONITOR: Monitor<{ CheckinId::COUNT }> = Monitor::new();
//!
//! // in the code being watched
//! let checkin = MONITOR.claim(CheckinId::Controller);
//! loop {
//!     checkin.good();
//!     // ... one cycle ...
//! }
//!
//! // or, around a future whose body you do not own
//! watch(MONITOR.claim(CheckinId::Storage), storage.task()).await;
//! ```

// Test helpers only: `assert_eq!`'s formatting and `std::vec::Vec` in a `no_std` crate.
#[cfg(test)]
extern crate std;

/// How often a loop that would otherwise only wake for work should wake anyway.
///
/// # A slot without a period is an exception, not the default
///
/// `_` in [`define_checkins!`] tells the host never to age that row -- so a task declared
/// that way can wedge forever and the table will not say so. That is the failure this whole
/// crate exists to catch, and declaring `_` opts out of catching it.
///
/// Almost no loop actually needs to. A task parked on `receive().await` looks identical to a
/// task parked on a lock that will never be released, and the fix is the same everywhere:
/// give the wait a timeout, so the loop turns over on a known cadence whether or not work
/// arrived, and declare that cadence. `embassy_time::with_timeout` for a single wait, or one
/// more `Timer::after` arm on an existing `select`. The heartbeat costs one timer per task.
///
/// What that buys is the thing a period-less row cannot have: the heartbeat stops when the
/// task is stuck *anywhere else in its body*, because the timeout is inside the loop rather
/// than outside it. A timer bolted onto the *wrapper* would keep firing on a wedged task and
/// prove nothing -- which is why [`watch`] does not have one.
///
/// # The timer must be inside the loop, and there is no weaker acceptable form
///
/// It is tempting, for a task parked on something that cannot be cancelled, to run a timer
/// *beside* the wait and check in from that -- the row gets a period and stops looking
/// neglected. Do not. That timer fires wherever the task is parked, including parked forever
/// inside a driver awaiting a notification that will never arrive, so the row renders green
/// through exactly the deadlock it was added to detect.
///
/// The ranking is not stale-beats-nothing, it is:
///
/// 1. a red or stale row -- says something true;
/// 2. a row with no period -- says nothing, and says so;
/// 3. **a fresh row that is wrong** -- the only one an operator will not investigate.
///
/// This crate had a `while_waiting` helper for the third case and it has been removed. If a
/// wait genuinely cannot be given a timeout, the answer is a check-in inside whatever it is
/// waiting on; failing that, leave the slot at `_` and say in its doc comment what would be
/// needed. `ble_slot_task` is the worked example.
///
/// Note how few waits actually resist a timeout. Every channel receive and signal wait here
/// is cancel-safe. Even `TcpSocket::accept` is: dropping the socket and re-listening happens
/// with no `await` in between, and on a cooperative executor the net task cannot run in that
/// gap, so no connection can be missed.
///
/// # Put the check-in in the loop the task actually lives in
///
/// The commonest mistake here, made three times while this was being written, is checking in
/// from an outer loop that only turns over on a state change. An accept loop runs once per
/// connection; a Wi-Fi loop runs once per link change; a BLE slot loop runs once per
/// reassignment. Check in there and the row ticks happily while the task is idle and goes
/// stale the moment it starts doing its job -- which is the exact inversion of what the table
/// is for, and it reads as a wedge on a subsystem that is working.
///
/// Find the innermost loop that turns over in the task's *dominant* state and put it there.
///
/// # Choosing the declared period
///
/// Declare roughly **three times** the loop's worst-case wake interval. The host flags a row
/// late past 1x and red at 2x, so declaring the interval exactly puts a healthy row on the
/// boundary and a 1 Hz sample will cross it. A loop on this constant declares 15 s.
///
/// **"Wake interval" means the whole body, not the wait.** A loop that parks on a channel for
/// at most 5 s and then spends 20 s erasing flash has a 25 s worst case, and declaring 15 s
/// makes every erase look like a stall. Where the body's slowest path is genuinely long,
/// either check in from inside it -- the two heating elements chunk their phase sleeps for
/// this reason -- or declare a period that covers it and accept the looser deadline. Do not
/// declare the wait and hope.
///
/// A task with two states whose cadences differ by orders of magnitude cannot be described by
/// one period at all. `Improv` and `ble_slot_task` are the examples; both declare `_` and say
/// why.
///
/// # What is left
///
/// One shape genuinely has no period: a future polled once and then never again by design.
/// Nothing in this tree is left in that state -- the RP2350's backlight holder was, and became
/// a heartbeat loop instead. Reach for `_` only after both forms above have been ruled out,
/// and say in the slot's doc comment which one was ruled out and why.
pub const HEARTBEAT: embassy_time::Duration = embassy_time::Duration::from_secs(5);

mod handle;
mod macros;
mod monitor;
mod pack;
pub(crate) mod watch;

pub use handle::{CheckinHandle, NULL_SLOT};
pub use monitor::{CheckinSlot, Monitor};
pub use watch::{watch, Watch};

pub use variegated_controller_types::debug::{
    CheckinDetail, CheckinEntry, CheckinStatus, MAX_CHECKINS,
};
