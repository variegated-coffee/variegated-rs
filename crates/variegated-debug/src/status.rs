//! The machine's published `Status`, carried on its own single-slot channel rather
//! than on the shared debug bus.
//!
//! # Why it is not on the bus
//!
//! `embassy-sync` 0.8's `PubSubState::get_message` moves a message *out* of the ring
//! only when it sits at index 0 **and** the last subscriber is the one taking it.
//! Every other case `clone()`s it, inside the same `inner.lock(..)` critical section
//! that the publish path takes. With a single subscriber that fast path always
//! applies and nothing clones, which is why this cost did not exist before.
//!
//! The inter-processor relay is the second subscriber. From the moment it exists,
//! every `DebugPayload::Status` frame handed to the first of the two subscribers is
//! a `Box<Status>` clone: a ~1.7 kB `LlffHeap::alloc` -- first-fit, so O(n) in the
//! free list -- plus a memcpy, with interrupts disabled on **both** cores, at 1 Hz,
//! forever, on the processor running the PID loops and the PIO pulse counter. It
//! needs no stall to trigger; it is the steady state.
//!
//! Filtering `Status` inside the relay does not avoid any of that. The clone happens
//! in the pubsub before the relay's code ever sees the frame.
//!
//! # Why latest-wins is right anyway
//!
//! `Status` is a *level*, not an event. A copy from three seconds ago tells a host
//! nothing it wants to know and the next one is a second away, so replacing an
//! undelivered one is the correct behaviour rather than a compromise -- unlike the
//! event stream on the bus, where every frame is a distinct occurrence and order and
//! completeness are the point.
//!
//! # Consumers
//!
//! Exactly one per firmware: the local transport (USB CDC on the application
//! processor, and Task 11's TCP server on the comms processor).
//!
//! **A second concurrent [`wait`] is worse than a lost wakeup -- it is a busy-loop.**
//! `Signal` holds a single waker, and `poll_wait` on a waker that does not match the
//! stored one *replaces* it and wakes the one it displaced (embassy-sync 0.8,
//! `signal.rs:101-105`). So two consumers do not quietly stop receiving; they wake
//! each other forever, spinning the executor on a processor that is running PID
//! loops and a PIO pulse counter. If a second consumer is ever needed, change the
//! primitive -- do not add another `wait`.
//!
//! The relay is deliberately **not** a consumer. `Status` does not cross the
//! inter-processor link at all; see `variegated_comms::debug_relay` and
//! [`crate::relay::relayable`].

use alloc::boxed::Box;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use portable_atomic::{AtomicBool, Ordering};
use variegated_controller_types::debug::{DebugFrame, DebugPayload};
use variegated_controller_types::Status;

use crate::bus;

static STATUS: Signal<CriticalSectionRawMutex, DebugFrame> = Signal::new();

/// Whether the local transport currently has a host on the other end.
///
/// Producers of `Status` consult this **before building one**, and that ordering is
/// the entire reason it exists. The transport's own attachment check
/// ([`crate::usb_cdc`]'s `dtr()`) is downstream of the caller's `Status::clone()` and
/// `Box::new`, so on a machine no host has ever attached to, the 1 Hz publisher was
/// paying a ~1.7 kB clone plus an `LlffHeap::alloc` -- first-fit, O(n) in the free
/// list, under `critical_section::with`, i.e. with interrupts disabled on both cores
/// -- and a matching `dealloc` a second later, forever, on the processor running the
/// PID loops. That is the cost the "`Status` off the bus" redesign was built to
/// eliminate; moving it off the pubsub removed the memcpy under the lock and left the
/// allocation.
///
/// The comms firmware already applies exactly this discipline to the identical line,
/// gating on `TCP_DEBUG_CLIENTS > 0` with the written justification *"do not produce
/// for a host that is not there"*. This is the application processor's version of that
/// gate. The comms firmware does not use it -- its own client count is the better
/// signal there, and it distinguishes which transport is attached.
static TRANSPORT_ATTACHED: AtomicBool = AtomicBool::new(false);

/// Report whether a host is attached. Called by the transport, from the same place it
/// decides whether to write.
pub fn note_transport_attached(attached: bool) {
    TRANSPORT_ATTACHED.store(attached, Ordering::Relaxed);
}

/// Whether it is worth building a `Status` at all. See [`TRANSPORT_ATTACHED`].
///
/// Deliberately a level rather than a handshake, and deliberately allowed to be stale:
/// the transport refreshes it on every frame it considers, and the bus carries frames
/// continuously whether or not any `Status` is produced, so a host that attaches is
/// seen within one frame and starts receiving `Status` on the next 1 Hz tick. Nothing
/// here waits for a host, which is the invariant the whole debug path is built on.
pub fn transport_attached() -> bool {
    TRANSPORT_ATTACHED.load(Ordering::Relaxed)
}

/// Offer the latest `Status` to the local transport, replacing any copy it has not
/// yet taken.
///
/// Never blocks and never back-pressures, exactly like `bus::publish` -- the
/// load-bearing invariant is unchanged.
///
/// The superseded frame is taken out and dropped *outside* `Signal::signal`'s
/// critical section on purpose: `Signal::signal` replaces the cell's contents while
/// holding the mutex, so dropping a `Box<Status>` there would run
/// `LlffHeap::dealloc` -- whose free-list insert is O(n) and takes a critical
/// section of its own -- with interrupts disabled. Taking first costs one extra
/// mutex acquisition and moves the free out of the critical path.
pub fn publish_with(uptime_ms: u64, status: Box<Status>) {
    let frame = bus::stamp(uptime_ms, DebugPayload::Status(status));

    // A frame still sitting here was never delivered: the transport is stalled or no
    // host is attached. That is a genuine loss, so it is counted as one.
    //
    // Benignly racy against a consumer taking between the two calls, which would
    // over-count by one. It cannot under-count, and the alternative -- not counting
    // at all -- would let a silently disappearing `Status` read as a healthy link.
    if let Some(superseded) = STATUS.try_take() {
        bus::note_dropped();
        drop(superseded);
    }

    STATUS.signal(frame);
}

/// [`publish_with`] against the running clock.
pub fn publish(status: Box<Status>) {
    publish_with(embassy_time::Instant::now().as_millis(), status);
}

/// Wait for the next `Status` frame. Cancel-safe: a value not polled to completion
/// stays in the slot, which is what lets a transport `select` this against the bus.
pub async fn wait() -> DebugFrame {
    STATUS.wait().await
}

/// Take the pending `Status` frame if there is one, without waiting.
pub fn try_take() -> Option<DebugFrame> {
    STATUS.try_take()
}

#[cfg(test)]
mod tests {
    use super::*;
    use variegated_controller_types::Status;

    fn status() -> Box<Status> {
        Box::new(Status::default())
    }

    /// One test rather than four, deliberately: `STATUS` is a single global slot and
    /// cargo runs tests in parallel threads, so separate test functions would race
    /// each other for it and fail intermittently for reasons that have nothing to do
    /// with the code under test.
    #[test]
    fn the_status_slot_stamps_supersedes_and_counts() {
        let _serialised = bus::COUNTER_DELTA_LOCK.lock();
        STATUS.reset();

        // Stamped like any other frame: this source, the shared sequence counter,
        // the caller's uptime.
        let emitted_before = bus::stats().emitted;
        publish_with(1234, status());
        assert_eq!(bus::stats().emitted, emitted_before + 1);

        let frame = try_take().expect("a status was published");
        assert_eq!(frame.uptime_ms, 1234);
        assert_eq!(frame.source, bus::SOURCE);
        assert!(matches!(frame.payload, DebugPayload::Status(_)));
        assert!(try_take().is_none(), "the slot must be empty after a take");

        // Two sequential publishes must not reuse a sequence number. A repeat, or a
        // gap, reads to a host as a lost frame -- which is why this path stamps
        // through `bus::stamp` instead of numbering itself.
        publish_with(1, status());
        let first = try_take().expect("a status is pending");
        publish_with(2, status());
        let second = try_take().expect("a status is pending");
        assert_eq!(second.seq, first.seq + 1);

        // Latest-wins, and the superseded frame is counted as the loss it is.
        publish_with(10, status());
        let dropped_before = bus::stats().dropped;
        publish_with(20, status());
        assert_eq!(bus::stats().dropped, dropped_before + 1);
        let survivor = try_take().expect("a status is pending");
        assert_eq!(survivor.uptime_ms, 20, "the newer status must win");
    }
}
