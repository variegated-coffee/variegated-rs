//! The debug bus, shared by whichever processor this build targets (see
//! `SOURCE`).
//!
//! `publish_immediate` is deliberate: an absent or lagging consumer loses old
//! frames instead of backpressuring a control task. Nothing on this path may ever
//! block on a host being attached.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::{PubSubChannel, Subscriber};
use portable_atomic::{AtomicU32, Ordering};
use variegated_controller_types::debug::{DebugEvent, DebugFrame, DebugPayload, DebugSource, DebugText, Severity};

/// Frames buffered per subscriber. `DebugFrame` is ~150 bytes, so this is ~2.5 kB
/// of static RAM.
pub const BUS_CAPACITY: usize = 16;
/// USB CDC writer + inter-processor relay.
pub const BUS_SUBSCRIBERS: usize = 2;

pub type DebugBus = PubSubChannel<CriticalSectionRawMutex, DebugFrame, BUS_CAPACITY, BUS_SUBSCRIBERS, 1>;

pub static BUS: DebugBus = PubSubChannel::new();

static SEQ: AtomicU32 = AtomicU32::new(0);
static EMITTED: AtomicU32 = AtomicU32::new(0);
static DROPPED: AtomicU32 = AtomicU32::new(0);
static SUPPRESSED: AtomicU32 = AtomicU32::new(0);

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Stats {
    pub emitted: u32,
    /// Frames lost against the operator's intent -- see [`note_dropped`].
    pub dropped: u32,
    /// Frames deliberately thinned before publication -- see [`note_suppressed`].
    /// Kept separate from `dropped` because the two mean opposite things about the
    /// health of the link.
    pub suppressed: u32,
}

pub fn stats() -> Stats {
    Stats {
        emitted: EMITTED.load(Ordering::Relaxed),
        dropped: DROPPED.load(Ordering::Relaxed),
        suppressed: SUPPRESSED.load(Ordering::Relaxed),
    }
}

/// Record that a frame was **lost**: a transport threw it away because no host was
/// attached or a buffer was full, or the bus evicted it unread.
///
/// This number rising means data the device wanted to send did not arrive. Do not
/// use it for frames the device chose not to send -- that is [`note_suppressed`].
/// Conflating them makes a working link read as a failing one.
pub fn note_dropped() {
    DROPPED.fetch_add(1, Ordering::Relaxed);
}

/// Record that a frame was **deliberately thinned** before it reached the bus,
/// because it repeated a recent message or hit the text rate cap.
///
/// Distinct from [`note_dropped`]: nothing is wrong, and the information is not
/// lost -- an identical frame was published moments earlier. A bench user watching
/// `dropped` climb at 10 Hz would reasonably conclude the transport was failing,
/// which is why de-duplication gets its own counter.
pub fn note_suppressed() {
    SUPPRESSED.fetch_add(1, Ordering::Relaxed);
}

pub fn subscriber() -> Option<Subscriber<'static, CriticalSectionRawMutex, DebugFrame, BUS_CAPACITY, BUS_SUBSCRIBERS, 1>> {
    BUS.subscriber().ok()
}

// Which processor this build stamps its frames with. Selected at compile time so the
// same crate serves both firmwares with no runtime init step and no wrong-default
// risk -- a mislabelled source would silently corrupt the host's per-source sequence
// accounting.
#[cfg(all(feature = "source-application", feature = "source-comms"))]
compile_error!("enable exactly one of `source-application` / `source-comms`, not both");
#[cfg(not(any(feature = "source-application", feature = "source-comms")))]
compile_error!("enable exactly one of `source-application` / `source-comms`");

#[cfg(all(feature = "source-application", not(feature = "source-comms")))]
pub const SOURCE: DebugSource = DebugSource::Application;
#[cfg(all(feature = "source-comms", not(feature = "source-application")))]
pub const SOURCE: DebugSource = DebugSource::Comms;

/// Publish with an explicit timestamp. This is the primitive so the accounting is
/// testable on a host, where `embassy_time` has no driver installed.
pub fn publish_with(uptime_ms: u64, payload: DebugPayload) {
    let frame = DebugFrame {
        source: SOURCE,
        seq: SEQ.fetch_add(1, Ordering::Relaxed),
        uptime_ms,
        payload,
    };
    // `immediate_publisher` needs no publisher slot and never awaits.
    BUS.immediate_publisher().publish_immediate(frame);
    EMITTED.fetch_add(1, Ordering::Relaxed);
}

pub fn publish(payload: DebugPayload) {
    publish_with(embassy_time::Instant::now().as_millis(), payload);
}

pub fn emit_event(event: DebugEvent) {
    publish(DebugPayload::Event(event));
}

pub fn emit_text(severity: Severity, message: DebugText) {
    publish(DebugPayload::Text(severity, message));
}

#[cfg(test)]
mod tests {
    use super::*;
    use variegated_controller_types::debug::{DebugEvent, DebugPayload};

    #[test]
    fn frames_are_stamped_with_the_compiled_in_source() {
        let mut sub = BUS.subscriber().unwrap();
        publish_with(0, DebugPayload::Event(DebugEvent::Boot));
        let frame = sub.try_next_message_pure().unwrap();
        assert_eq!(frame.source, SOURCE);
    }

    #[test]
    fn sequence_numbers_increase_per_frame() {
        let mut sub = BUS.subscriber().unwrap();
        publish_with(10, DebugPayload::Event(DebugEvent::Boot));
        publish_with(20, DebugPayload::Event(DebugEvent::Boot));

        let first = sub.try_next_message_pure().unwrap();
        let second = sub.try_next_message_pure().unwrap();
        assert_eq!(second.seq, first.seq + 1);
        assert_eq!(first.uptime_ms, 10);
        assert_eq!(second.uptime_ms, 20);
    }

    #[test]
    fn dropped_frames_are_counted() {
        let before = stats().dropped;
        note_dropped();
        note_dropped();
        assert_eq!(stats().dropped, before + 2);
    }
}
