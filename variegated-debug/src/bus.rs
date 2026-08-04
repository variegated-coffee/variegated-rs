//! The debug bus, shared by whichever processor this build targets (see
//! `SOURCE`).
//!
//! `publish_immediate` is deliberate: an absent or lagging consumer loses old
//! frames instead of backpressuring a control task. Nothing on this path may ever
//! block on a host being attached.
//!
//! One payload does **not** travel here: `DebugPayload::Status` has its own
//! single-slot channel in [`crate::status`], because a multi-subscriber pubsub
//! clones every message it hands out and this one is ~1.7 kB. See that module.

use core::fmt::Write as _;

use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, RawMutex};
use embassy_sync::channel::Sender;
use embassy_sync::pubsub::{PubSubChannel, Subscriber};
use portable_atomic::{AtomicBool, AtomicU32, Ordering};
use variegated_controller_types::debug::{DebugEvent, DebugFrame, DebugPayload, DebugSource, DebugText, Name, Severity};
use variegated_controller_types::debug_command::DebugCommand;

/// Frames buffered per subscriber. `DebugFrame` is ~150 bytes, so this is ~2.5 kB
/// of static RAM.
pub const BUS_CAPACITY: usize = 16;
/// USB CDC writer + inter-processor relay.
///
/// **Above one, every message on this bus is `clone()`d under the bus's critical
/// section** -- `embassy-sync` only moves a message out for the last subscriber
/// taking it at index 0. So no payload that travels here may own a heap allocation:
/// a clone would become an `LlffHeap::alloc` with interrupts disabled on both cores,
/// at the full frame rate. Every `DebugPayload` variant except `Status` is inline
/// (`heapless` strings and vectors), and `Status` is why [`crate::status`] exists.
/// Check this before adding a variant that owns anything.
pub const BUS_SUBSCRIBERS: usize = 2;

pub type DebugBus = PubSubChannel<CriticalSectionRawMutex, DebugFrame, BUS_CAPACITY, BUS_SUBSCRIBERS, 1>;

pub static BUS: DebugBus = PubSubChannel::new();

static SEQ: AtomicU32 = AtomicU32::new(0);
static EMITTED: AtomicU32 = AtomicU32::new(0);
static DROPPED: AtomicU32 = AtomicU32::new(0);
static SUPPRESSED: AtomicU32 = AtomicU32::new(0);
static RATE_LIMITED: AtomicU32 = AtomicU32::new(0);
static COMMANDS_DROPPED: AtomicU32 = AtomicU32::new(0);
/// Whether the last command offered to the executor was refused. Drives the
/// edge-triggering in [`note_command_dropped`]; see there for why it is one latch for
/// the whole firmware rather than one per transport.
static DROPPING_COMMANDS: AtomicBool = AtomicBool::new(false);

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Stats {
    pub emitted: u32,
    /// Frames lost in transit -- see [`note_dropped`].
    pub dropped: u32,
    /// Duplicate frames collapsed before publication -- see [`note_suppressed`].
    /// Costs nothing: an identical frame is already on the bus.
    pub suppressed: u32,
    /// Frames refused by the text rate cap -- see [`note_rate_limited`]. Unlike
    /// `suppressed`, these are **lost**: they were not duplicates of anything.
    pub rate_limited: u32,
    /// Injected commands the device could not queue -- see [`note_command_dropped`].
    ///
    /// The only one of these five that counts something *inbound*. Kept apart from
    /// `dropped` for that reason: an operator reading "the device is throwing frames
    /// away" and an operator reading "the device is ignoring what I type" are chasing
    /// different faults.
    pub commands_dropped: u32,
}

pub fn stats() -> Stats {
    Stats {
        emitted: EMITTED.load(Ordering::Relaxed),
        dropped: DROPPED.load(Ordering::Relaxed),
        suppressed: SUPPRESSED.load(Ordering::Relaxed),
        rate_limited: RATE_LIMITED.load(Ordering::Relaxed),
        commands_dropped: COMMANDS_DROPPED.load(Ordering::Relaxed),
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

/// Record that a **duplicate** frame was collapsed before it reached the bus.
///
/// Distinct from [`note_dropped`]: nothing is wrong and nothing is lost -- an
/// identical frame was published moments earlier, and the repeating condition is
/// re-announced on a heartbeat. A bench user watching `dropped` climb at 10 Hz
/// would reasonably conclude the transport was failing, which is why
/// de-duplication gets its own counter.
///
/// Only for exact repeats. A frame the sink refused for any other reason is
/// [`note_rate_limited`], because that one really does destroy information.
pub fn note_suppressed() {
    SUPPRESSED.fetch_add(1, Ordering::Relaxed);
}

/// Record that a frame was refused by the text rate cap.
///
/// This is a **loss**, not a saving: the frame was not a duplicate, so nothing
/// else on the bus carries what it would have said. It is counted apart from both
/// `dropped` (which means the transport failed) and `suppressed` (which costs
/// nothing) so that neither number can be read as covering this case.
///
/// Should be rare -- the bucket carries enough burst capacity for the boot log,
/// and only sustained diversity beyond the tracked set drains it. A non-zero value
/// here is worth investigating.
pub fn note_rate_limited() {
    RATE_LIMITED.fetch_add(1, Ordering::Relaxed);
}

/// The text of a dropped-command report, carrying the running total.
///
/// The total travels *in the reason* because the event is edge-triggered and the
/// counter is not on the wire: without it a second flood, an hour later, would say
/// exactly what the first one said. `Name` is 32 bytes and the widest this can render
/// is 31 (`"cmd queue full, lost "` plus ten digits), so it cannot truncate --
/// asserted by `a_dropped_command_report_fits_its_name`.
fn command_dropped_reason(total: u32) -> Name {
    let mut reason = Name::new();
    let _ = write!(reason, "cmd queue full, lost {total}");
    reason
}

/// Record a command that could not be handed to whoever executes it, and say whether
/// this one should be reported.
///
/// **Always counts; reports only on an edge.** The tension is real and was recorded
/// during Task 12: an event per drop lets an unauthenticated TCP peer turn a command
/// flood into a frame flood on a 16-slot bus, evicting the very frames an operator
/// needs to see what is happening. A counter cannot be flooded and an edge-triggered
/// event cannot be repeated, so this does both and neither can be used against the
/// device.
///
/// Returns `Some(reason)` on the transition from delivering to dropping, `None` while
/// a flood is already being reported. Split out from [`offer_command`] rather than
/// folded into it so the accounting is testable on a host, where `emit_event` has no
/// `embassy_time` driver to stamp a frame with -- the same reason [`publish_with`]
/// exists.
///
/// One latch for the firmware, not one per transport: a firmware has at most two
/// command sources and they share a single `DEBUG_COMMAND_CHANNEL`, so the condition
/// being reported ("the queue is full") is a property of that queue rather than of the
/// wire the command came in on. Per-transport latches would report the same fact twice.
pub fn note_command_dropped() -> Option<Name> {
    let total = COMMANDS_DROPPED.fetch_add(1, Ordering::Relaxed).saturating_add(1);
    if DROPPING_COMMANDS.swap(true, Ordering::Relaxed) {
        None
    } else {
        Some(command_dropped_reason(total))
    }
}

/// Re-arm [`note_command_dropped`]'s edge: a command got through, so the next refusal
/// is a new episode and worth a line of its own.
pub fn note_command_delivered() {
    DROPPING_COMMANDS.store(false, Ordering::Relaxed);
}

/// Hand a decoded command to whoever executes it, without ever waiting for them.
///
/// Every debug transport ends in this call, and all four of them used to spell it
/// `let _ = sink.try_send(command)`. `try_send` is right -- a reader that blocks on the
/// command queue is a debug path that can stall, which this feature is not allowed to
/// be -- but discarding the `Err` meant a command lost to a full queue produced no
/// counter, no event and no trace of any kind. `DebugEvent::CommandReceived` is emitted
/// *after* this point, so from the host's seat a dropped command was indistinguishable
/// from a dead link: an operator standing at a machine that is ignoring them, which is
/// the failure `debug::commands`' own module docs argue hardest against.
pub fn offer_command<M: RawMutex, const N: usize>(
    sink: &Sender<'_, M, DebugCommand, N>,
    command: DebugCommand,
) {
    if sink.try_send(command).is_ok() {
        note_command_delivered();
    } else if let Some(reason) = note_command_dropped() {
        emit_event(DebugEvent::CommandRejected { reason });
    }
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

/// Stamp a payload with this build's source, the next per-source sequence number and
/// `uptime_ms`, and count it as emitted -- without putting it anywhere.
///
/// Exists because not every frame travels on [`BUS`]. [`crate::status`] carries the
/// machine's `Status` on its own single-slot channel, and the host's gap detection
/// depends on both paths drawing from **one** sequence counter: a side channel with
/// its own numbering, or none at all, would make every `Status` look to a host like
/// a frame that went missing.
pub fn stamp(uptime_ms: u64, payload: DebugPayload) -> DebugFrame {
    let frame = DebugFrame {
        source: SOURCE,
        seq: SEQ.fetch_add(1, Ordering::Relaxed),
        uptime_ms,
        payload,
    };
    EMITTED.fetch_add(1, Ordering::Relaxed);
    frame
}

/// Publish with an explicit timestamp. This is the primitive so the accounting is
/// testable on a host, where `embassy_time` has no driver installed.
pub fn publish_with(uptime_ms: u64, payload: DebugPayload) {
    // `immediate_publisher` needs no publisher slot and never awaits.
    BUS.immediate_publisher().publish_immediate(stamp(uptime_ms, payload));
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

/// Serialises every test that **reads or writes** one of the global counters.
///
/// `SEQ`, `EMITTED` and `DROPPED` are process-wide and cargo runs tests in parallel
/// threads. Locking only the readers is not enough and was not enough: any test that
/// calls `publish_with` bumps `SEQ` and `EMITTED`, so an unlocked writer running
/// beside a locked reader still breaks it. That is not hypothetical -- it is how
/// `sequence_numbers_increase_per_frame` started failing once [`crate::status`]
/// added a second publisher to the suite.
///
/// The rule is therefore: **if a test touches a counter in either direction, take
/// this lock.** That covers `publish_with`, `stamp`, `note_*`, `stats`, and anything
/// in [`crate::status`], whose publish path goes through all three.
#[cfg(test)]
pub(crate) static COUNTER_DELTA_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

#[cfg(test)]
mod tests {
    use super::*;
    use variegated_controller_types::debug::{DebugEvent, DebugPayload};

    #[test]
    fn frames_are_stamped_with_the_compiled_in_source() {
        let _serialised = COUNTER_DELTA_LOCK.lock();
        let mut sub = BUS.subscriber().unwrap();
        publish_with(0, DebugPayload::Event(DebugEvent::Boot));
        let frame = sub.try_next_message_pure().unwrap();
        assert_eq!(frame.source, SOURCE);
    }

    #[test]
    fn sequence_numbers_increase_per_frame() {
        let _serialised = COUNTER_DELTA_LOCK.lock();
        let mut sub = BUS.subscriber().unwrap();
        publish_with(10, DebugPayload::Event(DebugEvent::Boot));
        publish_with(20, DebugPayload::Event(DebugEvent::Boot));

        let first = sub.try_next_message_pure().unwrap();
        let second = sub.try_next_message_pure().unwrap();
        assert_eq!(second.seq, first.seq + 1);
        assert_eq!(first.uptime_ms, 10);
        assert_eq!(second.uptime_ms, 20);
    }

    /// A frame published while nobody is subscribed does not exist. Not evicted
    /// early, not lagged -- never queued.
    ///
    /// `try_publish` returns `Ok(())` and touches nothing when
    /// `subscriber_count == 0` (`embassy-sync-0.8.0/src/pubsub/mod.rs:332-336`), and
    /// `PubSubChannel::subscriber` starts a new reader at the *current*
    /// `next_message_id` (`:100`), so a subscriber that arrives afterwards cannot
    /// recover it. `BUS_CAPACITY` has nothing to do with the outcome: the ring has
    /// 16 slots and one frame was published, and it is still gone.
    ///
    /// This is why the comms firmware claims its subscriber slot in `main`,
    /// synchronously, before the first publish rather than inside the task that
    /// reads: everything published before that call -- `DebugEvent::Boot`, the early
    /// `SpawnFailed` reports, the boot log -- was being discarded.
    #[test]
    fn a_frame_published_with_no_subscriber_is_unrecoverable() {
        let _serialised = COUNTER_DELTA_LOCK.lock();

        publish_with(1, DebugPayload::Event(DebugEvent::Boot));

        let mut late = BUS.subscriber().unwrap();
        assert!(
            late.try_next_message_pure().is_none(),
            "a subscriber created after the publish must not see the frame"
        );
    }

    /// The other half of the same fact, and the shape the fix relies on: claim the
    /// slot first, publish second, and the frame is delivered.
    #[test]
    fn a_frame_published_after_the_slot_is_claimed_is_delivered() {
        let _serialised = COUNTER_DELTA_LOCK.lock();

        let mut early = BUS.subscriber().unwrap();
        publish_with(2, DebugPayload::Event(DebugEvent::Boot));

        let frame = early
            .try_next_message_pure()
            .expect("a subscriber claimed before the publish must receive the frame");
        assert_eq!(frame.payload, DebugPayload::Event(DebugEvent::Boot));
        assert_eq!(frame.uptime_ms, 2);
    }

    /// Pins the accounting gap that makes the above invisible to the counters, so
    /// that changing it is a deliberate act rather than an accident.
    ///
    /// [`stamp`] increments `EMITTED` for a frame that `try_publish` then throws
    /// away, and nothing calls [`note_dropped`] on that path. So `frames_emitted` in
    /// a snapshot counts frames that were never deliverable, `frames_dropped` stays
    /// at zero, and the difference between them cannot reveal a subscriberless
    /// publish. Whether that is worth changing is Task 9's accounting to decide;
    /// this test only makes sure nobody changes it without noticing.
    #[test]
    fn a_discarded_frame_is_still_counted_as_emitted() {
        let _serialised = COUNTER_DELTA_LOCK.lock();

        let before = stats();
        publish_with(3, DebugPayload::Event(DebugEvent::Boot));
        let after = stats();

        assert_eq!(after.emitted, before.emitted + 1);
        assert_eq!(after.dropped, before.dropped);
    }

    #[test]
    fn dropped_frames_are_counted() {
        let _serialised = COUNTER_DELTA_LOCK.lock();
        let before = stats().dropped;
        note_dropped();
        note_dropped();
        assert_eq!(stats().dropped, before + 2);
    }

    /// The failure this accounting exists for: a command the device could not queue
    /// used to reach nothing at all -- no counter, no event -- while every other loss
    /// in this feature reached `frames_dropped`, `frames_suppressed` or
    /// `frames_rate_limited`.
    ///
    /// One test rather than three, for the reason `crate::status`'s is: the counter and
    /// the latch are process-wide globals and cargo runs tests in parallel threads, so
    /// separate functions would race each other for them.
    #[test]
    fn dropped_commands_are_always_counted_and_reported_on_an_edge() {
        let _serialised = COUNTER_DELTA_LOCK.lock();
        // The latch is global and another test may have left it set.
        note_command_delivered();

        let before = stats().commands_dropped;

        // The first refusal of an episode is the one that gets reported, and the
        // report carries the running total -- the counter is not on the wire, so
        // without it a second flood would say exactly what the first one said.
        let first = note_command_dropped().expect("the first drop of an episode reports");
        assert_eq!(first.as_str(), std::format!("cmd queue full, lost {}", before + 1));

        // Everything after it counts and stays quiet. This is what stops an
        // unauthenticated TCP peer turning a command flood into a frame flood that
        // evicts the 16-slot bus.
        for _ in 0..50 {
            assert!(note_command_dropped().is_none(), "a flood must report once");
        }
        assert_eq!(stats().commands_dropped, before + 51, "but every one is counted");

        // A command that gets through ends the episode, so the next refusal is news
        // again -- and says how many have been lost in total, not since the re-arm.
        note_command_delivered();
        let second = note_command_dropped().expect("a new episode reports again");
        assert_eq!(second.as_str(), std::format!("cmd queue full, lost {}", before + 52));

        note_command_delivered();
    }

    /// `Name` is 32 bytes and `write!` into a `heapless::String` truncates nothing --
    /// it fails and leaves whatever fitted. A reason that overflowed would therefore
    /// reach a host as a sentence cut off mid-number, which is worse than no number.
    #[test]
    fn a_dropped_command_report_fits_its_name() {
        let widest = command_dropped_reason(u32::MAX);
        assert_eq!(widest.as_str(), "cmd queue full, lost 4294967295");
        assert!(widest.len() <= variegated_controller_types::debug::NAME_LEN);
    }
}
