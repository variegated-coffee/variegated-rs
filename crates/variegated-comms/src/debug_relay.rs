//! Relays application-processor debug frames to the comms processor.
//!
//! Emission is always-on, so this shares the 576 kbaud link with Status and
//! Configuration permanently. A fixed-window byte budget guarantees debug traffic
//! can never starve them: frames that do not fit the window are dropped and
//! counted, which is the correct trade for debug data.
//!
//! # What crosses, and what does not
//!
//! `DebugPayload::Status` does not -- see [`variegated_debug::relay::relayable`] for
//! the two reasons. Everything else does, subject to the budget.
//!
//! # Version attestation, and why the mixed-source hazard does not arise here
//!
//! A frame crosses this link as a **typed value** inside a postcard-encoded
//! `ApplicationProcessorToCommsProcessorMessage`, not as a `variegated-debug-codec`
//! envelope. It therefore carries no `DEBUG_PROTOCOL_VERSION` byte, and the comms
//! processor re-encodes it with its *own* version when it serves it over TCP.
//!
//! That matters because the corroborating version watch in the codec keys its
//! mismatch run per link, and one link carrying two processors would break that: a
//! healthy processor's good frames would clear a stale processor's mismatch run and
//! no banner would ever appear. It does not happen on this path. Every frame on the
//! TCP link -- relayed or locally produced -- is stamped by one encoder with one
//! version, so a link still carries exactly one version and per-link keying stays
//! correct.
//!
//! **The hazard does not vanish, though: it moves, to a layer with less detection
//! than it had before. Do not read the paragraph above as "solved".**
//!
//! Three consequences, and they are the reason this section is long:
//!
//! 1. **A version-skewed application processor now fails silently.**
//!    `DEBUG_PROTOCOL_VERSION` lives in the same crate as `DebugFrame`, so a
//!    processor that disagrees about the version also disagrees about the frame's
//!    *shape*. The comms processor postcard-decodes `Debug(DebugFrame)` with its own
//!    types, gets a deserialize error, and drops it. From the host's seat:
//!    application frames simply stop arriving while comms frames keep coming, with no
//!    banner and no counter — the mixed-source scenario with the diagnostic removed
//!    rather than relocated. Worse, this side counts those frames as *relayed*,
//!    because they were handed to the TX queue successfully, so
//!    `link_frames_relayed` keeps climbing and `link_frames_dropped` stays at zero.
//!    Neither counter can be used to detect it.
//!
//! 2. **Re-stamping on the TCP path asserts health the encoder cannot vouch for.**
//!    The comms processor stamps its own version on frames it did not originate and
//!    cannot verify, so `VersionVerdict::Healthy` on a TCP link says nothing about
//!    the application processor. That is a real regression against the USB path,
//!    which carries the application processor's own attestation for the same frames.
//!    A relayed frame must never be presented to a host as version-verified.
//!
//! 3. **The detector belongs on the comms side**, because that is the only party that
//!    can tell a relayed frame from a local one. It cannot be built here.
//!    TODO: build it there; nothing detects a version-skewed application processor
//!    on the TCP path today.
//!
//! The neighbouring exposure is older than this relay and belongs to a different
//! protocol: an application processor and a comms processor built from different
//! commits disagree about the shape of `ApplicationProcessorToCommsProcessorMessage`
//! itself, and every message on the link -- Status, Configuration, this one --
//! mis-decodes. That is the inter-processor protocol's problem
//! (`variegated_controller_types::PROTOCOL_VERSION` and the `Hello` handshake, which
//! today nobody sends and nobody checks), not the debug protocol's. Its symptom on
//! *this* side is a `DebugEvent::LinkDecodeError`.
//!
//! **If this relay is ever changed to carry pre-encoded bytes** so that a frame keeps
//! its origin processor's version end to end, points 1 and 2 are fixed but the
//! mixed-source hazard becomes real, and the mismatch run must then be keyed per
//! relay segment -- the comms processor being the only party that can attribute a
//! frame to a link, since `source` lives inside the payload and is exactly the field
//! a mis-versioned frame cannot be trusted about.

use alloc::vec::Vec;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_sync::channel::Sender;
use embassy_sync::pubsub::WaitResult;
use postcard::to_allocvec_cobs;
use variegated_controller_types::debug::{name, DebugEvent};
use variegated_controller_types::ApplicationProcessorToCommsProcessorMessage;
use variegated_debug::bus;
use variegated_debug::rate::TokenBucket;
use variegated_debug::relay::relayable;

static FRAMES_RELAYED: AtomicU32 = AtomicU32::new(0);
static FRAMES_DROPPED: AtomicU32 = AtomicU32::new(0);

/// TX-queue slots the relay may never take, however much budget it has left.
///
/// `tx_channel` is **shared**: Status, Configuration, MachineDefinition and Routines
/// travel on it as well as debug frames, and `try_send` alone only stops the *relay*
/// from blocking. It does nothing to stop the relay from occupying all ten slots --
/// from a full token bucket it may emit `BURST_BYTES` of frames back to back -- and
/// the party that then blocks is the machine's own reader arm, which answers
/// `RequestConfiguration`, `RequestMachineDefinition` and `RequestRoutines` with a
/// `tx_sender.send(..).await` and forwards `MachineCommand`s while it is at it.
///
/// That was worse than latency on `single-boiler` while that board ran the link at 115200
/// with no RTS/CTS: `embassy_rp::uart::UartRx<Async>::read` arms DMA per call, so a parked
/// reader left only the 32-byte hardware FIFO -- about 2.8 ms before bytes were lost, COBS
/// desynchronised and the link reported a decode error.
///
/// Both boards now run 576 kbaud with hardware flow control, so a parked reader de-asserts
/// RTS and the far end stops rather than overrunning. The reservation stays: flow control
/// converts the failure from lost bytes into stalled telemetry, which is better but still
/// not something a debug feature may inflict on the machine's own traffic.
///
/// Two slots, because the reader needs one for the response it is sending and one for
/// the next one to have somewhere to go. Refusing here is a drop like any other and is
/// counted like one.
const TX_RESERVED_SLOTS: usize = 2;

/// What the relay has done with the frames it was offered.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct RelayStats {
    /// Frames handed to the link's TX queue.
    pub relayed: u32,
    /// Frames the relay threw away: refused by the byte budget, unencodable, or
    /// offered to a full TX queue. Filtered `Status` frames are **not** counted here
    /// -- those are a policy decision, not a loss, and there is nothing to lose since
    /// the comms processor receives `Status` by its own route.
    ///
    /// Every one of these is *also* counted in `bus::stats().dropped`; see
    /// [`note_dropped`].
    pub dropped: u32,
}

/// Read the relay's counters. Cheap and lock-free; called from the 1 Hz snapshot
/// task, which is what puts these numbers in `ApplicationState`.
pub fn relay_stats() -> RelayStats {
    RelayStats {
        relayed: FRAMES_RELAYED.load(Ordering::Relaxed),
        dropped: FRAMES_DROPPED.load(Ordering::Relaxed),
    }
}

/// A frame the relay lost. Counted both against the relay's own counter and against
/// the bus's global `dropped`, because the two answer different questions: "is this
/// link losing frames?" and "did the device fail to deliver something it wanted to
/// send?".
///
/// The global is therefore per *transport attempt*, not per frame -- consistent with
/// how the USB writer already feeds it from both its DTR and its lag paths. One frame
/// that neither transport delivers adds two. Said out loud because
/// `frames_dropped - link_frames_dropped` is a tempting way to isolate the USB
/// share, and it happens to be correct only because of this overlap.
fn note_dropped() {
    FRAMES_DROPPED.fetch_add(1, Ordering::Relaxed);
    bus::note_dropped();
}

/// `link_baud` is the UART's configured baud rate. The byte budget is a *fraction* of the
/// link rather than an absolute, because that is what its justification has always been:
/// the budget exists to leave the machine's own traffic room, and "room" is a proportion of
/// the wire, not a number of bytes.
///
/// Both boards now run 576 kbaud with hardware flow control. They did not always -- while
/// `single-boiler` ran 115 200 with no RTS/CTS, one absolute figure would have been 5% of
/// one link and 26% of the other, and on the board with no flow control there was nothing
/// to push back when debug traffic oversubscribed it. Keeping the budget proportional is
/// what made that difference harmless, and is why it survives the two rates converging: the
/// next board to differ costs nothing here.
pub async fn relay<M: embassy_sync::blocking_mutex::raw::RawMutex>(
    tx_sender: Sender<'_, M, Vec<u8>, 10>,
    link_baud: u32,
) {
    // Cannot happen with the bus as configured -- `BUS_SUBSCRIBERS` is 2 and there
    // are exactly two consumers -- but say so rather than dying quietly if a third
    // consumer ever takes the slot. A relay that silently does nothing looks
    // identical to a comms processor that is not listening.
    let Some(mut subscriber) = bus::subscriber() else {
        bus::emit_event(DebugEvent::SpawnFailed { task: name("debug_relay") });
        return;
    };
    let mut bucket = TokenBucket::for_baud(link_baud);

    loop {
        // `next_message`, not `next_message_pure`: the latter silently swallows
        // `Lagged`, so ring entries recycled out from under this subscriber would
        // never be counted and both `link_frames_dropped` and the bus's global
        // `dropped` would under-report exactly the condition they exist for. Each
        // lagged message is one genuinely lost frame, so count all `n`.
        let frame = match subscriber.next_message().await {
            WaitResult::Lagged(n) => {
                for _ in 0..n {
                    note_dropped();
                }
                continue;
            }
            WaitResult::Message(frame) => frame,
        };

        // Before encoding, not after: encoding is where the cost is.
        if !relayable(&frame.payload) {
            continue;
        }

        // Also before encoding, for the same reason -- and because a frame refused
        // here must not cost an `LlffHeap::alloc` on the processor running the PID
        // loops. The `try_send` below stays as the backstop for the slot that goes
        // between this check and that call; this is what stops the relay taking the
        // last of a queue the machine is about to need. See [`TX_RESERVED_SLOTS`].
        if tx_sender.free_capacity() <= TX_RESERVED_SLOTS {
            note_dropped();
            continue;
        }

        let wrapped = ApplicationProcessorToCommsProcessorMessage::Debug(frame);
        let Ok(encoded) = to_allocvec_cobs(&wrapped) else {
            note_dropped();
            continue;
        };

        let now_ms = embassy_time::Instant::now().as_millis();
        if !bucket.allow(now_ms, encoded.len() as u32) {
            note_dropped();
            continue;
        }

        // try_send: if the shared TX queue is full, Status traffic is mid-flight
        // and debug data yields. Never `send`, which would await -- nothing on the
        // debug path may block on the far end keeping up.
        if tx_sender.try_send(encoded).is_err() {
            note_dropped();
        } else {
            FRAMES_RELAYED.fetch_add(1, Ordering::Relaxed);
        }
    }
}
