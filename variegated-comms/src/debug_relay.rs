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
//! mismatch run per link, and Task 18 recorded a worry that one link carrying two
//! processors would break that: a healthy processor's good frames would clear a
//! stale processor's mismatch run and no banner would ever appear. It does not
//! happen on this path. Every frame on the TCP link -- relayed or locally produced
//! -- is stamped by one encoder with one version, so a link still carries exactly
//! one version and per-link keying stays correct.
//!
//! The exposure that *does* exist is a different one, and it is older than this
//! relay: an application processor and a comms processor built from different
//! commits disagree about the shape of `ApplicationProcessorToCommsProcessorMessage`
//! itself, and every message on the link -- Status, Configuration, this one --
//! mis-decodes. That is the inter-processor protocol's problem
//! (`variegated_controller_types::PROTOCOL_VERSION` and the `Hello` handshake, which
//! today nobody sends and nobody checks), not the debug protocol's. Its symptom here
//! is a `DebugEvent::LinkDecodeError`, not a version banner.
//!
//! **If this relay is ever changed to carry pre-encoded bytes** so that a frame keeps
//! its origin processor's version end to end, the mixed-source hazard becomes real
//! and the mismatch run must then be keyed per relay segment -- the comms processor
//! being the only party that can attribute a frame to a link, since `source` lives
//! inside the payload and is exactly the field a mis-versioned frame cannot be
//! trusted about.

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

/// What the relay has done with the frames it was offered.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct RelayStats {
    /// Frames handed to the link's TX queue.
    pub relayed: u32,
    /// Frames the relay threw away: refused by the byte budget, unencodable, or
    /// offered to a full TX queue. Filtered `Status` frames are **not** counted here
    /// -- those are a policy decision, not a loss, and there is nothing to lose since
    /// the comms processor receives `Status` by its own route.
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
fn note_dropped() {
    FRAMES_DROPPED.fetch_add(1, Ordering::Relaxed);
    bus::note_dropped();
}

pub async fn relay<M: embassy_sync::blocking_mutex::raw::RawMutex>(
    tx_sender: Sender<'_, M, Vec<u8>, 10>,
) {
    // Cannot happen with the bus as configured -- `BUS_SUBSCRIBERS` is 2 and there
    // are exactly two consumers -- but say so rather than dying quietly if a third
    // consumer ever takes the slot. A relay that silently does nothing looks
    // identical to a comms processor that is not listening.
    let Some(mut subscriber) = bus::subscriber() else {
        bus::emit_event(DebugEvent::SpawnFailed { task: name("debug_relay") });
        return;
    };
    let mut bucket = TokenBucket::new();

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
