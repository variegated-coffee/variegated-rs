//! Thinning for repetitive text frames, so a chatty control loop cannot empty the
//! bus ring of everything else.
//!
//! Lives here rather than in `variegated-log`'s sink because `variegated-log` sets
//! `test = false` and depends on `embassy-rp` with chip features, so nothing in it
//! can be exercised on a host. This module is pure -- the caller supplies the
//! timestamp -- so the decisions below are covered by real unit tests instead of a
//! simulation of a copy of the algorithm.
//!
//! # Why thinning at all
//!
//! `publish_immediate` evicts the **oldest** frame to make room. Under text
//! pressure the oldest frame is routinely a `DebugPayload::Status`, so a burst of
//! text does not merely add noise: it drops other frames before a host can read
//! them, and frees a `Box<Status>` inside the bus's critical section while doing
//! it.
//!
//! # Two layers
//!
//! 1. **Per-message**, the common case: an identical message repeats at most once
//!    per [`SUPPRESS_WINDOW_MS`]. Repetition is what a control loop produces, and
//!    collapsing it is nearly free.
//! 2. **A global cap**, the backstop: at most [`CAP_PER_WINDOW`] frames per
//!    [`CAP_WINDOW_MS`] regardless of message diversity. Without it, more
//!    simultaneously-hot messages than [`TRACKED`] does not degrade gracefully --
//!    the surplus messages thrash over the last slot, each evicting the other, and
//!    every one of them publishes at full loop rate. The cap turns that cliff into
//!    a bound.

use portable_atomic::{AtomicU32, Ordering};
use variegated_controller_types::debug::Severity;

/// How long an identical message is suppressed **after being published**.
///
/// The window restarts from each publish, not from each occurrence, so a
/// persistently repeating message keeps emitting at `1 / SUPPRESS_WINDOW_MS` for as
/// long as the condition holds. That heartbeat is the point: the boiler interlock
/// lines exist to tell a host *which* of three interlocks is active, and a signal
/// that appears once and never repeats is indistinguishable from one that has
/// stopped -- the surviving frame is evicted from the 16-slot ring within seconds
/// by ordinary 1 Hz traffic, and then the interlock is invisible.
///
/// 2 s rather than something shorter because the rate multiplies by the number of
/// hot messages: the dual boiler can hold all six interlock messages at once, which
/// at 2 s is ~3 frames/sec against ~4/sec of structured traffic. At 500 ms it would
/// be 12/sec, which turns the ring over about every 1.3 s and is most of the
/// original problem.
pub const SUPPRESS_WINDOW_MS: u32 = 2_000;

/// Number of distinct recent messages tracked.
///
/// A single slot only collapses *strictly consecutive* repeats: two sites that
/// alternate (brew and steam running the same interlock back to back, or any two
/// repeating sites interleaved from different tasks) produce A,B,A,B, and each
/// record clears the other's fingerprint.
///
/// 16 because the dual boiler declares six interlock messages before any info-level
/// traffic is counted, and the lookup is a linear scan of `u32` loads -- cheap
/// enough that headroom costs nothing measurable.
pub const TRACKED: usize = 16;

/// Length of the global cap's accounting window.
pub const CAP_WINDOW_MS: u32 = 500;
/// Frames admitted per [`CAP_WINDOW_MS`], across all messages.
///
/// 5 per 500 ms = 10/sec. Chosen to sit *above* the expected worst case (all six
/// interlocks at 0.5 Hz is ~3/sec) so it never engages in normal operation, while
/// still halving the pathological diversity case. It is a floor, not the primary
/// mechanism.
pub const CAP_PER_WINDOW: u32 = 5;

/// FNV-1a over the severity and the message.
///
/// Allocation-free and cheap on the emit path: one pass over at most `TEXT_LEN`
/// bytes. Never returns 0, so 0 can mean "slot empty" without a separate flag.
fn fingerprint(severity: Severity, msg: &str) -> u32 {
    let mut hash: u32 = 0x811c_9dc5;
    for byte in core::iter::once(severity as u8).chain(msg.as_bytes().iter().copied()) {
        hash ^= byte as u32;
        hash = hash.wrapping_mul(0x0100_0193);
    }
    if hash == 0 { 1 } else { hash }
}

/// Decides whether a text frame may go on the bus.
///
/// Racy by construction: two tasks logging concurrently can both observe the same
/// prior state and both publish. That is the correct trade -- the emit path may not
/// take a lock, and the cost of a missed suppression is one extra frame, not a
/// correctness problem.
pub struct Suppressor {
    /// Fingerprints of recently published messages; 0 means "slot empty".
    fingerprints: [AtomicU32; TRACKED],
    /// Uptime at which each slot was last **published**, truncated to 32 bits.
    /// `u32` rather than `u64` because Cortex-M33 has no native 64-bit atomic and
    /// the workspace builds `portable-atomic` without its critical-section
    /// fallback. Truncation is harmless: `wrapping_sub` measures the gap correctly
    /// across the ~49-day wrap, and the worst case at the wrap is one un-suppressed
    /// duplicate.
    published_ms: [AtomicU32; TRACKED],
    cap_window_start_ms: AtomicU32,
    cap_spent: AtomicU32,
}

impl Default for Suppressor {
    fn default() -> Self {
        Self::new()
    }
}

impl Suppressor {
    pub const fn new() -> Self {
        Self {
            fingerprints: [const { AtomicU32::new(0) }; TRACKED],
            published_ms: [const { AtomicU32::new(0) }; TRACKED],
            cap_window_start_ms: AtomicU32::new(0),
            cap_spent: AtomicU32::new(0),
        }
    }

    /// True if this frame may be published; false if it should be thinned.
    ///
    /// State is only advanced when the answer is true, so `published_ms` always
    /// means "when this message last actually reached the bus" -- which is what
    /// makes the heartbeat work.
    pub fn admit(&self, severity: Severity, msg: &str, now_ms: u32) -> bool {
        let fp = fingerprint(severity, msg);

        // Slot this message will occupy if it publishes: its own if already
        // tracked, otherwise the one whose last publish is oldest, so the set
        // holds the currently-hot messages rather than the most recently seen.
        let mut slot = 0usize;
        let mut oldest_age = 0u32;

        for i in 0..TRACKED {
            let seen = self.fingerprints[i].load(Ordering::Relaxed);
            let age = now_ms.wrapping_sub(self.published_ms[i].load(Ordering::Relaxed));

            if seen == fp {
                if age < SUPPRESS_WINDOW_MS {
                    return false;
                }
                // Due a heartbeat. Fall through to the cap; do not refresh the
                // timestamp unless it actually publishes.
                slot = i;
                break;
            }

            if seen == 0 {
                // An empty slot is infinitely old, so it wins eviction outright.
                slot = i;
                oldest_age = u32::MAX;
            } else if age >= oldest_age {
                slot = i;
                oldest_age = age;
            }
        }

        if !self.admit_globally(now_ms) {
            return false;
        }

        self.fingerprints[slot].store(fp, Ordering::Relaxed);
        self.published_ms[slot].store(now_ms, Ordering::Relaxed);
        true
    }

    /// Fixed-window cap across all messages. A hard window rather than a leaky
    /// bucket, matching `rate::TokenBucket` -- the arithmetic stays obvious and
    /// this only has to bound a pathological case, not shape traffic.
    fn admit_globally(&self, now_ms: u32) -> bool {
        let start = self.cap_window_start_ms.load(Ordering::Relaxed);
        if now_ms.wrapping_sub(start) >= CAP_WINDOW_MS {
            self.cap_window_start_ms.store(now_ms, Ordering::Relaxed);
            self.cap_spent.store(1, Ordering::Relaxed);
            return true;
        }

        let spent = self.cap_spent.load(Ordering::Relaxed);
        if spent < CAP_PER_WINDOW {
            self.cap_spent.store(spent + 1, Ordering::Relaxed);
            true
        } else {
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Drive `admit` at `hz` for `secs` and count how many frames it lets through.
    fn run(suppressor: &Suppressor, messages: &[&str], hz: u32, secs: u32) -> u32 {
        let step_ms = 1000 / hz;
        let mut published = 0;
        for tick in 0..(hz * secs) {
            let now = tick * step_ms;
            for msg in messages {
                if suppressor.admit(Severity::Warn, msg, now) {
                    published += 1;
                }
            }
        }
        published
    }

    /// The defect this module was rewritten for: refreshing the timestamp on the
    /// suppressed path meant a persistently repeating message published exactly
    /// once, ever, and then went silent -- after which its single frame was evicted
    /// from the ring and the condition became invisible.
    #[test]
    fn a_persistent_message_keeps_a_heartbeat() {
        let s = Suppressor::new();
        // 10 Hz for 10 s, one message: ticks at t = 0, 100, ... 9900.
        let published = run(&s, &["interlock"], 10, 10);
        // Publishes at t = 0, 2000, 4000, 6000, 8000. The next would be t = 10000,
        // one tick past the end of the run.
        assert_eq!(published, 5, "expected a heartbeat, got {published} frames");
        assert!(published > 1, "a single frame is the silent-forever bug");
    }

    /// Two alternating messages must both stay resident; with a single tracked
    /// slot each cleared the other's fingerprint and neither was ever suppressed.
    #[test]
    fn an_alternating_pair_is_suppressed_independently() {
        let s = Suppressor::new();
        let published = run(&s, &["brew interlock", "steam interlock"], 10, 10);
        // Each behaves exactly as the single-message case: 5 apiece, no
        // interference. A single tracked slot gave 200 here -- every occurrence.
        assert_eq!(published, 10, "expected 5 per message, got {published} total");
    }

    /// Exceeding `TRACKED` must not reinstate the full-rate flood. Before the
    /// global cap, the surplus messages thrashed over the last slot and published
    /// on every iteration.
    #[test]
    fn exceeding_the_tracked_set_stays_bounded() {
        let s = Suppressor::new();
        let many: [&str; TRACKED + 4] = [
            "m00", "m01", "m02", "m03", "m04", "m05", "m06", "m07", "m08", "m09",
            "m10", "m11", "m12", "m13", "m14", "m15", "m16", "m17", "m18", "m19",
        ];
        let published = run(&s, &many, 10, 10);
        std::eprintln!("{} messages over capacity published {published} frames", many.len());

        // The absolute bound is the global cap: 5 per 500 ms over 10 s.
        let cap = CAP_PER_WINDOW * (10_000 / CAP_WINDOW_MS);
        assert!(
            published <= cap,
            "published {published} frames, cap allows {cap}"
        );
        // And far below the un-thinned rate of 20 messages x 100 ticks.
        assert!(published < 200, "{published} frames is close to un-thinned");
    }

    /// A message that has not been seen for longer than the window publishes
    /// immediately rather than waiting out a stale timestamp.
    #[test]
    fn a_returning_message_publishes_at_once() {
        let s = Suppressor::new();
        assert!(s.admit(Severity::Warn, "x", 0));
        assert!(!s.admit(Severity::Warn, "x", 100));
        assert!(s.admit(Severity::Warn, "x", SUPPRESS_WINDOW_MS));
    }

    /// Severity is part of the fingerprint: the same text at two levels is two
    /// messages, because a warning and an error mean different things.
    #[test]
    fn severity_distinguishes_otherwise_identical_text() {
        let s = Suppressor::new();
        assert!(s.admit(Severity::Warn, "same", 0));
        assert!(s.admit(Severity::Error, "same", 0));
    }

    /// Distinct messages inside one window are all admitted up to the cap -- the
    /// per-message check must not serialise unrelated sites.
    #[test]
    fn distinct_messages_are_not_blocked_by_each_other() {
        let s = Suppressor::new();
        assert!(s.admit(Severity::Info, "a", 0));
        assert!(s.admit(Severity::Info, "b", 0));
        assert!(s.admit(Severity::Info, "c", 0));
    }

    /// The cap must not wedge: a refused frame charges nothing, and the next
    /// window admits again.
    #[test]
    fn the_global_cap_refills() {
        let s = Suppressor::new();
        for i in 0..CAP_PER_WINDOW {
            assert!(s.admit(Severity::Info, msg_for(i), 0), "frame {i} refused");
        }
        assert!(!s.admit(Severity::Info, "overflow", 0));
        assert!(s.admit(Severity::Info, "overflow", CAP_WINDOW_MS));
    }

    fn msg_for(i: u32) -> &'static str {
        ["c0", "c1", "c2", "c3", "c4", "c5"][i as usize]
    }
}
