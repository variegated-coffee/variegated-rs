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
//! `publish_immediate` evicts the **oldest** frame to make room, so a burst of text
//! does not merely add noise: it drops other frames -- typed events, counter
//! samples, the state snapshot -- before a host can read them.
//!
//! It used to also free a `Box<Status>` inside the bus's critical section, because
//! `DebugPayload::Status` was routinely the oldest frame. It no longer can: `Status`
//! moved to its own single-slot channel ([`crate::status`]) when the inter-processor
//! relay became a second bus subscriber. Nothing on the bus owns a heap allocation
//! any more, so eviction is now a plain drop. The thinning still matters for the
//! reason above.
//!
//! # Two layers
//!
//! 1. **Per-message**, the common case: an identical message repeats at most once
//!    per [`SUPPRESS_WINDOW_MS`]. Repetition is what a control loop produces, and
//!    collapsing it is nearly free.
//! 2. **A global token bucket**, the backstop. Without it, more simultaneously-hot
//!    messages than [`TRACKED`] does not degrade gracefully -- the surplus messages
//!    thrash over the last slot, each evicting the other, and every one of them
//!    publishes at full loop rate. The bucket turns that cliff into a bound.
//!
//! The second layer is a bucket rather than a fixed window because the two cases it
//! has to tell apart are *burst* and *sustained*, which a fixed window cannot
//! express. A fixed window of 5 per 500 ms throttled the boot log: about 21 distinct
//! init lines arrive within a few hundred milliseconds, and only 5 of them reached
//! the bus. Those are first sightings with no duplicate anywhere -- thinning them
//! destroys information rather than collapsing repetition, which is the exact
//! symptom this whole task exists to fix.
//!
//! Exempting first sightings would not work either: a message thrashing in and out
//! of the [`TRACKED`] slots presents as a first sighting on every re-insertion, so
//! the exemption would reopen the flood it is meant to bound.

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

/// Burst the global bucket absorbs from full, in frames.
///
/// Sized off the worst legitimate burst in the tree, the boot log. Enumerated from
/// the boot path rather than estimated: `main.rs` contributes 19 lines that fire on
/// a normal boot (5 in `main`, 1 in `storage_task`, 13 in `main_task`, with
/// mutually-exclusive branches collapsed to one apiece and error paths excluded),
/// and `coordinated_dual_heating_element.rs` adds 2 config-apply lines -- about 21.
/// Every one is a first sighting with no duplicate anywhere, so any of them the
/// bucket refuses is information destroyed.
///
/// 32 leaves ~11 frames of margin over that. An earlier value of 24 claimed "2x
/// headroom" against a miscount of "roughly a dozen"; against the real 21 its true
/// margin was about 4, which is thin for something whose failure mode is silently
/// thinning the boot log.
///
/// Not larger, because capacity is also the one-time burst the sustained bound
/// gives away: `exceeding_the_tracked_set_stays_bounded` allows
/// `CAP_CAPACITY + CAP_REFILL_PER_SEC * secs`. Raising this weakens that bound by
/// exactly this constant, once -- cheap at 32, less so unbounded.
pub const CAP_CAPACITY: u32 = 32;

/// Sustained rate the bucket refills at, in frames per second.
///
/// 10/sec, comfortably above the expected steady state -- all six boiler interlocks
/// holding at once is ~3 frames/sec -- so the bucket sits full and the cap never
/// engages in normal operation. A genuine flood settles here rather than at the
/// loop rate, which is the bound this layer exists to provide.
pub const CAP_REFILL_PER_SEC: u32 = 10;

/// Bucket level is tracked in thousandths of a frame so sub-frame refill is not
/// lost to integer truncation between closely spaced records. At
/// [`CAP_REFILL_PER_SEC`] = 10, one elapsed millisecond is worth exactly
/// `CAP_REFILL_PER_SEC` milli-tokens, which keeps the arithmetic exact.
const MILLI: u32 = 1_000;
const CAP_CAPACITY_MILLI: u32 = CAP_CAPACITY * MILLI;

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
    /// Bucket level in milli-tokens. Starts full so the boot burst passes.
    cap_tokens_milli: AtomicU32,
    /// Uptime the bucket was last refilled at.
    cap_refilled_ms: AtomicU32,
}

/// Why a frame was or was not admitted.
///
/// The three cases mean genuinely different things to a host, so they are not
/// collapsed into a bool: a duplicate costs nothing because an identical frame went
/// out moments ago, whereas a rate-limited frame is information that no longer
/// exists anywhere. Conflating them is how `frames_suppressed` would come to read
/// as "all fine" when it is not.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Admission {
    /// Publish it.
    Publish,
    /// Repeats a message published within [`SUPPRESS_WINDOW_MS`]. The information
    /// is already on the bus; nothing is lost.
    Duplicate,
    /// Refused by the global bucket. This frame is **lost** -- it is not a repeat
    /// of anything, and no other frame carries it.
    RateLimited,
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
            cap_tokens_milli: AtomicU32::new(CAP_CAPACITY_MILLI),
            cap_refilled_ms: AtomicU32::new(0),
        }
    }

    /// Decide whether this frame may be published, and why.
    ///
    /// State is only advanced on [`Admission::Publish`], so `published_ms` always
    /// means "when this message last actually reached the bus" -- which is what
    /// makes the heartbeat work.
    pub fn admit(&self, severity: Severity, msg: &str, now_ms: u32) -> Admission {
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
                    return Admission::Duplicate;
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

        if !self.take_token(now_ms) {
            return Admission::RateLimited;
        }

        self.fingerprints[slot].store(fp, Ordering::Relaxed);
        self.published_ms[slot].store(now_ms, Ordering::Relaxed);
        Admission::Publish
    }

    /// Take one token from the global bucket, refilling it for elapsed time first.
    ///
    /// A bucket rather than the fixed window this replaced, because the two cases
    /// that matter are burst and sustained, and a fixed window cannot tell them
    /// apart: 5 per 500 ms bounded the flood correctly but also throttled the boot
    /// log to 5 of its ~21 distinct lines, destroying information rather than
    /// collapsing repetition.
    ///
    /// Shares `rate::TokenBucket`'s intent but not its type: that one budgets
    /// *bytes* for the inter-processor link and takes `&mut self`, while this
    /// counts frames behind a `&self` static and so has to be atomic. Duplicating
    /// ~15 lines is cheaper than generalising over both.
    fn take_token(&self, now_ms: u32) -> bool {
        let last = self.cap_refilled_ms.load(Ordering::Relaxed);
        // `wrapping_sub` so elapsed time is correct across the 32-bit uptime wrap.
        let elapsed = now_ms.wrapping_sub(last);
        let mut tokens = self.cap_tokens_milli.load(Ordering::Relaxed);

        if elapsed > 0 {
            // Exact at CAP_REFILL_PER_SEC tokens/sec: one ms is that many
            // milli-tokens. Saturating so a long quiet period cannot overflow.
            let refill = elapsed.saturating_mul(CAP_REFILL_PER_SEC);
            tokens = tokens.saturating_add(refill).min(CAP_CAPACITY_MILLI);
            self.cap_refilled_ms.store(now_ms, Ordering::Relaxed);
        }

        if tokens >= MILLI {
            self.cap_tokens_milli.store(tokens - MILLI, Ordering::Relaxed);
            true
        } else {
            // A refusal charges nothing, so one refused frame cannot wedge the
            // bucket -- same property `rate::TokenBucket` documents.
            self.cap_tokens_milli.store(tokens, Ordering::Relaxed);
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 40 distinct messages, enough for any bound this module needs to exercise.
    /// Indexed rather than formatted so the harness stays allocation-free and
    /// usable from a `no_std` test build.
    const DISTINCT: [&str; 40] = [
        "m00", "m01", "m02", "m03", "m04", "m05", "m06", "m07", "m08", "m09",
        "m10", "m11", "m12", "m13", "m14", "m15", "m16", "m17", "m18", "m19",
        "m20", "m21", "m22", "m23", "m24", "m25", "m26", "m27", "m28", "m29",
        "m30", "m31", "m32", "m33", "m34", "m35", "m36", "m37", "m38", "m39",
    ];

    /// Drive `admit` at `hz` for `secs` and count how many frames it lets through.
    fn run(suppressor: &Suppressor, messages: &[&str], hz: u32, secs: u32) -> u32 {
        let step_ms = 1000 / hz;
        let mut published = 0;
        for tick in 0..(hz * secs) {
            let now = tick * step_ms;
            for msg in messages {
                if suppressor.admit(Severity::Warn, msg, now) == Admission::Publish {
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
        let secs = 10;
        let many = &DISTINCT[..TRACKED + 4];
        let published = run(&s, many, 10, secs);
        std::eprintln!("{} messages over capacity published {published} frames", many.len());

        // A sustained flood settles at the refill rate; the bucket can additionally
        // give away its initial capacity once.
        let bound = CAP_CAPACITY + CAP_REFILL_PER_SEC * secs;
        assert!(
            published <= bound,
            "published {published} frames, bucket allows at most {bound}"
        );
        // And far below the un-thinned rate of 20 messages x 100 ticks = 2000.
        assert!(published < 200, "{published} frames is close to un-thinned");
    }

    /// The regression guard for the defect a fixed window caused. Every line here
    /// is a first sighting with no duplicate anywhere, so thinning any of them
    /// destroys information rather than collapsing repetition -- and under the old
    /// 5-per-500 ms window only 5 reached the bus, on every single boot.
    ///
    /// **This guard alone does not pin bucket semantics.** It catches a window
    /// *narrower* than the burst, but a wide one -- 32 per 500 ms, the "just raise
    /// the count" alternative -- passes it and is caught only by
    /// `exceeding_the_tracked_set_stays_bounded`, which a wide window blows. It
    /// takes both: this one covers the burst side, that one covers the sustained
    /// side, and only together do they force a bucket.
    #[test]
    fn a_boot_burst_of_distinct_messages_all_publishes() {
        let s = Suppressor::new();
        // The enumerated boot path, in order -- see BOOT_BURST_LINES for where the
        // count comes from. Mutually-exclusive branches appear once (the PSRAM-ok
        // arm rather than both arms) and error-only paths are excluded, because
        // neither contributes to a normal boot.
        let boot = [
            "Initing!",
            "PSRAM initialized successfully, using PSRAM for heap",
            "Heap initialized in PSRAM",
            "Spawning display task on core 1",
            "Spawning backlight task on core 1",
            "Storage task started",
            "Starting!",
            "Resetting ADS124S08",
            "Done",
            "Data rate: 20",
            "System clock: 150000000",
            "RTC datetime: 2026-08-02 11:00:00",
            "Gravity sensor initialized - will attempt connection with retry",
            "Belka Portal device initialized",
            "Watchdog initialized with 8000 ms timeout",
            "Configuration loaded",
            "Dual boiler mechanism initialized",
            "Machine definition created",
            "Creating huge future join task",
            "Heating element interlock: true",
            "Heating element contention strategy: Alternate",
        ];

        let mut published = 0;
        for (i, msg) in boot.iter().enumerate() {
            // 15 ms apart, the tightest spacing these plausibly arrive at. The real
            // burst is spread wider -- flash config load, ADC reset delays and the
            // RTC read all buy back refill -- so this is the pessimistic case.
            let now = i as u32 * 15;
            if s.admit(Severity::Info, msg, now) == Admission::Publish {
                published += 1;
            }
        }

        assert_eq!(
            published,
            boot.len(),
            "boot log was thinned: {published} of {} lines reached the bus",
            boot.len()
        );
    }

    /// Rate-limited and duplicate are reported distinctly, because only one of them
    /// means "the information is still on the bus".
    #[test]
    fn the_two_thinning_reasons_are_distinguishable() {
        let s = Suppressor::new();
        assert_eq!(s.admit(Severity::Info, "x", 0), Admission::Publish);
        assert_eq!(s.admit(Severity::Info, "x", 10), Admission::Duplicate);

        // Drain the bucket with distinct messages, then a further new one is
        // refused by the cap rather than reported as a duplicate.
        let drained = Suppressor::new();
        for msg in DISTINCT.iter().take(CAP_CAPACITY as usize) {
            assert_eq!(drained.admit(Severity::Info, msg, 0), Admission::Publish);
        }
        assert_eq!(
            drained.admit(Severity::Info, "one too many", 0),
            Admission::RateLimited
        );
    }

    /// A message that has not been seen for longer than the window publishes
    /// immediately rather than waiting out a stale timestamp.
    #[test]
    fn a_returning_message_publishes_at_once() {
        let s = Suppressor::new();
        assert_eq!(s.admit(Severity::Warn, "x", 0), Admission::Publish);
        assert_eq!(s.admit(Severity::Warn, "x", 100), Admission::Duplicate);
        assert_eq!(
            s.admit(Severity::Warn, "x", SUPPRESS_WINDOW_MS),
            Admission::Publish
        );
    }

    /// Severity is part of the fingerprint: the same text at two levels is two
    /// messages, because a warning and an error mean different things.
    #[test]
    fn severity_distinguishes_otherwise_identical_text() {
        let s = Suppressor::new();
        assert_eq!(s.admit(Severity::Warn, "same", 0), Admission::Publish);
        assert_eq!(s.admit(Severity::Error, "same", 0), Admission::Publish);
    }

    /// Distinct messages inside one window are all admitted -- the per-message
    /// check must not serialise unrelated sites.
    #[test]
    fn distinct_messages_are_not_blocked_by_each_other() {
        let s = Suppressor::new();
        assert_eq!(s.admit(Severity::Info, "a", 0), Admission::Publish);
        assert_eq!(s.admit(Severity::Info, "b", 0), Admission::Publish);
        assert_eq!(s.admit(Severity::Info, "c", 0), Admission::Publish);
    }

    /// The bucket must not wedge: a refused frame charges nothing, and enough
    /// elapsed time admits again.
    #[test]
    fn the_global_bucket_refills() {
        let s = Suppressor::new();
        for msg in DISTINCT.iter().take(CAP_CAPACITY as usize) {
            assert_eq!(s.admit(Severity::Info, msg, 0), Admission::Publish);
        }
        assert_eq!(
            s.admit(Severity::Info, "overflow", 0),
            Admission::RateLimited
        );

        // One token is worth 1000 / CAP_REFILL_PER_SEC milliseconds.
        let one_token_ms = 1_000 / CAP_REFILL_PER_SEC;
        assert_eq!(
            s.admit(Severity::Info, "overflow", one_token_ms),
            Admission::Publish
        );
    }

    /// Distinct log lines a normal dual-boiler boot emits in a burst.
    ///
    /// Enumerated from the boot path, not estimated: 19 in
    /// `examples/dual-boiler/src/main.rs` -- 5 in `main`, 1 in `storage_task`,
    /// 13 in `main_task` -- plus 2 config-apply lines in
    /// `variegated-hal/src/gpio/coordinated_dual_heating_element.rs`.
    /// Mutually-exclusive branches count once (PSRAM ok *or* failed, never both)
    /// and error-only paths are excluded.
    ///
    /// **Revisit this if init logging grows.** Nothing checks it automatically --
    /// the boot sequence itself is not under test -- so a few added `log_info!`
    /// calls at startup will silently erode the margin `CAP_CAPACITY` is sized to
    /// give, and the symptom is a quietly thinned boot log.
    const BOOT_BURST_LINES: u32 = 21;

    /// Capacity has to cover the burst it was sized for, or the boot-log guard
    /// above passes only by accident of how many lines that test happens to list.
    #[test]
    fn capacity_covers_the_boot_burst() {
        assert!(
            CAP_CAPACITY >= BOOT_BURST_LINES,
            "capacity {CAP_CAPACITY} is below the {BOOT_BURST_LINES}-line boot burst"
        );
    }
}
