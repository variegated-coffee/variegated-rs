//! Byte-rate limiting for debug traffic sharing the inter-processor link.
//!
//! A token bucket, not a fixed window, and the distinction is the whole point: the
//! two cases this has to tell apart are **burst** and **sustained**, and a fixed
//! window cannot express the difference. This module *was* a fixed window, and it
//! reintroduced -- one layer below -- the exact defect `crate::suppress` was
//! rewritten to fix: 300 bytes per 100 ms bounded the sustained rate
//! correctly while admitting only 12 of the boot log's ~21 distinct lines. That is
//! information destroyed, not repetition collapsed, and it also threw away the
//! suppressor's own 32-token burst capacity, which was sized specifically to let
//! those 21 lines through.
//!
//! The budget is a *fraction of the link*, not an absolute. It has to be: the
//! constant's justification has always been "~5% so Status and Configuration always
//! win", which is a ratio, and the two boards do not share a baud rate --
//! `dual-boiler` runs the link at 576 kbaud with hardware flow control, while
//! `single-boiler` runs it at 115 200 with none. A fixed 3000 B/s was 5% of the
//! first and 26% of the second, and on the board with no RTS/CTS to push back,
//! oversubscribing does not merely delay Status, it overruns the receiver's FIFO and
//! corrupts it. Hence [`TokenBucket::for_baud`], and hence no `new()`: there is no
//! safe default for a link whose speed you have not been told.

/// Share of the inter-processor link debug traffic may occupy, as a denominator.
/// 20 means one twentieth: 5%.
///
/// Emission is always-on rather than subscription-gated, so this is a permanent
/// tax on the link, not a burst during a debug session. Status and Configuration
/// have to win every time.
pub const LINK_FRACTION_DENOMINATOR: u32 = 20;

/// Bits on the wire per byte at 8N1: one start, eight data, one stop. Both boards
/// configure the link this way (`uart::Config::default()`), so bytes per second is
/// baud / 10.
pub const WIRE_BITS_PER_BYTE: u32 = 10;

/// Debug bytes per second permitted on a link running at `baud`.
pub const fn bytes_per_sec_for_baud(baud: u32) -> u32 {
    baud / WIRE_BITS_PER_BYTE / LINK_FRACTION_DENOMINATOR
}

/// The faster of the two links in the tree: `dual-boiler`'s 576 kbaud UART.
pub const REFERENCE_BAUD: u32 = 576_000;

/// The reference link's sustained budget: 2880 B/s.
///
/// Named because it is the figure the steady-state measurement is quoted against.
/// It is **not** a default -- construct a bucket with [`TokenBucket::for_baud`] and
/// the link's real speed.
pub const DEBUG_RELAY_BYTES_PER_SEC: u32 = bytes_per_sec_for_baud(REFERENCE_BAUD);

/// Burst the bucket absorbs from full, in bytes.
///
/// Sized against the worst legitimate burst on the **slowest** link, which is the
/// binding case. The boot log is ~21 distinct lines within about 300 ms
/// (`suppress::BOOT_BURST_LINES`, enumerated from the boot path rather than
/// estimated), every one a first sighting with no duplicate anywhere, so any line
/// the bucket refuses is information destroyed. At the maximum `TEXT_LEN` width each
/// relayed `Text` frame runs to a little over a hundred bytes, and at 115 200 baud
/// only ~170 bytes refill across the burst -- so nearly the whole thing has to come
/// out of capacity. `relay::the_boot_burst_crosses_both_links` measures it on both
/// links rather than trusting this arithmetic.
///
/// Not smaller: the margin here is what the suppressor learned to protect. Its
/// `CAP_CAPACITY` was raised 24 -> 32 precisely because a thin margin silently
/// thins the boot log, which is the failure this constant exists to prevent.
///
/// Not larger, because capacity is the one-time burst the sustained bound gives
/// away -- `the_sustained_bound_survives_the_burst_capacity` allows
/// `BURST_BYTES + bytes_per_sec * secs`, so raising this weakens that bound by
/// exactly this much, once.
///
/// Note what capacity is *not*: it exceeds the codec's `MAX_FRAME` (2048), so any
/// frame that can be encoded at all can cross the link given a full bucket. In
/// particular the bucket is **not** a backstop against `DebugPayload::Status`.
/// Under the old fixed window a 1722-byte Status could not fit any window and was
/// refused by arithmetic; now [`crate::relay::relayable`] is the only thing keeping
/// it off the link. See that function for why that is the right place for the
/// decision anyway.
pub const BURST_BYTES: u32 = 2_560;

/// Bucket level is tracked in thousandths of a byte so sub-byte refill is not lost
/// to integer truncation between closely spaced frames. One elapsed millisecond is
/// worth exactly `bytes_per_sec` milli-bytes, which keeps the arithmetic exact at
/// any baud rate -- unlike whole bytes, where 2880 B/s truncates to 2 B/ms and loses
/// 30% of the refill against a caller that ticks every millisecond.
const MILLI: u32 = 1_000;
const BURST_MILLI: u32 = BURST_BYTES * MILLI;

/// Token bucket over link bytes. Starts full, so a boot burst is absorbed rather
/// than thinned.
pub struct TokenBucket {
    level_milli: u32,
    refilled_ms: u64,
    bytes_per_sec: u32,
}

impl TokenBucket {
    /// A bucket budgeting [`LINK_FRACTION_DENOMINATOR`]'s share of a link running at
    /// `baud`.
    ///
    /// The only constructor. A `new()` or `Default` would have to assume a baud
    /// rate, and assuming the fast one on the slow board is precisely the bug this
    /// signature exists to prevent.
    pub const fn for_baud(baud: u32) -> Self {
        Self {
            level_milli: BURST_MILLI,
            refilled_ms: 0,
            bytes_per_sec: bytes_per_sec_for_baud(baud),
        }
    }

    /// The sustained rate this bucket refills at, in bytes per second.
    pub const fn bytes_per_sec(&self) -> u32 {
        self.bytes_per_sec
    }

    /// Returns true and charges the bucket if `bytes` fit the current level.
    /// A refusal charges nothing, so one oversized item cannot wedge the bucket.
    pub fn allow(&mut self, now_ms: u64, bytes: u32) -> bool {
        // `saturating_sub` so a clock that appears to move backwards refills
        // nothing rather than underflowing.
        let elapsed = now_ms.saturating_sub(self.refilled_ms);
        if elapsed > 0 {
            let refill = elapsed.saturating_mul(self.bytes_per_sec as u64);
            let refill = refill.min(BURST_MILLI as u64) as u32;
            self.level_milli = self.level_milli.saturating_add(refill).min(BURST_MILLI);
            self.refilled_ms = now_ms;
        }

        let cost = (bytes as u64).saturating_mul(MILLI as u64);
        if cost <= self.level_milli as u64 {
            self.level_milli -= cost as u32;
            true
        } else {
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_budget_follows_the_link_speed() {
        // 5% of 57 600 B/s and of 11 520 B/s respectively. The second board is the
        // reason this is a function rather than a constant: at the old fixed
        // 3000 B/s it was taking 26% of a link with no hardware flow control.
        assert_eq!(bytes_per_sec_for_baud(576_000), 2_880);
        assert_eq!(bytes_per_sec_for_baud(115_200), 576);
        assert_eq!(DEBUG_RELAY_BYTES_PER_SEC, 2_880);
    }

    #[test]
    fn a_full_bucket_absorbs_a_burst_a_fixed_window_would_have_refused() {
        let mut bucket = TokenBucket::for_baud(REFERENCE_BAUD);
        // The whole burst inside a single millisecond, so no refill helps: this is
        // capacity alone. The old 300-byte window admitted two of these.
        let mut admitted = 0;
        for _ in 0..20 {
            if bucket.allow(0, 128) {
                admitted += 128;
            }
        }
        assert_eq!(admitted, (20 * 128u32).min(BURST_BYTES));
        assert!(admitted > 300, "a fixed 300 B window is what this replaced");
    }

    #[test]
    fn refill_is_exact_at_millisecond_granularity() {
        let mut bucket = TokenBucket::for_baud(REFERENCE_BAUD);
        assert!(bucket.allow(0, BURST_BYTES), "starts full");
        assert!(!bucket.allow(0, 1), "and is now empty");

        // 2880 B/s is 2.88 B/ms, which truncates to 2 in whole bytes. Ticking every
        // millisecond for half a second must still buy back the full 1440, not the
        // 1000 whole-byte arithmetic would give -- this is what the milli-byte level
        // is for. Half a second rather than a whole one because `BURST_BYTES` (2560)
        // is below one second of refill, so a full second would clamp at capacity and
        // the truncation would be invisible.
        for ms in 1..=500u64 {
            bucket.allow(ms, 0);
        }
        assert!(
            bucket.allow(500, 1_440),
            "500 ms of millisecond-granularity refill must be worth 1440 B, not 1000"
        );
        assert!(!bucket.allow(500, 1), "and no more than that");
    }

    #[test]
    fn the_sustained_bound_survives_the_burst_capacity() {
        // Half of a necessary pairing: a burst test alone cannot tell a
        // generous bucket from an unbounded one. Drive far past the budget for ten
        // seconds and confirm throughput settles at the refill rate plus the
        // one-time capacity, not at the offered rate.
        const SECS: u64 = 10;
        let mut bucket = TokenBucket::for_baud(REFERENCE_BAUD);
        let mut admitted: u64 = 0;
        // 100 B offered every millisecond = 100 kB/s, ~35x the budget.
        for ms in 0..(SECS * 1000) {
            if bucket.allow(ms, 100) {
                admitted += 100;
            }
        }

        let bound = BURST_BYTES as u64 + DEBUG_RELAY_BYTES_PER_SEC as u64 * SECS;
        assert!(
            admitted <= bound,
            "admitted {admitted} B over {SECS} s, above the {bound} B bound"
        );
        // And it really is converging on the refill rate rather than stalling.
        assert!(admitted >= DEBUG_RELAY_BYTES_PER_SEC as u64 * SECS);
    }

    #[test]
    fn the_slow_link_gets_the_slow_bound() {
        // Same drive against `single-boiler`'s link. Without `for_baud` this would
        // have admitted the reference link's 2880 B/s -- 26% of a 115 200 baud link
        // with no hardware flow control.
        const SECS: u64 = 10;
        let mut bucket = TokenBucket::for_baud(115_200);
        let mut admitted: u64 = 0;
        for ms in 0..(SECS * 1000) {
            if bucket.allow(ms, 100) {
                admitted += 100;
            }
        }
        let bound = BURST_BYTES as u64 + 576 * SECS;
        assert!(admitted <= bound, "admitted {admitted} B, above the {bound} B bound");
    }

    /// Idle time must not bank credit beyond capacity.
    ///
    /// The sustained test above cannot see this: it drains the bucket on every tick,
    /// so the level never has a chance to grow past capacity and an accumulator with
    /// no ceiling behaves identically. The failure this guards is a machine that sits
    /// quiet for an hour and then, on one bad event, is allowed an hour's worth of
    /// budget in a single instant -- which is precisely "the burst allowance quietly
    /// became an unbounded one".
    ///
    /// Verified by mutation: deleting the `.min(BURST_MILLI)` from `allow` passes
    /// every other test in this module and fails only this one.
    #[test]
    fn a_quiet_period_does_not_bank_unbounded_credit() {
        for quiet in [
            // One long gap. The per-call refill clamp alone covers this case.
            &[3_600_000u64][..],
            // The same idle time as a run of refills that charge nothing, which is
            // what a relay does while the machine is quiet: the level, not the
            // per-call refill, is what has to be bounded here.
            &[1_000, 2_000, 3_000, 4_000, 5_000, 6_000, 7_000, 8_000, 9_000, 10_000][..],
        ] {
            let mut bucket = TokenBucket::for_baud(REFERENCE_BAUD);
            assert!(bucket.allow(0, BURST_BYTES), "drain it");

            let mut now = 0;
            for &t in quiet {
                bucket.allow(t, 0);
                now = t;
            }

            // Now offer far more than capacity, all in the same instant.
            let mut admitted: u64 = 0;
            for _ in 0..1000 {
                if bucket.allow(now, 128) {
                    admitted += 128;
                }
            }

            assert!(
                admitted <= BURST_BYTES as u64,
                "banked {admitted} B across {} idle step(s), above the {BURST_BYTES} B capacity",
                quiet.len()
            );
        }
    }

    #[test]
    fn a_single_oversized_item_is_refused_not_wedged() {
        let mut bucket = TokenBucket::for_baud(REFERENCE_BAUD);
        assert!(!bucket.allow(0, BURST_BYTES + 1));
        assert!(bucket.allow(0, 1), "refusal must not consume budget");
    }

    #[test]
    fn capacity_covers_a_whole_encodable_frame() {
        // So a large-but-legal frame is never permanently refused the way a
        // 1722-byte Status was under the fixed window. This is also the assertion
        // that records the bucket is not a Status backstop -- see BURST_BYTES.
        assert!(
            BURST_BYTES >= variegated_debug_codec::MAX_FRAME as u32,
            "capacity {BURST_BYTES} is below MAX_FRAME, so some encodable frames could never cross"
        );
    }
}
