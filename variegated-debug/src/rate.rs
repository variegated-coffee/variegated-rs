//! Byte-rate limiting for debug traffic sharing the inter-processor link.

/// Debug bytes per second the relay may put on the 576 kbaud link (~57.6 kB/s).
/// Capping at roughly 5% guarantees Status and Configuration always win, which
/// matters because emission is always-on rather than subscription-gated.
pub const DEBUG_RELAY_BYTES_PER_SEC: u32 = 3_000;

/// Length of one accounting window.
pub const WINDOW_MS: u64 = 100;
/// Bytes allowed per window.
pub const WINDOW_BUDGET: u32 = DEBUG_RELAY_BYTES_PER_SEC / (1000 / WINDOW_MS) as u32;

/// Fixed-window byte budget. Not a leaky bucket: a hard window is enough here and
/// keeps the arithmetic obvious.
pub struct TokenBucket {
    window_start_ms: u64,
    spent: u32,
}

impl Default for TokenBucket {
    fn default() -> Self {
        Self::new()
    }
}

impl TokenBucket {
    pub const fn new() -> Self {
        Self { window_start_ms: 0, spent: 0 }
    }

    /// Returns true and charges the budget if `bytes` fit in the current window.
    /// A refusal charges nothing, so one oversized item cannot wedge the bucket.
    pub fn allow(&mut self, now_ms: u64, bytes: u32) -> bool {
        if now_ms.saturating_sub(self.window_start_ms) >= WINDOW_MS {
            self.window_start_ms = now_ms;
            self.spent = 0;
        }
        if self.spent.saturating_add(bytes) <= WINDOW_BUDGET {
            self.spent += bytes;
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
    fn spends_the_window_budget_then_refuses() {
        let mut bucket = TokenBucket::new();
        assert!(bucket.allow(0, WINDOW_BUDGET));
        assert!(!bucket.allow(0, 1));
    }

    #[test]
    fn refills_on_the_next_window() {
        let mut bucket = TokenBucket::new();
        assert!(bucket.allow(0, WINDOW_BUDGET));
        assert!(!bucket.allow(50, 1));
        assert!(bucket.allow(WINDOW_MS, 1));
    }

    #[test]
    fn a_single_oversized_item_is_refused_not_wedged() {
        let mut bucket = TokenBucket::new();
        assert!(!bucket.allow(0, WINDOW_BUDGET + 1));
        assert!(bucket.allow(0, 1), "refusal must not consume budget");
    }
}
