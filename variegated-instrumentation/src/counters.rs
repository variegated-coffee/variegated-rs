use portable_atomic::{AtomicU64, Ordering};

use crate::handle::CounterHandle;

/// Central registry for performance counters.
///
/// This type holds N counters, each identified by a u8 ID (0..N-1).
/// Each counter is backed by an AtomicU64, allowing lock-free reads and writes.
///
/// # Usage
///
/// Typically, you'll create a single static instance of this type:
///
/// ```ignore
/// static PERF: PerformanceCounters<10> = PerformanceCounters::new();
/// ```
///
/// Then obtain handles for writing and read counters directly:
///
/// ```ignore
/// let handle = PERF.handle(CounterId::LoopIterations);
/// handle.increment();
///
/// let count = PERF.read(CounterId::LoopIterations);
/// ```
pub struct PerformanceCounters<const N: usize> {
    counters: [AtomicU64; N],
}

impl<const N: usize> PerformanceCounters<N> {
    /// Creates a new performance counter registry with all counters initialized to zero.
    ///
    /// This is a const function, so it can be used in static initializers.
    #[inline]
    pub const fn new() -> Self {
        // Use array initialization with const repeat
        const INIT: AtomicU64 = AtomicU64::new(0);
        Self {
            counters: [INIT; N],
        }
    }

    /// Obtains a handle for writing to a specific counter.
    ///
    /// The handle provides methods for incrementing the counter.
    ///
    /// # Panics
    ///
    /// Panics if the counter ID is out of bounds (>= N).
    ///
    /// # Safety Contract
    ///
    /// The caller must ensure that only one handle per counter ID is actively
    /// used for writing. Multiple handles can be created, but only one should
    /// perform writes to maintain the single-writer guarantee.
    #[inline]
    pub fn handle(&'static self, id: impl Into<u8>) -> CounterHandle<N> {
        let id = id.into();
        assert!(
            (id as usize) < N,
            "Counter ID {} out of bounds (max {})",
            id,
            N - 1
        );
        CounterHandle::new(self, id)
    }

    /// Reads the current value of a counter.
    ///
    /// This operation uses `Acquire` ordering to ensure that all writes
    /// to the counter are visible.
    ///
    /// # Panics
    ///
    /// Panics if the counter ID is out of bounds (>= N).
    #[inline]
    pub fn read(&self, id: impl Into<u8>) -> u64 {
        let id = id.into();
        assert!(
            (id as usize) < N,
            "Counter ID {} out of bounds (max {})",
            id,
            N - 1
        );
        self.counters[id as usize].load(Ordering::Acquire)
    }

    /// Reads all counters and returns them as an array.
    ///
    /// This provides a consistent snapshot of all counter values at the time of the call.
    #[inline]
    pub fn read_all(&self) -> [u64; N] {
        let mut result = [0u64; N];
        for (i, counter) in self.counters.iter().enumerate() {
            result[i] = counter.load(Ordering::Acquire);
        }
        result
    }

    /// Internal method used by CounterHandle to increment a counter.
    ///
    /// Uses `Relaxed` ordering since we assume single-writer-per-counter.
    #[inline]
    pub(crate) fn increment_internal(&self, id: u8, delta: u64) {
        self.counters[id as usize].fetch_add(delta, Ordering::Relaxed);
    }
}

#[cfg(feature = "defmt")]
impl<const N: usize> defmt::Format for PerformanceCounters<N> {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "PerformanceCounters[{}](", N);
        for (i, counter) in self.counters.iter().enumerate() {
            if i > 0 {
                defmt::write!(fmt, ", ");
            }
            defmt::write!(fmt, "{}", counter.load(Ordering::Acquire));
        }
        defmt::write!(fmt, ")");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new() {
        let counters: PerformanceCounters<5> = PerformanceCounters::new();
        for i in 0..5 {
            assert_eq!(counters.read(i as u8), 0);
        }
    }

    #[test]
    fn test_read_all() {
        let counters: PerformanceCounters<3> = PerformanceCounters::new();
        counters.increment_internal(0, 10);
        counters.increment_internal(1, 20);
        counters.increment_internal(2, 30);

        let all = counters.read_all();
        assert_eq!(all, [10, 20, 30]);
    }

    #[test]
    #[should_panic(expected = "Counter ID 5 out of bounds")]
    fn test_read_out_of_bounds() {
        let counters: PerformanceCounters<5> = PerformanceCounters::new();
        let _ = counters.read(5u8);
    }
}
