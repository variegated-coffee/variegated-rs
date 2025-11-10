use crate::counters::PerformanceCounters;

/// A lightweight handle for writing to a specific performance counter.
///
/// This handle provides methods for incrementing a counter. It's designed to be
/// cheap to clone and pass around, containing only a counter ID and a static reference.
///
/// # Single Writer Contract
///
/// Each counter should have only one writer (one location in the code that increments it).
/// This contract is not enforced by the type system but is required for correct operation.
/// Multiple `CounterHandle`s can be created for the same counter ID, but only one should
/// actively write to it.
///
/// # Example
///
/// ```ignore
/// static PERF: PerformanceCounters<10> = PerformanceCounters::new();
///
/// #[embassy_executor::task]
/// async fn my_task() {
///     let handle = PERF.handle(CounterId::TaskIterations);
///     loop {
///         handle.increment();
///         // ... do work ...
///     }
/// }
/// ```
#[derive(Copy, Clone)]
pub struct CounterHandle<const N: usize> {
    counters: &'static PerformanceCounters<N>,
    id: u8,
}

impl<const N: usize> CounterHandle<N> {
    /// Creates a new counter handle.
    ///
    /// This is typically called via `PerformanceCounters::handle()` rather than directly.
    #[inline]
    pub(crate) const fn new(counters: &'static PerformanceCounters<N>, id: u8) -> Self {
        Self { counters, id }
    }

    /// Increments the counter by 1.
    ///
    /// This is the most common operation, optimized for minimal overhead.
    #[inline]
    pub fn increment(&self) {
        self.counters.increment_internal(self.id, 1);
    }

    /// Adds a specific value to the counter.
    ///
    /// Use this when you want to increment by more than 1, or when you have
    /// a variable increment amount.
    #[inline]
    pub fn add(&self, delta: u64) {
        self.counters.increment_internal(self.id, delta);
    }

    /// Returns the counter ID for this handle.
    #[inline]
    pub fn id(&self) -> u8 {
        self.id
    }
}

#[cfg(feature = "defmt")]
impl<const N: usize> defmt::Format for CounterHandle<N> {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "CounterHandle(id={})", self.id);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_increment() {
        static COUNTERS: PerformanceCounters<5> = PerformanceCounters::new();

        let handle = COUNTERS.handle(2u8);
        assert_eq!(COUNTERS.read(2u8), 0);

        handle.increment();
        assert_eq!(COUNTERS.read(2u8), 1);

        handle.increment();
        assert_eq!(COUNTERS.read(2u8), 2);
    }

    #[test]
    fn test_add() {
        static COUNTERS: PerformanceCounters<5> = PerformanceCounters::new();

        let handle = COUNTERS.handle(3u8);
        handle.add(10);
        assert_eq!(COUNTERS.read(3u8), 10);

        handle.add(25);
        assert_eq!(COUNTERS.read(3u8), 35);
    }

    #[test]
    fn test_id() {
        static COUNTERS: PerformanceCounters<5> = PerformanceCounters::new();
        let handle = COUNTERS.handle(4u8);
        assert_eq!(handle.id(), 4);
    }
}
