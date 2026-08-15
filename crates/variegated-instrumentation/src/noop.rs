/// No-op implementation of PerformanceCounters when instrumentation is disabled.
///
/// This type has the same API as the real `PerformanceCounters` but all operations
/// compile to nothing, ensuring zero runtime overhead when instrumentation is disabled.
pub struct PerformanceCounters<const N: usize>;

impl<const N: usize> PerformanceCounters<N> {
    /// Creates a new no-op performance counter registry.
    #[inline]
    pub const fn new() -> Self {
        Self
    }

    /// Returns a no-op handle that does nothing when incremented.
    #[inline]
    pub fn handle(&'static self, _id: impl Into<u8>) -> CounterHandle<N> {
        CounterHandle { _phantom: () }
    }

    /// Returns zero (counter is not tracked).
    #[inline]
    pub fn read(&self, _id: impl Into<u8>) -> u64 {
        0
    }

    /// Returns an array of zeros (counters are not tracked).
    #[inline]
    pub fn read_all(&self) -> [u64; N] {
        [0u64; N]
    }
}

/// No-op implementation of CounterHandle when instrumentation is disabled.
///
/// This type has the same API as the real `CounterHandle` but all operations
/// compile to nothing.
#[derive(Copy, Clone)]
pub struct CounterHandle<const N: usize> {
    _phantom: (),
}

impl<const N: usize> CounterHandle<N> {
    /// Does nothing (no-op).
    #[inline]
    pub fn increment(&self) {}

    /// Does nothing (no-op).
    #[inline]
    pub fn add(&self, _delta: u64) {}

    /// Returns 0 (counter ID is not tracked).
    #[inline]
    pub fn id(&self) -> u8 {
        0
    }
}

/// No-op implementation of PerformanceIndicators when instrumentation is disabled.
///
/// This type has the same API as the real `PerformanceIndicators` but all operations
/// compile to nothing, ensuring zero runtime overhead when instrumentation is disabled.
pub struct PerformanceIndicators<const N: usize>;

impl<const N: usize> PerformanceIndicators<N> {
    /// Creates a new no-op performance indicator registry.
    #[inline]
    pub const fn new() -> Self {
        Self
    }

    /// Returns a no-op handle that does nothing when set.
    #[inline]
    pub fn handle(&'static self, _id: impl Into<u8>) -> IndicatorHandle<N> {
        IndicatorHandle { _phantom: () }
    }

    /// Returns zero (indicator is not tracked).
    #[inline]
    pub fn read(&self, _id: impl Into<u8>) -> u64 {
        0
    }

    /// Returns an array of zeros (indicators are not tracked).
    #[inline]
    pub fn read_all(&self) -> [u64; N] {
        [0u64; N]
    }
}

/// No-op implementation of IndicatorHandle when instrumentation is disabled.
///
/// This type has the same API as the real `IndicatorHandle` but all operations
/// compile to nothing.
#[derive(Copy, Clone)]
pub struct IndicatorHandle<const N: usize> {
    _phantom: (),
}

impl<const N: usize> IndicatorHandle<N> {
    /// Does nothing (no-op).
    #[inline]
    pub fn set(&self, _value: u64) {}

    /// Returns 0 (indicator is not tracked).
    #[inline]
    pub fn get(&self) -> u64 {
        0
    }

    /// Returns 0 (indicator ID is not tracked).
    #[inline]
    pub fn id(&self) -> u8 {
        0
    }
}
