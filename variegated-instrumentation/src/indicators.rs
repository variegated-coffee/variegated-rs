use portable_atomic::{AtomicU64, Ordering};

use crate::indicator_handle::IndicatorHandle;

/// Central registry for performance indicators.
///
/// This type holds N indicators, each identified by a u8 ID (0..N-1).
/// Each indicator is backed by an AtomicU64, allowing lock-free reads and writes.
///
/// Unlike counters which are incremented, indicators are set to specific values,
/// making them ideal for tracking current state (temperature, pressure, queue depth, etc.).
///
/// # Usage
///
/// Typically, you'll create a single static instance of this type:
///
/// ```ignore
/// static INDICATORS: PerformanceIndicators<10> = PerformanceIndicators::new();
/// ```
///
/// Then obtain handles for writing and read indicators directly:
///
/// ```ignore
/// let handle = INDICATORS.handle(IndicatorId::BoilerTemp);
/// handle.set(95);
///
/// let value = INDICATORS.read(IndicatorId::BoilerTemp);
/// ```
pub struct PerformanceIndicators<const N: usize> {
    indicators: [AtomicU64; N],
}

impl<const N: usize> PerformanceIndicators<N> {
    /// Creates a new performance indicator registry with all indicators initialized to zero.
    ///
    /// This is a const function, so it can be used in static initializers.
    #[inline]
    pub const fn new() -> Self {
        const INIT: AtomicU64 = AtomicU64::new(0);
        Self {
            indicators: [INIT; N],
        }
    }

    /// Obtains a handle for writing to a specific indicator.
    ///
    /// The handle provides methods for setting the indicator value.
    ///
    /// # Panics
    ///
    /// Panics if the indicator ID is out of bounds (>= N).
    ///
    /// # Safety Contract
    ///
    /// The caller must ensure that only one handle per indicator ID is actively
    /// used for writing. Multiple handles can be created, but only one should
    /// perform writes to maintain the single-writer guarantee.
    #[inline]
    pub fn handle(&'static self, id: impl Into<u8>) -> IndicatorHandle<N> {
        let id = id.into();
        assert!(
            (id as usize) < N,
            "Indicator ID {} out of bounds (max {})",
            id,
            N - 1
        );
        IndicatorHandle::new(self, id)
    }

    /// Reads the current value of an indicator.
    ///
    /// This operation uses `Acquire` ordering to ensure that all writes
    /// to the indicator are visible.
    ///
    /// # Panics
    ///
    /// Panics if the indicator ID is out of bounds (>= N).
    #[inline]
    pub fn read(&self, id: impl Into<u8>) -> u64 {
        let id = id.into();
        assert!(
            (id as usize) < N,
            "Indicator ID {} out of bounds (max {})",
            id,
            N - 1
        );
        self.indicators[id as usize].load(Ordering::Acquire)
    }

    /// Reads all indicators and returns them as an array.
    ///
    /// This provides a consistent snapshot of all indicator values at the time of the call.
    #[inline]
    pub fn read_all(&self) -> [u64; N] {
        let mut result = [0u64; N];
        for (i, indicator) in self.indicators.iter().enumerate() {
            result[i] = indicator.load(Ordering::Acquire);
        }
        result
    }

    /// Internal method used by IndicatorHandle to set an indicator value.
    ///
    /// Uses `Relaxed` ordering since we assume single-writer-per-indicator.
    #[inline]
    pub(crate) fn set_internal(&self, id: u8, value: u64) {
        self.indicators[id as usize].store(value, Ordering::Relaxed);
    }
}

#[cfg(feature = "defmt")]
impl<const N: usize> defmt::Format for PerformanceIndicators<N> {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "PerformanceIndicators[{}](", N);
        for (i, indicator) in self.indicators.iter().enumerate() {
            if i > 0 {
                defmt::write!(fmt, ", ");
            }
            defmt::write!(fmt, "{}", indicator.load(Ordering::Acquire));
        }
        defmt::write!(fmt, ")");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new() {
        let indicators: PerformanceIndicators<5> = PerformanceIndicators::new();
        for i in 0..5 {
            assert_eq!(indicators.read(i as u8), 0);
        }
    }

    #[test]
    fn test_set_and_read() {
        let indicators: PerformanceIndicators<3> = PerformanceIndicators::new();
        indicators.set_internal(0, 10);
        indicators.set_internal(1, 20);
        indicators.set_internal(2, 30);

        assert_eq!(indicators.read(0u8), 10);
        assert_eq!(indicators.read(1u8), 20);
        assert_eq!(indicators.read(2u8), 30);
    }

    #[test]
    fn test_read_all() {
        let indicators: PerformanceIndicators<3> = PerformanceIndicators::new();
        indicators.set_internal(0, 100);
        indicators.set_internal(1, 200);
        indicators.set_internal(2, 300);

        let all = indicators.read_all();
        assert_eq!(all, [100, 200, 300]);
    }

    #[test]
    fn test_overwrite() {
        let indicators: PerformanceIndicators<1> = PerformanceIndicators::new();
        indicators.set_internal(0, 42);
        assert_eq!(indicators.read(0u8), 42);

        indicators.set_internal(0, 99);
        assert_eq!(indicators.read(0u8), 99);
    }

    #[test]
    #[should_panic(expected = "Indicator ID 5 out of bounds")]
    fn test_read_out_of_bounds() {
        let indicators: PerformanceIndicators<5> = PerformanceIndicators::new();
        let _ = indicators.read(5u8);
    }
}
