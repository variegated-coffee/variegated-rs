use crate::indicators::PerformanceIndicators;

/// A lightweight handle for writing to a specific performance indicator.
///
/// This handle provides methods for setting an indicator to a specific value.
/// It's designed to be cheap to clone and pass around, containing only an
/// indicator ID and a static reference.
///
/// # Single Writer Contract
///
/// Each indicator should have only one writer (one location in the code that sets it).
/// This contract is not enforced by the type system but is required for correct operation.
/// Multiple `IndicatorHandle`s can be created for the same indicator ID, but only one should
/// actively write to it.
///
/// # Example
///
/// ```ignore
/// static INDICATORS: PerformanceIndicators<10> = PerformanceIndicators::new();
///
/// #[embassy_executor::task]
/// async fn sensor_task() {
///     let handle = INDICATORS.handle(IndicatorId::Temperature);
///     loop {
///         let temp = read_sensor().await;
///         handle.set(temp as u64);
///         Timer::after_millis(100).await;
///     }
/// }
/// ```
#[derive(Copy, Clone)]
pub struct IndicatorHandle<const N: usize> {
    indicators: &'static PerformanceIndicators<N>,
    id: u8,
}

impl<const N: usize> IndicatorHandle<N> {
    /// Creates a new indicator handle.
    ///
    /// This is typically called via `PerformanceIndicators::handle()` rather than directly.
    #[inline]
    pub(crate) const fn new(indicators: &'static PerformanceIndicators<N>, id: u8) -> Self {
        Self { indicators, id }
    }

    /// Sets the indicator to a specific value.
    ///
    /// This overwrites any previous value stored in the indicator.
    #[inline]
    pub fn set(&self, value: u64) {
        self.indicators.set_internal(self.id, value);
    }

    /// Reads the current value of this indicator.
    ///
    /// This is a convenience method equivalent to calling `indicators.read(id)`.
    #[inline]
    pub fn get(&self) -> u64 {
        self.indicators.read(self.id)
    }

    /// Returns the indicator ID for this handle.
    #[inline]
    pub fn id(&self) -> u8 {
        self.id
    }
}

#[cfg(feature = "defmt")]
impl<const N: usize> defmt::Format for IndicatorHandle<N> {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "IndicatorHandle(id={})", self.id);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_set_and_get() {
        static INDICATORS: PerformanceIndicators<5> = PerformanceIndicators::new();

        let handle = INDICATORS.handle(2u8);
        assert_eq!(handle.get(), 0);

        handle.set(42);
        assert_eq!(handle.get(), 42);

        handle.set(99);
        assert_eq!(handle.get(), 99);
    }

    #[test]
    fn test_multiple_indicators() {
        static INDICATORS: PerformanceIndicators<3> = PerformanceIndicators::new();

        let handle0 = INDICATORS.handle(0u8);
        let handle1 = INDICATORS.handle(1u8);
        let handle2 = INDICATORS.handle(2u8);

        handle0.set(10);
        handle1.set(20);
        handle2.set(30);

        assert_eq!(handle0.get(), 10);
        assert_eq!(handle1.get(), 20);
        assert_eq!(handle2.get(), 30);
    }

    #[test]
    fn test_id() {
        static INDICATORS: PerformanceIndicators<5> = PerformanceIndicators::new();
        let handle = INDICATORS.handle(4u8);
        assert_eq!(handle.id(), 4);
    }
}
