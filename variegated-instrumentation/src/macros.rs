/// Defines a counter ID enum with automatic `From<Enum> for u8` implementation.
///
/// This macro simplifies the creation of type-safe counter IDs. It generates
/// a `#[repr(u8)]` enum and implements the `From` trait to convert enum variants
/// to u8 values.
///
/// # Example
///
/// ```ignore
/// use variegated_instrumentation::define_counters;
///
/// define_counters! {
///     enum CounterId {
///         LoopIterations = 0,
///         SensorReads = 1,
///         ControlUpdates = 2,
///     }
/// }
///
/// // Now you can use CounterId values as counter IDs
/// static PERF: PerformanceCounters<3> = PerformanceCounters::new();
/// let handle = PERF.handle(CounterId::LoopIterations);
/// ```
///
/// # Generated Code
///
/// The macro expands to:
/// - A `#[repr(u8)]` enum with the specified variants
/// - A `#[derive(Copy, Clone, Debug)]` attribute
/// - An implementation of `From<EnumName> for u8`
#[macro_export]
macro_rules! define_counters {
    (
        $(#[$enum_attr:meta])*
        enum $name:ident {
            $(
                $(#[$variant_attr:meta])*
                $variant:ident = $value:expr
            ),* $(,)?
        }
    ) => {
        $(#[$enum_attr])*
        #[derive(Copy, Clone, Debug)]
        #[repr(u8)]
        enum $name {
            $(
                $(#[$variant_attr])*
                $variant = $value,
            )*
        }

        impl From<$name> for u8 {
            #[inline]
            fn from(id: $name) -> u8 {
                id as u8
            }
        }
    };
}

/// Increments a performance counter (convenience macro).
///
/// This macro provides a shorter syntax for incrementing counters when
/// instrumentation is enabled. When the `instrumentation` feature is disabled,
/// this macro expands to nothing.
///
/// # Example
///
/// ```ignore
/// use variegated_instrumentation::count;
///
/// static PERF: PerformanceCounters<3> = PerformanceCounters::new();
///
/// #[embassy_executor::task]
/// async fn my_task() {
///     let handle = PERF.handle(CounterId::LoopIterations);
///     loop {
///         count!(handle);
///         // ... do work ...
///     }
/// }
/// ```
#[cfg(feature = "instrumentation")]
#[macro_export]
macro_rules! count {
    ($handle:expr) => {
        $handle.increment()
    };
    ($handle:expr, $delta:expr) => {
        $handle.add($delta)
    };
}

/// No-op version of count! macro when instrumentation is disabled.
#[cfg(not(feature = "instrumentation"))]
#[macro_export]
macro_rules! count {
    ($handle:expr) => {{}};
    ($handle:expr, $delta:expr) => {{}};
}

/// Defines an indicator ID enum with automatic `From<Enum> for u8` implementation.
///
/// This macro simplifies the creation of type-safe indicator IDs. It generates
/// a `#[repr(u8)]` enum and implements the `From` trait to convert enum variants
/// to u8 values.
///
/// # Example
///
/// ```ignore
/// use variegated_instrumentation::define_indicators;
///
/// define_indicators! {
///     enum IndicatorId {
///         BoilerTemp = 0,
///         GroupPressure = 1,
///         FlowRate = 2,
///     }
/// }
///
/// // Now you can use IndicatorId values as indicator IDs
/// static INDICATORS: PerformanceIndicators<3> = PerformanceIndicators::new();
/// let handle = INDICATORS.handle(IndicatorId::BoilerTemp);
/// ```
///
/// # Generated Code
///
/// The macro expands to:
/// - A `#[repr(u8)]` enum with the specified variants
/// - A `#[derive(Copy, Clone, Debug)]` attribute
/// - An implementation of `From<EnumName> for u8`
#[macro_export]
macro_rules! define_indicators {
    (
        $(#[$enum_attr:meta])*
        enum $name:ident {
            $(
                $(#[$variant_attr:meta])*
                $variant:ident = $value:expr
            ),* $(,)?
        }
    ) => {
        $(#[$enum_attr])*
        #[derive(Copy, Clone, Debug)]
        #[repr(u8)]
        enum $name {
            $(
                $(#[$variant_attr])*
                $variant = $value,
            )*
        }

        impl From<$name> for u8 {
            #[inline]
            fn from(id: $name) -> u8 {
                id as u8
            }
        }
    };
}

/// Sets a performance indicator value (convenience macro).
///
/// This macro provides a shorter syntax for setting indicators when
/// instrumentation is enabled. When the `instrumentation` feature is disabled,
/// this macro expands to nothing.
///
/// # Example
///
/// ```ignore
/// use variegated_instrumentation::indicate;
///
/// static INDICATORS: PerformanceIndicators<3> = PerformanceIndicators::new();
///
/// #[embassy_executor::task]
/// async fn sensor_task() {
///     let handle = INDICATORS.handle(IndicatorId::Temperature);
///     loop {
///         let temp = read_sensor().await;
///         indicate!(handle, temp);
///         Timer::after_millis(100).await;
///     }
/// }
/// ```
#[cfg(feature = "instrumentation")]
#[macro_export]
macro_rules! indicate {
    ($handle:expr, $value:expr) => {
        $handle.set($value)
    };
}

/// No-op version of indicate! macro when instrumentation is disabled.
#[cfg(not(feature = "instrumentation"))]
#[macro_export]
macro_rules! indicate {
    ($handle:expr, $value:expr) => {{}};
}
