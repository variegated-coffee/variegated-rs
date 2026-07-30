/// Shared expansion behind `define_counters!` and `define_indicators!`.
///
/// Not part of the public API: it is `#[macro_export]`ed only because a macro
/// invoked from another crate's expansion of `define_counters!` must be reachable
/// as `$crate::__variegated_define_metric_ids!`.
#[doc(hidden)]
#[macro_export]
macro_rules! __variegated_define_metric_ids {
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

        impl $name {
            /// Variant names in id order. Lets a debug transport send `MetricName`
            /// payloads without a hand-maintained table.
            ///
            /// This is index-ordered, not value-ordered: it assumes the declaration
            /// assigns ids `0..n` in order, as every call site does.
            pub const NAMES: &'static [&'static str] = &[
                $(stringify!($variant),)*
            ];

            /// Number of declared ids -- the `N` a `PerformanceCounters<N>` or
            /// `PerformanceIndicators<N>` should be sized to.
            pub const COUNT: usize = $name::NAMES.len();
        }
    };
}

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
/// - `$name::NAMES`, the variant names in declaration order, and `$name::COUNT`,
///   the number of variants -- both assume the declaration assigns ids `0..n` in
///   order, as every call site does.
#[macro_export]
macro_rules! define_counters {
    ($($tokens:tt)*) => { $crate::__variegated_define_metric_ids! { $($tokens)* } };
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
/// - `$name::NAMES`, the variant names in declaration order, and `$name::COUNT`,
///   the number of variants -- both assume the declaration assigns ids `0..n` in
///   order, as every call site does.
#[macro_export]
macro_rules! define_indicators {
    ($($tokens:tt)*) => { $crate::__variegated_define_metric_ids! { $($tokens)* } };
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

#[cfg(test)]
mod tests {
    define_counters! {
        enum TestCounterId {
            LoopIterations = 0,
            SensorReads = 1,
        }
    }

    define_indicators! {
        enum TestIndicatorId {
            BoilerTemp = 0,
        }
    }

    #[test]
    fn counter_names_follow_declaration_order() {
        assert_eq!(TestCounterId::NAMES, &["LoopIterations", "SensorReads"]);
        assert_eq!(TestCounterId::COUNT, 2);
    }

    #[test]
    fn indicator_names_are_generated_too() {
        assert_eq!(TestIndicatorId::NAMES, &["BoilerTemp"]);
        assert_eq!(TestIndicatorId::COUNT, 1);
    }

    #[test]
    fn ids_still_convert_to_u8() {
        assert_eq!(u8::from(TestCounterId::LoopIterations), 0);
        assert_eq!(u8::from(TestCounterId::SensorReads), 1);
        assert_eq!(u8::from(TestIndicatorId::BoilerTemp), 0);
    }
}
