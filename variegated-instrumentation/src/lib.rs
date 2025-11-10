#![no_std]
#![doc = include_str!("../README.md")]

//! # Variegated Instrumentation
//!
//! A lightweight performance instrumentation library for embedded systems.
//!
//! ## Features
//!
//! - **Zero-cost when disabled**: All instrumentation compiles to nothing without the `instrumentation` feature
//! - **Lock-free counters and indicators**: Uses atomic operations for minimal overhead
//! - **Type-safe IDs**: Counter and indicator IDs defined via enums with `Into<u8>` implementation
//! - **No allocation**: All storage is static and compile-time sized
//! - **Single writer per metric**: Each counter/indicator is written by one task, readable from anywhere
//!
//! ## Performance Counters
//!
//! Counters track events by incrementing. Ideal for counting occurrences (loop iterations, errors, etc.).
//!
//! ```ignore
//! use variegated_instrumentation::{PerformanceCounters, define_counters};
//!
//! define_counters! {
//!     enum CounterId {
//!         LoopIterations = 0,
//!         SensorReads = 1,
//!     }
//! }
//!
//! static COUNTERS: PerformanceCounters<2> = PerformanceCounters::new();
//!
//! #[embassy_executor::task]
//! async fn my_task() {
//!     let handle = COUNTERS.handle(CounterId::LoopIterations);
//!     loop {
//!         handle.increment();
//!         // ... work ...
//!     }
//! }
//! ```
//!
//! ## Performance Indicators
//!
//! Indicators track current state by setting values. Ideal for measurements (temperature, pressure, etc.).
//!
//! ```ignore
//! use variegated_instrumentation::{PerformanceIndicators, define_indicators};
//!
//! define_indicators! {
//!     enum IndicatorId {
//!         BoilerTemp = 0,
//!         Pressure = 1,
//!     }
//! }
//!
//! static INDICATORS: PerformanceIndicators<2> = PerformanceIndicators::new();
//!
//! #[embassy_executor::task]
//! async fn sensor_task() {
//!     let handle = INDICATORS.handle(IndicatorId::BoilerTemp);
//!     loop {
//!         let temp = read_sensor().await;
//!         handle.set(temp as u64);
//!     }
//! }
//! ```

// Performance counter implementation (enabled with instrumentation feature)
#[cfg(feature = "instrumentation")]
mod counters;
#[cfg(feature = "instrumentation")]
mod handle;

// Performance indicator implementation (enabled with instrumentation feature)
#[cfg(feature = "instrumentation")]
mod indicators;
#[cfg(feature = "instrumentation")]
mod indicator_handle;

// No-op implementation (default, zero overhead)
#[cfg(not(feature = "instrumentation"))]
mod noop;

// Macros are always available
mod macros;

// Public API exports - Counters
#[cfg(feature = "instrumentation")]
pub use counters::PerformanceCounters;
#[cfg(feature = "instrumentation")]
pub use handle::CounterHandle;

#[cfg(not(feature = "instrumentation"))]
pub use noop::{CounterHandle, PerformanceCounters};

// Public API exports - Indicators
#[cfg(feature = "instrumentation")]
pub use indicator_handle::IndicatorHandle;
#[cfg(feature = "instrumentation")]
pub use indicators::PerformanceIndicators;

#[cfg(not(feature = "instrumentation"))]
pub use noop::{IndicatorHandle, PerformanceIndicators};

// Legacy timing macros (kept for backwards compatibility)

#[macro_export]
macro_rules! async_task_loop {
    ($name:expr, $delay:expr, $body:block) => {
        {
            let mut last_log_time = ::embassy_time::Instant::now();

            loop {
                // $name loop

                // Capture start time
                let start_time = ::embassy_time::Instant::now();

                $body

                // Calculate execution time
                let elapsed = start_time.elapsed();

                // Only log if at least 1 second has passed since last log
                let now = ::embassy_time::Instant::now();
                if now.duration_since(last_log_time).as_secs() >= 1 {
                    //::defmt::debug!("{} loop: {} ms", $name, elapsed.as_millis());
                    last_log_time = now;
                }

                // Delay at the bottom of the loop
                if let Some(delay_duration) = $delay {
                    ::embassy_time::Timer::after(delay_duration).await;
                }
            }
        }
    };
}

#[macro_export]
macro_rules! instrumented_section {
    ($name:expr, $body:block) => {
        {
            let start_time = ::embassy_time::Instant::now();

            let foo = $body;

            let elapsed = start_time.elapsed();
            // ::defmt::debug!("{} section: {} ms", $name, elapsed.as_millis());

            foo
        }
    };
}