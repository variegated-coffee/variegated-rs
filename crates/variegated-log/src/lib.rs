#![no_std]
//! The `log_*!` macros and, optionally, the two bridges between them and the
//! structured debug stream.
//!
//! The macros and [`bus_sink`] are chip-agnostic and are used by both firmwares:
//! the RP2350 application processor and the ESP32-C6 comms processor. Everything
//! RP-specific -- [`logger_task`] and the `defmt-rtt` transport -- sits behind the
//! `rp-usb-logger` feature, which the `rp2040` / `rp235xa` / `rp235xb` chip
//! features turn on. Consumers name a chip as they always did; the gate exists so
//! that *not* naming one produces a crate that builds for riscv32.

#[cfg(feature = "rp-usb-logger")]
use embassy_rp::peripherals::USB;
#[cfg(feature = "rp-usb-logger")]
use embassy_rp::usb::Driver;

#[cfg(feature = "debug-bus")]
pub mod bus_sink;

use variegated_controller_types::debug::DebugEvent;

/// Publish a typed `DebugEvent` on the debug bus, if this build has the bridge
/// compiled in.
///
/// This exists so `variegated-hal` and `variegated-controller-lib` can emit
/// structured events without depending on `variegated-debug` themselves.
/// `variegated-debug` `compile_error!`s until a `source-application` /
/// `source-comms` feature is selected, and that is a choice a firmware binary
/// makes -- forcing it on every consumer of a HAL would be a debug feature
/// dictating a library's public build requirements.
///
/// With `debug-bus` off this is a no-op taking its argument by value, so the
/// event construction at the call site optimises away and the call sites need no
/// `#[cfg]` of their own.
#[cfg(feature = "debug-bus")]
pub fn emit_event(event: DebugEvent) {
    variegated_debug::bus::emit_event(event);
}

#[cfg(not(feature = "debug-bus"))]
pub fn emit_event(_event: DebugEvent) {}

// The defmt half of the dual-emit, split out so it can be switched off.
//
// **The `#[cfg]` has to be here rather than inside `log_info!` and friends.** A `#[cfg]`
// written in a `macro_rules!` body is evaluated where the macro is *expanded* -- in the
// consumer's crate, against the consumer's features -- so `#[cfg(feature = "defmt")]`
// inside `log_info!` would silently test whether the *caller* has a feature by that name.
// Defining a pair of helper macros here, one of which expands to nothing, resolves the
// condition while this crate is being compiled, which is the only place it means what it
// says.
//
// Why it can be switched off at all: with it on, any host build that expands one of these
// macros monomorphizes a `defmt::Format` impl, and defmt emits `extern "Rust"` references
// to `_defmt_acquire`/`_defmt_write`/`_defmt_timestamp` with no provider. The link then
// fails as "Too many sections!", which mentions neither defmt nor logging. That is what
// kept `variegated-controller-lib` -- 300-odd `log_*!` call sites -- off the host, and
// therefore untested.
//
// `defmt` is a **default** feature, so every consumer that does not opt out keeps the
// dual-emit exactly as before. The one consumer that passes `default-features = false` is
// the comms firmware, and it names `defmt` explicitly: its defmt output goes to
// `esp-println`, and blinding it was a real bug that took a day to find. See the block
// beside its dependency line.
#[cfg(feature = "defmt")]
#[macro_export]
#[doc(hidden)]
macro_rules! __defmt_emit {
    ($level:ident, $($arg:tt)*) => { defmt::$level!($($arg)*) };
}

#[cfg(not(feature = "defmt"))]
#[macro_export]
#[doc(hidden)]
macro_rules! __defmt_emit {
    ($level:ident, $($arg:tt)*) => {};
}

#[macro_export]
macro_rules! log_error {
    ($($arg:tt)*) => {
        {
            log::error!($($arg)*);
            $crate::__defmt_emit!(error, $($arg)*);
        }
    };
}

#[macro_export]
macro_rules! log_warn {
    ($($arg:tt)*) => {
        {
            log::warn!($($arg)*);
            $crate::__defmt_emit!(warn, $($arg)*);
        }
    };
}

#[macro_export]
macro_rules! log_info {
    ($($arg:tt)*) => {
        {
            log::info!($($arg)*);
            $crate::__defmt_emit!(info, $($arg)*);
        }
    };
}

#[macro_export]
macro_rules! log_debug {
    ($($arg:tt)*) => {
        {
            log::debug!($($arg)*);
            $crate::__defmt_emit!(debug, $($arg)*);
        }
    };
}

#[macro_export]
macro_rules! log_trace {
    ($($arg:tt)*) => {
        {
            log::trace!($($arg)*);
            $crate::__defmt_emit!(trace, $($arg)*);
        }
    };
}

/// The RP2350's USB serial `log` sink. Unrelated to [`bus_sink`], which both
/// firmwares use; this one is the CDC console on the application processor.
#[cfg(feature = "rp-usb-logger")]
#[embassy_executor::task]
pub async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}
