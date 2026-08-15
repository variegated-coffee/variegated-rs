//! defmt, when it is compiled in.
//!
//! The sibling trouble crates in this workspace take `defmt` as a hard dependency and call
//! `defmt::info!` directly. This crate cannot: `defmt` is optional here so the host test
//! suite builds without it, and `ble` does not imply it. These shims are the same trick
//! `trouble-host`'s own `fmt.rs` uses -- with the feature off they expand to nothing and
//! the arguments are never evaluated, which is also what `defmt`'s own macros promise.
//!
//! # Nothing routed through here may carry a password
//!
//! These reach the USB-Serial-JTAG and, on this firmware, the debug bus behind it. Log
//! decoded command *names* and byte *counts*, never packet contents -- a `WIFI_SETTINGS`
//! frame is mostly credential.

macro_rules! info {
    ($($arg:tt)*) => {{
        #[cfg(feature = "defmt")]
        ::defmt::info!($($arg)*);
    }};
}

macro_rules! warn_ {
    ($($arg:tt)*) => {{
        #[cfg(feature = "defmt")]
        ::defmt::warn!($($arg)*);
    }};
}

pub(crate) use {info, warn_};
