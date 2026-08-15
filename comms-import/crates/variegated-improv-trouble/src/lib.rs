#![no_std]
//! Improv Wi-Fi provisioning over BLE.
//!
//! <https://www.improv-wifi.com/ble/>
//!
//! This crate is the protocol, not the policy. It decodes and encodes Improv packets and
//! names the protocol's states; it does not decide whether provisioning is allowed, when to
//! advertise, or what to do with credentials. On this machine those are the application
//! processor's decisions and they arrive over the inter-processor link.
//!
//! # Nothing here may log a password
//!
//! [`codec::WifiSettings`] carries one. It has a hand-written `Debug`/`Format` that prints
//! the SSID and elides the password, and that is not decoration -- a derived impl on this
//! type would put a live Wi-Fi password into the debug stream, the TCP debug server and any
//! log a user pastes into an issue.

#[cfg(feature = "ble")]
mod fmt;

pub mod codec;
pub mod handler;

/// The BLE half: the GATT service, the advertisement and the loop that serves both.
///
/// Behind `ble` so the host test suite -- which has no `embassy-time` driver and no
/// `critical-section` implementation -- keeps building. See the manifest, which also explains
/// why the `#[gatt_*]` macros are invoked in this crate rather than in the firmware.
#[cfg(feature = "ble")]
pub mod service;
