#![no_std]

//! Driver for Bluetooth coffee scales
//!
//! This crate provides drivers for Bluetooth-enabled coffee scales.
//!
//! See SCALE_PROTOCOLS.md for protocol documentation. Frame parsing and command building
//! live in `variegated-scale-codec`, a crate of their own because this one cannot host a
//! test binary; the drivers here are the GATT plumbing around it.
//!
//! ## Available Drivers
//!
//! - [`acaia_old`] - ACAIA scales with the pre-2021 protocol (Lunar AL010, Pearl 2015)
//! - [`acaia_new`] - ACAIA scales from 2021 (Pyxis, Lunar AL014+, Pearl S, Cinco)
//! - [`bookoo`] - BooKoo Themis, Themis Mini and Themis Ultra
//!
//! The two ACAIA modules share their commands, which are byte-identical across the
//! generations; what differs is GATT topology and incoming framing. Felicita is documented
//! in SCALE_PROTOCOLS.md but not implemented.
//!
//! ## Example
//!
//! ```no_run
//! use variegated_scale_trouble_driver::acaia_old::{AcaiaOldDriver, ScaleEvent};
//! use embassy_futures::select::select;
//!
//! // Create driver and enable auto-reconnect
//! let driver = AcaiaOldDriver::new(device_handle, stack);
//! driver.set_maintain_connection(true).await;
//!
//! // Create GATT client and subscribe to events
//! let (_conn, mut gatt) = driver.gatt_client().await?;
//! select(gatt.task(), async {
//!     gatt.perform_handshake().await?;
//!     let mut stream = gatt.subscribe().await?;
//!     while let Ok(event) = stream.next().await {
//!         // Handle event
//!     }
//! }).await;
//! ```

pub mod acaia_new;
pub mod acaia_old;
pub mod bookoo;

/// Re-export trouble-host types for convenience
pub use variegated_trouble_connection_manager::{BdAddr, Connection, Controller, Stack};
pub use variegated_trouble_connection_manager::{DeviceHandle, GattClient, PacketPool};

/// The notification payload size trouble-host's GATT client hands a listener.
///
/// **Taken from trouble rather than written down**, which is the whole point of this
/// constant. Every `NotificationListener` in this crate used to say `512`, and that was
/// correct only because trouble-host 0.7 hardcoded `Notification<512>` regardless of the
/// packet pool. 0.8 derives it -- `GATT_CLIENT_NOTIFICATION_MTU` is the pool MTU less 7,
/// i.e. `ATT_MTU - 3` -- so with the comms firmware's `default-packet-pool-mtu-255` it is
/// 248, and all six sites stopped compiling at once.
///
/// That was the good outcome: a mismatch here is a type error rather than a silent
/// truncation of a notification payload. Referring to the constant keeps that property
/// while making the next MTU change a rebuild instead of an edit.
pub const NOTIF_MTU: usize = trouble_host::config::GATT_CLIENT_NOTIFICATION_MTU;
