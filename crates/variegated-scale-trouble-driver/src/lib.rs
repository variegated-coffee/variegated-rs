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
//! - [`acaia_old`] - ACAIA scales with old protocol (pre-2021 models)
//! - [`bookoo`] - BooKoo Themis, Themis Mini and Themis Ultra
//!
//! ACAIA's newer protocol and Felicita are documented in SCALE_PROTOCOLS.md but not
//! implemented.
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

pub mod acaia_old;
pub mod bookoo;

/// Re-export trouble-host types for convenience
pub use variegated_trouble_connection_manager::{BdAddr, Connection, Controller, Stack};
pub use variegated_trouble_connection_manager::{DeviceHandle, GattClient, PacketPool};
