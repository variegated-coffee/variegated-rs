#![no_std]

//! Driver for the Belka Portal BLE device
//!
//! This driver provides a high-level interface for interacting with the Belka Portal device,
//! which measures electrical conductivity (EC), temperature, and internal temperature.
//!
//! The driver uses the `variegated-trouble-connection-manager` for connection management,
//! allowing automatic reconnection and connection state tracking.

mod driver;
mod error;
mod types;

pub use driver::{BelkaGattClient, BelkaPortalDriver, MeasurementNotificationStream};
pub use error::Error;
pub use types::{
    Measurements, BELKA_SERVICE_UUID, COMMAND_CHAR_UUID, HIDE_GRAPH, MEASUREMENT_CHAR_UUID,
    SHOW_GRAPH,
};

/// Re-export connection manager types for convenience
pub use variegated_trouble_connection_manager::{
    BdAddr, BleConnectionManager, Controller, DeviceHandle, ManagerHandle, PacketPool,
};
