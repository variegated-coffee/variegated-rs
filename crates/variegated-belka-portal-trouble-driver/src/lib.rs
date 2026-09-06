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

/// The notification payload size trouble-host's GATT client hands a listener.
///
/// Taken from trouble rather than written down. This said `512` until trouble-host 0.8,
/// which derives the figure from the packet pool MTU (`ATT_MTU - 3`) instead of hardcoding
/// it -- 248 for the comms firmware's `default-packet-pool-mtu-255`. See the twin in
/// `variegated-scale-trouble-driver` for the longer note.
pub const NOTIF_MTU: usize = trouble_host::config::GATT_CLIENT_NOTIFICATION_MTU;
