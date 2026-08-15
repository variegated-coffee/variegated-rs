#![no_std]

//! Connection manager for trouble-host BLE stack
//!
//! This crate provides a connection manager that can maintain connections to multiple
//! BLE devices concurrently. It uses a mutex-based architecture for async communication.

mod handle;
mod manager;
mod types;

pub use handle::{DeviceHandle, ManagerHandle};
pub use manager::{BleConnectionManager, BleConnectionManagerShared, ScanRequest, ScanSink};
pub use types::{ConnectionState, DeviceEvent};

/// Re-export trouble-host types for convenience
pub use trouble_host::prelude::{BdAddr, Central, Connection, Controller, Stack};
pub use trouble_host::gatt::GattClient;
pub use trouble_host::PacketPool;
