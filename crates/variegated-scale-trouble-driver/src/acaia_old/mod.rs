//! ACAIA Old protocol driver module
//!
//! This module provides a driver for ACAIA scales using the old protocol (pre-2021 models).
//! The old protocol uses standard Bluetooth Weight Scale Service (UUID 0x1820).
//!
//! # Protocol Format
//!
//! Old Acaia protocol uses simpler frames than the new protocol:
//! - Frame format: `[0xEF, 0xDD, weight_lo, weight_hi, ?, ?, scale, sign, ...]`
//! - Fixed frame sizes (typically 10-14 bytes)
//! - No command/length bytes like the new protocol
//!
//! # Connection Handshake
//!
//! Before subscribing to notifications, you must perform the handshake sequence:
//! 1. Subscribe to notifications
//! 2. Wait 150ms
//! 3. Send identification message
//! 4. Send notification request message
//!
//! Use `gatt.initialize()` to perform this sequence automatically.
//!
//! # Events
//!
//! This driver surfaces only weight events:
//! - **Weight**: Weight measurements in grams
//!
//! Note that this is a statement about what the *stream yields*, not about what the scale
//! sends. `NOTIFICATION_REQUEST_MSG` asks for weight, battery, timer and button
//! notifications, and the parser recognises timer and button frames -- it discards them,
//! because nothing downstream consumes them.
//!
//! Commands are a separate matter, and the protocol is not symmetric here: the scale
//! accepts timer control (command `0x0D`, via [`AcaiaOldGattClient::send_timer`]) whether
//! or not anything listens to the notifications it sends back. The timer path in this
//! firmware is write-only throughout for that reason.
//!
//! # Example
//!
//! ```no_run
//! use variegated_scale_trouble_driver::acaia_old::{AcaiaOldDriver, ScaleEvent};
//! use embassy_futures::select::{select, Either};
//!
//! // Create driver from device handle and stack
//! let driver = AcaiaOldDriver::new(device_handle, stack);
//!
//! // Enable automatic reconnection
//! driver.set_maintain_connection(true).await;
//!
//! // Create GATT client
//! let (_conn, gatt) = driver.gatt_client().await?;
//!
//! // Run GATT task alongside operations
//! select(gatt.task(), async {
//!     // Initialize connection and subscribe to events
//!     let mut stream = gatt.initialize().await?;
//!
//!     loop {
//!         match stream.next().await? {
//!             ScaleEvent::Weight(w) => {
//!                 println!("Weight: {} g", w.weight);
//!             }
//!         }
//!     }
//! }).await;
//! ```

mod driver;
mod error;
mod types;

pub use driver::{AcaiaOldDriver, AcaiaOldGattClient, ScaleNotificationStream, TimerOp};
pub use error::Error;
pub use types::{
    ScaleEvent, WeightMeasurement,
    ACAIA_OLD_SERVICE_UUID, ACAIA_OLD_CHAR_UUID,
    IDENTIFICATION_MSG, NOTIFICATION_REQUEST_MSG, TARE_CMD, HEARTBEAT_MSG,
    TIMER_START_CMD, TIMER_STOP_CMD, TIMER_RESET_CMD,
};
