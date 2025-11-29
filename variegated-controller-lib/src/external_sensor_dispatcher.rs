use variegated_controller_types::{ExternalPeripheralSensorReading, PeripheralId};

/// Trait for dispatching external sensor readings to device-specific handlers.
///
/// Implementations of this trait receive sensor readings and connection status
/// updates from the comms layer and dispatch them to the appropriate device
/// controllers (e.g., BelkaDevice).
pub trait ExternalSensorDispatcher: Send + Sync {
    /// Dispatch a sensor reading to the appropriate handler.
    fn dispatch_reading(&self, reading: &ExternalPeripheralSensorReading);

    /// Dispatch a connection status change to the appropriate handler.
    fn dispatch_connection_status(&self, peripheral_id: PeripheralId, connected: bool);
}
