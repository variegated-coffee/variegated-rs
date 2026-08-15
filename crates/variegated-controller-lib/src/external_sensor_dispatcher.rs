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

/// A dispatcher for a machine with no externally-driven sensors.
///
/// `esp_transceiver_main` is generic over the dispatcher and takes an `Option` of one, but
/// a type still has to be named for the `None` -- so a board with nothing to dispatch to
/// needs a type that does nothing. Every such board would otherwise declare its own, and
/// one already had.
pub struct NoopDispatcher;

impl ExternalSensorDispatcher for NoopDispatcher {
    fn dispatch_reading(&self, _reading: &ExternalPeripheralSensorReading) {}
    fn dispatch_connection_status(&self, _peripheral_id: PeripheralId, _connected: bool) {}
}
