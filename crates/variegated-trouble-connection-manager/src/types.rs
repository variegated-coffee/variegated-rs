/// Events sent from the connection manager to device handles (future use)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DeviceEvent {
    /// Device has successfully connected
    Connected,
    /// Device has disconnected
    Disconnected,
    /// Connection attempt failed
    ConnectionFailed,
}

/// Connection state for a device
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ConnectionState {
    /// Device is not connected
    Disconnected,
    /// Device is currently connected
    Connected,
    /// Connection is in progress
    Connecting,
}
