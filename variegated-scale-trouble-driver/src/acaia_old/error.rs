/// Errors that can occur when using the ACAIA Old protocol driver
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Device is not connected
    NotConnected,
    /// Handshake sequence failed
    HandshakeFailed,
    /// Invalid frame length received
    InvalidFrameLength,
    /// Failed to parse event data
    ParseError,
    /// Failed to write command to characteristic
    WriteFailed,
    /// Failed to subscribe to notifications
    SubscribeFailed,
    /// GATT operation failed
    GattError,
    /// Service not found during discovery
    ServiceNotFound,
    /// Characteristic not found during discovery
    CharacteristicNotFound,
    /// Buffer overflow when accumulating fragmented messages
    BufferOverflow,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::NotConnected => write!(f, "Device is not connected"),
            Error::HandshakeFailed => write!(f, "Handshake sequence failed"),
            Error::InvalidFrameLength => write!(f, "Invalid frame length"),
            Error::ParseError => write!(f, "Failed to parse event data"),
            Error::WriteFailed => write!(f, "Failed to write command"),
            Error::SubscribeFailed => write!(f, "Failed to subscribe to notifications"),
            Error::GattError => write!(f, "GATT operation failed"),
            Error::ServiceNotFound => write!(f, "Service not found"),
            Error::CharacteristicNotFound => write!(f, "Characteristic not found"),
            Error::BufferOverflow => write!(f, "Buffer overflow"),
        }
    }
}
