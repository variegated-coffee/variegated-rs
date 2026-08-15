/// Errors that can occur when using the Belka Portal driver
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Device is not connected
    NotConnected,
    /// Service not found during discovery
    ServiceNotFound,
    /// Characteristic not found during discovery
    CharacteristicNotFound,
    /// Failed to read from characteristic
    ReadFailed,
    /// Failed to subscribe to notifications
    SubscribeFailed,
    /// Invalid data length received
    InvalidDataLength,
    /// Failed to parse measurement data
    ParseError,
    /// GATT operation failed
    GattError,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::NotConnected => write!(f, "Device is not connected"),
            Error::ServiceNotFound => write!(f, "Service not found"),
            Error::CharacteristicNotFound => write!(f, "Characteristic not found"),
            Error::ReadFailed => write!(f, "Failed to read from characteristic"),
            Error::SubscribeFailed => write!(f, "Failed to subscribe to notifications"),
            Error::InvalidDataLength => write!(f, "Invalid data length"),
            Error::ParseError => write!(f, "Failed to parse measurement data"),
            Error::GattError => write!(f, "GATT operation failed"),
        }
    }
}
