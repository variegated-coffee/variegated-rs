/// Errors that can occur when using the 2021+ ACAIA driver
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Device is not connected
    NotConnected,
    /// A notification could not be decoded
    ///
    /// Carries the codec's own reason, which distinguishes a frame that is merely
    /// incomplete from one that failed its checksum — the first is normal and the second is
    /// not.
    Parse(variegated_scale_codec::acaia::ParseError),
    /// Handshake sequence failed
    HandshakeFailed,
    /// Failed to write a command to the command characteristic
    WriteFailed,
    /// Failed to subscribe to notifications
    SubscribeFailed,
    /// GATT operation failed
    GattError,
    /// Service not found during discovery
    ///
    /// The first thing to suspect is the 128-bit UUID byte order; see
    /// [`crate::acaia_new::ACAIA_NEW_SERVICE_UUID`].
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
            Error::Parse(_) => write!(f, "Failed to parse notification"),
            Error::HandshakeFailed => write!(f, "Handshake sequence failed"),
            Error::WriteFailed => write!(f, "Failed to write command"),
            Error::SubscribeFailed => write!(f, "Failed to subscribe to notifications"),
            Error::GattError => write!(f, "GATT operation failed"),
            Error::ServiceNotFound => write!(f, "Service not found"),
            Error::CharacteristicNotFound => write!(f, "Characteristic not found"),
            Error::BufferOverflow => write!(f, "Buffer overflow"),
        }
    }
}

impl From<variegated_scale_codec::acaia::ParseError> for Error {
    fn from(e: variegated_scale_codec::acaia::ParseError) -> Self {
        Error::Parse(e)
    }
}
