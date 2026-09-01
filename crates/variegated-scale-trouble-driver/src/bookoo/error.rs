/// Errors that can occur when using the BooKoo Themis driver
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Device is not connected
    NotConnected,
    /// A notification could not be decoded
    ///
    /// Carries the codec's own reason, which distinguishes a frame from another device
    /// (`Product`) from one that is genuinely corrupt (`Checksum`) -- a distinction worth
    /// keeping, because the first is normal on a busy radio and the second is not.
    Parse(variegated_scale_codec::bookoo::ParseError),
    /// Failed to write a command to the command characteristic
    WriteFailed,
    /// Failed to subscribe to notifications
    SubscribeFailed,
    /// GATT operation failed
    GattError,
    /// Service not found during discovery
    ServiceNotFound,
    /// Characteristic not found during discovery
    CharacteristicNotFound,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::NotConnected => write!(f, "Device is not connected"),
            Error::Parse(_) => write!(f, "Failed to parse notification"),
            Error::WriteFailed => write!(f, "Failed to write command"),
            Error::SubscribeFailed => write!(f, "Failed to subscribe to notifications"),
            Error::GattError => write!(f, "GATT operation failed"),
            Error::ServiceNotFound => write!(f, "Service not found"),
            Error::CharacteristicNotFound => write!(f, "Characteristic not found"),
        }
    }
}

impl From<variegated_scale_codec::bookoo::ParseError> for Error {
    fn from(e: variegated_scale_codec::bookoo::ParseError) -> Self {
        Error::Parse(e)
    }
}
