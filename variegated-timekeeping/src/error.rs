//! Error types for the timekeeping crate

#[cfg(feature = "defmt")]
use defmt::Format;

/// Errors that can occur when working with the TimeKeeper
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub enum Error {
    /// The TimeKeeper has not been initialized with a time
    Uninitialized,
    /// Invalid date or time value
    InvalidDateTime,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::Uninitialized => write!(f, "TimeKeeper not initialized"),
            Error::InvalidDateTime => write!(f, "Invalid date or time value"),
        }
    }
}

/// Result type for timekeeping operations
pub type Result<T> = core::result::Result<T, Error>;