//! No-operation implementations for testing and development
//!
//! This module provides no-op implementations of various embedded-hal traits
//! that can be used for testing, development, or as placeholders when certain
//! hardware features are not needed.

#![allow(unused)]

use embedded_hal::digital::{OutputPin, ErrorType, Error, ErrorKind};

/// A no-operation output pin that always succeeds
///
/// This implementation can be used as a placeholder for output pins
/// that are not connected or not needed in certain configurations.
#[derive(Debug, Default, Clone, Copy)]
pub struct NoopOutputPin;

/// Error type for NoopOutputPin operations
///
/// While this pin never actually fails, an error type is still
/// required by the embedded-hal traits.
#[derive(Debug, Clone, Copy)]
pub struct NoopOutputPinError;

impl Error for NoopOutputPinError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

impl ErrorType for NoopOutputPin {
    type Error = NoopOutputPinError;
}

impl OutputPin for NoopOutputPin {
    /// Sets the pin to low state (no actual operation performed)
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    /// Sets the pin to high state (no actual operation performed)
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}