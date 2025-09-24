//! Builder pattern for display configuration

use display_interface::AsyncWriteOnlyDataCommand;
use embedded_hal::digital::OutputPin;

use crate::{
    display::DisplayVariant,
    displayrotation::DisplayRotation,
    mode::{displaymode::DisplayMode, raw::RawMode},
    properties::DisplayProperties,
    displays::nv3007::{Nv3007Variant, Nv3007_168_428},
};

/// Builder struct for configuring the display
#[derive(Clone, Copy)]
pub struct Builder<DV> {
    variant: DV,
    rotation: DisplayRotation,
}

impl<DV> Builder<DV> {
    /// Create new builder with a display variant
    pub fn new(variant: DV) -> Builder<DV> {
        Builder::<DV> {
            variant,
            rotation: DisplayRotation::Rotate0,
        }
    }
}

impl<DV> Builder<DV>
where
    DV: DisplayVariant,
{
    /// Set the rotation of the display
    pub fn with_rotation(self, rotation: DisplayRotation) -> Self {
        Self { rotation, ..self }
    }

    /// Connect the display with the given interface (no buffer)
    pub fn connect<DI>(self, interface: DI) -> DisplayMode<RawMode<DV, DI>>
    where
        DI: AsyncWriteOnlyDataCommand,
    {
        let properties = DisplayProperties::new(self.variant, interface, self.rotation);
        DisplayMode::<RawMode<DV, DI>>::new(properties)
    }

    /// Connect the display with a user-provided buffer for graphics mode
    pub fn connect_with_buffer<'a, DI>(
        self,
        interface: DI,
        buffer: &'a mut [u8],
    ) -> DisplayMode<crate::mode::graphics::GraphicsMode<'a, DV, DI>>
    where
        DI: AsyncWriteOnlyDataCommand,
    {
        let properties = DisplayProperties::new(self.variant, interface, self.rotation);
        DisplayMode::<crate::mode::graphics::GraphicsMode<'a, DV, DI>>::new_with_buffer(
            properties,
            buffer,
        )
    }
}

impl Builder<Nv3007_168_428> {
    /// Set the NV3007 variant to use
    pub fn with_variant(mut self, variant: Nv3007Variant) -> Self {
        self.variant.variant = variant;
        self
    }
}

/// Marker type for no reset pin
#[derive(Clone, Copy)]
pub enum NoOutputPin {}

impl OutputPin for NoOutputPin {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

impl embedded_hal::digital::ErrorType for NoOutputPin {
    type Error = core::convert::Infallible;
}