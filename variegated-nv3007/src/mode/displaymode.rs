//! Display mode trait and wrapper

use crate::{display, properties::DisplayProperties};
use display_interface::AsyncWriteOnlyDataCommand;

/// Trait for all display modes
pub trait DisplayModeTrait<DV, DI>
where
    DI: AsyncWriteOnlyDataCommand,
    DV: display::DisplayVariant,
{
    /// Create new display mode instance
    fn new(properties: DisplayProperties<DV, DI>) -> Self;

    /// Create new display mode instance with buffer
    fn new_with_buffer(properties: DisplayProperties<DV, DI>, _buffer: &mut [u8]) -> Self
    where
        Self: Sized,
    {
        Self::new(properties)
    }

    /// Release all resources used by the display mode
    fn release(self) -> DisplayProperties<DV, DI>;
}

/// Display mode wrapper
pub struct DisplayMode<DM> {
    mode: DM,
}

impl<DM> DisplayMode<DM> {
    /// Create new display mode
    pub(crate) fn new<DV, DI>(properties: DisplayProperties<DV, DI>) -> Self
    where
        DM: DisplayModeTrait<DV, DI>,
        DI: AsyncWriteOnlyDataCommand,
        DV: display::DisplayVariant,
    {
        Self {
            mode: DM::new(properties),
        }
    }

    /// Create new display mode with buffer
    pub(crate) fn new_with_buffer<DV, DI>(
        properties: DisplayProperties<DV, DI>,
        buffer: &mut [u8],
    ) -> Self
    where
        DM: DisplayModeTrait<DV, DI>,
        DI: AsyncWriteOnlyDataCommand,
        DV: display::DisplayVariant,
    {
        Self {
            mode: DM::new_with_buffer(properties, buffer),
        }
    }

    /// Release the display mode
    pub fn release<DV, DI>(self) -> DisplayProperties<DV, DI>
    where
        DM: DisplayModeTrait<DV, DI>,
        DI: AsyncWriteOnlyDataCommand,
        DV: display::DisplayVariant,
    {
        self.mode.release()
    }
}

impl<DM> core::ops::Deref for DisplayMode<DM> {
    type Target = DM;

    fn deref(&self) -> &Self::Target {
        &self.mode
    }
}

impl<DM> core::ops::DerefMut for DisplayMode<DM> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.mode
    }
}

impl<DV, DI, DM> From<DisplayProperties<DV, DI>> for DisplayMode<DM>
where
    DM: DisplayModeTrait<DV, DI>,
    DI: AsyncWriteOnlyDataCommand,
    DV: display::DisplayVariant,
{
    fn from(properties: DisplayProperties<DV, DI>) -> Self {
        Self::new(properties)
    }
}