//! Graphics mode with embedded-graphics support

use crate::{
    command::{self, Command},
    display::{self, DisplayVariant}, displayrotation::DisplayRotation, mode::displaymode::DisplayModeTrait,
    properties::DisplayProperties,
    displays::nv3007::Nv3007_168_428,
};
use display_interface::{AsyncWriteOnlyDataCommand, DataFormat, DisplayError};
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;

/// Graphics mode handler with external buffer support
pub struct GraphicsMode<'a, DV, DI>
where
    DI: AsyncWriteOnlyDataCommand,
    DV: display::DisplayVariant,
{
    properties: DisplayProperties<DV, DI>,
    buffer: &'a mut [u8],
}

impl<'a, DV, DI> DisplayModeTrait<DV, DI> for GraphicsMode<'a, DV, DI>
where
    DI: AsyncWriteOnlyDataCommand,
    DV: display::DisplayVariant,
{
    /// Create new GraphicsMode instance (requires buffer to be provided separately)
    fn new(_properties: DisplayProperties<DV, DI>) -> Self {
        panic!("GraphicsMode requires a buffer. Use new_with_buffer instead.")
    }

    /// Create new GraphicsMode instance with buffer
    fn new_with_buffer(properties: DisplayProperties<DV, DI>, buffer: &mut [u8]) -> Self
    where
        Self: Sized,
    {
        // Safety: We're transmuting the lifetime to 'a
        // This is safe because we know the buffer outlives the GraphicsMode
        let buffer = unsafe { core::mem::transmute::<&mut [u8], &'a mut [u8]>(buffer) };

        GraphicsMode { properties, buffer }
    }

    /// Release resources
    fn release(self) -> DisplayProperties<DV, DI> {
        self.properties
    }
}

impl<'a, DV, DI> GraphicsMode<'a, DV, DI>
where
    DI: AsyncWriteOnlyDataCommand,
    DV: display::DisplayVariant,
{
    /// Clear the display buffer
    pub fn clear(&mut self) {
        self.buffer.fill(0);
    }

    /// Reset display using reset pin
    pub fn reset<RST, DELAY, PinE>(
        &mut self,
        rst: &mut RST,
        delay: &mut DELAY,
    ) -> Result<(), PinE>
    where
        RST: OutputPin<Error = PinE>,
        DELAY: DelayNs,
    {
        rst.set_high()?;
        delay.delay_ms(1);
        rst.set_low()?;
        delay.delay_ms(10);
        rst.set_high()?;
        delay.delay_ms(120);
        Ok(())
    }

    /// Initialize the display
    pub async fn init(&mut self) -> Result<(), DisplayError> {
        // Initialize the display variant
        DV::init(&mut self.properties.iface).await?;

        // Set initial rotation
        self.set_rotation(self.properties.rotation).await?;

        Ok(())
    }

    /// Set display rotation
    pub async fn set_rotation(&mut self, rotation: DisplayRotation) -> Result<(), DisplayError> {
        self.properties.rotation = rotation;

        let madctl_value = match rotation {
            DisplayRotation::Rotate0 => command::madctl::RGB,
            DisplayRotation::Rotate90 => {
                command::madctl::MX | command::madctl::MV | command::madctl::RGB
            }
            DisplayRotation::Rotate180 => {
                command::madctl::MX | command::madctl::MY | command::madctl::RGB
            }
            DisplayRotation::Rotate270 => {
                command::madctl::MY | command::madctl::MV | command::madctl::RGB
            }
        };

        Command::MemoryAccessControl(madctl_value)
            .send(&mut self.properties.iface)
            .await
    }

    /// Write data to display
    pub async fn flush(&mut self) -> Result<(), DisplayError> {
        let (width, height) = self.effective_dimensions();

        // Set address window to full screen
        self.set_address_window(0, 0, width - 1, height - 1)
            .await?;

        // Start memory write
        Command::MemoryWrite.send(&mut self.properties.iface).await?;

        // Send buffer data
        self.properties
            .iface
            .send_data(DataFormat::U8(&self.buffer[..self.buffer_size()]))
            .await
    }

    /// Get effective dimensions based on rotation
    pub fn effective_dimensions(&self) -> (u16, u16) {
        let (width, height) = DV::dimensions();
        match self.properties.rotation {
            DisplayRotation::Rotate0 | DisplayRotation::Rotate180 => (width, height),
            DisplayRotation::Rotate90 | DisplayRotation::Rotate270 => (height, width),
        }
    }

    /// Set a pixel in the buffer (RGB565 format)
    pub fn set_pixel(&mut self, x: u16, y: u16, color: u16) {
        let (width, height) = self.effective_dimensions();
        if x >= width || y >= height {
            return;
        }

        let idx = ((y as usize) * (width as usize) + (x as usize)) * 2;
        if idx + 1 < self.buffer.len() {
            self.buffer[idx] = (color >> 8) as u8;
            self.buffer[idx + 1] = (color & 0xff) as u8;
        }
    }

    /// Get required buffer size
    fn buffer_size(&self) -> usize {
        let (width, height) = self.effective_dimensions();
        (width as usize) * (height as usize) * 2
    }

    /// Set address window for partial updates
    async fn set_address_window(
        &mut self,
        x0: u16,
        y0: u16,
        x1: u16,
        y1: u16,
    ) -> Result<(), DisplayError> {
        Command::ColumnAddressSet {
            start: x0 + DV::COLUMN_OFFSET,
            end: x1 + DV::COLUMN_OFFSET,
        }
        .send(&mut self.properties.iface)
        .await?;

        Command::RowAddressSet {
            start: y0 + DV::ROW_OFFSET,
            end: y1 + DV::ROW_OFFSET,
        }
        .send(&mut self.properties.iface)
        .await
    }
}

impl<'a, DI> GraphicsMode<'a, Nv3007_168_428, DI>
where
    DI: AsyncWriteOnlyDataCommand,
{
    /// Initialize the display using the variant-specific initialization
    pub async fn init_with_variant(&mut self) -> Result<(), DisplayError> {
        // Initialize the display variant using the specified variant
        self.properties.variant.init_self(&mut self.properties.iface).await?;

        // Set initial rotation
        self.set_rotation(self.properties.rotation).await?;

        Ok(())
    }

    /// Set address window for partial updates (NV3007 specific)
    async fn set_address_window_nv3007(
        &mut self,
        x0: u16,
        y0: u16,
        x1: u16,
        y1: u16,
    ) -> Result<(), DisplayError> {
        Command::ColumnAddressSet {
            start: x0 + Nv3007_168_428::COLUMN_OFFSET,
            end: x1 + Nv3007_168_428::COLUMN_OFFSET,
        }
        .send(&mut self.properties.iface)
        .await?;

        Command::RowAddressSet {
            start: y0 + Nv3007_168_428::ROW_OFFSET,
            end: y1 + Nv3007_168_428::ROW_OFFSET,
        }
        .send(&mut self.properties.iface)
        .await
    }

    /// Write data to display (NV3007 specific version)
    pub async fn flush_nv3007(&mut self) -> Result<(), DisplayError> {
        let (width, height) = self.effective_dimensions_nv3007();

        // Set address window to full screen
        self.set_address_window_nv3007(0, 0, width - 1, height - 1)
            .await?;

        // Start memory write
        Command::MemoryWrite.send(&mut self.properties.iface).await?;

        // Send buffer data
        self.properties
            .iface
            .send_data(DataFormat::U8(&self.buffer[..self.buffer_size_nv3007()]))
            .await
    }

    /// Get effective dimensions based on rotation (NV3007 specific)
    pub fn effective_dimensions_nv3007(&self) -> (u16, u16) {
        let (width, height) = Nv3007_168_428::dimensions();
        match self.properties.rotation {
            DisplayRotation::Rotate0 | DisplayRotation::Rotate180 => (width, height),
            DisplayRotation::Rotate90 | DisplayRotation::Rotate270 => (height, width),
        }
    }

    /// Get required buffer size (NV3007 specific)
    fn buffer_size_nv3007(&self) -> usize {
        let (width, height) = self.effective_dimensions_nv3007();
        (width as usize) * (height as usize) * 2
    }
}

#[cfg(feature = "graphics")]
use embedded_graphics_core::{
    draw_target::DrawTarget,
    geometry::{OriginDimensions, Size},
    pixelcolor::Rgb565,
    prelude::*,
    Pixel,
};

#[cfg(feature = "graphics")]
impl<'a, DV, DI> DrawTarget for GraphicsMode<'a, DV, DI>
where
    DI: AsyncWriteOnlyDataCommand,
    DV: display::DisplayVariant,
{
    type Color = Rgb565;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(coord, color) in pixels {
            if coord.x >= 0 && coord.y >= 0 {
                self.set_pixel(coord.x as u16, coord.y as u16, color.into_storage());
            }
        }
        Ok(())
    }
}

#[cfg(feature = "graphics")]
impl<'a, DV, DI> OriginDimensions for GraphicsMode<'a, DV, DI>
where
    DI: AsyncWriteOnlyDataCommand,
    DV: display::DisplayVariant,
{
    fn size(&self) -> Size {
        let (width, height) = self.effective_dimensions();
        Size::new(width as u32, height as u32)
    }
}