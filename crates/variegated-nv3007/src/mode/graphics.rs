//! Graphics mode with embedded-graphics support and optional delta updates

use crate::{
    command::{self, Command},
    display::{self, DisplayVariant},
    displayrotation::DisplayRotation,
    displays::nv3007::Nv3007_168_428,
    mode::displaymode::DisplayModeTrait,
    properties::DisplayProperties,
};
use display_interface::{AsyncWriteOnlyDataCommand, DataFormat, DisplayError};
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;

#[cfg(feature = "delta-updates")]
use crate::region_tracker::DoubleBuffer;

/// Graphics mode handler with external buffer support
///
/// Memory layout: Row-major (pixels stored left-to-right, top-to-bottom)
/// - Optimized for full-screen updates
/// - Natural memory order for better cache locality
/// - Supports optional delta updates with double-buffering (when `delta-updates` feature enabled)
pub struct GraphicsMode<'a, DV, DI>
where
    DI: AsyncWriteOnlyDataCommand,
    DV: display::DisplayVariant,
{
    properties: DisplayProperties<DV, DI>,
    buffer: &'a mut [u8],

    #[cfg(feature = "delta-updates")]
    previous_buffer: Option<&'a mut [u8]>,

    #[cfg(feature = "delta-updates")]
    double_buffer_tracker: Option<DoubleBuffer>,
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

        GraphicsMode {
            properties,
            buffer,
            #[cfg(feature = "delta-updates")]
            previous_buffer: None,
            #[cfg(feature = "delta-updates")]
            double_buffer_tracker: None,
        }
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
    /// Create new GraphicsMode instance with double buffering for delta updates
    #[cfg(feature = "delta-updates")]
    pub fn new_with_double_buffer(
        properties: DisplayProperties<DV, DI>,
        current_buffer: &mut [u8],
        previous_buffer: &mut [u8],
    ) -> Self
    where
        Self: Sized,
    {
        // Safety: We're transmuting the lifetime to 'a
        let current_buffer =
            unsafe { core::mem::transmute::<&mut [u8], &'a mut [u8]>(current_buffer) };
        let previous_buffer =
            unsafe { core::mem::transmute::<&mut [u8], &'a mut [u8]>(previous_buffer) };

        let (width, height) = DV::dimensions();
        let (eff_width, eff_height) = match properties.rotation {
            DisplayRotation::Rotate0 | DisplayRotation::Rotate180 => (width, height),
            DisplayRotation::Rotate90 | DisplayRotation::Rotate270 => (height, width),
        };

        // Initialize previous buffer to match current (starting clean)
        let copy_size = current_buffer.len().min(previous_buffer.len());
        previous_buffer[..copy_size].copy_from_slice(&current_buffer[..copy_size]);

        let double_buffer_tracker = DoubleBuffer::new(eff_width, eff_height);

        GraphicsMode {
            properties,
            buffer: current_buffer,
            previous_buffer: Some(previous_buffer),
            double_buffer_tracker: Some(double_buffer_tracker),
        }
    }

    // ========== CLEAR IMPLEMENTATION ==========

    /// Clear the display buffer
    pub fn clear(&mut self) {
        self.buffer.fill(0);
    }

    // ========== PIXEL OFFSET CALCULATION ==========

    /// Calculate pixel offset in row-major layout
    fn pixel_offset(&self, x: u16, y: u16) -> usize {
        let (width, _height) = self.effective_dimensions();
        ((y as usize) * (width as usize) + (x as usize)) * 2
    }

    // ========== COMMON METHODS ==========

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
    ///
    /// After calling `init()`, you should typically call `flush()` to send
    /// the initial framebuffer contents to the display.
    ///
    /// # Example
    /// ```ignore
    /// display.init().await?;
    /// display.flush().await?; // Send initial contents
    /// ```
    pub async fn init(&mut self) -> Result<(), DisplayError> {
        // Initialize the display variant
        DV::init(&mut self.properties.iface).await?;

        // Set initial rotation (handled by display controller via MADCTL)
        self.set_rotation(self.properties.rotation).await?;

        Ok(())
    }

    /// Set display rotation
    ///
    /// Note: Rotation is handled by the display controller via MADCTL register.
    ///
    /// **WARNING**: With delta-updates enabled, rotation changes after initialization
    /// may cause incorrect rendering because the double-buffer is based on the
    /// initial rotation. Set rotation via Builder::with_rotation() before calling init().
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
            .await?;

        Ok(())
    }

    // ========== FLUSH IMPLEMENTATIONS ==========

    /// Write data to display
    /// - Without delta-updates: always full screen flush
    /// - With delta-updates: smart region-based flush if double-buffer is configured
    #[cfg(feature = "delta-updates")]
    pub async fn flush(&mut self) -> Result<(), DisplayError> {
        // Detect changes and decide on update strategy
        let (do_full, regions) = if let (Some(ref tracker), Some(ref prev)) =
            (&self.double_buffer_tracker, &self.previous_buffer) {
            let regions = instrumented_section!("Determine Regions", { tracker.detect_changes(self.buffer, prev) });
            let do_full = instrumented_section!("Determine full", { tracker.should_full_update(&regions) });
            (do_full, Some(regions))
        } else {
            (true, None)
        };

        // Perform the update
        if do_full {
            instrumented_section!("Full Flush", {
                self.flush_full().await?;
            });
        } else if let Some(regions) = regions {
            instrumented_section!("Region flush", {
                self.flush_regions(&regions).await?;
            });
        }

        // Swap buffers after successful update
        if let (Some(ref tracker), Some(ref mut prev)) =
            (&self.double_buffer_tracker, &mut self.previous_buffer) {
            tracker.swap_buffers(self.buffer, prev);
        }

        Ok(())
    }

    /// Write data to display (always full flush without delta-updates)
    #[cfg(not(feature = "delta-updates"))]
    pub async fn flush(&mut self) -> Result<(), DisplayError> {
        self.flush_full().await
    }

    /// Force full screen update (single contiguous transfer)
    pub async fn flush_full(&mut self) -> Result<(), DisplayError> {
        let (width, height) = self.effective_dimensions();

        // Set address window to full screen
        self.set_address_window(0, 0, width - 1, height - 1)
            .await?;

        // Start memory write
        Command::MemoryWrite
            .send(&mut self.properties.iface)
            .await?;

        // Single contiguous transfer - entire framebuffer (zero-copy DMA!)
        self.properties
            .iface
            .send_data(DataFormat::U8(&self.buffer[..self.buffer_size()]))
            .await
    }

    /// Flush specific regions (delta-updates only)
    #[cfg(feature = "delta-updates")]
    async fn flush_regions(
        &mut self,
        regions: &[crate::region_tracker::Region],
    ) -> Result<(), DisplayError> {
        let (width, _) = self.effective_dimensions();

        for region in regions {
            if region.is_full_width(width) {
                // FAST PATH: Full-width region = single contiguous transfer
                self.flush_full_width_region(*region).await?;
            } else {
                // SLOWER PATH: Partial-width = row-by-row transfers
                self.flush_partial_width_region(*region).await?;
            }
        }
        Ok(())
    }

    /// Flush a full-width region efficiently
    #[cfg(feature = "delta-updates")]
    async fn flush_full_width_region(
        &mut self,
        region: crate::region_tracker::Region,
    ) -> Result<(), DisplayError> {
        let (width, _) = self.effective_dimensions();

        // Set address window once
        self.set_address_window(0, region.y0, width - 1, region.y1)
            .await?;
        Command::MemoryWrite
            .send(&mut self.properties.iface)
            .await?;

        // Calculate buffer offset and size
        let offset = (region.y0 as usize * width as usize) * 2;
        let size = ((region.y1 - region.y0 + 1) as usize * width as usize) * 2;

        // Single contiguous transfer for all rows!
        self.properties
            .iface
            .send_data(DataFormat::U8(&self.buffer[offset..offset + size]))
            .await
    }

    /// Flush a partial-width region row-by-row
    #[cfg(feature = "delta-updates")]
    async fn flush_partial_width_region(
        &mut self,
        region: crate::region_tracker::Region,
    ) -> Result<(), DisplayError> {
        let (width, _) = self.effective_dimensions();

        // Set address window once for entire region
        self.set_address_window(region.x0, region.y0, region.x1, region.y1)
            .await?;
        Command::MemoryWrite
            .send(&mut self.properties.iface)
            .await?;

        // Send row by row (display auto-increments to next row)
        let row_width = (region.x1 - region.x0 + 1) as usize;
        for y in region.y0..=region.y1 {
            let offset = (y as usize * width as usize + region.x0 as usize) * 2;
            let size = row_width * 2;

            self.properties
                .iface
                .send_data(DataFormat::U8(&self.buffer[offset..offset + size]))
                .await?;
        }
        Ok(())
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

        let idx = self.pixel_offset(x, y);
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
        self.properties
            .variant
            .init_self(&mut self.properties.iface)
            .await?;

        // Set initial rotation (handled by display controller via MADCTL)
        self.set_rotation(self.properties.rotation).await?;

        Ok(())
    }

    /// Set address window for partial updates (NV3007 specific)
    ///
    /// `pub` because this is a general driver for the panel, not for the one way the GS3
    /// firmware drives it: a consumer doing its own partial updates needs the window
    /// setter even though nothing in this crate calls it yet.
    pub async fn set_address_window_nv3007(
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

    /// Write data to display (NV3007 specific version - legacy compatibility)
    pub async fn flush_nv3007(&mut self) -> Result<(), DisplayError> {
        self.flush_full().await
    }

    /// Get effective dimensions based on rotation (NV3007 specific)
    pub fn effective_dimensions_nv3007(&self) -> (u16, u16) {
        let (width, height) = Nv3007_168_428::dimensions();
        match self.properties.rotation {
            DisplayRotation::Rotate0 | DisplayRotation::Rotate180 => (width, height),
            DisplayRotation::Rotate90 | DisplayRotation::Rotate270 => (height, width),
        }
    }

    /// Get required buffer size (NV3007 specific), accounting for rotation.
    ///
    /// `pub` for the same reason as `set_address_window_nv3007`: a caller allocating its
    /// own framebuffer needs this number, and the panel is what knows it.
    pub fn buffer_size_nv3007(&self) -> usize {
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
use variegated_instrumentation::instrumented_section;

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

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        if color == Rgb565::BLACK {
            self.clear();
        } else {
            let packed_color = color.into_storage();
            let high_byte = (packed_color >> 8) as u8;
            let low_byte = (packed_color & 0xff) as u8;
            for chunk in self.buffer.chunks_exact_mut(2) {
                chunk[0] = high_byte;
                chunk[1] = low_byte;
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
