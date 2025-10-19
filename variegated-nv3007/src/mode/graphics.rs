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
use crate::dirty_tracker::{DirtyTracker, TileRegion};

/// Configuration for delta updates
#[cfg(feature = "delta-updates")]
#[derive(Debug, Clone, Copy)]
pub struct DeltaConfig {
    /// Threshold for switching to full update (0.0-1.0, default 0.75)
    pub full_update_threshold: f32,
    /// Distance in tiles for merging nearby dirty regions (default 2)
    pub merge_distance: u16,
}

#[cfg(feature = "delta-updates")]
impl Default for DeltaConfig {
    fn default() -> Self {
        Self {
            full_update_threshold: 0.75,
            merge_distance: 2,
        }
    }
}

/// Calculate number of tiles needed for a dimension
#[cfg(feature = "delta-updates")]
const fn calc_tiles(dimension: u16, tile_size: u16) -> u16 {
    (dimension + tile_size - 1) / tile_size
}

/// Graphics mode handler with external buffer support
///
/// Memory layout depends on the `delta-updates` feature:
/// - **With `delta-updates`**: Tile-major layout optimized for partial updates
/// - **Without `delta-updates`**: Row-major layout optimized for full-screen updates
pub struct GraphicsMode<'a, DV, DI, const TILE_SIZE: usize = 8>
where
    DI: AsyncWriteOnlyDataCommand,
    DV: display::DisplayVariant,
{
    properties: DisplayProperties<DV, DI>,
    buffer: &'a mut [u8],

    #[cfg(feature = "delta-updates")]
    dirty_tracker: DirtyTracker<TILE_SIZE>,

    #[cfg(feature = "delta-updates")]
    config: DeltaConfig,
}

impl<'a, DV, DI, const TILE_SIZE: usize> DisplayModeTrait<DV, DI>
    for GraphicsMode<'a, DV, DI, TILE_SIZE>
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

        #[cfg(feature = "delta-updates")]
        let dirty_tracker = {
            // Use effective dimensions (accounts for rotation) to match framebuffer layout
            // This must match how pixel_offset() calculates tile positions
            let (width, height) = DV::dimensions();
            let (eff_width, eff_height) = match properties.rotation {
                DisplayRotation::Rotate0 | DisplayRotation::Rotate180 => (width, height),
                DisplayRotation::Rotate90 | DisplayRotation::Rotate270 => (height, width),
            };
            DirtyTracker::new(eff_width, eff_height)
        };

        GraphicsMode {
            properties,
            buffer,
            #[cfg(feature = "delta-updates")]
            dirty_tracker,
            #[cfg(feature = "delta-updates")]
            config: DeltaConfig::default(),
        }
    }

    /// Release resources
    fn release(self) -> DisplayProperties<DV, DI> {
        self.properties
    }
}

impl<'a, DV, DI, const TILE_SIZE: usize> GraphicsMode<'a, DV, DI, TILE_SIZE>
where
    DI: AsyncWriteOnlyDataCommand,
    DV: display::DisplayVariant,
{
    // ========== CLEAR IMPLEMENTATIONS ==========

    /// Clear the display buffer (smart clear with delta-updates, simple fill without)
    #[cfg(feature = "delta-updates")]
    pub fn clear(&mut self) {
        let tiles_x = self.dirty_tracker.tiles_x();
        let tiles_y = self.dirty_tracker.tiles_y();

        for tile_y in 0..tiles_y {
            for tile_x in 0..tiles_x {
                if self.is_tile_non_black(tile_x, tile_y) {
                    self.clear_tile_to_black(tile_x, tile_y);
                    self.dirty_tracker.mark_tile_dirty(tile_x, tile_y);
                }
            }
        }
    }

    /// Clear the display buffer (simple fill for row-major layout)
    #[cfg(not(feature = "delta-updates"))]
    pub fn clear(&mut self) {
        self.buffer.fill(0);
    }

    // ========== TILE-MAJOR LAYOUT HELPERS (delta-updates only) ==========

    #[cfg(feature = "delta-updates")]
    fn is_tile_non_black(&self, tile_x: u16, tile_y: u16) -> bool {
        let tile_data = self.get_tile_slice(tile_x, tile_y);

        // Check as u64 chunks for performance (32 pixels at once)
        tile_data
            .chunks_exact(8)
            .any(|chunk| u64::from_ne_bytes(chunk.try_into().unwrap()) != 0)
            || tile_data
                .chunks_exact(8)
                .remainder()
                .iter()
                .any(|&b| b != 0)
    }

    #[cfg(feature = "delta-updates")]
    fn clear_tile_to_black(&mut self, tile_x: u16, tile_y: u16) {
        let tiles_x = self.dirty_tracker.tiles_x();
        let tile_index = (tile_y * tiles_x + tile_x) as usize;
        let offset = tile_index * TILE_SIZE * TILE_SIZE * 2;
        let size = TILE_SIZE * TILE_SIZE * 2;

        if offset + size <= self.buffer.len() {
            self.buffer[offset..offset + size].fill(0);
        }
    }

    #[cfg(feature = "delta-updates")]
    fn get_tile_slice(&self, tile_x: u16, tile_y: u16) -> &[u8] {
        let tiles_x = self.dirty_tracker.tiles_x();
        let tile_index = (tile_y * tiles_x + tile_x) as usize;
        let offset = tile_index * TILE_SIZE * TILE_SIZE * 2;
        let size = TILE_SIZE * TILE_SIZE * 2;

        if offset + size <= self.buffer.len() {
            &self.buffer[offset..offset + size]
        } else {
            &[]
        }
    }

    // ========== PIXEL OFFSET CALCULATION ==========

    /// Calculate pixel offset in tile-major layout (delta-updates)
    /// Note: Uses effective dimensions (accounting for rotation) to match the dirty tracker
    /// and coordinate space used by embedded-graphics and set_pixel().
    #[cfg(feature = "delta-updates")]
    fn pixel_offset(&self, x: u16, y: u16) -> usize {
        // Use effective dimensions to match dirty tracker and drawing coordinate space
        let (width, _height) = self.effective_dimensions();
        let tiles_x = calc_tiles(width, TILE_SIZE as u16);

        let tile_x = x / TILE_SIZE as u16;
        let tile_y = y / TILE_SIZE as u16;
        let pixel_x_in_tile = x % TILE_SIZE as u16;
        let pixel_y_in_tile = y % TILE_SIZE as u16;

        let tile_index = tile_y * tiles_x + tile_x;
        let pixel_in_tile = pixel_y_in_tile * TILE_SIZE as u16 + pixel_x_in_tile;

        (tile_index as usize * TILE_SIZE * TILE_SIZE + pixel_in_tile as usize) * 2
    }

    /// Calculate pixel offset in row-major layout (no delta-updates)
    #[cfg(not(feature = "delta-updates"))]
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
    /// After calling `init()`, you should typically call `flush_full_force()` to send
    /// the initial framebuffer contents to the display (with delta-updates enabled).
    ///
    /// # Example
    /// ```ignore
    /// display.init().await?;
    /// display.flush_full_force().await?; // Send initial contents
    /// ```
    pub async fn init(&mut self) -> Result<(), DisplayError> {
        // Initialize the display variant
        DV::init(&mut self.properties.iface).await?;

        // Set initial rotation (handled by display controller via MADCTL)
        self.set_rotation(self.properties.rotation).await?;

        // Mark all tiles dirty for first flush
        #[cfg(feature = "delta-updates")]
        self.dirty_tracker.mark_all_dirty();

        Ok(())
    }

    /// Set display rotation
    ///
    /// Note: Rotation is handled by the display controller via MADCTL register.
    ///
    /// **WARNING**: With delta-updates enabled, rotation changes after initialization
    /// may cause incorrect rendering because the framebuffer layout is based on the
    /// initial rotation. Set rotation via Builder::with_rotation() before calling init().
    pub async fn set_rotation(&mut self, rotation: DisplayRotation) -> Result<(), DisplayError> {
        #[cfg(feature = "delta-updates")]
        {
            // Check if rotation has changed since initialization
            // Rotation changes with delta-updates would require reinitializing the dirty tracker
            // and potentially re-laying out the framebuffer, which is not currently supported
            if self.properties.rotation != rotation {
                // For now, we allow the change but warn that it may cause issues
                // In the future, could return an error or reinitialize dirty tracker
            }
        }

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

        // Mark all tiles dirty after rotation change
        #[cfg(feature = "delta-updates")]
        self.dirty_tracker.mark_all_dirty();

        Ok(())
    }

    // ========== FLUSH IMPLEMENTATIONS ==========

    /// Write data to display (smart flush with delta-updates, full flush without)
    #[cfg(feature = "delta-updates")]
    pub async fn flush(&mut self) -> Result<(), DisplayError> {
        self.flush_smart().await
    }

    /// Write data to display (always full flush without delta-updates)
    #[cfg(not(feature = "delta-updates"))]
    pub async fn flush(&mut self) -> Result<(), DisplayError> {
        self.flush_full().await
    }

    /// Smart flush - chooses between delta and full update (delta-updates only)
    #[cfg(feature = "delta-updates")]
    async fn flush_smart(&mut self) -> Result<(), DisplayError> {
        let dirty_ratio = self.dirty_tracker.get_dirty_ratio();

        if dirty_ratio >= self.config.full_update_threshold {
            self.flush_full().await
        } else {
            self.flush_dirty().await
        }
    }

    /// Flush only dirty regions (delta-updates only)
    #[cfg(feature = "delta-updates")]
    async fn flush_dirty(&mut self) -> Result<(), DisplayError> {
        let regions = self
            .dirty_tracker
            .get_dirty_regions(self.config.merge_distance);

        for region in regions.iter() {
            self.flush_region(*region).await?;
        }

        self.dirty_tracker.clear_all_dirty();
        Ok(())
    }

    /// Flush a specific tile region (delta-updates only)
    #[cfg(feature = "delta-updates")]
    async fn flush_region(&mut self, region: TileRegion) -> Result<(), DisplayError> {
        let (width, height) = self.effective_dimensions();
        let tiles_x = self.dirty_tracker.tiles_x();

        // Send each tile with its own address window (only if content changed)
        for tile_y in region.y0..=region.y1 {
            for tile_x in region.x0..=region.x1 {
                // Calculate tile location in framebuffer
                let tile_index = tile_y * tiles_x + tile_x;
                let offset = tile_index as usize * TILE_SIZE * TILE_SIZE * 2;
                let size = TILE_SIZE * TILE_SIZE * 2;

                if offset + size > self.buffer.len() {
                    continue; // Skip invalid tiles
                }

                // Check if tile content actually changed using checksum
                // This borrows buffer briefly to calculate checksum
                let content_changed = {
                    let tile_data = &self.buffer[offset..offset + size];
                    self.dirty_tracker.tile_content_changed(tile_x, tile_y, tile_data)
                };

                if !content_changed {
                    // Content unchanged - skip this tile (optimization!)
                    continue;
                }

                // Content changed - send this tile
                // Calculate pixel coordinates for this tile
                let x0 = tile_x * TILE_SIZE as u16;
                let y0 = tile_y * TILE_SIZE as u16;
                let x1 = ((tile_x + 1) * TILE_SIZE as u16).min(width) - 1;
                let y1 = ((tile_y + 1) * TILE_SIZE as u16).min(height) - 1;

                // Set address window for this tile
                self.set_address_window(x0, y0, x1, y1).await?;

                // Start memory write
                Command::MemoryWrite
                    .send(&mut self.properties.iface)
                    .await?;

                // Zero-copy DMA: send directly from framebuffer!
                self.properties
                    .iface
                    .send_data(DataFormat::U8(&self.buffer[offset..offset + size]))
                    .await?;
            }
        }

        Ok(())
    }

    /// Force full screen update (tile-major layout for delta-updates)
    #[cfg(feature = "delta-updates")]
    pub async fn flush_full(&mut self) -> Result<(), DisplayError> {
        let (width, height) = self.effective_dimensions();
        let tiles_x = self.dirty_tracker.tiles_x();
        let tiles_y = self.dirty_tracker.tiles_y();

        // Send each tile with its own address window (only if content changed)
        for tile_y in 0..tiles_y {
            for tile_x in 0..tiles_x {
                // Calculate tile location in framebuffer
                let tile_index = tile_y * tiles_x + tile_x;
                let offset = tile_index as usize * TILE_SIZE * TILE_SIZE * 2;
                let size = TILE_SIZE * TILE_SIZE * 2;

                if offset + size > self.buffer.len() {
                    continue; // Skip invalid tiles
                }

                // Check if tile content actually changed using checksum
                // This borrows buffer briefly to calculate checksum
                let content_changed = {
                    let tile_data = &self.buffer[offset..offset + size];
                    self.dirty_tracker.tile_content_changed(tile_x, tile_y, tile_data)
                };

                if !content_changed {
                    // Content unchanged - skip this tile (optimization!)
                    continue;
                }

                // Content changed - send this tile
                // Calculate pixel coordinates for this tile
                let x0 = tile_x * TILE_SIZE as u16;
                let y0 = tile_y * TILE_SIZE as u16;
                let x1 = ((tile_x + 1) * TILE_SIZE as u16).min(width) - 1;
                let y1 = ((tile_y + 1) * TILE_SIZE as u16).min(height) - 1;

                // Set address window for this tile
                self.set_address_window(x0, y0, x1, y1).await?;

                // Start memory write
                Command::MemoryWrite
                    .send(&mut self.properties.iface)
                    .await?;

                // Zero-copy DMA: send directly from framebuffer!
                self.properties
                    .iface
                    .send_data(DataFormat::U8(&self.buffer[offset..offset + size]))
                    .await?;
            }
        }

        self.dirty_tracker.clear_all_dirty();
        Ok(())
    }

    /// Force full screen update, ignoring checksums (delta-updates only)
    ///
    /// This sends ALL tiles to the display regardless of whether their content
    /// has changed according to checksums. Checksums are updated during the flush.
    ///
    /// Use this for:
    /// - Initial flush after init() (especially if framebuffer starts as all zeros)
    /// - After display reset or power cycle
    /// - When you need to guarantee display matches framebuffer state
    ///
    /// For normal updates, use `flush()` which only sends changed tiles.
    #[cfg(feature = "delta-updates")]
    pub async fn flush_full_force(&mut self) -> Result<(), DisplayError> {
        let (width, height) = self.effective_dimensions();
        let tiles_x = self.dirty_tracker.tiles_x();
        let tiles_y = self.dirty_tracker.tiles_y();

        // Send each tile with its own address window, ALWAYS updating checksums
        for tile_y in 0..tiles_y {
            for tile_x in 0..tiles_x {
                // Calculate tile location in framebuffer
                let tile_index = tile_y * tiles_x + tile_x;
                let offset = tile_index as usize * TILE_SIZE * TILE_SIZE * 2;
                let size = TILE_SIZE * TILE_SIZE * 2;

                if offset + size > self.buffer.len() {
                    continue; // Skip invalid tiles
                }

                // Get tile data and update checksum (no conditional check)
                let tile_data = &self.buffer[offset..offset + size];
                // Force checksum update by always calling tile_content_changed
                // (it will update the stored checksum even if content hasn't changed)
                let _ = self.dirty_tracker.tile_content_changed(tile_x, tile_y, tile_data);

                // Calculate pixel coordinates for this tile
                let x0 = tile_x * TILE_SIZE as u16;
                let y0 = tile_y * TILE_SIZE as u16;
                let x1 = ((tile_x + 1) * TILE_SIZE as u16).min(width) - 1;
                let y1 = ((tile_y + 1) * TILE_SIZE as u16).min(height) - 1;

                // Set address window for this tile
                self.set_address_window(x0, y0, x1, y1).await?;

                // Start memory write
                Command::MemoryWrite
                    .send(&mut self.properties.iface)
                    .await?;

                // Zero-copy DMA: send directly from framebuffer!
                self.properties
                    .iface
                    .send_data(DataFormat::U8(&self.buffer[offset..offset + size]))
                    .await?;
            }
        }

        self.dirty_tracker.clear_all_dirty();
        Ok(())
    }

    /// Force full screen update (row-major layout, single transfer)
    #[cfg(not(feature = "delta-updates"))]
    pub async fn flush_full(&mut self) -> Result<(), DisplayError> {
        let (width, height) = self.effective_dimensions();

        // Set address window to full screen
        self.set_address_window(0, 0, width - 1, height - 1)
            .await?;

        // Start memory write
        Command::MemoryWrite
            .send(&mut self.properties.iface)
            .await?;

        // Single contiguous transfer - entire framebuffer
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

        let idx = self.pixel_offset(x, y);
        if idx + 1 < self.buffer.len() {
            self.buffer[idx] = (color >> 8) as u8;
            self.buffer[idx + 1] = (color & 0xff) as u8;

            // Mark tile dirty (delta-updates only)
            #[cfg(feature = "delta-updates")]
            self.dirty_tracker.mark_pixel_dirty(x, y);
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

    // ========== CONFIGURATION METHODS (delta-updates only) ==========

    /// Set the threshold for automatic full update (0.0-1.0)
    #[cfg(feature = "delta-updates")]
    pub fn set_delta_threshold(&mut self, threshold: f32) {
        self.config.full_update_threshold = threshold.clamp(0.0, 1.0);
    }

    /// Set the merge distance for nearby dirty regions (in tiles)
    #[cfg(feature = "delta-updates")]
    pub fn set_merge_distance(&mut self, distance: u16) {
        self.config.merge_distance = distance;
    }

    /// Force all tiles to be marked as dirty
    #[cfg(feature = "delta-updates")]
    pub fn force_full_dirty(&mut self) {
        self.dirty_tracker.mark_all_dirty();
    }

    /// Get the current dirty ratio (0.0-1.0)
    #[cfg(feature = "delta-updates")]
    pub fn get_dirty_ratio(&self) -> f32 {
        self.dirty_tracker.get_dirty_ratio()
    }
}

impl<'a, DI, const TILE_SIZE: usize> GraphicsMode<'a, Nv3007_168_428, DI, TILE_SIZE>
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

        // Mark all tiles dirty for first flush
        #[cfg(feature = "delta-updates")]
        self.dirty_tracker.mark_all_dirty();

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
impl<'a, DV, DI, const TILE_SIZE: usize> DrawTarget for GraphicsMode<'a, DV, DI, TILE_SIZE>
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
impl<'a, DV, DI, const TILE_SIZE: usize> OriginDimensions for GraphicsMode<'a, DV, DI, TILE_SIZE>
where
    DI: AsyncWriteOnlyDataCommand,
    DV: display::DisplayVariant,
{
    fn size(&self) -> Size {
        let (width, height) = self.effective_dimensions();
        Size::new(width as u32, height as u32)
    }
}
