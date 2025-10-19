//! Dirty region tracking for delta updates

/// Calculate number of tiles needed for a dimension
const fn calc_tiles(dimension: u16, tile_size: u16) -> u16 {
    (dimension + tile_size - 1) / tile_size
}

/// Calculate number of bytes needed for bitmap
const fn calc_bitmap_bytes(num_tiles: usize) -> usize {
    (num_tiles + 7) / 8
}

/// A rectangular region of tiles
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TileRegion {
    /// Starting tile X coordinate (inclusive)
    pub x0: u16,
    /// Starting tile Y coordinate (inclusive)
    pub y0: u16,
    /// Ending tile X coordinate (inclusive)
    pub x1: u16,
    /// Ending tile Y coordinate (inclusive)
    pub y1: u16,
}

impl TileRegion {
    /// Create a new tile region
    pub fn new(x0: u16, y0: u16, x1: u16, y1: u16) -> Self {
        Self { x0, y0, x1, y1 }
    }

    /// Get the number of tiles in this region
    pub fn tile_count(&self) -> usize {
        ((self.x1 - self.x0 + 1) * (self.y1 - self.y0 + 1)) as usize
    }

    /// Check if this region is adjacent or overlapping with another
    fn can_merge_with(&self, other: &Self, merge_distance: u16) -> bool {
        // Check if regions are within merge_distance tiles of each other
        let x_gap = if self.x1 < other.x0 {
            other.x0.saturating_sub(self.x1).saturating_sub(1)
        } else if other.x1 < self.x0 {
            self.x0.saturating_sub(other.x1).saturating_sub(1)
        } else {
            0 // Overlapping or adjacent
        };

        let y_gap = if self.y1 < other.y0 {
            other.y0.saturating_sub(self.y1).saturating_sub(1)
        } else if other.y1 < self.y0 {
            self.y0.saturating_sub(other.y1).saturating_sub(1)
        } else {
            0 // Overlapping or adjacent
        };

        x_gap <= merge_distance && y_gap <= merge_distance
    }

    /// Merge with another region (creates bounding box)
    fn merge(&self, other: &Self) -> Self {
        Self {
            x0: self.x0.min(other.x0),
            y0: self.y0.min(other.y0),
            x1: self.x1.max(other.x1),
            y1: self.y1.max(other.y1),
        }
    }
}

/// Dirty tile tracker with configurable tile size
pub struct DirtyTracker<const TILE_SIZE: usize> {
    tiles_x: u16,
    tiles_y: u16,
    tile_bitmap: heapless::Vec<u8, 256>, // Max 256 bytes for bitmap (supports up to 2048 tiles)
    tile_checksums: heapless::Vec<u32, 2048>, // Checksum for each tile to detect content changes
}

impl<const TILE_SIZE: usize> DirtyTracker<TILE_SIZE> {
    /// Create a new dirty tracker for the given display dimensions
    pub fn new(width: u16, height: u16) -> Self {
        let tiles_x = calc_tiles(width, TILE_SIZE as u16);
        let tiles_y = calc_tiles(height, TILE_SIZE as u16);
        let total_tiles = (tiles_x as usize) * (tiles_y as usize);
        let bitmap_size = calc_bitmap_bytes(total_tiles);

        let mut tile_bitmap = heapless::Vec::new();
        tile_bitmap.resize(bitmap_size, 0).ok();

        let mut tile_checksums = heapless::Vec::new();
        tile_checksums.resize(total_tiles, 0).ok(); // Initialize all checksums to 0

        Self {
            tiles_x,
            tiles_y,
            tile_bitmap,
            tile_checksums,
        }
    }

    /// Mark a pixel as dirty (marks the containing tile)
    pub fn mark_pixel_dirty(&mut self, x: u16, y: u16) {
        let tile_x = x / TILE_SIZE as u16;
        let tile_y = y / TILE_SIZE as u16;
        self.mark_tile_dirty(tile_x, tile_y);
    }

    /// Mark a rectangular region of pixels as dirty
    pub fn mark_region_dirty(&mut self, x: u16, y: u16, w: u16, h: u16) {
        let tile_x0 = x / TILE_SIZE as u16;
        let tile_y0 = y / TILE_SIZE as u16;
        let tile_x1 = (x + w - 1) / TILE_SIZE as u16;
        let tile_y1 = (y + h - 1) / TILE_SIZE as u16;

        for ty in tile_y0..=tile_y1 {
            for tx in tile_x0..=tile_x1 {
                self.mark_tile_dirty(tx, ty);
            }
        }
    }

    /// Mark a specific tile as dirty
    pub fn mark_tile_dirty(&mut self, tile_x: u16, tile_y: u16) {
        if tile_x >= self.tiles_x || tile_y >= self.tiles_y {
            return;
        }

        let tile_idx = (tile_y * self.tiles_x + tile_x) as usize;
        let byte_idx = tile_idx / 8;
        let bit_idx = tile_idx % 8;

        if byte_idx < self.tile_bitmap.len() {
            self.tile_bitmap[byte_idx] |= 1 << bit_idx;
        }
    }

    /// Check if a specific tile is dirty
    pub fn is_tile_dirty(&self, tile_x: u16, tile_y: u16) -> bool {
        if tile_x >= self.tiles_x || tile_y >= self.tiles_y {
            return false;
        }

        let tile_idx = (tile_y * self.tiles_x + tile_x) as usize;
        let byte_idx = tile_idx / 8;
        let bit_idx = tile_idx % 8;

        if byte_idx < self.tile_bitmap.len() {
            (self.tile_bitmap[byte_idx] & (1 << bit_idx)) != 0
        } else {
            false
        }
    }

    /// Get the ratio of dirty tiles (0.0 to 1.0)
    pub fn get_dirty_ratio(&self) -> f32 {
        let total_tiles = (self.tiles_x as usize) * (self.tiles_y as usize);
        let dirty_count = self.tile_bitmap.iter()
            .map(|byte| byte.count_ones() as usize)
            .sum::<usize>();
        dirty_count as f32 / total_tiles as f32
    }

    /// Clear all dirty flags
    pub fn clear_all_dirty(&mut self) {
        self.tile_bitmap.iter_mut().for_each(|byte| *byte = 0);
    }

    /// Mark all tiles as dirty
    pub fn mark_all_dirty(&mut self) {
        self.tile_bitmap.iter_mut().for_each(|byte| *byte = 0xFF);
    }

    /// Get dirty regions with optional merging
    pub fn get_dirty_regions(&self, merge_distance: u16) -> heapless::Vec<TileRegion, 32> {
        let mut regions = heapless::Vec::new();

        // First pass: find all dirty tiles and group into horizontal runs
        for ty in 0..self.tiles_y {
            let mut run_start: Option<u16> = None;

            for tx in 0..=self.tiles_x {
                let is_dirty = if tx < self.tiles_x {
                    self.is_tile_dirty(tx, ty)
                } else {
                    false // End of row
                };

                match (run_start, is_dirty) {
                    (None, true) => {
                        // Start new run
                        run_start = Some(tx);
                    }
                    (Some(start), false) => {
                        // End run, create region
                        let region = TileRegion::new(start, ty, tx - 1, ty);
                        if regions.push(region).is_err() {
                            // Ran out of space, return what we have
                            return regions;
                        }
                        run_start = None;
                    }
                    _ => {}
                }
            }
        }

        // Second pass: merge regions if they're close enough
        if merge_distance > 0 {
            regions = self.merge_regions(regions, merge_distance);
        }

        regions
    }

    /// Merge nearby regions
    fn merge_regions(
        &self,
        mut regions: heapless::Vec<TileRegion, 32>,
        merge_distance: u16,
    ) -> heapless::Vec<TileRegion, 32> {
        let mut merged = true;

        while merged && regions.len() > 1 {
            merged = false;
            let mut new_regions = heapless::Vec::new();
            let mut i = 0;

            while i < regions.len() {
                let mut current = regions[i];
                let mut j = i + 1;

                // Try to merge with subsequent regions
                while j < regions.len() {
                    if current.can_merge_with(&regions[j], merge_distance) {
                        // Calculate cost of merging
                        let merged_region = current.merge(&regions[j]);
                        let current_pixels = current.tile_count() + regions[j].tile_count();
                        let merged_pixels = merged_region.tile_count();

                        // Only merge if we don't add too many extra tiles
                        // (2x overhead threshold - configurable)
                        if merged_pixels <= current_pixels * 2 {
                            current = merged_region;
                            regions.remove(j);
                            merged = true;
                        } else {
                            j += 1;
                        }
                    } else {
                        j += 1;
                    }
                }

                if new_regions.push(current).is_err() {
                    // Out of space, keep what we have
                    return new_regions;
                }
                i += 1;
            }

            regions = new_regions;
        }

        regions
    }

    /// Get tile grid dimensions
    pub fn tiles_x(&self) -> u16 {
        self.tiles_x
    }

    /// Get tile grid dimensions
    pub fn tiles_y(&self) -> u16 {
        self.tiles_y
    }

    /// Calculate checksum for a tile's data using Adler-32
    ///
    /// Adler-32 provides excellent collision resistance while remaining fast.
    /// It maintains two 16-bit sums (A and B) that are combined into a 32-bit checksum.
    /// This is significantly better than XOR for detecting changes in pixel data.
    ///
    /// For an 8×8 RGB565 tile (128 bytes), this performs approximately 128 additions
    /// and 2 modulo operations, which is still very fast compared to SPI transfer time.
    pub fn calculate_tile_checksum(tile_data: &[u8]) -> u32 {
        const ADLER_MOD: u32 = 65521; // Largest prime less than 2^16

        let mut a: u32 = 1; // Initialize to 1 (Adler-32 convention)
        let mut b: u32 = 0;

        // Process each byte
        for &byte in tile_data {
            a = (a + byte as u32) % ADLER_MOD;
            b = (b + a) % ADLER_MOD;
        }

        // Combine into 32-bit checksum: (B << 16) | A
        (b << 16) | a
    }

    /// Check if a tile's content has changed and update stored checksum
    /// Returns true if the tile content is different from the last known state
    pub fn tile_content_changed(&mut self, tile_x: u16, tile_y: u16, tile_data: &[u8]) -> bool {
        if tile_x >= self.tiles_x || tile_y >= self.tiles_y {
            return false;
        }

        let tile_idx = (tile_y * self.tiles_x + tile_x) as usize;
        if tile_idx >= self.tile_checksums.len() {
            return false;
        }

        let new_checksum = Self::calculate_tile_checksum(tile_data);
        let old_checksum = self.tile_checksums[tile_idx];

        if new_checksum != old_checksum {
            // Content changed - update stored checksum
            self.tile_checksums[tile_idx] = new_checksum;
            true
        } else {
            // Content unchanged
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_calc_tiles() {
        assert_eq!(calc_tiles(168, 8), 21);
        assert_eq!(calc_tiles(428, 8), 54);
        assert_eq!(calc_tiles(100, 8), 13);
    }

    #[test]
    fn test_dirty_tracker_basic() {
        let mut tracker = DirtyTracker::<8>::new(168, 428);

        assert!(!tracker.is_tile_dirty(0, 0));
        tracker.mark_tile_dirty(0, 0);
        assert!(tracker.is_tile_dirty(0, 0));

        tracker.clear_all_dirty();
        assert!(!tracker.is_tile_dirty(0, 0));
    }

    #[test]
    fn test_dirty_ratio() {
        let mut tracker = DirtyTracker::<8>::new(168, 428);

        assert_eq!(tracker.get_dirty_ratio(), 0.0);

        tracker.mark_all_dirty();
        assert_eq!(tracker.get_dirty_ratio(), 1.0);
    }

    #[test]
    fn test_region_merging() {
        let region1 = TileRegion::new(0, 0, 5, 0);
        let region2 = TileRegion::new(7, 0, 10, 0);

        assert!(region1.can_merge_with(&region2, 2));
        assert!(!region1.can_merge_with(&region2, 0));

        let merged = region1.merge(&region2);
        assert_eq!(merged, TileRegion::new(0, 0, 10, 0));
    }
}
