//! Double-buffering and region tracking for efficient delta updates

/// A rectangular region in pixel coordinates
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Region {
    /// Starting X coordinate (inclusive)
    pub x0: u16,
    /// Starting Y coordinate (inclusive)
    pub y0: u16,
    /// Ending X coordinate (inclusive)
    pub x1: u16,
    /// Ending Y coordinate (inclusive)
    pub y1: u16,
}

impl Region {
    /// Create a new region
    pub fn new(x0: u16, y0: u16, x1: u16, y1: u16) -> Self {
        Self { x0, y0, x1, y1 }
    }

    /// Check if region spans the full display width
    pub fn is_full_width(&self, width: u16) -> bool {
        self.x0 == 0 && self.x1 == width - 1
    }

    /// Get the number of pixels in this region
    pub fn pixel_count(&self) -> usize {
        ((self.x1 - self.x0 + 1) as usize) * ((self.y1 - self.y0 + 1) as usize)
    }

    /// Merge with another region (creates bounding box)
    pub fn merge(&self, other: &Self) -> Self {
        Self {
            x0: self.x0.min(other.x0),
            y0: self.y0.min(other.y0),
            x1: self.x1.max(other.x1),
            y1: self.y1.max(other.y1),
        }
    }

    /// Check if regions should be merged based on cost analysis
    ///
    /// Merging is beneficial if the overhead of setting up two separate SPI transfers
    /// is greater than the cost of sending the extra pixels in the merged region.
    ///
    /// overhead_factor: typical value 0.3-0.5 (30-50% overhead acceptable)
    pub fn should_merge_with(&self, other: &Self, overhead_factor: f32) -> bool {
        // Check if regions are close enough to consider merging
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

        // Don't consider merging if gaps are too large (heuristic: max 10 pixels)
        if x_gap > 10 || y_gap > 10 {
            return false;
        }

        // Calculate costs
        let merged = self.merge(other);
        let separate_pixels = self.pixel_count() + other.pixel_count();
        let merged_pixels = merged.pixel_count();
        let extra_pixels = merged_pixels.saturating_sub(separate_pixels);

        // Merge if extra pixels are within acceptable overhead
        extra_pixels as f32 <= (separate_pixels as f32 * overhead_factor)
    }
}

/// Double buffer with change detection
///
/// Tracks state for double-buffering without owning the buffers.
/// Buffers are passed in during detect_changes() to enable pixel-accurate
/// change detection and provide optimized rectangular regions for efficient SPI transfers.
pub struct DoubleBuffer {
    width: u16,
    height: u16,
}

impl DoubleBuffer {
    /// Create a new double buffer tracker
    ///
    /// # Arguments
    /// * `width` - Display width in pixels
    /// * `height` - Display height in pixels
    pub fn new(width: u16, height: u16) -> Self {
        Self { width, height }
    }

    /// Detect changed regions using fast comparison
    ///
    /// Returns a list of optimal rectangular regions that need to be updated.
    /// Uses u64 chunk comparison for fast pixel comparison, then builds
    /// horizontal runs and merges them into rectangles.
    ///
    /// # Arguments
    /// * `current` - Current framebuffer
    /// * `previous` - Previous framebuffer
    pub fn detect_changes(&self, current: &[u8], previous: &[u8]) -> heapless::Vec<Region, 128> {
        let mut regions = heapless::Vec::new();

        // Fast path: check if buffers are identical
        if current == previous {
            return regions; // No changes
        }

        // Build dirty pixel map using fast u64 comparison
        let row_stride = (self.width as usize) * 2; // bytes per row

        for y in 0..self.height {
            let row_offset = (y as usize) * row_stride;
            let row_end = row_offset + row_stride;

            if row_end > current.len() || row_end > previous.len() {
                break;
            }

            let current_row = &current[row_offset..row_end];
            let previous_row = &previous[row_offset..row_end];

            // Find changed pixels in this row using u64 chunks (4 pixels at once)
            let mut run_start: Option<u16> = None;

            for x in 0..self.width {
                let pixel_offset = (x as usize) * 2;
                let changed = if pixel_offset + 1 < current_row.len() {
                    current_row[pixel_offset] != previous_row[pixel_offset]
                        || current_row[pixel_offset + 1] != previous_row[pixel_offset + 1]
                } else {
                    false
                };

                match (run_start, changed) {
                    (None, true) => {
                        // Start new run
                        run_start = Some(x);
                    }
                    (Some(start), false) => {
                        // End run, create region
                        let region = Region::new(start, y, x - 1, y);
                        if regions.push(region).is_err() {
                            // Hit region limit - fall back to full screen update
                            let mut full_update = heapless::Vec::new();
                            full_update.push(Region::new(0, 0, self.width - 1, self.height - 1)).ok();
                            return full_update;
                        }
                        run_start = None;
                    }
                    _ => {}
                }
            }

            // Handle run extending to end of row
            if let Some(start) = run_start {
                let region = Region::new(start, y, self.width - 1, y);
                if regions.push(region).is_err() {
                    // Hit region limit - fall back to full screen update
                    let mut full_update = heapless::Vec::new();
                    full_update.push(Region::new(0, 0, self.width - 1, self.height - 1)).ok();
                    return full_update;
                }
            }
        }

        // Merge vertically adjacent regions and nearby regions
        self.merge_regions(regions, 0.4)
    }

    /// Merge nearby regions to reduce SPI overhead
    fn merge_regions(
        &self,
        mut regions: heapless::Vec<Region, 128>,
        overhead_factor: f32,
    ) -> heapless::Vec<Region, 128> {
        if regions.len() <= 1 {
            return regions;
        }

        let mut merged_any = true;
        let mut iterations = 0;
        const MAX_ITERATIONS: usize = 10; // Allow more iterations for better vertical merging

        while merged_any && iterations < MAX_ITERATIONS {
            merged_any = false;
            iterations += 1;

            let mut i = 0;
            while i < regions.len() {
                let mut j = i + 1;
                let mut current = regions[i];

                while j < regions.len() {
                    if self.should_merge_regions(&current, &regions[j], overhead_factor) {
                        // Merge regions[j] into current
                        current = current.merge(&regions[j]);
                        regions.remove(j);
                        merged_any = true;
                    } else {
                        j += 1;
                    }
                }

                regions[i] = current;
                i += 1;
            }
        }

        regions
    }

    /// Check if two regions should be merged (considers vertical adjacency and cost)
    fn should_merge_regions(&self, a: &Region, b: &Region, overhead_factor: f32) -> bool {
        // AGGRESSIVE VERTICAL MERGING: Check if Y ranges are adjacent/overlapping
        let y_adjacent = {
            let gap_y = if a.y1 < b.y0 {
                b.y0.saturating_sub(a.y1).saturating_sub(1)
            } else if b.y1 < a.y0 {
                a.y0.saturating_sub(b.y1).saturating_sub(1)
            } else {
                0 // Overlapping
            };
            gap_y <= 2 // Adjacent or very close vertically
        };

        if y_adjacent {
            // Check if X ranges overlap or are very close
            let x_overlap = {
                let gap_x = if a.x1 < b.x0 {
                    b.x0.saturating_sub(a.x1).saturating_sub(1)
                } else if b.x1 < a.x0 {
                    a.x0.saturating_sub(b.x1).saturating_sub(1)
                } else {
                    0 // Overlapping
                };
                gap_x <= 10 // Allow up to 10 pixels gap in X direction
            };

            if x_overlap {
                // Calculate cost of merging
                let merged = a.merge(b);
                let separate_pixels = a.pixel_count() + b.pixel_count();
                let merged_pixels = merged.pixel_count();
                let extra_pixels = merged_pixels.saturating_sub(separate_pixels);

                // More lenient for vertical merges - allow 60% overhead
                // (This helps merge multi-row text/UI elements)
                return extra_pixels as f32 <= (separate_pixels as f32 * 0.6);
            }
        }

        // General horizontal merging (stricter)
        a.should_merge_with(b, overhead_factor)
    }

    /// Check if we should do a full update instead of delta
    ///
    /// Full update is preferred when:
    /// - More than 70% of screen has changed
    /// - Too many small regions (SPI overhead dominates)
    pub fn should_full_update(&self, regions: &[Region]) -> bool {
        if regions.is_empty() {
            return false; // No changes, no update needed
        }

        // Calculate total changed pixels
        let total_changed: usize = regions.iter().map(|r| r.pixel_count()).sum();
        let total_pixels = (self.width as usize) * (self.height as usize);
        let change_ratio = total_changed as f32 / total_pixels as f32;

        // Threshold: >70% changed → full update
        if change_ratio > 0.70 {
            return true;
        }

        // Too many regions (>80) → full update more efficient
        // With aggressive vertical merging, we should rarely hit this limit
        if regions.len() > 80 {
            return true;
        }

        false
    }

    /// Swap buffers (copy current → previous after successful flush)
    ///
    /// # Arguments
    /// * `current` - Current framebuffer
    /// * `previous` - Previous framebuffer (will be updated to match current)
    pub fn swap_buffers(&self, current: &[u8], previous: &mut [u8]) {
        let copy_size = current.len().min(previous.len());
        previous[..copy_size].copy_from_slice(&current[..copy_size]);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_region_full_width() {
        let region = Region::new(0, 0, 167, 10);
        assert!(region.is_full_width(168));
        assert!(!region.is_full_width(200));

        let partial = Region::new(10, 0, 100, 10);
        assert!(!partial.is_full_width(168));
    }

    #[test]
    fn test_region_pixel_count() {
        let region = Region::new(0, 0, 9, 9);
        assert_eq!(region.pixel_count(), 100);

        let region2 = Region::new(5, 5, 14, 14);
        assert_eq!(region2.pixel_count(), 100);
    }

    #[test]
    fn test_region_merge() {
        let r1 = Region::new(0, 0, 10, 10);
        let r2 = Region::new(5, 5, 15, 15);
        let merged = r1.merge(&r2);

        assert_eq!(merged.x0, 0);
        assert_eq!(merged.y0, 0);
        assert_eq!(merged.x1, 15);
        assert_eq!(merged.y1, 15);
    }

    #[test]
    fn test_double_buffer_no_changes() {
        let current = vec![0u8; 168 * 428 * 2];
        let previous = vec![0u8; 168 * 428 * 2];

        let db = DoubleBuffer::new(168, 428);
        let regions = db.detect_changes(&current, &previous);

        assert_eq!(regions.len(), 0);
    }

    #[test]
    fn test_double_buffer_single_pixel() {
        let mut current = vec![0u8; 168 * 428 * 2];
        let previous = vec![0u8; 168 * 428 * 2];

        // Change one pixel at (10, 20)
        let offset = (20 * 168 + 10) * 2;
        current[offset] = 0xFF;
        current[offset + 1] = 0xFF;

        let db = DoubleBuffer::new(168, 428);
        let regions = db.detect_changes(&current, &previous);

        assert_eq!(regions.len(), 1);
        assert_eq!(regions[0].x0, 10);
        assert_eq!(regions[0].y0, 20);
        assert_eq!(regions[0].x1, 10);
        assert_eq!(regions[0].y1, 20);
    }

    #[test]
    fn test_double_buffer_horizontal_line() {
        let mut current = vec![0u8; 168 * 428 * 2];
        let previous = vec![0u8; 168 * 428 * 2];

        // Change entire row 50
        let row_offset = 50 * 168 * 2;
        for i in 0..168 * 2 {
            current[row_offset + i] = 0xFF;
        }

        let db = DoubleBuffer::new(168, 428);
        let regions = db.detect_changes(&current, &previous);

        assert_eq!(regions.len(), 1);
        assert!(regions[0].is_full_width(168));
        assert_eq!(regions[0].y0, 50);
        assert_eq!(regions[0].y1, 50);
    }

    #[test]
    fn test_double_buffer_swap() {
        let mut current = vec![0u8; 100];
        let mut previous = vec![0xFF; 100];

        let db = DoubleBuffer::new(5, 10);

        // Change current
        current[0] = 0xAA;

        // Swap
        db.swap_buffers(&current, &mut previous);

        // Previous should now match current
        assert_eq!(previous[0], 0xAA);
    }

    #[test]
    fn test_should_full_update() {
        let db = DoubleBuffer::new(168, 428);

        // Small change - delta update
        let small_regions = heapless::Vec::from_slice(&[Region::new(0, 0, 10, 10)]).unwrap();
        assert!(!db.should_full_update(&small_regions));

        // Large change - full update
        let large_region = Region::new(0, 0, 167, 350); // ~80% of screen
        let large_regions = heapless::Vec::from_slice(&[large_region]).unwrap();
        assert!(db.should_full_update(&large_regions));

        // Many small regions - full update
        let mut many_regions = heapless::Vec::new();
        for i in 0..25 {
            many_regions.push(Region::new(i * 6, 0, i * 6 + 5, 5)).ok();
        }
        assert!(db.should_full_update(&many_regions));
    }
}
