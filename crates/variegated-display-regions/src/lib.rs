#![no_std]
#![warn(missing_docs)]

//! Frame differencing for a double-buffered RGB565 display.
//!
//! Given the framebuffer a frame was drawn into and the one the panel is currently showing,
//! work out the smallest useful set of rectangles that have to be sent.
//!
//! # Why this is a crate and not a module of the driver
//!
//! The same reason `variegated-scale-codec` and `variegated-ulanzi-codec` are:
//! `variegated-nv3007` cannot host a test binary. It sets `[lib] test = false` and carries
//! `embassy-rp` and `cortex-m-rt` as dev-dependencies, so `cargo test` on it fails compiling
//! `embassy-rp` for the host (`cannot find msplim in register` -- the RP2350's stack-limit
//! register does not exist on the host's target). Its `#[cfg(test)] mod tests` therefore
//! never ran.
//!
//! This is pure index arithmetic over two byte slices, so it lives where it can be run.
//!
//! # Why the output is row bands and not rectangles
//!
//! The driver sends a full-width region as **one** contiguous transfer, and a partial-width
//! region **row by row** -- one `send_data` per row, each with its own bus lock and DMA
//! setup. That asymmetry decides the whole design.
//!
//! This started as arbitrary rectangles, which is the obvious thing and is wrong here. On a
//! real menu frame the diff came back as about fifty rectangles: text fragments a row into
//! runs, and runs on neighbouring rows do not line up. Fifty partial-width rectangles of
//! ~18 rows each is ~900 transfers, which is worse than simply resending the screen -- so
//! the driver correctly fell back to a full repaint every single frame, and the delta path
//! never ran at all.
//!
//! Bands sidestep it. Horizontal detail is exactly the part the hardware refuses to make
//! cheap, so this does not track it: a row either changed or it did not, and contiguous
//! changed rows become one full-width band sent as one transfer. A two-row menu selection
//! move is then 36 rows -- 30,816 bytes against the framebuffer's 143,808.
//!
//! The cost is sending untouched pixels either side of a narrow change. That is the right
//! trade for text and gauges, where a row that changes usually changes across its width;
//! it would be the wrong one for a display whose updates are tall and narrow, and on a much
//! faster bus the per-transfer overhead stops dominating and rectangles become worth their
//! complexity again. The arithmetic is written out here so the next person can redo it
//! rather than guess at it.

// The tests build framebuffers of a real panel's size, which wants an allocator. The crate
// itself stays `no_std`.
#[cfg(test)]
extern crate std;

/// The most bands a single frame may be described by.
///
/// A band needs at least one unchanged row to separate it from the next, so a panel of
/// `H` rows cannot produce more than `H / 2` bands however busy it is -- 84 for the GS3's
/// 168. This is sized past that on purpose: the bound is structural, and a cap that cannot
/// be reached needs no degradation path to get wrong.
pub const MAX_REGIONS: usize = 128;

/// A rectangular region in pixel coordinates, inclusive on both bounds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Region {
    /// Starting X coordinate (inclusive).
    pub x0: u16,
    /// Starting Y coordinate (inclusive).
    pub y0: u16,
    /// Ending X coordinate (inclusive).
    pub x1: u16,
    /// Ending Y coordinate (inclusive).
    pub y1: u16,
}

impl Region {
    /// Create a new region.
    pub const fn new(x0: u16, y0: u16, x1: u16, y1: u16) -> Self {
        Self { x0, y0, x1, y1 }
    }

    /// Whether this region spans the full display width.
    ///
    /// The driver's fast path: a full-width region is one contiguous run of the framebuffer
    /// and goes out as a single transfer.
    pub const fn is_full_width(&self, width: u16) -> bool {
        self.x0 == 0 && self.x1 == width - 1
    }

    /// The number of pixels this region covers.
    pub const fn pixel_count(&self) -> usize {
        ((self.x1 - self.x0 + 1) as usize) * ((self.y1 - self.y0 + 1) as usize)
    }

    /// The bounding box of this region and another.
    pub const fn merge(&self, other: &Self) -> Self {
        Self {
            x0: if self.x0 < other.x0 { self.x0 } else { other.x0 },
            y0: if self.y0 < other.y0 { self.y0 } else { other.y0 },
            x1: if self.x1 > other.x1 { self.x1 } else { other.x1 },
            y1: if self.y1 > other.y1 { self.y1 } else { other.y1 },
        }
    }

    /// Rows this region spans.
    pub const fn row_count(&self) -> u16 {
        self.y1 - self.y0 + 1
    }
}

/// Change detection between two framebuffers of a known geometry.
///
/// Holds no buffers: the caller owns those and passes them in, which is what lets the same
/// tracker serve a double-buffered display without this crate knowing where the memory
/// lives. On the GS3 it lives in PSRAM, and every byte read here crosses the QMI -- which is
/// why the scan below skips four pixels at a time and why [`DoubleBuffer::sync_previous`]
/// copies only what was sent.
pub struct DoubleBuffer {
    width: u16,
    height: u16,
}

impl DoubleBuffer {
    /// Create a tracker for a display of this pixel geometry.
    pub const fn new(width: u16, height: u16) -> Self {
        Self { width, height }
    }

    /// The full-width row bands that differ between `current` and `previous`.
    ///
    /// Empty when the two are identical. Each band is a maximal run of consecutive rows in
    /// which at least one pixel differs, and every band spans the full width -- see the
    /// module docs for why horizontal detail is deliberately not tracked.
    ///
    /// One slice comparison per row, and nothing else. Comparing two 856-byte rows is a
    /// `memcmp` the compiler turns into word-at-a-time work; the rectangle-based version
    /// this replaced walked every pixel of the panel individually and then ran up to ten
    /// O(n²) merge passes over the result.
    pub fn detect_changes(
        &self,
        current: &[u8],
        previous: &[u8],
    ) -> heapless::Vec<Region, MAX_REGIONS> {
        let mut regions: heapless::Vec<Region, MAX_REGIONS> = heapless::Vec::new();
        let row_stride = (self.width as usize) * 2;
        let last_column = self.width - 1;

        // The band being extended, if the previous row changed.
        let mut open: Option<Region> = None;

        for y in 0..self.height {
            let row_offset = (y as usize) * row_stride;
            let row_end = row_offset + row_stride;
            if row_end > current.len() || row_end > previous.len() {
                break;
            }

            let changed = current[row_offset..row_end] != previous[row_offset..row_end];

            match (&mut open, changed) {
                // Extend the band through this row.
                (Some(band), true) => band.y1 = y,
                // The band ended on the row above.
                (Some(_), false) => {
                    // Cannot fail in practice -- a band needs a gap row after it, so a panel
                    // of `height` rows yields at most `height / 2` bands and `MAX_REGIONS`
                    // is sized past that. Dropped rather than asserted if it ever does: a
                    // lost band is a stale strip on the panel, where a panic is a dead
                    // machine.
                    let _ = regions.push(open.take().expect("matched Some"));
                }
                (None, true) => open = Some(Region::new(0, y, last_column, y)),
                (None, false) => {}
            }
        }

        if let Some(band) = open.take() {
            let _ = regions.push(band);
        }

        regions
    }

    /// Whether sending `regions` is worse than simply sending the whole framebuffer.
    ///
    /// A full update is one contiguous transfer, so it wins once enough of the screen has
    /// changed that the rectangles stop saving bytes.
    pub fn should_full_update(&self, regions: &[Region]) -> bool {
        if regions.is_empty() {
            return false;
        }

        // By rows, because rows are what gets sent. Every band is full-width and costs one
        // transfer, so the only thing separating the delta path from the full path is how
        // many rows go out -- a frame is not more expensive for being detailed, and the
        // count of bands does not enter into it.
        //
        // 70% is where the saving stops being worth the extra transfers and the gap rows'
        // address-window commands. Below it the delta always wins; a fraction of the panel
        // costs that fraction of the wire.
        let changed_rows: usize = regions.iter().map(|r| r.row_count() as usize).sum();
        changed_rows * 10 > (self.height as usize) * 7
    }

    /// Bring `previous` up to date with what was actually sent.
    ///
    /// `full` means the whole framebuffer went out and `previous` becomes a copy of
    /// `current`; otherwise only `regions` were sent, and only those are copied. The rest of
    /// `previous` already matches by definition -- it is what the panel is still showing.
    ///
    /// The implementation this replaced copied all 143,808 bytes on every frame, including
    /// frames where nothing had changed at all.
    pub fn sync_previous(
        &self,
        current: &[u8],
        previous: &mut [u8],
        regions: &[Region],
        full: bool,
    ) {
        let len = current.len().min(previous.len());

        if full {
            previous[..len].copy_from_slice(&current[..len]);
            return;
        }

        let row_stride = (self.width as usize) * 2;
        for region in regions {
            let span = ((region.x1 - region.x0 + 1) as usize) * 2;
            for y in region.y0..=region.y1 {
                let offset = (y as usize) * row_stride + (region.x0 as usize) * 2;
                if offset + span > len {
                    break;
                }
                previous[offset..offset + span].copy_from_slice(&current[offset..offset + span]);
            }
        }
    }

    /// Bytes the driver will send for this set of rectangles.
    ///
    /// Reported as an indicator by the firmware, so a flush time can be read against the
    /// traffic that caused it rather than guessed at.
    pub fn bytes_for(&self, regions: &[Region], full: bool) -> u32 {
        if full {
            return (self.width as u32) * (self.height as u32) * 2;
        }
        regions.iter().map(|r| r.pixel_count() as u32 * 2).sum()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The GS3's panel after rotation: 428 columns, 168 rows.
    const W: u16 = 428;
    const H: u16 = 168;

    fn blank() -> std::vec::Vec<u8> {
        std::vec![0u8; (W as usize) * (H as usize) * 2]
    }

    fn set_pixel(buf: &mut [u8], x: u16, y: u16, colour: u16) {
        let offset = ((y as usize) * (W as usize) + (x as usize)) * 2;
        buf[offset] = (colour >> 8) as u8;
        buf[offset + 1] = (colour & 0xff) as u8;
    }

    fn fill_rows(buf: &mut [u8], y0: u16, y1: u16, colour: u16) {
        for y in y0..=y1 {
            for x in 0..W {
                set_pixel(buf, x, y, colour);
            }
        }
    }

    fn tracker() -> DoubleBuffer {
        DoubleBuffer::new(W, H)
    }

    /// Identical buffers produce no work at all.
    #[test]
    fn an_unchanged_frame_has_no_regions() {
        let previous = blank();
        let current = previous.clone();
        assert!(tracker().detect_changes(&current, &previous).is_empty());
    }

    /// One changed pixel is one band, one row tall and the full width.
    ///
    /// The width is the deliberate part: the driver sends a full-width band as a single
    /// contiguous transfer, and the columns either side of the changed pixel are cheaper to
    /// resend than to address separately.
    #[test]
    fn a_single_pixel_is_one_full_width_row() {
        let previous = blank();
        let mut current = previous.clone();
        set_pixel(&mut current, 100, 50, 0xF800);

        let regions = tracker().detect_changes(&current, &previous);
        assert_eq!(regions.as_slice(), &[Region::new(0, 50, W - 1, 50)]);
    }

    /// Consecutive changed rows are one band, however many there are.
    #[test]
    fn consecutive_changed_rows_are_one_band() {
        let previous = blank();
        let mut current = previous.clone();
        fill_rows(&mut current, 20, 150, 0x07E0);

        let regions = tracker().detect_changes(&current, &previous);
        assert_eq!(regions.as_slice(), &[Region::new(0, 20, W - 1, 150)]);
    }

    /// **The frame this rewrite exists for: a menu selection moving by one row.**
    ///
    /// The GS3 menu draws four 18px rows from y=21, the selected one on a filled bar. Moving
    /// the selection repaints two of those rows and nothing else. Text fragments each row
    /// into runs that do not line up between rows, which is what made the rectangle-based
    /// diff return ~50 partial-width rectangles -- roughly 900 row transfers, worse than
    /// resending the screen, so the driver fell back to a full repaint every frame.
    ///
    /// As bands it is two transfers of 18 rows each.
    #[test]
    fn a_menu_selection_move_is_two_small_bands() {
        const ROW_H: u16 = 18;
        const FIRST_ROW_Y: u16 = 21;

        // Text scattered across a row, the way glyphs land: short runs, unaligned between
        // rows, with untouched columns between them.
        let paint_texty_row = |buf: &mut [u8], top: u16, seed: u16| {
            for y in top..top + ROW_H {
                for run in 0..12u16 {
                    let x0 = 6 + run * 34 + (y % 3) + seed;
                    for x in x0..(x0 + 9).min(W - 1) {
                        set_pixel(buf, x, y, 0xFFFF);
                    }
                }
            }
        };

        let mut previous = blank();
        let mut current = blank();
        // Four rows of text, identical in both frames...
        for row in 0..4u16 {
            let top = FIRST_ROW_Y + row * ROW_H;
            paint_texty_row(&mut previous, top, 0);
            paint_texty_row(&mut current, top, 0);
        }
        // ...except that the selection bar moves from row 0 to row 1.
        fill_rows(&mut previous, FIRST_ROW_Y, FIRST_ROW_Y + ROW_H - 1, 0x0001);
        fill_rows(&mut current, FIRST_ROW_Y + ROW_H, FIRST_ROW_Y + 2 * ROW_H - 1, 0x0001);

        let t = tracker();
        let regions = t.detect_changes(&current, &previous);

        assert_eq!(
            regions.as_slice(),
            &[Region::new(0, FIRST_ROW_Y, W - 1, FIRST_ROW_Y + 2 * ROW_H - 1)],
            "the two repainted rows are adjacent, so they coalesce into one band"
        );
        assert!(
            !t.should_full_update(&regions),
            "36 of 168 rows must stay a delta"
        );
        assert_eq!(t.bytes_for(&regions, false), 36 * 428 * 2);
    }

    /// Bands separated by untouched rows stay separate.
    #[test]
    fn bands_separated_by_unchanged_rows_do_not_merge() {
        let previous = blank();
        let mut current = previous.clone();
        fill_rows(&mut current, 10, 20, 0xFFFF);
        fill_rows(&mut current, 40, 50, 0xFFFF);

        let regions = tracker().detect_changes(&current, &previous);
        assert_eq!(
            regions.as_slice(),
            &[Region::new(0, 10, W - 1, 20), Region::new(0, 40, W - 1, 50)]
        );
    }

    /// Changes on the same rows but far apart horizontally cost one band, not two.
    ///
    /// The property that makes this design work on a fragmented frame: horizontal detail
    /// does not multiply the transfer count.
    #[test]
    fn horizontally_scattered_changes_stay_one_band_per_row_run() {
        let previous = blank();
        let mut current = previous.clone();
        for y in 10..=20 {
            for x in 0..=30 {
                set_pixel(&mut current, x, y, 0xFFFF);
            }
            for x in 300..=330 {
                set_pixel(&mut current, x, y, 0xFFFF);
            }
        }

        let regions = tracker().detect_changes(&current, &previous);
        assert_eq!(regions.as_slice(), &[Region::new(0, 10, W - 1, 20)]);
    }

    /// The band count cannot exceed what the panel's height structurally allows.
    ///
    /// Every band needs an unchanged row after it, so alternating rows is the worst case:
    /// 84 bands on a 168-row panel, inside `MAX_REGIONS`. This is why the detector has no
    /// overflow path -- the old rectangle-based one did, and getting it wrong was what made
    /// a tall change silently become a full repaint.
    #[test]
    fn alternating_rows_produce_the_structural_maximum_of_bands() {
        let previous = blank();
        let mut current = previous.clone();
        let mut y = 0;
        while y < H {
            fill_rows(&mut current, y, y, 0xFFFF);
            y += 2;
        }

        let regions = tracker().detect_changes(&current, &previous);
        assert_eq!(regions.len(), (H as usize).div_ceil(2));
        assert!(regions.len() <= MAX_REGIONS);
    }

    /// Every changed pixel ends up inside some rectangle.
    ///
    /// The property that matters for correctness: whatever the rectangles look like, missing
    /// one changed pixel leaves the panel showing something stale.
    #[test]
    fn every_changed_pixel_is_covered() {
        let previous = blank();
        let mut current = previous.clone();

        // An awkward scatter: single pixels, a diagonal, and a block.
        let mut changed = std::vec::Vec::new();
        for i in 0..40u16 {
            let (x, y) = (i * 7 % W, i * 3 % H);
            set_pixel(&mut current, x, y, 0xABCD);
            changed.push((x, y));
        }
        for y in 60..=70 {
            for x in 200..=260 {
                set_pixel(&mut current, x, y, 0x1234);
                changed.push((x, y));
            }
        }

        let regions = tracker().detect_changes(&current, &previous);
        for (x, y) in changed {
            assert!(
                regions
                    .iter()
                    .any(|r| x >= r.x0 && x <= r.x1 && y >= r.y0 && y <= r.y1),
                "pixel ({x}, {y}) was changed but no region covers it"
            );
        }
    }

    /// A full repaint is recognised as one.
    #[test]
    fn a_whole_screen_change_asks_for_a_full_update() {
        let previous = blank();
        let mut current = previous.clone();
        fill_rows(&mut current, 0, H - 1, 0xFFFF);

        let t = tracker();
        let regions = t.detect_changes(&current, &previous);
        assert!(t.should_full_update(&regions));
    }

    /// `sync_previous` copies only what was sent, and that is enough.
    ///
    /// After syncing the regions a flush sent, a second diff of the same frame must find
    /// nothing -- which is the invariant that lets the copy be partial at all.
    #[test]
    fn syncing_only_the_sent_regions_leaves_the_buffers_agreeing() {
        let mut previous = blank();
        let mut current = previous.clone();
        fill_rows(&mut current, 30, 60, 0x07E0);
        for y in 100..=110 {
            for x in 5..=25 {
                set_pixel(&mut current, x, y, 0xF81F);
            }
        }

        let t = tracker();
        let regions = t.detect_changes(&current, &previous);
        assert!(!t.should_full_update(&regions));

        t.sync_previous(&current, &mut previous, &regions, false);

        assert!(
            t.detect_changes(&current, &previous).is_empty(),
            "a partial sync must leave previous matching current everywhere"
        );
        assert_eq!(previous, current);
    }

    /// A full sync copies everything.
    #[test]
    fn a_full_sync_copies_the_whole_buffer() {
        let mut previous = blank();
        let mut current = previous.clone();
        fill_rows(&mut current, 0, H - 1, 0x4444);

        let t = tracker();
        t.sync_previous(&current, &mut previous, &[], true);
        assert_eq!(previous, current);
    }

    /// A partial sync writes strictly inside the rectangles it is given.
    ///
    /// The sentinel sits outside every rectangle passed, so a sync that touched it would be
    /// copying bytes nobody asked to send. With an empty list nothing may be written at all
    /// -- which is the idle frame, where the old code copied all 143,808 bytes over the QMI
    /// every time round the loop.
    #[test]
    fn a_partial_sync_writes_only_inside_its_regions() {
        let mut previous = blank();
        let mut current = previous.clone();
        fill_rows(&mut current, 30, 40, 0x07E0);
        // Differs from `current`, and lies outside the rectangle synced below.
        previous[0] = 0xAA;

        let t = tracker();
        t.sync_previous(&current, &mut previous, &[], false);
        assert_eq!(previous[0], 0xAA, "an empty region list must write nothing");

        t.sync_previous(&current, &mut previous, &[Region::new(0, 30, W - 1, 40)], false);
        assert_eq!(previous[0], 0xAA, "row 0 is outside the synced rectangle");

        // ...and the rectangle it *was* given did get copied.
        let row_30 = (30 * W as usize) * 2;
        assert_eq!(previous[row_30..row_30 + 2], current[row_30..row_30 + 2]);
    }

    /// The byte count matches what the rectangles say.
    #[test]
    fn bytes_for_counts_the_rectangles_and_the_full_frame() {
        let t = tracker();
        assert_eq!(
            t.bytes_for(&[], true),
            (W as u32) * (H as u32) * 2
        );
        assert_eq!(
            t.bytes_for(&[Region::new(0, 0, 9, 9)], false),
            100 * 2
        );
    }

    /// Full-width detection drives the driver's single-transfer path.
    #[test]
    fn full_width_regions_are_recognised() {
        assert!(Region::new(0, 5, W - 1, 9).is_full_width(W));
        assert!(!Region::new(1, 5, W - 1, 9).is_full_width(W));
        assert!(!Region::new(0, 5, W - 2, 9).is_full_width(W));
    }

    /// A change in the very last pixel of the panel is still found.
    ///
    /// The bottom-right corner is where two off-by-ones show up: a row loop that stops one
    /// row short never looks at row 167, and a row slice one pixel short never sees column
    /// 427. Both would lose an edge of the panel silently -- the display simply keeps
    /// showing something stale there, with nothing to log.
    #[test]
    fn the_last_pixel_of_the_panel_is_scanned() {
        let previous = blank();
        let mut current = previous.clone();
        set_pixel(&mut current, W - 1, H - 1, 0x8888);

        let regions = tracker().detect_changes(&current, &previous);
        assert_eq!(regions.as_slice(), &[Region::new(0, H - 1, W - 1, H - 1)]);
    }
}
