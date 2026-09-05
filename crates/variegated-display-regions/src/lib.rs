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
//! # What the shape of the output costs
//!
//! The driver sends a full-width region as **one** contiguous transfer, and a partial-width
//! region **row by row**, one transfer per row. At the GS3's 10 MHz that per-transfer
//! overhead is small next to the bytes -- a 100x100 partial region is ~17 ms row-by-row
//! against ~68 ms if it were widened to the panel's 428 columns -- so this code minimises
//! *bytes*, not rectangles. Do not widen regions to reach the contiguous path; on a slower
//! bus that arithmetic reverses, and it is written out here so the next person can redo it
//! rather than guess.

// The tests build framebuffers of a real panel's size, which wants an allocator. The crate
// itself stays `no_std`.
#[cfg(test)]
extern crate std;

/// The most rectangles a single frame may be described by.
///
/// Reaching it is not a cliff. [`DoubleBuffer::detect_changes`] degrades by widening an
/// existing rectangle rather than giving up and redrawing the screen, which is what the
/// implementation this replaced did -- and it did it constantly, because it emitted one
/// rectangle per changed row *before* merging, so any change touching more than 128 rows of
/// a 168-row panel became a full-screen repaint.
pub const MAX_REGIONS: usize = 128;

/// The most separate changed spans one row may contribute before they are coalesced.
///
/// A row with more spans than this has its changes covered by a single span from the first
/// change to the last. That over-sends the gaps, which is cheaper than tracking detail no
/// realistic screen produces: this is text and gauges, not noise.
const MAX_RUNS_PER_ROW: usize = 32;

/// How close two spans must be, vertically, to be treated as one rectangle.
///
/// Rows are visited in order, so a rectangle is extended only from the row immediately
/// above. The tolerance is horizontal: a span may extend a rectangle whose columns it
/// overlaps, or misses by no more than this. Text descends and ascends by a pixel or two
/// between rows, and splitting a glyph into a rectangle per row is what makes a region list
/// explode.
const X_JOIN_TOLERANCE: u16 = 8;

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

    /// Whether a span on the row below could extend this region without widening it much.
    fn accepts_span(&self, span_x0: u16, span_x1: u16) -> bool {
        let gap = if self.x1 < span_x0 {
            span_x0 - self.x1
        } else if span_x1 < self.x0 {
            self.x0 - span_x1
        } else {
            0
        };
        gap <= X_JOIN_TOLERANCE
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

    /// The rectangles that differ between `current` and `previous`.
    ///
    /// Empty when the two are identical. Rectangles are built by scanning each row for
    /// changed spans and extending the previous row's rectangles into this one, so a band of
    /// rows that changed together comes back as one rectangle rather than one per row.
    pub fn detect_changes(
        &self,
        current: &[u8],
        previous: &[u8],
    ) -> heapless::Vec<Region, MAX_REGIONS> {
        let mut regions: heapless::Vec<Region, MAX_REGIONS> = heapless::Vec::new();
        let row_stride = (self.width as usize) * 2;

        // Indices into `regions` for the rectangles left open by the previous row, and the
        // ones this row leaves open. A rectangle not extended by a row is simply never
        // touched again -- there is no close step, because rows are visited in order.
        let mut open_prev: heapless::Vec<usize, MAX_RUNS_PER_ROW> = heapless::Vec::new();
        let mut open_cur: heapless::Vec<usize, MAX_RUNS_PER_ROW> = heapless::Vec::new();
        let mut spans: heapless::Vec<(u16, u16), MAX_RUNS_PER_ROW> = heapless::Vec::new();

        for y in 0..self.height {
            let row_offset = (y as usize) * row_stride;
            let row_end = row_offset + row_stride;
            if row_end > current.len() || row_end > previous.len() {
                break;
            }

            self.row_spans(
                &current[row_offset..row_end],
                &previous[row_offset..row_end],
                &mut spans,
            );

            open_cur.clear();
            if spans.is_empty() {
                // Nothing changed on this row, so no rectangle above it can reach any
                // further down and all of them are closed.
                open_prev.clear();
                continue;
            }

            for &(sx0, sx1) in spans.iter() {
                // The first still-unclaimed rectangle from the row above whose columns this
                // span can join. Claimed by removal, so two spans on this row cannot both
                // extend one rectangle and swallow the gap between them.
                let matched = open_prev
                    .iter()
                    .position(|&ri| regions[ri].accepts_span(sx0, sx1));

                match matched {
                    Some(pos) => {
                        let ri = open_prev.swap_remove(pos);
                        let region = &mut regions[ri];
                        region.y1 = y;
                        region.x0 = region.x0.min(sx0);
                        region.x1 = region.x1.max(sx1);
                        let _ = open_cur.push(ri);
                    }
                    None => {
                        let fresh = Region::new(sx0, y, sx1, y);
                        match regions.push(fresh) {
                            Ok(()) => {
                                let _ = open_cur.push(regions.len() - 1);
                            }
                            Err(_) => {
                                // Out of rectangles. Widen the nearest one rather than
                                // abandoning the delta -- an over-sent gap costs bytes,
                                // where a full-screen repaint costs the whole frame.
                                if let Some(nearest) = self.nearest_region(&regions, &fresh) {
                                    regions[nearest] = regions[nearest].merge(&fresh);
                                    let _ = open_cur.push(nearest);
                                }
                            }
                        }
                    }
                }
            }

            core::mem::swap(&mut open_prev, &mut open_cur);
        }

        regions
    }

    /// Changed spans on one row, as inclusive `(x0, x1)` column pairs.
    ///
    /// Compares four pixels at a time and only looks at individual pixels inside a group
    /// that differs. The implementation this replaced claimed to do this in a comment and
    /// did not: it compared two bytes per pixel with a bounds check inside the inner loop,
    /// over every pixel of the panel, on both buffers, every frame.
    fn row_spans(
        &self,
        current_row: &[u8],
        previous_row: &[u8],
        spans: &mut heapless::Vec<(u16, u16), MAX_RUNS_PER_ROW>,
    ) {
        spans.clear();

        // Whole-row equality first. Most rows of most frames are untouched, and this settles
        // them in one comparison.
        if current_row == previous_row {
            return;
        }

        const GROUP: u16 = 4;
        let mut run_start: Option<u16> = None;
        let mut x: u16 = 0;

        while x < self.width {
            let group = GROUP.min(self.width - x);
            let offset = (x as usize) * 2;
            let len = (group as usize) * 2;

            if current_row[offset..offset + len] == previous_row[offset..offset + len] {
                if let Some(start) = run_start.take() {
                    push_span(spans, start, x - 1);
                }
            } else {
                for i in 0..group {
                    let pixel = offset + (i as usize) * 2;
                    let changed = current_row[pixel] != previous_row[pixel]
                        || current_row[pixel + 1] != previous_row[pixel + 1];
                    let column = x + i;
                    match (run_start, changed) {
                        (None, true) => run_start = Some(column),
                        (Some(start), false) => {
                            push_span(spans, start, column - 1);
                            run_start = None;
                        }
                        _ => {}
                    }
                }
            }

            x += group;
        }

        if let Some(start) = run_start {
            push_span(spans, start, self.width - 1);
        }
    }

    /// The index of the region whose bounding box grows least by absorbing `candidate`.
    fn nearest_region(&self, regions: &[Region], candidate: &Region) -> Option<usize> {
        let mut best: Option<(usize, usize)> = None;
        for (index, region) in regions.iter().enumerate() {
            let cost = region.merge(candidate).pixel_count() - region.pixel_count();
            if best.map(|(_, b)| cost < b).unwrap_or(true) {
                best = Some((index, cost));
            }
        }
        best.map(|(index, _)| index)
    }

    /// Whether sending `regions` is worse than simply sending the whole framebuffer.
    ///
    /// A full update is one contiguous transfer, so it wins once enough of the screen has
    /// changed that the rectangles stop saving bytes.
    pub fn should_full_update(&self, regions: &[Region]) -> bool {
        if regions.is_empty() {
            return false;
        }

        let total_changed: usize = regions.iter().map(|r| r.pixel_count()).sum();
        let total_pixels = (self.width as usize) * (self.height as usize);

        // 70% by pixels. Deliberately not a rectangle count any more: the old rule also
        // forced a full update above 80 rectangles, which punished a frame for being
        // *detailed* rather than for being large, and a partial-width rectangle costs one
        // transfer per row regardless of how many rectangles share the frame.
        total_changed * 10 > total_pixels * 7
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

/// Record a span, coalescing into the last one if the row has produced too many.
fn push_span(spans: &mut heapless::Vec<(u16, u16), MAX_RUNS_PER_ROW>, x0: u16, x1: u16) {
    if spans.push((x0, x1)).is_err() {
        if let Some(last) = spans.last_mut() {
            last.1 = x1;
        }
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

    /// One changed pixel is one rectangle of one pixel.
    #[test]
    fn a_single_pixel_is_a_single_region() {
        let previous = blank();
        let mut current = previous.clone();
        set_pixel(&mut current, 100, 50, 0xF800);

        let regions = tracker().detect_changes(&current, &previous);
        assert_eq!(regions.as_slice(), &[Region::new(100, 50, 100, 50)]);
    }

    /// **The regression this rewrite exists for.**
    ///
    /// A band of full-width rows changing together -- a menu scrolling -- must come back as
    /// one rectangle. The previous implementation pushed one rectangle per changed row
    /// before merging, overflowed its 128-rectangle cap on row 128 of a 168-row panel, and
    /// returned a single full-screen rectangle, so every scroll frame repainted all 143,808
    /// bytes.
    #[test]
    fn a_tall_band_of_rows_is_one_region_not_one_per_row() {
        let previous = blank();
        let mut current = previous.clone();
        fill_rows(&mut current, 20, 150, 0x07E0);

        let regions = tracker().detect_changes(&current, &previous);
        assert_eq!(regions.as_slice(), &[Region::new(0, 20, W - 1, 150)]);
    }

    /// A change taller than the rectangle cap, but narrow, stays a delta.
    ///
    /// **The regression this rewrite exists for, in its sharpest form.** 131 rows is more
    /// than `MAX_REGIONS`, and the previous implementation emitted one rectangle per changed
    /// row before merging -- so it overflowed on row 128 and returned a single full-screen
    /// rectangle, turning a 9%-of-the-panel edit into a 143,808-byte repaint. Row count must
    /// have no bearing on the decision; only area may.
    #[test]
    fn a_tall_narrow_change_is_not_promoted_to_a_full_repaint() {
        let previous = blank();
        let mut current = previous.clone();
        for y in 10..=140 {
            for x in 0..=50 {
                set_pixel(&mut current, x, y, 0x001F);
            }
        }

        let t = tracker();
        let regions = t.detect_changes(&current, &previous);

        assert_eq!(regions.as_slice(), &[Region::new(0, 10, 50, 140)]);
        assert!(
            !t.should_full_update(&regions),
            "131 rows of 51 columns is 9% of the panel and must stay a delta, got {regions:?}"
        );
    }

    /// Two separated shapes stay two rectangles rather than one box spanning the gap.
    #[test]
    fn distant_changes_do_not_merge_into_one_box() {
        let previous = blank();
        let mut current = previous.clone();
        for y in 10..=20 {
            for x in 0..=30 {
                set_pixel(&mut current, x, y, 0xFFFF);
            }
        }
        for y in 10..=20 {
            for x in 300..=330 {
                set_pixel(&mut current, x, y, 0xFFFF);
            }
        }

        let regions = tracker().detect_changes(&current, &previous);
        assert_eq!(
            regions.as_slice(),
            &[Region::new(0, 10, 30, 20), Region::new(300, 10, 330, 20)]
        );
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

    /// A change in the last column and last row is still found.
    ///
    /// The row scan walks four pixels at a time and 428 is not a multiple of four, so the
    /// final group is short -- an off-by-one here loses the right-hand edge of the panel.
    #[test]
    fn the_final_partial_group_is_scanned() {
        let previous = blank();
        let mut current = previous.clone();
        set_pixel(&mut current, W - 1, H - 1, 0x8888);

        let regions = tracker().detect_changes(&current, &previous);
        assert_eq!(regions.as_slice(), &[Region::new(W - 1, H - 1, W - 1, H - 1)]);
    }
}
