//! List navigation over a single index space.

use core::ops::Range;

/// How big the list is and how much of it fits on screen.
///
/// Passed to the movement methods rather than stored, because both numbers are properties
/// of what is being shown at the moment -- a menu's row count changes when its content
/// does -- and because the two firmwares disagree about `wrap`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ListGeometry {
    /// Every row the user can land on, counted the same way everywhere.
    ///
    /// **This includes chrome.** The Silvia's "back" row is a row; so is its "Execute
    /// Routine" row. Counting them in the bounds check while excluding them from the render
    /// window is what produced the over-scroll, the blank bottom row and the scrollbar thumb
    /// drawn past the end of its track. Callers map rows to their own items; this crate never
    /// learns what a row means.
    pub total_rows: usize,
    /// How many rows are on screen at once.
    pub visible_rows: usize,
    /// Whether moving past an end comes back at the other.
    ///
    /// The GS3 wraps: it has one physical button per direction, and a button that does
    /// nothing reads as a broken machine. The Silvia clamps: a knob has no such problem, and
    /// wrapping a long settings list on an encoder is disorienting.
    pub wrap: bool,
}

/// Where the selection is, and how far the viewport has scrolled.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ListNav {
    selected: usize,
    offset: usize,
}

impl ListNav {
    /// A list scrolled to the top with the first row selected.
    pub const fn new() -> Self {
        Self { selected: 0, offset: 0 }
    }

    /// The selected row.
    pub const fn selected(&self) -> usize {
        self.selected
    }

    /// The first row on screen.
    pub const fn offset(&self) -> usize {
        self.offset
    }

    /// Move the selection towards row zero.
    pub fn up(&mut self, geo: ListGeometry) {
        if geo.total_rows == 0 {
            return;
        }
        if self.selected > 0 {
            self.selected -= 1;
        } else if geo.wrap {
            self.selected = geo.total_rows - 1;
        }
        self.reconcile(geo);
    }

    /// Move the selection away from row zero.
    pub fn down(&mut self, geo: ListGeometry) {
        if geo.total_rows == 0 {
            return;
        }
        let last = geo.total_rows - 1;
        if self.selected < last {
            self.selected += 1;
        } else if geo.wrap {
            self.selected = 0;
        }
        self.reconcile(geo);
    }

    /// Pull the selection and the offset back into range.
    ///
    /// Called after every move, and doing the work here rather than in the movement methods
    /// is what keeps the two of them symmetric. It also absorbs a list that shrank underneath
    /// a stale selection, which happens whenever menu content is data-driven.
    fn reconcile(&mut self, geo: ListGeometry) {
        if geo.total_rows == 0 {
            self.selected = 0;
            self.offset = 0;
            return;
        }
        self.selected = self.selected.min(geo.total_rows - 1);

        if geo.visible_rows == 0 {
            self.offset = 0;
            return;
        }
        if self.selected < self.offset {
            self.offset = self.selected;
        }
        if self.selected >= self.offset + geo.visible_rows {
            self.offset = self.selected + 1 - geo.visible_rows;
        }
        self.offset = self.offset.min(geo.total_rows.saturating_sub(geo.visible_rows));
    }

    /// The rows on screen, as a range that is always valid to slice with.
    ///
    /// Recomputed from `geo` rather than trusted from `offset`, so a caller that changes its
    /// row count between a move and a draw gets a correct window rather than a panic.
    pub fn visible_range(&self, geo: ListGeometry) -> Range<usize> {
        if geo.total_rows == 0 || geo.visible_rows == 0 {
            return 0..0;
        }
        let start = self.offset.min(geo.total_rows.saturating_sub(geo.visible_rows));
        let end = (start + geo.visible_rows).min(geo.total_rows);
        start..end
    }

    /// Scrollbar thumb as `(y_offset, height)` inside a track `track_px` tall, or `None` when
    /// the whole list fits and there is nothing to indicate.
    ///
    /// Integer arithmetic throughout: this runs on a Cortex-M33 and, more usefully, it makes
    /// the end-of-track case exact rather than a rounding question. The guarantee callers rely
    /// on is `y + height <= track_px`, for every reachable offset.
    pub fn thumb(&self, geo: ListGeometry, track_px: u32) -> Option<(u32, u32)> {
        if track_px == 0 || geo.visible_rows == 0 || geo.total_rows <= geo.visible_rows {
            return None;
        }
        // Both are non-zero from here: `total_rows > visible_rows >= 1`.
        let max_offset = (geo.total_rows - geo.visible_rows) as u64;
        let offset = (self.offset as u64).min(max_offset);

        let height = ((track_px as u64 * geo.visible_rows as u64) / geo.total_rows as u64)
            .max(1)
            .min(track_px as u64);
        let travel = track_px as u64 - height;
        // Rounded to nearest, so the last row lands exactly on `travel` rather than one pixel
        // short of it.
        let y = (travel * offset + max_offset / 2) / max_offset;

        Some((y as u32, height as u32))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn geo(total: usize, visible: usize, wrap: bool) -> ListGeometry {
        ListGeometry { total_rows: total, visible_rows: visible, wrap }
    }

    #[test]
    fn empty_list_does_not_move_or_panic() {
        // The Silvia's `navigate_down` computed `total_items - 1`, which underflows here.
        // It was unreachable only because `has_back_button()` was hard-coded true.
        let mut nav = ListNav::new();
        nav.down(geo(0, 5, false));
        nav.up(geo(0, 5, false));
        assert_eq!(nav.selected(), 0);
        assert_eq!(nav.offset(), 0);
    }

    #[test]
    fn clamps_at_both_ends() {
        let g = geo(3, 5, false);
        let mut nav = ListNav::new();
        nav.up(g);
        assert_eq!(nav.selected(), 0, "up from the first row stays put when not wrapping");
        for _ in 0..10 { nav.down(g); }
        assert_eq!(nav.selected(), 2, "down past the last row stays on the last row");
    }

    #[test]
    fn wraps_at_both_ends() {
        let g = geo(3, 5, true);
        let mut nav = ListNav::new();
        nav.up(g);
        assert_eq!(nav.selected(), 2, "up from the first row wraps to the last");
        nav.down(g);
        assert_eq!(nav.selected(), 0, "down from the last row wraps to the first");
    }

    #[test]
    fn every_row_is_reachable_and_selection_stays_in_the_window() {
        // The Silvia scrolled two rows early, which pinned the highlight to visual row 3
        // and made the bottom two rows positions the cursor could never occupy.
        for total in 0..12usize {
            for visible in 1..6usize {
                let g = geo(total, visible, false);
                let mut nav = ListNav::new();
                for expected in 0..total {
                    assert_eq!(nav.selected(), expected, "total={total} visible={visible}");
                    let window = nav.visible_range(g);
                    assert!(
                        window.contains(&nav.selected()),
                        "selection {} outside window {:?} (total={total} visible={visible})",
                        nav.selected(), window,
                    );
                    nav.down(g);
                }
            }
        }
    }

    #[test]
    fn offset_never_exceeds_the_last_full_page() {
        // The Silvia's guard counted rows *including* the back button while the render
        // window counted items *excluding* it, so it over-scrolled by one at the end of
        // every list and left a permanently blank bottom row.
        for total in 0..12usize {
            for visible in 1..6usize {
                let g = geo(total, visible, false);
                let mut nav = ListNav::new();
                for _ in 0..(total + 4) {
                    nav.down(g);
                    assert!(
                        nav.offset() <= total.saturating_sub(visible),
                        "offset {} past the last full page (total={total} visible={visible})",
                        nav.offset(),
                    );
                }
            }
        }
    }

    #[test]
    fn visible_range_is_always_a_valid_slice_range() {
        for total in 0..12usize {
            for visible in 0..6usize {
                let g = geo(total, visible, false);
                let mut nav = ListNav::new();
                for _ in 0..(total + 4) {
                    let r = nav.visible_range(g);
                    assert!(r.start <= r.end, "inverted range {r:?}");
                    assert!(r.end <= total, "range {r:?} past total={total}");
                    nav.down(g);
                }
            }
        }
    }

    #[test]
    fn no_thumb_when_everything_fits() {
        let nav = ListNav::new();
        assert_eq!(nav.thumb(geo(3, 5, false), 52), None);
        assert_eq!(nav.thumb(geo(5, 5, false), 52), None);
        assert_eq!(nav.thumb(geo(0, 5, false), 52), None);
    }

    #[test]
    fn thumb_never_leaves_its_track() {
        // The Silvia fed its scroll bar `items.len()` while the offset had been advanced
        // against a row count that included the back button, so the ratio exceeded 1 and the
        // thumb was drawn to y=68 on a 64px panel.
        for total in 1..20usize {
            for visible in 1..6usize {
                let g = geo(total, visible, false);
                let mut nav = ListNav::new();
                for _ in 0..(total + 4) {
                    if let Some((y, h)) = nav.thumb(g, 52) {
                        assert!(h >= 1, "zero-height thumb (total={total} visible={visible})");
                        assert!(
                            y + h <= 52,
                            "thumb {y}+{h} past the 52px track (total={total} visible={visible})",
                        );
                    }
                    nav.down(g);
                }
            }
        }
    }

    #[test]
    fn thumb_reaches_both_ends_of_the_track() {
        let g = geo(11, 5, false);
        let mut nav = ListNav::new();
        let (y_top, h) = nav.thumb(g, 52).expect("11 rows do not fit in 5");
        assert_eq!(y_top, 0, "at the top of the list the thumb is at the top of the track");
        for _ in 0..11 {
            nav.down(g);
        }
        let (y_bottom, h_bottom) = nav.thumb(g, 52).expect("still does not fit");
        assert_eq!(h_bottom, h, "the thumb does not change size as it travels");
        assert_eq!(y_bottom + h_bottom, 52, "at the end of the list it reaches the bottom");
    }

    #[test]
    fn degenerate_track_is_none_rather_than_a_divide_by_zero() {
        let nav = ListNav::new();
        assert_eq!(nav.thumb(geo(20, 5, false), 0), None);
        assert_eq!(nav.thumb(geo(20, 0, false), 52), None);
    }
}
