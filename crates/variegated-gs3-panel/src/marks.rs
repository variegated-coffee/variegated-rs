//! The five status marks: section 5 of the display specification.
//!
//! Reduced geometric marks rather than letters. Letters carry a language, they run out
//! (`W`, `S`, `B`, `T`, `P` already collided with the portal indicator), and none of them
//! says the thing a red mark has to say. A mark is a statement of consequence, not an alarm:
//! the strip never blinks and never takes the panel.
//!
//! # Why these are bitmaps and not primitives
//!
//! Every one of the specification's symbols is a stroked vector path, and at 16 px a
//! stroked circle of radius 3.2 is about eleven pixels -- whose exact eleven depend on the
//! rasteriser's rounding rather than on the drawing. The panel draws without anti-aliasing,
//! so the honest form for a 16 px mark is the sixteen rows of pixels themselves: exactly
//! reproducible, comparable against a reference image, and 32 bytes each.
//!
//! One row per `u32`, eighteen bits wide, the leftmost written bit at x = 0 and grouped in
//! sixes -- so the literals below read as the picture they draw.
//!
//! Everything is stroked two pixels. Read on the machine at one, behind curved glass and
//! unaliased, a 16 px mark gave about one lit pixel of evidence per feature: the colours
//! carried and the shapes did not.
//!
//! # One optical width
//!
//! The second review found the column's right edge ragged, with one mark reading a size
//! larger than the rest. Measured off these bitmaps the spread was worse than reported and
//! in the other direction: the scale and the probe spanned all sixteen columns of an
//! eighteen-pixel box, the network and the boiler fourteen, and the tank ten. A drop that
//! looks small beside a beam that touches both walls is not saying anything about the water
//! level.
//!
//! So all five now span [`INK_LEFT`]`..=`[`INK_RIGHT`] -- one left edge, one right edge, and
//! the column reads as five of one thing. [`the_marks_share_an_optical_width`] is what keeps
//! it that way, because this is a property no one of these literals can be checked against on
//! its own.
//!
//! [`the_marks_share_an_optical_width`]: tests::the_marks_share_an_optical_width

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;

use crate::geometry::{MARK_PITCH, MARK_SIZE, Window};
use crate::palette;
use crate::view::MarkState;

/// The leftmost column any mark lights, and the rightmost.
///
/// Fourteen pixels of the eighteen, which is where the network and boiler marks already sat.
/// Not the ten the review names -- that figure is measured off its own SVG stand-ins in a
/// sixteen-unit box, and pulling every mark down to it would give back most of what the first
/// remediation bought by going from sixteen pixels to eighteen.
pub const INK_LEFT: u32 = 2;
/// See [`INK_LEFT`].
pub const INK_RIGHT: u32 = 15;

/// Associated and reachable. Red: no link, and shots queue locally.
const NETWORK: [u32; 18] = [
    0b000000_000000_000000,
    0b000000_111111_000000,
    0b000011_111111_110000,
    0b000111_000000_111000,
    0b001100_000000_001100,
    0b000000_000000_000000,
    0b000000_000000_000000,
    0b000000_011110_000000,
    0b000001_111111_100000,
    0b000011_000000_110000,
    0b000000_000000_000000,
    0b000000_000000_000000,
    0b000000_001100_000000,
    0b000000_011110_000000,
    0b000000_000000_000000,
    0b000000_001100_000000,
    0b000000_001100_000000,
    0b000000_000000_000000,
];

/// Paired and reporting. Red: every gram field reads as no reading.
const SCALE: [u32; 18] = [
    0b000000_000000_000000,
    0b000000_000000_000000,
    0b000000_111111_000000,
    0b000011_100001_110000,
    0b000110_000000_011000,
    0b000110_000000_011000,
    0b000110_000000_011000,
    0b000110_000000_011000,
    0b000011_100001_110000,
    0b000000_111111_000000,
    0b000000_000000_000000,
    0b001111_111111_111100,
    0b001111_111111_111100,
    0b000000_000000_000000,
    0b000000_000000_000000,
    0b000000_000000_000000,
    0b000000_000000_000000,
    0b000000_000000_000000,
];

/// Steam boiler level satisfied. Red: needs filling, and steam is unavailable.
const STEAM_BOILER: [u32; 18] = [
    0b000000_000000_000000,
    0b000011_000000_110000,
    0b000110_000001_100000,
    0b000110_000001_100000,
    0b000011_000000_110000,
    0b000001_100000_011000,
    0b000001_100000_011000,
    0b000011_000000_110000,
    0b000110_000001_100000,
    0b000110_000001_100000,
    0b000011_000000_110000,
    0b000000_000000_000000,
    0b000000_000000_000000,
    0b000000_000000_000000,
    0b001111_111111_111100,
    0b001111_111111_111100,
    0b000000_000000_000000,
    0b000000_000000_000000,
];

/// Water present. Red: empty, and the pump will not start.
///
/// Widened from ten columns to fourteen. It was the narrowest mark in the strip by four
/// pixels, which made the one condition that stops the pump read as the least important thing
/// in the column.
const TANK: [u32; 18] = [
    0b000000_000000_000000,
    0b000000_000000_000000,
    0b000000_001100_000000,
    0b000000_011110_000000,
    0b000000_110011_000000,
    0b000001_100001_100000,
    0b000011_000000_110000,
    0b000110_000000_011000,
    0b001100_000000_001100,
    0b001100_000000_001100,
    0b001100_000000_001100,
    0b000110_000000_011000,
    0b000011_000000_110000,
    0b000001_100001_100000,
    0b000000_111111_000000,
    0b000000_000000_000000,
    0b000000_000000_000000,
    0b000000_000000_000000,
];

/// Conductivity probe fitted and reading. Red: no solids or extraction figures this shot.
const PROBE: [u32; 18] = [
    0b000000_000000_000000,
    0b000000_000000_000000,
    0b000000_000000_000000,
    0b000000_000000_000000,
    0b000000_000000_000000,
    0b000000_000000_000000,
    0b000000_000110_000000,
    0b001100_000110_001100,
    0b001100_011001_101100,
    0b001100_011001_101100,
    0b001100_110000_111100,
    0b001100_110000_111100,
    0b000000_000000_000000,
    0b000000_000000_000000,
    0b000000_000000_000000,
    0b000000_000000_000000,
    0b000000_000000_000000,
    0b000000_000000_000000,
];

/// The five marks in the order section 5 fixes: network, scale, steam boiler, tank, probe.
const BITMAPS: [&[u32; 18]; 5] = [&NETWORK, &SCALE, &STEAM_BOILER, &TANK, &PROBE];

/// The colour a mark in this condition is drawn in.
fn ink(state: MarkState) -> Rgb565 {
    match state {
        MarkState::Ok => palette::OK,
        MarkState::Attention => palette::DANGER,
        // Not fitted, or nothing known. Present so the strip keeps its layout -- a strip
        // that changed length would make the panel's right edge move between states -- but
        // claiming nothing.
        MarkState::Absent => palette::INK_FAINT,
    }
}

/// Draw one mark with its top-left at `origin`.
pub fn draw_one<D>(
    index: usize,
    state: MarkState,
    origin: Point,
    target: &mut D,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let rows = BITMAPS[index];
    let color = ink(state);
    // A mark owns its 16x16 the way a run of text owns its ink, and it is what a phrase that
    // grows a word runs into first. The whole box, not the lit pixels: a mark with a word
    // threaded between its strokes is not a mark anyone can read.
    #[cfg(test)]
    crate::draw::probe::record_rect(embedded_graphics::primitives::Rectangle::new(
        origin,
        embedded_graphics::prelude::Size::new(MARK_SIZE, MARK_SIZE),
    ));
    target.draw_iter(rows.iter().enumerate().flat_map(|(y, row)| {
        (0..MARK_SIZE).filter_map(move |x| {
            (row & (1 << (MARK_SIZE - 1 - x)) != 0)
                .then(|| Pixel(origin + Point::new(x as i32, y as i32), color))
        })
    }))
}

/// The vertical strip at the right edge. Every state, without exception.
///
/// Free-brewing used to lay the same marks along its header instead, closer together, because
/// it wanted the panel's full width for its rail. Read as a set that made the strip move and
/// shrink between states -- and status is the one thing on this panel that has to be findable
/// without being looked for, which means it has to be in the same place whatever is running.
/// The rail gives up the width instead; it had it to spare.
pub fn draw_column<D>(
    states: &[MarkState; 5],
    w: Window,
    target: &mut D,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let origin = w.strip_origin();
    for (i, state) in states.iter().enumerate() {
        draw_one(i, *state, origin + Point::new(0, i as i32 * MARK_PITCH), target)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A mark that drew nothing would be indistinguishable from a mark whose state is
    /// "absent", and the strip would silently lose a position.
    #[test]
    fn every_mark_has_pixels() {
        for (i, rows) in BITMAPS.iter().enumerate() {
            assert!(
                rows.iter().any(|row| *row != 0),
                "mark {i} is blank"
            );
        }
    }

    /// The bitmaps are 18 wide by construction, but a literal one bit too long would silently
    /// shift a whole mark left by a pixel, so assert the drawn columns stay inside the box.
    #[test]
    fn no_mark_escapes_its_box() {
        let mask = (1u32 << MARK_SIZE) - 1;
        for rows in BITMAPS.iter() {
            for row in rows.iter() {
                assert_eq!(row & !mask, 0, "{row:#020b} is wider than {MARK_SIZE}");
            }
        }
    }

    /// Two pixels of stroke is the whole point of the redraw, so it is asserted rather than
    /// trusted: every lit pixel must have a lit neighbour on one axis or the other. A stray
    /// single pixel is a one-pixel feature, which is what did not read on the machine.
    #[test]
    fn nothing_is_drawn_one_pixel_thin() {
        for (index, rows) in BITMAPS.iter().enumerate() {
            for (y, row) in rows.iter().enumerate() {
                for x in 0..MARK_SIZE {
                    let bit = |row: u32, x: u32| row & (1 << (MARK_SIZE - 1 - x)) != 0;
                    if !bit(*row, x) {
                        continue;
                    }
                    let horizontal = (x > 0 && bit(*row, x - 1))
                        || (x + 1 < MARK_SIZE && bit(*row, x + 1));
                    let vertical = (y > 0 && bit(rows[y - 1], x))
                        || (y + 1 < rows.len() && bit(rows[y + 1], x));
                    assert!(
                        horizontal || vertical,
                        "mark {index} has a lone pixel at {x},{y}",
                    );
                }
            }
        }
    }

    /// The five marks share one left edge and one right edge.
    ///
    /// This is the finding that cannot be seen in any one bitmap: each literal below looks
    /// like a reasonable drawing of its own subject, and the fault only exists in the set.
    /// Before this, the spread ran from ten columns to sixteen, so the column's right edge was
    /// ragged and the widest marks read a size larger than the narrowest.
    #[test]
    fn the_marks_share_an_optical_width() {
        for (index, rows) in BITMAPS.iter().enumerate() {
            let lit = |x: u32| rows.iter().any(|row| row & (1 << (MARK_SIZE - 1 - x)) != 0);
            let left = (0..MARK_SIZE).find(|x| lit(*x)).expect("a mark has ink");
            let right = (0..MARK_SIZE).rev().find(|x| lit(*x)).expect("a mark has ink");
            assert_eq!(left, INK_LEFT, "mark {index} starts at column {left}");
            assert_eq!(right, INK_RIGHT, "mark {index} ends at column {right}");
        }
    }

    /// The three conditions must be three colours: a strip where "absent" and "attention"
    /// looked alike would say a probe is faulty when it is simply not fitted.
    #[test]
    fn the_three_conditions_are_distinguishable() {
        let ok = ink(MarkState::Ok);
        let attention = ink(MarkState::Attention);
        let absent = ink(MarkState::Absent);
        assert_ne!(ok, attention);
        assert_ne!(ok, absent);
        assert_ne!(attention, absent);
    }
}
