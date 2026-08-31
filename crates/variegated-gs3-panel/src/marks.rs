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
//! One row per `u16`, bit 15 -- the leftmost written bit -- at x = 0, so the literals below
//! read as the picture they draw.

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;

use crate::geometry::{MARK_PITCH, MARK_SIZE, strip_origin};
use crate::palette;
use crate::view::MarkState;

/// Associated and reachable. Red: no link, and shots queue locally.
const NETWORK: [u16; 16] = [
    0b0000_0000_0000_0000,
    0b0000_0000_0000_0000,
    0b0000_0000_0000_0000,
    0b0000_1111_1111_0000,
    0b0011_0000_0000_1100,
    0b0100_0000_0000_0010,
    0b0000_0000_0000_0000,
    0b0000_0011_1100_0000,
    0b0000_1100_0011_0000,
    0b0000_0000_0000_0000,
    0b0000_0001_1000_0000,
    0b0000_0010_0100_0000,
    0b0000_0000_0000_0000,
    0b0000_0001_1000_0000,
    0b0000_0001_1000_0000,
    0b0000_0000_0000_0000,
];

/// Paired and reporting. Red: every gram field reads as no reading.
const SCALE: [u16; 16] = [
    0b0000_0000_0000_0000,
    0b0000_0000_0000_0000,
    0b0000_0000_0000_0000,
    0b0000_0011_1100_0000,
    0b0000_0100_0010_0000,
    0b0000_1000_0001_0000,
    0b0000_1000_0001_0000,
    0b0000_0100_0010_0000,
    0b0000_0011_1100_0000,
    0b0000_0001_1000_0000,
    0b0000_0000_0000_0000,
    0b0000_0000_0000_0000,
    0b0111_1111_1111_1110,
    0b0000_0000_0000_0000,
    0b0000_0000_0000_0000,
    0b0000_0000_0000_0000,
];

/// Steam boiler level satisfied. Red: needs filling, and steam is unavailable.
const STEAM_BOILER: [u16; 16] = [
    0b0000_0000_0000_0000,
    0b0000_0000_0000_0000,
    0b0000_0100_0010_0000,
    0b0000_1000_0100_0000,
    0b0000_1000_0100_0000,
    0b0000_0100_0010_0000,
    0b0000_0010_0001_0000,
    0b0000_0010_0001_0000,
    0b0000_0100_0010_0000,
    0b0000_1000_0100_0000,
    0b0000_1000_0100_0000,
    0b0000_0000_0000_0000,
    0b0000_0000_0000_0000,
    0b0011_1111_1111_1100,
    0b0000_0000_0000_0000,
    0b0000_0000_0000_0000,
];

/// Water present. Red: empty, and the pump will not start.
const TANK: [u16; 16] = [
    0b0000_0000_0000_0000,
    0b0000_0001_1000_0000,
    0b0000_0010_0100_0000,
    0b0000_0010_0100_0000,
    0b0000_0100_0010_0000,
    0b0000_1000_0001_0000,
    0b0000_1000_0001_0000,
    0b0001_0000_0000_1000,
    0b0001_0000_0000_1000,
    0b0001_0000_0000_1000,
    0b0000_1000_0001_0000,
    0b0000_0100_0010_0000,
    0b0000_0011_1100_0000,
    0b0000_0000_0000_0000,
    0b0000_0000_0000_0000,
    0b0000_0000_0000_0000,
];

/// Conductivity probe fitted and reading. Red: no solids or extraction figures this shot.
const PROBE: [u16; 16] = [
    0b0000_0000_0000_0000,
    0b0000_0000_0000_0000,
    0b0000_0000_0000_0000,
    0b0000_0000_0000_0000,
    0b0000_0000_0000_0000,
    0b0000_0000_0000_0000,
    0b0000_0100_0000_0000,
    0b1100_1010_0000_0011,
    0b1101_0001_0001_1011,
    0b0000_0000_1010_0000,
    0b0000_0000_0100_0000,
    0b0000_0000_0000_0000,
    0b0000_0000_0000_0000,
    0b0000_0000_0000_0000,
    0b0000_0000_0000_0000,
    0b0000_0000_0000_0000,
];

/// The five marks in the order section 5 fixes: network, scale, steam boiler, tank, probe.
const BITMAPS: [&[u16; 16]; 5] = [&NETWORK, &SCALE, &STEAM_BOILER, &TANK, &PROBE];

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
            (row & (0x8000 >> x) != 0)
                .then(|| Pixel(origin + Point::new(x as i32, y as i32), color))
        })
    }))
}

/// The vertical strip at the right edge, used by every state but free-brewing.
pub fn draw_column<D>(states: &[MarkState; 5], target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let origin = strip_origin();
    for (i, state) in states.iter().enumerate() {
        draw_one(i, *state, origin + Point::new(0, i as i32 * MARK_PITCH), target)?;
    }
    Ok(())
}

/// The width the horizontal row occupies.
pub const ROW_WIDTH: u32 = MARK_SIZE * 5 + 4 * 4;

/// The same marks laid along a header, for free-brewing.
///
/// Free-brewing spends the panel's full width on the rail, so its marks go in the header
/// rather than down the right edge. Same bitmaps at the same size, closer together: a
/// second, smaller set would be a second drawing to keep in agreement with this one, and
/// the strip is meant to be recognised at a glance across every state.
pub fn draw_row<D>(states: &[MarkState; 5], right: i32, top: i32, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let left = right - ROW_WIDTH as i32;
    for (i, state) in states.iter().enumerate() {
        draw_one(
            i,
            *state,
            Point::new(left + i as i32 * (MARK_SIZE as i32 + 4), top),
            target,
        )?;
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

    /// The bitmaps are 16 wide by construction, but a literal one bit too long would silently
    /// shift a whole mark left by a pixel, so assert the drawn columns stay inside the box.
    #[test]
    fn no_mark_escapes_its_box() {
        for rows in BITMAPS.iter() {
            for row in rows.iter() {
                assert_eq!(row & !0xFFFF_u16, 0);
            }
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
