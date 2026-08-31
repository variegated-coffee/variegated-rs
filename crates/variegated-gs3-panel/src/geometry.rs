//! Where things go: section 2 of the display specification.
//!
//! The panel is 428x168 and the bezel behind the badge window leaves 390x115 of it visible.
//! Nothing may be drawn outside that window -- the rest is addressable, and permanently
//! hidden. Every coordinate in `states/` is expressed through [`at`] or one of the region
//! helpers here, so there is exactly one number to change if the window moves.

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};

use crate::palette;

/// The full addressable panel.
pub const PANEL_SIZE: Size = Size::new(428, 168);

/// Top-left of the window the bezel leaves visible.
///
/// The specification's figures place this at `(19, 26)` -- centred, 19 px left and right,
/// 26/27 top and bottom of a 428x168 panel. The firmware has shipped `(25, 34)` since the
/// panel was brought up, which is 6 px and 8 px in from centred. Nothing records whether
/// that came from a measurement of the real bezel or from a guess, so the shipped value
/// stands until someone looks at the hardware.
///
/// **This is the one place to change it.** If content is clipped at the left or top edge,
/// the specification was right and this becomes `Point::new(19, 26)`; if there is a visible
/// band of unused panel at the right or bottom, it was already correct.
pub const WINDOW_ORIGIN: Point = Point::new(25, 34);

/// The design area. Treated as a hard clip.
pub const WINDOW_SIZE: Size = Size::new(390, 115);

/// Internal padding, 8 px, and 10 px on the left edge where a rule follows.
pub const PAD_TOP: i32 = 8;
/// See [`PAD_TOP`].
pub const PAD_RIGHT: i32 = 8;
/// See [`PAD_TOP`].
pub const PAD_BOTTOM: i32 = 8;
/// See [`PAD_TOP`].
pub const PAD_LEFT: i32 = 10;

/// A status mark is 16x16.
pub const MARK_SIZE: u32 = 16;

/// Marks are 5 px apart, so one row of the strip is 21 px.
pub const MARK_PITCH: i32 = MARK_SIZE as i32 + 5;

/// Five marks, fixed order.
pub const MARK_COUNT: usize = 5;

/// The height of the vertical strip: five marks and four gaps.
pub const STRIP_HEIGHT: i32 = MARK_PITCH * (MARK_COUNT as i32 - 1) + MARK_SIZE as i32;

/// A point `dx, dy` in from the top-left of the visible window.
pub const fn at(dx: i32, dy: i32) -> Point {
    Point::new(WINDOW_ORIGIN.x + dx, WINDOW_ORIGIN.y + dy)
}

/// The visible window.
pub fn window() -> Rectangle {
    Rectangle::new(WINDOW_ORIGIN, WINDOW_SIZE)
}

/// The window inset by the standard padding: where content starts.
pub fn content() -> Rectangle {
    Rectangle::new(
        at(PAD_LEFT, PAD_TOP),
        Size::new(
            WINDOW_SIZE.width - (PAD_LEFT + PAD_RIGHT) as u32,
            WINDOW_SIZE.height - (PAD_TOP + PAD_BOTTOM) as u32,
        ),
    )
}

/// Top-left of the vertical status strip, at the right edge.
///
/// Centred vertically in the window rather than hung from the padding: the strip is 100 px
/// against 99 px of padded content, so aligning it to the content box would push the
/// conductivity mark one pixel into the bezel.
pub fn strip_origin() -> Point {
    at(
        WINDOW_SIZE.width as i32 - PAD_RIGHT - MARK_SIZE as i32,
        (WINDOW_SIZE.height as i32 - STRIP_HEIGHT) / 2,
    )
}

/// The right edge of everything that is not the status strip.
///
/// The 10 px gap matches the one the specification's figures leave between the last column
/// and the strip.
pub fn body_right() -> i32 {
    strip_origin().x - 10
}

/// Draw a 1 px vertical region divider from `top` down `height` pixels.
pub fn hairline_v<D>(top: Point, height: u32, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    Rectangle::new(top, Size::new(1, height))
        .into_styled(PrimitiveStyle::with_fill(palette::HAIRLINE))
        .draw(target)
}

/// Draw a 1 px horizontal region divider from `left` across `width` pixels.
pub fn hairline_h<D>(left: Point, width: u32, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    Rectangle::new(left, Size::new(width, 1))
        .into_styled(PrimitiveStyle::with_fill(palette::HAIRLINE))
        .draw(target)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_window_fits_the_panel() {
        let w = window();
        assert!(w.top_left.x >= 0 && w.top_left.y >= 0);
        assert!(w.top_left.x + w.size.width as i32 <= PANEL_SIZE.width as i32);
        assert!(w.top_left.y + w.size.height as i32 <= PANEL_SIZE.height as i32);
    }

    /// The strip is one pixel taller than the padded content box, which is why
    /// `strip_origin` centres in the window instead. If that ever stops being true the
    /// comment there is wrong and someone should know.
    #[test]
    fn the_strip_is_centred_within_the_window() {
        let origin = strip_origin();
        assert!(origin.y >= WINDOW_ORIGIN.y);
        assert!(origin.y + STRIP_HEIGHT <= WINDOW_ORIGIN.y + WINDOW_SIZE.height as i32);
        assert_eq!(
            origin.x + MARK_SIZE as i32,
            WINDOW_ORIGIN.x + WINDOW_SIZE.width as i32 - PAD_RIGHT
        );
    }
}
