//! Where things go: section 2 of the display specification.
//!
//! The panel is 428x168 and the bezel behind the badge window leaves 390x115 of it visible.
//! Nothing may be drawn outside that window -- the rest is addressable, and permanently
//! hidden. Every coordinate in `states/` is expressed through [`Window::at`] or one of the
//! region helpers on [`Window`], so where the window sits is one value, passed in.
//!
//! It is passed in rather than compiled in because *where* the aperture sits is a property of
//! one physical machine, judged by eye from in front of it. The GS3 stores a trimmed origin
//! and hands it to [`crate::render`]; [`Window::DEFAULT`] is what a machine shows until
//! somebody trims it.

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};

use crate::palette;

/// The full addressable panel.
pub const PANEL_SIZE: Size = Size::new(428, 168);

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

/// Where the 390x115 visible window sits on the panel.
///
/// `Copy` and two `i32`s wide, so it is passed by value everywhere rather than reached for.
// No `defmt::Format`: it would have to come from `embedded-graphics`' own `defmt` feature
// for the `Point` inside, and nothing logs a window.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Window {
    origin: Point,
}

impl Default for Window {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl Window {
    /// What a machine shows until somebody trims it.
    ///
    /// The specification's figures place the window at `(19, 26)` -- centred, 19 px left and
    /// right, 26/27 top and bottom. The firmware has shipped `(25, 34)` since the panel was
    /// brought up, which is 6 px and 8 px in from centred, and nothing records whether that
    /// came from a measurement of the real bezel or from a guess. It stays the default
    /// because it is what every machine already shows; the trim is now a setting rather than
    /// an edit to this line.
    pub const DEFAULT: Window = Window {
        origin: Point::new(25, 34),
    };

    /// The furthest the window can travel and still fit the panel.
    ///
    /// 38 px across and 53 down, which is the panel minus the window. A trim is a trim, not a
    /// crop: past this the design would start leaving the addressable area entirely, and the
    /// drawing has nowhere to go.
    pub const MAX_ORIGIN: Point = Point::new(
        PANEL_SIZE.width as i32 - WINDOW_SIZE.width as i32,
        PANEL_SIZE.height as i32 - WINDOW_SIZE.height as i32,
    );

    /// A window at `x, y`, clamped into range.
    ///
    /// Clamped rather than asserted: this value comes off flash, and a machine that had been
    /// trimmed by a firmware with a different window size must come up drawing something
    /// rather than not at all.
    pub const fn new(x: i32, y: i32) -> Self {
        let x = if x < 0 {
            0
        } else if x > Self::MAX_ORIGIN.x {
            Self::MAX_ORIGIN.x
        } else {
            x
        };
        let y = if y < 0 {
            0
        } else if y > Self::MAX_ORIGIN.y {
            Self::MAX_ORIGIN.y
        } else {
            y
        };
        Window {
            origin: Point::new(x, y),
        }
    }

    /// The top-left corner.
    pub const fn origin(self) -> Point {
        self.origin
    }

    /// A point `dx, dy` in from the top-left of the visible window.
    pub const fn at(self, dx: i32, dy: i32) -> Point {
        Point::new(self.origin.x + dx, self.origin.y + dy)
    }

    /// The visible window.
    pub fn rect(self) -> Rectangle {
        Rectangle::new(self.origin, WINDOW_SIZE)
    }

    /// The window inset by the standard padding: where content starts.
    pub fn content(self) -> Rectangle {
        Rectangle::new(
            self.at(PAD_LEFT, PAD_TOP),
            Size::new(
                WINDOW_SIZE.width - (PAD_LEFT + PAD_RIGHT) as u32,
                WINDOW_SIZE.height - (PAD_TOP + PAD_BOTTOM) as u32,
            ),
        )
    }

    /// Top-left of the vertical status strip, at the right edge.
    ///
    /// Centred vertically in the window rather than hung from the padding: the strip is
    /// 100 px against 99 px of padded content, so aligning it to the content box would push
    /// the conductivity mark one pixel into the bezel.
    pub fn strip_origin(self) -> Point {
        self.at(
            WINDOW_SIZE.width as i32 - PAD_RIGHT - MARK_SIZE as i32,
            (WINDOW_SIZE.height as i32 - STRIP_HEIGHT) / 2,
        )
    }

    /// The right edge of everything that is not the status strip.
    ///
    /// The 10 px gap matches the one the specification's figures leave between the last
    /// column and the strip.
    pub fn body_right(self) -> i32 {
        self.strip_origin().x - 10
    }
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

    /// Every window a caller can construct, including the ones a bad stored value would
    /// produce.
    fn every_extreme() -> [Window; 5] {
        [
            Window::DEFAULT,
            Window::new(0, 0),
            Window::new(Window::MAX_ORIGIN.x, Window::MAX_ORIGIN.y),
            Window::new(-40, -40),
            Window::new(9_000, 9_000),
        ]
    }

    #[test]
    fn no_window_leaves_the_panel() {
        for window in every_extreme() {
            let rect = window.rect();
            assert!(rect.top_left.x >= 0 && rect.top_left.y >= 0, "{rect:?}");
            assert!(
                rect.top_left.x + rect.size.width as i32 <= PANEL_SIZE.width as i32,
                "{rect:?}"
            );
            assert!(
                rect.top_left.y + rect.size.height as i32 <= PANEL_SIZE.height as i32,
                "{rect:?}"
            );
        }
    }

    /// The clamp is what makes a stored value safe, so it is asserted rather than trusted.
    #[test]
    fn an_out_of_range_origin_is_pulled_in() {
        assert_eq!(Window::new(-1, -1).origin(), Point::new(0, 0));
        assert_eq!(Window::new(9_000, 9_000).origin(), Window::MAX_ORIGIN);
        assert_eq!(Window::new(19, 26).origin(), Point::new(19, 26));
    }

    /// The strip is one pixel taller than the padded content box, which is why
    /// `strip_origin` centres in the window instead. If that ever stops being true the
    /// comment there is wrong and someone should know.
    #[test]
    fn the_strip_is_centred_within_the_window() {
        for window in every_extreme() {
            let origin = window.strip_origin();
            assert!(origin.y >= window.origin().y);
            assert!(
                origin.y + STRIP_HEIGHT <= window.origin().y + WINDOW_SIZE.height as i32
            );
            assert_eq!(
                origin.x + MARK_SIZE as i32,
                window.origin().x + WINDOW_SIZE.width as i32 - PAD_RIGHT
            );
        }
    }
}
