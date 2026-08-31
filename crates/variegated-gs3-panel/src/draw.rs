//! The three things every state module does with text, in one place.
//!
//! `u8g2-fonts` returns its own error type, which wraps the display's but also carries
//! glyph-lookup failures a `DrawTarget` error cannot represent. There is nothing useful a
//! panel can do about a missing glyph mid-frame, so these swallow it -- the same choice the
//! firmware's renderer has always made, and safe here because every face is built with
//! `with_ignore_unknown_chars(true)` (see [`crate::type_scale`]), so a lookup failure costs
//! one glyph rather than the string.

use core::fmt::Arguments;

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use u8g2_fonts::FontRenderer;
use u8g2_fonts::types::{FontColor, HorizontalAlignment, VerticalPosition};

/// Draw a run of text from its left edge and return the x the next run starts at.
///
/// This is what makes a number and its unit sit together without a measured constant
/// between them: draw the number, and start the unit at what comes back.
pub fn run<D>(
    font: &FontRenderer,
    args: Arguments<'_>,
    at: Point,
    vpos: VerticalPosition,
    color: Rgb565,
    target: &mut D,
) -> i32
where
    D: DrawTarget<Color = Rgb565>,
{
    #[cfg(test)]
    probe::record(font, args, at, vpos, None);
    at.x + font
        .render(args, at, vpos, FontColor::Transparent(color), target)
        .map(|d| d.advance.x)
        .unwrap_or(0)
}

/// Draw a run anchored somewhere other than its left edge.
pub fn aligned<D>(
    font: &FontRenderer,
    args: Arguments<'_>,
    at: Point,
    vpos: VerticalPosition,
    halign: HorizontalAlignment,
    color: Rgb565,
    target: &mut D,
) where
    D: DrawTarget<Color = Rgb565>,
{
    #[cfg(test)]
    probe::record(font, args, at, vpos, Some(halign));
    let _ = font.render_aligned(args, at, vpos, halign, FontColor::Transparent(color), target);
}

/// Where each run of text actually put ink, so a test can assert that no two of them
/// collide.
///
/// Two panels that overlap is the failure this crate is most likely to have and least
/// likely to notice: `render_aligned` never reports it, the driver never reports it, and a
/// screen with one figure drawn through another is still a screen. Looking at a rendered
/// image catches it once; this catches it every time.
///
/// It measures **ink**, not the reported bounding box. A `_mn` cut's box is the font's
/// common glyph cell -- 65 px tall where the lit pixels are 52 -- so boxes touch long before
/// the drawing does, and an assertion built on them would fail on layouts that are correct.
/// The cost is drawing each run twice under `cfg(test)`, into a target that keeps only a
/// bounding rectangle.
#[cfg(test)]
pub(crate) mod probe {
    use super::*;
    use core::cell::RefCell;
    use embedded_graphics::primitives::Rectangle;
    use std::vec::Vec;

    std::thread_local! {
        static INK: RefCell<Vec<Rectangle>> = const { RefCell::new(Vec::new()) };
    }

    #[derive(Default)]
    struct InkBounds {
        min: Option<Point>,
        max: Option<Point>,
    }

    impl OriginDimensions for InkBounds {
        fn size(&self) -> Size {
            // Deliberately unbounded: a run drawn outside the panel has to be *recorded*
            // outside the panel, or the test that checks for exactly that cannot see it.
            Size::new(u32::MAX, u32::MAX)
        }
    }

    impl DrawTarget for InkBounds {
        type Color = Rgb565;
        type Error = core::convert::Infallible;

        fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
        where
            I: IntoIterator<Item = Pixel<Self::Color>>,
        {
            for Pixel(p, _) in pixels {
                self.min = Some(match self.min {
                    None => p,
                    Some(m) => Point::new(m.x.min(p.x), m.y.min(p.y)),
                });
                self.max = Some(match self.max {
                    None => p,
                    Some(m) => Point::new(m.x.max(p.x), m.y.max(p.y)),
                });
            }
            Ok(())
        }
    }

    pub(crate) fn record(
        font: &FontRenderer,
        args: Arguments<'_>,
        at: Point,
        vpos: VerticalPosition,
        halign: Option<HorizontalAlignment>,
    ) {
        let mut bounds = InkBounds::default();
        let colour = FontColor::Transparent(Rgb565::new(31, 63, 31));
        let _ = match halign {
            Some(halign) => font
                .render_aligned(args, at, vpos, halign, colour, &mut bounds)
                .map(|_| ()),
            None => font.render(args, at, vpos, colour, &mut bounds).map(|_| ()),
        };
        if let (Some(min), Some(max)) = (bounds.min, bounds.max) {
            let rect = Rectangle::with_corners(min, max);
            INK.with(|ink| ink.borrow_mut().push(rect));
        }
    }

    /// Record something that is not text but still owns its pixels -- a status mark.
    ///
    /// A mark is as much a thing that must not be drawn through as a figure is, and it is the
    /// one the panels are most likely to run into: it sits at the right edge, and a phrase
    /// that grows a word runs straight at it.
    pub(crate) fn record_rect(rect: Rectangle) {
        INK.with(|ink| ink.borrow_mut().push(rect));
    }

    /// Forget everything recorded so far.
    pub(crate) fn reset() {
        INK.with(|ink| ink.borrow_mut().clear());
    }

    /// Note that `area` has just been filled opaquely, painting out everything under it.
    ///
    /// An overlay box is not an overlap: the text beneath it is gone, not doubled. Runs
    /// wholly inside the fill are therefore dropped. A run only *partly* under it is kept,
    /// because half a word disappearing under a box is a defect of the same kind as two
    /// words on top of each other, and should still be found.
    pub(crate) fn occlude(area: Rectangle) {
        INK.with(|ink| {
            ink.borrow_mut()
                .retain(|rect| area.intersection(rect) != *rect)
        });
    }

    /// The ink rectangle of every run drawn since [`reset`].
    pub(crate) fn taken() -> Vec<Rectangle> {
        INK.with(|ink| ink.borrow().clone())
    }
}

/// How wide a run would be, without drawing it.
///
/// For laying out a row right-to-left, or centring a group of runs that are drawn
/// left-to-right.
pub fn width(font: &FontRenderer, args: Arguments<'_>) -> i32 {
    font.get_rendered_dimensions(args, Point::zero(), VerticalPosition::Baseline)
        .map(|d| d.advance.x)
        .unwrap_or(0)
}
