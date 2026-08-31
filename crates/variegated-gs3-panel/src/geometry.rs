//! Where things go: section 2 of the display specification.
//!
//! The panel is 428x168 and the bezel behind the badge window leaves 396x111 of it visible.
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
use embedded_graphics::primitives::{
    PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, StrokeAlignment,
};

use crate::palette;

/// The full addressable panel.
pub const PANEL_SIZE: Size = Size::new(428, 168);

/// The design area. Treated as a hard clip.
///
/// Six wider and four shorter than the 390x115 the first two rounds were drawn against. The
/// four pixels are not taken out of the row pitch: a four-row state has three gaps, so four
/// pixels is 1.33 each, and a pitch of 6, 5, 5 is precisely the defect the second remediation
/// existed to fix -- arrived at by arithmetic rather than by carelessness. Each state gives
/// them up from one named element chosen for tolerance instead; see the note on each.
pub const WINDOW_SIZE: Size = Size::new(396, 111);

/// Internal padding, 8 px, and 10 px on the left edge where a rule follows.
pub const PAD_TOP: i32 = 8;
/// See [`PAD_TOP`].
pub const PAD_RIGHT: i32 = 8;
/// See [`PAD_TOP`].
pub const PAD_BOTTOM: i32 = 8;
/// See [`PAD_TOP`].
pub const PAD_LEFT: i32 = 10;

/// The coloured square that marks a state word.
///
/// A square rather than a circle: at eight pixels unaliased a circle is a square with its
/// corners chipped, and the chipping is the only thing that distinguishes them.
pub const MARKER_SIZE: u32 = 8;

/// The channel the marker hangs in, left of the text column.
///
/// The second review found the marker sitting *in* the text column, so `READY` began eleven
/// pixels right of the `BREW` under it and the rows of one state had three different left
/// edges. A marker is not content and must not displace content, so it gets a channel of its
/// own and the text column starts after it -- on the rows that have no marker as much as on
/// the one that does, which is what gives the state one left edge.
///
/// Only the three states that carry a state word pay for it. Free-brewing and routine
/// execution have no marker to hang, and fourteen pixels is not free: free-brewing's bottom
/// row is four cells of 97 px holding 91 px of figures, and the gutter alone would have put
/// `PRESSURE 8.39 BAR` through the cell beside it.
pub const GUTTER: i32 = MARKER_SIZE as i32 + 6;

/// A status mark is 18x18.
///
/// Sixteen, at the 1 px stroke the specification's figures use, was about one lit pixel of
/// evidence per feature: read on the machine the colours carried and the shapes did not.
/// Eighteen at two pixels is what makes a mark identifiable rather than merely present.
pub const MARK_SIZE: u32 = 18;

/// Five marks, fixed order.
pub const MARK_COUNT: usize = 5;

/// The strip spreads over the window rather than sitting at a fixed gap.
///
/// Five 18 px marks and four 23 px steps fill the 111 px window exactly, one pixel down from
/// its top edge. That is what gives the marks the most separation the panel can offer, which
/// is the other half of making them identifiable -- and it is why this is derived from the
/// window rather than fixed: the strip stretches to whatever height the aperture turns out to
/// have, which is the one element that should absorb a resize without being asked.
pub const MARK_PITCH: i32 = 23;

/// Where the strip starts, from the window top.
pub const STRIP_TOP_DY: i32 = 1;

/// The height of the vertical strip: five marks and four steps.
pub const STRIP_HEIGHT: i32 = MARK_PITCH * (MARK_COUNT as i32 - 1) + MARK_SIZE as i32;

/// Where the visible window sits on the panel.
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
    /// What a machine shows until somebody trims it: centred on the panel.
    ///
    /// 16 px left and right of a 396 px window; 168 − 111 is odd, so 28 above and 29 below.
    ///
    /// **This changed with the window, and a stored trim did not.** The firmware shipped
    /// `(25, 34)` for a 390x115 window, which was 6 px and 8 px in from that window's own
    /// centre. Against a 396x111 window the same stored origin is 9 px and 6 px off centre
    /// instead -- an offset calibrated against the old shape now sits off-centre rather than
    /// merely off, and the two errors are not even in the same proportion. Any machine that
    /// has been trimmed wants re-trimming from the menu; nothing here can do it for them,
    /// because where the bezel's aperture actually sits is still a fact about one machine.
    pub const DEFAULT: Window = Window {
        origin: Point::new(
            (PANEL_SIZE.width as i32 - WINDOW_SIZE.width as i32) / 2,
            (PANEL_SIZE.height as i32 - WINDOW_SIZE.height as i32) / 2,
        ),
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
    /// Flush to the window's right edge rather than inside the padding: the strip is the one
    /// thing on this panel that is meant to be found without reading, and the edge is the
    /// easiest place on a small window to find.
    pub fn strip_origin(self) -> Point {
        self.at(
            WINDOW_SIZE.width as i32 - MARK_SIZE as i32,
            STRIP_TOP_DY,
        )
    }

    /// The left edge every row shares, in a state that carries a marker.
    ///
    /// See [`GUTTER`]. Left of it is the marker channel and nothing else.
    pub fn text_left(self) -> i32 {
        self.at(GUTTER, 0).x
    }

    /// Draw the coloured square that marks a state word, on the same cap box as the word.
    ///
    /// It is anchored to the word's baseline rather than to a row's top for the reason a chip
    /// is: the square and the word have to agree about where the row is, and only one of them
    /// can be the authority.
    pub fn marker<D>(
        self,
        baseline: i32,
        cap: i32,
        colour: Rgb565,
        target: &mut D,
    ) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        // Sunk to sit on the word's optical centre rather than its baseline: a square level
        // with the baseline reads as falling off the line.
        let top = baseline - (cap + MARKER_SIZE as i32) / 2;
        Rectangle::new(
            Point::new(self.origin.x, top),
            Size::new(MARKER_SIZE, MARKER_SIZE),
        )
        .into_styled(PrimitiveStyle::with_fill(colour))
        .draw(target)
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

/// The border of the calibration frame.
///
/// **Two pixels, where the field review asked for one.** Everything else read on the machine
/// had to be thickened, and this is the one element whose whole job is to be seen against a
/// bright printed surround from arm's length.
pub const FRAME_BORDER: u32 = 2;

/// How far a calibration tick reaches along the edge it belongs to.
pub const FRAME_TICK: u32 = 4;

/// Draw the border and ticks a trimmed offset is judged against.
///
/// A correct offset reads as an unbroken rectangle with even margins inside the aperture; any
/// error shows as a missing edge. The ticks are what turn "roughly centred" into a judgement
/// you can actually make -- a corner tells you an edge is present, a midpoint tells you the
/// margin above it matches the one below.
///
/// Everything is drawn inward from the window's own edges, so the frame marks the boundary
/// rather than straddling it: a border that leaked outward would be a border you were
/// aligning to that is not where the design stops.
///
/// Two callers, and they want the same rectangle for the same reason. The GS3's menu draws it
/// while either origin editor is open, and the `always-draw-bounds` feature draws it on every
/// frame -- see [`crate::render`].
pub fn calibration_frame<D>(window: Window, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let rect = window.rect();
    rect.into_styled(
        PrimitiveStyleBuilder::new()
            .stroke_color(palette::INK)
            .stroke_width(FRAME_BORDER)
            .stroke_alignment(StrokeAlignment::Inside)
            .build(),
    )
    .draw(target)?;

    let (x0, y0) = (rect.top_left.x, rect.top_left.y);
    let (w, h) = (rect.size.width as i32, rect.size.height as i32);
    // How far a tick runs along its edge, and how far it reaches in from it. A tick is the
    // border plus two, so it is thicker than the line it interrupts and reads as a mark
    // rather than as a bulge.
    let along = FRAME_TICK as i32;
    let reach = FRAME_BORDER as i32 + 2;
    let fill = PrimitiveStyleBuilder::new().fill_color(palette::INK).build();

    // Four corners and four midpoints, each reaching inward from the edge it belongs to. Both
    // extents are subtracted at the far edges: placing a tick at `edge - border` would put its
    // remaining two pixels outside the window, where the bezel eats them and the frame stops
    // marking the boundary it is being aligned against.
    for (x, y, across) in [
        (x0, y0, true),
        (x0 + w - along, y0, true),
        (x0, y0 + h - reach, true),
        (x0 + w - along, y0 + h - reach, true),
        (x0 + (w - along) / 2, y0, true),
        (x0 + (w - along) / 2, y0 + h - reach, true),
        (x0, y0 + (h - along) / 2, false),
        (x0 + w - reach, y0 + (h - along) / 2, false),
    ] {
        let size = if across {
            Size::new(FRAME_TICK, reach as u32)
        } else {
            Size::new(reach as u32, FRAME_TICK)
        };
        Rectangle::new(Point::new(x, y), size)
            .into_styled(fill)
            .draw(target)?;
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_graphics::Pixel;

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

    /// A target that keeps every pixel's coordinate.
    #[derive(Default)]
    struct Lit(std::vec::Vec<Point>);

    impl OriginDimensions for Lit {
        fn size(&self) -> Size {
            PANEL_SIZE
        }
    }

    impl DrawTarget for Lit {
        type Color = Rgb565;
        type Error = core::convert::Infallible;

        fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
        where
            I: IntoIterator<Item = Pixel<Self::Color>>,
        {
            self.0.extend(pixels.into_iter().map(|Pixel(p, _)| p));
            Ok(())
        }
    }

    /// The frame marks the window's boundary, so it must be exactly on the inside of it.
    ///
    /// The version this was lifted from placed its bottom and right ticks at `edge - border`
    /// while drawing them `border + 2` deep, which put two pixels of each outside the window.
    /// Behind the bezel that is invisible, which is precisely why it needs asserting: the one
    /// element whose entire purpose is to show where the design stops was, on two of its four
    /// edges, drawn past it.
    #[test]
    fn the_calibration_frame_stays_inside_the_window() {
        for window in every_extreme() {
            let rect = window.rect();
            let mut lit = Lit::default();
            calibration_frame(window, &mut lit).unwrap();
            assert!(!lit.0.is_empty());
            for p in &lit.0 {
                assert!(
                    rect.contains(*p),
                    "the frame lit {p:?}, which is outside {rect:?}",
                );
            }
        }
    }

    /// All four edges, and the middle of each. A frame missing an edge is how a bad offset is
    /// meant to read, so a frame that draws one badly says the offset is wrong when it is not.
    #[test]
    fn the_calibration_frame_marks_every_edge() {
        let window = Window::DEFAULT;
        let rect = window.rect();
        let mut lit = Lit::default();
        calibration_frame(window, &mut lit).unwrap();

        let (x0, y0) = (rect.top_left.x, rect.top_left.y);
        let (w, h) = (rect.size.width as i32, rect.size.height as i32);
        for expected in [
            Point::new(x0, y0),
            Point::new(x0 + w - 1, y0),
            Point::new(x0, y0 + h - 1),
            Point::new(x0 + w - 1, y0 + h - 1),
            Point::new(x0 + w / 2, y0),
            Point::new(x0 + w / 2, y0 + h - 1),
            Point::new(x0, y0 + h / 2),
            Point::new(x0 + w - 1, y0 + h / 2),
        ] {
            assert!(lit.0.contains(&expected), "the frame skipped {expected:?}");
        }
    }

    /// Five 18 px marks and four 24 px steps fill the window exactly, flush to its right
    /// edge. If that stops being true the strip has either lost its separation or grown out
    /// of the aperture, and both are what the redraw was for.
    #[test]
    fn the_strip_fills_the_window_flush_right() {
        for window in every_extreme() {
            let origin = window.strip_origin();
            assert!(origin.y >= window.origin().y);
            assert_eq!(
                origin.y + STRIP_HEIGHT,
                window.origin().y + WINDOW_SIZE.height as i32,
            );
            assert_eq!(
                origin.x + MARK_SIZE as i32,
                window.origin().x + WINDOW_SIZE.width as i32,
            );
        }
    }
}
