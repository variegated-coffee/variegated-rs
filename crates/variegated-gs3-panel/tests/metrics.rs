//! What each face in the type scale actually measures, and the column widths that depend on
//! it.
//!
//! The specification's figures are set in outline Helvetica Neue and Inconsolata at the
//! named sizes. The panel is set in U8g2's hand-drawn bitmaps at those sizes, and the two do
//! not have the same advances -- a u8g2 `inb49_mn` digit is wider than a 49 px web
//! Inconsolata one, which is enough to push a five-glyph clock out of the column the figure
//! puts it in.
//!
//! So the panel's columns are sized against the real faces, here, rather than against the
//! figure's. `cargo test -p variegated-gs3-panel --features fixtures -- --nocapture` prints
//! the table; the assertions are what stops a future change to the scale from silently
//! overflowing a region.

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use u8g2_fonts::FontRenderer;
use u8g2_fonts::types::{FontColor, VerticalPosition};
use variegated_gs3_panel::type_scale;

fn measure(font: &FontRenderer, text: &str) -> (i32, i32) {
    let dims = font
        .get_rendered_dimensions(text, Point::zero(), VerticalPosition::Top)
        .expect("measurable");
    (
        dims.advance.x,
        dims.bounding_box.map(|b| b.size.height as i32).unwrap_or(0),
    )
}

/// A target that records only where ink landed.
///
/// The reported bounding box of a `_mn` cut is the font's common glyph box, not the ink: it
/// says a digit is 65 px tall where the lit pixels are 36. Every column on this panel is
/// sized against the ink, so the ink is what gets measured.
#[derive(Default)]
struct InkBounds {
    min: Option<(i32, i32)>,
    max: Option<(i32, i32)>,
}

impl OriginDimensions for InkBounds {
    fn size(&self) -> Size {
        Size::new(1024, 1024)
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
                None => (p.x, p.y),
                Some((x, y)) => (x.min(p.x), y.min(p.y)),
            });
            self.max = Some(match self.max {
                None => (p.x, p.y),
                Some((x, y)) => (x.max(p.x), y.max(p.y)),
            });
        }
        Ok(())
    }
}

/// The ink extent of `text` drawn on a baseline at the origin: (width, height, top, bottom)
/// where top and bottom are relative to the baseline, negative being above it.
fn ink(font: &FontRenderer, text: &str) -> (i32, i32, i32, i32) {
    let mut bounds = InkBounds::default();
    font.render(
        text,
        Point::zero(),
        VerticalPosition::Baseline,
        FontColor::Transparent(Rgb565::WHITE),
        &mut bounds,
    )
    .expect("renderable");
    match (bounds.min, bounds.max) {
        (Some((x0, y0)), Some((x1, y1))) => (x1 - x0 + 1, y1 - y0 + 1, y0, y1),
        _ => (0, 0, 0, 0),
    }
}

/// Every region on this panel was sized against these numbers.
///
/// Swap a face for the rung above it and the panel still compiles, still renders, and still
/// passes the overlap check on the fixtures -- until a value one digit longer than the
/// fixture's arrives on a real machine. So the sizes the layout assumed are written down
/// here, and a change to the ladder has to come here first.
#[test]
fn the_faces_are_the_sizes_the_layout_assumes() {
    // (face, digit advance, digit ink height)
    let expected: [(&str, &FontRenderer, i32, i32); 7] = [
        ("HERO inb38", &type_scale::HERO, 31, 38),
        ("PRIMARY_46 inb33", &type_scale::PRIMARY_46, 27, 32),
        ("PRIMARY_30 inb24", &type_scale::PRIMARY_30, 21, 24),
        ("PRIMARY_27 inb21", &type_scale::PRIMARY_27, 18, 21),
        ("SECONDARY_21 inb21", &type_scale::SECONDARY_21, 18, 21),
        ("SECONDARY_19 inb19", &type_scale::SECONDARY_19, 15, 19),
        ("NUMBER_FLOOR inb16", &type_scale::NUMBER_FLOOR, 14, 16),
    ];
    for (name, font, advance, height) in expected {
        assert_eq!(measure(font, "0").0, advance, "{name} digit advance");
        let (_, ink_h, ..) = ink(font, "0");
        assert_eq!(ink_h, height, "{name} digit height");
    }

    // The word faces ink exactly their nominal size, which is why they were kept at the
    // sizes the specification names while the number faces had to be re-rung.
    for (name, font, height) in [
        ("STATE_WORD", &type_scale::STATE_WORD, 11),
        ("CHIP", &type_scale::CHIP, 8),
        ("LABEL", &type_scale::LABEL, 8),
        ("UNIT_12", &type_scale::UNIT_12, 12),
        ("UNIT_14", &type_scale::UNIT_14, 14),
    ] {
        let (_, ink_h, ..) = ink(font, "H");
        assert_eq!(ink_h, height, "{name} cap height");
    }
}

#[test]
fn print_the_scale() {
    let faces: [(&str, &FontRenderer, &str); 13] = [
        ("HERO inb38", &type_scale::HERO, "21:58"),
        ("PRIMARY_46 inb33", &type_scale::PRIMARY_46, "93.2"),
        ("PRIMARY_30 inb24", &type_scale::PRIMARY_30, "113.4"),
        ("PRIMARY_27 inb21", &type_scale::PRIMARY_27, "51.1"),
        ("SECONDARY_21 inb21", &type_scale::SECONDARY_21, "1.76"),
        ("SECONDARY_19 inb19", &type_scale::SECONDARY_19, "8.39"),
        ("NUMBER_FLOOR inb16", &type_scale::NUMBER_FLOOR, "1.42"),
        ("STATE_WORD helvB10", &type_scale::STATE_WORD, "HEATING"),
        ("STEP_OTHER helvR10", &type_scale::STEP_OTHER, "Declining profile"),
        ("CHIP helvB08", &type_scale::CHIP, "PUMP DUTY"),
        ("LABEL helvR08", &type_scale::LABEL, "ENDS AT 8.0 G IN CUP"),
        ("UNIT_12 helvR12", &type_scale::UNIT_12, "bar"),
        ("UNIT_14 helvR14", &type_scale::UNIT_14, "C"),
    ];
    for (name, font, sample) in faces {
        let (w, h) = measure(font, sample);
        let (digit_w, _) = measure(font, "0");
        let (ink_w, ink_h, top, bottom) = ink(font, sample);
        println!(
            "{name:22} {sample:24} adv {w:4} box {h:3} | digit adv {digit_w:3} | ink {ink_w:4}x{ink_h:3} top {top:4} bottom {bottom:3}"
        );
    }
}
