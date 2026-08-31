//! The pieces more than one state draws: a chip, a rail, a progress bar, and a number that
//! may not exist.

use core::fmt::Arguments;

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};
use u8g2_fonts::FontRenderer;
use u8g2_fonts::types::VerticalPosition;

use crate::draw;
use crate::palette;
use crate::type_scale;

/// A chip's height, filled or outlined.
pub const CHIP_HEIGHT: u32 = 13;

/// Draw a filled chip -- a word on a solid, in the chip face -- and return the x after it.
///
/// The specification uses one for the schedule action on the off panel. A filled chip says
/// the word is a *state of affairs*, where an outlined one says it is a setting; that is why
/// `NEXT ON` is filled and the free-brew mode chip is not.
pub fn chip_filled<D>(
    word: Arguments<'_>,
    fill: Rgb565,
    at: Point,
    target: &mut D,
) -> Result<i32, D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let text_width = draw::width(&type_scale::CHIP, word);
    let width = text_width as u32 + 8;
    // 13, not 11: the chip face inks 8 px, so an 11 px chip puts its own edge through the
    // last row of the word inside it.
    Rectangle::new(at, Size::new(width, CHIP_HEIGHT))
        .into_styled(PrimitiveStyle::with_fill(fill))
        .draw(target)?;
    draw::run(
        &type_scale::CHIP,
        word,
        at + Point::new(4, 3),
        VerticalPosition::Top,
        palette::CHIP_INK,
        target,
    );
    Ok(at.x + width as i32)
}

/// Draw an outlined chip in a pen's own hue, and return the x after it.
///
/// `outline` is the hue at rail luminance, so the border does not compete with the word.
pub fn chip_outlined<D>(
    word: Arguments<'_>,
    ink: Rgb565,
    outline: Rgb565,
    at: Point,
    target: &mut D,
) -> Result<i32, D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let text_width = draw::width(&type_scale::CHIP, word);
    let width = text_width as u32 + 8;
    // 13, not 11: the chip face inks 8 px, so an 11 px chip puts its own edge through the
    // last row of the word inside it.
    Rectangle::new(at, Size::new(width, CHIP_HEIGHT))
        .into_styled(PrimitiveStyle::with_stroke(outline, 1))
        .draw(target)?;
    draw::run(
        &type_scale::CHIP,
        word,
        at + Point::new(4, 3),
        VerticalPosition::Top,
        ink,
        target,
    );
    Ok(at.x + width as i32)
}

/// The height of a free-brewing rail.
pub const RAIL_HEIGHT: u32 = 12;

/// How far a 2 px command notch stands out of the rail, top and bottom.
const NOTCH_OVERHANG: i32 = 3;

/// Draw a rail: a trough, a fill for what the machine achieved, and a notch for what it was
/// asked for.
///
/// Deviation is read as the distance between the fill's edge and the notch, rather than as
/// arithmetic between two printed numbers -- which is the whole reason this is a rail and
/// not a third figure.
///
/// `fraction` and `notch` are 0..=1 of the rail's full scale. `notch` is `None` under duty
/// control, which has no downstream setpoint for the fill to be measured against.
pub fn rail<D>(
    area: Rectangle,
    fill_color: Rgb565,
    pen: Rgb565,
    fraction: f32,
    notch: Option<f32>,
    target: &mut D,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    Rectangle::new(area.top_left, area.size)
        .into_styled(PrimitiveStyle::with_fill(palette::TROUGH))
        .draw(target)?;

    let filled = (fraction.clamp(0.0, 1.0) * area.size.width as f32) as u32;
    if filled > 0 {
        Rectangle::new(area.top_left, Size::new(filled, area.size.height))
            .into_styled(PrimitiveStyle::with_fill(fill_color))
            .draw(target)?;
    }

    if let Some(notch) = notch {
        // Clamped one pixel in from the right edge so a command at full scale is still a
        // notch on the rail rather than a line beside it.
        let x = (notch.clamp(0.0, 1.0) * area.size.width as f32) as i32;
        let x = x.min(area.size.width as i32 - 2);
        Rectangle::new(
            Point::new(area.top_left.x + x, area.top_left.y - NOTCH_OVERHANG),
            Size::new(2, area.size.height + 2 * NOTCH_OVERHANG as u32),
        )
        .into_styled(PrimitiveStyle::with_fill(pen))
        .draw(target)?;
    }

    Ok(())
}

/// Draw a progress bar in the pen of the quantity being watched.
///
/// Used for a routine step's exit condition. A condition with no numeric progress does not
/// get an empty bar -- it gets no bar; see [`crate::view::ExitView`].
pub fn progress<D>(
    area: Rectangle,
    pen: Rgb565,
    fraction: f32,
    target: &mut D,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    Rectangle::new(area.top_left, area.size)
        .into_styled(PrimitiveStyle::with_fill(palette::TROUGH))
        .draw(target)?;
    let filled = (fraction.clamp(0.0, 1.0) * area.size.width as f32) as u32;
    if filled > 0 {
        Rectangle::new(area.top_left, Size::new(filled, area.size.height))
            .into_styled(PrimitiveStyle::with_fill(pen))
            .draw(target)?;
    }
    Ok(())
}

/// Draw a number, or the dash that stands for one nothing is reporting.
///
/// Returns the x the next run starts at, so a unit follows either without moving.
///
/// The distinction is the point: a sensor that is not answering is not a sensor reading
/// zero, and a panel that drew `0.0 g` for an unpaired scale would be lying about the shot.
pub fn value<D>(
    font: &FontRenderer,
    value: Option<f32>,
    decimals: usize,
    at: Point,
    vpos: VerticalPosition,
    color: Rgb565,
    target: &mut D,
) -> Result<i32, D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    match value {
        Some(v) => Ok(draw::run(
            font,
            format_args!("{:.*}", decimals, v),
            at,
            vpos,
            color,
            target,
        )),
        None => {
            // The dash is drawn on the baseline whatever the caller anchored the number
            // with, because a dash hung from the top of a 46 px cell would sit above the
            // digits it stands in for.
            let baseline = match vpos {
                VerticalPosition::Baseline => at,
                _ => Point::new(at.x, at.y + font_height(font) * 3 / 4),
            };
            let advance = type_scale::no_reading(baseline, color, target)?;
            Ok(at.x + advance as i32)
        }
    }
}

/// A face's rendered height, used only to drop a substituted dash onto the baseline.
fn font_height(font: &FontRenderer) -> i32 {
    font.get_rendered_dimensions(format_args!("0"), Point::zero(), VerticalPosition::Top)
        .ok()
        .and_then(|d| d.bounding_box)
        .map(|bb| bb.size.height as i32)
        .unwrap_or(8)
}
