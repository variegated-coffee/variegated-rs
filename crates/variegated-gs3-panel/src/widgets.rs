//! The pieces more than one state draws: a chip, a rail, a progress bar, and a number that
//! may not exist.

use core::fmt::Arguments;

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};
use u8g2_fonts::types::VerticalPosition;

use crate::draw;
use crate::palette;
use crate::type_scale::{self, Face};

/// The clear space between a chip's border and the word inside it.
///
/// Findings 21 and 22 were one fault: the rect was drawn around the text's *line* box with no
/// padding, so the word overflowed its own border horizontally -- `FLOW IN` and `PUMP DUTY`
/// both did, and `PRESSURE` escaped only by being narrower -- while the rect, centred on a
/// line box, sat below the baseline of the words beside it. Both are fixed by measuring the
/// string and padding it, at draw time, rather than by any constant width.
const CHIP_PAD_X: i32 = 3;
/// See [`CHIP_PAD_X`]. Two, so a 11 px cap box makes a 15 px chip.
const CHIP_PAD_Y: i32 = 2;

/// A chip's height: the chip face's cap box plus its padding, top and bottom.
pub const CHIP_HEIGHT: u32 = (type_scale::CHIP.ascent() + CHIP_PAD_Y * 2) as u32;

/// How far a chip's border paints above the cap box of the text it sits beside.
///
/// A chip is centred on its neighbour's cap box, so its rect reaches `CHIP_PAD_Y` past the
/// chip face's own ascent -- which is above the row's ink whenever the neighbour is set no
/// larger than the chip. Two pixels beside a `LABEL`, one beside a `STATE_WORD`.
///
/// A row that opens a stack has to allow for it: at the panel's 1 px top margin the `NEXT ON`
/// chip on the off panel was drawing its top edge one pixel outside the window, where the
/// bezel eats it and the chip reads as an open-topped bracket.
pub const fn chip_overhang(beside: &Face) -> i32 {
    let above = type_scale::CHIP.ascent() + CHIP_PAD_Y - beside.ascent();
    if above > 0 { above } else { 0 }
}

/// How wide the chip around `word` will be.
///
/// Exposed because a row that places a chip has to measure it *including* the padding -- the
/// other half of finding 22. A caller that measured the string alone would lay the next run
/// six pixels inside the border.
pub fn chip_width(word: Arguments<'_>) -> i32 {
    draw::width(&type_scale::CHIP, word) + CHIP_PAD_X * 2
}

/// Draw a chip so its word sits on `baseline`, the same baseline as the text beside it.
///
/// Anchoring on the baseline rather than on a top edge is what aligns the rect to its
/// neighbours' cap boxes: the border is derived from where the word will be, so the two
/// cannot disagree.
fn chip<D>(
    word: Arguments<'_>,
    ink: Rgb565,
    style: PrimitiveStyle<Rgb565>,
    left: i32,
    baseline: i32,
    target: &mut D,
) -> Result<i32, D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let width = chip_width(word);
    let top = baseline - type_scale::CHIP.ascent() - CHIP_PAD_Y;
    Rectangle::new(
        Point::new(left, top),
        Size::new(width as u32, CHIP_HEIGHT),
    )
    .into_styled(style)
    .draw(target)?;
    draw::run(
        &type_scale::CHIP,
        word,
        Point::new(left + CHIP_PAD_X, baseline),
        VerticalPosition::Baseline,
        ink,
        target,
    );
    Ok(left + width)
}

/// Draw a filled chip -- a word on a solid, in the chip face -- and return the x after it.
///
/// The specification uses one for the schedule action on the off panel. A filled chip says
/// the word is a *state of affairs*, where an outlined one says it is a setting; that is why
/// `NEXT ON` is filled and the free-brew mode chip is not.
pub fn chip_filled<D>(
    word: Arguments<'_>,
    fill: Rgb565,
    left: i32,
    baseline: i32,
    target: &mut D,
) -> Result<i32, D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    chip(
        word,
        palette::CHIP_INK,
        PrimitiveStyle::with_fill(fill),
        left,
        baseline,
        target,
    )
}

/// Draw an outlined chip in a pen's own hue, and return the x after it.
///
/// `outline` is the hue at rail luminance, so the border does not compete with the word.
pub fn chip_outlined<D>(
    word: Arguments<'_>,
    ink: Rgb565,
    outline: Rgb565,
    left: i32,
    baseline: i32,
    target: &mut D,
) -> Result<i32, D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    chip(
        word,
        ink,
        PrimitiveStyle::with_stroke(outline, 1),
        left,
        baseline,
        target,
    )
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

/// How wide [`value`] will draw, without drawing it.
///
/// For a caller that has to decide whether a cell fits before committing to it. It must stay
/// in step with [`value`]'s two branches -- the same format string, and the dash's own width
/// for a missing reading -- which is why it lives here rather than at the call site.
pub fn value_width(font: &Face, value: Option<f32>, decimals: usize) -> i32 {
    match value {
        Some(v) => draw::width(font, format_args!("{:.*}", decimals, v)),
        None => type_scale::NO_READING_WIDTH as i32,
    }
}

/// Draw a number, or the dash that stands for one nothing is reporting.
///
/// Returns the x the next run starts at, so a unit follows either without moving.
///
/// The distinction is the point: a sensor that is not answering is not a sensor reading
/// zero, and a panel that drew `0.0 g` for an unpaired scale would be lying about the shot.
pub fn value<D>(
    font: &Face,
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
            // digits it stands in for. The face's own ascent gives the drop exactly, where
            // the reported box used to give it to within a third of a cell.
            let baseline = match vpos {
                VerticalPosition::Baseline => at,
                _ => Point::new(at.x, at.y + font.ascent()),
            };
            let advance = type_scale::no_reading(baseline, color, target)?;
            Ok(at.x + advance as i32)
        }
    }
}
