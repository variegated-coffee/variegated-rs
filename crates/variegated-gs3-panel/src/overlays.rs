//! What goes over a state's panel.
//!
//! Three of them, and the rules about *when* each is drawn are the caller's -- the
//! provisioning banner and the activity box are both suppressed during a shot, and that is a
//! decision about the machine rather than about pixels. This module draws whatever it is
//! handed.

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyle, PrimitiveStyleBuilder, Rectangle};
use u8g2_fonts::types::{HorizontalAlignment, VerticalPosition};

use crate::draw;
use crate::geometry::{WINDOW_ORIGIN, WINDOW_SIZE, at, body_right};
use crate::palette;
use crate::type_scale;
use crate::view::Overlay;

/// The band the activity and dose overlays share.
///
/// Full window width, rather than the 200 px card the firmware drew before. A card narrower
/// than the panel has vertical edges, and a vertical edge lands mid-figure: the steam
/// temperature ran two pixels past the old card's right edge, leaving a lit sliver of a
/// digit beside it that reads as a rendering fault. A band has no vertical edges to cut on.
const BOX_HEIGHT: u32 = 66;

/// Where the band sits, from the window top.
///
/// Chosen so its horizontal edges do not cut either: it clears the labels above it and the
/// bottom rule below it on every state's panel.
const BOX_TOP_DY: i32 = 21;

/// The provisioning strip along the bottom edge. Tall enough to cover a bottom rule whole.
const BANNER_HEIGHT: u32 = 20;

/// Draw an overlay.
///
/// [`Overlay::Identify`] is handled in [`crate::render`] before anything else is drawn --
/// it replaces the screen rather than sitting on it -- so it is a no-op here.
pub fn draw<D>(overlay: &Overlay<'_>, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    match overlay {
        Overlay::Identify { .. } => Ok(()),
        Overlay::Provisioning { line } => provisioning(line, target),
        Overlay::Activity { label } => activity(label, target),
        Overlay::Dose { grams } => dose(*grams, target),
    }
}

/// The centred box, returning its top-left.
fn popup_box<D>(target: &mut D) -> Result<Point, D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let top_left = at(0, BOX_TOP_DY);
    // Stops short of the status strip. A red mark is a statement of consequence and the
    // strip is meant to be checkable at a glance; an overlay that covered it would hide
    // "the tank is empty" behind "hot water", which is the wrong way round.
    let size = Size::new((body_right() - WINDOW_ORIGIN.x) as u32, BOX_HEIGHT);
    // Opaque: the panel underneath is painted out, not shown through. Said here so the
    // layout test does not read a covered figure as one drawn through another.
    #[cfg(test)]
    crate::draw::probe::occlude(Rectangle::new(top_left, size));
    Rectangle::new(top_left, size)
        .into_styled(
            PrimitiveStyleBuilder::new()
                .fill_color(palette::SURFACE)
                .stroke_color(palette::HAIRLINE)
                .stroke_width(1)
                .build(),
        )
        .draw(target)?;
    Ok(top_left)
}

/// What the machine is doing, while it is doing it.
///
/// The tap and the steam valve had no feedback on the panel at all: the machine either made
/// a noise or it did not.
fn activity<D>(label: &str, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let top_left = popup_box(target)?;
    draw::aligned(
        &type_scale::STATE_WORD,
        format_args!("{label}"),
        Point::new(
            (WINDOW_ORIGIN.x + body_right()) / 2,
            top_left.y + BOX_HEIGHT as i32 / 2,
        ),
        VerticalPosition::Center,
        HorizontalAlignment::Center,
        palette::INK,
        target,
    );
    Ok(())
}

/// The dose the user just captured.
///
/// Two lines rather than one, and the number in the readout face: a dose is a value the
/// operator is checking, not an announcement.
fn dose<D>(grams: f32, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let top_left = popup_box(target)?;
    let centre_x = (WINDOW_ORIGIN.x + body_right()) / 2;
    draw::aligned(
        &type_scale::LABEL,
        format_args!("DOSE CAPTURED"),
        Point::new(centre_x, top_left.y + 16),
        VerticalPosition::Top,
        HorizontalAlignment::Center,
        palette::INK_MUTED,
        target,
    );
    // Centred as one unit: the number is measured, then the pair is placed, so the `g` does
    // not shift the digits off centre.
    let number_width = draw::width(&type_scale::PRIMARY_27, format_args!("{grams:.1}"));
    let unit_width = draw::width(&type_scale::UNIT_12, format_args!("g"));
    let left = centre_x - (number_width + 4 + unit_width) / 2;
    let baseline = top_left.y + 52;
    let after = draw::run(
        &type_scale::PRIMARY_27,
        format_args!("{grams:.1}"),
        Point::new(left, baseline),
        VerticalPosition::Baseline,
        palette::PEN_WEIGHT,
        target,
    );
    draw::run(
        &type_scale::UNIT_12,
        format_args!("g"),
        Point::new(after + 4, baseline),
        VerticalPosition::Baseline,
        palette::INK_MUTED,
        target,
    );
    Ok(())
}

/// The Improv provisioning window, along the bottom edge.
///
/// A strip rather than another mark in the status column: the column carries conditions the
/// operator does not act on, and a provisioning window is one that has to say what is
/// happening and where to go next. Covering the bottom rule is the right trade for a mode
/// that is transient, deliberately entered and self-expiring.
fn provisioning<D>(line: &str, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let top = at(0, WINDOW_SIZE.height as i32 - BANNER_HEIGHT as i32);
    // Stops short of the status strip, for the reason `popup_box` gives.
    let size = Size::new((body_right() - WINDOW_ORIGIN.x) as u32, BANNER_HEIGHT);
    // Opaque; see the note in `popup_box`.
    #[cfg(test)]
    crate::draw::probe::occlude(Rectangle::new(top, size));
    Rectangle::new(top, size)
        .into_styled(PrimitiveStyle::with_fill(palette::TROUGH))
        .draw(target)?;
    draw::aligned(
        &type_scale::STATE_WORD,
        format_args!("{line}"),
        Point::new(
            (WINDOW_ORIGIN.x + body_right()) / 2,
            top.y + BANNER_HEIGHT as i32 / 2,
        ),
        VerticalPosition::Center,
        HorizontalAlignment::Center,
        palette::INK,
        target,
    );
    Ok(())
}
