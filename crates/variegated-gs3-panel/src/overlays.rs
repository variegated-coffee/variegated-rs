//! What goes over a state's panel.
//!
//! Three of them, and the rules about *when* each is drawn are the caller's -- the
//! provisioning banner and the activity box are both suppressed during a shot, and that is a
//! decision about the machine rather than about pixels. This module draws whatever it is
//! handed.
//!
//! # Two rules, from the second review
//!
//! **An overlay takes the top rows, never the bottom.** The provisioning banner sat along the
//! bottom edge, where it covered the steam row and the lowest status mark -- so the state it
//! obscured was the one telling you whether the machine is safe to use. Everything on this
//! panel is composed downwards from the answer, which means the bottom row is where the
//! qualifications live, and burying a qualification is worse than burying a headline.
//!
//! **An overlay is sized to its content.** The activity box was 81 px tall and held one
//! centred word; a large ruled box holding `STEAMING` is a box saying *something has gone
//! wrong* about a machine doing exactly what it was asked. `STEAMING` needs a line, not a box.
//!
//! **And all three are left-aligned**, like every other row on this panel. There were two
//! alignments across three overlays -- `WI-FI SETUP: READY TO PAIR` centred, `DOSE 19.9 G` not
//! -- which made them read as three unrelated things rather than as one mechanism. Centring
//! also put the one word furthest from the left edge every other row starts at, which is the
//! edge the eye is already on.

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};
use u8g2_fonts::types::VerticalPosition;

use crate::draw;
use crate::geometry::Window;
use crate::palette;
use crate::rhythm;
use crate::type_scale;
use crate::view::Overlay;

/// The clear space inside a band, above and below its content.
const PAD: i32 = 6;

/// Draw an overlay.
///
/// [`Overlay::Identify`] is handled in [`crate::render`] before anything else is drawn --
/// it replaces the screen rather than sitting on it -- so it is a no-op here.
/// `floor` is the lowest edge at which this state's rows allow a band to stop; see
/// [`crate::states::overlay_floor`].
pub fn draw<D>(
    overlay: &Overlay<'_>,
    w: Window,
    floor: i32,
    target: &mut D,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    match overlay {
        Overlay::Identify { .. } => Ok(()),
        Overlay::Provisioning { line } => provisioning(line, w, floor, target),
        Overlay::Activity { label } => activity(label, w, floor, target),
        Overlay::Dose { grams } => dose(*grams, w, floor, target),
    }
}

/// Paint an opaque band across the top of the window, and return its top.
///
/// Flush to the window's top edge, so the band has one horizontal edge rather than two. Its
/// height is its own content plus padding, or `floor` if that is lower -- **sized to content,
/// but never stopping mid-row.** An edge through the middle of a 33 px temperature reads as a
/// rendering fault rather than as an overlay, and the states do not share a row grid, so where
/// the gaps are is a question only the state underneath can answer.
///
/// It stops short of the status strip for the same reason it no longer reaches the bottom
/// edge: a red mark is a statement of consequence, and an overlay that covered one would hide
/// "the tank is empty" behind "hot water".
/// Returns the baseline a single row of `face` sits on inside the band.
fn band<D>(
    w: Window,
    face: &crate::type_scale::Face,
    floor: i32,
    target: &mut D,
) -> Result<i32, D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let height = (face.height() + PAD * 2).max(floor);
    let top_left = w.at(0, 0);
    let size = Size::new((w.body_right() - w.origin().x) as u32, height as u32);
    // Opaque: the panel underneath is painted out, not shown through. Said here so the
    // layout test does not read a covered figure as one drawn through another.
    #[cfg(test)]
    crate::draw::probe::occlude(Rectangle::new(top_left, size));
    Rectangle::new(top_left, size)
        .into_styled(PrimitiveStyle::with_fill(palette::TROUGH))
        .draw(target)?;
    // Centred in whatever height the band ended up with, so a band pushed down to the state's
    // next gap holds its line in the middle rather than pinned to the top.
    Ok(top_left.y + (height - face.height()) / 2 + face.ascent())
}

/// What the machine is doing, while it is doing it.
///
/// The tap and the steam valve had no feedback on the panel at all: the machine either made
/// a noise or it did not. One line, in a band the height of that line -- an announcement, not
/// an alarm.
fn activity<D>(label: &str, w: Window, floor: i32, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let baseline = band(w, &type_scale::STATE_WORD, floor, target)?;
    draw::run(
        &type_scale::STATE_WORD,
        format_args!("{label}"),
        Point::new(w.text_left(), baseline),
        VerticalPosition::Baseline,
        palette::INK,
        target,
    );
    Ok(())
}

/// The dose the user just captured.
///
/// Two runs rather than two rows: a dose is a value the operator is checking, and it is being
/// checked against nothing, so it needs a label beside it and no more room than that.
fn dose<D>(grams: f32, w: Window, floor: i32, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let baseline = band(w, &type_scale::PRIMARY_27, floor, target)?;
    let after = draw::run(
        &type_scale::LABEL,
        format_args!("DOSE"),
        Point::new(w.text_left(), baseline),
        VerticalPosition::Baseline,
        palette::INK_MUTED,
        target,
    );
    let after = draw::run(
        &type_scale::PRIMARY_27,
        format_args!("{grams:.1}"),
        Point::new(after + rhythm::GAP, baseline),
        VerticalPosition::Baseline,
        palette::PEN_WEIGHT,
        target,
    );
    let after = draw::run(
        &type_scale::UNIT_12,
        format_args!("G"),
        Point::new(after + rhythm::TIGHT, baseline),
        VerticalPosition::Baseline,
        palette::INK_FAINT,
        target,
    );
    let after = rhythm::divider(after, baseline, &type_scale::PRIMARY_27, target)?;
    draw::run(
        &type_scale::LABEL,
        format_args!("CAPTURED"),
        Point::new(after, baseline),
        VerticalPosition::Baseline,
        palette::INK_FAINT,
        target,
    );
    Ok(())
}

/// The Improv provisioning window.
///
/// A strip rather than another mark in the status column: the column carries conditions the
/// operator does not act on, and a provisioning window is one that has to say what is
/// happening and where to go next. It takes the top rows, like every other overlay -- along
/// the bottom it covered the steam row and the lowest status mark, which is to say it hid
/// whether the machine was safe to use behind a transient, self-expiring mode the operator
/// entered on purpose.
fn provisioning<D>(line: &str, w: Window, floor: i32, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let baseline = band(w, &type_scale::STATE_WORD, floor, target)?;
    draw::run(
        &type_scale::STATE_WORD,
        format_args!("{line}"),
        Point::new(w.text_left(), baseline),
        VerticalPosition::Baseline,
        palette::INK,
        target,
    );
    Ok(())
}
