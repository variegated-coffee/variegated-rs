//! Section 6.2, as recomposed after the field review.
//!
//! Idle asks one question -- *can I pull a shot now?* -- and the specification answered it in
//! the smallest, dimmest, lowest element on the panel while two numbers that answer nothing
//! took the whole upper half. Read on the machine, `READY` was beaten for prominence by the
//! clock, both temperatures, the target and the steam pressure.
//!
//! So the state is laid out around the answer. `READY` is first and largest; the brew
//! temperature follows; steam is reduced to the one figure that decides whether it can steam.
//!
//! Two things the machine taught about colour, both in the palette rather than here:
//! **luminance outranks size on an emissive panel**, which is the opposite of paper -- the
//! brew temperature was set sixteen pixels larger than the steam one and read as the weaker
//! of the two -- and the signed deviation is gone, because a target beside the number says
//! the same thing without asking anyone to read a sign at the floor size.

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};
use u8g2_fonts::types::{HorizontalAlignment, VerticalPosition};

use crate::draw;
use crate::geometry::Window;
use crate::palette;
use crate::type_scale;
use crate::view::{IdleView, Readiness};
use crate::widgets;

/// Baseline of the top rule: the clock, and the next scheduled off.
const TOP_RULE_BASELINE: i32 = 12;

/// Baseline of the answer. `helvB24` inks 24 px above it.
const ANSWER_BASELINE: i32 = 42;

/// The dot beside it, and its size.
const ANSWER_DOT: u32 = 8;

/// Baseline of the `BREW` label. `helvB10` inks 11 px above it.
const BREW_LABEL_BASELINE: i32 = 54;

/// Baseline of the brew temperature. `inb33` inks 33 px above it.
const BREW_BASELINE: i32 = 90;

/// Baseline of the steam row. `inb19` inks 19 px above it, ending 4 px clear of the window.
const STEAM_BASELINE: i32 = 111;

pub(crate) fn draw<D>(view: &IdleView, w: Window, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    top_rule(view, w, target);
    answer(view, w, target)?;
    brew(view, w, target)?;
    steam(view, w, target)?;
    Ok(())
}

fn top_rule<D>(view: &IdleView, w: Window, target: &mut D)
where
    D: DrawTarget<Color = Rgb565>,
{
    if let Some(clock) = view.clock {
        // No seconds. On an idle machine they are a value nobody acts on, and dropping them
        // buys the width the schedule line needs to sit clear of the status marks.
        draw::run(
            &type_scale::STATE_WORD,
            format_args!("{:02}:{:02}", clock.hour, clock.minute),
            w.at(0, TOP_RULE_BASELINE),
            VerticalPosition::Baseline,
            palette::INK,
            target,
        );
    }
    if let Some(off) = view.next_off {
        draw::aligned(
            &type_scale::LABEL,
            format_args!("NEXT OFF {:02}:{:02}", off.hour, off.minute),
            Point::new(w.body_right(), w.at(0, TOP_RULE_BASELINE).y),
            VerticalPosition::Baseline,
            HorizontalAlignment::Right,
            palette::INK_FAINT,
            target,
        );
    }
}

/// The one element in this state that changes a decision, drawn first and largest.
fn answer<D>(view: &IdleView, w: Window, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let (word, colour) = match view.readiness {
        Readiness::Ready => ("READY", palette::OK),
        Readiness::Heating { .. } => ("HEATING", palette::WARN),
    };

    // A square rather than a circle: at eight pixels unaliased, a circle is a square with its
    // corners chipped, and the chipping is the only thing that distinguishes them.
    Rectangle::new(
        w.at(0, ANSWER_BASELINE - ANSWER_DOT as i32 - 6),
        Size::new(ANSWER_DOT, ANSWER_DOT),
    )
    .into_styled(PrimitiveStyle::with_fill(colour))
    .draw(target)?;

    let after = draw::run(
        &type_scale::ANSWER,
        format_args!("{word}"),
        w.at(ANSWER_DOT as i32 + 7, ANSWER_BASELINE),
        VerticalPosition::Baseline,
        colour,
        target,
    );

    // The estimate goes beside the word, where a number genuinely helps -- it is the one
    // thing a waiting operator wants that the word cannot carry.
    if let Readiness::Heating {
        eta_seconds: Some(eta),
    } = view.readiness
    {
        draw::run(
            &type_scale::LABEL,
            format_args!("{}:{:02} TO READY", eta / 60, eta % 60),
            Point::new(after + 9, w.at(0, ANSWER_BASELINE).y),
            VerticalPosition::Baseline,
            palette::INK_FAINT,
            target,
        );
    }

    Ok(())
}

fn brew<D>(view: &IdleView, w: Window, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    draw::run(
        &type_scale::LABEL,
        format_args!("BREW"),
        w.at(0, BREW_LABEL_BASELINE),
        VerticalPosition::Baseline,
        palette::INK_MUTED,
        target,
    );

    let baseline = w.at(0, BREW_BASELINE);
    let after = widgets::value(
        &type_scale::PRIMARY_46,
        view.brew_temperature,
        1,
        baseline,
        VerticalPosition::Baseline,
        palette::PEN_BREW_BOILER,
        target,
    )?;
    let after = after + 4;
    type_scale::degree(Point::new(after, baseline.y - 12), palette::INK_FAINT, target)?;
    let after = draw::run(
        &type_scale::UNIT_12,
        format_args!("C"),
        Point::new(after + 5, baseline.y),
        VerticalPosition::Baseline,
        palette::INK_FAINT,
        target,
    );

    // The target rather than a signed deviation. The deviation was set at the floor size and
    // asked the reader to parse a sign to learn something the target states directly.
    if let Some(setpoint) = view.brew_setpoint {
        draw::run(
            &type_scale::LABEL,
            format_args!("TARGET {setpoint:.1}"),
            Point::new(after + 10, baseline.y),
            VerticalPosition::Baseline,
            palette::INK_FAINT,
            target,
        );
    }

    Ok(())
}

/// Steam, reduced to the figure that decides whether it can steam.
///
/// The pressure carries the pen and the temperature drops to muted ink beside it, because
/// pressure is what a steam boiler is controlled on. On a temperature-controlled one the two
/// would swap -- see the note on [`IdleView::steam_pressure`].
fn steam<D>(view: &IdleView, w: Window, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let baseline = w.at(0, STEAM_BASELINE);
    let after = draw::run(
        &type_scale::LABEL,
        format_args!("STEAM"),
        baseline,
        VerticalPosition::Baseline,
        palette::INK_MUTED,
        target,
    );

    let after = widgets::value(
        &type_scale::SECONDARY_19,
        view.steam_pressure,
        2,
        Point::new(after + 10, baseline.y),
        VerticalPosition::Baseline,
        palette::PEN_STEAM_BOILER,
        target,
    )?;
    let after = draw::run(
        &type_scale::LABEL,
        format_args!("BAR"),
        Point::new(after + 4, baseline.y),
        VerticalPosition::Baseline,
        palette::INK_FAINT,
        target,
    );

    let after = widgets::value(
        &type_scale::SECONDARY_19,
        view.steam_temperature,
        1,
        Point::new(after + 10, baseline.y),
        VerticalPosition::Baseline,
        palette::INK_MUTED,
        target,
    )?;
    let after = after + 4;
    type_scale::degree(Point::new(after, baseline.y - 10), palette::INK_FAINT, target)?;
    let after = draw::run(
        &type_scale::LABEL,
        format_args!("C"),
        Point::new(after + 5, baseline.y),
        VerticalPosition::Baseline,
        palette::INK_FAINT,
        target,
    );

    if let Some(bar) = view.steam_target_bar {
        draw::run(
            &type_scale::LABEL,
            format_args!("TARGET {bar:.1}"),
            Point::new(after + 10, baseline.y),
            VerticalPosition::Baseline,
            palette::INK_FAINT,
            target,
        );
    }

    Ok(())
}
