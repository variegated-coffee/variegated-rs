//! Section 6.2, as recomposed after the second field review.
//!
//! Idle asks one question -- *can I pull a shot now?* -- and the first remediation put the
//! answer first and largest, which worked. What the second review found was everything
//! *around* it:
//!
//! * **The rhythm was inverted.** `READY` sat 1 px above `BREW`, which sat 0 px above `93.2`.
//!   Rows that had nothing to do with each other were tighter than a label and the number it
//!   names. Both gaps are now stated through [`crate::rhythm`] and come out 6 and inline.
//! * **The marker displaced the word.** `READY` began eleven pixels right of the `BREW` under
//!   it, because the coloured square was drawn in the text column. It hangs in the gutter now.
//! * **Heating was three ambers** -- the word in warn, the temperature in the brew pen, and an
//!   amber square -- in the one state where the word and the temperature are read together.
//!   The word is ink; the square carries the colour; the temperature keeps its pen.
//! * **The estimate was the dimmest thing on the screen.** `6:47 TO READY` is the only number
//!   on this panel anyone acts on -- it decides whether you wait or walk away -- and it was
//!   set at the floor beside a 25 px word that says what the machine's own noise already says.
//! * **The two boiler rows were built to different grammars**, brew stacked and steam inline,
//!   which made one look like a heading and the other like a footnote rather than one simply
//!   being larger. Both are inline now, label first: size carries the hierarchy, so the
//!   grammar does not have to carry it as well.

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use u8g2_fonts::types::{HorizontalAlignment, VerticalPosition};

use crate::draw;
use crate::geometry::Window;
use crate::palette;
use crate::rhythm::{self, Stack};
use crate::type_scale;
use crate::view::{IdleView, Readiness};
use crate::widgets;

/// Where the first row's ink starts, from the window top.
const TOP: i32 = rhythm::MARGIN;

/// Mid-gap between the answer and the brew row. See [`crate::states::overlay_floor`].
pub(crate) const OVERLAY_FLOOR: i32 = TOP
    + type_scale::STATE_WORD.height()
    + rhythm::PITCH
    + type_scale::ANSWER.height()
    + rhythm::PITCH / 2;

pub(crate) fn draw<D>(view: &IdleView, w: Window, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let mut stack = Stack::new(TOP);
    let clock_baseline = stack.row(&type_scale::STATE_WORD);
    let answer_baseline = stack.row(&type_scale::ANSWER);
    let brew_baseline = stack.row(&type_scale::PRIMARY_46);
    let steam_baseline = stack.row(&type_scale::SECONDARY_19);

    top_rule(view, w, w.at(0, clock_baseline).y, target);
    answer(view, w, w.at(0, answer_baseline).y, target)?;
    brew(view, w, w.at(0, brew_baseline).y, target)?;
    steam(view, w, w.at(0, steam_baseline).y, target)?;
    Ok(())
}

fn top_rule<D>(view: &IdleView, w: Window, baseline: i32, target: &mut D)
where
    D: DrawTarget<Color = Rgb565>,
{
    if let Some(clock) = view.clock {
        // No seconds. On an idle machine they are a value nobody acts on, and dropping them
        // buys the width the schedule line needs to sit clear of the status marks.
        draw::run(
            &type_scale::STATE_WORD,
            format_args!("{:02}:{:02}", clock.hour, clock.minute),
            Point::new(w.text_left(), baseline),
            VerticalPosition::Baseline,
            palette::INK,
            target,
        );
    }
    if let Some(off) = view.next_off {
        draw::aligned(
            &type_scale::LABEL,
            format_args!("NEXT OFF {:02}:{:02}", off.hour, off.minute),
            Point::new(w.body_right(), baseline),
            VerticalPosition::Baseline,
            HorizontalAlignment::Right,
            palette::INK_FAINT,
            target,
        );
    }
}

/// The one element in this state that changes a decision, drawn first and largest.
fn answer<D>(view: &IdleView, w: Window, baseline: i32, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let colour = match view.readiness {
        Readiness::Ready => palette::OK,
        Readiness::Heating { .. } => palette::WARN,
    };
    let word = match view.readiness {
        Readiness::Ready => "READY",
        Readiness::Heating { .. } => "HEATING",
    };

    w.marker(baseline, type_scale::ANSWER.ascent(), colour, target)?;

    // Ink, not the status hue. The square beside it is already saying green or amber, and a
    // word in the same amber as the temperature under it made heating read as one large
    // undifferentiated warning instead of as a state and a measurement.
    let after = draw::run(
        &type_scale::ANSWER,
        format_args!("{word}"),
        Point::new(w.text_left(), baseline),
        VerticalPosition::Baseline,
        palette::INK,
        target,
    );

    // The countdown is the half of `HEATING 6:47` that earns its size: the word says what the
    // machine is doing, which is audible from across the room, and the number says how long,
    // which is not knowable any other way.
    if let Readiness::Heating {
        eta_seconds: Some(eta),
    } = view.readiness
    {
        let after = draw::run(
            &type_scale::SECONDARY_19,
            format_args!("{}:{:02}", eta / 60, eta % 60),
            Point::new(after + rhythm::GAP, baseline),
            VerticalPosition::Baseline,
            palette::WARN,
            target,
        );
        draw::run(
            &type_scale::LABEL,
            format_args!("TO READY"),
            Point::new(after + rhythm::TIGHT, baseline),
            VerticalPosition::Baseline,
            palette::INK_FAINT,
            target,
        );
    }

    Ok(())
}

/// The brew boiler, inline and label first -- the same grammar as steam below it.
fn brew<D>(view: &IdleView, w: Window, baseline: i32, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let at = Point::new(w.text_left(), baseline);
    let after = draw::run(
        &type_scale::LABEL,
        format_args!("BREW"),
        at,
        VerticalPosition::Baseline,
        palette::INK_MUTED,
        target,
    );

    let after = widgets::value(
        &type_scale::PRIMARY_46,
        view.brew_temperature,
        1,
        Point::new(after + rhythm::GAP, baseline),
        VerticalPosition::Baseline,
        palette::PEN_BREW_BOILER,
        target,
    )?;
    let after = after + rhythm::TIGHT;
    type_scale::degree(
        Point::new(after, baseline - type_scale::UNIT_12.ascent()),
        palette::INK_FAINT,
        target,
    )?;
    let after = draw::run(
        &type_scale::UNIT_12,
        format_args!("C"),
        Point::new(after + 5, baseline),
        VerticalPosition::Baseline,
        palette::INK_FAINT,
        target,
    );

    // The target rather than a signed deviation, behind a rule rather than behind distance:
    // six pixels is not enough to divide two figures, and it is all a 6 px pitch allows.
    if let Some(setpoint) = view.brew_setpoint {
        let after = rhythm::divider(after, baseline, &type_scale::PRIMARY_46, target)?;
        draw::run(
            &type_scale::LABEL,
            format_args!("TARGET {setpoint:.1}"),
            Point::new(after, baseline),
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
fn steam<D>(view: &IdleView, w: Window, baseline: i32, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let face = &type_scale::SECONDARY_19;
    let at = Point::new(w.text_left(), baseline);
    let after = draw::run(
        &type_scale::LABEL,
        format_args!("STEAM"),
        at,
        VerticalPosition::Baseline,
        palette::INK_MUTED,
        target,
    );

    let after = widgets::value(
        face,
        view.steam_pressure,
        2,
        Point::new(after + rhythm::GAP, baseline),
        VerticalPosition::Baseline,
        palette::PEN_STEAM_BOILER,
        target,
    )?;
    let after = draw::run(
        &type_scale::LABEL,
        format_args!("BAR"),
        Point::new(after + rhythm::TIGHT, baseline),
        VerticalPosition::Baseline,
        palette::INK_FAINT,
        target,
    );

    let after = rhythm::divider(after, baseline, face, target)?;
    let after = widgets::value(
        face,
        view.steam_temperature,
        1,
        Point::new(after, baseline),
        VerticalPosition::Baseline,
        palette::INK_MUTED,
        target,
    )?;
    let after = after + rhythm::TIGHT;
    type_scale::degree(
        Point::new(after, baseline - type_scale::LABEL.ascent()),
        palette::INK_FAINT,
        target,
    )?;
    let after = draw::run(
        &type_scale::LABEL,
        format_args!("C"),
        Point::new(after + 5, baseline),
        VerticalPosition::Baseline,
        palette::INK_FAINT,
        target,
    );

    if let Some(bar) = view.steam_target_bar {
        let after = rhythm::divider(after, baseline, face, target)?;
        draw::run(
            &type_scale::LABEL,
            format_args!("TARGET {bar:.1}"),
            Point::new(after, baseline),
            VerticalPosition::Baseline,
            palette::INK_FAINT,
            target,
        );
    }

    Ok(())
}
