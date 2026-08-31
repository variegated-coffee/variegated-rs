//! Section 6.1: machine off, and power-save standby.
//!
//! An off machine is asked one question: when will it be on. So the clock is the hero, the
//! schedule sits beside it, and the residual boiler temperatures explain a machine that is
//! still warm.
//!
//! # What the second review changed
//!
//! * **An absent schedule is a different composition, not a blank field.** The divider was
//!   drawn whatever was beside it, so an unscheduled machine showed a rule with nothing to its
//!   right -- which reads as a screen that failed to load rather than as a machine with
//!   nothing planned. With no schedule there is no rule and the clock takes the window.
//! * **`41° 58° RESIDUAL` identified two boilers by colour alone**, and it was the only place
//!   in the set that did. The design system's rule is that a pen never travels without a
//!   label, and off is the state where nobody has the context to infer which boiler is amber:
//!   there is no `BREW` row above it to learn from, because the machine is off.

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use u8g2_fonts::types::{HorizontalAlignment, VerticalPosition};

use crate::draw;
use crate::geometry::{WINDOW_SIZE, Window, hairline_v};
use crate::palette;
use crate::rhythm::{self, Stack};
use crate::type_scale;
use crate::view::OffView;
use crate::widgets;

/// The right column's width.
///
/// 142, which is what `TOMORROW` beside a wait needs once the four label lines here came off
/// the 8 px floor.
const RIGHT_WIDTH: i32 = 142;

/// Where the first row's ink starts, from the window top.
const TOP: i32 = rhythm::MARGIN;

/// Where the footer's ink ends, relative to the window top.
const BOTTOM: i32 = WINDOW_SIZE.height as i32 - 1 - rhythm::MARGIN;

/// Mid-gap below the hero clock. See [`crate::states::overlay_floor`].
pub(crate) const OVERLAY_FLOOR: i32 = TOP
    + type_scale::LABEL.height()
    + rhythm::PITCH
    + type_scale::HERO.height()
    + rhythm::PITCH / 2;

/// The baseline of the footer row, which spans both columns.
fn footer_baseline(w: Window) -> i32 {
    w.at(0, BOTTOM).y - type_scale::LABEL.descent()
}

/// Where the columns stop, so neither runs into the footer.
fn columns_bottom() -> i32 {
    BOTTOM - type_scale::LABEL.height() - rhythm::PITCH
}

pub(crate) fn draw<D>(view: &OffView<'_>, w: Window, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    // No schedule, no division. A rule exists to separate two things.
    let split = view.next.is_some();
    let right_left = w.body_right() - RIGHT_WIDTH;

    left_column(view, w, split, target)?;

    if split {
        hairline_v(
            Point::new(right_left - rhythm::RULE_GAP, w.at(0, TOP).y),
            (columns_bottom() - TOP) as u32,
            target,
        )?;
        right_column(view, w, right_left, target)?;
    }

    // The residuals run the full width along the foot rather than sitting under the schedule.
    // Naming each boiler costs about 210 px where the bare figures cost 90, and the schedule
    // column is 142 -- but this row describes the machine, not what is planned for it, so the
    // width it needs is the panel's rather than a column's.
    residual(view, w, target)
}

fn left_column<D>(
    view: &OffView<'_>,
    w: Window,
    split: bool,
    target: &mut D,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let mut stack = Stack::new(TOP);
    let date_baseline = stack.row(&type_scale::LABEL);
    let clock_baseline = stack.row(&type_scale::HERO);

    // Centred when it has the window to itself, so an unscheduled panel is composed rather
    // than merely missing half of itself.
    let centre = (w.text_left() + w.body_right()) / 2;
    let (align, anchor) = if split {
        (HorizontalAlignment::Left, w.text_left())
    } else {
        (HorizontalAlignment::Center, centre)
    };

    if let Some(date) = view.date {
        draw::aligned(
            &type_scale::LABEL,
            format_args!("{date}"),
            Point::new(anchor, w.at(0, date_baseline).y),
            VerticalPosition::Baseline,
            align,
            palette::INK_MUTED,
            target,
        );
    }

    // The hero, and the seconds beside it in the word face at a fifth the size. Seconds on
    // an off machine are not a value anyone acts on; they are what makes the clock read as
    // running rather than as stopped.
    let baseline = w.at(0, clock_baseline).y;
    if let Some(clock) = view.clock {
        let hhmm = format_args!("{:02}:{:02}", clock.hour, clock.minute);
        let seconds = format_args!("{:02}", clock.second);
        let width = draw::width(&type_scale::HERO, hhmm)
            + rhythm::TIGHT
            + draw::width(&type_scale::UNIT_14, seconds);
        let left = match align {
            HorizontalAlignment::Center => anchor - width / 2,
            _ => anchor,
        };
        let after = draw::run(
            &type_scale::HERO,
            hhmm,
            Point::new(left, baseline),
            VerticalPosition::Baseline,
            palette::INK,
            target,
        );
        draw::run(
            &type_scale::UNIT_14,
            seconds,
            Point::new(after + rhythm::TIGHT, baseline),
            VerticalPosition::Baseline,
            palette::INK_FAINT,
            target,
        );
    }

    // Standby replaces the off-since line rather than joining it. Standby is a state the
    // machine chose and can leave on its own, and "off since 21:30" would say the opposite.
    let baseline = w.at(0, stack.row(&type_scale::STATE_WORD)).y;
    if view.standby {
        // Ink with a warn square, like every other state word on this panel.
        w.marker(
            baseline,
            type_scale::STATE_WORD.ascent(),
            palette::WARN,
            target,
        )?;
        draw::run(
            &type_scale::STATE_WORD,
            format_args!("STANDBY"),
            Point::new(w.text_left(), baseline),
            VerticalPosition::Baseline,
            palette::INK,
            target,
        );
    } else if let Some(since) = view.off_since {
        draw::run(
            &type_scale::LABEL,
            format_args!("OFF SINCE {:02}:{:02}", since.hour, since.minute),
            Point::new(w.text_left(), baseline),
            VerticalPosition::Baseline,
            palette::INK_FAINT,
            target,
        );
    }

    Ok(())
}

fn right_column<D>(view: &OffView<'_>, w: Window, left: i32, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let Some(next) = view.next else {
        return Ok(());
    };

    // The head row carries the schedule chip, whose border paints above the label's cap box.
    // Opening the stack at the panel's top margin would put that border outside the window.
    let mut stack = Stack::new(TOP + widgets::chip_overhang(&type_scale::LABEL));
    let head_baseline = stack.row(&type_scale::LABEL);
    let time_baseline = stack.paired(&type_scale::PRIMARY_30);
    let wait_baseline = stack.row(&type_scale::LABEL);

    let baseline = w.at(0, head_baseline).y;
    let after = draw::run(
        &type_scale::LABEL,
        format_args!("NEXT"),
        Point::new(left, baseline),
        VerticalPosition::Baseline,
        palette::INK_MUTED,
        target,
    );
    // Filled, because the chip states what will happen rather than naming a setting.
    // `affirmative` rather than a match on the word: the caller knows whether an event is the
    // one the operator is waiting for, and this crate should not learn a vocabulary of action
    // names to find out.
    widgets::chip_filled(
        format_args!("{}", next.action),
        if next.affirmative {
            palette::OK
        } else {
            palette::INK_MUTED
        },
        after + rhythm::GAP,
        baseline,
        target,
    )?;

    // Paired with the label above it: the time is what `NEXT` names.
    draw::run(
        &type_scale::PRIMARY_30,
        format_args!("{:02}:{:02}", next.at.hour, next.at.minute),
        Point::new(left, w.at(0, time_baseline).y),
        VerticalPosition::Baseline,
        palette::INK,
        target,
    );

    let baseline = w.at(0, wait_baseline).y;
    let mut x = draw::run(
        &type_scale::LABEL,
        format_args!("{}", next.day),
        Point::new(left, baseline),
        VerticalPosition::Baseline,
        palette::INK_FAINT,
        target,
    );
    if let Some(minutes) = next.wait_minutes {
        // The separator is drawn rather than typed; see `type_scale`.
        x += type_scale::separator(Point::new(x, baseline), palette::INK_FAINT, target)? as i32;
        let point = Point::new(x, baseline);
        if minutes >= 60 {
            draw::run(
                &type_scale::LABEL,
                format_args!("{}h {:02}m", minutes / 60, minutes % 60),
                point,
                VerticalPosition::Baseline,
                palette::INK_FAINT,
                target,
            );
        } else {
            draw::run(
                &type_scale::LABEL,
                format_args!("{minutes}m"),
                point,
                VerticalPosition::Baseline,
                palette::INK_FAINT,
                target,
            );
        }
    }

    Ok(())
}

/// The residual temperatures, each named by its boiler.
///
/// A machine that is off but still at 58 degrees is not a machine that needs twenty minutes to
/// be usable, which is the whole point of the row -- and which boiler is which is exactly the
/// thing a colour cannot say to someone who has no other row on the screen to learn it from.
fn residual<D>(view: &OffView<'_>, w: Window, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    if view.residual_brew.is_none() && view.residual_steam.is_none() {
        return Ok(());
    }

    // From the window's edge, like post-routine's stats footer and for the same reason: this
    // row spans both columns, so it is not in the stack the `STANDBY` marker heads.
    let baseline = footer_baseline(w);
    let mut x = w.at(0, 0).x;
    let mut first = true;
    for (label, temperature, pen) in [
        ("BREW", view.residual_brew, palette::PEN_BREW_BOILER),
        ("STEAM", view.residual_steam, palette::PEN_STEAM_BOILER),
    ] {
        let Some(temperature) = temperature else {
            continue;
        };
        if !first {
            x = rhythm::divider(x, baseline, &type_scale::LABEL, target)?;
        }
        first = false;
        let after = draw::run(
            &type_scale::LABEL,
            format_args!("{label}"),
            Point::new(x, baseline),
            VerticalPosition::Baseline,
            palette::INK_MUTED,
            target,
        );
        let after = draw::run(
            &type_scale::LABEL,
            format_args!("{temperature:.0}"),
            Point::new(after + rhythm::TIGHT, baseline),
            VerticalPosition::Baseline,
            pen,
            target,
        );
        x = after
            + type_scale::degree(
                Point::new(after + 1, baseline - type_scale::LABEL.ascent()),
                pen,
                target,
            )? as i32;
    }

    // Once, at the end: it qualifies both readings and is not part of either.
    let x = rhythm::divider(x, baseline, &type_scale::LABEL, target)?;
    draw::run(
        &type_scale::LABEL,
        format_args!("RESIDUAL"),
        Point::new(x, baseline),
        VerticalPosition::Baseline,
        palette::INK_FAINT,
        target,
    );

    Ok(())
}
