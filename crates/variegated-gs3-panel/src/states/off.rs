//! Section 6.1: machine off, and power-save standby.
//!
//! An off machine is asked one question: when will it be on. So the clock is the hero, the
//! schedule sits beside it, and the residual boiler temperatures explain a machine that is
//! still warm.

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use u8g2_fonts::types::VerticalPosition;

use crate::draw;
use crate::geometry::{PAD_BOTTOM, PAD_LEFT, PAD_TOP, Window, hairline_v};
use crate::palette;
use crate::type_scale;
use crate::view::OffView;
use crate::widgets;

/// The right column's width.
///
/// The figure gives 128, against its narrower outline face. 110 is what the panel's own
/// `07:00` needs -- five `inb24` cells at 19 px -- and the 18 px it gives back is what lets
/// the hero clock and its seconds share the left column.
const RIGHT_WIDTH: i32 = 110;

/// Baseline of the hero clock, relative to the window top. `inb38` inks 38 px above it.
const CLOCK_BASELINE: i32 = 66;

/// Top of the next-event time. `inb24` inks 24 px down from here.
const NEXT_TIME_TOP: i32 = 44;

/// Top of the day-and-wait line under it.
const NEXT_WAIT_TOP: i32 = 76;

/// Baseline of the residual temperatures, at the foot of the right column.
const RESIDUAL_BASELINE: i32 = 105;

/// Content bottom, relative to the window top.
const BOTTOM: i32 = crate::geometry::WINDOW_SIZE.height as i32 - PAD_BOTTOM;

pub(crate) fn draw<D>(view: &OffView<'_>, w: Window, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let right_left = w.body_right() - RIGHT_WIDTH;
    let divider_x = right_left - 10;

    left_column(view, w, target)?;

    hairline_v(
        Point::new(divider_x, w.at(0, PAD_TOP).y),
        (BOTTOM - PAD_TOP) as u32,
        target,
    )?;

    right_column(view, w, right_left, target)?;
    Ok(())
}

fn left_column<D>(view: &OffView<'_>, w: Window, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    if let Some(date) = view.date {
        draw::run(
            &type_scale::LABEL,
            format_args!("{date}"),
            w.at(PAD_LEFT, PAD_TOP),
            VerticalPosition::Top,
            palette::INK_MUTED,
            target,
        );
    }

    // The hero, and the seconds beside it in the word face at a fifth the size. Seconds on
    // an off machine are not a value anyone acts on; they are what makes the clock read as
    // running rather than as stopped.
    let baseline = w.at(PAD_LEFT, CLOCK_BASELINE);
    if let Some(clock) = view.clock {
        let after = draw::run(
            &type_scale::HERO,
            format_args!("{:02}:{:02}", clock.hour, clock.minute),
            baseline,
            VerticalPosition::Baseline,
            palette::INK,
            target,
        );
        draw::run(
            &type_scale::UNIT_14,
            format_args!("{:02}", clock.second),
            Point::new(after + 4, baseline.y),
            VerticalPosition::Baseline,
            palette::INK_FAINT,
            target,
        );
    }

    // Standby replaces the off-since line rather than joining it. Standby is a state the
    // machine chose and can leave on its own, and "off since 21:30" would say the opposite.
    if view.standby {
        draw::run(
            &type_scale::STATE_WORD,
            format_args!("STANDBY"),
            w.at(PAD_LEFT, BOTTOM),
            VerticalPosition::Bottom,
            palette::WARN,
            target,
        );
    } else if let Some(since) = view.off_since {
        draw::run(
            &type_scale::LABEL,
            format_args!("OFF SINCE {:02}:{:02}", since.hour, since.minute),
            w.at(PAD_LEFT, BOTTOM),
            VerticalPosition::Bottom,
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
    let top = w.at(0, PAD_TOP).y;

    if let Some(next) = view.next {
        let after = draw::run(
            &type_scale::LABEL,
            format_args!("NEXT"),
            Point::new(left, top + 2),
            VerticalPosition::Top,
            palette::INK_MUTED,
            target,
        );
        // Filled, because the chip states what will happen rather than naming a setting.
        // `affirmative` rather than a match on the word: the caller knows whether an event
        // is the one the operator is waiting for, and this crate should not learn a
        // vocabulary of action names to find out.
        widgets::chip_filled(
            format_args!("{}", next.action),
            if next.affirmative {
                palette::OK
            } else {
                palette::INK_MUTED
            },
            Point::new(after + 5, top),
            target,
        )?;

        draw::run(
            &type_scale::PRIMARY_30,
            format_args!("{:02}:{:02}", next.at.hour, next.at.minute),
            Point::new(left, w.at(0, NEXT_TIME_TOP).y),
            VerticalPosition::Top,
            palette::INK,
            target,
        );

        let mut x = draw::run(
            &type_scale::LABEL,
            format_args!("{}", next.day),
            Point::new(left, w.at(0, NEXT_WAIT_TOP).y),
            VerticalPosition::Top,
            palette::INK_FAINT,
            target,
        );
        if let Some(minutes) = next.wait_minutes {
            // The separator is drawn rather than typed; see `type_scale`.
            x += type_scale::separator(
                Point::new(x, w.at(0, NEXT_WAIT_TOP).y + 6),
                palette::INK_FAINT,
                target,
            )? as i32;
            let point = Point::new(x, w.at(0, NEXT_WAIT_TOP).y);
            if minutes >= 60 {
                draw::run(
                    &type_scale::LABEL,
                    format_args!("{}h {:02}m", minutes / 60, minutes % 60),
                    point,
                    VerticalPosition::Top,
                    palette::INK_FAINT,
                    target,
                );
            } else {
                draw::run(
                    &type_scale::LABEL,
                    format_args!("{minutes}m"),
                    point,
                    VerticalPosition::Top,
                    palette::INK_FAINT,
                    target,
                );
            }
        }
    }

    // Residual temperatures, hue-coded per boiler, with the word that says why they are
    // here: a machine that is off but still at 58 degrees is not a machine that needs
    // twenty minutes to be usable.
    let baseline = w.at(0, RESIDUAL_BASELINE).y;
    let mut x = left;
    for (temperature, pen) in [
        (view.residual_brew, palette::PEN_BREW_BOILER),
        (view.residual_steam, palette::PEN_STEAM_BOILER),
    ] {
        let Some(temperature) = temperature else {
            continue;
        };
        let after = draw::run(
            &type_scale::STEP_OTHER,
            format_args!("{temperature:.0}"),
            Point::new(x, baseline),
            VerticalPosition::Baseline,
            pen,
            target,
        );
        let after = after + type_scale::degree(Point::new(after + 1, baseline - 8), pen, target)? as i32;
        x = after + 5;
    }
    if view.residual_brew.is_some() || view.residual_steam.is_some() {
        draw::run(
            &type_scale::LABEL,
            format_args!("RESIDUAL"),
            Point::new(x, baseline),
            VerticalPosition::Baseline,
            palette::INK_FAINT,
            target,
        );
    }

    Ok(())
}
