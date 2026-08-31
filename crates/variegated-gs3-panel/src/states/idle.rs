//! Section 6.2: idle.
//!
//! Brew temperature dominates, because it is the one figure that decides whether to pull.
//! Steam sits second with its pressure. The bottom rule states readiness in a word and names
//! the routine that will run.

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, PrimitiveStyle};
use u8g2_fonts::types::{HorizontalAlignment, VerticalPosition};

use crate::draw;
use crate::geometry::{PAD_BOTTOM, PAD_LEFT, PAD_TOP, at, body_right, hairline_h, hairline_v};
use crate::palette;
use crate::type_scale;
use crate::view::{IdleView, Readiness};
use crate::widgets;

const BOTTOM: i32 = crate::geometry::WINDOW_SIZE.height as i32 - PAD_BOTTOM;

/// Where the steam cell starts. The brew cell gets the wider share -- it holds the larger
/// number, and it is the number being read.
const STEAM_LEFT_DX: i32 = 196;

/// The hairline between the two cells.
const CELL_DIVIDER_DX: i32 = 186;

/// The rule above the readiness row.
const RULE_DY: i32 = 90;

/// Baseline of the brew temperature. `inb33` inks 33 px above it.
const BREW_BASELINE: i32 = 72;

/// Top of the setpoint-and-deviation line under it.
const TARGET_DY: i32 = 76;

/// Baseline of the steam temperature. `inb24` inks 24 px above it.
const STEAM_BASELINE: i32 = 62;

/// Baseline of the steam pressure. `inb16` inks 16 px above it.
const STEAM_PRESSURE_BASELINE: i32 = 84;

/// How far from its setpoint a boiler is still "at temperature".
///
/// Half a degree. Tighter and a settled boiler flickers between colours on sensor noise
/// alone; wider and the deviation stops being the thing the number above is read against.
const AT_TEMPERATURE_C: f32 = 0.5;

pub(crate) fn draw<D>(view: &IdleView<'_>, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    // The strip's divider. Everything else lives to the left of it.
    let right = body_right();
    hairline_v(
        Point::new(right, at(0, PAD_TOP).y),
        (BOTTOM - PAD_TOP) as u32,
        target,
    )?;
    let content_right = right - 10;

    top_rule(view, content_right, target);
    brew_cell(view, target)?;

    hairline_v(
        Point::new(at(CELL_DIVIDER_DX, 0).x, at(0, 26).y),
        (RULE_DY - 32) as u32,
        target,
    )?;

    steam_cell(view, target)?;

    hairline_h(
        at(PAD_LEFT, RULE_DY),
        (content_right - at(PAD_LEFT, 0).x) as u32,
        target,
    )?;
    readiness_row(view, content_right, target)?;

    Ok(())
}

fn top_rule<D>(view: &IdleView<'_>, content_right: i32, target: &mut D)
where
    D: DrawTarget<Color = Rgb565>,
{
    if let Some(clock) = view.clock {
        // The idle clock is a numeral in the word face, not a readout: it is read as text
        // and never redrawn digit by digit, so it does not need Inconsolata's cells.
        draw::run(
            &type_scale::STATE_WORD,
            format_args!(
                "{:02}:{:02}:{:02}",
                clock.hour, clock.minute, clock.second
            ),
            at(PAD_LEFT, PAD_TOP),
            VerticalPosition::Top,
            palette::INK_MUTED,
            target,
        );
    }
    if let Some(off) = view.next_off {
        draw::aligned(
            &type_scale::LABEL,
            format_args!("NEXT OFF {:02}:{:02}", off.hour, off.minute),
            Point::new(content_right, at(0, PAD_TOP + 1).y),
            VerticalPosition::Top,
            HorizontalAlignment::Right,
            palette::INK_FAINT,
            target,
        );
    }
}

fn brew_cell<D>(view: &IdleView<'_>, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    draw::run(
        &type_scale::LABEL,
        format_args!("BREW"),
        at(PAD_LEFT, 26),
        VerticalPosition::Top,
        palette::INK_MUTED,
        target,
    );

    let baseline = at(PAD_LEFT, BREW_BASELINE);
    let after = widgets::value(
        &type_scale::PRIMARY_46,
        view.brew_temperature,
        1,
        baseline,
        VerticalPosition::Baseline,
        palette::PEN_BREW_BOILER,
        target,
    )?;
    // The unit sits on the number's baseline and the ring on the unit's cap, which is what
    // makes the two read as one `degrees C` rather than as a dot and a letter.
    let after = after + 4;
    type_scale::degree(Point::new(after, baseline.y - 13), palette::INK_MUTED, target)?;
    draw::run(
        &type_scale::UNIT_14,
        format_args!("C"),
        Point::new(after + 5, baseline.y),
        VerticalPosition::Baseline,
        palette::INK_MUTED,
        target,
    );

    // Setpoint and signed deviation. The deviation is what the number above is actually
    // being read against, and it is the one place on the panel that draws a sign.
    if let Some(setpoint) = view.brew_setpoint {
        let after = draw::run(
            &type_scale::LABEL,
            format_args!("TARGET {setpoint:.1}"),
            at(PAD_LEFT, TARGET_DY),
            VerticalPosition::Top,
            palette::INK_FAINT,
            target,
        );
        if let Some(temperature) = view.brew_temperature {
            let deviation = temperature - setpoint;
            let colour = if deviation.abs() <= AT_TEMPERATURE_C {
                palette::OK
            } else {
                palette::WARN
            };
            draw::run(
                &type_scale::LABEL,
                format_args!("{deviation:+.1}"),
                Point::new(after + 6, at(0, TARGET_DY).y),
                VerticalPosition::Top,
                colour,
                target,
            );
        }
    }

    Ok(())
}

fn steam_cell<D>(view: &IdleView<'_>, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    draw::run(
        &type_scale::LABEL,
        format_args!("STEAM"),
        at(STEAM_LEFT_DX, 26),
        VerticalPosition::Top,
        palette::INK_MUTED,
        target,
    );

    let baseline = at(STEAM_LEFT_DX, STEAM_BASELINE);
    let after = widgets::value(
        &type_scale::PRIMARY_30,
        view.steam_temperature,
        1,
        baseline,
        VerticalPosition::Baseline,
        palette::PEN_STEAM_BOILER,
        target,
    )?;
    let after = after + 4;
    type_scale::degree(Point::new(after, baseline.y - 11), palette::INK_MUTED, target)?;
    draw::run(
        &type_scale::UNIT_12,
        format_args!("C"),
        Point::new(after + 5, baseline.y),
        VerticalPosition::Baseline,
        palette::INK_MUTED,
        target,
    );

    let baseline = at(STEAM_LEFT_DX, STEAM_PRESSURE_BASELINE);
    let after = widgets::value(
        &type_scale::NUMBER_FLOOR,
        view.steam_pressure,
        2,
        baseline,
        VerticalPosition::Baseline,
        palette::INK,
        target,
    )?;
    draw::run(
        &type_scale::LABEL,
        format_args!("bar"),
        Point::new(after + 4, baseline.y),
        VerticalPosition::Baseline,
        palette::INK_FAINT,
        target,
    );

    Ok(())
}

fn readiness_row<D>(
    view: &IdleView<'_>,
    content_right: i32,
    target: &mut D,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let baseline = at(PAD_LEFT, BOTTOM);

    // Status is stated, not implied: the word carries it and the dot only reinforces.
    let (word, colour) = match view.readiness {
        Readiness::Ready => ("READY", palette::OK),
        Readiness::Heating { .. } => ("HEATING", palette::WARN),
    };

    Circle::new(Point::new(baseline.x, baseline.y - 8), 6)
        .into_styled(PrimitiveStyle::with_fill(colour))
        .draw(target)?;

    let after = draw::run(
        &type_scale::STATE_WORD,
        format_args!("{word}"),
        Point::new(baseline.x + 12, baseline.y),
        VerticalPosition::Baseline,
        colour,
        target,
    );

    if let Readiness::Heating {
        eta_seconds: Some(eta),
    } = view.readiness
    {
        draw::run(
            &type_scale::LABEL,
            format_args!("{}:{:02} TO READY", eta / 60, eta % 60),
            Point::new(after + 8, baseline.y),
            VerticalPosition::Baseline,
            palette::INK_FAINT,
            target,
        );
    }

    // The routine that will run and the dose it will run against, right-aligned. Drawn
    // right-to-left because the dose is the piece that must not be pushed off the edge by a
    // long routine name.
    let mut x = content_right;
    if let Some(dose) = view.dose_g {
        let w = draw::width(&type_scale::LABEL, format_args!("{dose:.1} g"));
        x -= w;
        draw::run(
            &type_scale::LABEL,
            format_args!("{dose:.1} g"),
            Point::new(x, baseline.y),
            VerticalPosition::Baseline,
            palette::INK_FAINT,
            target,
        );
        x -= type_scale::SEPARATOR_WIDTH as i32;
        type_scale::separator(Point::new(x, baseline.y), palette::INK_FAINT, target)?;
    }
    if let Some(routine) = view.routine {
        let w = draw::width(&type_scale::LABEL, format_args!("{routine}"));
        draw::run(
            &type_scale::LABEL,
            format_args!("{routine}"),
            Point::new(x - w, baseline.y),
            VerticalPosition::Baseline,
            palette::INK_FAINT,
            target,
        );
    }

    Ok(())
}
