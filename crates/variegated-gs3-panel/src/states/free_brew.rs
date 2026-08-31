//! Section 6.3: free-brewing.
//!
//! Free-brewing always commands exactly one variable -- a pressure, a flow, or a pump duty.
//! That command is named in the header chip and drawn as a notch on the rail; the fill is
//! what the machine actually achieved, so deviation is read as distance rather than as
//! arithmetic. The rail's unit and full scale change with the mode; nothing else moves.
//!
//! This is the one state that spends the panel's full width, so its status marks go along
//! the header rather than down the right edge.

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Rectangle;
use u8g2_fonts::types::{HorizontalAlignment, VerticalPosition};

use crate::draw;
use crate::geometry::{PAD_LEFT, Window, hairline_v};
use crate::marks;
use crate::palette;
use crate::type_scale;
use crate::view::{Command, FreeBrewView, MarkState};
use crate::widgets;

// Padding is 6 top and bottom here rather than 8: the header, the command line, a 12 px
// rail, its ticks and a two-line value row need the four pixels, and this state has no
// column down the right edge to line up with.

const CONTENT_LEFT_DX: i32 = PAD_LEFT;
const CONTENT_RIGHT_DX: i32 = crate::geometry::WINDOW_SIZE.width as i32 - PAD_LEFT;
const CONTENT_WIDTH: i32 = CONTENT_RIGHT_DX - CONTENT_LEFT_DX;

/// Top of the status-mark row, which shares the header with the state word.
const MARKS_DY: i32 = 3;

/// Baseline of the header's state word and the top of its chip.
const HEADER_BASELINE: i32 = 18;

/// Baseline of the command line. `inb21`, the tallest thing on it, inks 21 px above --
/// clear of the 16 px marks that end at `MARKS_DY + 16`.
const COMMAND_ROW_DY: i32 = 42;

/// The rail's top. Its notch overhangs 3 px either side.
const RAIL_DY: i32 = 48;

/// Top of the rail's tick labels, below the notch's lower overhang.
const TICKS_DY: i32 = 64;

/// Top of the bottom row's labels.
const CELLS_LABEL_DY: i32 = 78;

/// Baseline of the bottom row's values. `inb19` inks 19 px above it.
const CELLS_VALUE_DY: i32 = 107;

/// What a mode makes of the rail, the chip and the units.
struct Scale {
    /// The chip's word.
    word: &'static str,
    /// The unit beside the commanded and measured values.
    unit: &'static str,
    /// Full-scale deflection.
    full: f32,
    /// The tick labels under the rail, left to right, including both ends.
    ticks: &'static [u8],
    /// The pen the command is drawn in.
    pen: Rgb565,
    /// The rail's fill.
    fill: Rgb565,
    /// How many decimals the commanded value gets.
    decimals: usize,
}

fn scale_of(command: Command) -> (Scale, f32) {
    match command {
        Command::Pressure { bar } => (
            Scale {
                word: "PRESSURE",
                unit: "bar",
                full: 12.0,
                ticks: &[0, 4, 8, 12],
                pen: palette::PEN_PRESSURE,
                fill: palette::RAIL_FILL_PRESSURE,
                decimals: 1,
            },
            bar,
        ),
        Command::FlowIn { ml_s } => (
            Scale {
                word: "FLOW IN",
                unit: "mL/s",
                full: 6.0,
                ticks: &[0, 2, 4, 6],
                pen: palette::PEN_FLOW_OUT,
                fill: palette::RAIL_FILL_FLOW,
                decimals: 1,
            },
            ml_s,
        ),
        Command::Duty { percent } => (
            Scale {
                word: "PUMP DUTY",
                unit: "%",
                full: 100.0,
                ticks: &[0, 50, 100],
                pen: palette::INK_MUTED,
                fill: palette::RAIL_FILL_DUTY,
                decimals: 0,
            },
            percent,
        ),
    }
}

pub(crate) fn draw<D>(view: &FreeBrewView, w: Window, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let (scale, commanded) = scale_of(view.command);
    header(&scale, w, target)?;
    command_row(view, &scale, commanded, w, target)?;
    rail(view, &scale, commanded, w, target)?;
    bottom_row(view, w, target)?;
    Ok(())
}

/// The header row, including the status marks. The marks are drawn here rather than by
/// [`crate::render`] because this is the only state whose strip is horizontal.
pub(crate) fn header_marks<D>(
    states: &[MarkState; 5],
    w: Window,
    target: &mut D,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    marks::draw_row(states, w.at(CONTENT_RIGHT_DX, 0).x, w.at(0, MARKS_DY).y, target)
}

fn header<D>(scale: &Scale, w: Window, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let after = draw::run(
        &type_scale::STATE_WORD,
        format_args!("FREE BREW"),
        w.at(CONTENT_LEFT_DX, HEADER_BASELINE),
        VerticalPosition::Baseline,
        palette::INK,
        target,
    );
    // Outlined rather than filled: the chip names the variable being commanded, which is a
    // setting the operator chose, not something that has happened.
    widgets::chip_outlined(
        format_args!("{}", scale.word),
        scale.pen,
        scale.fill,
        Point::new(after + 7, w.at(0, HEADER_BASELINE - 10).y),
        target,
    )?;
    Ok(())
}

fn command_row<D>(
    view: &FreeBrewView,
    scale: &Scale,
    commanded: f32,
    w: Window,
    target: &mut D,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let baseline = w.at(CONTENT_LEFT_DX, COMMAND_ROW_DY);

    let after = draw::run(
        &type_scale::LABEL,
        format_args!("COMMAND"),
        baseline,
        VerticalPosition::Baseline,
        palette::INK_MUTED,
        target,
    );
    let after = draw::run(
        &type_scale::NUMBER_FLOOR,
        format_args!("{:.*}", scale.decimals, commanded),
        Point::new(after + 5, baseline.y),
        VerticalPosition::Baseline,
        scale.pen,
        target,
    );
    draw::run(
        &type_scale::LABEL,
        format_args!("{}", scale.unit),
        Point::new(after + 4, baseline.y),
        VerticalPosition::Baseline,
        palette::INK_FAINT,
        target,
    );

    // Under duty control there is nothing downstream to measure the command against, so the
    // measured slot is empty and both consequences go in the bottom row instead.
    if let Some(measured) = view.measured {
        let unit_width = draw::width(&type_scale::LABEL, format_args!("{} in", scale.unit));
        let right = w.at(CONTENT_RIGHT_DX, 0).x;
        draw::aligned(
            &type_scale::LABEL,
            format_args!("{} in", scale.unit),
            Point::new(right, baseline.y),
            VerticalPosition::Baseline,
            HorizontalAlignment::Right,
            palette::INK_FAINT,
            target,
        );
        draw::aligned(
            &type_scale::SECONDARY_21,
            format_args!("{:.2}", measured),
            Point::new(right - unit_width - 4, baseline.y),
            VerticalPosition::Baseline,
            HorizontalAlignment::Right,
            palette::INK,
            target,
        );
    }

    Ok(())
}

fn rail<D>(
    view: &FreeBrewView,
    scale: &Scale,
    commanded: f32,
    w: Window,
    target: &mut D,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let area = Rectangle::new(
        w.at(CONTENT_LEFT_DX, RAIL_DY),
        Size::new(CONTENT_WIDTH as u32, widgets::RAIL_HEIGHT),
    );

    // The fill is what the machine achieved. Under duty control the command *is* what the
    // machine achieved, so the fill stands alone and the notch is omitted.
    let achieved = view.measured.unwrap_or(commanded);
    let notch = view.measured.is_some().then_some(commanded / scale.full);
    widgets::rail(
        area,
        scale.fill,
        scale.pen,
        achieved / scale.full,
        notch,
        target,
    )?;

    // Ticks label the full scale, so the notch's position means something without a second
    // number to compare it against.
    let last = scale.ticks.len().saturating_sub(1).max(1) as i32;
    for (i, tick) in scale.ticks.iter().enumerate() {
        let x = w.at(CONTENT_LEFT_DX, 0).x + CONTENT_WIDTH * i as i32 / last;
        let align = if i == 0 {
            HorizontalAlignment::Left
        } else if i as i32 == last {
            HorizontalAlignment::Right
        } else {
            HorizontalAlignment::Center
        };
        draw::aligned(
            &type_scale::LABEL,
            format_args!("{tick}"),
            Point::new(x, w.at(0, TICKS_DY).y),
            VerticalPosition::Top,
            align,
            palette::INK_FAINT,
            target,
        );
    }

    Ok(())
}

fn bottom_row<D>(view: &FreeBrewView, w: Window, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    // Four cells. Which four depends on the mode: under duty control there is no downstream
    // setpoint, so pressure and flow -- the two consequences of a duty -- take equal weight
    // and the running total of water in gives up its cell.
    let fourth: (&str, Option<f32>, usize, &str, Rgb565) = match view.command {
        Command::Duty { .. } => (
            "FLOW",
            view.flow_in_ml_s,
            2,
            "mL/s",
            palette::PEN_FLOW_OUT,
        ),
        _ => ("IN", view.water_in_ml, 0, "mL", palette::PEN_WATER_IN),
    };

    let cells: [(&str, Option<f32>, usize, &str, Rgb565); 4] = [
        ("TIME", Some(view.elapsed_seconds), 1, "s", palette::INK),
        ("WEIGHT", view.weight_g, 1, "g", palette::PEN_WEIGHT),
        (
            "PRESSURE",
            view.pressure_bar,
            2,
            "bar",
            palette::PEN_PRESSURE,
        ),
        fourth,
    ];

    let cell_width = CONTENT_WIDTH / 4;
    for (i, (label, value, decimals, unit, pen)) in cells.iter().enumerate() {
        let left = w.at(CONTENT_LEFT_DX + cell_width * i as i32, 0).x;
        // A hairline between cells, not around them: the panel's only decoration is a
        // divider, and a box round each figure would be four more.
        if i > 0 {
            hairline_v(
                Point::new(left - 9, w.at(0, CELLS_LABEL_DY - 4).y),
                (CELLS_VALUE_DY - CELLS_LABEL_DY + 6) as u32,
                target,
            )?;
        }
        draw::run(
            &type_scale::LABEL,
            format_args!("{label}"),
            Point::new(left, w.at(0, CELLS_LABEL_DY).y),
            VerticalPosition::Top,
            palette::INK_MUTED,
            target,
        );
        let baseline = Point::new(left, w.at(0, CELLS_VALUE_DY).y);
        let after = widgets::value(
            &type_scale::SECONDARY_19,
            *value,
            *decimals,
            baseline,
            VerticalPosition::Baseline,
            *pen,
            target,
        )?;
        draw::run(
            &type_scale::LABEL,
            format_args!("{unit}"),
            Point::new(after + 3, baseline.y),
            VerticalPosition::Baseline,
            palette::INK_FAINT,
            target,
        );
    }

    Ok(())
}
