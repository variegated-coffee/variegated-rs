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
use crate::geometry::{Window, hairline_v};
use crate::marks;
use crate::palette;
use crate::type_scale;
use crate::view::{Command, FreeBrewView, MarkState};
use crate::widgets;

// This state spends the panel's full width on the rail, so it has no column down the right
// edge to line up with and its marks go along the header instead.

const CONTENT_LEFT_DX: i32 = 0;
const CONTENT_RIGHT_DX: i32 = crate::geometry::WINDOW_SIZE.width as i32;
const CONTENT_WIDTH: i32 = CONTENT_RIGHT_DX - CONTENT_LEFT_DX;

/// Top of the status-mark row, which shares the header with the state word.
const MARKS_DY: i32 = 0;

/// Baseline of the header's state word and the top of its chip.
const HEADER_BASELINE: i32 = 14;

/// Baseline of the command line. `inb21`, the tallest thing on it, inks 21 px above --
/// clear of the 18 px marks that end at `MARKS_DY + 18`.
const COMMAND_ROW_DY: i32 = 44;

/// The rail's top. Its notch overhangs 3 px either side.
const RAIL_DY: i32 = 52;

/// Top of the bottom row's labels.
///
/// The rail's tick labels are gone: the chip already names the scale, and four numbers under
/// a bar at the old 8 px floor were four things that could not be read saying what one word
/// says. The rail's job is the distance between the fill and the notch, which needs no axis.
const CELLS_LABEL_DY: i32 = 78;

/// Baseline of the bottom row's values. `inb19` inks 19 px above it.
const CELLS_VALUE_DY: i32 = 110;

/// What a mode makes of the rail, the chip and the units.
struct Scale {
    /// The chip's word.
    word: &'static str,
    /// The unit beside the commanded and measured values.
    unit: &'static str,
    /// Full-scale deflection.
    full: f32,
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
                unit: "BAR",
                full: 12.0,
                pen: palette::PEN_PRESSURE,
                fill: palette::RAIL_FILL_PRESSURE,
                decimals: 1,
            },
            bar,
        ),
        Command::FlowIn { ml_s } => (
            Scale {
                word: "FLOW IN",
                unit: "ML/S",
                full: 6.0,
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
        let unit_width = draw::width(&type_scale::LABEL, format_args!("{} IN", scale.unit));
        let right = w.at(CONTENT_RIGHT_DX, 0).x;
        draw::aligned(
            &type_scale::LABEL,
            format_args!("{} IN", scale.unit),
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

    Ok(())
}

fn bottom_row<D>(view: &FreeBrewView, w: Window, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    // Four cells. Which four depends on the mode: under duty control there is no downstream
    // setpoint, so pressure and flow -- the two consequences of a duty -- take equal weight
    // and the running total of water in gives up its cell.
    // Units in capitals, like every other label on the panel. At the raised floor they are
    // set in the same bold ten as the label above them, and a lowercase run beside an
    // uppercase one reads as two tiers where there is only one.
    let fourth: (&str, Option<f32>, usize, &str, Rgb565) = match view.command {
        Command::Duty { .. } => ("FLOW", view.flow_in_ml_s, 2, "ML/S", palette::PEN_FLOW_OUT),
        _ => ("IN", view.water_in_ml, 0, "ML", palette::PEN_WATER_IN),
    };

    let cells: [(&str, Option<f32>, usize, &str, Rgb565); 4] = [
        ("TIME", Some(view.elapsed_seconds), 1, "S", palette::INK),
        ("WEIGHT", view.weight_g, 1, "G", palette::PEN_WEIGHT),
        (
            "PRESSURE",
            view.pressure_bar,
            2,
            "BAR",
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
