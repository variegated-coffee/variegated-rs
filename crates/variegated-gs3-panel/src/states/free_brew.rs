//! Section 6.3: free-brewing.
//!
//! Free-brewing always commands exactly one variable -- a pressure, a flow, or a pump duty.
//! That command is named in the header chip and drawn as a notch on the rail; the fill is
//! what the machine actually achieved, so deviation is read as distance rather than as
//! arithmetic. The rail's unit and full scale change with the mode; nothing else moves.
//!
//! # What the second review changed
//!
//! * **The status marks come home.** This state used to lay them along its header, at half the
//!   spacing, so it could spend the panel's full width on the rail. Across the set that made
//!   the strip move and shrink on the four screens where the machine is actually doing
//!   something. The rail gives up twenty pixels instead, which it had to spare.
//! * **Hue no longer separates the command from the measurement.** `COMMAND 2.0 ML/S` was
//!   drawn in the quantity's pen and the achieved `1.76` in plain ink -- so the fact, what the
//!   machine is actually doing, was the one figure with no identity, and two readings of the
//!   same physical quantity were told apart by a colour that means the quantity. Both carry
//!   the pen now. **Hue answers *what*; size answers *which***: the measurement is large, the
//!   command is smaller and muted, and each is labelled.

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Rectangle;
use u8g2_fonts::types::VerticalPosition;

use crate::draw;
use crate::geometry::Window;
use crate::palette;
use crate::rhythm::{self, Stack};
use crate::type_scale;
use crate::view::{Command, FreeBrewView};
use crate::widgets;

/// Where the first row's ink starts, from the window top.
const TOP: i32 = rhythm::MARGIN;

/// Mid-gap between the values row and the rail. See [`crate::states::overlay_floor`].
pub(crate) const OVERLAY_FLOOR: i32 = TOP
    + type_scale::STATE_WORD.height()
    + rhythm::PITCH
    + type_scale::SECONDARY_21.height()
    + rhythm::PITCH / 2;

/// What a mode makes of the rail, the chip and the units.
struct Scale {
    /// The chip's word.
    word: &'static str,
    /// The unit beside the commanded and measured values.
    unit: &'static str,
    /// Full-scale deflection.
    full: f32,
    /// The pen both figures are drawn in.
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

    // The header carries the mode chip; see the note in `off`. A `STATE_WORD` is one pixel
    // taller than the chip face, so this costs nothing here -- but it is the same rule.
    let mut stack = Stack::new(TOP.max(widgets::chip_overhang(&type_scale::STATE_WORD)));
    let header_baseline = stack.row(&type_scale::STATE_WORD);
    let values_baseline = stack.row(&type_scale::SECONDARY_21);
    let rail_top = stack.block(widgets::RAIL_HEIGHT as i32);
    let cells_label = stack.row(&type_scale::LABEL);
    let cells_value = stack.paired(&type_scale::SECONDARY_19);

    header(&scale, w, w.at(0, header_baseline).y, target)?;
    values(view, &scale, commanded, w, w.at(0, values_baseline).y, target)?;
    rail(view, &scale, commanded, w, rail_top, target)?;
    bottom_row(
        view,
        w,
        w.at(0, cells_label).y,
        w.at(0, cells_value).y,
        target,
    )?;
    Ok(())
}

fn header<D>(scale: &Scale, w: Window, baseline: i32, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let after = draw::run(
        &type_scale::STATE_WORD,
        format_args!("FREE BREW"),
        Point::new(w.at(0, 0).x, baseline),
        VerticalPosition::Baseline,
        palette::INK,
        target,
    );
    // Outlined rather than filled: the chip names the variable being commanded, which is a
    // setting the operator chose, not something that has happened. On the word's own baseline,
    // so the rect aligns to the cap box beside it rather than to a line box neither shares.
    widgets::chip_outlined(
        format_args!("{}", scale.word),
        scale.pen,
        scale.fill,
        after + rhythm::GAP,
        baseline,
        target,
    )?;
    Ok(())
}

/// What the machine is doing, and what it was told to do.
///
/// The measurement first and largest. Under duty control there is nothing downstream to
/// measure the command against, so the commanded duty stands alone as the fact and both of its
/// consequences go in the bottom row.
fn values<D>(
    view: &FreeBrewView,
    scale: &Scale,
    commanded: f32,
    w: Window,
    baseline: i32,
    target: &mut D,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let left = w.at(0, 0).x;

    // `ACTUAL` and `COMMAND`, which is the pair the review asks for: two readings of one
    // quantity, told apart by what they are rather than by which is which colour. Naming the
    // measurement by its unit instead -- `ML/S 1.76` -- reads as a third quantity, and the
    // chip above has already said which one this is.
    let after = match view.measured {
        Some(measured) => {
            let after = draw::run(
                &type_scale::LABEL,
                format_args!("ACTUAL"),
                Point::new(left, baseline),
                VerticalPosition::Baseline,
                palette::INK_MUTED,
                target,
            );
            let after = draw::run(
                &type_scale::SECONDARY_21,
                format_args!("{measured:.2}"),
                Point::new(after + rhythm::GAP, baseline),
                VerticalPosition::Baseline,
                scale.pen,
                target,
            );
            let after = draw::run(
                &type_scale::LABEL,
                format_args!("{}", scale.unit),
                Point::new(after + rhythm::TIGHT, baseline),
                VerticalPosition::Baseline,
                palette::INK_FAINT,
                target,
            );
            rhythm::divider(after, baseline, &type_scale::SECONDARY_21, target)?
        }
        None => left,
    };

    // Same pen, smaller and muted. The command is the same quantity as the measurement, so
    // colour cannot be what tells them apart -- and the operator set it, so it is the one of
    // the two they already know.
    let after = draw::run(
        &type_scale::LABEL,
        format_args!("COMMAND"),
        Point::new(after, baseline),
        VerticalPosition::Baseline,
        palette::INK_MUTED,
        target,
    );
    let after = draw::run(
        &type_scale::NUMBER_FLOOR,
        format_args!("{:.*}", scale.decimals, commanded),
        Point::new(after + rhythm::GAP, baseline),
        VerticalPosition::Baseline,
        palette::dim(scale.pen),
        target,
    );
    draw::run(
        &type_scale::LABEL,
        format_args!("{}", scale.unit),
        Point::new(after + rhythm::TIGHT, baseline),
        VerticalPosition::Baseline,
        palette::INK_FAINT,
        target,
    );

    Ok(())
}

fn rail<D>(
    view: &FreeBrewView,
    scale: &Scale,
    commanded: f32,
    w: Window,
    top: i32,
    target: &mut D,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let left = w.at(0, 0).x;
    let area = Rectangle::new(
        Point::new(left, w.at(0, top).y),
        Size::new((w.body_right() - left) as u32, widgets::RAIL_HEIGHT),
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

fn bottom_row<D>(
    view: &FreeBrewView,
    w: Window,
    label_baseline: i32,
    value_baseline: i32,
    target: &mut D,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    // Four cells. Which four depends on the mode: under duty control there is no downstream
    // setpoint, so pressure and flow -- the two consequences of a duty -- take equal weight
    // and the running total of water in gives up its cell.
    // One decimal on the flow, where the row above gives the commanded flow two. Under duty
    // control flow is a consequence rather than the thing being aimed at, and `ML/S` is the
    // widest unit on the panel: at two decimals this cell is the fifteen pixels that push the
    // row into the status marks, and a hundredth of a millilitre per second is not a figure
    // anybody reads off a summary row.
    let fourth: (&str, Option<f32>, usize, &str, Rgb565) = match view.command {
        Command::Duty { .. } => ("FLOW", view.flow_in_ml_s, 1, "ML/S", palette::PEN_FLOW_OUT),
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

    // Laid out left to right from what each cell actually measures, rather than on a grid of
    // four equal columns: a grid cell whose content is one pixel wider than its share puts
    // `BAR` through the figure beside it, which four equal columns of 92 px did.
    //
    // Divided by rules. Six pixels of black between `8.39 BAR` and `IN` is not a division at
    // 1x -- the four cells read as one run of text, which is the same fault the review found
    // in the post-routine stats line and the same fix.
    let mut x = w.at(0, 0).x;
    for (i, (label, value, decimals, unit, pen)) in cells.iter().enumerate() {
        if i > 0 {
            x = rhythm::divider(x, value_baseline, &type_scale::SECONDARY_19, target)?;
        }
        // A cell is as wide as its widest row, which is not always the figure: with no scale
        // paired `WEIGHT` is 56 px of label over an 8 px dash, and advancing by the dash put
        // `PRESSURE` through the middle of it.
        let label_end = draw::run(
            &type_scale::LABEL,
            format_args!("{label}"),
            Point::new(x, label_baseline),
            VerticalPosition::Baseline,
            palette::INK_MUTED,
            target,
        );
        let after = widgets::value(
            &type_scale::SECONDARY_19,
            *value,
            *decimals,
            Point::new(x, value_baseline),
            VerticalPosition::Baseline,
            *pen,
            target,
        )?;
        let value_end = draw::run(
            &type_scale::LABEL,
            format_args!("{unit}"),
            Point::new(after + rhythm::TIGHT, value_baseline),
            VerticalPosition::Baseline,
            palette::INK_FAINT,
            target,
        );
        x = label_end.max(value_end);
    }

    Ok(())
}
