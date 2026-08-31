//! Section 6.5, as recomposed after the field review.
//!
//! Confirmation on the left, explanation on the right. The three figures worked on the
//! machine and are unchanged in kind. The curve did not, and it took a quarter of the window
//! to fail:
//!
//! * **A one-pixel trace in a darkened pen is not a line on this panel.** Both traces were
//!   present in the photograph and neither could be followed. Two pixels now, at full pen
//!   luminance, over a baseline.
//! * **The legend cost three 8 px lines to name two traces**, and buried the two numbers
//!   worth having -- the peak and the first drop -- inside label text. Each trace is now
//!   named in its own pen, once, and those two numbers are stated as numbers.
//!
//! Two traces do not need a key. Naming a trace at the trace is cheaper than naming it in
//! one, and that is what paid for the raised type floor here.

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Line, PrimitiveStyle, Rectangle};
use u8g2_fonts::types::VerticalPosition;

use crate::draw;
use crate::geometry::Window;
use crate::palette;
use crate::type_scale;
use crate::view::{Outcome, PostView};
use crate::widgets;

/// The left column's width. Holds `1:2.5` beside `FROM 20.0 G` at the floor.
const LEFT_WIDTH: i32 = 150;
const RIGHT_DX: i32 = LEFT_WIDTH + 10;

/// Baselines down the left column: outcome, time, weight out, ratio.
const OUTCOME_BASELINE: i32 = 12;
const TIME_BASELINE: i32 = 40;
const WEIGHT_BASELINE: i32 = 64;
const RATIO_BASELINE: i32 = 86;

/// The dot beside the outcome word.
const OUTCOME_DOT: u32 = 7;

/// The curve. Its width is whatever is left between the column and the status strip.
const CURVE_DY: i32 = 4;
const CURVE_HEIGHT: i32 = 64;

/// Baseline of the trace names, under the curve.
const NAMES_BASELINE: i32 = CURVE_DY + CURVE_HEIGHT + 12;

/// Baseline of the statistics line, at the foot.
///
/// It runs the **full width** rather than sitting in the right column as the figure draws
/// it. Three stated numbers at the raised floor need about 180 px and the right column has
/// 202, which sounds like enough until the routine's name joins them; the left column's foot
/// is empty below the ratio, so the footer takes both. It also puts the provenance back,
/// which the column version had to drop.
const STATS_BASELINE: i32 = 110;

pub(crate) fn draw<D>(view: &PostView<'_>, w: Window, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    left_column(view, w, target)?;
    curve(view, w, target)?;
    trace_names(view, w, target)?;
    stats(view, w, target)?;
    Ok(())
}

/// How wide the curve is, given where the status strip starts.
fn curve_width(w: Window) -> i32 {
    w.body_right() - w.at(RIGHT_DX, 0).x
}

fn left_column<D>(view: &PostView<'_>, w: Window, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let (word, colour) = match view.outcome {
        Outcome::Complete => ("COMPLETE", palette::OK),
        Outcome::Aborted => ("ABORTED", palette::DANGER),
    };
    Rectangle::new(
        w.at(0, OUTCOME_BASELINE - OUTCOME_DOT as i32 - 2),
        Size::new(OUTCOME_DOT, OUTCOME_DOT),
    )
    .into_styled(PrimitiveStyle::with_fill(colour))
    .draw(target)?;
    draw::run(
        &type_scale::STATE_WORD,
        format_args!("{word}"),
        w.at(OUTCOME_DOT as i32 + 6, OUTCOME_BASELINE),
        VerticalPosition::Baseline,
        colour,
        target,
    );

    // Ink, not a pen: a shot time is not a sensed quantity. On the machine blue had become
    // the default colour for a number, which is exactly what section 4 asks colour not to do.
    let baseline = w.at(0, TIME_BASELINE);
    let after = draw::run(
        &type_scale::PRIMARY_30,
        format_args!("{:.1}", view.shot_seconds),
        baseline,
        VerticalPosition::Baseline,
        palette::INK,
        target,
    );
    draw::run(
        &type_scale::UNIT_12,
        format_args!("S"),
        Point::new(after + 4, baseline.y),
        VerticalPosition::Baseline,
        palette::INK_FAINT,
        target,
    );

    let baseline = w.at(0, WEIGHT_BASELINE);
    let after = widgets::value(
        &type_scale::PRIMARY_27,
        view.weight_out_g,
        1,
        baseline,
        VerticalPosition::Baseline,
        palette::PEN_WEIGHT,
        target,
    )?;
    draw::run(
        &type_scale::LABEL,
        format_args!("G OUT"),
        Point::new(after + 5, baseline.y),
        VerticalPosition::Baseline,
        palette::INK_FAINT,
        target,
    );

    // The ratio needs both ends. With no scale paired there is no weight out and therefore
    // no ratio, and the row states so rather than inventing one.
    let baseline = w.at(0, RATIO_BASELINE);
    match (view.weight_out_g, view.dose_g) {
        (Some(out), Some(dose)) if dose > 0.0 => {
            let after = draw::run(
                &type_scale::NUMBER_FLOOR,
                format_args!("1:{:.1}", out / dose),
                baseline,
                VerticalPosition::Baseline,
                palette::INK,
                target,
            );
            draw::run(
                &type_scale::LABEL,
                // No unit: the row above states `G OUT`, and the other end of a brew ratio is
                // grams by definition. At the raised floor the three characters it saves are
                // what keep the ratio inside its column.
                format_args!("FROM {dose:.1}"),
                Point::new(after + 5, baseline.y),
                VerticalPosition::Baseline,
                palette::INK_FAINT,
                target,
            );
        }
        _ => {
            let after = widgets::value(
                &type_scale::NUMBER_FLOOR,
                None,
                1,
                baseline,
                VerticalPosition::Baseline,
                palette::INK_FAINT,
                target,
            )?;
            draw::run(
                &type_scale::LABEL,
                format_args!("RATIO"),
                Point::new(after + 5, baseline.y),
                VerticalPosition::Baseline,
                palette::INK_FAINT,
                target,
            );
        }
    }

    Ok(())
}

/// Map a bucket index to an x inside the curve box.
fn bucket_x(index: usize, len: usize, width: i32) -> i32 {
    if len <= 1 {
        return 0;
    }
    (index as i32 * (width - 1)) / (len as i32 - 1)
}

/// Map a time in seconds to an x inside the curve box.
fn time_x(seconds: f32, total: f32, width: i32) -> i32 {
    if total <= 0.0 {
        return 0;
    }
    ((seconds / total).clamp(0.0, 1.0) * (width - 1) as f32) as i32
}

fn curve<D>(view: &PostView<'_>, w: Window, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let origin = w.at(RIGHT_DX, CURVE_DY);
    let width = curve_width(w);

    // The baseline is drawn whether or not there is a trace: it is what makes the box read as
    // a chart rather than as two floating lines.
    Rectangle::new(
        Point::new(origin.x, origin.y + CURVE_HEIGHT - 1),
        Size::new(width as u32, 1),
    )
    .into_styled(PrimitiveStyle::with_fill(palette::HAIRLINE))
    .draw(target)?;

    let Some(trace) = view.trace else {
        return Ok(());
    };
    if trace.is_empty() {
        return Ok(());
    }

    let total = trace.elapsed_seconds();

    // First drop, as a dashed rule. The phase bands are gone: three near-black tints behind a
    // two-pixel trace is contrast the trace needs for itself, and the one boundary worth
    // marking is the one that gets a number in the statistics line.
    if let Some(seconds) = trace.first_drop_seconds() {
        let x = origin.x + time_x(seconds, total, width);
        let mut y = origin.y;
        while y < origin.y + CURVE_HEIGHT {
            Rectangle::new(Point::new(x, y), Size::new(1, 2))
                .into_styled(PrimitiveStyle::with_fill(palette::INK_FAINT))
                .draw(target)?;
            y += 4;
        }
    }

    // Two pens, two independent scales. There is no shared axis to put on a box this size,
    // and the question each trace answers -- did the pressure hold, did the weight arrive
    // smoothly -- is about its own shape rather than about the other's.
    let peak = trace.peak_pressure().unwrap_or(1.0).max(1.0);
    polyline(
        origin,
        width,
        trace.pressure(),
        peak,
        palette::PEN_PRESSURE,
        target,
    )?;
    if view.weight_out_g.is_some() {
        let final_weight = trace.weight().last().copied().unwrap_or(1.0).max(1.0);
        polyline(
            origin,
            width,
            trace.weight(),
            final_weight,
            palette::PEN_WEIGHT,
            target,
        )?;
    }

    Ok(())
}

/// Two pixels of stroke, drawn as two one-pixel polylines a row apart.
///
/// A stroked `Polyline` would be the obvious thing and is not available at this width without
/// pulling in a thick-line rasteriser; two passes cost one extra line per segment and give a
/// trace that can actually be followed at arm's length, which one pixel could not.
fn polyline<D>(
    origin: Point,
    width: i32,
    samples: &[f32],
    full_scale: f32,
    pen: Rgb565,
    target: &mut D,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let style = PrimitiveStyle::with_stroke(pen, 1);
    let y_of = |value: f32| {
        let fraction = (value / full_scale).clamp(0.0, 1.0);
        // One row up from the baseline, so a trace at zero sits on the axis rather than
        // through it.
        origin.y + CURVE_HEIGHT - 2 - (fraction * (CURVE_HEIGHT - 3) as f32) as i32
    };
    for pass in 0..2 {
        let mut previous: Option<Point> = None;
        for (i, value) in samples.iter().enumerate() {
            let point = Point::new(
                origin.x + bucket_x(i, samples.len(), width),
                y_of(*value) + pass,
            );
            if let Some(from) = previous {
                Line::new(from, point).into_styled(style).draw(target)?;
            }
            previous = Some(point);
        }
    }
    Ok(())
}

/// Each trace named in its own pen, once. This is the legend, and it is one line.
fn trace_names<D>(view: &PostView<'_>, w: Window, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let baseline = w.at(RIGHT_DX, NAMES_BASELINE);
    let after = draw::run(
        &type_scale::LABEL,
        format_args!("PRESSURE"),
        baseline,
        VerticalPosition::Baseline,
        palette::PEN_PRESSURE,
        target,
    );
    if view.weight_out_g.is_some() {
        draw::run(
            &type_scale::LABEL,
            format_args!("WEIGHT"),
            Point::new(after + 10, baseline.y),
            VerticalPosition::Baseline,
            palette::PEN_WEIGHT,
            target,
        );
    }

    Ok(())
}

/// The numbers the legend used to bury, stated as numbers.
fn stats<D>(view: &PostView<'_>, w: Window, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let baseline = w.at(0, STATS_BASELINE);
    let mut x = baseline.x;

    if let Some(first_drop) = view.trace.and_then(|t| t.first_drop_seconds()) {
        x = draw::run(
            &type_scale::LABEL,
            format_args!("DROP {first_drop:.1} S"),
            Point::new(x, baseline.y),
            VerticalPosition::Baseline,
            palette::INK_FAINT,
            target,
        ) + 10;
    }
    if let Some(peak) = view.trace.and_then(|t| t.peak_pressure()) {
        x = draw::run(
            &type_scale::LABEL,
            format_args!("PEAK {peak:.1} BAR"),
            Point::new(x, baseline.y),
            VerticalPosition::Baseline,
            palette::INK_FAINT,
            target,
        ) + 10;
    }
    if let Some(water) = view.water_in_ml {
        x = draw::run(
            &type_scale::LABEL,
            format_args!("{water:.0} ML IN"),
            Point::new(x, baseline.y),
            VerticalPosition::Baseline,
            palette::INK_FAINT,
            target,
        ) + 10;
    }
    if let Some(routine) = view.routine {
        draw::run(
            &type_scale::LABEL,
            format_args!("{routine}"),
            Point::new(x, baseline.y),
            VerticalPosition::Baseline,
            palette::INK_FAINT,
            target,
        );
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The curve must fill its box whatever the sample count, and must never run past it.
    #[test]
    fn buckets_span_the_curve_exactly() {
        let width = curve_width(Window::DEFAULT);
        for len in [2usize, 47, 95, 190] {
            assert_eq!(bucket_x(0, len, width), 0);
            assert_eq!(bucket_x(len - 1, len, width), width - 1);
        }
    }

    /// A one-sample trace has no span to divide by, and must not panic or draw off the left
    /// edge.
    #[test]
    fn a_single_sample_sits_at_the_origin() {
        assert_eq!(bucket_x(0, 1, curve_width(Window::DEFAULT)), 0);
    }

    /// A time past the end of the shot clamps rather than running off the chart, which a
    /// first-drop timestamp latched at the very last sample would otherwise do.
    #[test]
    fn a_marker_at_the_end_stays_on_the_chart() {
        let width = curve_width(Window::DEFAULT);
        assert_eq!(time_x(60.0, 51.1, width), width - 1);
        assert_eq!(time_x(0.0, 51.1, width), 0);
        assert_eq!(time_x(5.0, 0.0, width), 0);
    }
}
