//! Section 6.5: post-routine.
//!
//! Confirmation on the left, explanation on the right. Three figures settle whether the shot
//! was what was asked for; the curve says why, with the phase bands shaded behind it and
//! first drop marked. Both pens are labelled -- the traces are never left to colour.

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, Line, PrimitiveStyle, Rectangle};
use u8g2_fonts::types::VerticalPosition;

use crate::draw;
use crate::geometry::{PAD_BOTTOM, PAD_LEFT, PAD_TOP, at, hairline_v};
use crate::palette;
use crate::type_scale;
use crate::view::{Outcome, PostView};
use crate::widgets;

const BOTTOM: i32 = crate::geometry::WINDOW_SIZE.height as i32 - PAD_BOTTOM;

/// The left column's width.
///
/// The figure gives 118. 130 is what `1:2.5` plus `from 19.9 g` needs in the panel's own
/// faces, and the right half still has 196 px for a 190 px curve.
const LEFT_WIDTH: i32 = 130;
const DIVIDER_DX: i32 = PAD_LEFT + LEFT_WIDTH + 10;
const RIGHT_DX: i32 = DIVIDER_DX + 10;

/// Baseline of the shot time. `inb21` inks 21 px above it.
const TIME_BASELINE: i32 = 46;

/// Baseline of the weight out. `inb16` inks 16 px above it.
const WEIGHT_BASELINE: i32 = 68;

/// Baseline of the ratio.
const RATIO_BASELINE: i32 = 88;

/// The curve, at the size section 8 gives it beside a figure block.
const CURVE_WIDTH: i32 = 190;
const CURVE_HEIGHT: i32 = 56;
const CURVE_DY: i32 = PAD_TOP;

const LEGEND_DY: i32 = CURVE_DY + CURVE_HEIGHT + 8;

pub(crate) fn draw<D>(view: &PostView<'_>, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    left_column(view, target)?;
    hairline_v(at(DIVIDER_DX, PAD_TOP), (BOTTOM - PAD_TOP) as u32, target)?;
    curve(view, target)?;
    legend(view, target)?;
    Ok(())
}

fn left_column<D>(view: &PostView<'_>, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let (word, colour) = match view.outcome {
        Outcome::Complete => ("COMPLETE", palette::OK),
        Outcome::Aborted => ("ABORTED", palette::DANGER),
    };
    Circle::new(at(PAD_LEFT, PAD_TOP + 2), 6)
        .into_styled(PrimitiveStyle::with_fill(colour))
        .draw(target)?;
    draw::run(
        &type_scale::STATE_WORD,
        format_args!("{word}"),
        at(PAD_LEFT + 12, PAD_TOP),
        VerticalPosition::Top,
        colour,
        target,
    );

    let baseline = at(PAD_LEFT, TIME_BASELINE);
    let after = draw::run(
        &type_scale::PRIMARY_27,
        format_args!("{:.1}", view.shot_seconds),
        baseline,
        VerticalPosition::Baseline,
        palette::INK,
        target,
    );
    draw::run(
        &type_scale::UNIT_12,
        format_args!("s"),
        Point::new(after + 4, baseline.y),
        VerticalPosition::Baseline,
        palette::INK_MUTED,
        target,
    );

    let baseline = at(PAD_LEFT, WEIGHT_BASELINE);
    let after = widgets::value(
        &type_scale::NUMBER_FLOOR,
        view.weight_out_g,
        1,
        baseline,
        VerticalPosition::Baseline,
        palette::PEN_WEIGHT,
        target,
    )?;
    draw::run(
        &type_scale::LABEL,
        format_args!("g out"),
        Point::new(after + 4, baseline.y),
        VerticalPosition::Baseline,
        palette::INK_FAINT,
        target,
    );

    // The ratio needs both ends. With no scale paired there is no weight out and therefore
    // no ratio, and the row states the dose alone rather than inventing one.
    let baseline = at(PAD_LEFT, RATIO_BASELINE);
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
                format_args!("from {dose:.1} g"),
                Point::new(after + 4, baseline.y),
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
                format_args!("ratio"),
                Point::new(after + 4, baseline.y),
                VerticalPosition::Baseline,
                palette::INK_FAINT,
                target,
            );
        }
    }

    // Provenance: which routine, and how much water went in to produce the figures above.
    let baseline = at(PAD_LEFT, BOTTOM);
    let mut x = baseline.x;
    if let Some(routine) = view.routine {
        x = draw::run(
            &type_scale::LABEL,
            format_args!("{routine}"),
            baseline,
            VerticalPosition::Baseline,
            palette::INK_FAINT,
            target,
        );
        x += type_scale::separator(Point::new(x, baseline.y), palette::INK_FAINT, target)? as i32;
    }
    if let Some(water) = view.water_in_ml {
        draw::run(
            &type_scale::LABEL,
            format_args!("{water:.0} mL IN"),
            Point::new(x, baseline.y),
            VerticalPosition::Baseline,
            palette::INK_FAINT,
            target,
        );
    }

    Ok(())
}

/// Map a bucket index to an x inside the curve box.
fn bucket_x(index: usize, len: usize) -> i32 {
    if len <= 1 {
        return 0;
    }
    (index as i32 * (CURVE_WIDTH - 1)) / (len as i32 - 1)
}

/// Map a time in seconds to an x inside the curve box.
fn time_x(seconds: f32, total: f32) -> i32 {
    if total <= 0.0 {
        return 0;
    }
    ((seconds / total).clamp(0.0, 1.0) * (CURVE_WIDTH - 1) as f32) as i32
}

fn curve<D>(view: &PostView<'_>, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let Some(trace) = view.trace else {
        return Ok(());
    };
    if trace.is_empty() {
        return Ok(());
    }

    let origin = at(RIGHT_DX, CURVE_DY);
    let total = trace.elapsed_seconds();

    // Phase bands behind everything. They are the answer to "why did it do that", and a
    // curve drawn over them still reads because they are three near-black tints rather than
    // three colours.
    let saturation_x = trace.saturation_seconds().map(|s| time_x(s, total));
    let first_drop_x = trace.first_drop_seconds().map(|s| time_x(s, total));
    let mut band_start = 0;
    for (end, fill) in [
        (saturation_x, palette::PHASE_HEADSPACE),
        (first_drop_x, palette::PHASE_SATURATION),
        (Some(CURVE_WIDTH), palette::PHASE_POST_FIRST_DROP),
    ] {
        let Some(end) = end else { continue };
        if end > band_start {
            Rectangle::new(
                Point::new(origin.x + band_start, origin.y),
                Size::new((end - band_start) as u32, CURVE_HEIGHT as u32),
            )
            .into_styled(PrimitiveStyle::with_fill(fill))
            .draw(target)?;
        }
        band_start = end;
    }

    // Two pens, two independent scales. There is no shared axis to put on a 190x56 box, and
    // the question each trace answers -- did the pressure do what was asked, did the weight
    // arrive smoothly -- is about its own shape rather than about the other's.
    let peak = trace.peak_pressure().unwrap_or(1.0).max(1.0);
    polyline(origin, trace.pressure(), peak, palette::PEN_PRESSURE, target)?;
    if view.weight_out_g.is_some() {
        let final_weight = trace
            .weight()
            .last()
            .copied()
            .unwrap_or(1.0)
            .max(1.0);
        polyline(origin, trace.weight(), final_weight, palette::PEN_WEIGHT, target)?;
    }

    // First drop, as a dashed rule. Dashed rather than solid so it reads as an annotation on
    // the chart rather than as a third trace.
    if let Some(x) = first_drop_x {
        let mut y = origin.y;
        while y < origin.y + CURVE_HEIGHT {
            Rectangle::new(Point::new(origin.x + x, y), Size::new(1, 2))
                .into_styled(PrimitiveStyle::with_fill(palette::INK_MUTED))
                .draw(target)?;
            y += 4;
        }
    }

    Ok(())
}

fn polyline<D>(
    origin: Point,
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
        origin.y + CURVE_HEIGHT - 1 - (fraction * (CURVE_HEIGHT - 1) as f32) as i32
    };
    let mut previous: Option<Point> = None;
    for (i, value) in samples.iter().enumerate() {
        let point = Point::new(origin.x + bucket_x(i, samples.len()), y_of(*value));
        if let Some(from) = previous {
            Line::new(from, point).into_styled(style).draw(target)?;
        }
        previous = Some(point);
    }
    Ok(())
}

fn legend<D>(view: &PostView<'_>, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    // A pen is never identified by colour alone: every swatch carries its word, and the
    // pressure pen carries its peak because that is the figure the curve is read for.
    let mut x = at(RIGHT_DX, 0).x;
    let y = at(0, LEGEND_DY).y;

    Rectangle::new(Point::new(x, y + 3), Size::new(7, 2))
        .into_styled(PrimitiveStyle::with_fill(palette::PEN_PRESSURE))
        .draw(target)?;
    x += 11;
    x = match view.trace.and_then(|t| t.peak_pressure()) {
        Some(peak) => draw::run(
            &type_scale::LABEL,
            format_args!("PRESSURE {peak:.1} PEAK"),
            Point::new(x, y),
            VerticalPosition::Top,
            palette::INK_FAINT,
            target,
        ),
        None => draw::run(
            &type_scale::LABEL,
            format_args!("PRESSURE"),
            Point::new(x, y),
            VerticalPosition::Top,
            palette::INK_FAINT,
            target,
        ),
    };

    if view.weight_out_g.is_some() {
        x += 9;
        Rectangle::new(Point::new(x, y + 3), Size::new(7, 2))
            .into_styled(PrimitiveStyle::with_fill(palette::PEN_WEIGHT))
            .draw(target)?;
        draw::run(
            &type_scale::LABEL,
            format_args!("WEIGHT"),
            Point::new(x + 11, y),
            VerticalPosition::Top,
            palette::INK_FAINT,
            target,
        );
    }

    if let Some(first_drop) = view.trace.and_then(|t| t.first_drop_seconds()) {
        draw::run(
            &type_scale::LABEL,
            format_args!("FIRST DROP {first_drop:.1} s"),
            at(RIGHT_DX, BOTTOM),
            VerticalPosition::Bottom,
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
        for len in [2usize, 47, 95, 190] {
            assert_eq!(bucket_x(0, len), 0);
            assert_eq!(bucket_x(len - 1, len), CURVE_WIDTH - 1);
        }
    }

    /// A one-sample trace has no span to divide by, and must not panic or draw off the left
    /// edge.
    #[test]
    fn a_single_sample_sits_at_the_origin() {
        assert_eq!(bucket_x(0, 1), 0);
    }

    /// A time past the end of the shot clamps rather than running off the chart, which a
    /// first-drop timestamp latched at the very last sample would otherwise do.
    #[test]
    fn a_marker_at_the_end_stays_on_the_chart() {
        assert_eq!(time_x(60.0, 51.1), CURVE_WIDTH - 1);
        assert_eq!(time_x(0.0, 51.1), 0);
        assert_eq!(time_x(5.0, 0.0), 0);
    }
}
