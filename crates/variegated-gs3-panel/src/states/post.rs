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
use crate::geometry::{GUTTER, WINDOW_SIZE, Window};
use crate::palette;
use crate::rhythm::{self, Stack};
use crate::type_scale;
use crate::view::{Outcome, PostView};
use crate::widgets;

/// The left column's width. Holds `1:2.5` beside `FROM 20.0 G` at the floor.
///
/// Six wider than it was, which is where this state's share of the resize went. The curve
/// keeps the 185 px it had: the window grew by the same six.
const LEFT_WIDTH: i32 = 156;
const RIGHT_DX: i32 = GUTTER + LEFT_WIDTH + rhythm::RULE_WIDTH;

/// Where the first row's ink starts, from the window top.
const TOP: i32 = rhythm::MARGIN;

/// Mid-gap below the shot time. See [`crate::states::overlay_floor`].
pub(crate) const OVERLAY_FLOOR: i32 = TOP
    + type_scale::STATE_WORD.height()
    + rhythm::PITCH
    + type_scale::PRIMARY_30.height()
    + rhythm::PITCH / 2;

/// A ratio below this is not a ratio.
///
/// A shot stopped at a fifth of its dose is arithmetically `1:0.2` and informationally empty,
/// and printing it invites it to be read as a real result. One to one is where a number stops
/// describing an extraction and starts describing an interruption -- below it the row states
/// the dash it already has for the no-scale case, which says *there is no figure here* rather
/// than offering a bad one.
const RATIO_FLOOR: f32 = 1.0;

/// The baseline of the stats footer, which spans both columns.
///
/// Anchored to the foot of the window rather than to the column above it. The left column
/// loses a row when nothing is weighing, and a footer that followed the stack up would take
/// the curve with it -- the no-scale panel would draw a shorter chart and leave thirty pixels
/// of black under it, which is a different composition rather than the same one with a fact
/// missing.
const STATS_BASELINE: i32 = WINDOW_SIZE.height as i32 - 1 - rhythm::MARGIN;

/// Where the left column's rows sit, which depends on whether there is a scale.
struct Rows {
    outcome: i32,
    time: i32,
    /// The weight row, or the one line that stands for both when nothing is weighing.
    weight: i32,
    /// The ratio row. `None` with no scale: see [`no_scale`].
    ratio: Option<i32>,
}

/// The left column's rows.
fn rows(weighed: bool) -> Rows {
    let mut stack = Stack::new(TOP);
    let outcome = stack.row(&type_scale::STATE_WORD);
    let time = stack.row(&type_scale::PRIMARY_30);
    let weight = stack.row(if weighed {
        &type_scale::PRIMARY_27
    } else {
        &type_scale::LABEL
    });
    let ratio = weighed.then(|| stack.row(&type_scale::NUMBER_FLOOR));
    Rows {
        outcome,
        time,
        weight,
        ratio,
    }
}

pub(crate) fn draw<D>(view: &PostView<'_>, w: Window, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    // One stack for both columns: the curve, the trace names and the stats line all have to
    // land on the same rows as the figures beside them, and did not.
    let rows = rows(view.weight_out_g.is_some());

    left_column(view, w, &rows, target)?;
    curve(view, w, target)?;
    trace_names(view, w, target)?;
    stats(view, w, w.at(0, STATS_BASELINE).y, target)?;
    Ok(())
}

/// How wide the curve is, given where the status strip starts.
fn curve_width(w: Window) -> i32 {
    w.body_right() - w.at(RIGHT_DX, 0).x
}

/// The curve's box, and the baseline of the names under it.
///
/// The curve is a block rather than a row, but it shares the panel's rhythm like everything
/// else -- and it is the one element on this panel sized by what is left rather than by what
/// it wants. It starts where the outcome row ends, and it stops far enough above the stats
/// footer for the trace names to sit between them, which is what stopped the names row and the
/// footer from being drawn through each other.
fn curve_rows() -> (i32, i32, i32) {
    let stats_top = STATS_BASELINE - type_scale::LABEL.ascent();

    let mut right = Stack::new(TOP);
    right.row(&type_scale::STATE_WORD);
    let top = right.bottom() + rhythm::PITCH;
    // Names row and footer, measured back from the bottom.
    let names = stats_top - rhythm::PITCH;
    let height = names - type_scale::LABEL.ascent() - rhythm::PITCH - top;
    (top, height, names)
}

fn left_column<D>(
    view: &PostView<'_>,
    w: Window,
    rows: &Rows,
    target: &mut D,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let outcome = rows.outcome;
    let time = rows.time;
    let (word, colour) = match view.outcome {
        Outcome::Complete => ("COMPLETE", palette::OK),
        Outcome::Aborted => ("ABORTED", palette::DANGER),
    };
    let baseline = w.at(0, outcome).y;
    w.marker(baseline, type_scale::STATE_WORD.ascent(), colour, target)?;
    // Ink, with the square carrying the hue. See the note in `idle`.
    draw::run(
        &type_scale::STATE_WORD,
        format_args!("{word}"),
        Point::new(w.text_left(), baseline),
        VerticalPosition::Baseline,
        palette::INK,
        target,
    );

    // Ink, not a pen: a shot time is not a sensed quantity. On the machine blue had become
    // the default colour for a number, which is exactly what section 4 asks colour not to do.
    let baseline = w.at(0, time);
    let baseline = Point::new(w.text_left(), baseline.y);
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
        Point::new(after + rhythm::TIGHT, baseline.y),
        VerticalPosition::Baseline,
        palette::INK_FAINT,
        target,
    );

    // Nothing weighing means no weight out *and* no ratio, and the two absences were being
    // reported separately -- `-- G OUT` over `-- RATIO`, two placeholder rows and 27 px of
    // window spent saying the same nothing twice. One line says it once, and says *why*, which
    // neither dash did.
    let Some(weight_out) = view.weight_out_g else {
        return no_scale(w, w.at(0, rows.weight).y, target);
    };

    let baseline = Point::new(w.text_left(), w.at(0, rows.weight).y);
    let after = draw::run(
        &type_scale::PRIMARY_27,
        format_args!("{weight_out:.1}"),
        baseline,
        VerticalPosition::Baseline,
        palette::PEN_WEIGHT,
        target,
    );
    draw::run(
        &type_scale::LABEL,
        format_args!("G OUT"),
        Point::new(after + rhythm::TIGHT, baseline.y),
        VerticalPosition::Baseline,
        palette::INK_FAINT,
        target,
    );

    // The ratio needs both ends, and needs them to mean something. Below `RATIO_FLOOR` there
    // is a number but no result.
    let Some(ratio) = rows.ratio else {
        return Ok(());
    };
    let baseline = Point::new(w.text_left(), w.at(0, ratio).y);
    match (view.weight_out_g, view.dose_g) {
        (Some(out), Some(dose)) if dose > 0.0 && out / dose >= RATIO_FLOOR => {
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
                Point::new(after + rhythm::GAP, baseline.y),
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
                Point::new(after + rhythm::GAP, baseline.y),
                VerticalPosition::Baseline,
                palette::INK_FAINT,
                target,
            );
        }
    }

    Ok(())
}

/// One line where the weight and the ratio would have been.
///
/// Muted rather than faint: this is a statement about the machine's configuration, not a
/// missing reading, and it is the reason two of this panel's four figures are not here. The
/// status strip says the same thing in a red scale mark; this says what it means for the shot
/// that just finished.
fn no_scale<D>(w: Window, baseline: i32, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    draw::run(
        &type_scale::LABEL,
        format_args!("NO SCALE PAIRED"),
        Point::new(w.text_left(), baseline),
        VerticalPosition::Baseline,
        palette::INK_MUTED,
        target,
    );
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
    let (top, height, _) = curve_rows();
    let origin = w.at(RIGHT_DX, top);
    let width = curve_width(w);

    // The baseline is drawn whether or not there is a trace: it is what makes the box read as
    // a chart rather than as two floating lines.
    Rectangle::new(
        Point::new(origin.x, origin.y + height - 1),
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
        while y < origin.y + height {
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
        height,
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
            height,
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
#[allow(clippy::too_many_arguments)]
fn polyline<D>(
    origin: Point,
    width: i32,
    height: i32,
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
        origin.y + height - 2 - (fraction * (height - 3) as f32) as i32
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
    let baseline = w.at(RIGHT_DX, curve_rows().2);
    let after = draw::run(
        &type_scale::LABEL,
        format_args!("PRESSURE"),
        baseline,
        VerticalPosition::Baseline,
        palette::PEN_PRESSURE,
        target,
    );
    if view.weight_out_g.is_some() {
        let after = rhythm::divider(after, baseline.y, &type_scale::LABEL, target)?;
        draw::run(
            &type_scale::LABEL,
            format_args!("WEIGHT"),
            Point::new(after, baseline.y),
            VerticalPosition::Baseline,
            palette::PEN_WEIGHT,
            target,
        );
    }

    Ok(())
}

/// The numbers the legend used to bury, stated as numbers.
///
/// Four facts, and the review's clearest example of what a 6 px pitch costs horizontally:
/// `DROP 8.9 S PEAK 8.7 BAR 91 ML IN LEVER-LIKE` arrived as one run of text, because the ten
/// pixels that used to divide the clauses are now wider than the panel's own row pitch. A rule
/// with 6 px either side divides them in thirteen pixels where distance needed twenty-two, and
/// divides them better.
fn stats<D>(view: &PostView<'_>, w: Window, baseline: i32, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    // From the window's edge, not the text column. The marker channel belongs to the stack
    // the state word heads -- the four figures down the left -- and this row is not in it: it
    // runs under both columns, like free-brewing's cell row and routine's exit line, neither
    // of which has a gutter either. It is also the fourteen pixels that decide whether the
    // routine's name fits beside the three numbers.
    let mut x = w.at(0, 0).x;
    let mut first = true;
    let right = w.body_right();
    // Measured before it is drawn, and dropped from the end when it will not fit.
    //
    // The routine's name is the operator's, so this row has no bounded width: `LEVER-LIKE`
    // takes 80 px and the next routine somebody writes may take twice that. A clause that
    // does not fit is dropped whole rather than drawn into the status marks -- the provenance
    // is the least of the four facts here, and it is the one that runs last.
    let clause = |args: core::fmt::Arguments<'_>,
                      x: &mut i32,
                      first: &mut bool,
                      target: &mut D|
     -> Result<(), D::Error> {
        let lead = if *first { 0 } else { rhythm::RULE_WIDTH };
        if *x + lead + draw::width(&type_scale::LABEL, args) > right {
            return Ok(());
        }
        if !*first {
            *x = rhythm::divider(*x, baseline, &type_scale::LABEL, target)?;
        }
        *first = false;
        *x = draw::run(
            &type_scale::LABEL,
            args,
            Point::new(*x, baseline),
            VerticalPosition::Baseline,
            palette::INK_FAINT,
            target,
        );
        Ok(())
    };

    if let Some(first_drop) = view.trace.and_then(|t| t.first_drop_seconds()) {
        clause(
            format_args!("DROP {first_drop:.1} S"),
            &mut x,
            &mut first,
            target,
        )?;
    }
    if let Some(peak) = view.trace.and_then(|t| t.peak_pressure()) {
        clause(
            format_args!("PEAK {peak:.1} BAR"),
            &mut x,
            &mut first,
            target,
        )?;
    }
    if let Some(water) = view.water_in_ml {
        clause(format_args!("{water:.0} ML IN"), &mut x, &mut first, target)?;
    }
    if let Some(routine) = view.routine {
        clause(format_args!("{routine}"), &mut x, &mut first, target)?;
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
