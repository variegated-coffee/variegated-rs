//! Section 6.4: routine execution.
//!
//! Two questions during a routine: where am I, and what ends this step. The left spine
//! answers the first -- completed steps struck through, the current one on a tinted band
//! with a marker rule. The footer answers the second, naming the exit condition in words and
//! showing progress toward it.

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};
use u8g2_fonts::types::{HorizontalAlignment, VerticalPosition};

use crate::draw;
use crate::geometry::{PAD_BOTTOM, PAD_LEFT, PAD_TOP, at, body_right, hairline_v};
use crate::palette;
use crate::type_scale;
use crate::view::{ExitView, RoutineView};
use crate::widgets;

const BOTTOM: i32 = crate::geometry::WINDOW_SIZE.height as i32 - PAD_BOTTOM;

/// The spine's width.
///
/// The figure gives 122. 132 is what `Declining profile` needs beside its number in the
/// panel's own bold 10 px face -- at 122 the longest step name runs out past the band drawn
/// behind it, which reads as the highlight being the wrong size rather than as the text
/// being too long.
const SPINE_WIDTH: i32 = 132;
const SPINE_DIVIDER_DX: i32 = PAD_LEFT + SPINE_WIDTH + 10;
const RIGHT_DX: i32 = SPINE_DIVIDER_DX + 10;

/// Four step rows fit. Beyond that the window scrolls around the current step.
const VISIBLE_STEPS: usize = 4;
const STEP_PITCH: i32 = 15;
const FIRST_STEP_DY: i32 = 22;

/// The second column of the values block: weight, beside time in step.
const VALUES_SECOND_DX: i32 = RIGHT_DX + 112;

/// Water in, on the row under the two big figures. Further right than
/// [`VALUES_SECOND_DX`] because the pressure beside it carries its target as well, which is
/// the longest run on the panel's right half.
const WATER_DX: i32 = RIGHT_DX + 128;

/// Baseline of the two big figures. `inb24` inks 24 px above it.
const VALUES_BASELINE: i32 = 44;

/// Baseline of the pressure-and-water row. `inb16` inks 16 px above it.
const SECOND_ROW_BASELINE: i32 = 66;

pub(crate) fn draw<D>(view: &RoutineView<'_>, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    spine(view, target)?;
    hairline_v(
        at(SPINE_DIVIDER_DX, PAD_TOP),
        (BOTTOM - PAD_TOP) as u32,
        target,
    )?;
    values(view, target)?;
    exit_footer(view, target)?;
    Ok(())
}

/// Which step the top row shows.
///
/// Kept one row above the current step where there is one, so the step just finished stays
/// on screen: the question the spine answers is "where am I", and a window whose top row is
/// always the current step answers "what is next" instead.
fn window_start(current: usize, count: usize) -> usize {
    let last_start = count.saturating_sub(VISIBLE_STEPS);
    current.saturating_sub(1).min(last_start)
}

fn spine<D>(view: &RoutineView<'_>, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    draw::run(
        &type_scale::CHIP,
        format_args!("{}", view.name),
        at(PAD_LEFT, PAD_TOP),
        VerticalPosition::Top,
        palette::INK_MUTED,
        target,
    );

    let start = window_start(view.current_step, view.steps.len());
    for (row, index) in (start..view.steps.len()).take(VISIBLE_STEPS).enumerate() {
        let dy = FIRST_STEP_DY + row as i32 * STEP_PITCH;
        let top = at(PAD_LEFT, dy);
        let step = &view.steps[index];

        if index == view.current_step {
            // A tinted band and a 2 px marker rule, both bled 4 px left of the text so the
            // band reads as a region rather than as a highlight on the words.
            Rectangle::new(
                Point::new(top.x - 4, top.y - 3),
                Size::new(SPINE_WIDTH as u32 + 4, 14),
            )
            .into_styled(PrimitiveStyle::with_fill(palette::TROUGH))
            .draw(target)?;
            Rectangle::new(Point::new(top.x - 4, top.y - 3), Size::new(2, 14))
                .into_styled(PrimitiveStyle::with_fill(palette::INK))
                .draw(target)?;
        }

        let (number_ink, text_ink, face) = if index == view.current_step {
            (palette::INK, palette::INK, &type_scale::STEP_CURRENT)
        } else if index < view.current_step {
            (palette::OK, palette::INK_FAINT, &type_scale::STEP_OTHER)
        } else {
            (palette::INK_FAINT, palette::INK_MUTED, &type_scale::STEP_OTHER)
        };

        draw::run(
            &type_scale::LABEL,
            format_args!("{}", index + 1),
            top,
            VerticalPosition::Top,
            number_ink,
            target,
        );
        let text_left = top.x + 12;
        let text_width = draw::run(
            face,
            format_args!("{}", step.description),
            Point::new(text_left, top.y),
            VerticalPosition::Top,
            text_ink,
            target,
        ) - text_left;

        // Struck through, not just dimmed: a step behind you is done, and dimming alone is
        // the same signal this panel uses for a step that has not started.
        if index < view.current_step {
            Rectangle::new(
                Point::new(text_left, top.y + 5),
                Size::new(text_width.max(0) as u32, 1),
            )
            .into_styled(PrimitiveStyle::with_fill(palette::INK_FAINT))
            .draw(target)?;
        }
    }

    if let Some(total) = view.total_elapsed_s {
        draw::run(
            &type_scale::LABEL,
            format_args!("{total:.1} s TOTAL"),
            at(PAD_LEFT, BOTTOM),
            VerticalPosition::Bottom,
            palette::INK_FAINT,
            target,
        );
    }

    Ok(())
}

fn values<D>(view: &RoutineView<'_>, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    for (dx, label, value, pen) in [
        (RIGHT_DX, "IN STEP", view.step_elapsed_s, palette::INK),
        (VALUES_SECOND_DX, "WEIGHT", view.weight_g, palette::PEN_WEIGHT),
    ] {
        draw::run(
            &type_scale::LABEL,
            format_args!("{label}"),
            at(dx, PAD_TOP),
            VerticalPosition::Top,
            palette::INK_MUTED,
            target,
        );
        let baseline = at(dx, VALUES_BASELINE);
        let after = widgets::value(
            &type_scale::PRIMARY_30,
            value,
            1,
            baseline,
            VerticalPosition::Baseline,
            pen,
            target,
        )?;
        draw::run(
            &type_scale::UNIT_12,
            format_args!("{}", if label == "WEIGHT" { "g" } else { "s" }),
            Point::new(after + 3, baseline.y),
            VerticalPosition::Baseline,
            palette::INK_MUTED,
            target,
        );
    }

    // Pressure against what it was asked for, and water in. Both at the number floor: they
    // are context for the two figures above, not the figures themselves.
    let baseline = at(RIGHT_DX, SECOND_ROW_BASELINE);
    let after = widgets::value(
        &type_scale::NUMBER_FLOOR,
        view.pressure_bar,
        2,
        baseline,
        VerticalPosition::Baseline,
        palette::PEN_PRESSURE,
        target,
    )?;
    if let Some(target_bar) = view.pressure_target {
        draw::run(
            &type_scale::LABEL,
            format_args!("bar / {target_bar:.1}"),
            Point::new(after + 4, baseline.y),
            VerticalPosition::Baseline,
            palette::INK_FAINT,
            target,
        );
    } else {
        draw::run(
            &type_scale::LABEL,
            format_args!("bar"),
            Point::new(after + 4, baseline.y),
            VerticalPosition::Baseline,
            palette::INK_FAINT,
            target,
        );
    }

    let baseline = at(WATER_DX, SECOND_ROW_BASELINE);
    let after = widgets::value(
        &type_scale::NUMBER_FLOOR,
        view.water_in_ml,
        0,
        baseline,
        VerticalPosition::Baseline,
        palette::PEN_WATER_IN,
        target,
    )?;
    draw::run(
        &type_scale::LABEL,
        format_args!("mL in"),
        Point::new(after + 4, baseline.y),
        VerticalPosition::Baseline,
        palette::INK_FAINT,
        target,
    );

    Ok(())
}

fn exit_footer<D>(view: &RoutineView<'_>, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let left = at(RIGHT_DX, 0).x;
    let right = body_right();

    match view.exit {
        ExitView::Phrase(phrase) => {
            // No bar. A condition with nothing to measure would otherwise get a bar that
            // reads empty for the whole step, which says "no progress" rather than "not a
            // thing that has progress".
            draw::run(
                &type_scale::LABEL,
                format_args!("{phrase}"),
                at(RIGHT_DX, BOTTOM),
                VerticalPosition::Bottom,
                palette::INK_MUTED,
                target,
            );
        }
        ExitView::Progress {
            phrase,
            current,
            target: threshold,
            quantity,
        } => {
            draw::run(
                &type_scale::LABEL,
                format_args!("{phrase}"),
                at(RIGHT_DX, 88),
                VerticalPosition::Top,
                palette::INK_MUTED,
                target,
            );
            let decimals = quantity.decimals();
            match current {
                Some(value) => draw::aligned(
                    &type_scale::LABEL,
                    format_args!("{:.*} / {:.*}", decimals, value, decimals, threshold),
                    Point::new(right, at(0, 88).y),
                    VerticalPosition::Top,
                    HorizontalAlignment::Right,
                    palette::INK,
                    target,
                ),
                None => draw::aligned(
                    &type_scale::LABEL,
                    format_args!("/ {:.*}", decimals, threshold),
                    Point::new(right, at(0, 88).y),
                    VerticalPosition::Top,
                    HorizontalAlignment::Right,
                    palette::INK_FAINT,
                    target,
                ),
            }

            // A quantity nothing is reporting draws an empty bar rather than a full one:
            // the alternative reading of "no current value" is "we are there", and that is
            // the reading that would have someone lift a cup off a scale mid-shot.
            let fraction = match (current, threshold) {
                (Some(value), t) if t > 0.0 => value / t,
                _ => 0.0,
            };
            widgets::progress(
                Rectangle::new(
                    at(RIGHT_DX, 100),
                    Size::new((right - left) as u32, 5),
                ),
                palette::pen(quantity),
                fraction,
                target,
            )?;
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::window_start;

    /// Four or fewer steps never scroll.
    #[test]
    fn a_short_routine_starts_at_the_top() {
        for current in 0..4 {
            assert_eq!(window_start(current, 4), 0);
        }
    }

    /// The step just finished stays on screen while there is room for it.
    #[test]
    fn the_window_keeps_one_step_of_history() {
        assert_eq!(window_start(0, 8), 0);
        assert_eq!(window_start(1, 8), 0);
        assert_eq!(window_start(2, 8), 1);
        assert_eq!(window_start(5, 8), 4);
    }

    /// The last step must not scroll past the end and leave blank rows under it.
    #[test]
    fn the_window_stops_at_the_last_full_page() {
        assert_eq!(window_start(7, 8), 4);
        assert_eq!(window_start(6, 8), 4);
    }
}
