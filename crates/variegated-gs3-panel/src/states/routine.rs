//! Section 6.4, as redrawn after the field review.
//!
//! Two questions during a routine: where am I, and what ends this step. The left spine
//! answers the first, the right half the second.
//!
//! # What the machine changed
//!
//! Read at arm's length the large figures were excellent and everything at 8 px was not, so
//! the floor went up two rungs. The height came from deleting duplication rather than from
//! anywhere else, which is the useful part:
//!
//! * **The exit condition was stated three times** -- as a phrase, as a current-over-target
//!   pair, and as a bar -- and the two encodings that read at a glance were the two smallest.
//!   It is now stated once: the threshold, and the bar. The phrase survives only where there
//!   is no bar to draw.
//! * **The strike-through on a completed step destroyed it.** At 8 px the rule merged with
//!   the letterforms and the step name read as damage rather than as dim. A completed step is
//!   now marked by faint ink alone.
//! * **The routine name was stated twice** -- once as a heading and once by the running step
//!   -- so it is demoted to the top rule, where it carries the step position the spine cannot
//!   show.
//! * **The running total was never read.** It is gone.

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Rectangle;
use u8g2_fonts::types::{HorizontalAlignment, VerticalPosition};

use crate::draw;
use crate::geometry::{Window, hairline_v};
use crate::palette;
use crate::type_scale;
use crate::view::{ExitView, RoutineView};
use crate::widgets;

/// Baseline of the top rule.
const HEADER_BASELINE: i32 = 13;

/// Where the two columns start, below the rule.
const CONTENT_TOP: i32 = 18;

/// The spine's width. Holds `Declining profile` at 12 px beside its number.
const SPINE_WIDTH: i32 = 128;
const SPINE_DIVIDER_DX: i32 = SPINE_WIDTH + 10;
const RIGHT_DX: i32 = SPINE_DIVIDER_DX + 10;

/// Four step rows fit. Beyond that the window scrolls around the current step.
const VISIBLE_STEPS: usize = 4;
const STEP_PITCH: i32 = 18;

/// Baseline of the first step row.
const FIRST_STEP_BASELINE: i32 = CONTENT_TOP + 12;

/// The second column of the values block: weight, beside time in step.
const VALUES_SECOND_DX: i32 = RIGHT_DX + 104;

/// Water in, on the row under the two big figures.
///
/// Further right than the values column above it: the pressure beside it carries its target
/// as well, which is the longest run on this half of the panel.
const WATER_DX: i32 = RIGHT_DX + 138;

/// Baseline of the two big figures. `inb24` inks 24 px above it.
const VALUES_BASELINE: i32 = 52;

/// Baseline of the pressure-and-water row. `inb19` inks 19 px above it.
const SECOND_ROW_BASELINE: i32 = 76;

/// Baseline of the exit line.
const EXIT_BASELINE: i32 = 101;

/// The exit bar.
const EXIT_BAR_DY: i32 = 106;
const EXIT_BAR_HEIGHT: u32 = 5;

pub(crate) fn draw<D>(view: &RoutineView<'_>, w: Window, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    header(view, w, target);
    spine(view, w, target)?;
    hairline_v(
        w.at(SPINE_DIVIDER_DX, CONTENT_TOP),
        (crate::geometry::WINDOW_SIZE.height as i32 - CONTENT_TOP) as u32,
        target,
    )?;
    values(view, w, target)?;
    exit_footer(view, w, target)?;
    Ok(())
}

/// The top rule: where you are in the routine, and which routine it is.
///
/// The step *name* is not here -- the bold row in the spine states it once, and stating it
/// twice is what the 8 px tier was spending its height on.
fn header<D>(view: &RoutineView<'_>, w: Window, target: &mut D)
where
    D: DrawTarget<Color = Rgb565>,
{
    let after = draw::run(
        &type_scale::LABEL,
        format_args!("STEP"),
        w.at(0, HEADER_BASELINE),
        VerticalPosition::Baseline,
        palette::INK_MUTED,
        target,
    );
    draw::run(
        &type_scale::STATE_WORD,
        format_args!("{}/{}", view.current_step + 1, view.steps.len()),
        Point::new(after + 6, w.at(0, HEADER_BASELINE).y),
        VerticalPosition::Baseline,
        palette::INK,
        target,
    );
    draw::aligned(
        &type_scale::LABEL,
        format_args!("{}", view.name),
        Point::new(w.body_right(), w.at(0, HEADER_BASELINE).y),
        VerticalPosition::Baseline,
        HorizontalAlignment::Right,
        palette::INK_FAINT,
        target,
    );
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

fn spine<D>(view: &RoutineView<'_>, w: Window, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let start = window_start(view.current_step, view.steps.len());
    for (row, index) in (start..view.steps.len()).take(VISIBLE_STEPS).enumerate() {
        let baseline = w.at(0, FIRST_STEP_BASELINE + row as i32 * STEP_PITCH);
        let step = &view.steps[index];

        // Three states, three inks, and no rule through any of them. The number carries the
        // position; the weight carries which one is running.
        let (number_ink, text_ink, face) = if index == view.current_step {
            (palette::INK, palette::INK, &type_scale::STEP_CURRENT)
        } else if index < view.current_step {
            (palette::INK_FAINT, palette::INK_FAINT, &type_scale::STEP_OTHER)
        } else {
            (palette::INK_FAINT, palette::INK_MUTED, &type_scale::STEP_OTHER)
        };

        draw::run(
            &type_scale::LABEL,
            format_args!("{}", index + 1),
            baseline,
            VerticalPosition::Baseline,
            number_ink,
            target,
        );
        draw::run(
            face,
            format_args!("{}", step.description),
            Point::new(baseline.x + 13, baseline.y),
            VerticalPosition::Baseline,
            text_ink,
            target,
        );
    }

    Ok(())
}

fn values<D>(view: &RoutineView<'_>, w: Window, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    for (dx, label, value, unit, pen) in [
        (RIGHT_DX, "IN STEP", view.step_elapsed_s, "S", palette::INK),
        (
            VALUES_SECOND_DX,
            "WEIGHT",
            view.weight_g,
            "G",
            palette::PEN_WEIGHT,
        ),
    ] {
        draw::run(
            &type_scale::LABEL,
            format_args!("{label}"),
            w.at(dx, CONTENT_TOP + 10),
            VerticalPosition::Baseline,
            palette::INK_MUTED,
            target,
        );
        let baseline = w.at(dx, VALUES_BASELINE);
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
            format_args!("{unit}"),
            Point::new(after + 3, baseline.y),
            VerticalPosition::Baseline,
            palette::INK_FAINT,
            target,
        );
    }

    // Pressure against what it was asked for, and water in.
    let baseline = w.at(RIGHT_DX, SECOND_ROW_BASELINE);
    let after = widgets::value(
        &type_scale::SECONDARY_19,
        view.pressure_bar,
        2,
        baseline,
        VerticalPosition::Baseline,
        palette::PEN_PRESSURE,
        target,
    )?;
    let after = draw::run(
        &type_scale::LABEL,
        format_args!("BAR"),
        Point::new(after + 4, baseline.y),
        VerticalPosition::Baseline,
        palette::INK_FAINT,
        target,
    );
    if let Some(target_bar) = view.pressure_target {
        draw::run(
            &type_scale::UNIT_12,
            format_args!("/ {target_bar:.1}"),
            Point::new(after + 4, baseline.y),
            VerticalPosition::Baseline,
            palette::INK_MUTED,
            target,
        );
    }

    let baseline = w.at(WATER_DX, SECOND_ROW_BASELINE);
    let after = widgets::value(
        &type_scale::SECONDARY_19,
        view.water_in_ml,
        0,
        baseline,
        VerticalPosition::Baseline,
        palette::PEN_WATER_IN,
        target,
    )?;
    draw::run(
        &type_scale::LABEL,
        format_args!("ML IN"),
        Point::new(after + 4, baseline.y),
        VerticalPosition::Baseline,
        palette::INK_FAINT,
        target,
    );

    Ok(())
}

/// What ends this step: the threshold, and the bar. Once.
fn exit_footer<D>(view: &RoutineView<'_>, w: Window, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let left = w.at(RIGHT_DX, 0).x;
    let right = w.body_right();
    let baseline = w.at(RIGHT_DX, EXIT_BASELINE);

    match view.exit {
        ExitView::Phrase(phrase) => {
            // The phrase earns its place here and nowhere else: there is no bar to draw, so
            // words are the only encoding left.
            draw::run(
                &type_scale::LABEL,
                format_args!("{phrase}"),
                baseline,
                VerticalPosition::Baseline,
                palette::INK_MUTED,
                target,
            );
        }
        ExitView::Progress {
            phrase: _,
            current,
            target: threshold,
            quantity,
        } => {
            let after = draw::run(
                &type_scale::LABEL,
                format_args!("ENDS AT"),
                baseline,
                VerticalPosition::Baseline,
                palette::INK_MUTED,
                target,
            );
            let after = draw::run(
                &type_scale::NUMBER_FLOOR,
                format_args!("{:.*}", quantity.decimals(), threshold),
                Point::new(after + 5, baseline.y),
                VerticalPosition::Baseline,
                palette::INK,
                target,
            );
            draw::run(
                &type_scale::LABEL,
                format_args!("{}", quantity.unit_upper()),
                Point::new(after + 4, baseline.y),
                VerticalPosition::Baseline,
                palette::INK_FAINT,
                target,
            );

            // A quantity nothing is reporting draws an empty bar rather than a full one: the
            // alternative reading of "no current value" is "we are there", and that is the
            // reading that would have someone lift a cup off a scale mid-shot.
            let fraction = match (current, threshold) {
                (Some(value), t) if t > 0.0 => value / t,
                _ => 0.0,
            };
            widgets::progress(
                Rectangle::new(
                    w.at(RIGHT_DX, EXIT_BAR_DY),
                    Size::new((right - left) as u32, EXIT_BAR_HEIGHT),
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
