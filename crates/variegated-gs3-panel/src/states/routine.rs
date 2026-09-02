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
use crate::rhythm::{self, Stack};
use crate::slots::{self, Annotation, Cell};
use crate::type_scale;
use crate::view::{ExitView, RoutineView};
use crate::widgets;

/// Where the first row's ink starts, from the window top.
const TOP: i32 = rhythm::MARGIN;

/// The spine's width. Holds `Declining profile` at 12 px beside its number.
///
/// All six pixels of the resize went here, and they were needed: this is the only column in
/// the set whose content was being clipped, and six pixels is about one and a half characters
/// at `helvR12`. It is still not enough on its own -- a step name is the routine author's, so
/// no column width bounds it -- which is what [`draw::fitted`] is for.
const SPINE_WIDTH: i32 = 134;

/// Where a step's name starts, in from the spine's left edge: past its number.
const STEP_TEXT_DX: i32 = 12;
const SPINE_DIVIDER_DX: i32 = SPINE_WIDTH;
const RIGHT_DX: i32 = SPINE_DIVIDER_DX + rhythm::RULE_WIDTH;

/// Four step rows fit. Beyond that the window scrolls around the current step.
const VISIBLE_STEPS: usize = 4;

/// The exit bar.
const EXIT_BAR_HEIGHT: i32 = 5;

/// Mid-gap below the two big figures, which is also mid-gap in the spine beside them.
/// See [`crate::states::overlay_floor`].
pub(crate) const OVERLAY_FLOOR: i32 = TOP
    + type_scale::LABEL.height()
    + rhythm::PITCH
    + type_scale::LABEL.height()
    + rhythm::PAIR
    + type_scale::PRIMARY_30.height()
    + rhythm::PITCH / 2;

/// Where the right half's rows sit.
///
/// A struct because the two halves are laid out independently -- the spine's rhythm is its own
/// step pitch -- but both have to end inside the same window, and the exit bar is the thing
/// that finds out first if they do not.
struct Rows {
    header: i32,
    content_top: i32,
    label: i32,
    value: i32,
    second: i32,
    exit: i32,
    bar: i32,
}

fn rows() -> Rows {
    let mut stack = Stack::new(TOP);
    let header = stack.row(&type_scale::LABEL);
    let content_top = stack.bottom() + rhythm::PITCH;
    let label = stack.row(&type_scale::LABEL);
    // Paired: `IN STEP` and the figure under it are one object, and they were 0 px apart --
    // two glyph boxes touching, which reads as a collision rather than as a pair.
    let value = stack.paired(&type_scale::PRIMARY_30);
    let second = stack.row(&type_scale::SECONDARY_19);
    let exit = stack.row(&type_scale::LABEL);
    let bar = stack.block(EXIT_BAR_HEIGHT);
    Rows {
        header,
        content_top,
        label,
        value,
        second,
        exit,
        bar,
    }
}

pub(crate) fn draw<D>(view: &RoutineView<'_>, w: Window, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let rows = rows();
    header(view, w, w.at(0, rows.header).y, target);
    spine(view, w, rows.content_top, target)?;
    hairline_v(
        w.at(SPINE_DIVIDER_DX + rhythm::RULE_GAP, rows.content_top),
        (crate::geometry::WINDOW_SIZE.height as i32 - rows.content_top - 2) as u32,
        target,
    )?;
    values(view, w, &rows, target)?;
    exit_footer(view, w, &rows, target)?;
    Ok(())
}

/// The top rule: where you are in the routine, and which routine it is.
///
/// The step *name* is not here -- the bold row in the spine states it once, and stating it
/// twice is what the 8 px tier was spending its height on.
fn header<D>(view: &RoutineView<'_>, w: Window, baseline: i32, target: &mut D)
where
    D: DrawTarget<Color = Rgb565>,
{
    let after = draw::run(
        &type_scale::LABEL,
        format_args!("STEP"),
        Point::new(w.at(0, 0).x, baseline),
        VerticalPosition::Baseline,
        palette::INK_MUTED,
        target,
    );
    draw::run(
        &type_scale::LABEL,
        format_args!("{}/{}", view.current_step + 1, view.steps.len()),
        Point::new(after + rhythm::GAP, baseline),
        VerticalPosition::Baseline,
        palette::INK,
        target,
    );
    draw::aligned(
        &type_scale::LABEL,
        format_args!("{}", view.name),
        Point::new(w.body_right(), baseline),
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

fn spine<D>(
    view: &RoutineView<'_>,
    w: Window,
    content_top: i32,
    target: &mut D,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    // The step list is a stack like any other, so its rows sit at the panel's pitch instead of
    // at a step pitch of their own -- and the one face here that has a descent gets it counted,
    // which a fixed pitch could not do.
    let mut stack = Stack::new(content_top);
    let start = window_start(view.current_step, view.steps.len());
    for index in (start..view.steps.len()).take(VISIBLE_STEPS) {
        let baseline = Point::new(w.at(0, 0).x, w.at(0, stack.row(&type_scale::STEP_OTHER)).y);
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
        // Cut to the column rather than allowed to run into the rule. The current step is set
        // in the bold face, which is the widest one here and therefore the one that overran --
        // so the state where the name matters most was the state that broke.
        let (name, cut) = draw::fitted(face, step.description, SPINE_WIDTH - STEP_TEXT_DX);
        let after = draw::run(
            face,
            format_args!("{name}"),
            Point::new(baseline.x + STEP_TEXT_DX, baseline.y),
            VerticalPosition::Baseline,
            text_ink,
            target,
        );
        if cut {
            draw::run(
                face,
                format_args!("."),
                Point::new(after, baseline.y),
                VerticalPosition::Baseline,
                text_ink,
                target,
            );
        }
    }

    Ok(())
}

/// How precisely a reference value is drawn: one decimal less than the measurement it sits
/// beside.
///
/// A target and a limit are numbers a person set, not readings -- nobody dials a cap to a
/// hundredth of a millilitre per second -- and this is already how this panel treats the
/// distinction: free-brewing draws its measured value at two decimals and the command beside
/// it at one, for the same reason and in the same two quantities.
///
/// It also buys back about twelve pixels on a row that is 221 px wide and, at `MAX 4.00`,
/// was missing the next data point by three.
fn reference_decimals(quantity: crate::view::Quantity) -> usize {
    quantity.decimals().saturating_sub(1)
}

/// Draw the annotation a role carries, after `x` on `baseline`.
///
/// On the label row rather than beside the figure. `8.39 BAR / 9.0` was already the longest
/// run on this half of the panel at 19 px; at `PRIMARY_30` it overruns the 104 px column and
/// puts the target through the slot beside it. In the label face it costs nothing and leaves
/// the large figure to be a large figure.
fn annotation<D>(
    annotation: Annotation,
    quantity: crate::view::Quantity,
    x: i32,
    baseline: i32,
    target: &mut D,
) where
    D: DrawTarget<Color = Rgb565>,
{
    let decimals = reference_decimals(quantity);
    match annotation {
        Annotation::Target(value) => {
            draw::run(
                &type_scale::LABEL,
                format_args!("/ {value:.decimals$}"),
                Point::new(x + rhythm::TIGHT, baseline),
                VerticalPosition::Baseline,
                palette::INK_MUTED,
                target,
            );
        }
        // `MAX`, not `/`, because a cap and a setpoint are different promises and the slot
        // beside this one may well be carrying the other. Warn-coloured while it is actually
        // holding the machine back: armed is a setting, binding is a thing that is happening,
        // and this is the only place on the panel that difference can be seen.
        Annotation::Limit { value, binding } => {
            draw::run(
                &type_scale::LABEL,
                format_args!("MAX {value:.decimals$}"),
                Point::new(x + rhythm::TIGHT, baseline),
                VerticalPosition::Baseline,
                if binding {
                    palette::WARN
                } else {
                    palette::INK_MUTED
                },
                target,
            );
        }
    }
}

/// How wide an annotation draws, including the gap before it.
fn annotation_width(note: Annotation, quantity: crate::view::Quantity) -> i32 {
    let decimals = reference_decimals(quantity);
    let text = match note {
        Annotation::Target(value) => draw::width(
            &type_scale::LABEL,
            format_args!("/ {value:.decimals$}"),
        ),
        Annotation::Limit { value, .. } => draw::width(
            &type_scale::LABEL,
            format_args!("MAX {value:.decimals$}"),
        ),
    };
    rhythm::TIGHT + text
}

/// How wide a secondary cell needs.
fn secondary_width(cell: &Cell) -> i32 {
    let quantity = cell.offer.point.quantity();
    widgets::value_width(
        &type_scale::NUMBER_FLOOR,
        cell.offer.value,
        quantity.decimals(),
    ) + rhythm::TIGHT
        + draw::width(&type_scale::LABEL, format_args!("{}", quantity.unit_upper()))
        + cell
            .annotation
            .map_or(0, |note| annotation_width(note, quantity))
}

/// The hero cell: its name over its figure, at the top of the ranking.
fn hero_slot<D>(cell: &Cell, w: Window, rows: &Rows, x: i32, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let point = cell.offer.point;
    let quantity = point.quantity();

    let label_y = w.at(0, rows.label).y;
    let after = draw::run(
        &type_scale::LABEL,
        format_args!("{}", point.label()),
        Point::new(x, label_y),
        VerticalPosition::Baseline,
        palette::INK_MUTED,
        target,
    );
    if let Some(note) = cell.annotation {
        annotation(note, quantity, after, label_y, target);
    }

    let baseline = Point::new(x, w.at(0, rows.value).y);
    let after = widgets::value(
        &type_scale::PRIMARY_30,
        cell.offer.value,
        quantity.decimals(),
        baseline,
        VerticalPosition::Baseline,
        palette::pen(quantity),
        target,
    )?;
    draw::run(
        &type_scale::UNIT_12,
        format_args!("{}", quantity.unit_upper()),
        Point::new(after + rhythm::TIGHT, baseline.y),
        VerticalPosition::Baseline,
        palette::INK_FAINT,
        target,
    );

    Ok(())
}

/// A secondary cell: figure, unit, and a reference value where it carries one.
///
/// No name over it. Among the seven ranked points the unit is already unique -- `S`, `G`,
/// `BAR`, `ML/S`, `MS/CM`, `C`, `ML` -- so a label would spend 40 to 80 px restating what the
/// unit beside the figure has said, and this row does not have 80 px to spend. That is the
/// same reason the row this replaces drew `8.39 BAR` and `46 ML IN` with no label at all.
fn secondary_slot<D>(cell: &Cell, x: i32, baseline: i32, target: &mut D) -> Result<i32, D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let quantity = cell.offer.point.quantity();

    let after = widgets::value(
        &type_scale::NUMBER_FLOOR,
        cell.offer.value,
        quantity.decimals(),
        Point::new(x, baseline),
        VerticalPosition::Baseline,
        palette::pen(quantity),
        target,
    )?;
    let after = draw::run(
        &type_scale::LABEL,
        format_args!("{}", quantity.unit_upper()),
        Point::new(after + rhythm::TIGHT, baseline),
        VerticalPosition::Baseline,
        palette::INK_FAINT,
        target,
    );
    if let Some(note) = cell.annotation {
        annotation(note, quantity, after, baseline, target);
        return Ok(after + annotation_width(note, quantity));
    }
    Ok(after)
}

/// Draw the selected cells: one hero, then as many secondary cells as the row will hold.
///
/// # Why one hero rather than the two large figures this screen used to have
///
/// The right half is 221 px. Measured in the faces actually used, one `PRIMARY_30` figure
/// with its unit is 98 px for `99.9 S` and 143 px for `12.00 BAR`, so two of them run from
/// 197 px to 269 px: the pair fits only when both readings happen to be narrow, and which
/// data points land there is now decided per frame rather than at design time. Sizing the
/// columns off the live values instead would let a figure change size mid-shot as a reading
/// crossed a digit, which is worse than a figure that is consistently smaller.
///
/// So the top of the ranking gets the big figure and the rest go to the row below it, at
/// [`type_scale::NUMBER_FLOOR`] -- the size the type scale names as the floor for a number,
/// not a new tier.
///
/// # Why the row is packed and fit-checked
///
/// Cells are laid left to right from what each one actually measures, divided by hairlines,
/// exactly as `free_brew`'s bottom row is and for the same reason: a grid of equal columns
/// puts the unit of one cell through the figure of the next as soon as the content varies.
/// Because the content here varies by *step*, the row additionally stops early rather than
/// overrunning -- a cell that will not fit is not drawn. The ranking is what makes that safe:
/// the cell dropped is always the least important one on offer.
fn values<D>(
    view: &RoutineView<'_>,
    w: Window,
    rows: &Rows,
    target: &mut D,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let cells = slots::select(view);
    let left = w.at(RIGHT_DX, 0).x;
    let right = w.body_right();

    let mut remaining = cells.iter().flatten();

    if let Some(cell) = remaining.next() {
        // Never fit-checked: the hero is alone on its row, and the widest cell this panel can
        // build is narrower than the row. A check here could only ever suppress the most
        // important figure on the screen.
        hero_slot(cell, w, rows, left, target)?;
    }

    let baseline = w.at(0, rows.second).y;
    let mut x = left;
    let mut drawn = 0;
    for cell in remaining {
        let start = if drawn == 0 { x } else { x + rhythm::RULE_WIDTH };
        if start + secondary_width(cell) > right {
            break;
        }
        if drawn > 0 {
            rhythm::divider(x, baseline, &type_scale::NUMBER_FLOOR, target)?;
        }
        x = secondary_slot(cell, start, baseline, target)?;
        drawn += 1;
    }

    Ok(())
}

/// What ends this step: the threshold, and the bar. Once.
fn exit_footer<D>(
    view: &RoutineView<'_>,
    w: Window,
    rows: &Rows,
    target: &mut D,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let left = w.at(RIGHT_DX, 0).x;
    let right = w.body_right();
    let baseline = w.at(RIGHT_DX, rows.exit);

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
            point,
            current,
            target: threshold,
        } => {
            // Rank 1, and the only figure on this screen that is not in the grid. It reads
            // `WEIGHT 6.2 / 8.0 G`: the point's name, so the footer says which quantity the
            // bar under it is measuring, then where it is against where it has to get to.
            //
            // The module note above records that this condition was once stated three ways
            // and was cut back to one. This is two -- the pair and the bar -- and the phrase
            // stays gone. Deliberate: the threshold alone answers "what ends this step" but
            // not "how far along am I", and the bar is five pixels tall.
            let quantity = point.quantity();
            let decimals = quantity.decimals();

            let after = draw::run(
                &type_scale::LABEL,
                format_args!("{}", point.label()),
                baseline,
                VerticalPosition::Baseline,
                palette::INK_MUTED,
                target,
            );
            let after = widgets::value(
                &type_scale::NUMBER_FLOOR,
                current,
                decimals,
                Point::new(after + rhythm::TIGHT, baseline.y),
                VerticalPosition::Baseline,
                palette::pen(quantity),
                target,
            )?;
            let after = draw::run(
                &type_scale::LABEL,
                format_args!("/ {threshold:.decimals$}"),
                Point::new(after + rhythm::TIGHT, baseline.y),
                VerticalPosition::Baseline,
                palette::INK_MUTED,
                target,
            );
            draw::run(
                &type_scale::LABEL,
                format_args!("{}", quantity.unit_upper()),
                Point::new(after + rhythm::TIGHT, baseline.y),
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
                    w.at(RIGHT_DX, rows.bar),
                    Size::new((right - left) as u32, EXIT_BAR_HEIGHT as u32),
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
