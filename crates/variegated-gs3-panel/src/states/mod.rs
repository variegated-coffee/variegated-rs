//! One module per state, each drawing section 6's figure for it.
//!
//! Every one of them lays out from [`crate::geometry`] and draws only inside the visible
//! window. None of them clears the panel or draws the status strip: [`crate::render`] does
//! both, so the strip cannot end up in a different place in one state than in another.

pub mod free_brew;
pub mod idle;
pub mod off;
pub mod post;
pub mod routine;

use crate::view::StateView;

/// How often a state's values are worth redrawing, in milliseconds. Section 8.
///
/// The panel does not animate: each state is a fixed frame with a small number of value
/// windows, and nothing moves, resizes or reflows while a state is held. So the caller does
/// not need to redraw at the rate its loop happens to run at -- it redraws when the state
/// changes, and otherwise this often.
///
/// It is not free to redraw: the driver compares the new frame against the previous one to
/// find what to send, and that comparison is 143 KB whether or not anything moved. A
/// finished shot is a still image, and drawing it a hundred times a second was the largest
/// avoidable cost on this panel.
pub fn redraw_period_ms(state: &StateView<'_>) -> u32 {
    match state {
        // A clock with no seconds. Redrawing faster cannot change a pixel.
        StateView::Off(_) => 1_000,
        // Two temperatures and a deviation, against boilers whose time constants are in
        // seconds.
        StateView::Idle(_) => 500,
        // The rail and five values, while someone is standing there watching them.
        StateView::FreeBrew(_) | StateView::Routine(_) => 100,
        // Drawn once on entry. The period is a floor against a missed change, not a rate.
        StateView::Post(_) => 1_000,
    }
}
