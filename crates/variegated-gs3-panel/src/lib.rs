//! The GS3's 428x168 status panel.
//!
//! This crate is the display specification made executable: the geometry of the 390x115
//! window the bezel leaves visible, the two-family type ladder, the palette, the five
//! machine states and the overlays that go over them. It draws against any
//! [`DrawTarget`](embedded_graphics::draw_target::DrawTarget) whose colour is
//! [`Rgb565`](embedded_graphics::pixelcolor::Rgb565) and knows nothing else about the
//! machine.
//!
//! # Why it is a crate rather than a module in the firmware
//!
//! The firmware's only target sets `test = false` and depends on `embassy-rp`, so nothing
//! that lives there can be compiled on a host -- and a 390x115 pixel specification whose
//! only verification is "flash it and look" is a specification nobody checks. Everything
//! here builds and runs on a host, `cargo test` covers the arithmetic, and
//! `cargo run --example render_png` puts every state and every variant on disk as an image
//! to hold against the figures.
//!
//! That is the same reason `variegated-menu` and `variegated-machine-menu` exist, and the
//! bugs it is aimed at are the same kind: things that *disappear* without a trace, which is
//! not a class of fault you can find by looking at a panel.
//!
//! # What it takes
//!
//! [`PanelView`], which is entirely resolved values -- see [`view`] for the split between
//! what the caller formats and what this crate does.

#![no_std]
#![warn(missing_docs)]

// The test harness is a std binary, and the layout probe in `draw` keeps its rectangles in a
// `Vec`. Nothing outside `cfg(test)` names it.
#[cfg(test)]
extern crate std;

mod draw;
#[cfg(any(test, feature = "fixtures"))]
pub mod fixtures;
pub mod geometry;
pub mod marks;
pub mod overlays;
pub mod palette;
pub mod states;
pub mod trace;
pub mod type_scale;
pub mod view;
mod widgets;

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;

pub use geometry::Window;
pub use trace::{Phase, ShotTrace};
pub use view::{
    Clock, Command, ExitView, FreeBrewView, HourMinute, IdleView, Mark, MarkState, NextEvent,
    OffView, Outcome, Overlay, PanelView, PostView, Quantity, Readiness, RoutineView, StateView,
    StepView,
};

/// Draw a whole frame.
///
/// The panel is cleared to black first and everything is redrawn. There is no dirty
/// tracking here on purpose: the NV3007 driver compares the frame against the previous one
/// and sends only the regions that changed, so a second layer of it in the drawing code
/// would be two sources of truth about what moved. What the caller *should* do is not call
/// this every 10 ms -- see the redraw budget in section 8, and [`states::redraw_period_ms`].
pub fn render<D>(view: &PanelView<'_>, w: Window, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    // An identify flash replaces the screen rather than sitting on it. The point of Improv
    // Identify is to answer "which of these machines am I talking to" for someone standing
    // in the room, and a panel that alternates fully lit and fully dark answers that in a
    // way no amount of text on the usual screen can.
    if let Some(Overlay::Identify { lit }) = view.overlay {
        return target.clear(if lit { palette::INK } else { palette::SURFACE });
    }

    target.clear(palette::SURFACE)?;

    match &view.state {
        StateView::Off(off) => states::off::draw(off, w, target)?,
        StateView::Idle(idle) => states::idle::draw(idle, w, target)?,
        StateView::FreeBrew(brew) => states::free_brew::draw(brew, w, target)?,
        StateView::Routine(routine) => states::routine::draw(routine, w, target)?,
        StateView::Post(post) => states::post::draw(post, w, target)?,
    }

    // Free-brewing puts its marks in the header, in a horizontal row, because the panel's
    // full width is spent on the rail. Every other state has the column at the right edge.
    if matches!(view.state, StateView::FreeBrew(_)) {
        states::free_brew::header_marks(&view.marks, w, target)?;
    } else {
        marks::draw_column(&view.marks, w, target)?;
    }

    // After the state renderer, so an overlay is on top in every state rather than in the
    // ones that happened to be considered.
    if let Some(overlay) = view.overlay {
        overlays::draw(&overlay, w, target)?;
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_graphics::primitives::Rectangle;
    use std::vec::Vec;

    /// A target that keeps the bounding box of everything drawn into it, except the
    /// background.
    #[derive(Default)]
    struct Bounds {
        min: Option<Point>,
        max: Option<Point>,
    }

    impl OriginDimensions for Bounds {
        fn size(&self) -> Size {
            geometry::PANEL_SIZE
        }
    }

    impl DrawTarget for Bounds {
        type Color = Rgb565;
        type Error = core::convert::Infallible;

        fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
        where
            I: IntoIterator<Item = Pixel<Self::Color>>,
        {
            for Pixel(p, _) in pixels {
                self.min = Some(match self.min {
                    None => p,
                    Some(m) => Point::new(m.x.min(p.x), m.y.min(p.y)),
                });
                self.max = Some(match self.max {
                    None => p,
                    Some(m) => Point::new(m.x.max(p.x), m.y.max(p.y)),
                });
            }
            Ok(())
        }

        /// The background is the whole panel by definition, so recording it would make the
        /// bounds meaningless.
        fn clear(&mut self, _color: Self::Color) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    fn drawn(view: &PanelView<'_>, w: Window) -> (Option<Rectangle>, Vec<Rectangle>) {
        draw::probe::reset();
        let mut bounds = Bounds::default();
        render(view, w, &mut bounds).unwrap();
        let rect = match (bounds.min, bounds.max) {
            (Some(min), Some(max)) => Some(Rectangle::with_corners(min, max)),
            _ => None,
        };
        (rect, draw::probe::taken())
    }

    /// The windows a trimmed machine can actually be in.
    ///
    /// The default, and both ends of the travel. The extremes are the ones worth having: a
    /// layout that only holds at `(25, 34)` is a layout that breaks the first time somebody
    /// uses the setting.
    fn every_window() -> [Window; 3] {
        [
            Window::DEFAULT,
            Window::new(0, 0),
            Window::new(Window::MAX_ORIGIN.x, Window::MAX_ORIGIN.y),
        ]
    }

    /// Nothing may live outside the 390x115 window: the rest of the panel is addressable and
    /// permanently behind the bezel, so a figure drawn there is a figure nobody will ever
    /// see.
    #[test]
    fn nothing_escapes_the_visible_window() {
        let trace = fixtures::lever_like_trace();
        let aborted = fixtures::lever_like_trace_to(12.4);
        for w in every_window() {
            let window = w.rect();
            for (name, view) in fixtures::all(&trace, &aborted) {
                // The identify flash is a deliberate full-panel takeover.
                if matches!(view.overlay, Some(Overlay::Identify { .. })) {
                    continue;
                }
                let (Some(rect), _) = drawn(&view, w) else {
                    panic!("{name} drew nothing at all");
                };
                assert!(
                    window.intersection(&rect) == rect,
                    "{name} drew {rect:?}, which leaves the window {window:?}",
                );
            }
        }
    }

    /// Moving the window moves everything, by exactly the same amount.
    ///
    /// This is the assertion the origin had to become a value to make possible, and it is
    /// aimed at the one failure this refactor can introduce: a coordinate that was written
    /// absolute rather than window-relative. Such a thing still draws, still stays inside
    /// the window at the default origin, and still passes the overlap check -- it only comes
    /// apart once somebody trims the panel, which is the moment nobody is watching a test.
    #[test]
    fn shifting_the_window_shifts_every_pixel_with_it() {
        let trace = fixtures::lever_like_trace();
        let aborted = fixtures::lever_like_trace_to(12.4);
        let from = Window::DEFAULT;
        let to = Window::new(from.origin().x + 7, from.origin().y + 11);
        let delta = to.origin() - from.origin();

        for (name, view) in fixtures::all(&trace, &aborted) {
            let (_, before) = drawn(&view, from);
            let (_, after) = drawn(&view, to);
            // Without this the loop below is vacuous on an empty recording, and the test
            // would pass by measuring nothing.
            assert!(!before.is_empty(), "{name} recorded no runs at all");
            assert_eq!(
                before.len(),
                after.len(),
                "{name} drew a different number of runs at a different origin",
            );
            for (a, b) in before.iter().zip(after.iter()) {
                assert_eq!(
                    b.top_left - a.top_left,
                    delta,
                    "{name}: a run moved by {:?} instead of {delta:?}",
                    b.top_left - a.top_left,
                );
                assert_eq!(a.size, b.size, "{name}: a run changed size");
            }
        }
    }

    /// No two runs of text may share a pixel.
    ///
    /// The first pass of this crate had five panels with figures drawn through their own
    /// labels, and every one of them rendered without an error from anything. The faces are
    /// u8g2 bitmaps at sizes the specification's outline figures do not predict, so this is
    /// not something a layout can be reasoned into being right about -- it has to be
    /// measured, on every variant, every time.
    #[test]
    fn no_two_runs_of_text_overlap() {
        let trace = fixtures::lever_like_trace();
        let aborted = fixtures::lever_like_trace_to(12.4);
        let mut failures = std::vec::Vec::new();
        for (name, view) in fixtures::all(&trace, &aborted) {
            let (_, runs) = drawn(&view, Window::DEFAULT);
            for (i, a) in runs.iter().enumerate() {
                for b in &runs[i + 1..] {
                    let overlap = a.intersection(b);
                    if !overlap.is_zero_sized() {
                        failures.push(std::format!("{name}: {a:?} overlaps {b:?}"));
                    }
                }
            }
        }
        assert!(
            failures.is_empty(),
            "text overlaps in {} place(s):\n{}",
            failures.len(),
            failures.join("\n")
        );
    }
}
