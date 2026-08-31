//! The GS3's 428x168 status panel.
//!
//! This crate is the display specification made executable: the geometry of the 396x111
//! window the bezel leaves visible, the two-family type ladder, the palette, the five
//! machine states and the overlays that go over them. It draws against any
//! [`DrawTarget`](embedded_graphics::draw_target::DrawTarget) whose colour is
//! [`Rgb565`](embedded_graphics::pixelcolor::Rgb565) and knows nothing else about the
//! machine.
//!
//! # Why it is a crate rather than a module in the firmware
//!
//! The firmware's only target sets `test = false` and depends on `embassy-rp`, so nothing
//! that lives there can be compiled on a host -- and a 396x111 pixel specification whose
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
pub mod rhythm;
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
///
/// # `always-draw-bounds`
///
/// With that feature on, every frame ends with [`geometry::calibration_frame`] over the top
/// of it -- the same border and ticks the origin editors draw, on every state, all the time.
///
/// It answers a different question from the editors. They ask how well the drawing is
/// centred in the aperture, with nothing else on the screen to judge it against; this asks
/// whether the bezel's cutout is where the drawing thinks it is *at all*, against whatever
/// the machine happens to be showing. A build-time feature rather than a setting because the
/// frame sits on the status strip and the marks, which are flush to the window's edges: it
/// costs something on every screen, and that is a decision for a manifest rather than for
/// whoever was last in the menu.
pub fn render<D>(view: &PanelView<'_>, w: Window, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    // An identify flash replaces the screen rather than sitting on it. The point of Improv
    // Identify is to answer "which of these machines am I talking to" for someone standing
    // in the room, and a panel that alternates fully lit and fully dark answers that in a
    // way no amount of text on the usual screen can. `always-draw-bounds` does not reach it:
    // the frame is invisible on the lit phase and would put a lit rectangle on the dark one,
    // which is the half of the flash that carries the signal.
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

    // One position, one size, every state. Free-brewing used to lay its marks along the
    // header at half the spacing so the rail could have the panel's full width; read as a
    // set, that made the status strip move and shrink on exactly the four screens where the
    // machine is doing something.
    marks::draw_column(&view.marks, w, target)?;

    // After the state renderer, so an overlay is on top in every state rather than in the
    // ones that happened to be considered.
    if let Some(overlay) = view.overlay {
        overlays::draw(&overlay, w, states::overlay_floor(&view.state), target)?;
    }

    // Last of all, so nothing can cover the one thing being looked at. It sits on the status
    // strip and the marks, which are flush to the window's edges -- that is the cost of the
    // feature, and the reason it is not something a machine ships with.
    #[cfg(feature = "always-draw-bounds")]
    geometry::calibration_frame(w, target)?;

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

    /// Nothing may live outside the visible window: the rest of the panel is addressable and
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

    /// An unpaired scale changes the mark and nothing else about idle.
    ///
    /// The third review looked at two idle renders that differed only in one mark being red
    /// instead of green, both saying `READY`, and could not tell from the images whether that
    /// was correct or one of them drawing a stale status. It is correct: the routine and the
    /// dose came off this panel in the first remediation, so nothing idle draws depends on a
    /// scale -- and a scale has nothing to do with whether the boilers are at temperature,
    /// which is the only thing `READY` claims.
    ///
    /// Asserted rather than left in a comment, because the question will be asked again by
    /// whoever next reads the two side by side. If a weight ever returns to this state, this
    /// fails and says so.
    #[test]
    fn an_unpaired_scale_changes_only_the_mark_in_idle() {
        let (_, ready) = drawn(&fixtures::idle_ready(), Window::DEFAULT);
        let (_, no_scale) = drawn(&fixtures::idle_no_scale(), Window::DEFAULT);
        assert_eq!(
            ready, no_scale,
            "idle drew something different with no scale paired",
        );
    }

    /// How much room each state has left at the foot of the window.
    ///
    /// The third remediation shortened the window by four pixels and asked which element each
    /// state should give them up from, working from heights read off rendered images. Read off
    /// the layout instead, no state had to give up anything -- but three of them now finish
    /// within two pixels of the bottom edge, which is the answer to that document's own open
    /// question about whether 111 is the floor. It is: one more pixel and idle, routine
    /// execution and post-routine all break together.
    ///
    /// `cargo test -p variegated-gs3-panel --features fixtures -- --nocapture` prints the
    /// table. The assertion is only that nothing has run out, because the point of the number
    /// is to be read before the window changes again, not to be defended at some value.
    #[test]
    fn every_state_reports_its_headroom() {
        let trace = fixtures::lever_like_trace();
        let aborted = fixtures::lever_like_trace_to(12.4);
        let w = Window::DEFAULT;
        let floor = w.origin().y + geometry::WINDOW_SIZE.height as i32;
        for (name, view) in fixtures::all(&trace, &aborted) {
            if matches!(view.overlay, Some(Overlay::Identify { .. })) {
                continue;
            }
            let (Some(rect), _) = drawn(&view, w) else {
                continue;
            };
            let headroom = floor - (rect.top_left.y + rect.size.height as i32);
            std::println!("{name:28} {headroom:3} px of headroom");
            assert!(headroom >= 0, "{name} has overrun the window by {headroom}");
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
