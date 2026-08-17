//! Press, chord and press-and-hold recognition from raw button samples.
//!
//! Extracted from the GS3 firmware because it is the highest-bug-density code on that
//! machine's input path and there was no way to test it. It stayed untestable for one
//! reason: it took an `embassy_time::Instant`, and a host test binary that links
//! `embassy-time` without a time driver fails at link on `_embassy_time_now` -- a failure
//! that mentions neither time nor testing. So this crate takes **milliseconds as a plain
//! `u64`** and the firmware converts at the one call site.
//!
//! The bug that forced the extraction: a chord was recognised on the way down and then
//! thrown away on the way up. Releasing part of a held set was indistinguishable from
//! forming a new, smaller set, so `{3,5}` became `{5}` the instant one finger lifted --
//! and since two fingers cannot leave within one sample, *every* chord degraded to
//! whichever button was released last. See [`ButtonEventRecognizer`].

#![no_std]
#![warn(missing_docs)]

// The crate itself is alloc-free; its tests collect emitted events into a `Vec`.
#[cfg(test)]
extern crate std;

mod recognizer;

pub use recognizer::{
    ButtonEvent, ButtonEventRecognizer, ButtonSet, PRESS_AND_HOLD_THRESHOLD_MS,
    SETTLING_DELAY_MS,
};
