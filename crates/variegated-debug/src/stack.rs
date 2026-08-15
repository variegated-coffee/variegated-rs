//! How much of core 0's stack has ever been used.
//!
//! # Why this is worth having permanently
//!
//! Core 0 on the application processor has **no stack guard unless
//! `install_core0_stack_guard()` is called**, and until 2026-08-10 it was not. The board
//! overflowed this stack silently for an unknown length of time, and the failure surfaced
//! as a HardFault in the timer queue -- nowhere near the cause. A number in the 1 Hz debug
//! snapshot is what makes "we are close to the edge" visible before it is fatal.
//!
//! # How it works
//!
//! No painting of its own is needed. `cortex-m-rt`'s `paint-stack` feature fills
//! everything between `__sheap` and `_stack_start` with `0xCCCC_CCCC` before `main` runs.
//! Core 0's stack grows *down* from `_stack_start`, so untouched paint survives at the
//! bottom, and the high-water mark is the distance from the last painted word to the top.
//!
//! **The consuming binary must enable `cortex-m-rt/paint-stack`.** Without it these
//! functions read whatever was in SRAM at reset and report a meaningless number rather
//! than failing, so there is nothing here that can check it.
//!
//! # Why it is in this crate
//!
//! Three copies of this existed, in two examples, and they had already disagreed: one pair
//! read `__sheap`/`_stack_start`, a third read `_stack_end`/`_stack_start`, and both were
//! live in the same binary reporting against the same debug link. The only consumer is the
//! debug snapshot, which is why this is here rather than in `variegated-hal` -- and this
//! crate already carries the chip features, for `usb-cdc-rp`.
//!
//! Core 1 is not here. Its stack is a `static` declared by whichever binary calls
//! `spawn_core1`, painted by that binary, and only one board has one -- there is nothing
//! to share.

/// Total bytes available to core 0's stack: `_stack_start - __sheap`.
///
/// A function rather than a constant because both bounds are linker symbols, resolved at
/// link time. This is also the value `install_core0_stack_guard()` programs into `MSPLIM`,
/// so a high-water reading approaching it means the guard is about to fire.
pub fn core0_span() -> usize {
    unsafe extern "C" {
        static mut __sheap: u32;
        static mut _stack_start: u32;
    }

    // No `unsafe` block: `&raw const` on a `static mut` creates a pointer without reading
    // through it, which is safe. Only the dereference would need one, and there is none.
    ((&raw const _stack_start) as usize) - ((&raw const __sheap) as usize)
}

/// Deepest point core 0's stack has ever reached, in bytes.
///
/// A value at or near [`core0_span`] means the paint was consumed entirely: the true
/// requirement is unknown and at least this large.
pub fn core0_high_water() -> usize {
    unsafe extern "C" {
        static mut __sheap: u32;
        static mut _stack_start: u32;
    }

    /// What `cortex-m-rt/paint-stack` fills the region with.
    const PAINT: u32 = 0xCCCC_CCCC;

    // SAFETY: reads only, and only of the region cortex-m-rt painted. A torn read against a
    // word being pushed concurrently moves the answer by one frame, which does not matter
    // for a high-water estimate.
    unsafe {
        let bottom = (&raw const __sheap) as usize;
        let top = (&raw const _stack_start) as usize;
        let span = top - bottom;

        let mut untouched = 0usize;
        while untouched < span {
            let word = core::ptr::read_volatile((bottom + untouched) as *const u32);
            if word != PAINT {
                break;
            }
            untouched += 4;
        }
        span - untouched
    }
}
