//! How deep the main task's stack has actually gone.
//!
//! # Why this exists
//!
//! `.stack` on this chip is not a size anyone chose. `esp-hal`'s `ld/sections/stack.x`
//! defines it as whatever is left of RWDATA after `.data` and `.bss`, and `esp_rtos::start`
//! hands exactly that span to the main task. So it moves whenever an unrelated static
//! changes size, and it has twice been changed by accident:
//!
//! * At 90,144 bytes the main task overflowed inside `esp_radio::wifi::new()`. `esp-rtos`
//!   caught it once (`Stack pointer: 40857b20, Task stack range: 40857d78 ..=`, 600 bytes
//!   past the floor); the other attempts presented as a load access fault in
//!   `chip_v7_set_chan` with a different address per build, which is the overrun landing on
//!   whatever the linker had put at the top of `.bss`.
//! * At 97,616 bytes it does not.
//!
//! **That is the entire state of knowledge, and it is not enough to size anything with.**
//! Nobody has ever measured what the main task actually needs; ~90 kB of call depth for a
//! radio bring-up is implausible enough that a single oversized frame is the likelier
//! explanation, and the difference decides whether tens of kilobytes can be handed to the
//! heap -- which has *also* run out, at 121,552 bytes of 122,880 during a provisioning
//! reconnect. The two come from one pool and both edges have now been hit.
//!
//! The heap has had `esp_alloc::HEAP.stats().max_usage` reported at 1 Hz for a while, which
//! is why that failure was visible in the log before it was fatal. This is the other half.

/// Written across the unused stack before the executor starts, so depth can be read back.
///
/// Not `0x00`: a freshly-pushed frame is largely zeroes, so zero cannot distinguish
/// "never touched" from "touched and happens to be zero". The application processor's
/// `CORE1_STACK_PAINT` is `0xC5` for the same reason.
const PAINT: u32 = 0xC5C5_C5C5;

/// Bytes at the bottom of the stack left unpainted, to clear `esp-rtos`'s guard word.
///
/// `esp_rtos::start` reads `__stack_chk_guard`, which the linker script places at
/// `_stack_end_cpu0 + ESP_HAL_CONFIG_STACK_GUARD_OFFSET` (default 60), remembers its value,
/// and compares that location on every context switch. Painting over it would work by
/// accident -- the value read at start-up would be the paint -- but it would also silently
/// disable `-Zstack-protector` if this firmware ever enabled it, and it would make the
/// guard's value depend on this module. 64 clears it and stays 4-aligned.
const GUARD_SKIP: usize = 64;

/// Bytes below the stack pointer left unpainted when [`paint`] runs.
///
/// **This is the safety margin on writing to your own stack.** Everything from here up is
/// live: `paint`'s own frame, its caller's, and any interrupt that lands mid-loop, since
/// RISC-V interrupts run on the current stack. Painting into that corrupts the machine in
/// the least debuggable way available.
///
/// 8 kB is far more than the few hundred bytes actually at risk, and it costs nothing that
/// matters: the unpainted top means [`high_water`] cannot report a figure *below* ~8 kB, and
/// the measurement of interest is two orders of magnitude larger than that.
const HEADROOM: usize = 8 * 1024;

fn bounds() -> (usize, usize) {
    unsafe extern "C" {
        static _stack_end_cpu0: u32;
        static _stack_start_cpu0: u32;
    }
    // The linker script names these the other way round from how they read: `_stack_end` is
    // the *low* address, because the stack grows down and ends there.
    //
    // No `unsafe` needed: taking a raw pointer to an extern static is safe in edition 2024,
    // and only dereferencing it is not.
    (
        (&raw const _stack_end_cpu0) as usize,
        (&raw const _stack_start_cpu0) as usize,
    )
}

/// Total bytes the main task's stack can occupy.
///
/// A function rather than a constant because both bounds are linker symbols. This is also
/// the span `esp_rtos` range-checks against, so a [`high_water`] approaching it means the
/// overflow assert is about to fire.
pub fn span() -> usize {
    let (bottom, top) = bounds();
    top - bottom
}

/// Fill the unused part of the main stack with [`PAINT`].
///
/// **Call this as the first statement in `main`, and nowhere else.** It writes everything
/// between the guard word and [`HEADROOM`] below the current stack pointer, which is only
/// safe while that region is genuinely dead -- before the executor starts, before any task
/// is spawned, and before anything below this frame has run.
pub fn paint() {
    let (bottom, top) = bounds();

    // The stack pointer, read directly rather than approximated by the address of a local:
    // a local's address depends on frame layout the optimiser is free to change, and being
    // wrong in the unsafe direction here corrupts the caller's frame.
    let sp: usize;
    // SAFETY: reads a register.
    unsafe { core::arch::asm!("mv {}, sp", out(reg) sp, options(nomem, nostack, preserves_flags)) };

    // **Refuse to paint unless we are demonstrably on the stack we think we are.**
    //
    // Everything below rests on `sp` pointing into `.stack`. If `#[esp_rtos::main]` ever
    // starts running its body on a task stack of its own -- allocated from the heap, as
    // `esp-rtos` does for every other task -- then `sp` would be somewhere else entirely and
    // `sp - HEADROOM` would be an address in the middle of unrelated memory. That is a
    // silent whole-machine corruption, and it is the kind of thing a dependency bump does.
    // Cheap to rule out; impossible to debug afterwards.
    if sp <= bottom || sp > top {
        return;
    }

    let start = bottom + GUARD_SKIP;
    let end = sp.saturating_sub(HEADROOM);
    if end <= start {
        // The stack is already deeper than the headroom allows for, which means this is not
        // being called where its doc comment says. Paint nothing rather than scribble on a
        // live frame; `high_water` will report the full span and say it learned nothing.
        return;
    }

    // SAFETY: `start..end` is below the current frame by at least `HEADROOM` and above the
    // guard word, and nothing else exists yet to be holding a reference into it.
    unsafe {
        let mut addr = start;
        while addr < end {
            core::ptr::write_volatile(addr as *mut u32, PAINT);
            addr += 4;
        }
    }
}

/// Deepest point the main task's stack has reached, in bytes.
///
/// The stack grows down, so surviving paint collects at the bottom and the high-water mark
/// is the distance from the lowest surviving word up to the top.
///
/// Two readings mean "unknown", not "fine":
///
/// * A value at or near [`span`] means the paint was consumed to the guard word. The true
///   requirement is at least this and the stack has probably already overflowed.
/// * A value near [`HEADROOM`] means the stack never entered the painted region at all,
///   which on this firmware would mean [`paint`] did not run.
pub fn high_water() -> usize {
    let (bottom, top) = bounds();
    let mut addr = bottom + GUARD_SKIP;

    // Volatile so the loop is not hoisted or vectorised into something that reads words the
    // stack is concurrently writing. A torn read moves the answer by one frame, which does
    // not matter for a high-water estimate.
    //
    // The scan is cheapest exactly when it matters: it walks *surviving* paint, so a stack
    // that has gone deep leaves little to walk. The worst case -- an idle stack, the whole
    // span painted -- is ~24k word reads, which is microseconds at 1 Hz.
    unsafe {
        while addr < top && core::ptr::read_volatile(addr as *const u32) == PAINT {
            addr += 4;
        }
    }

    top - addr
}
