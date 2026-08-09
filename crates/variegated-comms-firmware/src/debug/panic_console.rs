//! The last thing this processor ever writes: a panic banner and backtrace, as raw
//! bytes, on the same UART0 wire the structured stream uses.
//!
//! Being on a UART rather than USB-Serial-JTAG matters most *here*, of everywhere it
//! matters. A panic is exactly when the host's view of a USB device is least reliable --
//! the device may be about to be reset by the watchdog, taking its enumeration with it --
//! and the panic banner is exactly the output you cannot afford to lose to that. A UART
//! has no enumeration to lose: the bytes are on the wire as soon as they are in the FIFO,
//! and a receiver that was already listening sees them.
//!
//! # Why this exists at all
//!
//! Task 9 set `esp-println` to `no-op`, for two good reasons that both still hold:
//! the ROM console would otherwise interleave its bytes into the COBS stream, and
//! `tx_flush` busy-waited with interrupts disabled for ~2.8 ms per 32-byte chunk on a
//! processor running Wi-Fi and BLE. The cost was that `esp-backtrace`'s only sink went
//! with it, so a panic here produced *nothing*: no backtrace, no banner, no clue.
//!
//! The bus cannot cover for that. Publishing a frame is the easy half; getting it out
//! needs [`crate::debug::usb`]'s writer task, and by the time a panic handler runs the
//! executor will never poll anything again. Whatever the panic path emits, it has to
//! emit itself, synchronously, from inside the handler.
//!
//! So this module writes plain text, and the host has been taught to display it --
//! see `variegated_debug_codec::is_text_run`. That is what turns text on this wire
//! from corruption into content, and it is why the ordinary logging path is
//! *unchanged*: `esp-println` stays `no-op`, the bus still carries every `log_*!`, and
//! nothing here runs until the machine is already dead.
//!
//! # Why blocking is correct here and nowhere else
//!
//! Every other writer in this firmware drops rather than waits, because an always-on
//! debug path that can block perturbs the timing it exists to observe -- and on this
//! chip it would take Wi-Fi, BLE and the ESPHome server down with it, since they share
//! one executor. None of that applies once the panic handler is running. There is no
//! executor left to starve, no task left to perturb, and no work the processor could
//! be doing that is worth more than the backtrace. Waiting is the right trade exactly
//! once, and this is it.
//!
//! It is still *bounded*, by [`SPIN_LIMIT`], and that is not a hedge against the
//! argument above. It is because "wait for the host" and "wait forever for a host that
//! is not there" are different things: with nothing attached, the IN endpoint fills
//! and never drains, and an unbounded spin would sit there rather than reaching the
//! halt loop. The observable difference is small -- a halted processor either way --
//! but a bounded loop is the one you can reason about, and it leaves the watchdog
//! ([`crate::watchdog`], TIMG1) free to reset the chip on schedule rather than at
//! whatever moment USB happens to unblock. Interrupts are off by the time this runs, so
//! the feeder task is not going to run again; the reset follows within
//! [`crate::watchdog::TIMEOUT`].
//!
//! # The delimiters
//!
//! The text is written between two `0x00` bytes, and both are load-bearing:
//!
//! * The **leading** one terminates whatever COBS frame was in flight when the machine
//!   died. Without it the truncated frame and the panic text arrive as one run between
//!   delimiters -- a concatenation of postcard bytes and ASCII, which the host's
//!   classifier correctly refuses as corruption, taking the backtrace down with it.
//!   With it, the half-frame is judged as the framing error it is and the text is
//!   judged as text.
//! * The **trailing** one terminates the text run so the host renders it immediately.
//!   A decoder only acts on a run when the next delimiter arrives, and the next
//!   delimiter after a panic is one a dead processor is never going to send.

use esp_hal::peripherals::UART0;
// The decoder's own per-byte predicate, imported rather than restated. These are two
// halves of one rule living in two crates, and a copy that drifted would silence the
// panic path at the far end with nothing here to show for it.
use variegated_debug_codec::is_text_byte;

/// How many times to re-check the transmit FIFO before giving up on a byte.
///
/// Generous to the point of irrelevance in normal operation: the FIFO drains at the line
/// rate whatever is on the other end, so a slot frees every ~11 µs at
/// [`crate::config::DEBUG_UART_BAUD`] and this is never approached. It stays because the
/// panic path must not be the reason the chip fails to reset -- if the transmitter is
/// wedged for a reason this code cannot see (a clock gated by whatever caused the panic,
/// flow control somehow asserted), the handler gives up and reaches its halt loop, leaving
/// the watchdog free to reset on schedule.
///
/// It is far less load-bearing than it was on USB-Serial-JTAG, where "no host attached"
/// was a permanent and entirely ordinary state in which the FIFO *never* drained. On a
/// UART there is no such state.
const SPIN_LIMIT: u32 = 2_000_000;

/// Depth of the UART transmit FIFO, and therefore the fullness threshold to wait on.
const TX_FIFO_DEPTH: u32 = 128;

/// Substituted for any byte the host's text classifier would refuse.
const REPLACEMENT: u8 = b'?';

/// A blocking, bounded writer on UART0, driven entirely through the peripheral's static
/// register accessor.
///
/// **No `esp_hal` driver is constructed, and that is deliberate.** `Uart::new` goes
/// through `PeripheralClockControl::enable`, which takes `PERIPHERAL_REF_COUNT` -- a
/// `NonReentrantMutex` whose re-entry path is itself a panic. A panic taken inside *any*
/// driver constructor (and this firmware builds a good many: UART, TimerGroup,
/// BleConnector, the Wi-Fi stack) would therefore recurse through the panic handler
/// instead of printing anything. The panic path should be the code least able to fail, so
/// it touches no lock and allocates no driver: it reads `status` and writes `fifo`
/// directly.
///
/// It also cannot use the running transport's `UartTx`, which is owned by a task and
/// borrowed by whatever was mid-write when the machine died.
///
/// The cost is one narrow window: a panic before `main` configures UART0 finds the
/// peripheral unclocked and its pins unassigned, and produces nothing. That window is the
/// top of `main`, and trading it for the removal of a recursion hazard that spans every
/// driver constructor is clearly the right way round.
pub struct PanicConsole {
    /// Set once a spin ran out. Every later write is skipped rather than re-spending the
    /// budget: a backtrace is many writes long, and if the first byte could not get out
    /// none of the rest will either.
    abandoned: bool,
}

impl PanicConsole {
    /// Start writing on UART0 regardless of who else holds it.
    ///
    /// # Safety
    ///
    /// Only sound from a panic handler, or somewhere else where it is known that no
    /// task will ever run again. Anywhere else this interleaves with the transport in
    /// [`crate::debug::uart`], which owns the same peripheral.
    pub unsafe fn seize() -> Self {
        Self { abandoned: false }
    }

    /// Whether the transmit FIFO has room for another byte.
    ///
    /// `txfifo_cnt` is the number of bytes waiting, so room exists while it is below the
    /// FIFO depth. Read directly rather than through `esp-hal`'s blocking `write`, which
    /// spins on a different and unbounded condition.
    fn has_room() -> bool {
        UART0::regs().status().read().txfifo_cnt().bits() < TX_FIFO_DEPTH as u8
    }

    /// Push one byte with no filtering at all. Every caller goes through this; only
    /// [`PanicConsole::write_delimiter`] passes it something the classifier would
    /// refuse, and that is the point of it.
    fn put(&mut self, byte: u8) {
        if self.abandoned {
            return;
        }
        let mut spins = 0u32;
        while !Self::has_room() {
            spins += 1;
            if spins >= SPIN_LIMIT {
                self.abandoned = true;
                return;
            }
            core::hint::spin_loop();
        }
        UART0::regs()
            .fifo()
            .write(|w| unsafe { w.rxfifo_rd_byte().bits(byte) });
    }

    /// Write text, substituting `?` for anything the host would refuse.
    ///
    /// The substitution is not cosmetic. `is_text_run` is all-or-nothing over a whole
    /// run, and this handler writes its banner, the `PanicInfo` and every backtrace
    /// address as *one* run between two delimiters -- so a single non-ASCII byte
    /// anywhere in a panic message would take the addresses down with it. One
    /// Unicode quote in an `expect()` string, or one accented character in a path,
    /// and the output this task exists to restore is discarded in its entirety at the
    /// far end.
    ///
    /// Substituting makes that impossible rather than unlikely, and it does it here
    /// rather than by relaxing the host rule -- which would have to admit arbitrary
    /// UTF-8 and would forfeit the structural guarantee that no frame can be read as
    /// text. A mangled character in a panic message costs nothing; the frame
    /// addresses under it are the part that has to survive.
    pub fn write_bytes(&mut self, bytes: &[u8]) {
        for &byte in bytes {
            self.put(if is_text_byte(byte) { byte } else { REPLACEMENT });
        }
    }

    /// Write a COBS delimiter, bypassing the substitution above -- `0x00` is the one
    /// byte that must reach the wire unmodified, and the one byte `write_bytes` will
    /// never emit.
    pub fn write_delimiter(&mut self) {
        self.put(0x00);
    }

    /// Wait for the transmitter to actually drain before the caller halts.
    ///
    /// On USB-Serial-JTAG this marked a part-filled packet done, because bytes sat in an
    /// endpoint buffer until something said "send". A UART needs the opposite: the bytes
    /// are already going out, but the caller is about to halt the processor and the
    /// watchdog is about to reset it, and a reset mid-frame truncates whatever is still in
    /// the FIFO. So this drains rather than flushes.
    ///
    /// Bounded by the same [`SPIN_LIMIT`], and for the same reason: the panic path must
    /// never be why the chip fails to reset.
    pub fn flush(&mut self) {
        if self.abandoned {
            return;
        }
        let mut spins = 0u32;
        while UART0::regs().status().read().txfifo_cnt().bits() > 0 {
            spins += 1;
            if spins >= SPIN_LIMIT {
                return;
            }
            core::hint::spin_loop();
        }
    }
}

impl core::fmt::Write for PanicConsole {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.write_bytes(s.as_bytes());
        // Never an error, however little got out. A `write!` in a panic handler has
        // nothing useful to do with a failure and `?` on one is a way to skip the rest
        // of the backtrace.
        Ok(())
    }
}
