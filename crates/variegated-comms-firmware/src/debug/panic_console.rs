//! The last thing this processor ever writes: a panic banner and backtrace, as raw
//! bytes, on the same USB-Serial-JTAG wire the structured stream uses.
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
//! but a bounded loop is the one you can reason about, and it leaves the RTC watchdog
//! free to reset the chip on schedule rather than at whatever moment USB happens to
//! unblock.
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

use esp_hal::peripherals::USB_DEVICE;
use esp_hal::usb_serial_jtag::{UsbSerialJtag, UsbSerialJtagTx};
use esp_hal::Blocking;

/// How many times to re-check the IN endpoint before giving up on a byte.
///
/// A host that is draining frees a 64-byte buffer in well under a millisecond, so any
/// plausible value clears this by orders of magnitude; a host that is not attached
/// never frees it at all, so no value is too small. At a 160 MHz core clock this is a
/// few tens of milliseconds of spinning before the handler concludes nobody is
/// listening and goes to its halt loop -- long enough not to truncate a backtrace on a
/// slow host, short enough not to be the reason the chip fails to reset.
const SPIN_LIMIT: u32 = 2_000_000;

/// The IN endpoint FIFO packet size, and the unit the hardware sends in.
const PACKET: usize = 64;

/// A blocking, bounded writer on USB-Serial-JTAG, taken by force.
///
/// `steal()` is sound here for a reason specific to this call site and no other: the
/// only other owner is [`crate::debug::usb`]'s writer task, and a panic handler runs
/// after the executor has stopped scheduling. There is no concurrent user to race
/// with because there is no concurrency left.
pub struct PanicConsole {
    tx: UsbSerialJtagTx<'static, Blocking>,
    /// Bytes written into the current packet, so it can be marked done at 64 rather
    /// than relying on a flush that would then block on an unattached host.
    in_packet: usize,
    /// Set once a spin ran out. Every later write is skipped rather than re-spending
    /// the budget: with no host attached the first `SPIN_LIMIT` is diagnostic and
    /// every one after it is dead time, and a backtrace is many writes long.
    abandoned: bool,
}

impl PanicConsole {
    /// Take the USB-Serial-JTAG peripheral away from whoever had it.
    ///
    /// # Safety
    ///
    /// Only sound from a panic handler, or somewhere else where it is known that no
    /// task will ever run again. Anywhere else this aliases the transport in
    /// [`crate::debug::usb`].
    pub unsafe fn steal() -> Self {
        let device = unsafe { USB_DEVICE::steal() };
        let (_rx, tx) = UsbSerialJtag::new(device).split();
        Self { tx, in_packet: 0, abandoned: false }
    }

    /// Hand the current packet to the hardware.
    ///
    /// Through the peripheral's static register accessor, the same way
    /// [`crate::debug::usb`] reads `serial_in_ep_data_free`, because `esp-hal`'s own
    /// `flush_tx` sets this bit and then spins on an *unbounded* condition -- which is
    /// the one thing this module must not do.
    fn mark_packet_done() {
        USB_DEVICE::regs()
            .ep1_conf()
            .modify(|_, w| w.wr_done().set_bit());
    }

    pub fn write_bytes(&mut self, bytes: &[u8]) {
        for &byte in bytes {
            if self.abandoned {
                return;
            }
            // `write_byte_nb` consults `serial_in_ep_data_free` itself and refuses
            // rather than writing into a full FIFO -- unlike the blocking `write`,
            // which spins, and unlike `write_async`, which pushes regardless and lets
            // the hardware discard the bytes. Retrying it *is* the bounded wait.
            let mut spins = 0u32;
            loop {
                match self.tx.write_byte_nb(byte) {
                    Ok(()) => break,
                    Err(_) => {
                        spins += 1;
                        if spins >= SPIN_LIMIT {
                            self.abandoned = true;
                            return;
                        }
                        core::hint::spin_loop();
                    }
                }
            }
            self.in_packet += 1;
            if self.in_packet == PACKET {
                self.in_packet = 0;
                Self::mark_packet_done();
            }
        }
    }

    /// Push out a part-filled packet. Called once, at the very end: calling it per
    /// line would emit a short USB packet per line for no benefit.
    pub fn flush(&mut self) {
        if self.in_packet > 0 {
            self.in_packet = 0;
            Self::mark_packet_done();
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
