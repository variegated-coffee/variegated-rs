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

use esp_hal::peripherals::USB_DEVICE;
// The decoder's own per-byte predicate, imported rather than restated. These are two
// halves of one rule living in two crates, and a copy that drifted would silence the
// panic path at the far end with nothing here to show for it.
use variegated_debug_codec::is_text_byte;

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

/// Substituted for any byte the host's text classifier would refuse.
const REPLACEMENT: u8 = b'?';

/// A blocking, bounded writer on USB-Serial-JTAG, driven entirely through the
/// peripheral's static register accessor.
///
/// **No `esp_hal` driver is constructed, and that is deliberate.**
/// `UsbSerialJtag::new` goes through `PeripheralClockControl::enable`, which takes
/// `PERIPHERAL_REF_COUNT` -- a `NonReentrantMutex` whose re-entry path is itself a
/// panic. A panic taken inside *any* driver constructor (and this firmware builds a
/// good many: UART, TimerGroup, BleConnector, the Wi-Fi stack) would therefore recurse
/// through the panic handler instead of printing anything. The panic path should be
/// the code least able to fail, so it touches no lock and allocates no driver: it
/// reads and writes `ep1`/`ep1_conf` directly, exactly as
/// [`crate::debug::usb::run`] already reads `serial_in_ep_data_free`.
///
/// The cost is one narrow window: a panic before `main` reaches
/// `UsbSerialJtag::new(peripherals.USB_DEVICE)` finds the peripheral clock ungated and
/// produces nothing. That window is the top of `main` -- `esp_hal::init`, the logger
/// install, the heap allocators and the channel `init`s -- and trading it for the
/// removal of a recursion hazard that spans every driver constructor in the firmware
/// is clearly the right way round.
pub struct PanicConsole {
    /// Bytes written into the current packet, so it can be marked done at 64 rather
    /// than relying on a flush that would then block on an unattached host.
    in_packet: usize,
    /// Set once a spin ran out. Every later write is skipped rather than re-spending
    /// the budget: with no host attached the first `SPIN_LIMIT` is diagnostic and
    /// every one after it is dead time, and a backtrace is many writes long.
    abandoned: bool,
}

impl PanicConsole {
    /// Start writing on USB-Serial-JTAG regardless of who else holds it.
    ///
    /// # Safety
    ///
    /// Only sound from a panic handler, or somewhere else where it is known that no
    /// task will ever run again. Anywhere else this interleaves with the transport in
    /// [`crate::debug::usb`], which owns the same endpoint.
    pub unsafe fn seize() -> Self {
        Self { in_packet: 0, abandoned: false }
    }

    /// Whether the IN endpoint has room for another byte -- the same
    /// `serial_in_ep_data_free` bit [`crate::debug::usb::run`] consults, and the
    /// reason neither path can use `esp-hal`'s blocking `write`, which spins on a
    /// different and unbounded condition.
    fn has_room() -> bool {
        USB_DEVICE::regs()
            .ep1_conf()
            .read()
            .serial_in_ep_data_free()
            .bit_is_set()
    }

    /// Hand the current packet to the hardware.
    fn mark_packet_done() {
        USB_DEVICE::regs()
            .ep1_conf()
            .modify(|_, w| w.wr_done().set_bit());
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
        USB_DEVICE::regs()
            .ep1()
            .write(|w| unsafe { w.rdwr_byte().bits(byte) });
        self.in_packet += 1;
        if self.in_packet == PACKET {
            self.in_packet = 0;
            Self::mark_packet_done();
        }
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
