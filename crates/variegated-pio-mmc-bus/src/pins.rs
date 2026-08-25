//! The six GPIOs, claimed and conditioned.

use embassy_rp::Peri;
use embassy_rp::gpio::{Drive, Pull, SlewRate};
use embassy_rp::pio::{Common, Instance, Pin, PioPin};

use crate::window;

/// CLK, CMD and DAT0..DAT3, after `make_pio_pin` and the pad setup they need.
pub(crate) struct PioPins<'d, P: Instance> {
    pub clk: Pin<'d, P>,
    pub cmd: Pin<'d, P>,
    pub dat: [Pin<'d, P>; 4],
    /// The PIO block's `GPIOBASE` window, 0 or 16. Always 0 on RP2040.
    pub gpio_base: u8,
}

impl<'d, P: Instance> PioPins<'d, P> {
    /// Claim the pins and condition the pads.
    ///
    /// # Panics
    ///
    /// If DAT0..DAT3 are not four consecutive ascending GPIOs, or if the six pins do not
    /// all fit in one `GPIOBASE` window. Both are wiring mistakes that cannot be
    /// recovered from at runtime, and both otherwise surface as a panic deep inside
    /// embassy's `set_config` that says nothing about SD cards.
    pub fn new(
        common: &mut Common<'d, P>,
        clk: Peri<'d, impl PioPin>,
        cmd: Peri<'d, impl PioPin>,
        dat0: Peri<'d, impl PioPin>,
        dat1: Peri<'d, impl PioPin>,
        dat2: Peri<'d, impl PioPin>,
        dat3: Peri<'d, impl PioPin>,
    ) -> Self {
        let mut clk = common.make_pio_pin(clk);
        let mut cmd = common.make_pio_pin(cmd);
        let mut dat = [
            common.make_pio_pin(dat0),
            common.make_pio_pin(dat1),
            common.make_pio_pin(dat2),
            common.make_pio_pin(dat3),
        ];

        // `in pins, 4` and `out pins, 4` address a four-pin window based at DAT0, so the
        // order is not a convention -- DAT1 on a lower GPIO than DAT0 would silently
        // transpose two of the four data lines, which reads as a CRC failure on every
        // block rather than as a wiring error.
        for (i, p) in dat.iter().enumerate() {
            assert!(
                p.pin() == dat[0].pin() + i as u8,
                "DAT0..DAT3 must be four consecutive ascending GPIOs"
            );
        }

        // RP2350 gives each PIO block a 32-GPIO window at either 0 or 16, and
        // `set_config` picks it per config from that config's own pins. This driver
        // applies six different configs across two state machines over its life; if they
        // did not all agree, they would fight over one register. Compute it once here and
        // reject anything that cannot work, rather than letting the fifth config panic
        // mid-transfer. See `crate::window` for the arithmetic and its tests.
        let pins = [clk.pin(), cmd.pin(), dat[0].pin(), dat[1].pin(), dat[2].pin(), dat[3].pin()];
        let gpio_base = window::gpio_base(&pins)
            .expect("SD pins must all be below GPIO 32 or all at GPIO 16 and above");

        // CLK is the only pin driven hard and fast: it is the one whose edges every other
        // signal is timed against, and the only one that ever runs at the full bus rate
        // with nothing on the other end acknowledging it.
        clk.set_drive_strength(Drive::_8mA);
        clk.set_slew_rate(SlewRate::Fast);

        // `make_pio_pin` clears both pulls, so without this CMD and DAT float during
        // every turnaround -- the window between the host releasing the line and the card
        // driving it. These internal pull-ups are roughly 50 kOhm and are a floor, not a
        // substitute for external 10-50 kOhm ones; at 25 MHz the internal ones alone will
        // not pull a loaded line high inside a bit period.
        cmd.set_pull(Pull::Up);
        cmd.set_drive_strength(Drive::_8mA);
        for p in dat.iter_mut() {
            p.set_pull(Pull::Up);
            p.set_drive_strength(Drive::_8mA);
        }

        // Bypass the PIO input synchronisers on all six.
        //
        // Not an optimisation. The synchroniser costs two `clk_sys` cycles on every input,
        // and `rd_data` is a pure clock follower -- it samples DAT on an edge it observes
        // on CLK through that same delay. At a 20 MHz data clock a bit period is about
        // seven system cycles, so two cycles of skew is a third of the eye. Reads then
        // fail as data CRC errors, only at the top of the frequency range, which looks
        // exactly like a signal-integrity problem in the wiring.
        //
        // Done through `Common` with a mask we build ourselves rather than through
        // `Pin::set_input_sync_bypass`, which is unusable above GPIO 31: it computes
        // `1 << pin` from the *absolute* number into a `u32`. See
        // `window::sync_bypass_mask` for what that does on a card wired to the high bank.
        let mask = window::sync_bypass_mask(&pins, gpio_base);
        common.set_input_sync_bypass(mask, mask);

        // Every window-relative number this driver computes, in one line.
        //
        // Here rather than left to be inferred because all three of the five-bit fields --
        // `WAIT`'s GPIO index, `INPUT_SYNC_BYPASS` and `EXECCTRL.JMP_PIN` -- fail the same
        // silent way when they are wrong: the block watches a pin nobody chose, and the
        // symptom is a timeout that names neither the pin nor the window. A card that will
        // not come up is diagnosed from this line first.
        mmc_trace!(
            "pio-mmc: gpiobase {=u8}, clk {=u8}->{=u8}, cmd {=u8}->{=u8}, dat0 {=u8}->{=u8}, bypass {=u32:#010x}",
            gpio_base,
            clk.pin(),
            window::relative(clk.pin(), gpio_base),
            cmd.pin(),
            window::relative(cmd.pin(), gpio_base),
            dat[0].pin(),
            window::relative(dat[0].pin(), gpio_base),
            mask
        );

        Self { clk, cmd, dat, gpio_base }
    }

    /// The CLK pin as `wait gpio` needs it: relative to the block's `GPIOBASE`.
    ///
    /// Every other pin reference in a PIO program is relative to a base in `PINCTRL`, but
    /// `wait gpio` names a GPIO directly -- which on RP2350 means "directly within the
    /// window", hence the subtraction. The instruction's index field is five bits, so on
    /// a 48-GPIO part the absolute number does not even fit; see [`crate::window`].
    pub fn clk_wait_index(&self) -> u8 {
        window::relative(self.clk.pin(), self.gpio_base)
    }

    /// DAT0..DAT3 as `set_pin_dirs` and the pin-list config setters want them.
    pub fn dat_refs(&self) -> [&Pin<'d, P>; 4] {
        [&self.dat[0], &self.dat[1], &self.dat[2], &self.dat[3]]
    }
}
