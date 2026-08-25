//! The RP2350 `GPIOBASE` window, and pin numbers relative to it.
//!
//! A PIO block sees 32 GPIOs at a time. On RP2040 those are always GPIO 0..31 and none of
//! this matters. On RP2350B there are 48, and `GPIOBASE` selects whether the block's view
//! starts at 0 or at 16 -- the datasheet describes it as relocating "GPIO 0, from PIO's
//! point of view". Everything inside the block is then numbered in *that* view: the
//! `PINCTRL` bases, the `INPUT_SYNC_BYPASS` bits, and the five-bit GPIO index in a `WAIT`
//! instruction.
//!
//! That five-bit field is the reason this has to be right rather than merely plausible.
//! It can only encode 0..31, so if it were absolute, GPIO 32..47 would be permanently
//! unreachable by `WAIT` -- `GPIOBASE` would relocate the pin *bases* but leave a hole
//! exactly where it was introduced to help. Relative is the only reading under which the
//! feature is coherent.
//!
//! Pure arithmetic, kept out of `pins` so it can be tested on a host. It is the kind of
//! code that is correct on every board but one.

/// Which `GPIOBASE` a set of pins requires, or `None` if no single window holds them.
///
/// Mirrors embassy's own arithmetic and tie-break in `StateMachine::set_config`, which
/// recomputes this per config from that config's pins. This driver applies six configs
/// across two state machines, so they must all agree or they would fight over one
/// register -- computing it once up front is what guarantees that.
///
/// The tie-break matters: pins entirely within 16..31 satisfy both windows, and embassy
/// picks 0. Picking 16 here instead would produce configs that disagree with the ones
/// embassy derives, which is worse than either choice on its own.
pub const fn gpio_base(pins: &[u8]) -> Option<u8> {
    let mut low_ok = true;
    let mut high_ok = true;
    let mut i = 0;
    while i < pins.len() {
        let p = pins[i];
        if p >= 32 {
            low_ok = false;
        }
        if p < 16 {
            high_ok = false;
        }
        i += 1;
    }
    if low_ok {
        Some(0)
    } else if high_ok {
        Some(16)
    } else {
        None
    }
}

/// A pin number as the PIO block sees it.
///
/// What `WAIT`'s five-bit GPIO index and the `INPUT_SYNC_BYPASS` bit position are both
/// counted in.
pub const fn relative(pin: u8, base: u8) -> u8 {
    pin - base
}

/// `INPUT_SYNC_BYPASS` bits for a set of pins, in the block's own numbering.
///
/// Deliberately *not* `Pin::set_input_sync_bypass`, and this is not a stylistic
/// preference. That helper computes `1 << pin` from the **absolute** GPIO number into a
/// `u32`, so on a board with the card above GPIO 31 it is a shift overflow: a panic in a
/// debug build, and on release ARM masks the shift count to five bits, so `1 << 41`
/// silently becomes `1 << 9` and the synchroniser is bypassed on an unrelated pin while
/// the ones that need it keep their two cycles of delay.
///
/// The symptom would be data CRC failures at the top of the frequency range on a board
/// where the bypass had apparently already been applied -- which is precisely the
/// conclusion that sends someone off to re-check their wiring.
pub const fn sync_bypass_mask(pins: &[u8], base: u8) -> u32 {
    let mut mask = 0u32;
    let mut i = 0;
    while i < pins.len() {
        mask |= 1u32 << relative(pins[i], base);
        i += 1;
    }
    mask
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_low_bank_card_uses_base_zero() {
        // The common case: everything under GPIO 32.
        assert_eq!(gpio_base(&[10, 11, 12, 13, 14, 15]), Some(0));
        assert_eq!(gpio_base(&[0, 1, 2, 3, 4, 5]), Some(0));
    }

    /// The pinout this was written for: CLK 41, CMD 42, DAT0..3 on 43..46.
    #[test]
    fn a_high_bank_card_uses_base_sixteen() {
        let pins = [41u8, 42, 43, 44, 45, 46];
        assert_eq!(gpio_base(&pins), Some(16));

        // CLK's WAIT index has to fit the instruction's five-bit field, which is the
        // whole reason the window exists -- the absolute 41 does not fit, so a relative
        // reading is not a preference here but the only one that can work.
        let clk = relative(41, 16);
        assert_eq!(clk, 25);
        assert!(clk < 32, "a relative CLK index must fit five bits");
    }

    #[test]
    fn pins_straddling_the_windows_have_no_base() {
        // GPIO 15 needs base 0; GPIO 32 needs base 16. Nothing holds both.
        assert_eq!(gpio_base(&[15, 32]), None);
        assert_eq!(gpio_base(&[0, 47]), None);
    }

    /// Pins entirely inside 16..31 fit either window, and the tie-break must match
    /// embassy's or the six configs would disagree about one register.
    #[test]
    fn the_overlap_breaks_the_tie_towards_zero() {
        assert_eq!(gpio_base(&[16, 17, 18, 19, 20, 21]), Some(0));
        assert_eq!(gpio_base(&[31]), Some(0));
    }

    /// The bug this module exists to prevent.
    #[test]
    fn the_sync_bypass_mask_is_window_relative_and_never_overflows() {
        let pins = [41u8, 42, 43, 44, 45, 46];
        let mask = sync_bypass_mask(&pins, 16);

        // Bits 25..=30 -- 41-16 through 46-16.
        assert_eq!(mask, 0x7E00_0000);
        assert_eq!(mask.count_ones(), 6, "one bit per pin");
        for p in pins {
            assert_ne!(mask & (1 << (p - 16)), 0, "GPIO {p} missing from the mask");
        }

        // What the absolute computation would have produced on release ARM, where the
        // shift count is masked to five bits: bits 9..14, six entirely unrelated pins.
        let broken: u32 = pins.iter().fold(0, |m, &p| m | 1u32 << (p % 32));
        assert_ne!(mask, broken);
        assert_eq!(mask & broken, 0, "the wrong mask does not even overlap the right one");
    }

    /// `EXECCTRL.JMP_PIN` is the third register with the five-bit trap, and the one that
    /// actually stopped a card coming up.
    ///
    /// embassy's `Config::set_jmp_pin` stores `pin.pin()` -- the *absolute* number -- and
    /// `set_config` subtracts the `GPIOBASE` shift from the `PINCTRL` bases but not from
    /// this. `rp_pac` then masks it to five bits on write, so CMD on GPIO 42 is silently
    /// recorded as 10 and `jmp PIN` tests some entirely different pin.
    #[test]
    fn the_jmp_pin_index_is_window_relative_and_fits_five_bits() {
        let base = 16;

        for (pin, want) in [(42u8, 26u8), (43, 27)] {
            let relative = relative(pin, base);
            assert_eq!(relative, want);
            assert!(relative < 32, "a JMP_PIN index must fit the five-bit field");

            // What embassy stores, and what the hardware keeps of it. Not merely a
            // different pin -- a different pin that is *in use on this board*: 42 lands on
            // 10 and 43 on 11, which under a base-16 window are the card-detect line and
            // the settings flash's chip select.
            let embassy_absolute = pin;
            assert_eq!(embassy_absolute & 0x1F, pin - 32);
            assert_ne!(
                embassy_absolute & 0x1F,
                relative,
                "the truncated absolute index must not be mistaken for the relative one"
            );
        }
    }

    #[test]
    fn a_low_bank_mask_is_unchanged_by_the_window() {
        let pins = [10u8, 11, 12, 13, 14, 15];
        assert_eq!(sync_bypass_mask(&pins, 0), 0b1111_1100_0000_0000);
        for p in pins {
            assert_ne!(sync_bypass_mask(&pins, 0) & (1 << p), 0);
        }
    }
}
