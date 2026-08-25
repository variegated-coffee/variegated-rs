//! This transport's phase table, and the divider sized against it.
//!
//! # One divider, several clock rates
//!
//! All six programs share a single divider, and they do *not* all cost the same number of
//! PIO cycles per SD clock -- the `[n]` delay slots differ from phase to phase. So the
//! effective SD clock changes depending on what the bus is doing, and a 25 MHz "bus
//! frequency" means 25 MHz during a command and 20 MHz during a data read.
//!
//! This is SdFat's design and it is kept deliberately. It is also why [`divider`] is
//! defined against the *fastest* phase: sizing the divider so the 4-cycle phases hit the
//! requested frequency means every other phase comes out slower, and the bus can never
//! exceed what the caller asked for. Sizing it against an average would overclock the
//! command phase, which is the one running before the card has agreed to anything.
//!
//! The arithmetic itself lives in `variegated-rp-pio`, which takes the cycle count as a
//! parameter precisely so a transport with a phase table can pass its own. What is here is
//! the table and the two thin wrappers that bind it.

use fixed::FixedU32;
use fixed::types::extra::U8;

pub use variegated_rp_pio::clock::{phase_hz, SPEC_CEILING_HZ};

/// PIO cycles per SD clock, by phase. See the module docs for why these differ.
pub mod cycles {
    /// `cmd_rsp`, shifting a command frame out.
    pub const CMD_SEND: u32 = 4;
    /// `cmd_rsp`, hunting for the response start bit.
    pub const RSP_WAIT: u32 = 8;
    /// `cmd_rsp`, shifting a response in.
    pub const RSP_READ: u32 = 6;
    /// `rd_clk`, hunting for the data start bit on DAT0.
    pub const RD_SEARCH: u32 = 8;
    /// `rd_clk`, clocking a data block in.
    pub const RD_DATA: u32 = 5;
    /// `wr_data`, clocking a data block out.
    pub const WR_DATA: u32 = 4;
    /// `wr_resp`, clocking the CRC status token in.
    pub const WR_RESP: u32 = 10;

    /// The cheapest phase, and therefore the one that runs fastest for a given divider.
    pub const FASTEST: u32 = CMD_SEND;
}

/// The PIO clock divider that keeps *every* phase at or below `target_hz`.
///
/// Sized against [`cycles::FASTEST`], so the cheapest phase lands on the target and the
/// rest come out slower. See the module docs.
pub fn divider(clk_sys_hz: u32, target_hz: u32) -> FixedU32<U8> {
    variegated_rp_pio::clock::divider(clk_sys_hz, target_hz, cycles::FASTEST)
}

/// The fastest SD clock this transport can actually reach on this system clock.
pub fn max_sd_clock(clk_sys_hz: u32) -> u32 {
    variegated_rp_pio::clock::max_sd_clock(clk_sys_hz, cycles::FASTEST)
}

#[cfg(test)]
mod tests {
    use super::*;

    const SYS: [u32; 4] = [125_000_000, 133_000_000, 150_000_000, 200_000_000];
    const TARGETS: [u32; 5] = [400_000, 12_500_000, 20_000_000, 25_000_000, 1_000_000];

    /// The property that actually matters, and the reason the table is bound to the divider
    /// here rather than left to the caller: **no** phase may exceed the requested clock, not
    /// just the one the divider was sized against.
    #[test]
    fn no_phase_ever_runs_faster_than_requested() {
        for sys in SYS {
            for hz in TARGETS {
                let div = divider(sys, hz);
                for c in [
                    cycles::CMD_SEND,
                    cycles::RSP_WAIT,
                    cycles::RSP_READ,
                    cycles::RD_SEARCH,
                    cycles::RD_DATA,
                    cycles::WR_DATA,
                    cycles::WR_RESP,
                ] {
                    let actual = phase_hz(sys, div, c);
                    assert!(
                        actual <= hz,
                        "sys {sys} target {hz} cycles {c}: got {actual}, div {div}"
                    );
                }
            }
        }
    }

    /// Erring slow is fine; erring *very* slow means the rounding is wrong rather than
    /// conservative. The fastest phase should land within a percent of the target.
    #[test]
    fn the_fastest_phase_lands_close_to_the_target() {
        for sys in SYS {
            for hz in TARGETS {
                let div = divider(sys, hz);
                let actual = phase_hz(sys, div, cycles::FASTEST);
                assert!(
                    actual as u64 * 100 >= hz as u64 * 99,
                    "sys {sys} target {hz}: got {actual}, more than 1% slow"
                );
            }
        }
    }

    /// `FASTEST` must really be the cheapest entry, or the divider is sized against the
    /// wrong phase and the guarantee above silently stops holding.
    #[test]
    fn fastest_is_the_cheapest_phase_in_the_table() {
        for c in [
            cycles::CMD_SEND,
            cycles::RSP_WAIT,
            cycles::RSP_READ,
            cycles::RD_SEARCH,
            cycles::RD_DATA,
            cycles::WR_DATA,
            cycles::WR_RESP,
        ] {
            assert!(cycles::FASTEST <= c, "{c} is cheaper than FASTEST");
        }
    }

    /// Both chips at their default clocks clear the ceiling; a system clock dropped for
    /// power reports honestly rather than over-promising.
    #[test]
    fn both_chips_reach_the_ceiling_at_their_default_clocks() {
        assert_eq!(max_sd_clock(125_000_000), SPEC_CEILING_HZ); // RP2040, 31.25 MHz raw
        assert_eq!(max_sd_clock(150_000_000), SPEC_CEILING_HZ); // RP2350, 37.5 MHz raw
        assert_eq!(max_sd_clock(48_000_000), 12_000_000);
    }

    /// The identification divider, pinned because it is the one with an external reference
    /// to check against.
    ///
    /// SdFat computes `ceil(clk_sys / (4 * 400 kHz))` into an integer-valued float, so at
    /// 125 MHz it uses 79 and clocks at 395.6 kHz. PIO's divider is 24.8 fixed point and
    /// `Config::clock_divider` carries the fraction, so we can use 78.125 and land on
    /// 400 kHz exactly. Faster than SdFat here, but still not *over* the ceiling, which is
    /// the only thing the specification cares about.
    #[test]
    fn the_identification_divider_uses_the_fraction_sdfat_rounds_away() {
        let div = divider(125_000_000, 400_000);
        assert_eq!(div, FixedU32::<U8>::from_num(78.125));
        assert_eq!(phase_hz(125_000_000, div, cycles::CMD_SEND), 400_000);
    }
}
