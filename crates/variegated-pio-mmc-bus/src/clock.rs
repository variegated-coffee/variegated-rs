//! Turning a target SD clock into a PIO clock divider.
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

use fixed::FixedU32;
use fixed::types::extra::U8;

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

/// The smallest divider `set_config` accepts, in raw 24.8 fixed-point bits.
const MIN_RAW: u64 = 1 << 8;
/// The largest divider `set_config` accepts (65536.0), in raw 24.8 fixed-point bits.
const MAX_RAW: u64 = 65_536 << 8;

/// The PIO clock divider that keeps the SD clock at or below `target_hz`.
///
/// Rounded **up** to the next 1/256, never down. The difference matters at the extremes:
/// at 400 kHz during identification the specification gives a hard ceiling, and rounding
/// down would sit just above it on most system clocks. Erring slow costs a fraction of a
/// percent of throughput and cannot break anything.
///
/// Clamped to the range `set_config` will accept, so a nonsensical `target_hz` produces a
/// slow bus rather than a panic deep inside embassy.
pub fn divider(clk_sys_hz: u32, target_hz: u32) -> FixedU32<U8> {
    // `clk_sys * 256 / 4` folded into one multiply. In u64 because a 150 MHz system clock
    // times 64 is 9.6e9, which a u32 cannot hold -- and the overflow would wrap to a
    // *small* divider, i.e. an overclocked bus, which is the worst possible failure here.
    let numerator = (clk_sys_hz as u64) * (256 / cycles::FASTEST) as u64;
    let raw = numerator.div_ceil(target_hz.max(1) as u64).clamp(MIN_RAW, MAX_RAW);
    FixedU32::<U8>::from_bits(raw as u32)
}

/// The ceiling this crate will let the `sdio` crate ask for, in Hz.
///
/// Not because the PIO cannot go faster, but because above 25 MHz `sdio::sd::Card::acquire`
/// switches the card to a high-speed signalling mode and then calls `tune_bus`, and this
/// bus implements neither. A card left in default-speed mode while the host clocks it at
/// 40 MHz is out of specification even when it happens to work.
pub const SPEC_CEILING_HZ: u32 = 25_000_000;

/// The fastest SD clock actually reachable on this system clock.
///
/// **This is a function of `clk_sys`, and the two chips in this family do not run at the
/// same speed** -- RP2040 boots at 125 MHz and RP2350 at 150 MHz, and firmware may change
/// either. The PIO divider cannot go below 1.0, so the cheapest phase costs a hard
/// [`cycles::FASTEST`] system clocks per SD clock and nothing can be done about it.
///
/// Both chips at their default clocks clear [`SPEC_CEILING_HZ`] comfortably (31.25 MHz
/// and 37.5 MHz respectively), so in practice this returns the ceiling. It matters for a
/// system clock that has been dropped for power: reporting 25 MHz from a 48 MHz `clk_sys`
/// would have the card negotiate a speed the host then silently fails to produce.
pub fn max_sd_clock(clk_sys_hz: u32) -> u32 {
    (clk_sys_hz / cycles::FASTEST).min(SPEC_CEILING_HZ)
}

/// The SD clock a phase actually runs at, given a divider.
///
/// Reported rather than used: it exists so `trace` output can say what the bus is really
/// doing, and so the tests can assert the ceiling is respected.
pub fn phase_hz(clk_sys_hz: u32, div: FixedU32<U8>, cycles_per_bit: u32) -> u32 {
    let raw = div.to_bits() as u64;
    if raw == 0 {
        return 0;
    }
    // hz = clk_sys / (cycles * div), with div carried as raw/256.
    (((clk_sys_hz as u64) * 256) / (cycles_per_bit as u64 * raw)) as u32
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every system clock this family runs at, against every frequency the `sdio` crate
    /// will ask for: 400 kHz during identification, then whatever the caller requested up
    /// to `supports_frequency()`.
    const SYS: [u32; 4] = [125_000_000, 133_000_000, 150_000_000, 200_000_000];
    const TARGETS: [u32; 5] = [400_000, 12_500_000, 20_000_000, 25_000_000, 1_000_000];

    /// The property that actually matters: no phase may ever exceed the requested clock.
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

    /// `set_config` asserts both ends of this range; tripping it is a panic inside
    /// embassy with no mention of SD cards, so the clamp belongs here.
    #[test]
    fn the_divider_stays_within_what_set_config_accepts() {
        for sys in SYS {
            for hz in [1u32, 400_000, 25_000_000, u32::MAX] {
                let div = divider(sys, hz);
                assert!(div >= FixedU32::<U8>::from_num(1), "{sys}/{hz} -> {div}");
                assert!(div <= FixedU32::<U8>::from_num(65536), "{sys}/{hz} -> {div}");
            }
        }
        // A zero target must not divide by zero on the way to the clamp.
        assert_eq!(divider(150_000_000, 0), FixedU32::<U8>::from_num(65536));
    }

    /// The two chips in this family do not share a default system clock, and the reachable
    /// SD clock follows `clk_sys` directly. Both defaults must still clear the ceiling, or
    /// `supports_frequency()` would report a speed the host cannot actually produce.
    #[test]
    fn both_chips_reach_the_ceiling_at_their_default_clocks() {
        assert_eq!(max_sd_clock(125_000_000), SPEC_CEILING_HZ); // RP2040, 31.25 MHz raw
        assert_eq!(max_sd_clock(150_000_000), SPEC_CEILING_HZ); // RP2350, 37.5 MHz raw

        // A system clock dropped for power reports honestly rather than over-promising.
        assert_eq!(max_sd_clock(48_000_000), 12_000_000);
        // ...and the divider then bottoms out at 1.0 rather than going below it.
        assert_eq!(divider(48_000_000, SPEC_CEILING_HZ), FixedU32::<U8>::from_num(1));
    }

    /// The identification divider, pinned because it is the one with an external
    /// reference to check against.
    ///
    /// SdFat computes `ceil(clk_sys / (4 * 400 kHz))` into an integer-valued float, so at
    /// 125 MHz it uses 79 and clocks at 395.6 kHz. PIO's divider is 24.8 fixed point and
    /// `Config::clock_divider` carries the fraction, so we can use 78.125 and land on
    /// 400 kHz exactly. Faster than SdFat here, but still not *over* the ceiling, which
    /// is the only thing the specification cares about.
    #[test]
    fn the_identification_divider_uses_the_fraction_sdfat_rounds_away() {
        let div = divider(125_000_000, 400_000);
        assert_eq!(div, FixedU32::<U8>::from_num(78.125));
        assert_eq!(phase_hz(125_000_000, div, cycles::CMD_SEND), 400_000);
    }
}
