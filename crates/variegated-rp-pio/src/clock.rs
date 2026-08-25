//! Turning a target bit rate into a PIO clock divider.
//!
//! # Cycles per bit is a parameter, and that is not incidental
//!
//! A PIO program's cost in cycles per bit is set by its instruction count and its `[n]`
//! delay slots, and different programs differ. A mode-0 SPI master is a flat **4 cycles per
//! bit** for everything it does, so a requested frequency is the actual frequency.
//!
//! A native 4-bit SD host is not flat: its phases cost 4, 5, 6, 8 or 10 cycles depending on
//! whether it is sending a command, hunting for a start bit or streaming data, and all of
//! them share one divider. There a "25 MHz bus" means 25 MHz during a command and 20 MHz
//! during a data read, and the divider has to be sized against the *cheapest* phase so no
//! phase can exceed what the caller asked for.
//!
//! [`divider`] therefore takes `cycles_per_bit` rather than assuming one. A caller with
//! several phases passes its fastest; a caller with one passes that one.
//!
//! # The SD numbers here
//!
//! [`SPEC_CEILING_HZ`] and [`max_sd_clock`] are SD-specific in a crate that otherwise is
//! not. They are here rather than duplicated into each transport because both need them and
//! both would need the same rationale comment, and a constant whose reasoning is copied is a
//! constant that gets changed in one place.

use fixed::FixedU32;
use fixed::types::extra::U8;

/// PIO cycles per bit for a mode-0 SPI master.
///
/// `out pins, 1 side 0 [1]` then `in pins, 1 side 1 [1]` -- two instructions, two delay
/// slots, four cycles, and the same four for every byte in either direction.
pub const SPI_CYCLES_PER_BIT: u32 = 4;

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
pub fn divider(clk_sys_hz: u32, target_hz: u32, cycles_per_bit: u32) -> FixedU32<U8> {
    // `cycles_per_bit` is deliberately *not* folded into the numerator as `256 /
    // cycles_per_bit`. That is integer division, and it is only exact when the count
    // divides 256 -- so it is right for 4 and 8 and silently wrong for 5, 6 and 10, where
    // it truncates and produces a divider that is too *small*, i.e. a bus faster than
    // asked for. At 400 kHz that is the identification ceiling breached, which is the one
    // place this must never happen. Carrying it in the denominator keeps it exact for any
    // count.
    //
    // u64 throughout: a 200 MHz system clock times 256 is 5.12e10, which a u32 cannot hold,
    // and the overflow would wrap to a small divider -- an overclocked bus, the worst
    // possible failure here.
    let numerator = (clk_sys_hz as u64) * 256;
    let denominator = (cycles_per_bit.max(1) as u64) * (target_hz.max(1) as u64);
    let raw = numerator.div_ceil(denominator).clamp(MIN_RAW, MAX_RAW);
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
pub fn max_sd_clock(clk_sys_hz: u32, cycles_per_bit: u32) -> u32 {
    (clk_sys_hz / cycles_per_bit.max(1)).min(SPEC_CEILING_HZ)
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

    /// The property that actually matters: the bus may never exceed the requested clock.
    ///
    /// Stated over a range of cycle counts rather than just this crate's four, because
    /// [`divider`] takes the count as a parameter and a caller with a costlier program has
    /// the same right to the guarantee.
    #[test]
    fn the_bus_never_runs_faster_than_requested() {
        for sys in SYS {
            for hz in TARGETS {
                for c in [SPI_CYCLES_PER_BIT, 5, 6, 8, 10] {
                    let div = divider(sys, hz, c);
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
    /// conservative. It should land within a percent of the target.
    #[test]
    fn the_bus_lands_close_to_the_target() {
        for sys in SYS {
            for hz in TARGETS {
                let div = divider(sys, hz, SPI_CYCLES_PER_BIT);
                let actual = phase_hz(sys, div, SPI_CYCLES_PER_BIT);
                assert!(
                    actual as u64 * 100 >= hz as u64 * 99,
                    "sys {sys} target {hz}: got {actual}, more than 1% slow"
                );
            }
        }
    }

    /// The two rates this transport actually uses on an RP2350, pinned because
    /// `PioSpiBus::set_hz` warns when a rate is not exactly reachable and the warning would
    /// be noise if these ever stopped being exact.
    ///
    /// embassy's own divider truncates rather than rounding up, so it can land *above* the
    /// requested frequency. Both of these are exactly representable in the 24.8 divider --
    /// 93.75 and 15.0 -- so the two computations agree and the warning stays silent. At
    /// 400 kHz that exactness is not cosmetic: it is the identification ceiling.
    #[test]
    fn the_rates_this_transport_uses_are_exactly_reachable_on_rp2350() {
        const RP2350: u32 = 150_000_000;
        for hz in [400_000, 10_000_000] {
            let div = divider(RP2350, hz, SPI_CYCLES_PER_BIT);
            assert_eq!(
                phase_hz(RP2350, div, SPI_CYCLES_PER_BIT),
                hz,
                "{hz} Hz should be exact on a {RP2350} Hz system clock"
            );
        }
    }

    /// `set_config` asserts both ends of this range; tripping it is a panic inside
    /// embassy with no mention of SD cards, so the clamp belongs here.
    #[test]
    fn the_divider_stays_within_what_set_config_accepts() {
        for sys in SYS {
            for hz in [1u32, 400_000, 25_000_000, u32::MAX] {
                let div = divider(sys, hz, SPI_CYCLES_PER_BIT);
                assert!(div >= FixedU32::<U8>::from_num(1), "{sys}/{hz} -> {div}");
                assert!(div <= FixedU32::<U8>::from_num(65536), "{sys}/{hz} -> {div}");
            }
        }
        // A zero target must not divide by zero on the way to the clamp, and neither must
        // a zero cycle count -- both are `max(1)`ed rather than trusted.
        assert_eq!(
            divider(150_000_000, 0, SPI_CYCLES_PER_BIT),
            FixedU32::<U8>::from_num(65536)
        );
        assert_eq!(divider(150_000_000, 400_000, 0), divider(150_000_000, 400_000, 1));
    }

    /// The two chips in this family do not share a default system clock, and the reachable
    /// SD clock follows `clk_sys` directly. Both defaults must still clear the ceiling, or
    /// `supports_frequency()` would report a speed the host cannot actually produce.
    #[test]
    fn both_chips_reach_the_ceiling_at_their_default_clocks() {
        let c = SPI_CYCLES_PER_BIT;
        assert_eq!(max_sd_clock(125_000_000, c), SPEC_CEILING_HZ); // RP2040, 31.25 MHz raw
        assert_eq!(max_sd_clock(150_000_000, c), SPEC_CEILING_HZ); // RP2350, 37.5 MHz raw

        // A system clock dropped for power reports honestly rather than over-promising.
        assert_eq!(max_sd_clock(48_000_000, c), 12_000_000);
        // ...and the divider then bottoms out at 1.0 rather than going below it.
        assert_eq!(
            divider(48_000_000, SPEC_CEILING_HZ, c),
            FixedU32::<U8>::from_num(1)
        );
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
        let div = divider(125_000_000, 400_000, SPI_CYCLES_PER_BIT);
        assert_eq!(div, FixedU32::<U8>::from_num(78.125));
        assert_eq!(phase_hz(125_000_000, div, SPI_CYCLES_PER_BIT), 400_000);
    }
}
