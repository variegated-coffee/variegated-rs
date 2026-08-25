//! A PIO-backed 4-bit SD/MMC host bus for RP2040 and RP2350.
//!
//! Implements [`sdio::MmcBus`] using two PIO state machines, so an SD card can be driven
//! over a real four-line data bus instead of over SPI. The `sdio` crate already knows how
//! to negotiate 4-bit mode with a card -- it issues ACMD6 and then hands the width to
//! [`sdio::MmcBus::set_bus`] -- but its only shipped transport is `sdio::spi::SpiMmcBus`,
//! whose `set_bus` discards the width argument because SPI mode is physically one data
//! line. This crate is the missing half: the host side that actually reconfigures its
//! pads and clocks four lines.
//!
//! # Provenance
//!
//! Adapted from [SdFat](https://github.com/greiman/SdFat) by Bill Greiman, specifically
//! `src/SdCard/PioSdio/` at commit `cda0573`. SdFat is MIT licensed and so is this crate;
//! the full notice is in `LICENSE-SdFat.md` next to this file. What follows is what is
//! whose, stated in both directions -- a bare "based on SdFat" would over-claim for the
//! parts that are ours and under-claim for the parts that are his.
//!
//! ## Ported from SdFat, structurally unchanged
//!
//! * The five PIO programs `cmd_rsp`, `rd_clk`, `rd_data`, `wr_data` and `wr_resp`,
//!   transcribed instruction for instruction **including every `[n]` delay slot**. The
//!   delays are not padding: they set how many PIO cycles one SD clock costs in each
//!   phase (command 4, response wait 8, response read 6, start-bit search 8, read data 5,
//!   write data 4, write response 10), and since a single clock divider serves all of
//!   them, those counts *are* the design.
//! * The X/Y register preloads -- `X = 55` for a command frame, `Y = 8 * n_rsp - 1` for a
//!   response -- and the response lengths 0/6/17 bytes.
//! * The read flow-control invariant: one word in `rd_clk`'s TX FIFO buys exactly eight
//!   SD clocks, prefill eight, refill one-for-one, and drain the tail without refilling.
//!   See [`bus`] for why this is what makes the whole driver safe to `await` inside.
//! * Response framing and validation, including that R3 carries no CRC and is checked
//!   instead by its two all-ones reserved fields.
//! * The 80-clock power-up sequence with CMD held high, ending with CLK low.
//! * The write framing: start token `0xFFFF_FFF0`, end token `0xFFFF_FFFF`, a nibble
//!   count of `8 + 2*B + 16 + 1`, and the `(status & 0x1F) == 0b0_010_1` accept test.
//! * Where the byte swap sits relative to the CRC -- on read, CRC the word as received
//!   and swap only on the way to memory; on write, swap first and CRC the swapped word.
//! * Using PIO IRQ flag 7 as the `rd_clk` -> `rd_data` and `wr_data` -> `wr_resp` handoff.
//!
//! ## Changed here
//!
//! * `async` throughout, against `embassy-time` deadlines, replacing SdFat's `Timeout`
//!   plus `__time_critical_func` plus 4x-unrolled polling. This is a throughput
//!   trade-off, not a correctness one; [`bus`] explains why.
//! * An optional DMA data path. SdFat uses no DMA and no interrupts at all.
//! * Arbitrary block sizes. SdFat only ever transfers 512-byte sectors plus a handful of
//!   fixed-size registers, whereas `sdio` asks for 8-byte (ACMD51) and 64-byte (CMD6,
//!   ACMD13) DAT reads through the same entry point.
//! * The [`sdio::MmcBus`] surface rather than SdFat's card class. Command *selection* is
//!   the caller's, so there is no single-block/multi-block distinction down here.
//!
//! ## Original to this crate, not from SdFat
//!
//! * **`rd_data_1bit` and the entire 1-bit data path.** SdFat is 4-bit only -- every DAT
//!   transfer in its life is four lines wide. That is viable for SdFat because it sets
//!   4-bit width before its first data transfer, but `sdio::sd::Card::acquire` reads the
//!   SCR with ACMD51 *before* it issues ACMD6, so the first data transfer of every card's
//!   life happens at one bit. Without a 1-bit reader, no card could be identified at all.
//! * **The data CRC-16.** Written from the SD Physical Layer Simplified Specification
//!   (CRC-16-CCITT, polynomial `x^16 + x^12 + x^5 + 1`, zero initial value, no reflection,
//!   no final inversion, computed independently per data line) as a table-driven
//!   implementation, and deliberately *not* ported from SdFat's `crc16()`. That routine
//!   is a bit-sliced `uint64_t` construction whose own comment records it as "Modified
//!   from `sdio_crc16_4bit_checksum()` in ZuluSCSI-firmware" -- a GPL-family project,
//!   which raises a licence question relative to this crate's MIT terms that is not ours
//!   to resolve. Ours is a different algorithm (a nibble-transpose table feeding a
//!   byte-wise CRC table) reached from the specification, and it is pinned against
//!   published CRC-16/XMODEM vectors rather than against SdFat's output. See [`crc`].
//!
//! The CRC-7 is taken from the sibling `sdio` crate's SPI transport -- same workspace,
//! same authorship, no third party involved.
//!
//! The `sdio` crate whose trait this implements is separately licensed; see its own
//! repository.
//!
//! # Hardware requirements
//!
//! * `DAT0`..`DAT3` must be **four consecutive GPIOs in ascending order**. `CLK` and
//!   `CMD` may be anywhere.
//! * On RP2350 all six pins must fall in one `GPIOBASE` window -- all below 32, or all at
//!   16 and above. The high window is supported and is not a second-class path: pin
//!   numbers reaching the PIO block are translated into its own numbering throughout, and
//!   [`window`] carries the arithmetic and its tests. Note that embassy's own
//!   `Pin::set_input_sync_bypass` cannot be used above GPIO 31 -- it shifts by the
//!   absolute pin number into a `u32` -- which is why this crate builds that mask itself.
//! * An entire PIO block is not required, but two state machines and 31 of the block's 32
//!   instruction slots are.
//! * External pull-ups on `CMD` and `DAT0`..`DAT3` are required. The internal ones this
//!   crate enables are a floor, not a substitute.

#![cfg_attr(not(test), no_std)]

/// Wire-level tracing of commands, responses and data tokens.
///
/// Behind `trace` rather than `defmt`, and logged at `info` rather than `trace`, for the
/// same reason as the identical macro in the `sdio` crate: this is the output you want
/// when a card will not initialise, and a trace you have to rebuild the firmware to see
/// is no use in exactly the situation it exists for. At `info` it survives whatever
/// level the consuming firmware filters at.
#[cfg(all(feature = "trace", feature = "defmt"))]
macro_rules! mmc_trace {
    ($($arg:tt)*) => { defmt::info!($($arg)*) };
}
#[cfg(not(all(feature = "trace", feature = "defmt")))]
macro_rules! mmc_trace {
    ($($arg:tt)*) => {};
}

/// Faults worth reporting to firmware that is *not* debugging this driver.
///
/// Behind `defmt` alone rather than `trace`, which is the point: a marginal bus is
/// something the layer above wants to know about during normal operation. `sdio`'s
/// `BlockDevice` retries CRC failures up to ten times and succeeds silently, so without
/// this the difference between a healthy card and one a single bit-flip from failing is
/// not observable at all.
#[cfg(feature = "defmt")]
macro_rules! mmc_warn {
    ($($arg:tt)*) => { defmt::warn!($($arg)*) };
}
#[cfg(not(feature = "defmt"))]
macro_rules! mmc_warn {
    ($($arg:tt)*) => {};
}

pub mod clock;
pub mod crc;
pub mod frame;
pub mod programs;

/// The `GPIOBASE` window arithmetic, re-exported from `variegated-rp-pio`.
///
/// It lives in a leaf crate because the SPI-mode transport needs it too, and neither
/// transport should depend on the other. Re-exported rather than referenced through its own
/// path so this crate's modules can keep saying `crate::window`, and so a reader of
/// `pins.rs` finds it where the five-bit-field comments say it is.
pub use variegated_rp_pio::window;

#[cfg(feature = "_chip")]
mod bus;
#[cfg(feature = "_chip")]
mod install;
#[cfg(feature = "_chip")]
mod pins;

#[cfg(feature = "_chip")]
pub use bus::PioMmcBus;

// Silence `unused_macros` in the pure-host build, where nothing that uses them is
// compiled. Deliberately not `#[allow]` on the macros themselves: they *should* warn if
// they fall out of use in a build that does compile the hardware modules.
#[cfg(not(feature = "_chip"))]
const _: () = {
    #[allow(unused)]
    fn _macros_are_used_by_the_hardware_modules() {
        mmc_trace!("");
        mmc_warn!("");
    }
};
