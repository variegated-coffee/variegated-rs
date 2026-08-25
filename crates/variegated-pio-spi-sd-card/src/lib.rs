//! An SD card in SPI mode, on a PIO-driven SPI master.
//!
//! For a board whose SPI peripherals are all spoken for. The Silvia has two -- SPI0 drives
//! the display and SPI1 the internal ADC bus -- and a card slot with no peripheral left to
//! reach it.
//!
//! # What this crate is, and what it is not
//!
//! It is **not** an SPI driver. embassy-rp already ships one:
//! `embassy_rp::pio_programs::spi::Spi` is the RP2350 datasheet program from section 11.6.1,
//! full duplex, with a DMA constructor and an `embedded_hal_async::spi::SpiBus<u8>` impl.
//!
//! It is not an SD driver either. `sdio::spi::SpiMmcBus` is a complete SPI-mode transport
//! and `sdio::BlockDevice` the card state machine above it.
//!
//! What was missing is the seam between them. `SpiMmcBus` needs its bus to implement
//! `SpiBus<u8>` **and** `sdio::spi::SetHz`, embassy's driver supplies the first and not the
//! second, and the orphan rule forbids adding a trait from one foreign crate to a type from
//! another. So [`PioSpiBus`] is a newtype that forwards five methods and adds one -- plus
//! the two pieces of arithmetic that newtype needs to be correct.
//!
//! # Why it is named for SD cards rather than for SPI
//!
//! Because [`PioSpiBus`] implements a trait from `sdio` and exists only to feed
//! `sdio::spi::SpiMmcBus`. A genuinely general PIO SPI master is embassy's, unwrapped. The
//! parts that *are* general -- the `GPIOBASE` window arithmetic and the clock divider --
//! live in `variegated-rp-pio`, shared with the native 4-bit transport.
//!
//! # The five-bit fields
//!
//! `variegated_rp_pio::window` exists because three PIO registers take a GPIO index in five
//! bits, so on RP2350B they must be counted relative to `GPIOBASE`: `WAIT`'s index,
//! `INPUT_SYNC_BYPASS`, and `EXECCTRL.JMP_PIN`. Getting any of them wrong makes the block
//! watch a pin nobody chose, and the symptom names neither the pin nor the window.
//!
//! The datasheet SPI program reaches none of them -- no `jmp PIN`, no `wait gpio`, and
//! embassy never touches the sync bypass, so only the four `PINCTRL` bases are in play and
//! `set_config` shifts those correctly. [`PioSpiBus::new`] still uses the window arithmetic,
//! to reject a pinout no single window can hold before embassy panics about it in terms
//! that name neither the board nor the card.

#![cfg_attr(not(test), no_std)]
#![warn(missing_docs)]

/// Wire-level tracing of the card's bring-up.
///
/// At `info` rather than `debug`, so it survives whatever level the consuming firmware
/// filters at -- a bring-up log nobody can see is not a bring-up log. Off unless both
/// `trace` and `defmt` are on.
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
/// Behind `defmt` alone rather than `trace`, which is the point: a bus that cannot produce
/// the clock it was asked for is something the layer above wants to know about during
/// normal operation.
#[cfg(feature = "defmt")]
macro_rules! mmc_warn {
    ($($arg:tt)*) => { defmt::warn!($($arg)*) };
}
#[cfg(not(feature = "defmt"))]
macro_rules! mmc_warn {
    ($($arg:tt)*) => {};
}

#[cfg(feature = "_chip")]
mod spi;

#[cfg(feature = "_chip")]
pub use spi::PioSpiBus;

// Silence `unused_macros` in the pure-host build, where nothing that uses them is compiled.
// Deliberately not `#[allow]` on the macros themselves: they *should* warn if they fall out
// of use in a build that does compile the hardware module.
#[cfg(not(feature = "_chip"))]
const _: () = {
    #[allow(unused)]
    fn _macros_are_used_by_the_hardware_module() {
        mmc_trace!("");
        mmc_warn!("");
    }
};
