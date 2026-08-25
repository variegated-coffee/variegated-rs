//! An SD card on a PIO-driven SPI master.
//!
//! # How this differs from the rest of the crate
//!
//! [`crate::PioMmcBus`] is a native 4-bit host: it implements `sdio::MmcBus` itself, drives
//! CMD and DAT0..DAT3 directly, and is three thousand lines of PIO programs, command framing
//! and CRCs. This module is the opposite in every way -- it implements no protocol at all.
//!
//! An SD card also speaks plain SPI, over **the same physical pins**: CLK becomes SCK, CMD
//! becomes MOSI, DAT0 becomes MISO and DAT3 becomes CS. `sdio` already ships a complete SPI
//! transport (`sdio::spi::SpiMmcBus`), and embassy-rp already ships a PIO SPI master
//! (`embassy_rp::pio_programs::spi`, the RP2350 datasheet program from section 11.6.1). The
//! only thing standing between them is that `SpiMmcBus` wants its bus to implement
//! `sdio::spi::SetHz` as well as `SpiBus<u8>`, and the orphan rule forbids implementing a
//! trait from one foreign crate on a type from another.
//!
//! So this is a newtype. It forwards five methods and adds one.
//!
//! # What it is for
//!
//! A board whose SPI peripherals are all spoken for. The Silvia has two -- SPI0 drives the
//! display and SPI1 the internal ADC bus -- and a card slot wired to GPIO 41-46 with no
//! peripheral left to reach it.
//!
//! # The five-bit fields, and why none of them bite here
//!
//! [`crate::window`] exists because three PIO registers take a GPIO index in five bits, so on
//! RP2350B they must be counted relative to `GPIOBASE`: `WAIT`'s index, `INPUT_SYNC_BYPASS`,
//! and `EXECCTRL.JMP_PIN`. Getting any of them wrong makes the block watch a pin nobody
//! chose, and the symptom names neither the pin nor the window.
//!
//! The datasheet SPI program reaches none of them. It has no `jmp PIN` and no `wait gpio`,
//! and embassy's driver never touches the sync bypass -- it uses only the four `PINCTRL`
//! bases, which `set_config` shifts correctly. The one thing still worth doing here is
//! rejecting a pinout that no single window can hold, which [`PioSpiBus::new`] does before
//! embassy can panic about it in terms that name neither the board nor the card.

use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::pio::{Common, Instance, PioPin, StateMachine};
use embassy_rp::pio_programs::spi::{Error, Spi};
use embassy_rp::spi::{Async, Config};
use embassy_rp::{dma, interrupt, Peri};
use embedded_hal::spi::ErrorType;
use embedded_hal_async::spi::SpiBus;

use crate::{clock, window};

/// A PIO state machine driving SCK, MOSI and MISO as an SPI master.
///
/// Wraps `embassy_rp::pio_programs::spi::Spi` and adds [`sdio::spi::SetHz`], which is the
/// one thing `sdio::spi::SpiMmcBus` needs that embassy's driver does not provide.
///
/// **CS is not here, deliberately.** `SpiMmcBus` owns the chip select and holds it low across
/// a whole command -- header, response and data phase together -- so it must be an ordinary
/// GPIO handed to that type, never a PIO side-set and never an `SpiDevice`-style
/// per-transaction select. See `variegated_controller_lib::sd_card`'s module docs for the
/// long version of why the per-transaction form cannot work for a card.
pub struct PioSpiBus<'d, P: Instance, const SM: usize> {
    spi: Spi<'d, P, SM, Async>,
}

impl<'d, P: Instance, const SM: usize> PioSpiBus<'d, P, SM> {
    /// Claim a state machine, three pins and two DMA channels.
    ///
    /// Performs no I/O and cannot fail, which is the same property
    /// `variegated_controller_lib::sd_card::new_sd_card_device` is careful to have: the pins
    /// are consumed exactly once per boot, so a construction that could fail would make a
    /// card seated a second late unreachable until reboot. Every bring-up is a retry.
    ///
    /// `config.frequency` is the *identification* clock if the caller sets one, but `sdio`
    /// overrides it immediately -- it calls `set_hz(400_000)` before CMD0 and `set_hz` again
    /// with the operating clock once the card is identified. Mode 0 is what an SD card in SPI
    /// mode requires and is `Config::default()`'s phase and polarity already.
    ///
    /// # Panics
    ///
    /// If SCK, MOSI and MISO do not all fit in one `GPIOBASE` window -- all below GPIO 32, or
    /// all at GPIO 16 and above. That is a board wiring mistake with no runtime recovery, and
    /// embassy's own check for it panics from inside `set_config` with a message about pin
    /// ranges that mentions neither SPI nor the card.
    #[allow(clippy::too_many_arguments)]
    pub fn new<TxDma: dma::ChannelInstance, RxDma: dma::ChannelInstance>(
        common: &mut Common<'d, P>,
        sm: StateMachine<'d, P, SM>,
        sck: Peri<'d, impl PioPin>,
        mosi: Peri<'d, impl PioPin>,
        miso: Peri<'d, impl PioPin>,
        tx_dma: Peri<'d, TxDma>,
        rx_dma: Peri<'d, RxDma>,
        irq: impl interrupt::typelevel::Binding<TxDma::Interrupt, dma::InterruptHandler<TxDma>>
            + interrupt::typelevel::Binding<RxDma::Interrupt, dma::InterruptHandler<RxDma>>
            + 'd,
        config: Config,
    ) -> Self {
        // `Peri` derefs to the pin, and `PioPin: gpio::Pin`, so the numbers are readable
        // before the pins are consumed.
        let pins = [sck.pin(), mosi.pin(), miso.pin()];
        // `_gpio_base` for the same reason as `_e` and `_i` elsewhere in this crate: it is
        // read only from inside `mmc_trace!`, which expands to nothing without the feature.
        // The `expect` is the part that has to run in every build.
        let _gpio_base = window::gpio_base(&pins)
            .expect("SD SPI pins must all be below GPIO 32 or all at GPIO 16 and above");

        mmc_trace!(
            "pio-spi: gpiobase {=u8}, sck {=u8}, mosi {=u8}, miso {=u8}",
            _gpio_base,
            pins[0],
            pins[1],
            pins[2]
        );

        let spi = Spi::new(common, sm, sck, mosi, miso, tx_dma, rx_dma, irq, config);
        Self { spi }
    }
}

impl<'d, P: Instance, const SM: usize> ErrorType for PioSpiBus<'d, P, SM> {
    /// embassy's, and it is an empty enum -- this bus cannot fail. Forwarded rather than
    /// wrapped so there is no error type here that could imply otherwise.
    type Error = Error;
}

impl<'d, P: Instance, const SM: usize> SpiBus<u8> for PioSpiBus<'d, P, SM> {
    async fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        // Forwarded faithfully, and **not** what an SD card read goes through. Embassy's
        // `read` clocks zeros onto MOSI, while the SD physical layer requires the host to
        // hold DI high for the whole time it is clocking a response out; a card clocked with
        // DI low answers with bit-misaligned garbage. `sdio` knows this and routes every read
        // through `transfer` against a 0xFF source instead -- see `read_high` in
        // `sdio::spi`. This method exists to satisfy the trait, not to be used here.
        self.spi.read(words).await
    }

    async fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.spi.write(words).await
    }

    async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        self.spi.transfer(read, write).await
    }

    async fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        self.spi.transfer_in_place(words).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        // The *inherent* `flush`, which is synchronous and really does drain -- it spins
        // until the TX FIFO is empty and the state machine has stalled on it. Embassy's own
        // async `SpiBus::flush` returns `Ok(())` without doing anything, on the grounds that
        // its DMA paths join both transfers before returning. Draining is the stronger
        // guarantee and costs nothing here, since `sdio` never calls this at all.
        self.spi.flush()
    }
}

impl<'d, P: Instance, const SM: usize> sdio::spi::SetHz for PioSpiBus<'d, P, SM> {
    /// Reprogram the state machine's clock divider.
    ///
    /// Called exactly twice per bring-up and never mid-command: `sdio` sets 400 kHz before
    /// CMD0 and the operating clock after the card is identified. Infallible by signature --
    /// a failure here has nowhere to go, which is why the check below logs rather than
    /// returns.
    ///
    /// # The rounding, which matters at 400 kHz
    ///
    /// embassy computes `clk_sys / (hz * 4)` and **truncates**, so it can land on a divider
    /// smaller than asked for and therefore a bus *faster* than requested. At the operating
    /// clock that is harmless. At 400 kHz it is not: the SD physical layer gives 400 kHz as a
    /// hard ceiling for identification, and exceeding it is exactly the kind of fault that
    /// presents as a card that will not enumerate on one board and will on another.
    ///
    /// [`clock::divider`] rounds *up* and so can never exceed the target. Both are computed
    /// and compared rather than one being used, because embassy owns the state machine and
    /// does not expose it -- so the honest thing available here is to say when the two
    /// disagree instead of silently running out of specification.
    ///
    /// On RP2350 at 150 MHz the two rates in use are both exactly representable in the 24.8
    /// fixed-point divider -- 93.75 at 400 kHz and 1.5 at 25 MHz -- so this is a guard
    /// against a future system clock, not a live defect.
    fn set_hz(&mut self, hz: u32) {
        let clk_sys = clk_sys_freq();
        let safe = clock::divider(clk_sys, hz);
        let actual = clock::phase_hz(clk_sys, safe, clock::cycles::FASTEST);

        if actual < hz {
            // `safe` had to round up to stay at or below `hz`, which means embassy's
            // truncating division will land below `safe` and clock the bus faster than
            // asked. Worth a line: at 400 kHz that is a specification violation.
            mmc_warn!(
                "pio-spi: {=u32} Hz is not exactly reachable on a {=u32} Hz system clock; \
                 the nearest safe rate is {=u32} Hz and embassy will round the other way",
                hz,
                clk_sys,
                actual
            );
        }

        self.spi.set_frequency(hz);
    }
}
