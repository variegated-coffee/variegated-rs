//! An SD card on its own PIO block, four data lines wide.
//!
//! # How this differs from [`crate::sd_card`]
//!
//! The SPI module next door exists almost entirely to solve a sharing problem: the card
//! and the NV3007 display are on the same SPI0, an SD command holds CS low across several
//! `SpiBus` calls, and so the bus has to be leased for a whole filesystem operation rather
//! than per transaction. Every awkward thing in that module -- [`crate::sd_card::SharedSpiBus`],
//! the lease, the timeout that must be applied *inside* it -- follows from that.
//!
//! None of it applies here. A PIO card owns its block, its two state machines and its six
//! GPIOs outright; there is nothing to share and nothing to lease, so the constructor
//! takes the pins and the re-acquire is an ordinary call with an ordinary timeout.
//!
//! Everything *above* the transport is unchanged and reused as-is:
//! [`crate::sd_card::PartitionOffset`] is generic over the block device, and so is the
//! shot-log storage on top of it.
//!
//! Roughly four times the throughput of the SPI path -- four data lines at up to 25 MHz
//! rather than one at 10 -- which is why it is worth having a second transport at all.

use embassy_rp::Peri;
use embassy_rp::dma;
use embassy_rp::pio::{Common, Instance, PioPin, StateMachine};
use variegated_log::log_info;
use variegated_pio_mmc_bus::PioMmcBus;

/// The whole PIO card stack, named once -- the native-bus counterpart of
/// [`crate::sd_card::SdCardBlockDevice`].
///
/// Spelled out here for the same reason as that one: it keeps `sdio` and
/// `variegated-pio-mmc-bus` out of the firmwares' dependency lists, so a firmware names
/// only its own PIO instance and state-machine indices.
///
/// Note there is no bus-lease type parameter. That is the whole point of this path.
pub type SdCardPioBlockDevice<'d, P, const SM_DAT: usize, const SM_CLK: usize> = sdio::BlockDevice<
    sdio::sd::Card,
    PioMmcBus<'d, P, SM_DAT, SM_CLK>,
    embassy_time::Delay,
    512,
>;

/// Wrap the pins into a block device without touching the bus.
///
/// Deliberately **not** `new_sd_card`, for exactly the reason spelled out on
/// [`crate::sd_card::new_sd_card_device`]: identification as part of construction would
/// consume the pins on failure, and there is one set of pins per boot. A card seated a
/// second late would then disable storage until reboot. This performs no I/O and cannot
/// fail; every real bring-up goes through [`reacquire_pio_sd_card`], which can be retried.
///
/// `SM_CLK` must be greater than `SM_DAT` -- see [`PioMmcBus`] -- and DAT0..DAT3 must be
/// four consecutive ascending GPIOs.
#[allow(clippy::too_many_arguments)]
pub fn new_pio_sd_card_device<'d, P: Instance, const SM_DAT: usize, const SM_CLK: usize>(
    common: &mut Common<'d, P>,
    sm_dat: StateMachine<'d, P, SM_DAT>,
    sm_clk: StateMachine<'d, P, SM_CLK>,
    clk: Peri<'d, impl PioPin>,
    cmd: Peri<'d, impl PioPin>,
    dat0: Peri<'d, impl PioPin>,
    dat1: Peri<'d, impl PioPin>,
    dat2: Peri<'d, impl PioPin>,
    dat3: Peri<'d, impl PioPin>,
) -> SdCardPioBlockDevice<'d, P, SM_DAT, SM_CLK> {
    let bus = PioMmcBus::new(common, sm_dat, sm_clk, clk, cmd, dat0, dat1, dat2, dat3);
    sdio::BlockDevice::new_uninit_sd_card(bus, embassy_time::Delay)
}

/// As [`new_pio_sd_card_device`], but the 4-bit data phases run on DMA.
///
/// Two channels, which is the cost. What it buys is the executor being free during the
/// ~50 us a 512-byte block spends on the wire; the CRC pass afterwards is unchanged, so
/// this is not a doubling of throughput.
#[allow(clippy::too_many_arguments)]
pub fn new_pio_sd_card_device_with_dma<'d, P: Instance, const SM_DAT: usize, const SM_CLK: usize>(
    common: &mut Common<'d, P>,
    sm_dat: StateMachine<'d, P, SM_DAT>,
    sm_clk: StateMachine<'d, P, SM_CLK>,
    clk: Peri<'d, impl PioPin>,
    cmd: Peri<'d, impl PioPin>,
    dat0: Peri<'d, impl PioPin>,
    dat1: Peri<'d, impl PioPin>,
    dat2: Peri<'d, impl PioPin>,
    dat3: Peri<'d, impl PioPin>,
    rx: dma::Channel<'d>,
    tx: dma::Channel<'d>,
) -> SdCardPioBlockDevice<'d, P, SM_DAT, SM_CLK> {
    let bus = PioMmcBus::new_with_dma(
        common, sm_dat, sm_clk, clk, cmd, dat0, dat1, dat2, dat3, rx, tx,
    );
    sdio::BlockDevice::new_uninit_sd_card(bus, embassy_time::Delay)
}

/// Re-run identification on a card that is already wrapped in a block device.
///
/// How a card swap is handled, and the same argument applies as on the SPI path: the pins
/// are moved into the bus on construction and nothing gives them back, so there is one
/// bus per boot and it has to be re-usable. `reacquire` re-runs CMD0/CMD8/ACMD41 and
/// re-reads the CSD, which is what makes addressing a *different* card safe -- capacity
/// and byte-versus-block addressing both come from the card, and reusing the previous
/// card's values would read from the wrong offsets rather than fail outright.
///
/// `operating_hz` is the clock to run at *after* identification. `sdio` drives the
/// identification sequence at its own 400 kHz and only then moves up, and the bus clamps
/// whatever is asked for to what the current system clock can actually produce -- which is
/// not the same number on RP2040 and RP2350.
///
/// Unlike the SPI path, the timeout here may safely be applied by the caller instead; it
/// is taken as an argument only so the two functions read the same way. There is no lease
/// to strand, so a cancelled future leaves nothing held.
pub async fn reacquire_pio_sd_card<'d, P: Instance, const SM_DAT: usize, const SM_CLK: usize>(
    device: &mut SdCardPioBlockDevice<'d, P, SM_DAT, SM_CLK>,
    operating_hz: u32,
    timeout: embassy_time::Duration,
) -> Result<(), sdio::MmcError> {
    log_info!("SD: identifying card on the PIO bus");
    match embassy_time::with_timeout(timeout, device.reacquire(operating_hz)).await {
        Ok(inner) => inner,
        Err(_) => Err(sdio::MmcError::Timeout),
    }
}

/// Compile-time proof that the PIO card plugs into `exfat-slim` with no adapter.
///
/// Mirrors `_sdio_is_an_exfat_block_device` in [`crate::sd_card`], and for the same
/// reason: the two crates meet directly only while cargo unifies them on one
/// `block-device-driver` and one `aligned`, and a version bump that split those would
/// surface as an unsatisfied bound somewhere deep in the storage layer instead of here.
/// Never called; instantiating it is the point.
#[allow(dead_code)]
fn _pio_sdio_is_an_exfat_block_device<'d, P, const SM_DAT: usize, const SM_CLK: usize>()
where
    P: Instance + 'd,
    SdCardPioBlockDevice<'d, P, SM_DAT, SM_CLK>: exfat_slim::asynchronous::BlockDevice<512>,
{
}
