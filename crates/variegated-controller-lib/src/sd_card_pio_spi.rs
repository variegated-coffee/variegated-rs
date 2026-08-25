//! An SD card in SPI mode, on a PIO-driven SPI master.
//!
//! # How this differs from its two neighbours
//!
//! [`crate::sd_card`] runs a card over the RP2350's *hardware* SPI, shared with the display,
//! and almost everything in it exists to solve that sharing: the lease, the timeout applied
//! inside it, `SharedSpiBus`. [`crate::sd_card_pio`] runs a card over a *native 4-bit* PIO
//! bus, which shares nothing but implements the whole SD protocol itself.
//!
//! This is the third combination and the simplest of the three: plain SPI framing, which
//! `sdio` already implements, over a PIO master, which owns its pins outright. There is no
//! lease because there is nothing to share, and no protocol here because `sdio::spi` has it.
//!
//! # The same pins
//!
//! An SD card in SPI mode uses the same physical contacts as it does in native mode, so a
//! slot wired for 4-bit needs no rewiring:
//!
//! | SPI | native |
//! |---|---|
//! | SCK | CLK |
//! | MOSI | CMD |
//! | MISO | DAT0 |
//! | CS | DAT3 |
//!
//! DAT1 and DAT2 go unused and should be left pulled high. **CS is an ordinary
//! [`Output`]**, not a PIO pin: `sdio::spi::SpiMmcBus` owns it and holds it low across a
//! whole command, header and data phase together.

use embassy_rp::gpio::Output;
use embassy_rp::pio::Instance;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use variegated_log::log_info;
use variegated_pio_mmc_bus::PioSpiBus;

use crate::sd_card::PartitionOffset;
use crate::shot_log_storage::SdShotLogStorage;

/// The whole SPI-mode PIO card stack, named once.
///
/// The counterpart of [`crate::sd_card::SdCardBlockDevice`] and
/// [`crate::sd_card_pio::SdCardPioBlockDevice`]. Spelled out here for the same reason as
/// both: it keeps `sdio` and `variegated-pio-mmc-bus` out of the firmwares' dependency
/// lists, so a firmware names only its own PIO instance and state-machine index.
///
/// Note what is *absent* compared with the SPI alias next door -- there is no bus-lease type
/// parameter and no mutex, because a PIO master has no second user to arbitrate against.
pub type SdCardPioSpiBlockDevice<'d, P, const SM: usize> = sdio::BlockDevice<
    sdio::sd::Card,
    sdio::spi::SpiMmcBus<PioSpiBus<'d, P, SM>, Output<'static>, embassy_time::Delay>,
    embassy_time::Delay,
    512,
>;

/// Wrap the bus and CS pin into a block device without touching the card.
///
/// Deliberately **not** `new_sd_card`, for the reason spelled out on
/// [`crate::sd_card::new_sd_card_device`]: identification as part of construction would
/// consume the pins on failure, and there is one set of pins per boot, so a card seated a
/// second late would be unreachable until reboot. This performs no I/O and cannot fail;
/// every real bring-up goes through [`reacquire_pio_spi_sd_card`], which can be retried.
pub fn new_pio_spi_sd_card_device<'d, P: Instance, const SM: usize>(
    bus: PioSpiBus<'d, P, SM>,
    cs: Output<'static>,
) -> SdCardPioSpiBlockDevice<'d, P, SM> {
    let bus = sdio::spi::SpiMmcBus::new(bus, cs, embassy_time::Delay);
    sdio::BlockDevice::new_uninit_sd_card(bus, embassy_time::Delay)
}

/// Re-run identification on a card that is already wrapped in a block device.
///
/// How a card swap is handled, and the same argument applies as on both other paths: the
/// pins are moved into the bus on construction and nothing gives them back, so there is one
/// bus per boot and it has to be re-usable. `reacquire` re-runs the SPI-mode identification
/// sequence and re-reads the CSD, which is what makes addressing a *different* card safe.
///
/// `operating_hz` is the clock to run at **after** identification, not during it. `sdio`
/// drives CMD0/CMD8/ACMD41 at its own 400 kHz `INIT_FREQ` and only then moves up -- it calls
/// `SetHz` itself at both points, so the frequency is never chosen here. It also clamps
/// whatever is asked for to 25 MHz, which is what its SPI transport reports as its ceiling.
///
/// Unlike the hardware-SPI path there is no lease to strand, so the timeout may safely be
/// applied by the caller instead; it is taken as an argument only so the three functions
/// read the same way.
pub async fn reacquire_pio_spi_sd_card<'d, P: Instance, const SM: usize>(
    device: &mut SdCardPioSpiBlockDevice<'d, P, SM>,
    operating_hz: u32,
    timeout: embassy_time::Duration,
) -> Result<(), sdio::MmcError> {
    log_info!("SD: identifying card on the PIO SPI bus");
    match embassy_time::with_timeout(timeout, device.reacquire(operating_hz)).await {
        Ok(inner) => inner,
        Err(_) => Err(sdio::MmcError::Timeout),
    }
}

/// The card as the filesystem sees it: shifted to the start of its partition.
pub type SdCardPioSpiVolume<'d, P, const SM: usize> =
    PartitionOffset<SdCardPioSpiBlockDevice<'d, P, SM>>;

/// Shot-log storage on a PIO SPI card.
///
/// [`SdShotLogStorage`] is generic over an *optional* shared SPI bus, because the hardware
/// SPI transport has to lease the display's bus around every filesystem operation. A PIO
/// master owns its pins outright, so the bus is always `None` and the lease is a no-op.
///
/// That still leaves the two bus type parameters to name. They are pinned here to
/// `NoopRawMutex` and `()` -- a pair that satisfies the bounds and is never instantiated,
/// since `Option::None` carries no value. Exactly as
/// [`crate::sd_card_pio::SdCardPioShotLogStorage`] does it, and for the same reason: a PIO
/// board should not have to name SPI-lease types for a bus it does not have.
pub type SdCardPioSpiShotLogStorage<'d, P, const SM: usize> =
    SdShotLogStorage<'static, SdCardPioSpiVolume<'d, P, SM>, NoopRawMutex, ()>;

/// Mount an identified card at the start of its partition.
///
/// `first_lba` comes from [`crate::sd_card::probe_volume_start`], which is generic over the
/// block device and works unchanged here. Re-probe it on every bring-up rather than caching:
/// a swapped card need not be partitioned like the one before it, and a stale offset reads a
/// perfectly good card at the wrong place instead of failing.
pub fn mount_pio_spi_sd_card<'d, P: Instance, const SM: usize>(
    device: SdCardPioSpiBlockDevice<'d, P, SM>,
    first_lba: u32,
) -> SdCardPioSpiShotLogStorage<'d, P, SM> {
    SdShotLogStorage::new(PartitionOffset::new(device, first_lba), None)
}

/// Compile-time proof that the PIO SPI card plugs into `exfat-slim` with no adapter.
///
/// Mirrors `_sdio_is_an_exfat_block_device` in [`crate::sd_card`] and
/// `_pio_sdio_is_an_exfat_block_device` in [`crate::sd_card_pio`], and for the same reason:
/// the two crates meet directly only while cargo unifies them on one `block-device-driver`
/// and one `aligned`, and a version bump that split those would surface as an unsatisfied
/// bound deep in the storage layer instead of here. Never called; instantiating it is the
/// point.
#[allow(dead_code)]
fn _pio_spi_sdio_is_an_exfat_block_device<'d, P, const SM: usize>()
where
    P: Instance + 'd,
    SdCardPioSpiBlockDevice<'d, P, SM>: exfat_slim::asynchronous::BlockDevice<512>,
{
}
