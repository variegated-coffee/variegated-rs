//! An SD card sharing the display's SPI bus.
//!
//! # Why this module exists
//!
//! The card and the NV3007 display are wired to the same SPI0 on this board, and the
//! display owns the bus behind an async `Mutex`. That is ordinarily what
//! `embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice` is for -- but it is not
//! usable here, and the reason is worth stating because the obvious fix is wrong.
//!
//! An SD card in SPI mode holds CS **low across a whole command**: `sdio`'s
//! `SpiMmcBus` calls `select()`, then issues the command frame, polls for R1, streams
//! the data token and the CRC as *separate* `SpiBus` calls, and only then
//! `deselect()`s. A `SpiDevice` locks per transaction, so between two of those calls
//! the display could take the bus, assert its own CS and reprogram the clock. The card
//! would go on interpreting whatever the display clocked out as the tail of its own
//! command. So the lock has to be coarser than a transaction, not finer.
//!
//! The previous implementation reached for the other obvious fix -- a blocking
//! `SpiDevice` shim driving the async one through `embassy_futures::block_on` -- and
//! that deadlocks. `block_on` busy-polls with a no-op waker and never re-enters the
//! executor, while the async `SpiDevice`'s first act is `bus.lock().await`. If the
//! display task held the bus at that moment, it could never be scheduled to release
//! it, and core 1 wedged permanently. (It appeared to work only because the one call
//! site that exercised it ran inside `executor1.run`'s init closure, before any task
//! had been polled, so the mutex was always free.)
//!
//! [`SharedSpiBus`] takes the third option: acquire the bus with a real `.await`, hold
//! it for a whole *filesystem operation*, and hand `sdio` a [`SpiBusLease`] that
//! borrows the held guard. No `block_on`, no interleaving, and the display simply
//! waits -- which is why it must use `SpiDeviceWithConfig` and re-apply its own clock
//! per transaction, since [`SpiBusLease::set_hz`] reprograms the bus underneath it.

use core::cell::RefCell;

use embassy_embedded_hal::SetConfig;
use embassy_rp::spi::Config as SpiConfig;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::mutex::{Mutex, MutexGuard};
use embedded_hal_async::spi::{Error as SpiError, ErrorKind, ErrorType, SpiBus};
use variegated_log::{log_info, log_warn};

/// The whole card stack, named once.
///
/// Spelling this out here rather than at each call site is what keeps `sdio` and
/// `exfat-slim` out of the examples' dependency lists: an example only names its own
/// SPI instance and CS pin types and lets this alias supply the rest.
pub type SdCardBlockDevice<'a, M, BUS, CS> = sdio::BlockDevice<
    sdio::sd::Card,
    sdio::spi::SpiMmcBus<SpiBusLease<'a, M, BUS>, CS, embassy_time::Delay>,
    embassy_time::Delay,
    512,
>;

/// Wrap the CS pin into a block device without touching the bus.
///
/// Deliberately **not** `new_sd_card`, which identifies the card as part of
/// construction and consumes the bus -- and with it the CS pin -- if that fails. There
/// is exactly one CS pin per boot, so a construction that can fail is a bring-up that
/// can only ever be attempted once: a card seated a second late, or a first attempt
/// that lands while the card is still powering up, would disable storage until reboot.
///
/// `new_uninit_sd_card` performs no I/O, so this cannot fail, and every actual bring-up
/// goes through [`reacquire_sd_card`] -- which can be retried as often as we like.
pub fn new_sd_card_device<'a, M, BUS, CS>(
    shared: &'a SharedSpiBus<'a, M, BUS>,
    cs: CS,
) -> SdCardBlockDevice<'a, M, BUS, CS>
where
    M: RawMutex,
    BUS: SpiBus<u8> + SetConfig<Config = SpiConfig>,
    CS: embedded_hal::digital::OutputPin,
{
    let bus = sdio::spi::SpiMmcBus::new(shared.handle(), cs, embassy_time::Delay);
    sdio::BlockDevice::new_uninit_sd_card(bus, embassy_time::Delay)
}

/// Compile-time proof that `sdio` plugs into `exfat-slim` with no adapter.
///
/// `exfat_slim::asynchronous::BlockDevice` is a re-export of
/// `block_device_driver::BlockDevice`, which `sdio::BlockDevice` implements -- so the
/// two crates meet directly, but *only* while cargo unifies them on one
/// `block-device-driver` and one `aligned`. If a version bump ever split those, the
/// impl would stop applying and the failure would surface as an unsatisfied trait
/// bound somewhere deep in the storage layer. This says it here instead, next to the
/// claim it is checking. It is never called; instantiating it is the whole point.
#[allow(dead_code)]
fn _sdio_is_an_exfat_block_device<'a, M, BUS, CS>()
where
    M: RawMutex + 'a,
    BUS: SpiBus<u8> + SetConfig<Config = SpiConfig> + 'a,
    CS: embedded_hal::digital::OutputPin,
    SdCardBlockDevice<'a, M, BUS, CS>: exfat_slim::asynchronous::BlockDevice<512>,
{
}

/// Re-run identification on a card that is already wrapped in a block device.
///
/// This is how a card swap is handled. Rebuilding the device from scratch is not an
/// option: the CS pin is moved into `SpiMmcBus` on construction and neither `sdio` nor
/// `exfat-slim` gives it back, so there is exactly one `SpiMmcBus` per boot and it has
/// to be re-usable. `reacquire` re-runs CMD0/ACMD41 and re-reads the CSD, which is
/// what makes addressing a *different* card safe -- capacity and byte-versus-block
/// addressing both come from the card, and reusing the previous card's values would
/// read from the wrong offsets rather than fail.
/// `operating_hz` is the clock to run at *after* identification, not during it --
/// `sdio` drives CMD0/CMD8/ACMD41 at its own 400 kHz `INIT_FREQ` and only then moves up.
/// Pinning the bus at 10 MHz throughout is what the previous implementation got wrong.
///
/// The timeout is not optional and is applied **inside** the lease rather than by the
/// caller. Identification against an empty slot reads 0xFF forever, and the whole
/// sequence runs with the display's bus held: an unbounded bring-up would not merely
/// fail slowly, it would freeze the panel until reboot. Wrapping this call from outside
/// would be worse than useless -- dropping the future mid-lease leaves the guard held
/// and the bus locked for good, because releasing it is not something a cancelled
/// future gets to do.
pub async fn reacquire_sd_card<'a, M, BUS, CS>(
    shared: &'a SharedSpiBus<'a, M, BUS>,
    device: &mut SdCardBlockDevice<'a, M, BUS, CS>,
    operating_hz: u32,
    timeout: embassy_time::Duration,
) -> Result<(), sdio::MmcError>
where
    M: RawMutex,
    BUS: SpiBus<u8> + SetConfig<Config = SpiConfig>,
    CS: embedded_hal::digital::OutputPin,
{
    // Bounded, and traced either side, because a wedge here is invisible otherwise:
    // the caller's only symptom is a task that never comes back to its select loop, with
    // no log line to say which half it stopped in.
    log_info!("SD: waiting for the SPI bus");
    if !shared.lease_within(timeout).await {
        log_warn!("SD: could not take the SPI bus (display holding or starving it)");
        return Err(sdio::MmcError::Timeout);
    }
    log_info!("SD: bus acquired, identifying");

    let result = embassy_time::with_timeout(timeout, device.reacquire(operating_hz)).await;
    shared.release();

    match result {
        Ok(inner) => inner,
        Err(_) => Err(sdio::MmcError::Timeout),
    }
}

/// A block device shifted to the start of a partition.
///
/// `exfat-slim` mounts a *volume*: it expects block 0 of whatever it is given to be the
/// exFAT boot sector. A card formatted by macOS, Windows or the SD Association's tool is
/// MBR-partitioned, so block 0 is the partition table and the volume starts further in
/// -- typically LBA 2048. Handing the raw card straight to the filesystem therefore
/// fails with `InvalidJumpBoot` on a perfectly good card, which reads as "wrong
/// filesystem" and sends you off to reformat something that was already correct.
pub struct PartitionOffset<BD> {
    inner: BD,
    first_lba: u32,
}

impl<BD> PartitionOffset<BD> {
    pub fn new(inner: BD, first_lba: u32) -> Self {
        Self { inner, first_lba }
    }

    /// Where this partition starts on the card.
    pub fn first_lba(&self) -> u32 {
        self.first_lba
    }

    /// Recover the underlying card, e.g. to re-identify it after a swap.
    pub fn into_inner(self) -> BD {
        self.inner
    }
}

impl<BD> exfat_slim::asynchronous::BlockDevice<512> for PartitionOffset<BD>
where
    BD: exfat_slim::asynchronous::BlockDevice<512>,
{
    type Error = BD::Error;
    type Align = BD::Align;

    async fn read(
        &mut self,
        block_address: u32,
        data: &mut [aligned::Aligned<Self::Align, [u8; 512]>],
    ) -> Result<(), Self::Error> {
        self.inner
            .read(block_address.saturating_add(self.first_lba), data)
            .await
    }

    async fn write(
        &mut self,
        block_address: u32,
        data: &[aligned::Aligned<Self::Align, [u8; 512]>],
    ) -> Result<(), Self::Error> {
        self.inner
            .write(block_address.saturating_add(self.first_lba), data)
            .await
    }

    async fn size(&mut self) -> Result<u64, Self::Error> {
        let total = self.inner.size().await?;
        Ok(total.saturating_sub(self.first_lba as u64 * 512))
    }
}

/// Where the filesystem volume starts on this card.
///
/// Returns 0 for a "superfloppy" card formatted without a partition table, otherwise
/// the first sector of the first partition that looks like one we can mount.
///
/// Checks for a volume boot record *before* parsing an MBR, because the two are told
/// apart by content rather than position: an exFAT VBR also carries `0x55AA` at offset
/// 510, so treating that signature as proof of a partition table would parse the boot
/// sector's BPB as partition entries and yield nonsense.
pub async fn probe_volume_start<BD>(dev: &mut BD) -> Result<u32, BD::Error>
where
    BD: exfat_slim::asynchronous::BlockDevice<512>,
{
    let mut buf: [aligned::Aligned<BD::Align, [u8; 512]>; 1] = [aligned::Aligned([0u8; 512])];
    dev.read(0, &mut buf).await?;
    let sector: &[u8; 512] = &buf[0];

    // exFAT volume boot record: jump boot EB 76 90, then the file-system name.
    if sector[0] == 0xEB && sector[1] == 0x76 && sector[2] == 0x90 && &sector[3..11] == b"EXFAT   " {
        log_info!("SD: exFAT volume at LBA 0 (no partition table)");
        return Ok(0);
    }

    if sector[510] != 0x55 || sector[511] != 0xAA {
        log_warn!("SD: no MBR signature and no exFAT VBR at LBA 0");
        return Ok(0);
    }

    // Four 16-byte entries at 446. Type at +4, first LBA at +8, little endian.
    for i in 0..4 {
        let entry = &sector[446 + i * 16..446 + (i + 1) * 16];
        let kind = entry[4];
        let start = u32::from_le_bytes([entry[8], entry[9], entry[10], entry[11]]);

        // 0x07 is exFAT (and NTFS/HPFS -- the MBR type byte does not distinguish them,
        // which is why the mount that follows is what actually decides).
        let mountable = matches!(kind, 0x07 | 0x0B | 0x0C | 0x01 | 0x04 | 0x06);
        if mountable && start != 0 {
            log_info!(
                "SD: partition {} type {:#04x} starts at LBA {}",
                i,
                kind,
                start
            );
            return Ok(start);
        }
    }

    log_warn!("SD: MBR present but no mountable partition; trying LBA 0");
    Ok(0)
}

/// Failures the lease itself can produce, as distinct from the card's.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
// Derived unconditionally: this crate depends on defmt outright and has no `defmt`
// feature, so a `cfg_attr` guard here would simply never fire.
#[derive(defmt::Format)]
pub enum SpiLeaseError {
    /// A bus operation was attempted without the bus having been leased.
    ///
    /// This is a programming error -- some path reached `sdio` without going through
    /// [`SharedSpiBus::lease`] -- but it is reported rather than panicked because a
    /// panic here takes the machine down mid-shot, and a failed SD write must not.
    NotLeased,
    /// The underlying SPI peripheral reported an error.
    Bus,
}

impl SpiError for SpiLeaseError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

/// The bus, plus whatever guard is currently held on it.
///
/// Shared by reference between the storage layer (which leases and releases) and the
/// [`SpiBusLease`] buried inside `sdio` (which reads the held guard). Both borrow the
/// same value, so it has to outlive them: construct it once and leak it, next to the
/// display buffers.
///
/// `RefCell` rather than a mutex because every user is on the same executor on core 1.
/// A re-entrant borrow is a bug -- two SD operations overlapping -- and panicking on it
/// is the correct outcome, since silently proceeding would corrupt the card.
pub struct SharedSpiBus<'a, M: RawMutex, BUS> {
    bus: &'a Mutex<M, BUS>,
    held: RefCell<Option<MutexGuard<'a, M, BUS>>>,
    /// Phase and polarity are the board's and never change; only the frequency does.
    /// Kept so `set_hz` can rebuild a full `Config` -- embassy-rp has no
    /// frequency-only setter, and defaulting the rest would silently drop the mode.
    base_config: RefCell<SpiConfig>,
}

impl<'a, M: RawMutex, BUS> SharedSpiBus<'a, M, BUS> {
    /// `base_config` must be the same phase/polarity the bus was built with.
    pub fn new(bus: &'a Mutex<M, BUS>, base_config: SpiConfig) -> Self {
        Self {
            bus,
            held: RefCell::new(None),
            base_config: RefCell::new(base_config),
        }
    }

    /// Take the bus and hold it until [`release`](Self::release).
    ///
    /// Await this once around a whole filesystem operation, never per command. It is
    /// idempotent: leasing an already-leased bus is a no-op rather than a deadlock,
    /// so a nested helper cannot hang the caller that already holds it.
    pub async fn lease(&self) {
        if self.held.borrow().is_some() {
            return;
        }
        let guard = self.bus.lock().await;
        *self.held.borrow_mut() = Some(guard);
    }

    /// [`lease`](Self::lease), but give up rather than wait forever.
    ///
    /// Returns whether the bus was obtained. This exists because an unbounded wait here
    /// is not a slow path, it is a wedged task: `embassy_sync::mutex::Mutex` holds a
    /// single `WakerRegistration`, so a second waiter displaces the first, and a holder
    /// that re-locks promptly in a loop -- which the 10 ms display task does -- can
    /// starve a waiter indefinitely. The waiter has no way to tell that apart from a
    /// holder that has stopped releasing at all.
    ///
    /// Cancelling the `lock()` future is safe: it only deregisters the waker, and the
    /// guard is only stored once actually acquired, so a timeout cannot leave the bus
    /// half-taken.
    pub async fn lease_within(&self, timeout: embassy_time::Duration) -> bool {
        if self.held.borrow().is_some() {
            return true;
        }
        match embassy_time::with_timeout(timeout, self.bus.lock()).await {
            Ok(guard) => {
                *self.held.borrow_mut() = Some(guard);
                true
            }
            Err(_) => false,
        }
    }

    /// Give the bus back so the display can draw again.
    pub fn release(&self) {
        self.held.borrow_mut().take();
    }

    /// Whether the bus is currently held.
    pub fn is_leased(&self) -> bool {
        self.held.borrow().is_some()
    }

    /// A handle to hand to `sdio::spi::SpiMmcBus`.
    pub fn handle(&'a self) -> SpiBusLease<'a, M, BUS> {
        SpiBusLease { shared: self }
    }
}

/// The `SpiBus` implementation `sdio` drives, reading through the held guard.
///
/// Every method fails with [`SpiLeaseError::NotLeased`] when the bus has not been
/// leased, rather than blocking to acquire it. Acquiring here would reintroduce
/// exactly the hazard this module exists to prevent: the lock would be taken and
/// dropped *inside* a command, between CS going low and CS coming back up.
pub struct SpiBusLease<'a, M: RawMutex, BUS> {
    shared: &'a SharedSpiBus<'a, M, BUS>,
}

impl<'a, M: RawMutex, BUS> ErrorType for SpiBusLease<'a, M, BUS> {
    type Error = SpiLeaseError;
}

/// Run `$op` against the leased bus, mapping both failure modes onto `SpiLeaseError`.
///
/// The `RefMut` is deliberately held across the await: the borrow *is* the exclusive
/// access, and dropping it mid-transfer would let another borrower see a bus that is
/// halfway through a command.
macro_rules! with_bus {
    ($self:expr, |$bus:ident| $op:expr) => {{
        let mut held = $self.shared.held.borrow_mut();
        let $bus = held.as_mut().ok_or(SpiLeaseError::NotLeased)?;
        $op.await.map_err(|_| SpiLeaseError::Bus)
    }};
}

impl<'a, M: RawMutex, BUS> SpiBus<u8> for SpiBusLease<'a, M, BUS>
where
    BUS: SpiBus<u8>,
{
    async fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        with_bus!(self, |bus| bus.read(words))
    }

    async fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        with_bus!(self, |bus| bus.write(words))
    }

    async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        with_bus!(self, |bus| bus.transfer(read, write))
    }

    async fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        with_bus!(self, |bus| bus.transfer_in_place(words))
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        with_bus!(self, |bus| bus.flush())
    }
}

/// Lets `sdio` run identification at 400 kHz and then move to the operating clock.
///
/// This is the half of the fix that the old code was missing outright: the bus was
/// fixed at 10 MHz and the card was expected to enumerate there, which the SD physical
/// specification does not permit -- CMD0/CMD8/ACMD41 require 100-400 kHz. `sdio` drives
/// this itself via its `INIT_FREQ`, so the frequency is never chosen here.
impl<'a, M: RawMutex, BUS> sdio::spi::SetHz for SpiBusLease<'a, M, BUS>
where
    BUS: SetConfig<Config = SpiConfig>,
{
    fn set_hz(&mut self, hz: u32) {
        let mut config = self.shared.base_config.borrow().clone();
        config.frequency = hz;

        let mut held = self.shared.held.borrow_mut();
        let Some(bus) = held.as_mut() else {
            // Not reachable through `sdio`, which only reconfigures inside an
            // operation -- but `set_hz` cannot report an error, so say so and leave
            // the bus alone rather than pretending the clock changed.
            log_warn!("SD: set_hz({}) with the bus unleased; ignored", hz);
            return;
        };

        if bus.set_config(&config).is_err() {
            log_warn!("SD: SPI reconfiguration to {} Hz failed", hz);
            return;
        }

        *self.shared.base_config.borrow_mut() = config;
    }
}
