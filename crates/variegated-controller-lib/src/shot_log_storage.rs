//! Shot logs on an exFAT SD card.
//!
//! A stored shot is exactly `postcard::to_allocvec_crc32(&ShotLog)` -- the postcard
//! encoding followed by a CRC-32/ISCSI trailer, no header, magic or container of our
//! own. That is deliberate: whatever the download path hands to a host *is* the record
//! as it sits on the card, so there is nothing to keep in sync between the two.
//!
//! The layout is `SHOTS/<YYYYMMDD>/<HHMMSSxx>.BIN`, spelled out by
//! [`ShotLogId`]. Shots taken before the clock has synced go to `SHOTS/NODATE/`.
//! A day directory exists because filenames carry only a time of day, so a flat
//! directory silently collides across days -- which is what the previous
//! implementation did, with a `generate_subdir_name` that described this layout and was
//! never called.
//!
//! # Locking
//!
//! Every method here brackets its work with [`SharedSpiBus::lease`] and
//! `release`, because the card shares the display's SPI bus and an SD command spans
//! several bus transfers with CS held low throughout. See [`crate::sd_card`] for why
//! that has to be the granularity. The bracket is written out at each entry point
//! rather than wrapped in a guard type: `Drop` cannot be async, and a guard that
//! released on drop would still be correct only because release happens to be
//! synchronous -- which is a property a future refactor could remove silently.

extern crate alloc;

use alloc::vec::Vec;
use core::fmt::Debug;

use crc::{Crc, CRC_32_ISCSI};
use exfat_slim::asynchronous::file::OpenOptions;
use exfat_slim::asynchronous::file_system::FileSystem;
use exfat_slim::asynchronous::BlockDevice;
use variegated_controller_types::shot_log::{ShotLogId, SHOTS_DIR};
use variegated_controller_types::{
    ShotAnnotations, ShotLog, ShotLogList, ShotLogListEntry, SHOT_LOG_FORMAT_VERSION,
};
use variegated_log::{log_debug, log_warn};

use crate::sd_card::SharedSpiBus;

/// Bytes per block. exFAT calls these sectors; the card calls them blocks.
pub const BLOCK_SIZE: usize = 512;

/// Sector cache depth handed to `exfat_slim::FileSystem`.
///
/// It is instantiated three times inside the filesystem -- the FAT, the allocation
/// bitmap and the data-block cache each get one -- so the cost is roughly
/// `3 * SLOT_CACHE * BLOCK_SIZE` bytes, here about 12 kB.
///
/// That cost lands wherever the `SdShotLogStorage` lives. Held directly in the storage
/// task it sits in the executor's task arena, which is SRAM and is sized by
/// `EMBASSY_EXECUTOR_TASK_ARENA_SIZE`; boxed, it goes to the heap, which on this board
/// is backed by the RP2350's 8 MB of PSRAM (the display's two 143,808-byte buffers are
/// already allocated that way). Boxing is the cheaper option if the arena gets tight --
/// the SD path is not latency-critical enough for the indirection to matter.
///
/// Eight rather than two because every extra slot is a directory or FAT sector not
/// re-read over a 10 MHz bus, and a listing walks a lot of both.
pub const SLOT_CACHE: usize = 8;

/// How many bytes a single `read_chunk` will return.
///
/// Re-exported rather than defined here: it is the capacity of the `heapless::Vec` in the
/// inter-processor chunk message, so it belongs with the wire types, where both
/// processors can see the same constant.
pub use variegated_controller_types::shot_log::SHOT_LOG_CHUNK_LEN;

/// How much of a stored shot is read to recover its annotations.
///
/// A maximal annotation block encodes to 545 bytes -- eight entries, every string at its
/// bound -- so a kilobyte is roughly twice what is ever needed. It is read as a fixed
/// prefix rather than incrementally because a short read costs a second round trip over
/// a 10 MHz bus and a listing does one of these per file.
///
/// This works only because of the file's field order: [`ShotLog::version`] first, then
/// `metadata`, whose own first field is `annotations`. postcard is sequential, so those
/// two values sit at offset zero and decode without touching the samples that follow.
/// `variegated_controller_types::shot_log`'s `annotations_decode_from_a_prefix` test is
/// what holds that property in place.
///
/// The prefix read deliberately skips CRC verification. The trailer covers the whole
/// file and cannot be checked from a prefix, so corruption surfaces on download rather
/// than in the list -- which is the right place for it, since a listing that failed on
/// one bad file would hide every good one.
pub const SHOT_LOG_ANNOTATION_PREFIX_LEN: usize = 1024;

/// How long an operation waits for the display's SPI bus before giving up.
///
/// Generous, because a display flush of the full 143,808-byte framebuffer at 10 MHz is
/// itself over 100 ms and must not be mistaken for a stuck bus -- but finite, because an
/// unbounded wait here wedges the storage task with no diagnostic at all.
pub const BUS_LEASE_TIMEOUT: embassy_time::Duration = embassy_time::Duration::from_secs(2);

/// Re-exported from its home in `variegated-controller-types`.
///
/// It lives there rather than here because it travels on the inter-processor wire, in
/// `ApplicationProcessorToCommsProcessorMessage::ShotLogError` -- the comms processor
/// needs to tell "the card is not there" from "the application processor never answered",
/// and without a carried reason every request against an empty slot would cost a full
/// timeout and report nothing useful. Re-exported so the many `use
/// crate::shot_log_storage::ShotLogStorageError` paths in this crate keep working, and so
/// that this module still reads as the place the storage vocabulary is defined.
pub use variegated_controller_types::ShotLogStorageError;

/// Outcome of one [`ShotLogStorage::read_chunk`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
// Derived unconditionally: this crate depends on defmt outright and has no `defmt`
// feature, so a `cfg_attr` guard here would simply never fire.
#[derive(defmt::Format)]
pub struct ChunkRead {
    /// Bytes written into the caller's buffer.
    pub len: usize,
    /// Total size of the record, so a caller can tell how far along it is without a
    /// separate stat call.
    pub total: u32,
    /// Whether this chunk reached the end of the record.
    pub last: bool,
}

/// How many bytes the self-test writes.
///
/// **This must exceed one exFAT cluster, and the reason is a bug it failed to catch.**
/// The pattern used to be 4096 bytes, which fits inside a single cluster on every card --
/// so a write path that filled the first cluster and then wrote nothing at all passed the
/// self-test cleanly while silently truncating every shot longer than 32 KiB.
///
/// The card cannot be asked its cluster size: `exfat_slim::FileSystem` keeps
/// `FileSystemDetails` behind a `pub(crate)` field. So the size is chosen to beat the
/// largest cluster anyone will plausibly format:
///
/// | Volume size | Cluster (Windows/macOS default) | Clusters spanned here |
/// |---|---|---|
/// | 256 MB - 32 GB | 32 KiB | 16 |
/// | 32 GB - 1 TB and beyond | 128 KiB | 4 |
///
/// Several clusters rather than a bare two, so the test covers *continuing* across
/// boundaries and not merely reaching one -- a chain that advances once and then stops
/// would pass a two-cluster test.
///
/// The costs, both acceptable for a command someone types deliberately: one 512 KiB
/// allocation, which lands on the heap and therefore in the RP2350's 8 MB of PSRAM rather
/// than in the executor's task arena; and a second or two of bus time each way at 10 MHz,
/// during which the self-test holds the display's SPI lease and the panel does not
/// update.
pub const SELF_TEST_PATTERN_LEN: usize = 512 * 1024;

/// The byte the self-test expects at `offset`.
///
/// A counting pattern rather than a constant fill: a constant cannot distinguish a
/// correct read from a misaligned one, a repeated block, or a stale cache slot. The
/// multiply changes both low and high bits across a 512-byte boundary, and -- unlike a
/// bare `offset as u8` -- does not repeat every 256 bytes, so a block written to the
/// wrong sector shows up as a mismatch rather than as coincidentally correct data.
///
/// A function rather than a second buffer: verification regenerates each byte instead of
/// holding a 384 KiB copy of what was written, which halves the peak allocation and lets
/// the comparison report the exact offset it first diverged at.
pub fn self_test_byte(offset: usize) -> u8 {
    (offset.wrapping_mul(31).wrapping_add(7) & 0xFF) as u8
}

/// Result of [`SdShotLogStorage::self_test`], one flag per step.
///
/// Ordered as the steps run, and every flag starts `false`, so the report reads as a
/// high-water mark: the first `false` is where it stopped. A single pass/fail bool
/// would say "the card does not work" without saying whether the card, the
/// filesystem, or the seek arithmetic is at fault -- which is the whole question.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[derive(defmt::Format)]
pub struct SelfTestReport {
    /// The volume mounted. `false` here with a card inserted almost always means the
    /// card is FAT32 rather than exFAT.
    pub mounted: bool,
    /// `SHOTS/` exists or could be created.
    pub directory: bool,
    /// The scratch file was written and closed.
    pub wrote: bool,
    /// The size of the pattern this run used, so the report says what it set out to
    /// cover rather than leaving the reader to assume [`SELF_TEST_PATTERN_LEN`].
    ///
    /// Not "bytes written" -- it is populated before the write is attempted, and reads
    /// the full pattern length even when `wrote` is `false`. Naming it otherwise made a
    /// failed run report `bytes_written: 524288` beside `wrote: false`, which is a
    /// contradiction a reader has to stop and resolve.
    pub pattern_len: u32,
    /// Its length matched what was written.
    pub size_matches: bool,
    /// It could be read back in full through a fresh handle.
    pub read_back: bool,
    /// The bytes read back were the bytes written.
    pub contents_match: bool,
    /// Offset of the first byte that read back wrong.
    ///
    /// The single most useful number in this report when it fails, and the one whose
    /// absence cost a laptop and a hex dump to recover: a truncated write reports the
    /// exact boundary it stopped at, which for the bug this test now exists to catch is
    /// the cluster size.
    pub first_mismatch: Option<u32>,
    /// How many bytes read back before the file ended early, if it did.
    ///
    /// Distinct from `first_mismatch`: a short read means the filesystem stopped
    /// supplying data, whereas a mismatch means it supplied the wrong data. The
    /// truncated-write bug produces a *mismatch* against a full-length file, because the
    /// unwritten tail reads back as zeros rather than as end-of-file.
    pub read_back_len: u32,
    /// Every seek probe landed on the right byte.
    ///
    /// Probes straddle each plausible cluster boundary, not just one arbitrary offset --
    /// see `SEEK_PROBE_OFFSETS`.
    pub seek_accurate: bool,
    /// The directory could be listed.
    pub listed: bool,
    /// How many entries the listing saw.
    pub listed_entries: usize,
    /// The scratch file was removed.
    pub deleted: bool,
    /// And is really gone.
    pub gone_after_delete: bool,
}

impl SelfTestReport {
    /// Whether every step passed.
    pub fn passed(&self) -> bool {
        self.mounted
            && self.directory
            && self.wrote
            && self.size_matches
            && self.read_back
            && self.contents_match
            && self.seek_accurate
            && self.listed
            && self.deleted
            && self.gone_after_delete
    }
}

/// Persistent storage for completed shot logs.
///
/// Async because the card is reached over an async SPI bus. The previous version of
/// this trait was synchronous, and bridging that gap with `block_on` is what
/// deadlocked core 1.
#[allow(async_fn_in_trait)]
pub trait ShotLogStorage {
    /// Store a completed shot, returning where it landed.
    async fn store_shot(&mut self, shot: &ShotLog) -> Result<ShotLogId, ShotLogStorageError>;

    /// The most recent `limit` shots, newest first, with their annotations.
    async fn list_shots(&mut self, limit: usize) -> Result<ShotLogList, ShotLogStorageError>;

    /// Read up to `buf.len()` bytes of a stored record starting at `offset`.
    async fn read_chunk(
        &mut self,
        id: ShotLogId,
        offset: u32,
        buf: &mut [u8],
    ) -> Result<ChunkRead, ShotLogStorageError>;

    /// The annotations on one stored shot.
    async fn read_annotations(
        &mut self,
        id: ShotLogId,
    ) -> Result<ShotAnnotations, ShotLogStorageError>;

    /// Replace the annotations on one stored shot, rewriting the whole record.
    async fn set_annotations(
        &mut self,
        id: ShotLogId,
        annotations: ShotAnnotations,
    ) -> Result<(), ShotLogStorageError>;

    /// Delete a stored shot.
    async fn delete_shot(&mut self, id: ShotLogId) -> Result<(), ShotLogStorageError>;

    /// Whether a card is believed to be present and usable.
    fn is_available(&self) -> bool;
}

/// Serialize a shot into the on-card representation.
///
/// Free-standing and `pub` so the host tests can build a record without a card.
pub fn encode_shot(shot: &ShotLog) -> Result<Vec<u8>, ShotLogStorageError> {
    let crc = Crc::<u32>::new(&CRC_32_ISCSI);
    postcard::to_allocvec_crc32(shot, crc.digest())
        .map_err(|_| ShotLogStorageError::SerializationError)
}

/// Parse the on-card representation, verifying the CRC trailer **and the format version**.
///
/// The version check is not belt-and-braces. postcard is positional, so decoding a file
/// written in another format does not fail -- it reads each field's bytes as the next
/// field's and returns a `ShotLog` full of plausible nonsense. `version` is the first
/// value in the file precisely so that this function can reject it before any of that
/// matters.
pub fn decode_shot(data: &[u8]) -> Result<ShotLog, ShotLogStorageError> {
    let crc = Crc::<u32>::new(&CRC_32_ISCSI);
    let shot: ShotLog = match postcard::from_bytes_crc32(data, crc.digest()) {
        Ok(shot) => shot,
        // A failed decode is not automatically damage, and saying so was wrong in the one
        // case that matters most: a file from an *earlier* format. Version 3 gained a
        // trailing metadata field in version 4, so a version 3 file read as version 4 runs
        // out of bytes and fails here -- never reaching the version check below, and being
        // reported to the user as "that shot is corrupt on the card" when it is a perfectly
        // good shot this build simply cannot read.
        Err(_) => return Err(classify_decode_failure(data, &crc)),
    };

    if !shot.version_supported() {
        log_warn!(
            "SD: shot log is format version {}, this build reads {}",
            shot.version,
            SHOT_LOG_FORMAT_VERSION
        );
        return Err(ShotLogStorageError::UnsupportedVersion);
    }
    Ok(shot)
}

/// Why a decode that got past the CRC flavor failed.
///
/// Re-checks the trailer by hand, which is the only way to tell the two apart: the CRC
/// flavor reports one error for "the checksum did not match" and for "the checksum matched
/// and the bytes still would not parse", and those want opposite things said about them.
///
/// Order matters and follows the same reasoning as the host tool's: damage is reported as
/// damage first. A corrupted file can easily present a plausible leading version byte, and
/// "recorded by a different firmware version" would send someone looking for a format
/// problem they do not have.
fn classify_decode_failure(data: &[u8], crc: &Crc<u32>) -> ShotLogStorageError {
    const TRAILER_LEN: usize = 4;

    if data.len() < TRAILER_LEN {
        return ShotLogStorageError::CrcError;
    }
    let (body, trailer) = data.split_at(data.len() - TRAILER_LEN);
    // Little-endian, because that is how postcard's CRC flavor writes it.
    let stored = u32::from_le_bytes([trailer[0], trailer[1], trailer[2], trailer[3]]);
    if crc.checksum(body) != stored {
        return ShotLogStorageError::CrcError;
    }

    // Intact bytes that will not parse. The leading varint is the version, and it is the
    // first thing in the file precisely so it can be read without understanding the rest.
    match postcard::take_from_bytes::<u32>(body) {
        Ok((version, _rest)) if version != SHOT_LOG_FORMAT_VERSION => {
            log_warn!(
                "SD: shot log is format version {}, this build reads {}",
                version,
                SHOT_LOG_FORMAT_VERSION
            );
            ShotLogStorageError::UnsupportedVersion
        }
        _ => ShotLogStorageError::SerializationError,
    }
}

/// SD-card-backed [`ShotLogStorage`].
///
/// `BD` is the block device, which in the firmware is
/// `sdio::BlockDevice<sdio::sd::Card, SpiMmcBus<SpiBusLease<..>, .., Delay>, Delay, 512>`
/// -- but it is left generic so the host tests can drive the whole layer against a
/// file-backed device with no SD or SPI code linked at all. That seam is the reason
/// this logic is testable now and was not before.
pub struct SdShotLogStorage<'a, BD, M, BUS>
where
    BD: BlockDevice<BLOCK_SIZE>,
    M: embassy_sync::blocking_mutex::raw::RawMutex,
{
    fs: FileSystem<BD, BLOCK_SIZE, SLOT_CACHE>,
    /// `None` in host tests, where there is no bus to arbitrate.
    bus: Option<&'a SharedSpiBus<'a, M, BUS>>,
    available: bool,
}

impl<'a, BD, M, BUS> SdShotLogStorage<'a, BD, M, BUS>
where
    BD: BlockDevice<BLOCK_SIZE>,
    BD::Error: Debug,
    M: embassy_sync::blocking_mutex::raw::RawMutex,
{
    pub fn new(device: BD, bus: Option<&'a SharedSpiBus<'a, M, BUS>>) -> Self {
        Self {
            fs: FileSystem::new(device),
            bus,
            available: true,
        }
    }

    /// Take the SPI bus for the duration of one filesystem operation.
    ///
    /// Bounded rather than an open wait: the display re-locks this bus every 10 ms, and
    /// `embassy_sync::mutex::Mutex` keeps only one waker, so losing the race repeatedly
    /// looks exactly like the bus never being released. Failing after
    /// [`BUS_LEASE_TIMEOUT`] turns that into a logged error and a task that is still
    /// answering requests, instead of one wedged mid-operation with nothing to say.
    async fn lease(&self) -> bool {
        match self.bus {
            Some(bus) => bus.lease_within(BUS_LEASE_TIMEOUT).await,
            // Host tests have no bus to arbitrate.
            None => true,
        }
    }

    fn release(&self) {
        if let Some(bus) = self.bus {
            bus.release();
        }
    }

    /// Tell the storage the card went away, so the next operation re-mounts instead of
    /// trusting cached filesystem state from a card that is no longer there.
    pub fn set_card_present(&mut self, present: bool) {
        if self.available != present {
            log_debug!("SD: card presence -> {}", present);
        }
        self.available = present;
    }

    /// Exercise the whole card path and report what worked.
    ///
    /// This exists because "is the SD card working" was previously answered by pulling
    /// a shot and seeing whether a file appeared -- a slow, destructive test with a
    /// dozen ways to be inconclusive. Every step below is one the real path depends on,
    /// in the order it depends on them, so the first `false` in the report names the
    /// layer that is broken.
    ///
    /// The scratch file is `SELFTEST.BIN`, which is deliberately *not* a valid shot
    /// name: [`ShotLogId::parse_file_name`] requires eight digits, so a leftover from
    /// an interrupted run is invisible to `list_shots` and can never be mistaken for a
    /// shot. It is written under `SHOTS/` rather than the root so the test also proves
    /// directory creation, which is where a read-only or full card first fails.
    pub async fn self_test(&mut self) -> SelfTestReport {
        if !self.lease().await {
            log_warn!("SD self-test: could not take the SPI bus");
            // Every flag stays false, so the report reads as "stopped before mounting"
            // -- which is exactly what happened.
            return SelfTestReport::default();
        }
        let report = self.self_test_inner().await;
        self.release();
        report
    }

    async fn self_test_inner(&mut self) -> SelfTestReport {
        const PATTERN_LEN: usize = SELF_TEST_PATTERN_LEN;
        let pattern: Vec<u8> = (0..PATTERN_LEN).map(self_test_byte).collect();

        let mut report = SelfTestReport::default();
        report.pattern_len = PATTERN_LEN as u32;

        if self.mount().await.is_err() {
            return report;
        }
        report.mounted = true;

        let mut shots_root = alloc::string::String::from("/");
        shots_root.push_str(SHOTS_DIR);
        if self.ensure_dir(&shots_root).await.is_err() {
            return report;
        }
        report.directory = true;

        let mut path = shots_root.clone();
        path.push_str("/SELFTEST.BIN");

        // Write.
        //
        // Each failure is logged with its cause. These three steps previously discarded
        // the error and simply returned a `false` flag, which says the write failed but
        // not whether the file could not be created, the data could not be written, or
        // the close could not flush -- three different faults with one symptom.
        let opts = OpenOptions::new().write(true).create(true).truncate(true);
        let mut file = match self.fs.open(&path, opts).await {
            Ok(f) => f,
            Err(e) => {
                log_warn!(
                    "SD self-test: create failed: {:?}",
                    defmt::Debug2Format(&e)
                );
                return report;
            }
        };
        if let Err(e) = file.write(&mut self.fs, &pattern).await {
            log_warn!("SD self-test: write failed: {:?}", defmt::Debug2Format(&e));
            let _ = file.close(&mut self.fs).await;
            return report;
        }
        if let Err(e) = file.close(&mut self.fs).await {
            log_warn!(
                "SD self-test: close/flush failed: {:?}",
                defmt::Debug2Format(&e)
            );
            return report;
        }
        report.wrote = true;

        // Between a successful close and a failing reopen there are two very different
        // possibilities: the directory entry never reached the card, or it did and the
        // lookup cannot see it. Ask both ways before moving on, because the answer picks
        // the layer to investigate -- and neither question can be asked afterwards, once
        // the reopen has already failed.
        match self.fs.exists(&path).await {
            Ok(present) => log_debug!("SD self-test: exists({}) = {}", path.as_str(), present),
            Err(e) => log_warn!(
                "SD self-test: exists() failed: {:?}",
                defmt::Debug2Format(&e)
            ),
        }
        match self.dir_names(&shots_root).await {
            Ok(names) => {
                log_debug!("SD self-test: {} entry/entries in SHOTS/", names.len());
                for name in names.iter() {
                    log_debug!("SD self-test:   {}", name.as_str());
                }
            }
            Err(e) => log_warn!(
                "SD self-test: listing SHOTS/ failed: {:?}",
                defmt::Debug2Format(&e)
            ),
        }

        // Read back, in chunks, from a freshly opened handle -- reading through the
        // handle we just wrote would be satisfied from the cache and prove nothing
        // about what reached the card.
        let mut file = match self.fs.open(&path, OpenOptions::new().read(true)).await {
            Ok(f) => f,
            Err(e) => {
                log_warn!(
                    "SD self-test: reopen failed: {:?}",
                    defmt::Debug2Format(&e)
                );
                return report;
            }
        };
        let len = file.metadata().len();
        report.size_matches = len == PATTERN_LEN as u64;
        if !report.size_matches {
            log_warn!(
                "SD self-test: size {} bytes, expected {}",
                len,
                PATTERN_LEN as u64
            );
        }

        // Read back a chunk at a time and compare against the generator rather than
        // against a second 384 kB buffer. Two reasons: peak memory stays at one copy of
        // the pattern, and the comparison can name the exact offset it first diverged at
        // -- which for a truncated write *is* the cluster size, the number that otherwise
        // takes a card reader and a hex dump to recover.
        let mut scratch = alloc::vec![0u8; 512];
        let mut cursor = 0usize;
        let mut read_ok = true;
        while cursor < PATTERN_LEN {
            let want = core::cmp::min(scratch.len(), PATTERN_LEN - cursor);
            match file.read(&mut self.fs, &mut scratch[..want]).await {
                // End of file before the length said so. Recorded via `read_back_len`
                // rather than as a mismatch: the filesystem stopping is a different fault
                // from the filesystem lying.
                Ok(Some(0)) | Ok(None) => break,
                Ok(Some(n)) => {
                    if report.first_mismatch.is_none() {
                        // Compared byte by byte against the generator. An explicit loop
                        // rather than a zip-and-position so the offset it reports is the
                        // absolute one in the file, not one relative to this chunk.
                        for (i, &got) in scratch[..n].iter().enumerate() {
                            if got != self_test_byte(cursor + i) {
                                report.first_mismatch = Some((cursor + i) as u32);
                                break;
                            }
                        }
                    }
                    cursor += n;
                }
                Err(_) => {
                    read_ok = false;
                    break;
                }
            }
        }
        report.read_back_len = cursor as u32;
        report.read_back = read_ok && cursor == PATTERN_LEN;
        report.contents_match = report.read_back && report.first_mismatch.is_none();

        if let Some(offset) = report.first_mismatch {
            // Logged separately from the report, and loudly, because this is the line
            // that identifies the bug: a mismatch at a power-of-two offset with the rest
            // of the file reading back as zeros is a write that stopped at a cluster
            // boundary, not a corrupted byte.
            log_warn!(
                "SD self-test: contents diverge at offset {} of {} -- if this is a round \
                 power of two, the write stopped at a cluster boundary",
                offset,
                PATTERN_LEN as u32
            );
        }

        // Seek probes. The chunked download path is built entirely on `seek`, and an
        // off-by-one there still produces plausible-looking data.
        //
        // The offsets straddle every cluster size a card is likely to be formatted with,
        // rather than sampling one arbitrary place: a seek that cannot cross a cluster
        // boundary fails at exactly those offsets and nowhere else, and the old single
        // probe at 1337 sat inside the first cluster where nothing can go wrong.
        const SEEK_PROBE_OFFSETS: &[usize] = &[
            1337,             // unaligned, inside the first cluster
            32 * 1024 - 1,    // last byte of a 32 KiB cluster
            32 * 1024,        // first byte of the next one
            32 * 1024 + 1,
            64 * 1024,
            128 * 1024 - 1,   // and the same for a 128 KiB cluster
            128 * 1024,
            128 * 1024 + 1,
            256 * 1024,       // second and third 128 KiB boundaries, so the probes reach
            384 * 1024,       // past a chain that advances once and then stops
            PATTERN_LEN - 1,  // the very last byte
        ];
        report.seek_accurate = true;
        for &probe in SEEK_PROBE_OFFSETS {
            if probe >= PATTERN_LEN {
                continue;
            }
            let mut one = [0u8; 1];
            let ok = file.seek(&mut self.fs, probe as u64).await.is_ok()
                && matches!(file.read(&mut self.fs, &mut one).await, Ok(Some(1)))
                && one[0] == self_test_byte(probe);
            if !ok {
                log_warn!("SD self-test: seek probe at offset {} failed", probe as u32);
                report.seek_accurate = false;
            }
        }
        let _ = file.close(&mut self.fs).await;

        // List. The scratch file is not a valid shot name, so it must NOT appear.
        if let Ok(names) = self.dir_names(&shots_root).await {
            report.listed_entries = names.len();
            report.listed = true;
        }

        report.deleted = self.fs.remove_file(&path).await.is_ok();
        report.gone_after_delete =
            report.deleted && matches!(self.fs.exists(&path).await, Ok(false));

        report
    }

    /// Hand the block device back, discarding all filesystem state.
    ///
    /// This is the only correct way to survive a card swap. `FileSystem` caches the
    /// boot sector, the upcase table and the allocation bitmap after its first
    /// successful `mount`, and nothing invalidates that on its own -- so a second card
    /// would be addressed through the first one's geometry, reading real data from the
    /// wrong offsets instead of failing. Dropping the whole filesystem and building a
    /// new one over the same device is what guarantees the next `mount` re-reads
    /// everything.
    pub fn into_device(self) -> BD {
        self.fs.unmount()
    }

    /// Mount the volume, mapping the failure that actually happens in the field.
    async fn mount(&mut self) -> Result<(), ShotLogStorageError> {
        self.fs.mount().await.map_err(|e| {
            log_warn!("SD: mount failed: {:?}", defmt::Debug2Format(&e));
            ShotLogStorageError::NotExfat
        })
    }

    /// Create `dir` if it is not already there.
    async fn ensure_dir(&mut self, dir: &str) -> Result<(), ShotLogStorageError> {
        match self.fs.exists(dir).await {
            Ok(true) => Ok(()),
            Ok(false) => self.fs.create_directory(dir).await.map_err(|e| {
                log_warn!(
                    "SD: could not create {}: {:?}",
                    dir,
                    defmt::Debug2Format(&e)
                );
                ShotLogStorageError::DirectoryError
            }),
            Err(e) => {
                log_warn!(
                    "SD: could not stat {}: {:?}",
                    dir,
                    defmt::Debug2Format(&e)
                );
                Err(ShotLogStorageError::DirectoryError)
            }
        }
    }

    /// Names of the entries directly under `path`, unsorted.
    async fn dir_names(&mut self, path: &str) -> Result<Vec<alloc::string::String>, ShotLogStorageError> {
        let mut iter = self.fs.read_dir(path).await.map_err(|e| {
            log_warn!(
                "SD: could not list {}: {:?}",
                path,
                defmt::Debug2Format(&e)
            );
            ShotLogStorageError::DirectoryError
        })?;

        // The caller supplies the name buffer since exfat-slim 0.6.0; `DirectoryEntry`
        // borrows it rather than owning a `String`, which is what lets the iterator run
        // without allocating per entry.
        //
        // Sized at `MAX_NAME_LEN` (765 bytes, the longest exFAT name) rather than at
        // something that merely fits *our* names, which are 8 and 12 characters. A short
        // buffer returns `FileNameBufferTooSmall`, and the loop below treats an error as
        // fatal to the whole listing -- so one long-named file a user dropped on the card
        // would hide every shot on it. On the heap rather than the stack because this is
        // held across an await, and therefore lives in the storage task's future.
        let mut name_buf = alloc::vec![0u8; exfat_slim::asynchronous::directory::MAX_NAME_LEN];

        let mut out = Vec::new();
        loop {
            match iter.next_entry(&mut self.fs, &mut name_buf).await {
                // Copied out immediately: `entry.name` borrows `name_buf`, which the next
                // iteration overwrites.
                Ok(Some(entry)) => out.push(alloc::string::String::from(entry.name)),
                Ok(None) => break,
                Err(e) => {
                    log_warn!(
                        "SD: listing {} failed partway: {:?}",
                        path,
                        defmt::Debug2Format(&e)
                    );
                    return Err(ShotLogStorageError::DirectoryError);
                }
            }
        }
        Ok(out)
    }

    /// The body of `store_shot`, run with the bus already leased.
    async fn store_shot_inner(
        &mut self,
        shot: &ShotLog,
        id: ShotLogId,
    ) -> Result<ShotLogId, ShotLogStorageError> {
        // Encode before touching the card. A serialization failure must not be able to
        // leave a truncated file behind, and `truncate` below is destructive.
        let bytes = encode_shot(shot)?;

        self.mount().await?;

        let mut shots_root = alloc::string::String::from("/");
        shots_root.push_str(SHOTS_DIR);
        self.ensure_dir(&shots_root).await?;

        let mut day_dir = shots_root.clone();
        day_dir.push('/');
        day_dir.push_str(id.dir_name().as_str());
        self.ensure_dir(&day_dir).await?;

        let path = id.path();
        let options = OpenOptions::new()
            .write(true)
            .create(true)
            .truncate(true);

        let mut file = self
            .fs
            .open(path.as_str(), options)
            .await
            .map_err(|e| {
                log_warn!(
                    "SD: could not create {}: {:?}",
                    path.as_str(),
                    defmt::Debug2Format(&e)
                );
                ShotLogStorageError::WriteError
            })?;

        if let Err(e) = file.write(&mut self.fs, &bytes).await {
            log_warn!(
                "SD: write of {} failed: {:?}",
                path.as_str(),
                defmt::Debug2Format(&e)
            );
            // Still close, so the filesystem is not left holding an open handle.
            let _ = file.close(&mut self.fs).await;
            return Err(ShotLogStorageError::WriteError);
        }

        file.close(&mut self.fs).await.map_err(|e| {
            log_warn!(
                "SD: close of {} failed: {:?}",
                path.as_str(),
                defmt::Debug2Format(&e)
            );
            ShotLogStorageError::WriteError
        })?;

        log_debug!("SD: stored {} ({} bytes)", path.as_str(), bytes.len());
        Ok(id)
    }

    async fn list_shots_inner(
        &mut self,
        limit: usize,
    ) -> Result<ShotLogList, ShotLogStorageError> {
        self.mount().await?;

        let mut shots_root = alloc::string::String::from("/");
        shots_root.push_str(SHOTS_DIR);

        // A card with no SHOTS directory is empty, not broken -- a machine that has
        // never stored a shot must list cleanly rather than reporting an error.
        if !matches!(self.fs.exists(&shots_root).await, Ok(true)) {
            return Ok(ShotLogList {
                entries: Vec::new(),
                truncated: false,
            });
        }

        let mut days: Vec<alloc::string::String> = self
            .dir_names(&shots_root)
            .await?
            .into_iter()
            .filter(|name| ShotLogId::parse_dir_name(name).is_some())
            .collect();
        // Newest first. Fixed-width names make the string order the chronological
        // order; `NODATE` sorts after every digit, which puts undated shots last.
        days.sort_unstable_by(|a, b| b.cmp(a));

        let mut entries = Vec::new();
        let mut truncated = false;

        for day in days {
            let Some(day_value) = ShotLogId::parse_dir_name(&day) else {
                continue;
            };

            let mut day_path = shots_root.clone();
            day_path.push('/');
            day_path.push_str(&day);

            let mut files: Vec<alloc::string::String> = self
                .dir_names(&day_path)
                .await?
                .into_iter()
                .filter(|name| ShotLogId::parse_file_name(name).is_some())
                .collect();
            files.sort_unstable_by(|a, b| b.cmp(a));

            for file in files {
                if entries.len() >= limit {
                    truncated = true;
                    break;
                }
                let Some(time) = ShotLogId::parse_file_name(&file) else {
                    continue;
                };

                let mut path = day_path.clone();
                path.push('/');
                path.push_str(&file);

                // Size and annotations come from one open of the file rather than from
                // the directory entry plus a second open. `DirectoryEntry::metadata` and
                // `File::metadata` report the same `FileDetails`, so nothing is lost, and
                // opening a file on this filesystem is the expensive part.
                let mut file = match self.fs.open(&path, OpenOptions::new().read(true)).await {
                    Ok(f) => f,
                    Err(e) => {
                        // One unreadable shot must not fail the whole listing.
                        log_warn!(
                            "SD: skipping unreadable {}: {:?}",
                            path.as_str(),
                            defmt::Debug2Format(&e)
                        );
                        continue;
                    }
                };
                let size_bytes = file.metadata().len() as u32;
                // An annotation block that cannot be read leaves the entry with an empty
                // one rather than dropping the shot: the record is still downloadable,
                // and a missing row is a worse answer than a row with no beans on it.
                let annotations = self
                    .read_annotations_from(&mut file)
                    .await
                    .unwrap_or_default();
                let _ = file.close(&mut self.fs).await;

                entries.push(ShotLogListEntry {
                    id: ShotLogId {
                        day: day_value,
                        time,
                    },
                    size_bytes,
                    annotations,
                });
            }

            if truncated {
                break;
            }
        }

        if truncated {
            log_debug!("SD: listing capped at {} entries", limit);
        }

        Ok(ShotLogList {
            entries,
            truncated,
        })
    }

    /// Recover the annotation block from the front of an already-open record.
    ///
    /// Takes the file rather than a path so a listing pays for one open per shot instead
    /// of two. It reads from wherever the handle currently sits, which for a freshly
    /// opened file is offset zero -- the only offset at which this is meaningful.
    async fn read_annotations_from(
        &mut self,
        file: &mut exfat_slim::asynchronous::file::File,
    ) -> Result<ShotAnnotations, ShotLogStorageError> {
        let mut prefix = alloc::vec![0u8; SHOT_LOG_ANNOTATION_PREFIX_LEN];
        let mut filled = 0usize;
        // Loop rather than a single read: `read` is permitted to return a short count at
        // a cluster boundary, and a short first read would truncate the block mid-string.
        // A record shorter than the prefix ends early, which is not an error -- a shot
        // with no annotations and no samples is legitimately tiny.
        while filled < prefix.len() {
            match file.read(&mut self.fs, &mut prefix[filled..]).await {
                Ok(Some(0)) | Ok(None) => break,
                Ok(Some(n)) => filled += n,
                Err(e) => {
                    log_warn!(
                        "SD: reading the annotation prefix failed: {:?}",
                        defmt::Debug2Format(&e)
                    );
                    return Err(ShotLogStorageError::ReadError);
                }
            }
        }

        // Version first, then annotations -- the same two `take_from_bytes` calls the
        // format is arranged around, and the same ones
        // `annotations_decode_from_a_prefix` pins.
        let (version, rest) = postcard::take_from_bytes::<u32>(&prefix[..filled])
            .map_err(|_| ShotLogStorageError::SerializationError)?;

        if version != SHOT_LOG_FORMAT_VERSION {
            // Checked here as well as in `decode_shot`, because a listing never calls
            // that: it reads only this prefix. Without the check, a file from another
            // format would have its *next* field decoded as an annotation block and
            // appear in the list with invented dose and beans.
            log_warn!(
                "SD: skipping annotations of a version {} shot log (this build reads {})",
                version,
                SHOT_LOG_FORMAT_VERSION
            );
            return Err(ShotLogStorageError::UnsupportedVersion);
        }

        postcard::take_from_bytes::<ShotAnnotations>(rest)
            .map(|(annotations, _rest)| annotations)
            .map_err(|_| ShotLogStorageError::SerializationError)
    }

    async fn read_annotations_inner(
        &mut self,
        id: ShotLogId,
    ) -> Result<ShotAnnotations, ShotLogStorageError> {
        self.mount().await?;

        let path = id.path();
        let mut file = self
            .fs
            .open(path.as_str(), OpenOptions::new().read(true))
            .await
            .map_err(|_| ShotLogStorageError::NotFound)?;

        let result = self.read_annotations_from(&mut file).await;
        let _ = file.close(&mut self.fs).await;
        result
    }

    /// Read a whole record, replace its annotations, and write it back.
    ///
    /// Decode and re-encode rather than splicing new bytes over the old annotation block.
    /// Splicing would avoid materialising the sample vector, but it would also rewrite a
    /// file whose contents this firmware cannot read -- and the CRC trailer covers the
    /// whole record, so the bytes would have to be read in full either way. Failing the
    /// decode is the honest outcome: a record we cannot parse is one we have no business
    /// rewriting.
    ///
    /// **Not power-fail atomic.** The file is truncated before the new contents are
    /// written, so a brownout mid-edit loses that shot. Accepted with the whole-file
    /// layout: the alternative is a temporary file and a rename, which costs a second
    /// directory entry and a second failure mode on a filesystem whose own README warns
    /// that power loss during writes can leak clusters.
    async fn set_annotations_inner(
        &mut self,
        id: ShotLogId,
        annotations: ShotAnnotations,
    ) -> Result<(), ShotLogStorageError> {
        self.mount().await?;

        let path = id.path();

        let mut file = self
            .fs
            .open(path.as_str(), OpenOptions::new().read(true))
            .await
            .map_err(|_| ShotLogStorageError::NotFound)?;

        let total = file.metadata().len() as usize;
        let mut bytes = alloc::vec![0u8; total];
        let mut filled = 0usize;
        while filled < total {
            match file.read(&mut self.fs, &mut bytes[filled..]).await {
                Ok(Some(0)) | Ok(None) => break,
                Ok(Some(n)) => filled += n,
                Err(e) => {
                    log_warn!(
                        "SD: reading {} for annotation failed: {:?}",
                        path.as_str(),
                        defmt::Debug2Format(&e)
                    );
                    let _ = file.close(&mut self.fs).await;
                    return Err(ShotLogStorageError::ReadError);
                }
            }
        }
        let _ = file.close(&mut self.fs).await;

        if filled != total {
            log_warn!(
                "SD: {} is {} bytes but only {} could be read",
                path.as_str(),
                total,
                filled
            );
            return Err(ShotLogStorageError::ReadError);
        }

        // `decode_shot` refuses a file whose format version this build does not read, and
        // that refusal matters most here of all the places it applies. An edit is the one
        // operation that *writes back*: without the check, annotating a shot from another
        // format version would decode it wrongly, re-encode it in this one, and overwrite
        // the original -- destroying a file that was perfectly readable by the firmware
        // that wrote it. Reading such a shot merely fails; editing it would be
        // unrecoverable.
        //
        // The check happens before the file is reopened for writing, so a refusal leaves
        // the record exactly as it was.
        let mut shot = decode_shot(&bytes)?;
        shot.metadata.annotations = annotations;
        // Re-encoded before the file is touched, for the same reason `store_shot_inner`
        // encodes first: `truncate` is destructive, and a failure after it would leave a
        // shot that existed a moment ago as a zero-length file.
        let encoded = encode_shot(&shot)?;

        let options = OpenOptions::new().write(true).create(false).truncate(true);
        let mut file = self
            .fs
            .open(path.as_str(), options)
            .await
            .map_err(|e| {
                log_warn!(
                    "SD: could not reopen {} for writing: {:?}",
                    path.as_str(),
                    defmt::Debug2Format(&e)
                );
                ShotLogStorageError::WriteError
            })?;

        if let Err(e) = file.write(&mut self.fs, &encoded).await {
            log_warn!(
                "SD: rewriting {} failed: {:?}",
                path.as_str(),
                defmt::Debug2Format(&e)
            );
            let _ = file.close(&mut self.fs).await;
            return Err(ShotLogStorageError::WriteError);
        }

        file.close(&mut self.fs).await.map_err(|e| {
            log_warn!(
                "SD: close of {} failed: {:?}",
                path.as_str(),
                defmt::Debug2Format(&e)
            );
            ShotLogStorageError::WriteError
        })?;

        log_debug!(
            "SD: annotations rewritten on {} ({} bytes)",
            path.as_str(),
            encoded.len()
        );
        Ok(())
    }

    async fn read_chunk_inner(
        &mut self,
        id: ShotLogId,
        offset: u32,
        buf: &mut [u8],
    ) -> Result<ChunkRead, ShotLogStorageError> {
        self.mount().await?;

        let path = id.path();
        let mut file = self
            .fs
            .open(path.as_str(), OpenOptions::new().read(true))
            .await
            .map_err(|_| ShotLogStorageError::NotFound)?;

        let total = file.metadata().len();

        // Seeking past the end is an error in exfat-slim, but for a caller walking a
        // file to its end it is the natural terminating condition. Report it as an
        // empty final chunk instead of a failure.
        if offset as u64 >= total {
            let _ = file.close(&mut self.fs).await;
            return Ok(ChunkRead {
                len: 0,
                total: total as u32,
                last: true,
            });
        }

        if let Err(e) = file.seek(&mut self.fs, offset as u64).await {
            log_warn!(
                "SD: seek to {} in {} failed: {:?}",
                offset,
                path.as_str(),
                defmt::Debug2Format(&e)
            );
            let _ = file.close(&mut self.fs).await;
            return Err(ShotLogStorageError::ReadError);
        }

        let read = match file.read(&mut self.fs, buf).await {
            Ok(Some(n)) => n,
            // `None` is end of file.
            Ok(None) => 0,
            Err(e) => {
                log_warn!(
                    "SD: read of {} failed: {:?}",
                    path.as_str(),
                    defmt::Debug2Format(&e)
                );
                let _ = file.close(&mut self.fs).await;
                return Err(ShotLogStorageError::ReadError);
            }
        };

        let _ = file.close(&mut self.fs).await;

        Ok(ChunkRead {
            len: read,
            total: total as u32,
            last: offset as u64 + read as u64 >= total,
        })
    }

    async fn delete_shot_inner(&mut self, id: ShotLogId) -> Result<(), ShotLogStorageError> {
        self.mount().await?;
        let path = id.path();
        self.fs
            .remove_file(path.as_str())
            .await
            .map_err(|_| ShotLogStorageError::NotFound)
    }
}

impl<'a, BD, M, BUS> ShotLogStorage for SdShotLogStorage<'a, BD, M, BUS>
where
    BD: BlockDevice<BLOCK_SIZE>,
    BD::Error: Debug,
    M: embassy_sync::blocking_mutex::raw::RawMutex,
{
    async fn store_shot(&mut self, shot: &ShotLog) -> Result<ShotLogId, ShotLogStorageError> {
        if !self.available {
            return Err(ShotLogStorageError::CardNotPresent);
        }
        let id = current_shot_id(shot);

        if !self.lease().await {
            return Err(ShotLogStorageError::BusUnavailable);
        }
        let result = self.store_shot_inner(shot, id).await;
        self.release();
        result
    }

    async fn list_shots(&mut self, limit: usize) -> Result<ShotLogList, ShotLogStorageError> {
        if !self.available {
            return Err(ShotLogStorageError::CardNotPresent);
        }
        if !self.lease().await {
            return Err(ShotLogStorageError::BusUnavailable);
        }
        let result = self.list_shots_inner(limit).await;
        self.release();
        result
    }

    async fn read_annotations(
        &mut self,
        id: ShotLogId,
    ) -> Result<ShotAnnotations, ShotLogStorageError> {
        if !self.available {
            return Err(ShotLogStorageError::CardNotPresent);
        }
        if !self.lease().await {
            return Err(ShotLogStorageError::BusUnavailable);
        }
        let result = self.read_annotations_inner(id).await;
        self.release();
        result
    }

    async fn set_annotations(
        &mut self,
        id: ShotLogId,
        annotations: ShotAnnotations,
    ) -> Result<(), ShotLogStorageError> {
        if !self.available {
            return Err(ShotLogStorageError::CardNotPresent);
        }
        if !self.lease().await {
            return Err(ShotLogStorageError::BusUnavailable);
        }
        let result = self.set_annotations_inner(id, annotations).await;
        self.release();
        result
    }

    async fn read_chunk(
        &mut self,
        id: ShotLogId,
        offset: u32,
        buf: &mut [u8],
    ) -> Result<ChunkRead, ShotLogStorageError> {
        if !self.available {
            return Err(ShotLogStorageError::CardNotPresent);
        }
        if !self.lease().await {
            return Err(ShotLogStorageError::BusUnavailable);
        }
        let result = self.read_chunk_inner(id, offset, buf).await;
        self.release();
        result
    }

    async fn delete_shot(&mut self, id: ShotLogId) -> Result<(), ShotLogStorageError> {
        if !self.available {
            return Err(ShotLogStorageError::CardNotPresent);
        }
        if !self.lease().await {
            return Err(ShotLogStorageError::BusUnavailable);
        }
        let result = self.delete_shot_inner(id).await;
        self.release();
        result
    }

    fn is_available(&self) -> bool {
        self.available
    }
}

/// Where a shot should be filed, from the clock if it has one.
///
/// `start_time_millis` is monotonic uptime, not wall clock, so it cannot supply a date
/// on its own; it is only the fallback that keeps two undated shots from colliding.
///
/// The shot's own `recorded_at_unix_millis` is preferred over reading the clock again
/// here, and the difference is not cosmetic: this runs at *store* time, which is after
/// the shot ended and after the log made its way down a channel, so asking the clock
/// afresh names the moment the file was written rather than the moment the shot was
/// pulled. Since version 4 the blob carries the answer, and a filename that disagreed
/// with the timestamp inside it would be a genuinely confusing thing to ship.
fn current_shot_id(shot: &ShotLog) -> ShotLogId {
    use chrono::{DateTime, Datelike, Timelike, Utc};

    let recorded_at = shot
        .metadata
        .recorded_at_unix_millis
        .and_then(DateTime::<Utc>::from_timestamp_millis);

    match recorded_at.or_else(variegated_timekeeping::TimeKeeper::now_utc) {
        Some(utc) => ShotLogId {
            day: Some(
                utc.year() as u32 * 10_000 + utc.month() * 100 + utc.day(),
            ),
            // The two trailing digits disambiguate shots inside one second. Two shots
            // starting in the same second on one group is not physically possible, but
            // the id is also what a re-store would overwrite, so it costs nothing.
            time: utc.hour() * 1_000_000
                + utc.minute() * 10_000
                + utc.second() * 100
                + (shot.metadata.start_time_millis % 100) as u32,
        },
        None => ShotLogId {
            day: None,
            time: (shot.metadata.start_time_millis % 100_000_000) as u32,
        },
    }
}
