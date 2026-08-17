#![cfg_attr(not(test), no_std)]

//! Creating an exFAT volume from scratch.
//!
//! Cards ship FAT32 -- every SDHC card up to 32 GB does -- and `exfat-slim` cannot read
//! FAT32, so a new card is unusable until someone reformats it. This module is what lets
//! that happen on the machine instead of on a laptop.
//!
//! # Derived from a reference, not from the spec alone
//!
//! Every geometry rule here was measured against volumes produced by macOS's
//! `newfs_exfat` at 16 MB, 64 MB, 96 MB, 128 MB, 192 MB, 256 MB, 1 GB and 8 GB, rather
//! than inferred from the specification. The specification says what is *legal*; a
//! formatter that is merely legal produces volumes that mount here and behave oddly
//! elsewhere. The two rules that were not obvious beforehand:
//!
//! * `fat_length` is the FAT's own requirement rounded **up to 128 sectors**, with 128 as
//!   a floor -- not the exact requirement. A 256 MB volume needs 64 sectors of FAT and
//!   gets 128.
//! * The cluster heap begins immediately at `fat_offset + fat_length`, with no further
//!   padding.
//!
//! The up-case table is likewise extracted rather than written out; see
//! [`crate::exfat_upcase`] and `scripts/extract-upcase-table.py`.
//!
//! # Superfloppy, no partition table
//!
//! The volume starts at sector 0. That is what `newfs_exfat` produces for a raw device,
//! and [`crate::sd_card::probe_volume_start`] already handles both that and the
//! MBR-partitioned layout a card arrives with from the factory -- it looks for a volume
//! boot record before it parses an MBR, precisely because an exFAT boot sector also
//! carries `0x55AA` at offset 510.
//!
//! Formatting therefore *removes* any partition table that was there. That is intended:
//! the alternative is preserving a partition whose geometry was chosen for a filesystem
//! being replaced.

use core::fmt::Debug;

use aligned::{Aligned, A4};
use exfat_slim::asynchronous::BlockDevice;

pub mod upcase;

use crate::upcase::{UPCASE_TABLE, UPCASE_TABLE_CHECKSUM};

/// Bytes per sector. Fixed: SD cards are 512-byte addressed, and `BLOCK_SIZE` throughout
/// this crate assumes it.
const BYTES_PER_SECTOR: usize = 512;
/// `log2(BYTES_PER_SECTOR)`, as the boot sector stores it.
const BYTES_PER_SECTOR_SHIFT: u8 = 9;

/// Sectors occupied by one boot region: 1 boot sector, 8 extended, OEM parameters,
/// reserved, checksum.
const BOOT_REGION_SECTORS: u32 = 12;
/// Where the backup boot region begins. The main region is 0..12, the backup 12..24.
const BACKUP_BOOT_SECTOR: u32 = BOOT_REGION_SECTORS;

/// Sector alignment for the FAT on volumes below [`LARGE_VOLUME_SECTORS`].
///
/// Serves as the FAT's start sector, its minimum length, and the granularity its length
/// rounds up to -- all three are the same number on every reference volume. It only has to
/// clear the 24 sectors of boot regions; the rest is alignment headroom.
const SMALL_ALIGNMENT: u32 = 128;

/// The same, for volumes at or above [`LARGE_VOLUME_SECTORS`].
///
/// Puts the cluster heap at a 2 MiB boundary or beyond, which is flash-erase-block
/// friendly on the larger cards this applies to.
const LARGE_ALIGNMENT: u32 = 2048;

/// Where the alignment changes, in sectors: 4 GiB.
///
/// Bisected against `newfs_exfat`, not assumed -- a 3 GB volume gets `fat_offset` 128 and a
/// 4 GB volume gets 2048.
const LARGE_VOLUME_SECTORS: u64 = 4 * 1024 * 1024 * 1024 / BYTES_PER_SECTOR as u64;

/// FAT alignment for a volume of `sectors`.
fn fat_alignment(sectors: u64) -> u32 {
    if sectors >= LARGE_VOLUME_SECTORS {
        LARGE_ALIGNMENT
    } else {
        SMALL_ALIGNMENT
    }
}

/// exFAT revision 1.00, as the boot sector encodes it.
const FILE_SYSTEM_REVISION: u16 = 0x0100;

/// FAT entry values.
const FAT_MEDIA_DESCRIPTOR: u32 = 0xFFFF_FFF8;
const FAT_RESERVED: u32 = 0xFFFF_FFFF;
const FAT_END_OF_CHAIN: u32 = 0xFFFF_FFFF;

/// The first cluster a volume can address. Clusters 0 and 1 do not exist; the FAT's first
/// two entries are the media descriptor and a reserved value.
const FIRST_CLUSTER: u32 = 2;

/// Longest volume label exFAT can store, in UTF-16 code units.
const MAX_LABEL_LEN: usize = 11;

/// What went wrong.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FormatError {
    /// The device is too small to hold a boot region, a FAT and a usable heap.
    TooSmall,
    /// The device reported a size this code cannot represent, or zero.
    BadSize,
    /// A read or write failed.
    Io,
    /// The up-case table constant does not match its recorded checksum.
    ///
    /// A build-time fault rather than a runtime one -- the table is a `const` -- so this
    /// firing means the generated constant was edited or truncated. Checked anyway,
    /// because writing a table whose checksum does not describe it produces a volume that
    /// mounts and then matches filenames wrongly.
    UpcaseTableCorrupt,
}

/// Where everything lives on the volume being created.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Geometry {
    /// Total addressable sectors.
    pub volume_length: u64,
    /// First sector of the FAT.
    pub fat_offset: u32,
    /// FAT length in sectors.
    pub fat_length: u32,
    /// First sector of the cluster heap.
    pub cluster_heap_offset: u32,
    /// Clusters in the heap.
    pub cluster_count: u32,
    /// `log2(sectors per cluster)`.
    pub sectors_per_cluster_shift: u8,
    /// First cluster of the root directory.
    pub first_cluster_of_root: u32,
    /// First cluster of the allocation bitmap.
    pub bitmap_cluster: u32,
    /// Bitmap length in bytes: one bit per cluster.
    pub bitmap_length: u32,
    /// First cluster of the up-case table.
    pub upcase_cluster: u32,
}

impl Geometry {
    pub fn sectors_per_cluster(&self) -> u32 {
        1 << self.sectors_per_cluster_shift
    }

    pub fn bytes_per_cluster(&self) -> u32 {
        self.sectors_per_cluster() * BYTES_PER_SECTOR as u32
    }

    /// First sector of a cluster.
    pub fn cluster_sector(&self, cluster: u32) -> u32 {
        self.cluster_heap_offset + (cluster - FIRST_CLUSTER) * self.sectors_per_cluster()
    }

    /// Clusters needed to hold `bytes`.
    fn clusters_for(&self, bytes: u32) -> u32 {
        bytes.div_ceil(self.bytes_per_cluster()).max(1)
    }

    /// Sectors [`format`] writes for this layout.
    ///
    /// Exposed so a caller can size a timeout for a format it is about to run. The
    /// formatter writes one sector at a time throughout -- `put` is a single-block write --
    /// so duration tracks the *sector count*, not the byte count, and the FAT dominates it:
    ///
    /// | card | cluster size | sectors written |
    /// |---|---|---|
    /// | 16 GB | 32 KiB | 4,376 |
    /// | 32 GB | 128 KiB | 2,840 |
    /// | 128 GB | 128 KiB | 8,984 |
    /// | 512 GB | 128 KiB | 34,328 |
    /// | 1 TB | 128 KiB | 68,120 |
    ///
    /// A single constant is therefore wrong at both ends -- generous enough for the large
    /// card and it is no bound at all on the small one; tight enough for the small card
    /// and it aborts the large one part-written, which is a destroyed volume.
    ///
    /// Note that the sequence is **not monotonic in card size**: a 16 GB card writes more
    /// sectors than a 32 GB one, because the 32 KiB -> 128 KiB cluster step at 32 GB
    /// quarters the cluster count and so quarters the FAT. Anything scaling a bound by
    /// capacity rather than by this figure gets that backwards.
    ///
    /// This is the same arithmetic the writers use rather than an independent estimate, so
    /// the two cannot drift. `sectors_written_matches_the_format` holds that in place: it
    /// counts what a real format actually issues and requires this to be an exact match.
    pub fn sectors_written(&self) -> u32 {
        let fat = self.fat_length;
        let bitmap = self.clusters_for(self.bitmap_length) * self.sectors_per_cluster();
        let upcase = self.clusters_for(UPCASE_TABLE.len() as u32) * self.sectors_per_cluster();
        // The root is one cluster: the entry sector plus the rest zeroed behind it.
        let root = self.sectors_per_cluster();
        // Main and backup, each a full boot region plus its checksum sector.
        let boot = BOOT_REGION_SECTORS * 2;

        fat.saturating_add(bitmap)
            .saturating_add(upcase)
            .saturating_add(root)
            .saturating_add(boot)
    }
}

/// Sectors per cluster for a volume of `sectors` 512-byte sectors.
///
/// The conventional exFAT table, confirmed against `newfs_exfat` at eight sizes:
///
/// | Volume | Cluster |
/// |---|---|
/// | up to 256 MB | 4 KiB |
/// | 256 MB to 32 GB | 32 KiB |
/// | above 32 GB | 128 KiB |
///
/// The boundary at 256 MB was bisected rather than assumed: 192 MB gets 4 KiB and 256 MB
/// gets 32 KiB.
///
/// Returned as a shift because that is how the boot sector stores it, and because it
/// guarantees a power of two -- which the format requires and a byte count would not.
fn sectors_per_cluster_shift(sectors: u64) -> u8 {
    const MB_256: u64 = 256 * 1024 * 1024 / BYTES_PER_SECTOR as u64;
    const GB_32: u64 = 32 * 1024 * 1024 * 1024 / BYTES_PER_SECTOR as u64;

    if sectors < MB_256 {
        3 // 8 sectors, 4 KiB
    } else if sectors < GB_32 {
        6 // 64 sectors, 32 KiB
    } else {
        8 // 256 sectors, 128 KiB
    }
}

/// Work out the layout for a device of `total_sectors`.
///
/// The FAT's length depends on the cluster count, which depends on where the heap starts,
/// which depends on the FAT's length. Rather than solve that, it is iterated: start from
/// the whole device being heap, compute the FAT that would need, recompute. It converges
/// in two passes because each round can only shrink the heap, and the shrink is bounded by
/// the rounding granularity -- the loop bound is a backstop, not the mechanism.
pub fn geometry(total_sectors: u64) -> Result<Geometry, FormatError> {
    if total_sectors == 0 || total_sectors > u32::MAX as u64 {
        // The boot sector stores offsets as u32 sectors, so a device beyond that cannot be
        // described whatever the volume length field can hold.
        return Err(FormatError::BadSize);
    }

    let shift = sectors_per_cluster_shift(total_sectors);
    let sectors_per_cluster = 1u32 << shift;

    let alignment = fat_alignment(total_sectors);
    let fat_offset = alignment;

    let mut fat_length = alignment;
    let mut cluster_count;

    for _ in 0..4 {
        let heap_offset = fat_offset + fat_length;
        let heap_sectors = total_sectors
            .checked_sub(heap_offset as u64)
            .ok_or(FormatError::TooSmall)?;
        cluster_count = (heap_sectors / sectors_per_cluster as u64) as u32;

        if cluster_count < 4 {
            // Three system structures plus somewhere to put a file.
            return Err(FormatError::TooSmall);
        }

        // Two entries for clusters 0 and 1, which do not exist but are still stored.
        let needed_bytes = (cluster_count as u64 + FIRST_CLUSTER as u64) * 4;
        let needed = needed_bytes.div_ceil(BYTES_PER_SECTOR as u64) as u32;
        let next = needed.div_ceil(alignment) * alignment;
        let next = next.max(alignment);

        if next == fat_length {
            let cluster_heap_offset = fat_offset + fat_length;

            // The three system structures, laid out in the order every reference volume
            // uses: bitmap, then up-case table, then the root directory.
            let mut geo = Geometry {
                volume_length: total_sectors,
                fat_offset,
                fat_length,
                cluster_heap_offset,
                cluster_count,
                sectors_per_cluster_shift: shift,
                // Filled in below, once the cluster size is known.
                first_cluster_of_root: 0,
                bitmap_cluster: FIRST_CLUSTER,
                bitmap_length: cluster_count.div_ceil(8),
                upcase_cluster: 0,
            };

            // **Not hardcoded**, and that is the trap this avoids: the up-case table is
            // 5836 bytes, so it occupies two 4 KiB clusters but only one 32 KiB cluster.
            // The root therefore lands at cluster 5 on a small volume and cluster 4 on a
            // large one -- which is exactly what the reference volumes show.
            let bitmap_clusters = geo.clusters_for(geo.bitmap_length);
            geo.upcase_cluster = geo.bitmap_cluster + bitmap_clusters;
            let upcase_clusters = geo.clusters_for(UPCASE_TABLE.len() as u32);
            geo.first_cluster_of_root = geo.upcase_cluster + upcase_clusters;

            return Ok(geo);
        }

        fat_length = next;
    }

    Err(FormatError::BadSize)
}

/// The boot region checksum: rotate right one bit, add, over sectors 0 through 10.
///
/// Three bytes of sector 0 are skipped -- `volume_flags` at 106..108 and `percent_in_use`
/// at 112 -- because they are mutable during normal operation and the checksum must stay
/// valid when they change.
fn boot_checksum(sectors: &[[u8; BYTES_PER_SECTOR]]) -> u32 {
    let mut checksum: u32 = 0;
    for (index, sector) in sectors.iter().enumerate() {
        for (offset, &byte) in sector.iter().enumerate() {
            if index == 0 && (offset == 106 || offset == 107 || offset == 112) {
                continue;
            }
            checksum = ((checksum << 31) | (checksum >> 1)).wrapping_add(byte as u32);
        }
    }
    checksum
}

/// The up-case table's checksum, over its bytes.
///
/// Same rotate-and-add as [`boot_checksum`] but with nothing skipped. Verified against a
/// reference volume: the table this crate embeds checksums to the value that volume stored
/// for it.
fn table_checksum(data: &[u8]) -> u32 {
    let mut checksum: u32 = 0;
    for &byte in data {
        checksum = ((checksum << 31) | (checksum >> 1)).wrapping_add(byte as u32);
    }
    checksum
}

/// Build the 512-byte main boot sector.
fn boot_sector(geo: &Geometry, volume_serial: u32, percent_in_use: u8) -> [u8; BYTES_PER_SECTOR] {
    let mut s = [0u8; BYTES_PER_SECTOR];

    // JumpBoot, then the file system name. Both are checked by every implementation that
    // identifies a volume, including `exfat-slim`'s own `InvalidJumpBoot`.
    s[0..3].copy_from_slice(&[0xEB, 0x76, 0x90]);
    s[3..11].copy_from_slice(b"EXFAT   ");
    // 11..64 MustBeZero, and it is: this is how an implementation tells exFAT from FAT,
    // whose BPB occupies exactly this range.

    // PartitionOffset. Zero rather than the volume's media offset -- legal, and honest for
    // a superfloppy that starts at sector 0.
    s[64..72].copy_from_slice(&0u64.to_le_bytes());
    s[72..80].copy_from_slice(&geo.volume_length.to_le_bytes());
    s[80..84].copy_from_slice(&geo.fat_offset.to_le_bytes());
    s[84..88].copy_from_slice(&geo.fat_length.to_le_bytes());
    s[88..92].copy_from_slice(&geo.cluster_heap_offset.to_le_bytes());
    s[92..96].copy_from_slice(&geo.cluster_count.to_le_bytes());
    s[96..100].copy_from_slice(&geo.first_cluster_of_root.to_le_bytes());
    s[100..104].copy_from_slice(&volume_serial.to_le_bytes());
    s[104..106].copy_from_slice(&FILE_SYSTEM_REVISION.to_le_bytes());
    // VolumeFlags: no active FAT bit (one FAT), volume not dirty, no media failure.
    s[106..108].copy_from_slice(&0u16.to_le_bytes());
    s[108] = BYTES_PER_SECTOR_SHIFT;
    s[109] = geo.sectors_per_cluster_shift;
    // NumberOfFats. One: two is only for TexFAT, which nothing here implements.
    s[110] = 1;
    // DriveSelect, the value the reference volumes carry.
    s[111] = 0x80;
    s[112] = percent_in_use;
    // 113..120 Reserved, 120..510 BootCode -- both left zero.
    s[510] = 0x55;
    s[511] = 0xAA;

    s
}

/// One 512-byte sector, aligned as the block device requires.
fn sector_buf(bytes: &[u8; BYTES_PER_SECTOR]) -> Aligned<A4, [u8; BYTES_PER_SECTOR]> {
    Aligned(*bytes)
}

/// Write a single sector.
async fn put<BD>(device: &mut BD, sector: u32, bytes: &[u8; BYTES_PER_SECTOR]) -> Result<(), FormatError>
where
    BD: BlockDevice<BYTES_PER_SECTOR, Align = A4>,
    BD::Error: Debug,
{
    device
        .write(sector, &[sector_buf(bytes)])
        .await
        .map_err(|_| FormatError::Io)
}

/// Write a complete exFAT volume over whatever was on `device`.
///
/// **Destructive and not resumable.** Everything previously on the card is gone from the
/// first write; a failure part way through leaves a volume that will not mount, which is
/// no worse than the FAT32 or damaged volume that motivated formatting it, but is worth
/// knowing before calling.
///
/// The order is chosen so the volume is unmountable until it is complete: the boot region
/// -- the only thing that makes a volume identifiable at all -- is written **last**. An
/// interrupted format therefore leaves something no implementation will mistake for a
/// valid filesystem, rather than a valid-looking volume with no FAT behind it.
pub async fn format<BD>(device: &mut BD, label: &str, volume_serial: u32) -> Result<Geometry, FormatError>
where
    BD: BlockDevice<BYTES_PER_SECTOR, Align = A4>,
    BD::Error: Debug,
{
    // The embedded table must describe itself before any of it is written.
    if table_checksum(&UPCASE_TABLE) != UPCASE_TABLE_CHECKSUM {
        return Err(FormatError::UpcaseTableCorrupt);
    }

    let size = device.size().await.map_err(|_| FormatError::Io)?;
    let total_sectors = size / BYTES_PER_SECTOR as u64;
    let geo = geometry(total_sectors)?;

    write_fat(device, &geo).await?;
    write_bitmap(device, &geo).await?;
    write_upcase(device, &geo).await?;
    write_root(device, &geo, label).await?;
    write_boot_regions(device, &geo, volume_serial).await?;

    Ok(geo)
}

/// The FAT: two reserved entries, then a chain per system structure.
///
/// Every system structure here is contiguous, so each chain is a run of "next cluster"
/// entries ending in end-of-chain. The rest of the FAT is zero, which is what marks a
/// cluster free.
async fn write_fat<BD>(device: &mut BD, geo: &Geometry) -> Result<(), FormatError>
where
    BD: BlockDevice<BYTES_PER_SECTOR, Align = A4>,
    BD::Error: Debug,
{
    let bitmap_clusters = geo.clusters_for(geo.bitmap_length);
    let upcase_clusters = geo.clusters_for(UPCASE_TABLE.len() as u32);
    let used_end = geo.first_cluster_of_root + 1;

    let entries_per_sector = (BYTES_PER_SECTOR / 4) as u32;

    for sector_index in 0..geo.fat_length {
        let mut sector = [0u8; BYTES_PER_SECTOR];
        let first_entry = sector_index * entries_per_sector;

        // Only the first sectors carry anything; the rest are free-cluster zeros. Writing
        // them all is still required -- a card arrives with whatever the previous
        // filesystem left, and a stale non-zero entry reads as an allocated cluster.
        for slot in 0..entries_per_sector {
            let cluster = first_entry + slot;
            if cluster >= geo.cluster_count + FIRST_CLUSTER {
                break;
            }

            let value = match cluster {
                0 => FAT_MEDIA_DESCRIPTOR,
                1 => FAT_RESERVED,
                c if c >= FIRST_CLUSTER && c < used_end => {
                    // Which run is this cluster in, and is it the last of it?
                    let bitmap_end = geo.bitmap_cluster + bitmap_clusters;
                    let upcase_end = geo.upcase_cluster + upcase_clusters;
                    let last_of_run = c + 1 == bitmap_end
                        || c + 1 == upcase_end
                        || c == geo.first_cluster_of_root;
                    if last_of_run { FAT_END_OF_CHAIN } else { c + 1 }
                }
                _ => 0,
            };

            let offset = (slot * 4) as usize;
            sector[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
        }

        put(device, geo.fat_offset + sector_index, &sector).await?;
    }

    Ok(())
}

/// The allocation bitmap: one bit per cluster, least significant bit first, set for used.
async fn write_bitmap<BD>(device: &mut BD, geo: &Geometry) -> Result<(), FormatError>
where
    BD: BlockDevice<BYTES_PER_SECTOR, Align = A4>,
    BD::Error: Debug,
{
    let used = geo.first_cluster_of_root + 1 - FIRST_CLUSTER;
    let sectors = geo.clusters_for(geo.bitmap_length) * geo.sectors_per_cluster();
    let base = geo.cluster_sector(geo.bitmap_cluster);

    for sector_index in 0..sectors {
        let mut sector = [0u8; BYTES_PER_SECTOR];
        let first_bit = sector_index as u64 * BYTES_PER_SECTOR as u64 * 8;

        for bit in 0..(BYTES_PER_SECTOR as u64 * 8) {
            let cluster_index = first_bit + bit;
            if cluster_index >= used as u64 {
                break;
            }
            sector[(bit / 8) as usize] |= 1 << (bit % 8);
        }

        put(device, base + sector_index, &sector).await?;
    }

    Ok(())
}

/// The up-case table, padded to a whole number of clusters.
async fn write_upcase<BD>(device: &mut BD, geo: &Geometry) -> Result<(), FormatError>
where
    BD: BlockDevice<BYTES_PER_SECTOR, Align = A4>,
    BD::Error: Debug,
{
    let sectors = geo.clusters_for(UPCASE_TABLE.len() as u32) * geo.sectors_per_cluster();
    let base = geo.cluster_sector(geo.upcase_cluster);

    for sector_index in 0..sectors {
        let mut sector = [0u8; BYTES_PER_SECTOR];
        let start = sector_index as usize * BYTES_PER_SECTOR;
        if start < UPCASE_TABLE.len() {
            let end = (start + BYTES_PER_SECTOR).min(UPCASE_TABLE.len());
            sector[..end - start].copy_from_slice(&UPCASE_TABLE[start..end]);
        }
        put(device, base + sector_index, &sector).await?;
    }

    Ok(())
}

/// The root directory: a volume label, then the bitmap and up-case entries.
///
/// The order matters less than the presence: `exfat-slim`'s mount refuses a root that does
/// not carry both an allocation bitmap and an up-case table entry, and so does every other
/// implementation.
async fn write_root<BD>(device: &mut BD, geo: &Geometry, label: &str) -> Result<(), FormatError>
where
    BD: BlockDevice<BYTES_PER_SECTOR, Align = A4>,
    BD::Error: Debug,
{
    let mut first = [0u8; BYTES_PER_SECTOR];

    // 0x83 -- Volume Label. UTF-16LE, with a length in code units.
    //
    // Truncated at the code-unit bound rather than rejected: a label is cosmetic, and
    // failing a format over one would be a poor trade. Characters outside the basic
    // multilingual plane need two code units and are dropped rather than half-encoded.
    let mut units = 0usize;
    for ch in label.chars() {
        let mut buf = [0u16; 2];
        let encoded = ch.encode_utf16(&mut buf);
        if units + encoded.len() > MAX_LABEL_LEN {
            break;
        }
        for &unit in encoded.iter() {
            let offset = 2 + units * 2;
            first[offset..offset + 2].copy_from_slice(&unit.to_le_bytes());
            units += 1;
        }
    }
    first[0] = 0x83;
    first[1] = units as u8;

    // 0x81 -- Allocation Bitmap.
    let bitmap = &mut first[32..64];
    bitmap[0] = 0x81;
    // BitmapFlags zero: the first (and only) FAT's bitmap.
    bitmap[20..24].copy_from_slice(&geo.bitmap_cluster.to_le_bytes());
    bitmap[24..32].copy_from_slice(&(geo.bitmap_length as u64).to_le_bytes());

    // 0x82 -- Up-case Table.
    let upcase = &mut first[64..96];
    upcase[0] = 0x82;
    upcase[4..8].copy_from_slice(&UPCASE_TABLE_CHECKSUM.to_le_bytes());
    upcase[20..24].copy_from_slice(&geo.upcase_cluster.to_le_bytes());
    upcase[24..32].copy_from_slice(&(UPCASE_TABLE.len() as u64).to_le_bytes());

    // Everything past those three entries stays zero, which is end-of-directory.
    let base = geo.cluster_sector(geo.first_cluster_of_root);
    put(device, base, &first).await?;

    // The rest of the root's cluster has to be zeroed too: a stale byte here reads as a
    // directory entry.
    let blank = [0u8; BYTES_PER_SECTOR];
    for sector_index in 1..geo.sectors_per_cluster() {
        put(device, base + sector_index, &blank).await?;
    }

    Ok(())
}

/// The main and backup boot regions, written last.
async fn write_boot_regions<BD>(
    device: &mut BD,
    geo: &Geometry,
    volume_serial: u32,
) -> Result<(), FormatError>
where
    BD: BlockDevice<BYTES_PER_SECTOR, Align = A4>,
    BD::Error: Debug,
{
    // Percent of the heap in use, rounded down. With three system structures on a card of
    // any size this is zero, but it is computed rather than assumed so it stays right if
    // the layout ever grows.
    let used = geo.first_cluster_of_root + 1 - FIRST_CLUSTER;
    let percent = ((used as u64 * 100) / geo.cluster_count as u64) as u8;

    let boot = boot_sector(geo, volume_serial, percent);

    // Extended boot sectors: all zero except a signature in the last four bytes.
    let mut extended = [0u8; BYTES_PER_SECTOR];
    extended[BYTES_PER_SECTOR - 4..].copy_from_slice(&0xAA55_0000u32.to_le_bytes());

    // OEM parameters and the reserved sector are both legitimately all-zero.
    let blank = [0u8; BYTES_PER_SECTOR];

    // Sectors 0..11, in order, so the checksum can be taken over them.
    let mut region = [[0u8; BYTES_PER_SECTOR]; 11];
    region[0] = boot;
    for sector in region.iter_mut().take(9).skip(1) {
        *sector = extended;
    }
    region[9] = blank;
    region[10] = blank;

    let checksum = boot_checksum(&region);
    let mut checksum_sector = [0u8; BYTES_PER_SECTOR];
    for slot in checksum_sector.chunks_exact_mut(4) {
        slot.copy_from_slice(&checksum.to_le_bytes());
    }

    // Backup first. If power is lost between the two, a volume with a valid backup and a
    // blank main region is recoverable; the reverse is a volume that claims to be
    // mountable and has no backup to fall back on.
    for (index, sector) in region.iter().enumerate() {
        put(device, BACKUP_BOOT_SECTOR + index as u32, sector).await?;
    }
    put(device, BACKUP_BOOT_SECTOR + 11, &checksum_sector).await?;

    for (index, sector) in region.iter().enumerate() {
        put(device, index as u32, sector).await?;
    }
    put(device, 11, &checksum_sector).await?;

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The embedded table describes itself.
    ///
    /// The formatter checks this at runtime too, but a build that fails here never reaches
    /// a card.
    #[test]
    fn upcase_table_matches_its_checksum() {
        assert_eq!(table_checksum(&UPCASE_TABLE), UPCASE_TABLE_CHECKSUM);
        assert_eq!(UPCASE_TABLE.len(), 5836);
    }

    /// Geometry against volumes `newfs_exfat` produced, at the sizes that were measured.
    ///
    /// **Every number here was read out of a real volume**, not derived from this code's
    /// own rules and not inferred from the rows either side of it. That distinction is not
    /// pedantry: the 192 MB row was originally written as root cluster 5, by pattern
    /// matching the other 4 KiB rows, and the correct answer is 6 -- its allocation bitmap
    /// is 6136 bytes, which spills into a second 4 KiB cluster and pushes everything after
    /// it along. The formatter had it right and the expectation was wrong.
    ///
    /// That is exactly the case worth pinning, and it is why the root cluster cannot be a
    /// constant: it moves with both the cluster size *and* the bitmap length, which moves
    /// with the cluster count.
    #[test]
    fn geometry_matches_reference_volumes() {
        // (megabytes, cluster shift, fat_length, heap offset, cluster count, root cluster)
        //
        // Fourteen sizes from 16 MB to 64 GB, chosen to sit either side of every boundary
        // rather than to sample evenly: 192/256 MB brackets the 4 KiB -> 32 KiB cluster
        // step, 3/4 GB brackets the FAT alignment step, and 16/32 GB brackets the
        // 32 KiB -> 128 KiB step.
        let cases = [
            (16u64, 3u8, 128u32, 256u32, 4064u32, 5u32),
            (64, 3, 128, 256, 16352, 5),
            (96, 3, 256, 384, 24528, 5),
            (128, 3, 256, 384, 32720, 5),
            // Two-cluster bitmap; see the note above.
            (192, 3, 384, 512, 49088, 6),
            // 4 KiB -> 32 KiB clusters.
            (256, 6, 128, 256, 8188, 4),
            (1024, 6, 256, 384, 32762, 4),
            (2048, 6, 512, 640, 65526, 4),
            (3072, 6, 768, 896, 98290, 4),
            // fat_offset 128 -> 2048, and the FAT rounds up to 2048 from a need of 1024.
            (4096, 6, 2048, 4096, 131008, 4),
            (8192, 6, 2048, 4096, 262080, 4),
            // Another two-cluster bitmap, this time at 32 KiB clusters.
            (16384, 6, 4096, 6144, 524192, 5),
            // 32 KiB -> 128 KiB clusters, at exactly 32 GiB.
            (32768, 8, 2048, 4096, 262128, 4),
            (65536, 8, 4096, 6144, 524264, 4),
        ];

        for (mb, shift, fat_length, heap, clusters, root) in cases {
            let sectors = mb * 1024 * 1024 / 512;
            let geo = geometry(sectors).expect("reference sizes must produce a geometry");

            assert_eq!(geo.sectors_per_cluster_shift, shift, "cluster shift at {mb} MB");
            assert_eq!(geo.fat_length, fat_length, "fat_length at {mb} MB");
            assert_eq!(geo.cluster_heap_offset, heap, "heap offset at {mb} MB");
            assert_eq!(geo.cluster_count, clusters, "cluster count at {mb} MB");
            assert_eq!(geo.first_cluster_of_root, root, "root cluster at {mb} MB");
        }
    }

    /// The heap must end exactly at the end of the volume, or short of it -- never past.
    #[test]
    fn the_heap_fits_inside_the_volume() {
        for mb in [16u64, 64, 96, 256, 1024, 8192, 15_193] {
            let sectors = mb * 1024 * 1024 / 512;
            let geo = geometry(sectors).expect("must produce a geometry");
            let end = geo.cluster_heap_offset as u64
                + geo.cluster_count as u64 * geo.sectors_per_cluster() as u64;
            assert!(end <= geo.volume_length, "heap overruns the volume at {mb} MB");
            assert!(
                geo.volume_length - end < geo.sectors_per_cluster() as u64,
                "more than a cluster wasted at {mb} MB"
            );
        }
    }

    /// The FAT must be able to address every cluster the geometry claims.
    #[test]
    fn the_fat_covers_every_cluster() {
        for mb in [16u64, 64, 256, 1024, 8192, 15_193] {
            let sectors = mb * 1024 * 1024 / 512;
            let geo = geometry(sectors).expect("must produce a geometry");
            let entries = geo.fat_length as u64 * (BYTES_PER_SECTOR as u64 / 4);
            assert!(
                entries >= geo.cluster_count as u64 + FIRST_CLUSTER as u64,
                "FAT too short at {mb} MB"
            );
        }
    }

    /// A device too small for the boot regions and a FAT is refused rather than
    /// half-formatted.
    #[test]
    fn tiny_devices_are_refused() {
        assert_eq!(geometry(0), Err(FormatError::BadSize));
        assert_eq!(geometry(64), Err(FormatError::TooSmall));
        assert_eq!(geometry(256), Err(FormatError::TooSmall));
    }
}
