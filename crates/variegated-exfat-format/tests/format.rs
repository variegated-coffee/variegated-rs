//! End-to-end checks on a formatted volume.
//!
//! The crate's own unit tests cover the *geometry* -- where everything goes -- against
//! volumes `newfs_exfat` produced. These cover what is actually written into those places,
//! which the geometry tests cannot see at all.
//!
//! Three independent judges, deliberately:
//!
//! 1. **This file**, parsing the structures back out. Catches a field in the wrong place.
//! 2. **`exfat-slim` itself**, mounting the volume and round-tripping a file. This is the
//!    one that matters most, because it is the actual consumer on the device -- a volume
//!    that satisfies the spec but not this implementation is still useless to us.
//! 3. **`fsck_exfat`**, when the host has it. An implementation with no shared code or
//!    assumptions with ours, which is the only way to catch a mistake we have made
//!    consistently in both the writer and the reader.

use std::path::PathBuf;

use aligned::{Aligned, A4};
use exfat_slim::asynchronous::BlockDevice;
use futures_lite::future::block_on;
use variegated_exfat_format::{format, geometry};

const SECTOR: usize = 512;

/// A block device backed by memory.
///
/// In memory rather than a file, because the formatter writes every sector of the FAT and
/// the tests run over volumes up to a few gigabytes of *addressable* space -- but only the
/// first few megabytes are ever touched, so a sparse `Vec` sized to the touched region
/// would be wrong and a real one would be enormous. This stores only what is written and
/// reports zeros elsewhere, which is exactly how a freshly-erased card behaves.
struct MemDevice {
    sectors: std::collections::BTreeMap<u32, [u8; SECTOR]>,
    total_sectors: u64,
    /// Sectors written, counting rewrites. Distinct from `sectors.len()`, which counts
    /// distinct addresses -- and it is the *issued* count that a format's duration tracks,
    /// so it is the one `Geometry::sectors_written` has to predict.
    sectors_written: u32,
}

impl MemDevice {
    fn new(total_sectors: u64) -> Self {
        Self {
            sectors: std::collections::BTreeMap::new(),
            total_sectors,
            sectors_written: 0,
        }
    }

    fn sector(&self, index: u32) -> [u8; SECTOR] {
        self.sectors.get(&index).copied().unwrap_or([0u8; SECTOR])
    }

    /// Materialise the whole volume, for handing to an external tool.
    fn to_image(&self) -> Vec<u8> {
        let mut image = vec![0u8; self.total_sectors as usize * SECTOR];
        for (&index, bytes) in &self.sectors {
            let start = index as usize * SECTOR;
            image[start..start + SECTOR].copy_from_slice(bytes);
        }
        image
    }
}

#[derive(Debug)]
struct OutOfRange;

impl BlockDevice<SECTOR> for MemDevice {
    type Error = OutOfRange;
    type Align = A4;

    async fn read(
        &mut self,
        block_address: u32,
        data: &mut [Aligned<Self::Align, [u8; SECTOR]>],
    ) -> Result<(), Self::Error> {
        for (offset, block) in data.iter_mut().enumerate() {
            let index = block_address + offset as u32;
            if index as u64 >= self.total_sectors {
                return Err(OutOfRange);
            }
            **block = self.sector(index);
        }
        Ok(())
    }

    async fn write(
        &mut self,
        block_address: u32,
        data: &[Aligned<Self::Align, [u8; SECTOR]>],
    ) -> Result<(), Self::Error> {
        for (offset, block) in data.iter().enumerate() {
            let index = block_address + offset as u32;
            // A formatter that runs off the end of the device is the bug most likely to
            // brick a card, and the one least likely to show up as anything but silent
            // corruption. Refused rather than ignored.
            if index as u64 >= self.total_sectors {
                return Err(OutOfRange);
            }
            self.sectors.insert(index, **block);
            self.sectors_written += 1;
        }
        Ok(())
    }

    async fn size(&mut self) -> Result<u64, Self::Error> {
        Ok(self.total_sectors * SECTOR as u64)
    }
}

fn formatted(megabytes: u64) -> MemDevice {
    let sectors = megabytes * 1024 * 1024 / SECTOR as u64;
    let mut device = MemDevice::new(sectors);
    block_on(format(&mut device, "VARIEGATED", 0x1234_5678)).expect("format must succeed");
    device
}

/// `Geometry::sectors_written` predicts exactly what a format issues.
///
/// This exists because a caller sizes a **timeout** from that number, and both directions
/// of error are expensive: too low and a real format is abandoned part-written, which is a
/// destroyed volume; too high and the bound stops being one. An estimate maintained
/// separately from the writers would drift the first time a structure gained a sector, and
/// nothing else in the tree would notice.
///
/// Exact rather than an upper bound, deliberately. A `>=` assertion would pass just as well
/// if the formatter stopped writing the FAT altogether, which is precisely the regression
/// worth catching here.
///
/// Sizes chosen to straddle the layout steps the geometry tests already pin: the
/// 4 KiB -> 32 KiB cluster change at 256 MB, the FAT alignment change at 4 GB, and the
/// 32 KiB -> 128 KiB change at 32 GB. The per-cluster terms scale with cluster size, so a
/// single size would miss a mistake in any of them.
#[test]
fn sectors_written_matches_the_format() {
    for mb in [16u64, 64, 192, 256, 1024, 4096, 16384, 32768, 65536] {
        let sectors = mb * 1024 * 1024 / SECTOR as u64;
        let geo = geometry(sectors).expect("reference sizes must produce a geometry");

        let mut device = MemDevice::new(sectors);
        block_on(format(&mut device, "VARIEGATED", 0x1234_5678)).expect("format must succeed");

        assert_eq!(
            geo.sectors_written(),
            device.sectors_written,
            "predicted vs issued sector writes at {mb} MB"
        );
    }
}

/// The figures in `sectors_written`'s own doc comment.
///
/// Pinned because a caller reads that table to pick a per-sector allowance, and a table
/// that has quietly drifted is worse than none -- it reads as measured when it is not.
/// Formatting a 1 TB volume in `sectors_written_matches_the_format` would mean a 1 TB
/// image, so these are checked against the geometry alone.
///
/// The 16 GB row is the one worth having: it is **larger** than the 32 GB row, because the
/// 32 KiB -> 128 KiB cluster step quarters the FAT. Anyone who "fixes" this table by making
/// it monotonic will fail here.
#[test]
fn the_documented_sector_counts_are_current() {
    // (gigabytes, cluster KiB, sectors written)
    let cases = [
        (16u64, 32u32, 4_376u32),
        (32, 128, 2_840),
        (128, 128, 8_984),
        (512, 128, 34_328),
        (1024, 128, 68_120),
    ];

    for (gb, cluster_kib, written) in cases {
        let sectors = gb * 1024 * 1024 * 1024 / SECTOR as u64;
        let geo = geometry(sectors).expect("documented sizes must produce a geometry");
        assert_eq!(geo.bytes_per_cluster() / 1024, cluster_kib, "cluster size at {gb} GB");
        assert_eq!(geo.sectors_written(), written, "documented sector count at {gb} GB");
    }
}

fn le32(bytes: &[u8], at: usize) -> u32 {
    u32::from_le_bytes(bytes[at..at + 4].try_into().unwrap())
}

fn le64(bytes: &[u8], at: usize) -> u64 {
    u64::from_le_bytes(bytes[at..at + 8].try_into().unwrap())
}

/// The boot sector says what the geometry says, and carries the signatures every
/// implementation identifies a volume by.
#[test]
fn the_boot_sector_describes_the_volume() {
    let device = formatted(64);
    let geo = geometry(64 * 1024 * 1024 / SECTOR as u64).unwrap();
    let boot = device.sector(0);

    assert_eq!(&boot[0..3], &[0xEB, 0x76, 0x90], "jump boot");
    assert_eq!(&boot[3..11], b"EXFAT   ", "file system name");
    // 11..64 must be zero: it is where FAT keeps its BPB, and a non-zero byte here is how
    // an implementation decides a volume is FAT rather than exFAT.
    assert!(boot[11..64].iter().all(|&b| b == 0), "MustBeZero is not zero");

    assert_eq!(le64(&boot, 72), geo.volume_length, "volume length");
    assert_eq!(le32(&boot, 80), geo.fat_offset, "fat offset");
    assert_eq!(le32(&boot, 84), geo.fat_length, "fat length");
    assert_eq!(le32(&boot, 88), geo.cluster_heap_offset, "cluster heap offset");
    assert_eq!(le32(&boot, 92), geo.cluster_count, "cluster count");
    assert_eq!(le32(&boot, 96), geo.first_cluster_of_root, "root cluster");
    assert_eq!(boot[108], 9, "bytes per sector shift");
    assert_eq!(boot[109], geo.sectors_per_cluster_shift, "cluster shift");
    assert_eq!(boot[110], 1, "number of FATs");
    assert_eq!([boot[510], boot[511]], [0x55, 0xAA], "boot signature");
}

/// The backup boot region is a copy, and both carry a checksum that describes them.
///
/// The checksum is the part worth testing rather than assuming: it skips three mutable
/// bytes of sector 0, and a checksum computed over all of them would be self-consistent
/// here and rejected by everything else.
#[test]
fn both_boot_regions_are_present_and_checksummed() {
    let device = formatted(64);

    for base in [0u32, 12] {
        let mut region = Vec::new();
        for index in 0..11 {
            region.push(device.sector(base + index));
        }

        let mut expected: u32 = 0;
        for (index, sector) in region.iter().enumerate() {
            for (offset, &byte) in sector.iter().enumerate() {
                if index == 0 && (offset == 106 || offset == 107 || offset == 112) {
                    continue;
                }
                expected = ((expected << 31) | (expected >> 1)).wrapping_add(byte as u32);
            }
        }

        let checksum_sector = device.sector(base + 11);
        for slot in checksum_sector.chunks_exact(4) {
            assert_eq!(
                u32::from_le_bytes(slot.try_into().unwrap()),
                expected,
                "boot checksum at sector {}",
                base + 11
            );
        }

        // Extended boot sectors carry a signature of their own.
        for index in 1..9 {
            let sector = device.sector(base + index);
            assert_eq!(
                le32(&sector, SECTOR - 4),
                0xAA55_0000,
                "extended boot signature at sector {}",
                base + index
            );
        }
    }

    // The two regions must be identical, or a recovery that falls back to the backup gets
    // a different volume than the one it was using.
    for index in 0..12 {
        assert_eq!(
            device.sector(index),
            device.sector(12 + index),
            "backup boot region differs at sector {index}"
        );
    }
}

/// The FAT marks the system structures allocated and everything else free.
#[test]
fn the_fat_chains_the_system_structures() {
    let device = formatted(64);
    let geo = geometry(64 * 1024 * 1024 / SECTOR as u64).unwrap();

    let entry = |cluster: u32| -> u32 {
        let byte = cluster as usize * 4;
        let sector = device.sector(geo.fat_offset + (byte / SECTOR) as u32);
        le32(&sector, byte % SECTOR)
    };

    assert_eq!(entry(0), 0xFFFF_FFF8, "media descriptor");
    assert_eq!(entry(1), 0xFFFF_FFFF, "reserved entry");

    // Every allocated cluster either points at the next of its run or ends the chain.
    for cluster in 2..=geo.first_cluster_of_root {
        let value = entry(cluster);
        assert!(
            value == 0xFFFF_FFFF || value == cluster + 1,
            "cluster {cluster} chains to {value:#010x}"
        );
    }
    assert_eq!(
        entry(geo.first_cluster_of_root),
        0xFFFF_FFFF,
        "the root directory must end its chain"
    );

    // The first cluster past the system structures is free, or nothing could be written.
    assert_eq!(entry(geo.first_cluster_of_root + 1), 0, "first free cluster");
}

/// The allocation bitmap agrees with the FAT about what is in use.
///
/// These are two independent records of the same fact, and a volume where they disagree
/// passes a casual look and then allocates a cluster that is already in use.
#[test]
fn the_bitmap_agrees_with_the_fat() {
    let device = formatted(64);
    let geo = geometry(64 * 1024 * 1024 / SECTOR as u64).unwrap();

    let used = geo.first_cluster_of_root + 1 - 2;
    let base = geo.cluster_sector(geo.bitmap_cluster);

    for cluster_index in 0..used + 8 {
        let bit = cluster_index as usize;
        let sector = device.sector(base + (bit / (SECTOR * 8)) as u32);
        let within = bit % (SECTOR * 8);
        let set = sector[within / 8] & (1 << (within % 8)) != 0;
        assert_eq!(
            set,
            cluster_index < used,
            "bitmap disagrees about cluster {}",
            cluster_index + 2
        );
    }
}

/// The root directory carries the two entries a mount requires, plus a label.
#[test]
fn the_root_directory_has_its_system_entries() {
    let device = formatted(64);
    let geo = geometry(64 * 1024 * 1024 / SECTOR as u64).unwrap();
    let root = device.sector(geo.cluster_sector(geo.first_cluster_of_root));

    assert_eq!(root[0], 0x83, "volume label entry");
    assert_eq!(root[1], 10, "label length in UTF-16 code units");

    let bitmap = &root[32..64];
    assert_eq!(bitmap[0], 0x81, "allocation bitmap entry");
    assert_eq!(le32(bitmap, 20), geo.bitmap_cluster, "bitmap cluster");
    assert_eq!(le64(bitmap, 24), geo.bitmap_length as u64, "bitmap length");

    let upcase = &root[64..96];
    assert_eq!(upcase[0], 0x82, "up-case table entry");
    assert_eq!(
        le32(upcase, 4),
        variegated_exfat_format::upcase::UPCASE_TABLE_CHECKSUM,
        "up-case checksum"
    );
    assert_eq!(le32(upcase, 20), geo.upcase_cluster, "up-case cluster");
    assert_eq!(
        le64(upcase, 24),
        variegated_exfat_format::upcase::UPCASE_TABLE.len() as u64,
        "up-case length"
    );

    // Anything past the three entries must read as end-of-directory.
    assert_eq!(root[96], 0, "directory must terminate after its system entries");
}

/// The volume the formatter writes is one `exfat-slim` will mount and use.
///
/// The most important test here: it is the same code path the firmware runs, so a volume
/// that is legal but that this implementation dislikes fails here rather than on a machine.
///
/// # Release only, because of an upstream overflow
///
/// `exfat_slim::utils::encode_utf16_upcase_and_hash` computes the exFAT filename hash with
/// plain `+` on a `u16`. The specification defines that hash as *wrapping*, so the release
/// behaviour is correct -- but a debug build has overflow checks on and panics on the first
/// filename whose hash carries, which is most of them.
///
/// So this is skipped in debug rather than left failing. The firmware is unaffected: it
/// ships release, where the arithmetic wraps as intended. It is worth fixing upstream
/// (`wrapping_add`), and worth *not* fixing locally without deciding to, because
/// `exfat-slim` is currently an unmodified pin to upstream rather than a fork we maintain.
#[test]
fn exfat_slim_mounts_and_round_trips_a_file() {
    use exfat_slim::asynchronous::file::OpenOptions;
    use exfat_slim::asynchronous::file_system::FileSystem;

    if cfg!(debug_assertions) {
        eprintln!(
            "skipping: exfat-slim's filename hash overflows a u16 under debug overflow \
             checks. Run with --release to exercise this."
        );
        return;
    }

    let device = formatted(64);

    block_on(async {
        let mut fs: FileSystem<MemDevice, SECTOR, 4> = FileSystem::new(device);
        fs.mount().await.expect("a freshly formatted volume must mount");

        fs.create_directory("/SHOTS").await.expect("create directory");
        assert!(matches!(fs.exists("/SHOTS").await, Ok(true)));

        // Past one cluster, so this exercises allocation rather than just the first
        // cluster the root already owns.
        let payload: Vec<u8> = (0..9000u32).map(|i| (i.wrapping_mul(31) & 0xFF) as u8).collect();

        let mut file = fs
            .open(
                "/SHOTS/00000042.BIN",
                OpenOptions::new().write(true).create(true).truncate(true),
            )
            .await
            .expect("create file");
        file.write(&mut fs, &payload).await.expect("write");
        file.close(&mut fs).await.expect("close");

        let mut file = fs
            .open("/SHOTS/00000042.BIN", OpenOptions::new().read(true))
            .await
            .expect("reopen");
        assert_eq!(file.metadata().len(), payload.len() as u64, "stored length");

        let mut read_back = vec![0u8; payload.len()];
        let mut cursor = 0usize;
        while cursor < payload.len() {
            match file.read(&mut fs, &mut read_back[cursor..]).await {
                Ok(Some(0)) | Ok(None) => break,
                Ok(Some(n)) => cursor += n,
                Err(e) => panic!("read failed: {e:?}"),
            }
        }
        file.close(&mut fs).await.expect("close");

        assert_eq!(cursor, payload.len(), "short read");
        assert_eq!(read_back, payload, "contents did not survive the round trip");
    });
}

/// Every size the geometry tests cover also formats without running off the device.
///
/// `MemDevice::write` refuses an out-of-range sector, so this is really a test that the
/// formatter stays inside the volume it was given -- at every cluster size and both FAT
/// alignments.
#[test]
fn formatting_stays_inside_the_volume() {
    for megabytes in [16u64, 64, 192, 256, 1024, 4096, 16384, 32768] {
        let sectors = megabytes * 1024 * 1024 / SECTOR as u64;
        let mut device = MemDevice::new(sectors);
        block_on(format(&mut device, "T", 1)).unwrap_or_else(|e| {
            panic!("format failed at {megabytes} MB: {e:?}");
        });
    }
}

/// The card actually in the machine, at its actual size.
///
/// Every other size here is a round number chosen to bracket a rule. This one is a
/// measurement: a card sold as "32 GB" reports C_SIZE = 60872 through CMD9, which is
/// (60872 + 1) * 1024 = 62,333,952 sectors — 29.7 GiB, and *below* the 32 GiB threshold
/// where the cluster shift goes to 8. So the marketing number and the number the geometry
/// rules see fall on opposite sides of the boundary, and picking the shift by reading the
/// label would get it wrong.
///
/// Worth its own test because it is the case that runs on hardware. A rule that is right
/// at 16384 MB and right at 32768 MB can still be wrong in between if the boundary is
/// compared against the wrong unit.
#[test]
fn the_cards_real_reported_size_formats_with_the_expected_geometry() {
    let sectors = 62_333_952u64;
    let mut device = MemDevice::new(sectors);
    let geometry = block_on(format(&mut device, "VARIEGATED", 1)).expect("format");

    assert_eq!(geometry.volume_length, sectors);
    assert_eq!(
        geometry.sectors_per_cluster_shift, 6,
        "29.7 GiB is under the 32 GiB threshold, so 32 KiB clusters"
    );

    // The heap has to fit, with every cluster it claims addressable.
    let heap_sectors = sectors - geometry.cluster_heap_offset as u64;
    let sectors_per_cluster = 1u64 << geometry.sectors_per_cluster_shift;
    assert!(
        geometry.cluster_count as u64 <= heap_sectors / sectors_per_cluster,
        "claimed {} clusters, only {} fit",
        geometry.cluster_count,
        heap_sectors / sectors_per_cluster
    );

    // And the bitmap has to have a bit for each of them.
    assert!(
        geometry.bitmap_length as u64 * 8 >= geometry.cluster_count as u64,
        "bitmap covers {} clusters, heap has {}",
        geometry.bitmap_length as u64 * 8,
        geometry.cluster_count
    );
}

/// Hand the volume to an implementation that shares nothing with ours.
///
/// Skipped rather than failed where the tools are absent: `fsck_exfat` and `hdiutil` ship
/// with macOS, and a Linux host would need `exfatprogs` and a loop device. A test that
/// cannot run is not a test that failed, but one that silently never runs is worse than
/// either -- hence the printed notice.
///
/// **The image is attached as a device rather than handed to `fsck_exfat` as a file.**
/// That is not a preference. `fsck_exfat` opens its argument and asks for the block count
/// and block size by `ioctl`; against a regular file both fail with `ENOTTY`, and it then
/// reports
///
/// ```text
/// Main boot region is invalid. Trying alternate boot region.
/// ```
///
/// for *any* content whatsoever -- a volume this crate wrote and a file of zeros are
/// rejected identically, so the check was passing judgement on the file type rather than on
/// the volume. Apple's own `newfs_exfat` will not touch a plain file either; it prepends
/// `/dev/` to its argument and fails outright. Attached with `hdiutil` the same bytes pass
/// completely: "The volume VARIEGATED appears to be OK".
///
/// Note for a sandboxed or containerised runner: `hdiutil attach` needs to create a device
/// node. Where it cannot, this skips with a notice rather than failing, on the same
/// reasoning as a missing `fsck_exfat`.
#[test]
fn fsck_accepts_the_volume() {
    let fsck = PathBuf::from("/sbin/fsck_exfat");
    if !fsck.exists() {
        eprintln!("skipping: {} not present", fsck.display());
        return;
    }
    let hdiutil = PathBuf::from("/usr/bin/hdiutil");
    if !hdiutil.exists() {
        eprintln!("skipping: {} not present", hdiutil.display());
        return;
    }

    let device = formatted(64);
    let path = std::env::temp_dir().join("variegated-exfat-format-test.img");
    std::fs::write(&path, device.to_image()).expect("write image");

    // `-nomount` because the volume is to be inspected, not used, and letting Disk
    // Arbitration mount it would put a second implementation between the bytes and the
    // check. `CRawDiskImage` because the file is a bare volume image with no partition map
    // or trailer -- without it hdiutil looks for a format it recognises and finds none.
    let attach = std::process::Command::new(&hdiutil)
        .args(["attach", "-nomount", "-imagekey", "diskimage-class=CRawDiskImage"])
        .arg(&path)
        .output()
        .expect("run hdiutil attach");

    if !attach.status.success() {
        let _ = std::fs::remove_file(&path);
        eprintln!(
            "skipping: hdiutil could not attach the image\n{}",
            String::from_utf8_lossy(&attach.stderr)
        );
        return;
    }

    let attached = String::from_utf8_lossy(&attach.stdout);
    let node = attached
        .lines()
        .next()
        .and_then(|line| line.split_whitespace().next())
        .expect("hdiutil named a device")
        .to_string();

    let output = std::process::Command::new(&fsck).arg("-n").arg(&node).output();

    // Detached before the assertion, never after: an `assert!` unwinds, and a device left
    // attached by a failing run outlives the test process and accumulates across runs.
    let _ = std::process::Command::new(&hdiutil)
        .args(["detach", &node])
        .output();
    let _ = std::fs::remove_file(&path);

    let output = output.expect("run fsck_exfat");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);

    assert!(
        output.status.success(),
        "fsck_exfat rejected the volume\n--- stdout ---\n{stdout}\n--- stderr ---\n{stderr}"
    );
}
