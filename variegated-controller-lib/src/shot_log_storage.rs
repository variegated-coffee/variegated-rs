//! Shot log persistent storage
//!
//! Provides a trait and SD card implementation for storing shot logs
//! to FAT32-formatted SD cards.

extern crate alloc;

use alloc::string::{String, ToString};
use alloc::vec::Vec;
use core::fmt::Debug;
use crc::{Crc, CRC_32_ISCSI};
use defmt::{debug, error, warn};
use embedded_sdmmc::{BlockDevice, Mode, RawDirectory, RawVolume, TimeSource, Timestamp, VolumeIdx, VolumeManager};
use variegated_controller_types::ShotLog;
use variegated_timekeeping::TimeKeeper;

/// Cached filesystem state to avoid repeated open/close cycles.
///
/// This holds the volume and SHOTS directory handles that are opened
/// on first use and kept until `invalidate()` is called or the storage
/// is dropped.
struct CachedVolumeState {
    /// The opened volume handle
    volume: RawVolume,
    /// The SHOTS directory handle
    shots_dir: RawDirectory,
}

/// Errors that can occur during shot log storage operations.
#[derive(Debug, Clone)]
pub enum ShotLogStorageError {
    /// SD card not present or not initialized
    CardNotPresent,
    /// Failed to open or access volume/filesystem
    FilesystemError,
    /// Failed to create or write file
    WriteError,
    /// Failed to read file
    ReadError,
    /// Serialization/deserialization error
    SerializationError,
    /// CRC validation failed
    CrcError,
    /// Directory not found or could not be created
    DirectoryError,
    /// File not found
    FileNotFound,
    /// Storage is full
    StorageFull,
}

impl defmt::Format for ShotLogStorageError {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            ShotLogStorageError::CardNotPresent => defmt::write!(fmt, "CardNotPresent"),
            ShotLogStorageError::FilesystemError => defmt::write!(fmt, "FilesystemError"),
            ShotLogStorageError::WriteError => defmt::write!(fmt, "WriteError"),
            ShotLogStorageError::ReadError => defmt::write!(fmt, "ReadError"),
            ShotLogStorageError::SerializationError => defmt::write!(fmt, "SerializationError"),
            ShotLogStorageError::CrcError => defmt::write!(fmt, "CrcError"),
            ShotLogStorageError::DirectoryError => defmt::write!(fmt, "DirectoryError"),
            ShotLogStorageError::FileNotFound => defmt::write!(fmt, "FileNotFound"),
            ShotLogStorageError::StorageFull => defmt::write!(fmt, "StorageFull"),
        }
    }
}

/// Metadata about a stored shot log file.
#[derive(Debug, Clone)]
pub struct ShotLogFileInfo {
    /// Filename (8.3 format, e.g., "20250115.BIN")
    pub filename: String,
    /// File size in bytes
    pub size_bytes: u32,
}

/// Trait for shot log storage backends.
///
/// Implementations should handle errors gracefully and not block
/// machine operation. Storage failures should be logged but not
/// propagated as fatal errors.
pub trait ShotLogStorage {
    /// Store a completed shot log, returns filename on success.
    fn store_shot(&mut self, shot: &ShotLog) -> Result<String, ShotLogStorageError>;

    /// List all stored shot logs.
    fn list_shots(&mut self) -> Result<Vec<ShotLogFileInfo>, ShotLogStorageError>;

    /// Retrieve a shot log by filename.
    fn get_shot(&mut self, filename: &str) -> Result<ShotLog, ShotLogStorageError>;

    /// Delete a shot log by filename.
    fn delete_shot(&mut self, filename: &str) -> Result<(), ShotLogStorageError>;

    /// Check if storage is available (SD card present and initialized).
    fn is_available(&self) -> bool;
}

/// TimeSource implementation using variegated-timekeeping.
///
/// Falls back to 1980-01-01 00:00:00 if TimeKeeper hasn't been
/// initialized with a valid time.
pub struct VariegatedTimeSource;

impl TimeSource for VariegatedTimeSource {
    fn get_timestamp(&self) -> Timestamp {
        if let Some(utc) = TimeKeeper::now_utc() {
            use chrono::Datelike;
            use chrono::Timelike;
            Timestamp::from_calendar(
                utc.year() as u16,
                utc.month() as u8,
                utc.day() as u8,
                utc.hour() as u8,
                utc.minute() as u8,
                utc.second() as u8,
            )
            .unwrap_or_else(|_| Timestamp::from_calendar(1980, 1, 1, 0, 0, 0).unwrap())
        } else {
            // Fallback when time not set
            Timestamp::from_calendar(1980, 1, 1, 0, 0, 0).unwrap()
        }
    }
}

/// SD card-based shot log storage.
///
/// Uses a FAT32-formatted SD card to store shot logs as binary files
/// in a `/SHOTS` directory. Files are named using the timestamp or
/// a unique identifier if time is not available.
///
/// # Type Parameters
///
/// * `BD` - The block device type (should be `YieldingBlockDevice<SdCard<...>>`)
/// * `TS` - The time source type (typically `VariegatedTimeSource`)
/// * `MAX_DIRS` - Maximum number of simultaneously open directories
/// * `MAX_FILES` - Maximum number of simultaneously open files
/// * `MAX_VOLUMES` - Maximum number of simultaneously open volumes
pub struct SdCardShotLogStorage<
    BD: BlockDevice,
    TS: TimeSource,
    const MAX_DIRS: usize,
    const MAX_FILES: usize,
    const MAX_VOLUMES: usize,
> where
    BD::Error: Debug,
{
    volume_manager: VolumeManager<BD, TS, MAX_DIRS, MAX_FILES, MAX_VOLUMES>,
    /// Cached volume and directory handles for efficient reuse
    cached_state: Option<CachedVolumeState>,
    buffer: Vec<u8>,
    available: bool,
}

impl<BD, TS, const MAX_DIRS: usize, const MAX_FILES: usize, const MAX_VOLUMES: usize>
    SdCardShotLogStorage<BD, TS, MAX_DIRS, MAX_FILES, MAX_VOLUMES>
where
    BD: BlockDevice,
    BD::Error: Debug,
    TS: TimeSource,
{
    /// Create a new SD card shot log storage.
    ///
    /// The VolumeManager should already be initialized with the SD card
    /// and time source.
    pub fn new(volume_manager: VolumeManager<BD, TS, MAX_DIRS, MAX_FILES, MAX_VOLUMES>) -> Self {
        Self {
            volume_manager,
            cached_state: None,
            buffer: Vec::with_capacity(8192),
            available: true,
        }
    }

    /// Get a reference to the underlying VolumeManager.
    pub fn volume_manager(&self) -> &VolumeManager<BD, TS, MAX_DIRS, MAX_FILES, MAX_VOLUMES> {
        &self.volume_manager
    }

    /// Get a mutable reference to the underlying VolumeManager.
    pub fn volume_manager_mut(
        &mut self,
    ) -> &mut VolumeManager<BD, TS, MAX_DIRS, MAX_FILES, MAX_VOLUMES> {
        &mut self.volume_manager
    }

    /// Generate a unique filename for a shot log.
    ///
    /// Format: 8.3 filename like "HHMMSSXX.BIN" where XX is derived from
    /// start_time_millis for uniqueness within the same second.
    fn generate_filename(&self, shot: &ShotLog) -> String {
        if let Some(utc) = TimeKeeper::now_utc() {
            use chrono::Timelike;
            // Use HHMMSS + last 2 digits of millis for uniqueness
            let unique = (shot.metadata.start_time_millis % 100) as u8;
            alloc::format!(
                "{:02}{:02}{:02}{:02}.BIN",
                utc.hour(),
                utc.minute(),
                utc.second(),
                unique
            )
        } else {
            // Fallback using start_time_millis
            let id = shot.metadata.start_time_millis % 100_000_000;
            alloc::format!("{:08}.BIN", id)
        }
    }

    /// Generate a subdirectory name based on date.
    ///
    /// Format: "YYYYMMDD" for organizing shots by day.
    fn generate_subdir_name(&self) -> String {
        if let Some(utc) = TimeKeeper::now_utc() {
            use chrono::Datelike;
            alloc::format!("{:04}{:02}{:02}", utc.year(), utc.month(), utc.day())
        } else {
            "NODATE".into()
        }
    }

    /// Serialize a shot log with CRC32 checksum.
    fn serialize(&mut self, shot: &ShotLog) -> Result<&[u8], ShotLogStorageError> {
        let crc = Crc::<u32>::new(&CRC_32_ISCSI);
        self.buffer = postcard::to_allocvec_crc32(shot, crc.digest())
            .map_err(|_| ShotLogStorageError::SerializationError)?;
        Ok(&self.buffer)
    }

    /// Deserialize a shot log and verify CRC32 checksum.
    fn deserialize(&self, data: &[u8]) -> Result<ShotLog, ShotLogStorageError> {
        let crc = Crc::<u32>::new(&CRC_32_ISCSI);
        postcard::from_bytes_crc32(data, crc.digest()).map_err(|_| ShotLogStorageError::CrcError)
    }

    /// Ensure filesystem is ready and return the cached SHOTS directory handle.
    ///
    /// Opens volume and SHOTS directory on first call, reuses cached handles
    /// on subsequent calls. The volume and directory remain open until
    /// `invalidate()` is called or the storage is dropped.
    fn ensure_filesystem(&mut self) -> Result<RawDirectory, ShotLogStorageError> {
        // If already cached, return the shots_dir handle
        if let Some(ref state) = self.cached_state {
            return Ok(state.shots_dir);
        }

        // Open volume
        let volume = self
            .volume_manager
            .open_raw_volume(VolumeIdx(0))
            .map_err(|e| {
                warn!("Failed to open volume: {:?}", defmt::Debug2Format(&e));
                ShotLogStorageError::FilesystemError
            })?;

        // Open root directory
        let root_dir = self
            .volume_manager
            .open_root_dir(volume)
            .map_err(|e| {
                warn!("Failed to open root dir: {:?}", defmt::Debug2Format(&e));
                let _ = self.volume_manager.close_volume(volume);
                ShotLogStorageError::DirectoryError
            })?;

        // Try to open SHOTS directory, create if it doesn't exist
        let shots_dir = match self.volume_manager.open_dir(root_dir, "SHOTS") {
            Ok(dir) => {
                // Close root dir since we have shots dir now
                let _ = self.volume_manager.close_dir(root_dir);
                dir
            }
            Err(_) => {
                // Create SHOTS directory
                debug!("Creating SHOTS directory");
                if let Err(e) = self.volume_manager.make_dir_in_dir(root_dir, "SHOTS") {
                    warn!("Failed to create SHOTS dir: {:?}", defmt::Debug2Format(&e));
                    let _ = self.volume_manager.close_dir(root_dir);
                    let _ = self.volume_manager.close_volume(volume);
                    return Err(ShotLogStorageError::DirectoryError);
                }

                let dir = self.volume_manager.open_dir(root_dir, "SHOTS").map_err(|e| {
                    warn!("Failed to open created SHOTS dir: {:?}", defmt::Debug2Format(&e));
                    let _ = self.volume_manager.close_dir(root_dir);
                    let _ = self.volume_manager.close_volume(volume);
                    ShotLogStorageError::DirectoryError
                })?;

                let _ = self.volume_manager.close_dir(root_dir);
                dir
            }
        };

        // Cache the state for future operations
        self.cached_state = Some(CachedVolumeState { volume, shots_dir });

        Ok(shots_dir)
    }

    /// Invalidate cached filesystem state.
    ///
    /// Call this when:
    /// - SD card is physically removed
    /// - SD card is swapped for a different one
    /// - After filesystem errors to allow recovery
    ///
    /// After calling this, the next operation will re-open the volume
    /// and SHOTS directory.
    pub fn invalidate(&mut self) {
        if let Some(state) = self.cached_state.take() {
            // Close in reverse order: directory first, then volume
            if let Err(e) = self.volume_manager.close_dir(state.shots_dir) {
                warn!("Error closing cached SHOTS dir: {:?}", defmt::Debug2Format(&e));
            }
            if let Err(e) = self.volume_manager.close_volume(state.volume) {
                warn!("Error closing cached volume: {:?}", defmt::Debug2Format(&e));
            }
            debug!("Invalidated cached filesystem state");
        }
    }

    /// Check if filesystem state is currently cached.
    pub fn is_cached(&self) -> bool {
        self.cached_state.is_some()
    }
}

impl<BD, TS, const MAX_DIRS: usize, const MAX_FILES: usize, const MAX_VOLUMES: usize> ShotLogStorage
    for SdCardShotLogStorage<BD, TS, MAX_DIRS, MAX_FILES, MAX_VOLUMES>
where
    BD: BlockDevice,
    BD::Error: Debug,
    TS: TimeSource,
{
    fn store_shot(&mut self, shot: &ShotLog) -> Result<String, ShotLogStorageError> {
        if !self.available {
            return Err(ShotLogStorageError::CardNotPresent);
        }

        // Serialize the shot log
        let data = self.serialize(shot)?;
        let data_len = data.len();

        // Generate filename
        let filename = self.generate_filename(shot);
        debug!("Storing shot log as {}, size={}", filename.as_str(), data_len);

        // Get cached SHOTS directory (opens volume/dir on first call)
        let shots_dir = self.ensure_filesystem()?;
        debug!("Shot dir: {:?}", shots_dir);
        // Create and write the file
        let file = self
            .volume_manager
            .open_file_in_dir(shots_dir, filename.as_str(), Mode::ReadWriteCreateOrTruncate)
            .map_err(|e| {
                warn!("Failed to create file: {:?}", defmt::Debug2Format(&e));
                ShotLogStorageError::WriteError
            })?;
        debug!("Shot file: {:?}", file);
        // Write data - need to reborrow since serialize borrowed self
        let write_result = self.volume_manager.write(file, &self.buffer[..data_len]);
        if let Err(e) = write_result {
            warn!("Failed to write file: {:?}", defmt::Debug2Format(&e));
            let _ = self.volume_manager.close_file(file);
            return Err(ShotLogStorageError::WriteError);
        }
        debug!("Shot file written");
        // Close file (directory stays cached)
        if let Err(e) = self.volume_manager.close_file(file) {
            warn!("Failed to close file: {:?}", defmt::Debug2Format(&e));
        }

        error!("Shot log saved successfully");
        Ok(filename)
    }

    fn list_shots(&mut self) -> Result<Vec<ShotLogFileInfo>, ShotLogStorageError> {
        if !self.available {
            return Err(ShotLogStorageError::CardNotPresent);
        }

        let mut result = Vec::new();

        // Get cached SHOTS directory
        let shots_dir = self.ensure_filesystem()?;

        self.volume_manager
            .iterate_dir(shots_dir, |entry| {
                if !entry.attributes.is_directory() {
                    // Check if it's a .BIN file
                    let name = entry.name.to_string();
                    if name.ends_with(".BIN") {
                        result.push(ShotLogFileInfo {
                            filename: name,
                            size_bytes: entry.size,
                        });
                    }
                }
            })
            .map_err(|e| {
                warn!("Failed to iterate directory: {:?}", defmt::Debug2Format(&e));
                ShotLogStorageError::DirectoryError
            })?;

        Ok(result)
    }

    fn get_shot(&mut self, filename: &str) -> Result<ShotLog, ShotLogStorageError> {
        if !self.available {
            return Err(ShotLogStorageError::CardNotPresent);
        }

        // Get cached SHOTS directory
        let shots_dir = self.ensure_filesystem()?;

        let file = self
            .volume_manager
            .open_file_in_dir(shots_dir, filename, Mode::ReadOnly)
            .map_err(|e| {
                warn!("Failed to open file: {:?}", defmt::Debug2Format(&e));
                ShotLogStorageError::FileNotFound
            })?;

        // Get file length
        let file_length = self.volume_manager.file_length(file).map_err(|e| {
            warn!("Failed to get file length: {:?}", defmt::Debug2Format(&e));
            let _ = self.volume_manager.close_file(file);
            ShotLogStorageError::ReadError
        })?;

        // Read file contents
        let mut data = alloc::vec![0u8; file_length as usize];
        let bytes_read = self.volume_manager.read(file, &mut data).map_err(|e| {
            warn!("Failed to read file: {:?}", defmt::Debug2Format(&e));
            let _ = self.volume_manager.close_file(file);
            ShotLogStorageError::ReadError
        })?;

        // Close file (directory stays cached)
        let _ = self.volume_manager.close_file(file);

        if bytes_read != file_length as usize {
            warn!(
                "Read {} bytes but expected {}",
                bytes_read, file_length
            );
            return Err(ShotLogStorageError::ReadError);
        }

        self.deserialize(&data)
    }

    fn delete_shot(&mut self, filename: &str) -> Result<(), ShotLogStorageError> {
        if !self.available {
            return Err(ShotLogStorageError::CardNotPresent);
        }

        // Get cached SHOTS directory
        let shots_dir = self.ensure_filesystem()?;

        self.volume_manager
            .delete_file_in_dir(shots_dir, filename)
            .map_err(|e| {
                warn!("Failed to delete file: {:?}", defmt::Debug2Format(&e));
                ShotLogStorageError::FileNotFound
            })?;

        Ok(())
    }

    fn is_available(&self) -> bool {
        self.available
    }
}

impl<BD, TS, const MAX_DIRS: usize, const MAX_FILES: usize, const MAX_VOLUMES: usize> Drop
    for SdCardShotLogStorage<BD, TS, MAX_DIRS, MAX_FILES, MAX_VOLUMES>
where
    BD: BlockDevice,
    BD::Error: Debug,
    TS: TimeSource,
{
    fn drop(&mut self) {
        // Close cached handles when storage is dropped
        if let Some(state) = self.cached_state.take() {
            let _ = self.volume_manager.close_dir(state.shots_dir);
            let _ = self.volume_manager.close_volume(state.volume);
        }
    }
}
