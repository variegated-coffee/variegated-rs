use alloc::vec;
use core::ops::{DerefMut, Range};
use variegated_log::log_info;
use variegated_controller_types::debug::{name, DebugEvent};
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::mutex::Mutex;
use embedded_storage_async::nor_flash::MultiwriteNorFlash;
use sequential_storage::cache::Cache;
use sequential_storage::map::{MapConfig, MapStorage, Value};
use crate::flash::BorrowedFlash;

/// Keys within a shared settings flash range.
///
/// [`SequentialStorageSettingsStorage`] stores one value under one key in a
/// `sequential_storage` map, and several stores may share a range. That only works if the
/// allocation is recorded in one place: two stores that pick the same number silently
/// overwrite each other, and the symptom is a setting that reverts rather than anything
/// that looks like a collision.
///
/// Append here; never renumber. A key is baked into the flash of every machine already
/// running this firmware, so changing one is a silent factory reset of that value.
pub mod key {
    /// The machine's persistent configuration -- the value already stored on every
    /// machine, so this one is not merely a convention but a fact about existing flash.
    pub const CONFIGURATION: u8 = 0;
    /// Wi-Fi credentials, provisioned over Improv.
    pub const WIFI_CREDENTIALS: u8 = 1;
    /// Bluetooth peripheral associations. Formerly at a flash range of their own
    /// (`0x0010_0000..0x0012_0000`); machines upgraded across that move lose their
    /// pairings once and re-pair.
    pub const BLUETOOTH_ASSOCIATIONS: u8 = 2;
    /// Where finished shot logs are uploaded, and the token that authorises it. Set over
    /// the debug bus by `MachineCommand::SetShotUploadConfig`, and pushed to the comms
    /// processor, which has no flash to keep it in.
    pub const SHOT_UPLOAD_CONFIG: u8 = 3;
    /// The machine's timezone, as an IANA zone name. Scheduling runs on it; logs stay UTC.
    ///
    /// A key of its own, and that is the whole reason it is not a field on the configuration
    /// blob: appending there would make every previously stored copy fail to deserialize, and
    /// `load_settings` maps that to `Default` -- a silent factory reset of every machine's
    /// boiler setpoints in the field, to buy one string. See the note on this module.
    pub const TIMEZONE: u8 = 4;
}

/// The flash range every settings store lives in.
///
/// **A constant rather than a literal at each call site.** Both boards wrote
/// `0x0000_0000..0x0008_0000` into their own `main.rs`, three times each, alongside the
/// same explanatory comments -- so changing where a machine keeps its settings meant
/// finding six literals across two binaries, and getting one wrong reads as that machine
/// having been reset to defaults rather than as a mistake.
///
/// The stores that share it are distinguished by [`key`], not by address.
pub const SETTINGS_RANGE: Range<u32> = 0x0000_0000..0x0008_0000;

/// The flash range the routine repository lives in, on every board that has one.
///
/// Here rather than beside [`crate::routine::SequentialStorageRoutineRepository`] for the
/// same reason [`SETTINGS_RANGE`] is here: this module is where the external flash's map is
/// written down, including the range abandoned by the Bluetooth associations below. A range
/// recorded next to the code that reads it is a range the next allocation cannot see.
///
/// Unlike the settings, this one is a whole range rather than a key, because a routine
/// repository is a keyed collection of its own -- `u16` storage indices carrying a
/// [`variegated_controller_types::RoutineIndex`] -- and not one value under one key.
pub const ROUTINES_RANGE: Range<u32> = 0x0008_0000..0x0010_0000;

/// The five stores every machine keeps, over one flash range.
///
/// Returned rather than boxed into a struct because each has a different `SettingsT` and
/// the caller wraps them in `Mutex`es of its own choosing.
///
/// The configuration store's type is the caller's: `DualBoilerSingleGroupPersistentConfiguration`
/// on one board and `SingleBoilerSingleGroupPersistentConfiguration` on the other. That is
/// the only thing that genuinely differed between the two copies of this.
pub fn machine_stores<'a, M, T, ConfigT>(
    flash: &'a Mutex<M, T>,
) -> (
    SequentialStorageSettingsStorage<'a, M, T, ConfigT>,
    SequentialStorageSettingsStorage<'a, M, T, variegated_controller_types::bluetooth::BluetoothAssociations>,
    SequentialStorageSettingsStorage<'a, M, T, variegated_controller_types::wifi::StoredWifiCredentials>,
    SequentialStorageSettingsStorage<'a, M, T, variegated_controller_types::shot_upload::ShotUploadConfig>,
    SequentialStorageSettingsStorage<'a, M, T, variegated_controller_types::timezone::TimezoneSetting>,
)
where
    M: RawMutex,
    T: MultiwriteNorFlash,
    ConfigT: for<'b> Value<'b> + Default + Clone + PartialEq,
{
    (
        SequentialStorageSettingsStorage::new(flash, SETTINGS_RANGE),
        // Keys rather than ranges of their own. Appending these to the configuration blob
        // instead would make every previously stored copy fail to deserialize -- postcard
        // is positional and these blobs carry no version -- and silently reset the machine
        // to defaults on the first boot after the upgrade.
        //
        // The associations formerly had a range of their own, `0x0010_0000..0x0012_0000`.
        // It is abandoned rather than reused, so a rolled-back firmware still finds them;
        // machines upgraded across that change forget their pairings once.
        SequentialStorageSettingsStorage::new_with_key(
            flash,
            SETTINGS_RANGE,
            key::BLUETOOTH_ASSOCIATIONS,
        ),
        SequentialStorageSettingsStorage::new_with_key(
            flash,
            SETTINGS_RANGE,
            key::WIFI_CREDENTIALS,
        ),
        SequentialStorageSettingsStorage::new_with_key(
            flash,
            SETTINGS_RANGE,
            key::SHOT_UPLOAD_CONFIG,
        ),
        SequentialStorageSettingsStorage::new_with_key(
            flash,
            SETTINGS_RANGE,
            key::TIMEZONE,
        ),
    )
}

// See `RoutineRepository` in `routine.rs`: one executor, one core, so the `Send` bound the
// lint wants to make available would never be asked for.
#[allow(async_fn_in_trait)]
pub trait SettingsStorage<SettingsT: Default> {
    async fn load_settings(&mut self) -> Result<SettingsT, &'static str>;
    async fn save_settings(&mut self, data: &SettingsT) -> Result<(), &'static str>;
    async fn optimize_storage(&mut self) -> Result<(), &'static str>;
}

pub struct SequentialStorageSettingsStorage<'a, M: RawMutex, T: MultiwriteNorFlash, SettingsT: for<'b> Value<'b> + Default + Clone + PartialEq> {
    _phantom: core::marker::PhantomData<SettingsT>,
    flash: &'a Mutex<M, T>,
    range: Range<u32>,
    /// Which key in `range` this store owns.
    ///
    /// Load, save and optimize all act on this key alone, which is what lets several
    /// stores share one flash range. See [`key`] for the allocation.
    key: u8,
    deserialization_buffer: [u8; 2048],
    cached_value: Option<SettingsT>,
}

impl <'a, M: RawMutex, T: MultiwriteNorFlash, SettingsT: for<'b> Value<'b> + Default + Clone + PartialEq> SequentialStorageSettingsStorage<'a, M, T, SettingsT> {
    /// A store owning [`key::CONFIGURATION`] in `range`.
    pub fn new(flash: &'a Mutex<M, T>, range: Range<u32>) -> Self {
        Self::new_with_key(flash, range, key::CONFIGURATION)
    }

    /// A store owning `key` in `range`.
    ///
    /// Several stores may share a range provided they take different keys from [`key`].
    pub fn new_with_key(flash: &'a Mutex<M, T>, range: Range<u32>, key: u8) -> Self {
        Self {
            _phantom: core::marker::PhantomData,
            flash,
            range,
            key,
            deserialization_buffer: [0u8; 2048],
            cached_value: None,
        }
    }
}

impl<'a, M: RawMutex, T: MultiwriteNorFlash, SettingsT: for<'b> Value<'b> + Default + Clone + PartialEq> SettingsStorage<SettingsT> for SequentialStorageSettingsStorage<'a, M, T, SettingsT> {
    async fn load_settings(&mut self) -> Result<SettingsT, &'static str>
    {
        // Return cached value if available
        if let Some(cached) = &self.cached_value {
            return Ok(cached.clone());
        }

        // Otherwise load from flash
        let mut flash = self.flash.lock().await;
        let mut storage = MapStorage::<u8, _, _>::new(
            BorrowedFlash(flash.deref_mut()),
            MapConfig::try_new(self.range.clone()).map_err(|_| "Invalid settings flash range")?,
            Cache::new_uncached(),
        );

        let item = storage
            .fetch_item::<SettingsT>(&mut self.deserialization_buffer, &self.key)
            .await;

        if let Ok(Some(data)) = item {
            // Cache the loaded value
            self.cached_value = Some(data.clone());
            return Ok(data)
        } else {
            if let Err(e) = &item {
                match e {
                    sequential_storage::Error::Storage { value: _ } => {
                        log_info!("Error Storage");
                    }
                    sequential_storage::Error::FullStorage => {
                        log_info!("Error FullStorage");
                    }
                    sequential_storage::Error::Corrupted {} => {
                        log_info!("Error Corrupted");
                    },
                    sequential_storage::Error::BufferTooBig => {
                        log_info!("Error BufferTooBig");
                    },
                    // A provided buffer was too small to be used (the value is the size needed)
                    sequential_storage::Error::BufferTooSmall(needed) => {
                        log_info!("Error BufferTooSmall: {}", needed);
                    },
                    // A serialization error (from the key or value)
                    sequential_storage::Error::SerializationError(err) => {
                        log_info!("Error SerializationError: {:?}", err);
                    },
                    sequential_storage::Error::ItemTooBig => {
                        log_info!("Error ItemTooBig");
                    },
                    _ => {
                        log_info!("Some other error");
                    }
                }
            }
            log_info!("No settings found, using default");
        }

        // Use default and cache it
        let default_settings = SettingsT::default();
        self.cached_value = Some(default_settings.clone());
        Ok(default_settings)
    }

    async fn save_settings(&mut self, settings: &SettingsT) -> Result<(), &'static str> {
        // Check if settings are the same as cached value
        if let Some(ref cached) = self.cached_value {
            if cached == settings {
                log_info!("Settings unchanged, skipping flash write");
                return Ok(());
            }
        }

        let mut flash = self.flash.lock().await;
        let mut storage = MapStorage::<u8, _, _>::new(
            BorrowedFlash(flash.deref_mut()),
            MapConfig::try_new(self.range.clone()).map_err(|_| "Invalid settings flash range")?,
            Cache::new_uncached(),
        );

        let mut data_buffer = vec![0u8; 40*1024];

        log_info!("Setting settings");

        storage.store_item(
            &mut data_buffer,
            &self.key,
            settings
        ).await.expect("Failed to store item");

        // Update the cache with the new settings
        self.cached_value = Some(settings.clone());

        // `index` carries the key, not a constant 0. Stores sharing a range are otherwise
        // indistinguishable in the debug stream, and "which of three settings blobs just
        // wrote" is the entire question a reader of this event has.
        variegated_log::emit_event(DebugEvent::StorageWrite { store: name("settings"), index: self.key as u16 });

        Ok(())
    }

    async fn optimize_storage(&mut self) -> Result<(), &'static str> {
        log_info!("Optimizing configuration storage");

        // Ensure we have loaded/cached the current settings
        if self.cached_value.is_none() {
            self.load_settings().await?;
        }

        // Clone the current settings from cache
        let current_settings = self.cached_value.as_ref()
            .ok_or("Failed to load settings for optimization")?
            .clone();

        // Remove this store's item only.
        //
        // **Not `remove_all_items`**, which is what this used to call. That marks *every*
        // key in the range deleted, and this method then writes back only its own -- so on
        // a range shared by several stores, optimizing one silently destroyed the others,
        // with the loss invisible until the next boot re-read them as absent. Nothing
        // depended on the wider erase: the point of this method is to compact the log for
        // the value it owns, and `remove_item` walks the same `remove_item_inner` path with
        // a key filter.
        {
            let mut flash = self.flash.lock().await;
            let mut storage = MapStorage::<u8, _, _>::new(
                BorrowedFlash(flash.deref_mut()),
                MapConfig::try_new(self.range.clone()).map_err(|_| "Invalid settings flash range")?,
                Cache::new_uncached(),
            );

            storage
                .remove_item(&mut self.deserialization_buffer, &self.key)
                .await
                .map_err(|_| "Failed to remove item")?;
        }

        // Write the current settings back
        log_info!("Rewriting optimized configuration");
        self.save_settings(&current_settings).await?;

        log_info!("Configuration storage optimization complete");
        Ok(())
    }
}