use alloc::vec;
use core::ops::{Deref, DerefMut, Range};
use defmt::info;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::mutex::Mutex;
use embedded_storage_async::nor_flash::{ErrorType, MultiwriteNorFlash, NorFlash};
use variegated_controller_types::{BoilerConfiguration, GroupConfiguration};
use heapless::Vec;
use postcard::to_slice;
use sequential_storage::cache::Cache;
use sequential_storage::map::{Key, MapConfig, MapStorage, SerializationError, Value};
use crate::flash::BorrowedFlash;

pub trait SettingsStorage<SettingsT: Default> {
    async fn load_settings(&mut self) -> Result<SettingsT, &'static str>;
    async fn save_settings(&mut self, data: &SettingsT) -> Result<(), &'static str>;
    async fn optimize_storage(&mut self) -> Result<(), &'static str>;
}

pub struct SequentialStorageSettingsStorage<'a, M: RawMutex, T: MultiwriteNorFlash, SettingsT: for<'b> Value<'b> + Default + Clone + PartialEq> {
    _phantom: core::marker::PhantomData<SettingsT>,
    flash: &'a Mutex<M, T>,
    range: Range<u32>,
    deserialization_buffer: [u8; 2048],
    cached_value: Option<SettingsT>,
}

impl <'a, M: RawMutex, T: MultiwriteNorFlash, SettingsT: for<'b> Value<'b> + Default + Clone + PartialEq> SequentialStorageSettingsStorage<'a, M, T, SettingsT> {
    pub fn new(flash: &'a Mutex<M, T>, range: Range<u32>) -> Self {
        Self {
            _phantom: core::marker::PhantomData,
            flash,
            range,
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
            .fetch_item::<SettingsT>(&mut self.deserialization_buffer, &0)
            .await;

        if let Ok(Some(data)) = item {
            // Cache the loaded value
            self.cached_value = Some(data.clone());
            return Ok(data)
        } else {
            if let Err(e) = &item {
                match e {
                    sequential_storage::Error::Storage { value: _ } => {
                        info!("Error Storage");
                    }
                    sequential_storage::Error::FullStorage => {
                        info!("Error FullStorage");
                    }
                    sequential_storage::Error::Corrupted {} => {
                        info!("Error Corrupted");
                    },
                    sequential_storage::Error::BufferTooBig => {
                        info!("Error BufferTooBig");
                    },
                    /// A provided buffer was to small to be used (usize is size needed)
                    sequential_storage::Error::BufferTooSmall(usize) => {
                        info!("Error BufferTooSmall: {}", usize);
                    },
                    /// A serialization error (from the key or value)
                    sequential_storage::Error::SerializationError(SerializationError) => {
                        info!("Error SerializationError: {:?}", SerializationError);
                    },
                    sequential_storage::Error::ItemTooBig => {
                        info!("Error ItemTooBig");
                    },
                    _ => {
                        info!("Some other error");
                    }
                }
            }
            info!("No settings found, using default");
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
                info!("Settings unchanged, skipping flash write");
                return Ok(());
            }
        }

        let mut flash = self.flash.lock().await;
        let mut storage = MapStorage::<u8, _, _>::new(
            BorrowedFlash(flash.deref_mut()),
            MapConfig::try_new(self.range.clone()).map_err(|_| "Invalid settings flash range")?,
            Cache::new_uncached(),
        );

//        let mut serialization_buffer = [0u8; 1024];
        let mut data_buffer = vec![0u8; 40*1024];

        info!("Setting settings");

  //      let s = to_slice(settings, &mut serialization_buffer).map_err(|_| "Serialization failed")?;

    //    info!("Storing settings to flash, actual len = {}", s.len());

        storage.store_item(
            &mut data_buffer,
            &0u8,
            settings
        ).await.expect("Failed to store item");

        // Update the cache with the new settings
        self.cached_value = Some(settings.clone());

        info!("Settings stored successfully");

        Ok(())
    }

    async fn optimize_storage(&mut self) -> Result<(), &'static str> {
        info!("Optimizing configuration storage");

        // Ensure we have loaded/cached the current settings
        if self.cached_value.is_none() {
            self.load_settings().await?;
        }

        // Clone the current settings from cache
        let current_settings = self.cached_value.as_ref()
            .ok_or("Failed to load settings for optimization")?
            .clone();

        // Remove everything
        {
            let mut flash = self.flash.lock().await;
            let mut storage = MapStorage::<u8, _, _>::new(
                BorrowedFlash(flash.deref_mut()),
                MapConfig::try_new(self.range.clone()).map_err(|_| "Invalid settings flash range")?,
                Cache::new_uncached(),
            );

            storage
                .remove_all_items(&mut self.deserialization_buffer)
                .await
                .map_err(|_| "Failed to remove all")?;
        }

        // Write the current settings back
        info!("Rewriting optimized configuration");
        self.save_settings(&current_settings).await?;

        info!("Configuration storage optimization complete");
        Ok(())
    }
}