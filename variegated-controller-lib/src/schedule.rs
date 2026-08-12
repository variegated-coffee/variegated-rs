use alloc::collections::btree_map::BTreeMap;
use alloc::vec::Vec;
use core::ops::{DerefMut, Range};
use chrono::{DateTime, Datelike, Duration, Timelike, TimeZone};
use variegated_log::{log_info, log_warn};
use variegated_controller_types::debug::{name, DebugEvent};
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::channel::{Sender};
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use embedded_storage_async::nor_flash::{MultiwriteNorFlash, NorFlash};
use sequential_storage::cache::Cache;
use sequential_storage::map::{MapConfig, MapStorage};
use crate::flash::BorrowedFlash;
use variegated_controller_types::{MachineCommand, ScheduleItem};
use variegated_timekeeping::TimeKeeper;

/// Calculate when a schedule will next trigger
fn calculate_next_trigger(trigger: &variegated_controller_types::ScheduleTrigger, now: variegated_timekeeping::DateTimeInZone) -> Option<variegated_timekeeping::DateTimeInZone> {
    use variegated_timekeeping::TimeKeeper;

    let timezone = TimeKeeper::timezone();

    // If there's a specific date, the schedule ONLY triggers on that date
    // (for recurring schedules, use on_days instead of on_date)
    if let Some(target_date) = trigger.on_date {
        // Create a datetime for the target date at the specified time
        let target_datetime = target_date.and_hms_opt(trigger.on_hour as u32, trigger.on_minute as u32, 0)?;
        let target_with_tz = timezone.from_local(&target_datetime)?;

        // Only return this schedule if it's in the future
        // Once the time passes, this schedule is done (regardless of 'once' flag)
        if target_with_tz > now {
            return Some(target_with_tz);
        } else {
            return None;
        }
    }

    // Calculate next occurrence based on time and optional days
    // Start with today
    let mut candidate_date = now.date_naive();
    let mut days_checked = 0;

    loop {
        if days_checked > 7 {
            // Prevent infinite loop - checked a full week
            return None;
        }

        let candidate_weekday = candidate_date.weekday();

        // Check if this day matches the day filter (if specified)
        let day_matches = if let Some(ref days) = trigger.on_days {
            days.contains(&candidate_weekday)
        } else {
            true // No day filter, all days match
        };

        if day_matches {
            // Check if we can trigger today or need to wait until this day at the specified time
            let target_time = chrono::NaiveTime::from_hms_opt(trigger.on_hour as u32, trigger.on_minute as u32, 0)?;
            let candidate_datetime = candidate_date.and_time(target_time);
            let candidate_with_tz = timezone.from_local(&candidate_datetime)?;

            if candidate_with_tz > now {
                return Some(candidate_with_tz);
            }
        }

        // Move to next day
        candidate_date = candidate_date.succ_opt()?;
        days_checked += 1;
    }
}

pub async fn run_schedule<M1: RawMutex, M2: RawMutex, ScheduleStoreT: ScheduleStore, const CH_N: usize>(store: &Mutex<M1, ScheduleStoreT>, command_channel: Sender<'static, M2, MachineCommand, CH_N>) -> () {
    loop {
        log_info!("Running schedule task");

        let now = TimeKeeper::now_local();
        if let Some(now) = now {
            let mut store_guard = store.lock().await;
            let schedules = store_guard.schedules_triggering_at(now).await;

            for schedule in schedules {
                log_info!("Schedule triggered: {:?}", schedule);
                for action in &schedule.commands {
                    let command = action.to_machine_command();
                    command_channel.send(command).await;
                    log_info!("Sent scheduled action: {:?}", action);
                }
            }
        }

        // Wait until 5 seconds past the next minute
        if let Some(now) = TimeKeeper::now_local() {
            let seconds_until_next_minute = 60 - now.second() as i64;
            let next_check = now + Duration::seconds(seconds_until_next_minute + 5);

            if let Some(timer) = TimeKeeper::timer_until_local(next_check) {
                timer.await;
            } else {
                // Fallback if timer creation fails
                Timer::after_secs(60).await;
            }
        } else {
            // Fallback if we can't get current time
            Timer::after_secs(60).await;
        }
    }
}

pub trait ScheduleStore {
    async fn add_schedule(&mut self, item: ScheduleItem);
    async fn get_schedules(&mut self) -> impl Iterator<Item = &ScheduleItem>;
    async fn remove_schedule(&mut self, index: usize) -> Option<ScheduleItem>;
    async fn update_schedule(&mut self, index: usize, item: ScheduleItem) -> Result<(), &'static str>;
    async fn get_schedule_count(&mut self) -> usize;
    async fn optimize_storage(&mut self) -> Result<(), &'static str>;

    /// Get the next scheduled event and when it will trigger
    async fn get_next_schedule(&mut self) -> Option<(ScheduleItem, variegated_timekeeping::DateTimeInZone)> {
        let now = TimeKeeper::now_local()?;

        let schedules = self.get_schedules().await;

        let mut next_schedule: Option<(ScheduleItem, variegated_timekeeping::DateTimeInZone)> = None;

        for schedule in schedules {
            if !schedule.trigger_at.enabled {
                continue;
            }

            if let Some(next_trigger) = calculate_next_trigger(&schedule.trigger_at, now) {
                match &mut next_schedule {
                    None => {
                        next_schedule = Some((schedule.clone(), next_trigger));
                    }
                    Some((_, current_next)) if next_trigger < *current_next => {
                        next_schedule = Some((schedule.clone(), next_trigger));
                    }
                    _ => {}
                }
            }
        }

        next_schedule
    }

    async fn schedules_triggering_at(&mut self, time: variegated_timekeeping::DateTimeInZone) -> impl Iterator<Item = &ScheduleItem> {
        let hour = time.hour() as u8;
        let minute = time.minute() as u8;
        let weekday = time.weekday();
        let date = time.date_naive();

        let schedules = self.get_schedules().await;

        schedules.filter(move |schedule| {
            let trigger = &schedule.trigger_at;

            // Check if enabled
            if !trigger.enabled {
                return false;
            }

            // Check hour and minute
            if trigger.on_hour != hour || trigger.on_minute != minute {
                return false;
            }

            // Check day of week if specified
            if let Some(ref days) = trigger.on_days {
                if !days.contains(&weekday) {
                    return false;
                }
            }

            // Check specific date if specified
            if let Some(trigger_date) = trigger.on_date {
                if date != trigger_date {
                    return false;
                }
            }

            true
        })
    }
}

pub struct InMemoryScheduleStore {
    schedules: BTreeMap<usize, ScheduleItem>,
    next_index: usize,
}

impl InMemoryScheduleStore {
    pub fn new() -> Self {
        Self {
            schedules: BTreeMap::new(),
            next_index: 0,
        }
    }
}

impl ScheduleStore for InMemoryScheduleStore {
    async fn add_schedule(&mut self, item: ScheduleItem) {
        // Find the first available index (hole-filling strategy)
        let index = (0..self.next_index)
            .find(|&i| !self.schedules.contains_key(&i))
            .unwrap_or_else(|| {
                // No holes found, use next_index and increment it
                let idx = self.next_index;
                self.next_index += 1;
                idx
            });

        self.schedules.insert(index, item);
    }

    async fn get_schedules(&mut self) -> impl Iterator<Item = &ScheduleItem> {
        self.schedules.values()
    }

    async fn remove_schedule(&mut self, index: usize) -> Option<ScheduleItem> {
        self.schedules.remove(&index)
    }

    async fn update_schedule(&mut self, index: usize, item: ScheduleItem) -> Result<(), &'static str> {
        if self.schedules.contains_key(&index) {
            self.schedules.insert(index, item);
            Ok(())
        } else {
            Err("Index out of bounds")
        }
    }

    async fn get_schedule_count(&mut self) -> usize {
        self.schedules.len()
    }

    async fn optimize_storage(&mut self) -> Result<(), &'static str> {
        // Compact indices to be consecutive (0, 1, 2, ...)
        let schedules: Vec<ScheduleItem> = self.schedules.values().cloned().collect();
        self.schedules.clear();

        for (new_index, schedule) in schedules.into_iter().enumerate() {
            self.schedules.insert(new_index, schedule);
        }

        // Reset next_index to the number of schedules
        self.next_index = self.schedules.len();

        Ok(())
    }
}

pub struct SequentialStorageScheduleStore<'a, M: RawMutex, T: MultiwriteNorFlash> {
    flash: &'a Mutex<M, T>,
    range: Range<u32>,
    deserialization_buffer: [u8; 2048],
    cache: BTreeMap<usize, ScheduleItem>,
    cache_initialized: bool,
    next_index: usize,
}

impl <'a, M: RawMutex, T: MultiwriteNorFlash> SequentialStorageScheduleStore<'a, M, T> {
    pub fn new(flash: &'a Mutex<M, T>, range: Range<u32>) -> Self {
        Self {
            flash,
            range,
            deserialization_buffer: [0u8; 2048],
            cache: BTreeMap::new(),
            cache_initialized: false,
            next_index: 0,
        }
    }

    pub async fn load_from_flash(&mut self) -> Result<(), &'static str> {
        if self.cache_initialized {
            log_info!("Schedule store cache already initialized, skipping load");
            return Ok(());
        }

        let mut guard = self.flash.lock().await;
        let mut storage = MapStorage::<u16, _, _>::new(
            BorrowedFlash(guard.deref_mut()),
            MapConfig::try_new(self.range.clone()).map_err(|_| "Invalid schedule flash range")?,
            Cache::new_uncached(),
        );

        log_info!("Loading schedules from flash...");
        // Create the iterator of map items
        let mut iterator = storage
            .fetch_all_items(&mut self.deserialization_buffer)
            .await
            .unwrap();

        let mut max_index = 0usize;
        while let item = iterator
            .next::<Option<ScheduleItem>>(&mut self.deserialization_buffer)
            .await
        {
            let Ok(item) = item else {
                log_warn!("Invalid schedule item encountered in flash, stopping load");
                break;
            };

            let Some((key, value)) = item else {
                log_info!("Skipping invalid schedule item in flash");
                break;
            };

            log_info!("Loaded schedule at index {}", key);
            let index = key as usize;
            if index > max_index {
                max_index = index;
            }

            if let Some(schedule) = value {
                self.cache.insert(index, schedule);
            } else {
                self.cache.remove(&index);
            }
        }

        // Set next_index to one past the highest loaded index
        // If no schedules were loaded, max_index is 0, so next_index will be 0
        // If schedules exist, next_index will be max_index + 1
        if !self.cache.is_empty() {
            self.next_index = max_index + 1;
        } else {
            self.next_index = 0;
        }

        self.cache_initialized = true;

        Ok(())
    }

    async fn store_in_flash(&mut self, index: usize, schedule: &Option<ScheduleItem>) -> Result<(), &'static str> {
        let mut guard = self.flash.lock().await;
        let mut storage = MapStorage::<u16, _, _>::new(
            BorrowedFlash(guard.deref_mut()),
            MapConfig::try_new(self.range.clone()).map_err(|_| "Invalid schedule flash range")?,
            Cache::new_uncached(),
        );

        let key = index as u16;

        storage.store_item(
            &mut self.deserialization_buffer,
            &key,
            schedule
        ).await
            .map_err(|_| "Failed to store schedule item in flash")?;

        // @todo Handle full storage by erasing the range and rewriting all items

        variegated_log::emit_event(DebugEvent::StorageWrite { store: name("schedules"), index: index as u16 });

        Ok(())
    }
}

impl <'a, M: RawMutex, T: MultiwriteNorFlash> ScheduleStore for SequentialStorageScheduleStore<'a, M, T> {
    async fn add_schedule(&mut self, item: ScheduleItem) {
        self.load_from_flash().await.ok().unwrap();

        // Find the first available index (hole-filling strategy)
        let index = (0..self.next_index)
            .find(|&i| !self.cache.contains_key(&i))
            .unwrap_or_else(|| {
                // No holes found, use next_index and increment it
                let idx = self.next_index;
                self.next_index += 1;
                idx
            });

        let opt = Some(item);
        self.store_in_flash(index, &opt).await.expect("Failed to store schedule in flash");
        self.cache.insert(index, opt.unwrap());
    }

    async fn get_schedules(&mut self) -> impl Iterator<Item = &ScheduleItem> {
        let res = self.load_from_flash().await;
        if res.is_err() {
            log_info!("Error loading schedules from flash: {:?}", res.err());
        }

        self.cache.values()
    }

    async fn remove_schedule(&mut self, index: usize) -> Option<ScheduleItem> {
        let schedule = self.cache.remove(&index);
        if schedule.is_some() {
            let opt: Option<ScheduleItem> = None;
            let _ = self.store_in_flash(index, &opt).await;
        }

        schedule
    }

    async fn update_schedule(&mut self, index: usize, item: ScheduleItem) -> Result<(), &'static str> {
        self.load_from_flash().await?;

        if self.cache.contains_key(&index) {
            let opt = Some(item);
            self.store_in_flash(index, &opt).await?;
            self.cache.insert(index, opt.unwrap());
            Ok(())
        } else {
            Err("Index out of bounds")
        }
    }

    async fn get_schedule_count(&mut self) -> usize {
        if let Err(e) = self.load_from_flash().await {
            log_info!("Error loading schedules from flash: {:?}", e);
            return 0;
        }

        self.cache.len()
    }

    async fn optimize_storage(&mut self) -> Result<(), &'static str> {
        log_info!("Optimizing schedule storage");

        // Load all schedules into cache if not already loaded
        self.load_from_flash().await?;

        // Collect schedules and reassign to consecutive indices (0, 1, 2, ...)
        let schedules_to_store: Vec<ScheduleItem> = self.cache.values()
            .cloned()
            .collect();

        // Clear the cache as we'll rebuild it with new indices
        self.cache.clear();

        // Remove everything
        {
            let mut flash = self.flash.lock().await;
            let mut storage = MapStorage::<u16, _, _>::new(
                BorrowedFlash(flash.deref_mut()),
                MapConfig::try_new(self.range.clone()).map_err(|_| "Invalid schedule flash range")?,
                Cache::new_uncached(),
            );

            storage
                .remove_all_items(&mut self.deserialization_buffer)
                .await
                .map_err(|_| "Failed to remove schedule item in flash")?;
        }

        // Re-store all schedules with consecutive indices starting from 0
        log_info!("Rewriting {} schedules with compacted indices", schedules_to_store.len());
        for (new_index, schedule) in schedules_to_store.iter().enumerate() {
            let opt = Some(schedule.clone());
            self.store_in_flash(new_index, &opt).await?;
            self.cache.insert(new_index, schedule.clone());
        }

        // Reset next_index to the number of schedules
        self.next_index = schedules_to_store.len();

        log_info!("Schedule storage optimization complete, next_index reset to {}", self.next_index);
        Ok(())
    }
}