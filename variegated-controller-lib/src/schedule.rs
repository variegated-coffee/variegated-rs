use alloc::collections::btree_map::BTreeMap;
use alloc::vec::Vec;
use core::ops::{DerefMut, Range};
use chrono::{DateTime, Datelike, Duration, Timelike, TimeZone};
use defmt::info;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::channel::{Sender};
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use embedded_storage_async::nor_flash::NorFlash;
use sequential_storage::cache::NoCache;
use sequential_storage::map::{fetch_all_items, store_item};
use variegated_controller_types::{MachineCommand, ScheduleItem};
use variegated_timekeeping::TimeKeeper;

pub async fn run_schedule<M1: RawMutex, M2: RawMutex, ScheduleStoreT: ScheduleStore, const CH_N: usize>(store: &Mutex<M1, ScheduleStoreT>, command_channel: Sender<'static, M2, MachineCommand, CH_N>) -> () {
    loop {
        info!("Running schedule task");

        let now = TimeKeeper::now_local();
        if let Some(now) = now {
            let mut store_guard = store.lock().await;
            let schedules = store_guard.schedules_triggering_at(now).await;

            for schedule in schedules {
                info!("Schedule triggered: {:?}", schedule);
                for command in &schedule.commands {
                    command_channel.send(command.clone()).await;
                    info!("Sent scheduled command: {:?}", command);
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

    async fn schedules_triggering_at<Tz: TimeZone>(&mut self, time: DateTime<Tz>) -> impl Iterator<Item = &ScheduleItem> {
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
    schedules: Vec<ScheduleItem>,
}

impl InMemoryScheduleStore {
    pub fn new() -> Self {
        Self {
            schedules: Vec::new(),
        }
    }
}

impl ScheduleStore for InMemoryScheduleStore {
    async fn add_schedule(&mut self, item: ScheduleItem) {
        self.schedules.push(item);
    }

    async fn get_schedules(&mut self) -> impl Iterator<Item = &ScheduleItem> {
        self.schedules.iter()
    }

    async fn remove_schedule(&mut self, index: usize) -> Option<ScheduleItem> {
        if index < self.schedules.len() {
            Some(self.schedules.remove(index))
        } else {
            None
        }
    }

    async fn update_schedule(&mut self, index: usize, item: ScheduleItem) -> Result<(), &'static str> {
        if index < self.schedules.len() {
            self.schedules[index] = item;
            Ok(())
        } else {
            Err("Index out of bounds")
        }
    }

    async fn get_schedule_count(&mut self) -> usize {
        self.schedules.len()
    }
}

pub struct SequentialStorageScheduleStore<'a, M: RawMutex, T: NorFlash> {
    flash: &'a Mutex<M, T>,
    range: Range<u32>,
    deserialization_buffer: [u8; 2048],
    cache: BTreeMap<usize, ScheduleItem>,
    cache_initialized: bool
}

impl <'a, M: RawMutex, T: NorFlash> SequentialStorageScheduleStore<'a, M, T> {
    pub fn new(flash: &'a Mutex<M, T>, range: Range<u32>) -> Self {
        Self {
            flash,
            range,
            deserialization_buffer: [0u8; 2048],
            cache: BTreeMap::new(),
            cache_initialized: false
        }
    }

    pub async fn load_from_flash(&mut self) -> Result<(), &'static str> {
        if self.cache_initialized {
            info!("Schedule store cache already initialized, skipping load");
            return Ok(());
        }

        let mut cache = NoCache::new();

        let mut guard = self.flash.lock().await;
        let flash_ref = guard.deref_mut();

        info!("Loading schedules from flash...");
        // Create the iterator of map items
        let mut iterator = fetch_all_items::<u16, _, _>(
            flash_ref,
            self.range.clone(),
            &mut cache,
            &mut self.deserialization_buffer
        )
        .await
        .unwrap();

        while let Some((key, value)) = iterator
            .next::<Option<ScheduleItem>>(&mut self.deserialization_buffer)
            .await
            .unwrap()
        {
            info!("Loaded schedule at index {}", key);
            if let Some(schedule) = value {
                self.cache.insert(key as usize, schedule);
            } else {
                self.cache.remove(&(key as usize));
            }
        }

        self.cache_initialized = true;

        Ok(())
    }

    async fn store_in_flash(&mut self, index: usize, schedule: &Option<ScheduleItem>) -> Result<(), &'static str> {
        let mut guard = self.flash.lock().await;
        let flash_ref = guard.deref_mut();

        let mut cache = NoCache::new();

        let key = index as u16;

        store_item(
            flash_ref,
            self.range.clone(),
            &mut cache,
            &mut self.deserialization_buffer,
            &key,
            schedule
        ).await
            .map_err(|_| "Failed to store schedule item in flash")?;

        // @todo Handle full storage by erasing the range and rewriting all items

        info!("Stored schedule item at index {} in flash", index);

        Ok(())
    }
}

impl <'a, M: RawMutex, T: NorFlash> ScheduleStore for SequentialStorageScheduleStore<'a, M, T> {
    async fn add_schedule(&mut self, item: ScheduleItem) {
        self.load_from_flash().await.ok().unwrap();
        let index = self.cache.len();
        let opt = Some(item);
        self.store_in_flash(index, &opt).await.expect("Failed to store schedule in flash");
        self.cache.insert(index, opt.unwrap());
    }

    async fn get_schedules(&mut self) -> impl Iterator<Item = &ScheduleItem> {
        let res = self.load_from_flash().await;
        if res.is_err() {
            info!("Error loading schedules from flash: {:?}", res.err());
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
            info!("Error loading schedules from flash: {:?}", e);
            return 0;
        }

        self.cache.len()
    }
}