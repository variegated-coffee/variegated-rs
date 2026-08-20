use alloc::collections::btree_map::BTreeMap;
use alloc::vec::Vec;
use core::ops::{DerefMut, Range};
use chrono::{Datelike, Duration};
use variegated_log::{log_info, log_warn};
use variegated_controller_types::debug::{name, CheckinDetail, CheckinStatus, DebugEvent};
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::channel::{Sender};
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use embedded_storage_async::nor_flash::MultiwriteNorFlash;
use sequential_storage::cache::Cache;
use sequential_storage::map::{MapConfig, MapStorage};
use crate::flash::BorrowedFlash;
use variegated_controller_types::{MachineCommand, ScheduleItem};
use variegated_timekeeping::TimeKeeper;

/// Does `trigger` fire at `time`?
///
/// A free function rather than a closure inside the store, because it is the one piece of this
/// file that two callers must agree on and the only one a host test can state on its own -- it
/// needs no store, no flash and no [`TimeKeeper`], since a `DateTimeInZone` can be built
/// directly.
///
/// **Minute resolution, deliberately.** [`run_schedule`] wakes five seconds past each minute,
/// so seconds are ignored entirely and a schedule fires at most once per minute per pass.
///
/// A `None` `on_days` means every day; an *empty* day set means no day at all. Those are
/// different answers and collapsing them would turn a schedule that can never fire into a
/// daily one.
pub fn trigger_matches(
    trigger: &variegated_controller_types::ScheduleTrigger,
    time: variegated_timekeeping::DateTimeInZone,
) -> bool {
    if !trigger.enabled {
        return false;
    }

    if trigger.on_hour != time.hour() as u8 || trigger.on_minute != time.minute() as u8 {
        return false;
    }

    if let Some(ref days) = trigger.on_days {
        if !days.contains(&time.weekday()) {
            return false;
        }
    }

    // A dated trigger fires on that date and no other, which is also what makes it
    // self-expiring: once the date is past, nothing matches it again.
    if let Some(trigger_date) = trigger.on_date {
        if time.date_naive() != trigger_date {
            return false;
        }
    }

    true
}

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

/// Fire scheduled actions, forever.
///
/// `checkin` reports what each pass could actually do. The distinction it carries is the one
/// this loop is otherwise silent about: with no wall clock -- no RTC, or an RTC that has
/// never been set -- it wakes on schedule and fires nothing, and from outside that is
/// indistinguishable from a machine with nothing scheduled. Pass
/// [`variegated_checkin::CheckinHandle::none`] to opt out.
pub async fn run_schedule<M1: RawMutex, M2: RawMutex, ScheduleStoreT: ScheduleStore, const CH_N: usize>(store: &Mutex<M1, ScheduleStoreT>, command_channel: Sender<'static, M2, MachineCommand, CH_N>, checkin: variegated_checkin::CheckinHandle) -> () {
    loop {
        log_info!("Running schedule task");

        let now = TimeKeeper::now_local();
        // Reported before the work rather than after it, because the branch below is where
        // the answer is already known and the tail of this loop has two exits.
        checkin.record(match now {
            Some(_) => CheckinStatus::Good,
            None => CheckinStatus::Warning(CheckinDetail::PreconditionUnmet),
        });

        if let Some(now) = now {
            // Collected, and the guard released, before anything is sent. Two reasons, both
            // load-bearing:
            //
            // * The iterator this replaced held the store's `&mut self` borrow for the whole
            //   loop, so writing `enabled = false` back inside it was `E0499`.
            //
            // * `command_channel` is ten deep and its consumer is the controller's own task.
            //   Awaiting a `send` on a full channel while holding this lock stalls
            //   `create_general_configuration`, whose 100 ms `with_timeout` on this very lock
            //   then expires -- and its timeout path publishes an **empty** schedule list to
            //   every browser and to the comms processor. The GS3 panel already drops this
            //   lock before sending; this was the last place that did not.
            let mut store_guard = store.lock().await;
            let triggering = store_guard.schedules_triggering_at_with_indices(now).await;

            // Spent *before* the actions are dispatched, not after. A one-shot whose action is
            // `SetMachineMode(On)` must not be able to fire twice if the sends below park, and
            // this loop's next pass is a minute away either way.
            //
            // **Disabled, not removed.** The user wrote this schedule; re-enabling or re-timing
            // it is one press where re-creating it is not. `get_next_schedule` filters on
            // `enabled`, so it leaves the "next scheduled" display either way, and the row stays
            // visible in the menu and the browser as *spent* rather than vanishing unexplained.
            let mut spent_any = false;
            for (index, item) in &triggering {
                if !item.trigger_at.once {
                    continue;
                }

                let mut updated = item.clone();
                updated.trigger_at.enabled = false;
                match store_guard.update_schedule(*index, updated).await {
                    Ok(()) => {
                        log_info!("Schedule {} was once-only; disabled", index);
                        spent_any = true;
                    }
                    // Logged and then ignored: a flash write that failed must not stop the
                    // machine doing what it was told to do at this minute. The cost is a
                    // one-shot that fires again after a reboot, which is the lesser failure.
                    Err(e) => log_warn!("Failed to disable one-shot schedule {}: {}", index, e),
                }
            }
            drop(store_guard);

            for (_, schedule) in &triggering {
                log_info!("Schedule triggered: {:?}", schedule);
                for action in &schedule.commands {
                    let command = action.to_machine_command();
                    command_channel.send(command).await;
                    log_info!("Sent scheduled action: {:?}", action);
                }
            }

            // The store just changed and nothing else knows. `RequestConfiguration` is the
            // controller's "republish what you have", and it is this task's only route to a
            // browser -- without it a schedule that has just spent itself keeps reading as
            // enabled until something else dirties the configuration.
            if spent_any {
                command_channel.send(MachineCommand::RequestConfiguration).await;
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

// See `RoutineRepository` in `routine.rs` for why these are `async fn` rather than
// `-> impl Future + Send`: one executor, one core, and `get_schedules` returns an iterator
// borrowed from `&mut self`.
#[allow(async_fn_in_trait)]
pub trait ScheduleStore {
    /// Store a new schedule, and say where it went.
    ///
    /// `Err` rather than a panic. This is reached from `MachineCommand::AddScheduleItem` --
    /// so from a browser, the debug link or the GS3 panel -- and a storage range that had
    /// filled up used to take the machine down with
    /// `expect("Failed to store schedule in flash")`.
    ///
    /// The index is returned because it is not the caller's to predict: the store fills holes
    /// left by [`Self::remove_schedule`], so it is neither the count nor the last index plus
    /// one. `RoutineRepository::add_routine` already answers the same way.
    async fn add_schedule(&mut self, item: ScheduleItem) -> Result<usize, &'static str>;
    async fn get_schedules(&mut self) -> impl Iterator<Item = &ScheduleItem>;

    /// Every stored schedule, with the index it is stored under.
    ///
    /// **Not derivable from [`Self::get_schedules`]**, which yields `BTreeMap::values()` and so
    /// discards them. The indices are sparse: `add_schedule` fills holes left by
    /// `remove_schedule`, so after one removal the *n*th value is no longer index *n*. Every
    /// write command -- [`MachineCommand::UpdateScheduleItem`], [`MachineCommand::RemoveScheduleItem`]
    /// -- names the storage index, so anything that resolves a user's choice back into a command
    /// needs this rather than a position. `RoutineRepository::iterate_routines_with_indices`
    /// exists for the same reason.
    ///
    /// No default body: a store added later has to answer this deliberately, because the
    /// obvious default -- enumerating [`Self::get_schedules`] -- is exactly the bug.
    async fn iterate_schedules_with_indices(&mut self) -> impl Iterator<Item = (usize, &ScheduleItem)>;
    /// Forget a stored schedule.
    ///
    /// `Ok(None)` means there was nothing at `index`. `Err` means there *was* and it could not
    /// be erased. Those used to be the same answer: the flash write's `Result` was discarded
    /// with `let _ =` and the cache entry removed *first*, so a failed erase reported success,
    /// the schedule disappeared from every UI, and it came back at the next boot.
    async fn remove_schedule(&mut self, index: usize) -> Result<Option<ScheduleItem>, &'static str>;
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

    /// Every schedule that fires at `time`, with the index it is stored under.
    ///
    /// **Owned, and that is the whole point.** The iterator this replaced borrowed `&mut self`
    /// for as long as it was alive, so writing back to the store while walking it -- which is
    /// exactly what honouring [`ScheduleTrigger::once`] requires -- was `E0499`. Collecting
    /// also lets [`run_schedule`] release the store lock before it sends anything on the
    /// command channel; see the note there, because that matters more than it looks.
    ///
    /// The indices are the same sparse storage indices
    /// [`Self::iterate_schedules_with_indices`] yields, for the same reason: every write
    /// command names one, and the *n*th value is not index *n* after a removal.
    ///
    /// Defaulted rather than required, unlike `iterate_schedules_with_indices`: this body is
    /// derivable from that one without the bug that motivated its "no default body" note,
    /// because it is that method it enumerates.
    async fn schedules_triggering_at_with_indices(
        &mut self,
        time: variegated_timekeeping::DateTimeInZone,
    ) -> Vec<(usize, ScheduleItem)> {
        self.iterate_schedules_with_indices()
            .await
            .filter(|(_, item)| trigger_matches(&item.trigger_at, time))
            .map(|(index, item)| (index, item.clone()))
            .collect()
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
    async fn add_schedule(&mut self, item: ScheduleItem) -> Result<usize, &'static str> {
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
        // Infallible here, and the `Result` is the trait's rather than this store's: the
        // flash-backed one is the one that can fail.
        Ok(index)
    }

    async fn get_schedules(&mut self) -> impl Iterator<Item = &ScheduleItem> {
        self.schedules.values()
    }

    async fn iterate_schedules_with_indices(&mut self) -> impl Iterator<Item = (usize, &ScheduleItem)> {
        self.schedules.iter().map(|(index, item)| (*index, item))
    }

    async fn remove_schedule(&mut self, index: usize) -> Result<Option<ScheduleItem>, &'static str> {
        Ok(self.schedules.remove(&index))
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
            //log_info!("Schedule store cache already initialized, skipping load");
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
        // `loop`, not `while let item = ...`: that pattern is irrefutable, so it never ended
        // the loop and only read as though it might. What actually terminates this is the
        // two `else { break }` arms below -- an `Err` from the iterator, or the `None` that
        // means the end of the map. Written as a `while let` it looked bounded, and losing
        // either `break` would hang the machine at boot while loading the schedule.
        loop {
            let item = iterator
                .next::<Option<ScheduleItem>>(&mut self.deserialization_buffer)
                .await;

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

        // `try_from`, not `as`. The flash key is the storage index narrowed to sixteen bits,
        // and a silent wrap would write schedule 65 536 over schedule 0 -- a data loss that
        // presents as a schedule the user never created.
        let key = u16::try_from(index).map_err(|_| "Schedule index does not fit a u16 flash key")?;

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
    async fn add_schedule(&mut self, item: ScheduleItem) -> Result<usize, &'static str> {
        // `?`, not `.ok().unwrap()`. A flash range that will not load is a reason to refuse the
        // write, not to panic -- and this is reached from a web request.
        self.load_from_flash().await?;

        // Hole-filling: reuse the first index a removal left free.
        let hole = (0..self.next_index).find(|i| !self.cache.contains_key(i));
        let index = hole.unwrap_or(self.next_index);

        // Flash first, cache second, `next_index` last -- the order `update_schedule` already
        // used. The previous order advanced `next_index` *before* the write, so a failed write
        // burned an index; that only looked harmless because the `expect` that followed had
        // already killed the machine.
        let opt = Some(item);
        self.store_in_flash(index, &opt).await?;
        self.cache.insert(index, opt.unwrap());
        if hole.is_none() {
            self.next_index += 1;
        }

        Ok(index)
    }

    async fn get_schedules(&mut self) -> impl Iterator<Item = &ScheduleItem> {
        let res = self.load_from_flash().await;
        if res.is_err() {
            log_info!("Error loading schedules from flash: {:?}", res.err());
        }

        self.cache.values()
    }

    async fn iterate_schedules_with_indices(&mut self) -> impl Iterator<Item = (usize, &ScheduleItem)> {
        let res = self.load_from_flash().await;
        if res.is_err() {
            log_info!("Error loading schedules from flash: {:?}", res.err());
        }

        self.cache.iter().map(|(index, item)| (*index, item))
    }

    async fn remove_schedule(&mut self, index: usize) -> Result<Option<ScheduleItem>, &'static str> {
        // This was the only method here that never loaded. On a cold store a remove read an
        // empty cache, answered "nothing there", and left the item in flash to reappear at the
        // next boot.
        self.load_from_flash().await?;

        if !self.cache.contains_key(&index) {
            return Ok(None);
        }

        // Flash first, cache second. The reverse -- which this did -- leaves RAM claiming a
        // schedule flash still holds, and the disagreement only surfaces after a reboot, by
        // which time nothing connects it to the deletion that failed.
        let opt: Option<ScheduleItem> = None;
        self.store_in_flash(index, &opt).await?;

        Ok(self.cache.remove(&index))
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
        //
        // **The cache is deliberately not cleared here.** It used to be emptied before the
        // erase below, so a `?` on that erase returned with RAM empty and flash intact: every
        // later read answered "no schedules" against a store that still had them, with no
        // error anywhere, because `cache_initialized` was still true.
        let schedules_to_store: Vec<ScheduleItem> = self.cache.values()
            .cloned()
            .collect();

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

        // Past this point flash has been erased and `schedules_to_store` is the only copy of
        // the machine's schedules, so **nothing below may return early**. The `?` that used to
        // be on `store_in_flash` dropped that local on the way out and lost every schedule the
        // machine had -- not a divergence, an erasure. Keep going, keep whatever lands, and
        // report at the end; a later `optimize_storage` or any `update_schedule` can then
        // write the rest.
        log_info!("Rewriting {} schedules with compacted indices", schedules_to_store.len());
        let mut rebuilt: BTreeMap<usize, ScheduleItem> = BTreeMap::new();
        let mut first_error: Option<&'static str> = None;
        for (new_index, schedule) in schedules_to_store.iter().enumerate() {
            let opt = Some(schedule.clone());
            match self.store_in_flash(new_index, &opt).await {
                Ok(()) => {
                    rebuilt.insert(new_index, schedule.clone());
                }
                Err(e) => {
                    log_warn!("Failed to rewrite schedule {} during optimization: {}", new_index, e);
                    let _ = first_error.get_or_insert(e);
                }
            }

            // Yield, so a long rewrite cannot starve the watchdog. The routine repository's
            // optimizer already does this and this one did not, which on a full store is the
            // difference between a compaction and a reset.
            Timer::after_millis(1).await;
        }

        // The cache is replaced only now, and only with what flash actually took, so RAM and
        // flash agree even on the partial-failure path. Holes are fine -- `add_schedule` fills
        // them, which is what its hole-filling strategy is for.
        self.cache = rebuilt;
        self.next_index = self.cache.keys().next_back().map_or(0, |k| k + 1);

        match first_error {
            Some(e) => Err(e),
            None => {
                log_info!(
                    "Schedule storage optimization complete, next_index reset to {}",
                    self.next_index
                );
                Ok(())
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;
    use chrono::{NaiveDate, Weekday};
    use heapless::index_set::FnvIndexSet;
    use variegated_controller_types::{MachineMode, ScheduleAction, ScheduleTrigger};
    use variegated_timekeeping::DateTimeInZone;

    /// Drive one of [`InMemoryScheduleStore`]'s futures to completion.
    ///
    /// Safe here and only here: none of that store's methods actually suspend -- they are
    /// `async` to satisfy the trait, and every body is straight-line over a `BTreeMap`.
    /// `block_on` busy-polls with a no-op waker and never re-enters the executor, so pointing
    /// it at anything that genuinely awaits deadlocks. `sd_card.rs` carries the scar tissue.
    ///
    /// The flash-backed [`SequentialStorageScheduleStore`] is deliberately **not** covered by
    /// any of this: it needs a `MultiwriteNorFlash`, which a host build has no implementation
    /// of. Its error paths are covered by review and by the on-target build only.
    fn run<T>(future: impl core::future::Future<Output = T>) -> T {
        embassy_futures::block_on(future)
    }

    /// 2026-08-20 is a Thursday, which the day-set test below depends on.
    fn at(hour: u32, minute: u32) -> DateTimeInZone {
        DateTimeInZone::Utc(
            NaiveDate::from_ymd_opt(2026, 8, 20)
                .unwrap()
                .and_hms_opt(hour, minute, 0)
                .unwrap()
                .and_utc(),
        )
    }

    fn trigger(hour: u8, minute: u8) -> ScheduleTrigger {
        ScheduleTrigger {
            on_hour: hour,
            on_minute: minute,
            on_days: None,
            on_date: None,
            enabled: true,
            once: false,
        }
    }

    fn item(trigger_at: ScheduleTrigger) -> ScheduleItem {
        ScheduleItem {
            trigger_at,
            commands: vec![ScheduleAction::SetMachineMode(MachineMode::On)],
        }
    }

    // ---- trigger_matches ------------------------------------------------------------------

    #[test]
    fn trigger_matches_on_hour_and_minute() {
        let t = trigger(7, 30);
        assert!(trigger_matches(&t, at(7, 30)));
        assert!(!trigger_matches(&t, at(7, 31)));
        assert!(!trigger_matches(&t, at(8, 30)));
    }

    #[test]
    fn trigger_matches_refuses_a_disabled_trigger() {
        let mut t = trigger(7, 30);
        t.enabled = false;
        assert!(!trigger_matches(&t, at(7, 30)));
    }

    #[test]
    fn trigger_matches_respects_on_days() {
        let mut days: FnvIndexSet<Weekday, 8> = FnvIndexSet::new();
        let _ = days.insert(Weekday::Thu);
        let mut t = trigger(7, 30);
        t.on_days = Some(days);
        assert!(trigger_matches(&t, at(7, 30)));

        let mut other: FnvIndexSet<Weekday, 8> = FnvIndexSet::new();
        let _ = other.insert(Weekday::Mon);
        t.on_days = Some(other);
        assert!(!trigger_matches(&t, at(7, 30)));
    }

    /// An *empty* day set can never fire, and must not be read as "every day". Collapsing the
    /// two would turn a schedule that never runs into a daily one.
    #[test]
    fn an_empty_day_set_never_matches() {
        let mut t = trigger(7, 30);
        t.on_days = Some(FnvIndexSet::new());
        assert!(!trigger_matches(&t, at(7, 30)));

        t.on_days = None;
        assert!(trigger_matches(&t, at(7, 30)), "None must still mean every day");
    }

    #[test]
    fn trigger_matches_respects_on_date() {
        let mut t = trigger(7, 30);
        t.on_date = NaiveDate::from_ymd_opt(2026, 8, 20);
        assert!(trigger_matches(&t, at(7, 30)));

        t.on_date = NaiveDate::from_ymd_opt(2026, 8, 21);
        assert!(!trigger_matches(&t, at(7, 30)));
    }

    // ---- the store ------------------------------------------------------------------------

    #[test]
    fn add_schedule_fills_holes_and_reports_the_index() {
        let mut store = InMemoryScheduleStore::new();
        assert_eq!(run(store.add_schedule(item(trigger(6, 0)))).unwrap(), 0);
        assert_eq!(run(store.add_schedule(item(trigger(7, 0)))).unwrap(), 1);
        assert_eq!(run(store.add_schedule(item(trigger(8, 0)))).unwrap(), 2);

        assert!(run(store.remove_schedule(1)).unwrap().is_some());

        // The hole, not 3. This is why the index cannot be derived from the count.
        assert_eq!(run(store.add_schedule(item(trigger(9, 0)))).unwrap(), 1);
    }

    #[test]
    fn remove_schedule_distinguishes_absent_from_present() {
        let mut store = InMemoryScheduleStore::new();
        run(store.add_schedule(item(trigger(6, 0)))).unwrap();

        assert!(run(store.remove_schedule(0)).unwrap().is_some());
        // Absent is `Ok(None)`, not an error. `Err` is reserved for a write that failed, which
        // this store cannot do -- that is the distinction the old `Option` return could not
        // make and the flash-backed store needed.
        assert!(run(store.remove_schedule(0)).unwrap().is_none());
        assert!(run(store.remove_schedule(99)).unwrap().is_none());
    }

    /// A firing schedule must report its *storage* index, not its position in the list.
    ///
    /// `[0, 2]` is the shape hole-filling produces after a removal. A caller using the position
    /// would name index 1 for the second entry and rewrite a schedule the user never touched --
    /// or, once the hole is filled, a different one entirely.
    #[test]
    fn triggering_indices_survive_a_removal() {
        let mut store = InMemoryScheduleStore::new();
        run(store.add_schedule(item(trigger(6, 0)))).unwrap();
        run(store.add_schedule(item(trigger(7, 0)))).unwrap();
        run(store.add_schedule(item(trigger(8, 0)))).unwrap();
        run(store.remove_schedule(1)).unwrap();

        let firing = run(store.schedules_triggering_at_with_indices(at(8, 0)));
        assert_eq!(firing.len(), 1);
        assert_eq!(firing[0].0, 2, "reported its position rather than its storage index");

        // And once the hole is filled, the new occupant is index 1 while sorting third by time.
        assert_eq!(run(store.add_schedule(item(trigger(9, 0)))).unwrap(), 1);
        let firing = run(store.schedules_triggering_at_with_indices(at(9, 0)));
        assert_eq!(firing[0].0, 1);
    }

    #[test]
    fn only_matching_schedules_are_reported() {
        let mut store = InMemoryScheduleStore::new();
        run(store.add_schedule(item(trigger(6, 0)))).unwrap();
        run(store.add_schedule(item(trigger(7, 0)))).unwrap();

        assert_eq!(run(store.schedules_triggering_at_with_indices(at(7, 0))).len(), 1);
        assert!(run(store.schedules_triggering_at_with_indices(at(5, 0))).is_empty());
    }

    /// The sequence [`run_schedule`] performs for a spent one-shot.
    ///
    /// The item must still be **there**, at the **same index**, and merely disabled -- so the
    /// user can re-enable or re-time it rather than re-create it, and so nothing renumbers
    /// under an open menu. Driving `run_schedule` itself would need timers and a channel; this
    /// covers the decision it makes.
    #[test]
    fn a_once_schedule_is_disabled_not_removed() {
        let mut store = InMemoryScheduleStore::new();
        let mut t = trigger(7, 30);
        t.once = true;
        run(store.add_schedule(item(t))).unwrap();

        let firing = run(store.schedules_triggering_at_with_indices(at(7, 30)));
        assert_eq!(firing.len(), 1);

        for (index, fired) in &firing {
            let mut updated = fired.clone();
            updated.trigger_at.enabled = false;
            run(store.update_schedule(*index, updated)).unwrap();
        }

        assert_eq!(run(store.get_schedule_count()), 1, "the item was removed, not disabled");

        let stored: Vec<(usize, ScheduleItem)> = run(store.iterate_schedules_with_indices())
            .map(|(index, item)| (index, item.clone()))
            .collect();
        assert_eq!(stored[0].0, 0, "the storage index moved");
        assert!(!stored[0].1.trigger_at.enabled);
        assert!(stored[0].1.trigger_at.once, "the once flag itself must survive");
        assert_eq!(stored[0].1.commands.len(), 1, "the actions must survive");

        // And it does not fire again.
        assert!(run(store.schedules_triggering_at_with_indices(at(7, 30))).is_empty());
    }

    #[test]
    fn optimize_storage_compacts_indices() {
        let mut store = InMemoryScheduleStore::new();
        run(store.add_schedule(item(trigger(6, 0)))).unwrap();
        run(store.add_schedule(item(trigger(7, 0)))).unwrap();
        run(store.add_schedule(item(trigger(8, 0)))).unwrap();
        run(store.remove_schedule(0)).unwrap();

        run(store.optimize_storage()).unwrap();

        let indices: Vec<usize> =
            run(store.iterate_schedules_with_indices()).map(|(index, _)| index).collect();
        assert_eq!(indices, vec![0, 1]);
        assert_eq!(run(store.get_schedule_count()), 2);
    }
}