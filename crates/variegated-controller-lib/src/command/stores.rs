//! The commands that edit the routine repository and the schedule store.
//!
//! Both controllers hold these two behind `&'static Mutex<M, _>` -- unlike their settings
//! stores, which differ -- so these functions can take the mutex directly and do the whole
//! job, lock and timeout included.
//!
//! Whether the machine *has* a schedule store is a separate question from what these do with
//! one. The single-boiler machine had none at all, which is why three of these commands were
//! silently dropped there while the web UI, the uplink and the comms processor's HTTP handler
//! all went on sending them to any machine.

use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{with_timeout, Duration};
use variegated_controller_types::{Routine, RoutineIndex, ScheduleItem};
use variegated_log::{log_info, log_warn};

use crate::routine::RoutineRepository;
use crate::schedule::ScheduleStore;

/// How long a command will wait for one of these stores.
///
/// **An operational timeout, not a failsafe one**, and deliberately short: the caller is the
/// control loop, and the thing most likely to be holding either lock is a flash erase that
/// takes far longer than this. Giving up and logging is the right answer there; blocking is
/// not, because this loop also runs the PID and the interlocks.
const LOCK_TIMEOUT: Duration = Duration::from_millis(100);

/// Whether the caller should republish `Configuration`.
///
/// `#[must_use]` because dropping it is invisible: the edit reaches flash either way, and
/// what goes missing is the *browser's* view of it, which then shows a stale schedule list
/// until something else happens to trigger a publish.
#[must_use = "a schedule edit that does not republish leaves every client showing the old list"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Publish(bool);

impl Publish {
    /// Something changed; `Configuration` should go out again.
    pub const YES: Self = Self(true);
    /// Nothing changed, so there is nothing new to say.
    pub const NO: Self = Self(false);

    /// Whether a republish is wanted.
    pub const fn wanted(self) -> bool {
        self.0
    }
}

/// `AddRoutine`.
pub async fn add_routine<M: RawMutex, R: RoutineRepository>(
    repository: &Mutex<M, R>,
    routine: Routine,
) {
    log_info!("Adding new routine");
    match with_timeout(LOCK_TIMEOUT, repository.lock()).await {
        Ok(mut repo) => match repo.add_routine(routine).await {
            Ok(index) => log_info!("Added routine at index {:?}", index),
            Err(e) => log_warn!("Failed to add routine: {}", e),
        },
        Err(_) => log_warn!("Failed to acquire routine_repository lock (timeout)"),
    }
}

/// `RemoveRoutine`.
///
/// The `Ok(None)` and `Err` arms are deliberately distinct, and used to be the same answer:
/// a client naming an index that is not there is a different problem from a flash write that
/// failed, and only one of them is the machine's fault.
pub async fn remove_routine<M: RawMutex, R: RoutineRepository>(
    repository: &Mutex<M, R>,
    index: RoutineIndex,
) {
    match with_timeout(LOCK_TIMEOUT, repository.lock()).await {
        Ok(mut repo) => match repo.remove_routine(index).await {
            Ok(Some(_)) => {}
            Ok(None) => log_warn!("No routine at index {:?} to remove", index),
            Err(e) => log_warn!("Failed to remove routine at index {:?}: {}", index, e),
        },
        Err(_) => log_warn!("Failed to acquire routine_repository lock (timeout)"),
    }
}

/// `UpdateRoutine`.
pub async fn update_routine<M: RawMutex, R: RoutineRepository>(
    repository: &Mutex<M, R>,
    index: RoutineIndex,
    routine: Routine,
) {
    match with_timeout(LOCK_TIMEOUT, repository.lock()).await {
        Ok(mut repo) => {
            if let Err(e) = repo.update_routine(index, routine).await {
                log_warn!("Failed to update routine at index {:?}: {}", index, e);
            }
        }
        Err(_) => log_warn!("Failed to acquire routine_repository lock (timeout)"),
    }
}

/// `AddScheduleItem`.
pub async fn add_schedule<M: RawMutex, S: ScheduleStore>(
    store: &Mutex<M, S>,
    item: ScheduleItem,
) -> Publish {
    log_info!("Adding new schedule item");
    match with_timeout(LOCK_TIMEOUT, store.lock()).await {
        Ok(mut store) => match store.add_schedule(item).await {
            Ok(index) => {
                log_info!("Added schedule at index {}", index);
                Publish::YES
            }
            Err(e) => {
                log_warn!("Failed to add schedule: {}", e);
                Publish::NO
            }
        },
        Err(_) => {
            log_warn!("Failed to acquire schedule_store lock (timeout)");
            Publish::NO
        }
    }
}

/// `RemoveScheduleItem`.
///
/// Same distinction as [`remove_routine`]: an index that holds nothing is a client error, an
/// `Err` is a flash write that failed, and neither is a reason to republish.
pub async fn remove_schedule<M: RawMutex, S: ScheduleStore>(
    store: &Mutex<M, S>,
    index: u32,
) -> Publish {
    log_info!("Removing schedule item at index {}", index);
    match with_timeout(LOCK_TIMEOUT, store.lock()).await {
        Ok(mut store) => match store.remove_schedule(index as usize).await {
            Ok(Some(_)) => Publish::YES,
            Ok(None) => {
                log_warn!("No schedule at index {} to remove", index);
                Publish::NO
            }
            Err(e) => {
                log_warn!("Failed to remove schedule at index {}: {}", index, e);
                Publish::NO
            }
        },
        Err(_) => {
            log_warn!("Failed to acquire schedule_store lock (timeout)");
            Publish::NO
        }
    }
}

/// `UpdateScheduleItem`.
pub async fn update_schedule<M: RawMutex, S: ScheduleStore>(
    store: &Mutex<M, S>,
    index: u32,
    item: ScheduleItem,
) -> Publish {
    log_info!("Updating schedule item at index {}", index);
    match with_timeout(LOCK_TIMEOUT, store.lock()).await {
        Ok(mut store) => match store.update_schedule(index as usize, item).await {
            Ok(()) => Publish::YES,
            Err(e) => {
                log_warn!("Failed to update schedule at index {}: {}", index, e);
                Publish::NO
            }
        },
        Err(_) => {
            log_warn!("Failed to acquire schedule_store lock (timeout)");
            Publish::NO
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;
    use embassy_sync::blocking_mutex::raw::NoopRawMutex;
    use variegated_controller_types::{MachineMode, ScheduleAction, ScheduleTrigger};

    use crate::routine::InMemoryRoutineRepository;
    use crate::schedule::InMemoryScheduleStore;

    /// Drive one of these futures to completion.
    ///
    /// Safe for the in-memory stores and only for those: none of their methods actually
    /// suspend, and neither does an uncontended `Mutex::lock`, so `with_timeout` returns on
    /// its first poll without ever reaching the timer. `block_on` busy-polls with a no-op
    /// waker, so anything that genuinely awaits would deadlock here -- see the same helper in
    /// `crate::schedule`, and the scar tissue in `sd_card.rs`.
    fn run<T>(future: impl core::future::Future<Output = T>) -> T {
        embassy_futures::block_on(future)
    }

    fn a_schedule() -> ScheduleItem {
        ScheduleItem {
            trigger_at: ScheduleTrigger {
                on_minute: 30,
                on_hour: 7,
                on_days: None,
                on_date: None,
                enabled: true,
                once: false,
            },
            commands: vec![ScheduleAction::SetMachineMode(MachineMode::On)],
        }
    }

    fn store() -> Mutex<NoopRawMutex, InMemoryScheduleStore> {
        Mutex::new(InMemoryScheduleStore::new())
    }

    /// How many schedules the store holds. Both the lock and the count are `async`.
    fn schedule_count(store: &Mutex<NoopRawMutex, InMemoryScheduleStore>) -> usize {
        run(async { store.lock().await.get_schedule_count().await })
    }

    /// Adding a schedule stores it and asks for a republish.
    ///
    /// The command the single-boiler machine dropped on the floor, which is what started all
    /// of this.
    #[test]
    fn adding_a_schedule_stores_it_and_republishes() {
        let store = store();

        let publish = run(add_schedule(&store, a_schedule()));

        assert_eq!(publish, Publish::YES);
        assert_eq!(schedule_count(&store), 1);
    }

    /// Removing a schedule that is there removes it, and asks for a republish.
    #[test]
    fn removing_a_present_schedule_republishes() {
        let store = store();
        let _ = run(add_schedule(&store, a_schedule()));

        let publish = run(remove_schedule(&store, 0));

        assert_eq!(publish, Publish::YES);
        assert_eq!(schedule_count(&store), 0);
    }

    /// Removing an index that holds nothing changes nothing and does not republish.
    ///
    /// The `Ok(None)` case. Republishing here would be harmless but wrong; treating it as an
    /// error would be worse, and the two were once the same answer.
    #[test]
    fn removing_an_absent_schedule_does_not_republish() {
        let store = store();
        let _ = run(add_schedule(&store, a_schedule()));

        let publish = run(remove_schedule(&store, 7));

        assert_eq!(publish, Publish::NO);
        assert_eq!(schedule_count(&store), 1, "the store was disturbed");
    }

    /// Updating a stored schedule replaces it and asks for a republish.
    #[test]
    fn updating_a_present_schedule_republishes() {
        let store = store();
        let _ = run(add_schedule(&store, a_schedule()));

        let mut edited = a_schedule();
        edited.trigger_at.on_hour = 9;
        let publish = run(update_schedule(&store, 0, edited));

        assert_eq!(publish, Publish::YES);
        let hour = run(async {
            store.lock().await.get_schedules().await.next().unwrap().trigger_at.on_hour
        });
        assert_eq!(hour, 9);
    }

    /// Updating an index that holds nothing does not republish.
    #[test]
    fn updating_an_absent_schedule_does_not_republish() {
        let store = store();

        let publish = run(update_schedule(&store, 4, a_schedule()));

        assert_eq!(publish, Publish::NO);
    }

    /// Removing a routine that is not there leaves the repository alone.
    ///
    /// The routine half of the same `Ok(None)` distinction.
    #[test]
    fn removing_an_absent_routine_is_survivable() {
        let repository: Mutex<NoopRawMutex, _> = Mutex::new(InMemoryRoutineRepository::new());

        run(remove_routine(&repository, RoutineIndex::Custom(3)));

        let count = run(async { repository.lock().await.get_routine_count().await });
        assert_eq!(count, 0);
    }
}
