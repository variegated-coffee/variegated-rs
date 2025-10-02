//! # Variegated Timekeeping
//!
//! Date and time handling for embedded espresso machine controllers.
//!
//! This crate provides a `TimeKeeper` singleton that maintains a mapping between
//! `embassy_time::Instant` (monotonic system time) and real-world date/time
//! with timezone support.
//!
//! ## Features
//!
//! - `no_std` compatible with `alloc`
//! - Timezone and DST support via `chrono-tz`
//! - Thread-safe singleton pattern
//! - Static API similar to `embassy_time::Instant::now()`
//!
//! ## Usage
//!
//! ```rust,ignore
//! use variegated_timekeeping::TimeKeeper;
//! use chrono_tz::America::Los_Angeles;
//!
//! #[embassy_executor::main]
//! async fn main(_spawner: Spawner) {
//!     // Initialize with timezone
//!     TimeKeeper::init(Los_Angeles);
//!
//!     // Set the current time
//!     let datetime = NaiveDate::from_ymd_opt(2025, 9, 30)
//!         .unwrap()
//!         .and_hms_opt(14, 30, 0)
//!         .unwrap()
//!         .and_utc();
//!     TimeKeeper::set_time(datetime).unwrap();
//!
//!     // Use from anywhere in your code
//!     let now_utc = TimeKeeper::now_utc().unwrap();
//!     let now_local = TimeKeeper::now_local().unwrap();
//! }
//! ```

#![no_std]
#![warn(missing_docs)]

extern crate alloc;

use core::cell::RefCell;
use chrono::{DateTime, Datelike, TimeZone, Utc, Weekday};
//use chrono_tz::Tz;
use embassy_sync::blocking_mutex::{raw::CriticalSectionRawMutex, Mutex};
use embassy_time::Instant;

pub mod error;

pub use error::{Error, Result};

/// Internal state of the TimeKeeper
struct TimeKeeperState {
    /// Anchor point: embassy_time::Instant when the time was set
    anchor_instant: Option<Instant>,
    /// Anchor point: UTC DateTime when the time was set
    anchor_datetime: Option<DateTime<Utc>>,
    // Current timezone
//    timezone: Tz,
}

/// Global storage for TimeKeeper state
static STATE: Mutex<CriticalSectionRawMutex, RefCell<Option<TimeKeeperState>>> =
    Mutex::new(RefCell::new(None));

/// TimeKeeper singleton for managing system time.
///
/// This zero-sized type provides static methods for accessing and manipulating
/// the system time. It must be initialized once using `TimeKeeper::init()` before use.
pub struct TimeKeeper;

impl TimeKeeper {
    /// Initialize the TimeKeeper with a timezone.
    ///
    /// This must be called exactly once before any other TimeKeeper methods.
    /// Calling this multiple times will panic.
    ///
    /// # Example
    ///
    /// ```ignore
    /// use variegated_timekeeping::{TimeKeeper, chrono_tz::America::Los_Angeles};
    ///
    /// TimeKeeper::init(Los_Angeles);
    /// ```
    pub fn init() {
        STATE.lock(|cell| {
            let mut opt = cell.borrow_mut();
            if opt.is_some() {
                panic!("TimeKeeper already initialized");
            }
            *opt = Some(TimeKeeperState {
                anchor_instant: None,
                anchor_datetime: None,
//                timezone,
            });
        });
    }

    /// Set the current time.
    ///
    /// This establishes the anchor point for all future time calculations.
    ///
    /// # Example
    ///
    /// ```ignore
    /// use variegated_timekeeping::{TimeKeeper, chrono::Utc};
    ///
    /// let now = Utc::now();
    /// TimeKeeper::set_time(now).unwrap();
    /// ```
    pub fn set_time(datetime: DateTime<Utc>) -> Result<()> {
        let now = Instant::now();

        STATE.lock(|cell| {
            let mut opt = cell.borrow_mut();
            let s = opt.as_mut().ok_or(Error::Uninitialized)?;
            s.anchor_instant = Some(now);
            s.anchor_datetime = Some(datetime);
            Ok(())
        })
    }
/*
    /// Set the timezone.
    ///
    /// # Example
    ///
    /// ```ignore
    /// use variegated_timekeeping::{TimeKeeper, chrono_tz::America::New_York};
    ///
    /// TimeKeeper::set_timezone(New_York).unwrap();
    /// ```
    pub fn set_timezone(timezone: Tz) -> Result<()> {
        STATE.lock(|cell| {
            let mut opt = cell.borrow_mut();
            let s = opt.as_mut().ok_or(Error::Uninitialized)?;
            s.timezone = timezone;
            Ok(())
        })
    }
*/
    /// Get the current UTC time.
    ///
    /// Returns `None` if the TimeKeeper has not been initialized with `set_time()`.
    ///
    /// # Example
    ///
    /// ```ignore
    /// use variegated_timekeeping::TimeKeeper;
    ///
    /// if let Some(now) = TimeKeeper::now_utc() {
    ///     println!("Current time: {}", now);
    /// }
    /// ```
    pub fn now_utc() -> Option<DateTime<Utc>> {
        STATE.lock(|cell| {
            let opt = cell.borrow();
            let s = opt.as_ref()?;
            let anchor_instant = s.anchor_instant?;
            let anchor_datetime = s.anchor_datetime?;

            let elapsed = Instant::now().duration_since(anchor_instant);
            let duration = chrono::Duration::microseconds(elapsed.as_micros() as i64);

            anchor_datetime.checked_add_signed(duration)
        })
    }
/*
    /// Get the current local time in the configured timezone.
    ///
    /// Returns `None` if the TimeKeeper has not been initialized with `set_time()`.
    ///
    /// # Example
    ///
    /// ```ignore
    /// use variegated_timekeeping::TimeKeeper;
    ///
    /// if let Some(local) = TimeKeeper::now_local() {
    ///     println!("Local time: {}", local);
    /// }
    /// ```
    pub fn now_local() -> Option<DateTime<Tz>> {
        STATE.lock(|cell| {
            let opt = cell.borrow();
            let s = opt.as_ref()?;
            let anchor_instant = s.anchor_instant?;
            let anchor_datetime = s.anchor_datetime?;
            let timezone = s.timezone;

            let elapsed = Instant::now().duration_since(anchor_instant);
            let duration = chrono::Duration::microseconds(elapsed.as_micros() as i64);

            let utc_time = anchor_datetime.checked_add_signed(duration)?;
            Some(utc_time.with_timezone(&timezone))
        })
    }
*/
    /// Convert an embassy_time::Instant to a DateTime<Utc>.
    ///
    /// Returns `None` if the TimeKeeper has not been initialized or if the
    /// instant is before the anchor point.
    pub fn instant_to_datetime(instant: Instant) -> Option<DateTime<Utc>> {
        STATE.lock(|cell| {
            let opt = cell.borrow();
            let s = opt.as_ref()?;
            let anchor_instant = s.anchor_instant?;
            let anchor_datetime = s.anchor_datetime?;

            if instant < anchor_instant {
                return None;
            }

            let elapsed = instant.duration_since(anchor_instant);
            let duration = chrono::Duration::microseconds(elapsed.as_micros() as i64);

            anchor_datetime.checked_add_signed(duration)
        })
    }

    /// Convert a DateTime<Utc> to an embassy_time::Instant.
    ///
    /// Returns `None` if the TimeKeeper has not been initialized or if the
    /// datetime is before the anchor point.
    pub fn datetime_to_instant(datetime: DateTime<Utc>) -> Option<Instant> {
        STATE.lock(|cell| {
            let opt = cell.borrow();
            let s = opt.as_ref()?;
            let anchor_instant = s.anchor_instant?;
            let anchor_datetime = s.anchor_datetime?;

            if datetime < anchor_datetime {
                return None;
            }

            let duration = datetime.signed_duration_since(anchor_datetime);
            let micros = duration.num_microseconds()?;

            Some(anchor_instant + embassy_time::Duration::from_micros(micros as u64))
        })
    }

    pub fn timer_until(datetime: DateTime<Utc>) -> Option<embassy_time::Timer> {
        let now = Self::now_utc()?;
        if datetime <= now {
            return None;
        }
        let duration = datetime.signed_duration_since(now);
        let micros = duration.num_microseconds()? as u64;
        Some(embassy_time::Timer::after(embassy_time::Duration::from_micros(micros)))
    }
/*
    pub fn timer_until_local(datetime: DateTime<Tz>) -> Option<embassy_time::Timer> {
        let now = Self::now_local()?;
        if datetime <= now {
            return None;
        }
        let duration = datetime.signed_duration_since(now);
        let micros = duration.num_microseconds()? as u64;
        Some(embassy_time::Timer::after(embassy_time::Duration::from_micros(micros)))
    }

    pub fn timer_until_next_local(second: u8, minute: u8, hour: u8, day_of_week: Option<Weekday>) -> Option<embassy_time::Timer> {
        let now = Self::now_local()?;
        let timezone = Self::timezone();

        let next = now.date_naive().and_hms_opt(hour as u32, minute as u32, second as u32)?;
        let mut next = timezone.from_local_datetime(&next).earliest()?;

        if let Some(dow) = day_of_week {
            while next.weekday() != dow {
                next = next + chrono::Duration::days(1);
            }
        }

        if next <= now {
            next = next + chrono::Duration::days(1);
            if let Some(dow) = day_of_week {
                while next.weekday() != dow {
                    next = next + chrono::Duration::days(1);
                }
            }
        }

        Self::timer_until_local(next)
    }

    /// Get the configured timezone.
    pub fn timezone() -> Tz {
        STATE.lock(|cell| {
            let opt = cell.borrow();
            opt.as_ref().map(|s| s.timezone).unwrap_or(Tz::UTC)
        })
    }
*/
    /// Check if the TimeKeeper has been initialized with a time.
    pub fn is_initialized() -> bool {
        STATE.lock(|cell| {
            let opt = cell.borrow();
            opt.as_ref()
                .map_or(false, |s| s.anchor_instant.is_some() && s.anchor_datetime.is_some())
        })
    }
}