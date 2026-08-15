//! # Variegated Timekeeping
//!
//! Date and time handling for embedded espresso machine controllers.
//!
//! This crate provides a `TimeKeeper` singleton that maintains a mapping between
//! `embassy_time::Instant` (monotonic system time) and real-world date/time
//! with flexible timezone support.
//!
//! ## Features
//!
//! - `no_std` compatible with `alloc`
//! - Multiple timezone types supported:
//!   - Fixed UTC offsets via `chrono::FixedOffset` (e.g., `+05:00`)
//!   - UTC timezone
//!   - Named timezones with DST support via `chrono-tz` (requires `named-timezones` feature)
//! - Thread-safe singleton pattern
//! - Static API similar to `embassy_time::Instant::now()`
//!
//! ## Cargo Features
//!
//! - `named-timezones` - Enable support for named timezones (e.g., `America/New_York`) with DST support via `chrono-tz`.
//!   Without this feature, only `FixedOffset` and `Utc` are available, reducing dependency size.
//! - `defmt` - Enable defmt logging support
//! - `serde` - Enable serde support for chrono types
//!
//! ## Usage
//!
//! ### Basic usage with FixedOffset (no extra features required)
//!
//! ```rust,ignore
//! use variegated_timekeeping::TimeKeeper;
//! use chrono::FixedOffset;
//!
//! #[embassy_executor::main]
//! async fn main(_spawner: Spawner) {
//!     // Initialize with fixed offset (e.g., +5 hours)
//!     TimeKeeper::init(FixedOffset::east_opt(5 * 3600).unwrap());
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
//!
//! ### With named timezones (requires `named-timezones` feature)
//!
//! ```rust,ignore
//! use variegated_timekeeping::TimeKeeper;
//! use chrono_tz::America::Los_Angeles;
//!
//! #[embassy_executor::main]
//! async fn main(_spawner: Spawner) {
//!     // Initialize with named timezone (with DST support)
//!     TimeKeeper::init(Los_Angeles);
//!
//!     // ... rest is the same
//! }
//! ```

#![no_std]
#![warn(missing_docs)]

extern crate alloc;

use core::cell::RefCell;
use chrono::{DateTime, Datelike, FixedOffset, NaiveDateTime, TimeZone, Timelike, Utc, Weekday};
#[cfg(feature = "named-timezones")]
use chrono_tz::Tz;
use embassy_sync::blocking_mutex::{raw::CriticalSectionRawMutex, Mutex};
use embassy_sync::signal::Signal;
use embassy_time::Instant;

pub mod error;

pub use error::{Error, Result};

/// Wrapper enum that supports multiple timezone types from chrono.
///
/// This allows TimeKeeper to work with different timezone implementations:
/// - Named timezones with DST support (via chrono-tz, requires `named-timezones` feature)
/// - Fixed UTC offsets without DST
/// - UTC timezone
#[derive(Debug, Clone, Copy)]
pub enum TimeZoneWrapper {
    /// A named timezone with DST support (e.g., America/New_York, Europe/London)
    ///
    /// Only available with the `named-timezones` feature enabled.
    #[cfg(feature = "named-timezones")]
    Named(Tz),
    /// A fixed UTC offset (e.g., +05:00, -08:00)
    Fixed(FixedOffset),
    /// UTC timezone
    Utc,
}

/// DateTime in a specific timezone, supporting multiple timezone types.
///
/// This enum wraps different DateTime types to allow working with various
/// timezone implementations while maintaining type safety.
#[derive(Debug, Clone, Copy)]
pub enum DateTimeInZone {
    /// DateTime in a named timezone (with DST support)
    ///
    /// Only available with the `named-timezones` feature enabled.
    #[cfg(feature = "named-timezones")]
    Named(DateTime<Tz>),
    /// DateTime with a fixed UTC offset
    Fixed(DateTime<FixedOffset>),
    /// DateTime in UTC
    Utc(DateTime<Utc>),
}

impl DateTimeInZone {
    /// Get the underlying datetime as UTC
    pub fn to_utc(&self) -> DateTime<Utc> {
        match self {
            #[cfg(feature = "named-timezones")]
            DateTimeInZone::Named(dt) => dt.with_timezone(&Utc),
            DateTimeInZone::Fixed(dt) => dt.with_timezone(&Utc),
            DateTimeInZone::Utc(dt) => *dt,
        }
    }

    /// Get the naive local datetime (without timezone info)
    pub fn naive_local(&self) -> NaiveDateTime {
        match self {
            #[cfg(feature = "named-timezones")]
            DateTimeInZone::Named(dt) => dt.naive_local(),
            DateTimeInZone::Fixed(dt) => dt.naive_local(),
            DateTimeInZone::Utc(dt) => dt.naive_utc(),
        }
    }

    /// Get the naive date
    pub fn date_naive(&self) -> chrono::NaiveDate {
        match self {
            #[cfg(feature = "named-timezones")]
            DateTimeInZone::Named(dt) => dt.date_naive(),
            DateTimeInZone::Fixed(dt) => dt.date_naive(),
            DateTimeInZone::Utc(dt) => dt.date_naive(),
        }
    }

    /// Get the weekday
    pub fn weekday(&self) -> Weekday {
        match self {
            #[cfg(feature = "named-timezones")]
            DateTimeInZone::Named(dt) => dt.weekday(),
            DateTimeInZone::Fixed(dt) => dt.weekday(),
            DateTimeInZone::Utc(dt) => dt.weekday(),
        }
    }

    /// Get the hour component (0-23)
    pub fn hour(&self) -> u32 {
        match self {
            #[cfg(feature = "named-timezones")]
            DateTimeInZone::Named(dt) => dt.hour(),
            DateTimeInZone::Fixed(dt) => dt.hour(),
            DateTimeInZone::Utc(dt) => dt.hour(),
        }
    }

    /// Get the minute component (0-59)
    pub fn minute(&self) -> u32 {
        match self {
            #[cfg(feature = "named-timezones")]
            DateTimeInZone::Named(dt) => dt.minute(),
            DateTimeInZone::Fixed(dt) => dt.minute(),
            DateTimeInZone::Utc(dt) => dt.minute(),
        }
    }

    /// Get the second component (0-59)
    pub fn second(&self) -> u32 {
        match self {
            #[cfg(feature = "named-timezones")]
            DateTimeInZone::Named(dt) => dt.second(),
            DateTimeInZone::Fixed(dt) => dt.second(),
            DateTimeInZone::Utc(dt) => dt.second(),
        }
    }

    /// Calculate signed duration since another DateTimeInZone
    pub fn signed_duration_since(&self, other: &DateTimeInZone) -> chrono::Duration {
        self.to_utc().signed_duration_since(other.to_utc())
    }
}

impl PartialEq for DateTimeInZone {
    fn eq(&self, other: &Self) -> bool {
        self.to_utc() == other.to_utc()
    }
}

impl Eq for DateTimeInZone {}

impl PartialOrd for DateTimeInZone {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for DateTimeInZone {
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        self.to_utc().cmp(&other.to_utc())
    }
}

impl core::ops::Add<chrono::Duration> for DateTimeInZone {
    type Output = DateTimeInZone;

    fn add(self, rhs: chrono::Duration) -> Self::Output {
        match self {
            #[cfg(feature = "named-timezones")]
            DateTimeInZone::Named(dt) => DateTimeInZone::Named(dt + rhs),
            DateTimeInZone::Fixed(dt) => DateTimeInZone::Fixed(dt + rhs),
            DateTimeInZone::Utc(dt) => DateTimeInZone::Utc(dt + rhs),
        }
    }
}

impl TimeZoneWrapper {
    /// Convert a UTC datetime to local time in this timezone
    pub fn to_local(&self, utc: DateTime<Utc>) -> DateTimeInZone {
        match self {
            #[cfg(feature = "named-timezones")]
            TimeZoneWrapper::Named(tz) => DateTimeInZone::Named(utc.with_timezone(tz)),
            TimeZoneWrapper::Fixed(offset) => DateTimeInZone::Fixed(utc.with_timezone(offset)),
            TimeZoneWrapper::Utc => DateTimeInZone::Utc(utc),
        }
    }

    /// Convert a local datetime to this timezone, returning the earliest match
    /// in case of DST ambiguity
    pub fn from_local(&self, local: &NaiveDateTime) -> Option<DateTimeInZone> {
        match self {
            #[cfg(feature = "named-timezones")]
            TimeZoneWrapper::Named(tz) => {
                tz.from_local_datetime(local).earliest().map(DateTimeInZone::Named)
            }
            TimeZoneWrapper::Fixed(offset) => {
                offset.from_local_datetime(local).earliest().map(DateTimeInZone::Fixed)
            }
            TimeZoneWrapper::Utc => {
                Utc.from_local_datetime(local).earliest().map(DateTimeInZone::Utc)
            }
        }
    }
}

#[cfg(feature = "named-timezones")]
impl From<Tz> for TimeZoneWrapper {
    fn from(tz: Tz) -> Self {
        TimeZoneWrapper::Named(tz)
    }
}

impl From<FixedOffset> for TimeZoneWrapper {
    fn from(offset: FixedOffset) -> Self {
        TimeZoneWrapper::Fixed(offset)
    }
}

impl From<Utc> for TimeZoneWrapper {
    fn from(_: Utc) -> Self {
        TimeZoneWrapper::Utc
    }
}

/// Internal state of the TimeKeeper
struct TimeKeeperState {
    /// Anchor point: embassy_time::Instant when the time was set
    anchor_instant: Option<Instant>,
    /// Anchor point: UTC DateTime when the time was set
    anchor_datetime: Option<DateTime<Utc>>,
    /// Current timezone
    timezone: TimeZoneWrapper,
}

/// Global storage for TimeKeeper state
static STATE: Mutex<CriticalSectionRawMutex, RefCell<Option<TimeKeeperState>>> =
    Mutex::new(RefCell::new(None));

/// Raised by [`TimeKeeper::set_time_authoritative`], awaited by
/// [`TimeKeeper::wait_for_authoritative_set`].
///
/// This exists so that a hardware clock can be corrected the moment the network supplies a
/// better time, rather than on a poll. It is a `Signal` and therefore has room for exactly
/// one waiter, which is the right shape here: the thing that owns the battery-backed RTC is
/// singular by construction.
static AUTHORITATIVE_SET: Signal<CriticalSectionRawMutex, ()> = Signal::new();

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
    /// Accepts any timezone type that can be converted to `TimeZoneWrapper`:
    /// - `chrono::FixedOffset` for fixed UTC offsets
    /// - `chrono::Utc` for UTC timezone
    /// - `chrono_tz::Tz` for named timezones with DST support (requires `named-timezones` feature)
    ///
    /// # Example
    ///
    /// ```ignore
    /// use variegated_timekeeping::TimeKeeper;
    /// use chrono::FixedOffset;
    ///
    /// // With fixed offset
    /// TimeKeeper::init(FixedOffset::east_opt(5 * 3600).unwrap());
    /// ```
    ///
    /// With the `named-timezones` feature enabled:
    ///
    /// ```ignore
    /// use chrono_tz::America::Los_Angeles;
    ///
    /// // With named timezone
    /// TimeKeeper::init(Los_Angeles);
    /// ```
    pub fn init(timezone: impl Into<TimeZoneWrapper>) {
        STATE.lock(|cell| {
            let mut opt = cell.borrow_mut();
            if opt.is_some() {
                panic!("TimeKeeper already initialized");
            }
            *opt = Some(TimeKeeperState {
                anchor_instant: None,
                anchor_datetime: None,
                timezone: timezone.into(),
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
    /// Silent, and that is the distinction from
    /// [`set_time_authoritative`](Self::set_time_authoritative). Use this for a re-anchor
    /// against a clock that is already the reference -- reading a battery-backed RTC, say.
    /// Announcing those would make the announcement useless: whatever listens in order to
    /// *write* such a clock would be woken by its own reads.
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

    /// Set the current time from a source that outranks whatever is holding it now, and say
    /// so.
    ///
    /// Identical to [`set_time`](Self::set_time) except that it raises the signal
    /// [`wait_for_authoritative_set`](Self::wait_for_authoritative_set) awaits. Reserved for
    /// a correction that arrives from outside the machine -- in practice SNTP -- so that a
    /// local hardware clock can be rewritten immediately rather than at the next poll.
    ///
    /// The signal is raised even if the time did not change by much. Whether a correction is
    /// worth acting on is the listener's judgement, not this function's, and a listener that
    /// wants a threshold can compare before and after.
    pub fn set_time_authoritative(datetime: DateTime<Utc>) -> Result<()> {
        Self::set_time(datetime)?;
        AUTHORITATIVE_SET.signal(());
        Ok(())
    }

    /// Wait until someone calls [`set_time_authoritative`](Self::set_time_authoritative).
    ///
    /// One waiter only -- see [`AUTHORITATIVE_SET`]. A signal raised while nobody is waiting
    /// is remembered, so a listener that starts late still sees the most recent correction
    /// rather than missing it.
    pub async fn wait_for_authoritative_set() {
        AUTHORITATIVE_SET.wait().await
    }

    /// Set the timezone.
    ///
    /// Accepts any timezone type that can be converted to `TimeZoneWrapper`:
    /// - `chrono::FixedOffset` for fixed UTC offsets
    /// - `chrono::Utc` for UTC timezone
    /// - `chrono_tz::Tz` for named timezones with DST support (requires `named-timezones` feature)
    ///
    /// # Example
    ///
    /// ```ignore
    /// use variegated_timekeeping::TimeKeeper;
    /// use chrono::FixedOffset;
    ///
    /// // With fixed offset
    /// TimeKeeper::set_timezone(FixedOffset::east_opt(-5 * 3600).unwrap()).unwrap();
    /// ```
    ///
    /// With the `named-timezones` feature enabled:
    ///
    /// ```ignore
    /// use chrono_tz::America::New_York;
    ///
    /// // With named timezone
    /// TimeKeeper::set_timezone(New_York).unwrap();
    /// ```
    pub fn set_timezone(timezone: impl Into<TimeZoneWrapper>) -> Result<()> {
        STATE.lock(|cell| {
            let mut opt = cell.borrow_mut();
            let s = opt.as_mut().ok_or(Error::Uninitialized)?;
            s.timezone = timezone.into();
            Ok(())
        })
    }

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
    ///     println!("Local time: {:?}", local);
    /// }
    /// ```
    pub fn now_local() -> Option<DateTimeInZone> {
        STATE.lock(|cell| {
            let opt = cell.borrow();
            let s = opt.as_ref()?;
            let anchor_instant = s.anchor_instant?;
            let anchor_datetime = s.anchor_datetime?;
            let timezone = s.timezone;

            let elapsed = Instant::now().duration_since(anchor_instant);
            let duration = chrono::Duration::microseconds(elapsed.as_micros() as i64);

            let utc_time = anchor_datetime.checked_add_signed(duration)?;
            Some(timezone.to_local(utc_time))
        })
    }

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

    /// Create a timer that will fire when the given UTC datetime is reached
    ///
    /// Returns `None` if the datetime is in the past or if microsecond conversion overflows
    pub fn timer_until(datetime: DateTime<Utc>) -> Option<embassy_time::Timer> {
        let now = Self::now_utc()?;
        if datetime <= now {
            return None;
        }
        let duration = datetime.signed_duration_since(now);
        let micros = duration.num_microseconds()? as u64;
        Some(embassy_time::Timer::after(embassy_time::Duration::from_micros(micros)))
    }

    /// Create a timer that will fire when the given local datetime is reached
    ///
    /// Returns `None` if the datetime is in the past or if microsecond conversion overflows
    pub fn timer_until_local(datetime: DateTimeInZone) -> Option<embassy_time::Timer> {
        let now = Self::now_local()?;
        if datetime <= now {
            return None;
        }
        let duration = datetime.signed_duration_since(&now);
        let micros = duration.num_microseconds()? as u64;
        Some(embassy_time::Timer::after(embassy_time::Duration::from_micros(micros)))
    }

    /// Create a timer that will fire at the next occurrence of a specific time
    ///
    /// Returns `None` if the TimeKeeper is not initialized
    pub fn timer_until_next_local(second: u8, minute: u8, hour: u8, day_of_week: Option<Weekday>) -> Option<embassy_time::Timer> {
        let now = Self::now_local()?;
        let timezone = Self::timezone();

        let next = now.date_naive().and_hms_opt(hour as u32, minute as u32, second as u32)?;
        let mut next = timezone.from_local(&next)?;

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
    pub fn timezone() -> TimeZoneWrapper {
        STATE.lock(|cell| {
            let opt = cell.borrow();
            opt.as_ref().map(|s| s.timezone).unwrap_or(TimeZoneWrapper::Utc)
        })
    }

    /// Check if the TimeKeeper has been initialized with a time.
    pub fn is_initialized() -> bool {
        STATE.lock(|cell| {
            let opt = cell.borrow();
            opt.as_ref()
                .map_or(false, |s| s.anchor_instant.is_some() && s.anchor_datetime.is_some())
        })
    }
}