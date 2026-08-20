//! Schedules as menu rows, and editing the one field a four-button panel can edit.
//!
//! The machine already has schedules -- stored in flash, evaluated once a minute, edited in
//! the browser. What it did not have was any way to reach them from the panel, and
//! `MENU-STRUCTURE.md` §5 says why: a recurrence is seven independent switches and a date is
//! a calendar, neither of which four buttons can express.
//!
//! The slice that *can* be expressed is a schedule's **time** and its **enabled** flag, and
//! that is what this module models. Everything else about a schedule is read-only here.
//!
//! # Why the arithmetic is in this crate and not in the renderer
//!
//! Two of the functions below fail *invisibly* when they are wrong, which is precisely the
//! kind of thing neither firmware can test:
//!
//! - [`apply_schedule_change`] rebuilds a whole `ScheduleItem`, because
//!   `MachineCommand::UpdateScheduleItem` replaces one wholesale. A field it forgets to copy
//!   across is silently deleted -- toggling a schedule off and on again would drop its day
//!   set and its actions, and nothing would report it.
//! - [`schedule_rows`] carries the **storage index**, which is sparse. Getting it wrong
//!   rewrites a different schedule than the one on screen.
//!
//! Neither is checkable by looking at the panel, for the reason `rows` gives about
//! truncation: the failure *is* the absence.

use chrono::{Datelike, NaiveDate, Weekday};
use core::fmt::Write as _;
use heapless::{String, Vec};
use variegated_controller_types::{MachineMode, ScheduleAction, ScheduleItem, ScheduleTrigger};

use crate::rows::COLUMNS;

/// How many schedules the panel will list.
///
/// Matches `MAX_SCHEDULES` in the browser's `ScheduleBuilder.tsx`, which is the only ceiling
/// stated anywhere in the system -- the flash store itself is uncapped. Matching it means the
/// panel never silently omits a schedule the web UI was willing to create.
///
/// A list that somehow exceeds it is truncated rather than grown, for [`crate::routine_rows`]'
/// reason: a menu is not worth taking the machine down for.
pub const MAX_MENU_SCHEDULES: usize = 64;

/// How wide a schedule's list label can be.
///
/// Twelve, because that is the character LCD's label field in the `{:<12.12}{:>4.4}` split,
/// and `pad_or_truncate_to_16` cuts from the right -- so an over-long label there does not
/// truncate itself, it eats the `ON`/`OFF` beside it.
pub const SCHEDULE_LABEL_LEN: usize = 12;

/// How wide a recurrence description can be.
///
/// Matches the GS3's `INFO_TEXT_LEN`, which is the buffer its read-only value path returns.
/// Sized for the widest thing [`schedule_recurrence`] can produce -- see
/// `the_widest_recurrence_fits_its_buffer`, which exists because `heapless::String::push_str`
/// rejects the *whole* write when it does not fit, making an overflow here a **blank**
/// recurrence rather than a truncated one.
pub const SCHEDULE_RECURRENCE_LEN: usize = 32;

/// The seven weekdays, in the order a person reads them.
///
/// **Iterated instead of the set itself.** `ScheduleTrigger::on_days` is an
/// `Option<FnvIndexSet<Weekday, 8>>`, and a hash set has no defined iteration order -- so
/// formatting one by iterating it would put Thursday before Monday depending on nothing the
/// user can see. Every read of a day set in this module goes through this array.
const WEEK: [Weekday; 7] = [
    Weekday::Mon,
    Weekday::Tue,
    Weekday::Wed,
    Weekday::Thu,
    Weekday::Fri,
    Weekday::Sat,
    Weekday::Sun,
];

/// Three-letter names for [`WEEK`], in the same order.
const WEEK_NAMES: [&str; 7] = ["Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"];

/// Monday through Friday, as a [`DaySet`] mask.
const WEEKDAYS: u8 = 0b0001_1111;

/// Saturday and Sunday, as a [`DaySet`] mask.
const WEEKENDS: u8 = 0b0110_0000;

/// Every day, as a [`DaySet`] mask.
const EVERY_DAY: u8 = 0b0111_1111;

/// Which weekdays a schedule fires on, as a bitmask over [`WEEK`].
///
/// A mask rather than the `FnvIndexSet` the trigger stores, for two reasons. It is `Copy`, so
/// a row carrying one can ride a `Watch` payload; and it has an order, which the set does not.
///
/// **`None` and the empty set are different things and must stay that way.** A trigger whose
/// `on_days` is `None` fires *every* day; one whose set is empty fires on *no* day. Collapsing
/// them would turn a schedule that never runs into one that runs daily.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DaySet {
    /// Bit *n* is `WEEK[n]`. Only meaningful when `every_day` is false.
    mask: u8,
    /// Whether the trigger said `None`, meaning every day regardless of `mask`.
    every_day: bool,
}

impl DaySet {
    /// Read a trigger's day set, testing the seven weekdays in order.
    pub fn from_trigger(trigger: &ScheduleTrigger) -> Self {
        match &trigger.on_days {
            None => Self { mask: 0, every_day: true },
            Some(days) => {
                let mut mask = 0u8;
                for (bit, day) in WEEK.iter().enumerate() {
                    if days.contains(day) {
                        mask |= 1 << bit;
                    }
                }
                Self { mask, every_day: false }
            }
        }
    }

    /// Whether this fires on every day of the week.
    pub const fn is_every_day(&self) -> bool {
        self.every_day || self.mask == EVERY_DAY
    }

    /// Whether this fires on no day at all -- a storable state that can never trigger.
    pub const fn is_never(&self) -> bool {
        !self.every_day && self.mask == 0
    }
}

/// What a schedule's first action does, as a drawable category.
///
/// The *discriminant* rather than the action, so a row stays `Copy` and free of a
/// `heapless::String`. A four-button panel can neither list actions nor edit them; this
/// exists so a user can tell one schedule from another in the list.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ScheduleActionKind {
    /// No actions at all. Storable, and does nothing when it fires.
    #[default]
    Empty,
    /// `SetMachineMode(On)`.
    On,
    /// `SetMachineMode(Off)`.
    Off,
    /// `SetMachineMode(PowerSaveStandby)`.
    Sleep,
    /// `RunRoutine`.
    Run,
    /// `CancelRoutine`.
    Cancel,
    /// Either of the boiler-target actions.
    Boiler,
}

impl ScheduleActionKind {
    /// Categorise a schedule's first action.
    pub fn of(commands: &[ScheduleAction]) -> Self {
        match commands.first() {
            None => Self::Empty,
            Some(ScheduleAction::SetMachineMode(MachineMode::On)) => Self::On,
            Some(ScheduleAction::SetMachineMode(MachineMode::Off)) => Self::Off,
            Some(ScheduleAction::SetMachineMode(MachineMode::PowerSaveStandby)) => Self::Sleep,
            Some(ScheduleAction::RunRoutine(..)) => Self::Run,
            Some(ScheduleAction::CancelRoutine) => Self::Cancel,
            Some(ScheduleAction::SetBoilerControlTarget(..))
            | Some(ScheduleAction::SetBoilerControlTargetValues(..)) => Self::Boiler,
        }
    }
}

/// What a schedule does, in **five** characters or fewer.
///
/// [`schedule_label`] has twelve columns and spends six on `HH:MM `, which leaves six -- and
/// the sixth is reserved for the `+` that marks a schedule with more than one action. So a
/// summary may be five, not six: a six-character one would fit a single-action schedule and
/// then silently lose its marker on a multi-action one, because `heapless::String` stops
/// accepting characters rather than reporting that it is full.
///
/// That is why `Cancel` reads `Stop` and the boiler actions read `Blr`.
/// `the_marker_survives_every_summary` holds all of it to the rule.
///
/// [`ScheduleActionKind::Empty`] is the exception at six characters, and safely: a schedule
/// with no actions cannot have more than one, so no marker can follow it. It says `(none)`
/// rather than leaving the half-row blank, because an empty half-row reads as something that
/// failed to load -- the failure mode this whole layer exists to avoid.
pub fn schedule_action_summary(kind: ScheduleActionKind) -> &'static str {
    match kind {
        ScheduleActionKind::Empty => "(none)",
        ScheduleActionKind::On => "On",
        ScheduleActionKind::Off => "Off",
        ScheduleActionKind::Sleep => "Sleep",
        ScheduleActionKind::Run => "Run",
        ScheduleActionKind::Cancel => "Stop",
        ScheduleActionKind::Boiler => "Blr",
    }
}

/// One schedule, as a menu row.
///
/// **Carries `index` rather than leaving it to be recovered from the row number**, for
/// [`crate::RoutineRow`]'s reason and one sharper one: schedule indices are not merely ordered
/// differently, they are *sparse*. `ScheduleStore::add_schedule` fills holes left by
/// `remove_schedule`, so after one removal the fifth row is not index 5 -- and
/// `MachineCommand::UpdateScheduleItem` names the index. A row that derived it from its
/// position would rewrite a different schedule, or none at all.
///
/// **Does not carry the `ScheduleItem`.** Rewriting one needs the whole item, to preserve
/// `on_days`, `on_date`, `once` and `commands`; a copy held here would be a copy that goes
/// stale the moment the browser changes a schedule this panel has open. The button task
/// re-reads the item from the store at commit time instead, which cannot go stale.
///
/// **No pre-rendered strings**, so 64 of these stay around 1 kB in a `Watch` payload rather
/// than 4. It is also the rule the GS3's `MenuValue` already states: a value is not rendered
/// text, because the two panels draw it differently.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ScheduleRow {
    /// The storage index. What `UpdateScheduleItem` names.
    pub index: u32,
    /// The trigger hour, 0..24.
    pub hour: u8,
    /// The trigger minute, 0..60.
    pub minute: u8,
    /// Whether it currently fires.
    pub enabled: bool,
    /// Whether it is marked to run only once.
    pub once: bool,
    /// Which weekdays it fires on.
    pub days: DaySet,
    /// The single date it fires on, if it is a dated trigger rather than a recurring one.
    pub on_date: Option<NaiveDate>,
    /// What its first action does.
    pub action: ScheduleActionKind,
    /// Whether it has more than one action.
    pub more_actions: bool,
}

/// Hand-written for `ScheduleTrigger`'s reason: `chrono::NaiveDate` does not implement
/// `defmt::Format`, so the date has to be spelled out field by field rather than derived.
#[cfg(feature = "defmt")]
impl defmt::Format for ScheduleRow {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "ScheduleRow {{ #{} {:02}:{:02} {}",
            self.index,
            self.hour,
            self.minute,
            schedule_action_summary(self.action)
        );

        if self.more_actions {
            defmt::write!(f, "+");
        }

        if let Some(ref date) = self.on_date {
            defmt::write!(f, " date:{}-{:02}-{:02}", date.year(), date.month(), date.day());
        } else {
            defmt::write!(f, " days:{:07b}", self.days.mask);
            if self.days.every_day {
                defmt::write!(f, " (all)");
            }
        }

        if !self.enabled {
            defmt::write!(f, " DISABLED");
        }

        if self.once {
            defmt::write!(f, " once");
        }

        defmt::write!(f, " }}");
    }
}

/// A schedule list, in storage-index order.
pub type ScheduleRows = Vec<ScheduleRow, MAX_MENU_SCHEDULES>;

/// Schedules as menu rows, in storage-index order.
///
/// **Ascending by index and deliberately not by time of day.** A list sorted by time reorders
/// itself the instant a time is edited, so the row under the user's finger after committing an
/// edit would be a *different* schedule from the one they just changed. The index is stable for
/// the life of the schedule; the time is the thing this menu exists to change.
///
/// Rows past [`MAX_MENU_SCHEDULES`] are dropped, for [`crate::routine_rows`]' reason.
pub fn schedule_rows<'a>(
    schedules: impl Iterator<Item = (usize, &'a ScheduleItem)>,
) -> ScheduleRows {
    let mut rows = ScheduleRows::new();

    for (index, item) in schedules {
        let row = ScheduleRow {
            index: index as u32,
            hour: item.trigger_at.on_hour,
            minute: item.trigger_at.on_minute,
            enabled: item.trigger_at.enabled,
            once: item.trigger_at.once,
            days: DaySet::from_trigger(&item.trigger_at),
            on_date: item.trigger_at.on_date,
            action: ScheduleActionKind::of(&item.commands),
            more_actions: item.commands.len() > 1,
        };

        if rows.push(row).is_err() {
            break;
        }
    }

    rows
}

/// A schedule's list label: `HH:MM <action>`, at most [`SCHEDULE_LABEL_LEN`] columns.
///
/// The time leads because it is what distinguishes one schedule from another at a glance, and
/// because it is the field this menu edits. The action follows so that two schedules at the
/// same time are still tellable apart.
///
/// A `+` marks a schedule with more than one action -- not to say what the others are, which
/// this panel cannot, but so that a user editing the time of a multi-action schedule knows
/// there is more to it than the row says.
pub fn schedule_label(row: &ScheduleRow) -> String<SCHEDULE_LABEL_LEN> {
    let mut out = String::new();
    let _ = write!(out, "{:02}:{:02} ", row.hour, row.minute);
    let _ = out.push_str(schedule_action_summary(row.action));
    if row.more_actions {
        let _ = out.push('+');
    }
    out
}

/// When a schedule repeats, in words.
///
/// Read-only on this panel: a day set is seven independent switches and a date is a calendar,
/// and neither fits four buttons -- see `MENU-STRUCTURE.md` §5.
///
/// The three common sets collapse to a word, because `Mon Tue Wed Thu Fri` is something a user
/// has to *read* where `Weekdays` is something they recognise.
///
/// An empty day set reads `Never` rather than blank. It is a storable state that can never
/// fire, and a blank row would be indistinguishable from one still loading.
pub fn schedule_recurrence(row: &ScheduleRow) -> String<SCHEDULE_RECURRENCE_LEN> {
    let mut out: String<SCHEDULE_RECURRENCE_LEN> = String::new();

    // A dated trigger fires on that date and never recurs, so the day set is not what governs
    // it and printing both would suggest otherwise.
    if let Some(date) = row.on_date {
        let _ = write!(out, "{:04}-{:02}-{:02}", date.year(), date.month(), date.day());
    } else if row.days.is_every_day() {
        let _ = out.push_str("Every day");
    } else if row.days.is_never() {
        let _ = out.push_str("Never");
    } else if row.days.mask == WEEKDAYS {
        let _ = out.push_str("Weekdays");
    } else if row.days.mask == WEEKENDS {
        let _ = out.push_str("Weekends");
    } else {
        for (bit, name) in WEEK_NAMES.iter().enumerate() {
            if row.days.mask & (1 << bit) != 0 {
                if !out.is_empty() {
                    let _ = out.push(' ');
                }
                let _ = out.push_str(name);
            }
        }
    }

    if row.once {
        let _ = out.push_str(" once");
    }

    out
}

/// What one press changes about a stored schedule.
///
/// Names the change rather than carrying a rewritten `ScheduleItem`, because building one needs
/// the stored item and only an async context can read the store. The GS3's button task defers
/// this from its synchronous press path into its task loop.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ScheduleChange {
    /// Switch it on or off, keeping everything else.
    Enabled(bool),
    /// Move it, keeping everything else -- including the days, the date and the actions.
    Time {
        /// The new hour, 0..24.
        hour: u8,
        /// The new minute, 0..60.
        minute: u8,
    },
}

/// Apply a change to a stored schedule, preserving every field it does not name.
///
/// **The preservation is the whole of this function.** `MachineCommand::UpdateScheduleItem`
/// replaces the stored item wholesale, so anything not copied across here is silently deleted:
/// a user toggling a schedule off and on again would lose its day set, its date, its `once`
/// flag and every action, with nothing reporting that it happened.
///
/// Here rather than in the firmware because that crate cannot host a test binary, and this is
/// the function whose failure stays invisible until someone's weekday-only warm-up starts
/// running on a Sunday.
pub fn apply_schedule_change(item: &ScheduleItem, change: ScheduleChange) -> ScheduleItem {
    let mut updated = item.clone();

    match change {
        ScheduleChange::Enabled(enabled) => updated.trigger_at.enabled = enabled,
        ScheduleChange::Time { hour, minute } => {
            updated.trigger_at.on_hour = hour;
            updated.trigger_at.on_minute = minute;
        }
    }

    updated
}

/// Which half of `HH:MM` buttons 1 and 2 are moving.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TimeField {
    /// The hours. Where the editor starts.
    #[default]
    Hour,
    /// The minutes.
    Minute,
}

/// How far one press moves the minutes.
///
/// Five, not one: a schedule is a thing you set to "about half past seven", and stepping a full
/// hour at one minute a press is fifty-nine presses on a panel with one button per direction.
pub const MINUTE_STEP: u8 = 5;

/// How many characters `HH:MM` occupies.
pub const TIME_TEXT_LEN: usize = 5;

/// A time being edited on a four-button panel.
///
/// **Not an [`variegated_menu::Adjustable`], and the difference is not cosmetic.** `Adjustable`
/// saturates; a clock wraps. At 23:55 a saturating editor would leave button 2 doing nothing,
/// which on a panel with one button per direction reads as a broken machine -- the same failure
/// `ListGeometry::wrap` exists to avoid.
///
/// The two fields step independently on purpose. Carrying from minutes into hours would move a
/// number the user is not looking at, on a screen whose whole job is that they are looking at
/// one of two.
///
/// `Copy`, because the GS3 publishes it inside `MenuSnapshot`, which is a `Watch` payload.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TimeEdit {
    hour: u8,
    minute: u8,
    field: TimeField,
}

impl TimeEdit {
    /// Seed from a stored trigger, starting on the hours.
    ///
    /// Out-of-range values are folded in rather than trusted. A stored `25:07` is what an older
    /// firmware or a half-written flash page produces, and an editor showing an impossible time
    /// is one whose buttons appear not to work.
    ///
    /// **A minute that is not a multiple of [`MINUTE_STEP`] is kept as it is.** There is no
    /// cancel on this screen, so opening a schedule stored at 07:07 merely to look at it and
    /// pressing Done must not quietly move it to 07:05. It snaps to the step on the first
    /// press instead -- see [`Self::increase`].
    pub fn new(hour: u8, minute: u8) -> Self {
        Self { hour: hour % 24, minute: minute % 60, field: TimeField::Hour }
    }

    /// The hour, 0..24.
    pub const fn hour(&self) -> u8 {
        self.hour
    }

    /// The minute, 0..60.
    pub const fn minute(&self) -> u8 {
        self.minute
    }

    /// Which field buttons 1 and 2 are moving.
    pub const fn field(&self) -> TimeField {
        self.field
    }

    /// One step up, wrapping. Hours 23 to 00; minutes 55 to 00, hour unchanged.
    ///
    /// A minute off the step lands on the next multiple rather than five past itself, so 07
    /// becomes 10. That is what makes an off-step stored time editable back onto the grid
    /// without a separate control.
    pub fn increase(&mut self) {
        match self.field {
            TimeField::Hour => self.hour = (self.hour + 1) % 24,
            TimeField::Minute => {
                let next = self.minute / MINUTE_STEP * MINUTE_STEP + MINUTE_STEP;
                self.minute = if next >= 60 { 0 } else { next };
            }
        }
    }

    /// One step down, wrapping. A minute off the step lands on the multiple below, so 07
    /// becomes 05.
    pub fn decrease(&mut self) {
        match self.field {
            TimeField::Hour => self.hour = (self.hour + 23) % 24,
            TimeField::Minute => {
                let floor = self.minute / MINUTE_STEP * MINUTE_STEP;
                self.minute = if floor != self.minute {
                    floor
                } else if self.minute == 0 {
                    60 - MINUTE_STEP
                } else {
                    self.minute - MINUTE_STEP
                };
            }
        }
    }

    /// Move to the other field. Button 3, which on every other editor confirms.
    pub fn next_field(&mut self) {
        self.field = match self.field {
            TimeField::Hour => TimeField::Minute,
            TimeField::Minute => TimeField::Hour,
        };
    }

    /// `HH:MM`.
    pub fn text(&self) -> String<TIME_TEXT_LEN> {
        let mut out = String::new();
        let _ = write!(out, "{:02}:{:02}", self.hour, self.minute);
        out
    }

    /// Which characters of [`Self::text`] the selected field occupies, as `start..end`.
    ///
    /// **Here rather than in either renderer**, because both of them mark the selected field
    /// and they have to mark the same one: the TFT draws those characters white and the rest
    /// grey, and the character LCD parks the HD44780 block cursor on the first of them. Two
    /// implementations of "which two characters are the hours" is two chances to be off by one
    /// on one panel only, and the panels are never looked at side by side.
    pub const fn field_span(&self) -> (usize, usize) {
        match self.field {
            TimeField::Hour => (0, 2),
            TimeField::Minute => (3, 5),
        }
    }

    /// Which column of a right-aligned sixteen-column row the selected field starts at.
    ///
    /// The character LCD's editor row is `{:>16}`, so `HH:MM` sits in columns 11..=15 and the
    /// span above shifts by the padding. Derived rather than written as literals, so the
    /// cursor cannot drift from the text it is marking.
    pub const fn cursor_column(&self) -> u8 {
        let (start, _) = self.field_span();
        (COLUMNS - TIME_TEXT_LEN + start) as u8
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;
    use heapless::index_set::FnvIndexSet;
    use variegated_controller_types::{BoilerIndex, ScheduleItem, ScheduleTrigger};

    /// A trigger at `hour:minute` on the given days, or every day when `days` is `None`.
    fn trigger(hour: u8, minute: u8, days: Option<&[Weekday]>) -> ScheduleTrigger {
        ScheduleTrigger {
            on_hour: hour,
            on_minute: minute,
            on_days: days.map(|days| {
                let mut set: FnvIndexSet<Weekday, 8> = FnvIndexSet::new();
                for day in days {
                    let _ = set.insert(*day);
                }
                set
            }),
            on_date: None,
            enabled: true,
            once: false,
        }
    }

    fn item(trigger_at: ScheduleTrigger, commands: alloc::vec::Vec<ScheduleAction>) -> ScheduleItem {
        ScheduleItem { trigger_at, commands }
    }

    fn row_of(item: &ScheduleItem) -> ScheduleRow {
        schedule_rows(core::iter::once((0usize, item)))[0]
    }

    // ---- apply_schedule_change: the silent-data-loss guards -------------------------------

    /// Toggling `Enabled` must not disturb anything else about the schedule.
    ///
    /// `UpdateScheduleItem` replaces the item wholesale, so a field this drops is a field the
    /// user loses with no error anywhere. Asserted field by field rather than with one
    /// equality, so a failure names *which* field went missing.
    #[test]
    fn toggling_enabled_keeps_the_days_the_date_the_once_flag_and_every_action() {
        let mut original = trigger(7, 30, Some(&[Weekday::Mon, Weekday::Wed, Weekday::Fri]));
        original.on_date = NaiveDate::from_ymd_opt(2026, 8, 20);
        original.once = true;
        let original = item(
            original,
            vec![
                ScheduleAction::SetMachineMode(MachineMode::On),
                ScheduleAction::SetBoilerControlTargetValues(BoilerIndex::from(0u8), Default::default()),
            ],
        );

        let updated = apply_schedule_change(&original, ScheduleChange::Enabled(false));

        assert!(!updated.trigger_at.enabled, "the change itself did not take");
        assert_eq!(updated.trigger_at.on_hour, 7);
        assert_eq!(updated.trigger_at.on_minute, 30);
        assert_eq!(updated.trigger_at.on_date, original.trigger_at.on_date, "date lost");
        assert!(updated.trigger_at.once, "once flag lost");
        assert_eq!(updated.commands.len(), 2, "actions lost");
        let days = updated.trigger_at.on_days.expect("day set lost entirely");
        assert_eq!(days.len(), 3, "day set truncated");
        for day in [Weekday::Mon, Weekday::Wed, Weekday::Fri] {
            assert!(days.contains(&day), "{day:?} lost from the day set");
        }
    }

    /// The same guarantee for a time change, which is the one the editor sends.
    #[test]
    fn changing_the_time_keeps_everything_else() {
        let mut original = trigger(7, 30, Some(&[Weekday::Sat]));
        original.once = true;
        let original = item(original, vec![ScheduleAction::CancelRoutine]);

        let updated =
            apply_schedule_change(&original, ScheduleChange::Time { hour: 6, minute: 45 });

        assert_eq!(updated.trigger_at.on_hour, 6);
        assert_eq!(updated.trigger_at.on_minute, 45);
        assert!(updated.trigger_at.enabled, "enabled flag disturbed by a time change");
        assert!(updated.trigger_at.once, "once flag lost");
        assert_eq!(updated.commands.len(), 1, "actions lost");
        assert!(
            updated.trigger_at.on_days.expect("day set lost").contains(&Weekday::Sat),
            "day set lost"
        );
    }

    /// Both halves land. A change that wrote only the hour would still pass a test that
    /// checked the item was rebuilt.
    #[test]
    fn changing_the_time_changes_both_halves() {
        let original = item(trigger(7, 30, None), vec![]);
        let updated =
            apply_schedule_change(&original, ScheduleChange::Time { hour: 23, minute: 55 });
        assert_eq!((updated.trigger_at.on_hour, updated.trigger_at.on_minute), (23, 55));
    }

    // ---- schedule_rows -------------------------------------------------------------------

    /// The row must carry the *storage* index, not its position in the list.
    ///
    /// `[0, 3, 7]` is the shape hole-filling produces after removals. A row that reported its
    /// position would name index 1 for the middle row and rewrite a schedule the user never
    /// looked at -- or, if index 1 is empty, silently do nothing.
    #[test]
    fn a_row_carries_the_storage_index_not_its_position() {
        let a = item(trigger(6, 0, None), vec![]);
        let b = item(trigger(7, 0, None), vec![]);
        let c = item(trigger(8, 0, None), vec![]);
        let rows = schedule_rows([(0usize, &a), (3, &b), (7, &c)].into_iter());

        assert_eq!(rows.len(), 3);
        assert_eq!(rows[0].index, 0);
        assert_eq!(rows[1].index, 3);
        assert_eq!(rows[2].index, 7);
    }

    /// The list is capped, and hitting the cap truncates rather than panicking.
    #[test]
    fn the_list_truncates_rather_than_panicking_when_full() {
        let one = item(trigger(6, 0, None), vec![]);
        let stored: alloc::vec::Vec<(usize, &ScheduleItem)> =
            (0..MAX_MENU_SCHEDULES + 8).map(|index| (index, &one)).collect();

        assert_eq!(schedule_rows(stored.into_iter()).len(), MAX_MENU_SCHEDULES);
    }

    #[test]
    fn an_empty_store_is_an_empty_list() {
        assert!(schedule_rows(core::iter::empty()).is_empty());
    }

    #[test]
    fn a_row_reflects_the_triggers_time_and_enabled_flag() {
        let mut t = trigger(7, 5, None);
        t.enabled = false;
        let row = row_of(&item(t, vec![]));

        assert_eq!((row.hour, row.minute), (7, 5));
        assert!(!row.enabled);
    }

    // ---- labels --------------------------------------------------------------------------

    /// Every action category, at every extreme of the clock, must fit twelve columns.
    ///
    /// Twelve is the character LCD's label field, and it truncates from the right -- so a
    /// label one character too long does not lose its own tail, it eats the `ON`/`OFF` beside
    /// it. `heapless::String` stops accepting characters when full, so an overflow here is a
    /// *silently shortened* label rather than a panic.
    #[test]
    fn the_widest_label_fits_its_field() {
        let kinds = [
            ScheduleActionKind::Empty,
            ScheduleActionKind::On,
            ScheduleActionKind::Off,
            ScheduleActionKind::Sleep,
            ScheduleActionKind::Run,
            ScheduleActionKind::Cancel,
            ScheduleActionKind::Boiler,
        ];

        for kind in kinds {
            assert!(
                schedule_action_summary(kind).len() <= 6,
                "{kind:?} summary is wider than the six columns the label leaves it"
            );

            for (hour, minute) in [(0u8, 0u8), (23, 59)] {
                for more_actions in [false, true] {
                    let row = ScheduleRow {
                        hour,
                        minute,
                        action: kind,
                        more_actions,
                        ..Default::default()
                    };
                    let label = schedule_label(&row);
                    assert!(
                        label.len() <= SCHEDULE_LABEL_LEN,
                        "label {label:?} exceeds {SCHEDULE_LABEL_LEN} columns"
                    );
                    assert!(
                        label.starts_with(&alloc::format!("{hour:02}:{minute:02}")),
                        "label {label:?} lost its time"
                    );
                    assert!(
                        label.contains(schedule_action_summary(kind)),
                        "label {label:?} lost its action summary"
                    );
                }
            }
        }
    }

    /// The `+` must survive for **every** action category, at the widest clock.
    ///
    /// This is the test the six-character summaries failed. `07:30 Cancel+` is thirteen
    /// characters into a twelve-character buffer, and `heapless::String` stops accepting
    /// characters rather than reporting that it is full -- so the marker vanished, silently,
    /// for exactly the two categories whose names were longest. Asserting the label's *width*
    /// alone did not catch it, because the truncated label was still twelve wide and still
    /// contained its summary.
    #[test]
    fn the_marker_survives_every_summary() {
        let kinds = [
            ScheduleActionKind::On,
            ScheduleActionKind::Off,
            ScheduleActionKind::Sleep,
            ScheduleActionKind::Run,
            ScheduleActionKind::Cancel,
            ScheduleActionKind::Boiler,
        ];

        for kind in kinds {
            let row = ScheduleRow {
                hour: 23,
                minute: 59,
                action: kind,
                more_actions: true,
                ..Default::default()
            };
            let label = schedule_label(&row);

            assert!(
                label.ends_with('+'),
                "{kind:?} lost its multi-action marker: {label:?}"
            );
            assert!(label.len() <= SCHEDULE_LABEL_LEN, "{label:?} overflows the label field");
            assert!(
                label.contains(schedule_action_summary(kind)),
                "{kind:?} lost its summary: {label:?}"
            );
        }

        // `Empty` is the one summary allowed six characters, because a schedule with no
        // actions can never have more than one and so can never take a marker.
        let empty = ScheduleRow {
            hour: 23,
            minute: 59,
            action: ScheduleActionKind::Empty,
            more_actions: false,
            ..Default::default()
        };
        assert_eq!(schedule_label(&empty).as_str(), "23:59 (none)");
    }

    /// A schedule with no actions says so, rather than rendering as a bare time with a blank
    /// half-row after it.
    #[test]
    fn an_empty_action_list_says_so_rather_than_nothing() {
        let row = row_of(&item(trigger(7, 30, None), vec![]));
        assert_eq!(row.action, ScheduleActionKind::Empty);
        assert_eq!(schedule_label(&row).as_str(), "07:30 (none)");
    }

    /// More than one action is marked, so a user editing the time of a multi-action schedule
    /// knows the row is not the whole story.
    #[test]
    fn a_multi_action_schedule_is_marked() {
        let one = row_of(&item(
            trigger(7, 30, None),
            vec![ScheduleAction::SetMachineMode(MachineMode::On)],
        ));
        let two = row_of(&item(
            trigger(7, 30, None),
            vec![
                ScheduleAction::SetMachineMode(MachineMode::On),
                ScheduleAction::CancelRoutine,
            ],
        ));

        assert!(!one.more_actions);
        assert!(two.more_actions);
        assert!(!schedule_label(&one).ends_with('+'));
        assert!(schedule_label(&two).ends_with('+'));
    }

    #[test]
    fn every_action_variant_is_categorised() {
        for (commands, expected) in [
            (vec![ScheduleAction::SetMachineMode(MachineMode::On)], ScheduleActionKind::On),
            (vec![ScheduleAction::SetMachineMode(MachineMode::Off)], ScheduleActionKind::Off),
            (
                vec![ScheduleAction::SetMachineMode(MachineMode::PowerSaveStandby)],
                ScheduleActionKind::Sleep,
            ),
            (vec![ScheduleAction::CancelRoutine], ScheduleActionKind::Cancel),
            (vec![], ScheduleActionKind::Empty),
        ] {
            assert_eq!(ScheduleActionKind::of(&commands), expected);
        }
    }

    // ---- recurrence ----------------------------------------------------------------------

    /// The sets a person recognises collapse to a word; the rest list their days in week
    /// order, never in the hash set's order.
    #[test]
    fn recurrence_collapses_the_sets_people_recognise() {
        let cases: [(Option<&[Weekday]>, &str); 6] = [
            (None, "Every day"),
            (
                Some(&[
                    Weekday::Mon,
                    Weekday::Tue,
                    Weekday::Wed,
                    Weekday::Thu,
                    Weekday::Fri,
                    Weekday::Sat,
                    Weekday::Sun,
                ]),
                "Every day",
            ),
            (
                Some(&[Weekday::Mon, Weekday::Tue, Weekday::Wed, Weekday::Thu, Weekday::Fri]),
                "Weekdays",
            ),
            (Some(&[Weekday::Sat, Weekday::Sun]), "Weekends"),
            (Some(&[Weekday::Mon, Weekday::Wed]), "Mon Wed"),
            (Some(&[]), "Never"),
        ];

        for (days, expected) in cases {
            let row = row_of(&item(trigger(7, 30, days), vec![]));
            assert_eq!(schedule_recurrence(&row).as_str(), expected, "for {days:?}");
        }
    }

    /// Days are listed in week order regardless of what order they went into the set, because
    /// `FnvIndexSet` has none.
    #[test]
    fn days_are_listed_in_week_order() {
        let row = row_of(&item(
            trigger(7, 30, Some(&[Weekday::Sun, Weekday::Tue, Weekday::Sat])),
            vec![],
        ));
        assert_eq!(schedule_recurrence(&row).as_str(), "Tue Sat Sun");
    }

    /// An empty day set can never fire. Saying `Never` is the difference between a schedule a
    /// user can see is broken and a blank row that reads as still loading.
    #[test]
    fn an_empty_day_set_is_never_not_every_day() {
        let empty = row_of(&item(trigger(7, 30, Some(&[])), vec![]));
        let unset = row_of(&item(trigger(7, 30, None), vec![]));

        assert!(empty.days.is_never());
        assert!(!empty.days.is_every_day());
        assert!(unset.days.is_every_day());
        assert!(!unset.days.is_never());
    }

    /// A dated trigger fires on that date and never recurs, so the date is what gets shown.
    #[test]
    fn a_dated_trigger_reads_as_a_date() {
        let mut t = trigger(7, 30, Some(&[Weekday::Mon]));
        t.on_date = NaiveDate::from_ymd_opt(2026, 8, 20);
        let row = row_of(&item(t, vec![]));
        assert_eq!(schedule_recurrence(&row).as_str(), "2026-08-20");
    }

    #[test]
    fn once_is_marked() {
        let mut t = trigger(7, 30, None);
        t.once = true;
        let row = row_of(&item(t, vec![]));
        assert_eq!(schedule_recurrence(&row).as_str(), "Every day once");
    }

    /// The widest recurrence must fit its buffer.
    ///
    /// `heapless::String::push_str` rejects the **whole** write when it does not fit, so an
    /// overflow here does not truncate the recurrence -- it blanks whatever part did not fit,
    /// which on the panel is indistinguishable from a value that failed to load.
    #[test]
    fn the_widest_recurrence_fits_its_buffer() {
        // A set that collapses to nothing, so every name is spelled out, plus the `once`
        // suffix: the longest string this function can produce.
        let mut t = trigger(
            7,
            30,
            Some(&[Weekday::Mon, Weekday::Tue, Weekday::Thu, Weekday::Sat, Weekday::Sun]),
        );
        t.once = true;
        let row = row_of(&item(t, vec![]));
        let text = schedule_recurrence(&row);

        assert_eq!(text.as_str(), "Mon Tue Thu Sat Sun once");
        assert!(text.len() <= SCHEDULE_RECURRENCE_LEN);
        assert!(!text.is_empty());
    }

    // ---- TimeEdit ------------------------------------------------------------------------

    /// Minutes wrap within the hour, in both directions, without disturbing the hour.
    ///
    /// This is the property the whole `TimeEdit`-instead-of-`Adjustable` decision exists for:
    /// `Adjustable` would saturate at 55 and leave button 2 inert.
    #[test]
    fn minutes_wrap_without_touching_the_hour() {
        let mut time = TimeEdit::new(7, 55);
        time.next_field();
        time.increase();
        assert_eq!((time.hour(), time.minute()), (7, 0));

        let mut time = TimeEdit::new(7, 0);
        time.next_field();
        time.decrease();
        assert_eq!((time.hour(), time.minute()), (7, 55));
    }

    #[test]
    fn hours_wrap() {
        let mut time = TimeEdit::new(23, 0);
        time.increase();
        assert_eq!(time.hour(), 0);

        let mut time = TimeEdit::new(0, 0);
        time.decrease();
        assert_eq!(time.hour(), 23);
    }

    /// An off-step stored minute is preserved until the user actually presses something.
    ///
    /// There is no cancel on this screen, so opening 07:07 to read it and pressing Done must
    /// give back 07:07. The first press snaps it onto the grid, in the direction pressed.
    #[test]
    fn an_off_step_minute_is_kept_until_it_is_touched() {
        assert_eq!(TimeEdit::new(7, 7).minute(), 7);

        let mut up = TimeEdit::new(7, 7);
        up.next_field();
        up.increase();
        assert_eq!(up.minute(), 10);

        let mut down = TimeEdit::new(7, 7);
        down.next_field();
        down.decrease();
        assert_eq!(down.minute(), 5);
    }

    /// A stored time that cannot exist is folded in rather than shown.
    #[test]
    fn an_impossible_stored_time_is_folded_in() {
        let time = TimeEdit::new(25, 61);
        assert!(time.hour() < 24);
        assert!(time.minute() < 60);
    }

    /// Switching fields moves the selection and nothing else, and two presses return.
    #[test]
    fn switching_field_does_not_change_the_time() {
        let mut time = TimeEdit::new(7, 30);
        assert_eq!(time.field(), TimeField::Hour);

        time.next_field();
        assert_eq!(time.field(), TimeField::Minute);
        assert_eq!((time.hour(), time.minute()), (7, 30));

        time.next_field();
        assert_eq!(time.field(), TimeField::Hour);
        assert_eq!((time.hour(), time.minute()), (7, 30));
    }

    /// A full cycle of one field leaves the other exactly where it was.
    #[test]
    fn stepping_moves_only_the_selected_field() {
        let mut time = TimeEdit::new(7, 30);
        for _ in 0..24 {
            time.increase();
        }
        assert_eq!((time.hour(), time.minute()), (7, 30), "hours did not cycle cleanly");

        time.next_field();
        for _ in 0..(60 / MINUTE_STEP) {
            time.increase();
        }
        assert_eq!((time.hour(), time.minute()), (7, 30), "minutes did not cycle cleanly");
    }

    /// The span must select the digits it claims to, or the TFT colours one field and the LCD
    /// marks the other.
    #[test]
    fn the_field_span_matches_the_text() {
        for (hour, minute) in [(0u8, 0u8), (7, 30), (23, 55)] {
            let mut time = TimeEdit::new(hour, minute);
            let text = time.text();

            let (start, end) = time.field_span();
            assert_eq!(&text[start..end], &alloc::format!("{hour:02}"), "hours span wrong");

            time.next_field();
            let (start, end) = time.field_span();
            assert_eq!(&text[start..end], &alloc::format!("{minute:02}"), "minutes span wrong");
        }
    }

    /// The cursor column must land on the first character of the selected field once the row
    /// is right-aligned, and must never leave the display.
    #[test]
    fn the_cursor_column_lands_on_the_selected_field() {
        let mut time = TimeEdit::new(7, 30);
        let row = alloc::format!("{:>16.16}", time.text());

        let hour_col = time.cursor_column() as usize;
        assert!(hour_col < COLUMNS);
        assert_eq!(row.as_bytes()[hour_col], b'0', "cursor is not on the first hour digit");

        time.next_field();
        let minute_col = time.cursor_column() as usize;
        assert!(minute_col < COLUMNS);
        assert_eq!(row.as_bytes()[minute_col], b'3', "cursor is not on the first minute digit");
    }
}
