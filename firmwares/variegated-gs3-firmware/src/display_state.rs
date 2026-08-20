//! Shared display state model for dual boiler espresso machines
//!
//! This module provides a shared state model that can be used by both
//! the LCD renderer and the graphical renderer. It determines the current
//! display mode based on machine status and provides common formatting utilities.

use alloc::format;
use alloc::string::{String, ToString};
use core::time::Duration;
use embassy_time::Instant;
use variegated_controller_lib::routine::Routine;
use variegated_controller_types::{MachineMode, RoutineIndex, Status, SingleGroupControllerGroups};
use variegated_machine_menu::RoutineRows;
use crate::menu::MenuSnapshot;

/// Duration to display post-brew summary after brewing completes (milliseconds)
const POST_BREW_DISPLAY_DURATION_MS: u64 = 3000;

/// How long the dose popup stays up.
const DOSE_POPUP_DURATION_MS: u64 = 5000;

/// Display mode enum representing the current state of the machine UI
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DisplayMode {
    /// Machine is off
    Off,
    /// Machine is in power save standby
    PowerSaveStandby,
    /// Machine is idle and ready
    Idle,
    /// Machine is actively brewing
    Brewing,
    /// Post-brew summary display (within 3 seconds after brew)
    PostBrew,
    /// Routine execution in progress
    RoutineExecution,
}

/// Something the machine is doing that is worth putting a box on the screen for.
///
/// Brewing is not here: it has a whole [`DisplayMode`] of its own, with numbers a box would
/// cover. This is for the two operations that had no feedback at all.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActivityOverlay {
    /// The water tap is running.
    HotWater,
    /// The steam valve is open.
    Steaming,
}

impl ActivityOverlay {
    /// The one line the overlay draws.
    ///
    /// Short on purpose. The TFT box is 200px wide and every font in this firmware is a
    /// `_tr` variant -- glyphs 32..127 -- and `render_aligned` resolves the whole bounding
    /// box before drawing, so a string that overruns or carries a non-ASCII character is
    /// dropped entirely and shows as an empty box. The character LCD has 16 columns.
    pub fn label(self) -> &'static str {
        match self {
            ActivityOverlay::HotWater => "Hot Water",
            ActivityOverlay::Steaming => "Steaming",
        }
    }
}

/// Shared display state tracker
pub struct DisplayState {
    /// Current system status
    pub status: Status,
    /// Last completed brew time for standby display
    pub last_brew_time: Option<Duration>,
    /// Track previous brewing state to detect transitions
    was_brewing: bool,
    /// Last display update time for rate limiting
    ///
    /// Read only by `should_update`, which only the character LCD task calls -- the TFT
    /// renderer paces itself. `cfg_attr` rather than a bare `allow` so this still reports
    /// as dead if the LCD stops using it too.
    #[cfg_attr(not(feature = "character-display"), allow(dead_code))]
    last_update: Instant,
    /// Where the menu is, cached from `MENU_WATCH` by whichever display task owns this.
    pub menu: MenuSnapshot,
    /// The routines the menu is listing, cached from the repository by the display task.
    ///
    /// Fetched here rather than published in `MenuSnapshot` because that payload is `Copy`
    /// and this is not, and because a routine's name -- unlike its value column -- does not
    /// change under a stationary selection, so there is nothing for a stale copy to get
    /// wrong. Both this and the button task build it through
    /// `variegated_machine_menu::routine_rows`, so the order is the same list on both sides.
    pub menu_routines: Option<RoutineRows>,
    /// What this machine declares it can sense, for deciding whether a routine can run.
    ///
    /// `None` until the first menu fetch populates it from `MACHINE_DEFINITION_REF`; that
    /// global is behind an async mutex and `menu_data` is sync, so it is latched at the one
    /// point that is already async. `None` reads as runnable -- see `menu::routine_runnable`.
    pub machine_definition: Option<&'static variegated_controller_types::MachineDefinition>,
    /// The routine whose parameter screen is open, and which one it is.
    ///
    /// The parameter rows need its names and units on every frame. The Silvia re-locks the
    /// repository for these per frame; this caches, the way `current_routine` already does
    /// for the routine-execution screen.
    ///
    /// Doubly optional, and the distinction matters. The outer says whether the fetch has
    /// happened; the inner is its result, `None` for a routine that has been deleted over
    /// HTTP while its screen was open. Collapsing them would make "not found" indistinguishable
    /// from "not fetched", and `menu_pending_fetch` would then ask again on every frame --
    /// re-taking the repository lock in a render loop, forever.
    pub menu_routine: Option<(RoutineIndex, Option<Routine>)>,
    /// The dose on the pending annotations as of the previous status.
    previous_dose_weight: Option<f32>,
    /// Whether any status has been seen yet.
    ///
    /// Without it the first status after boot -- which carries whatever the controller was
    /// already holding -- reads as a fresh capture and pops for a dose tagged before this task
    /// existed.
    dose_tracking_initialized: bool,
    /// When the dose popup expires, if one is up.
    dose_popup_until: Option<Instant>,
    /// What the menu reads out of `Configuration`, and the associations it lists.
    ///
    /// This task subscribes to the configuration channel purely for these. Until the
    /// Settings menu grew rows backed by `Configuration` it had no reason to -- the button
    /// task kept the one float that was needed, and both renderers passed `None`, which is
    /// why a config-backed row would have drawn blank here.
    ///
    /// `None` until the first `Configuration` arrives. The controller republishes every ten
    /// seconds whether or not anything changed, so the gap after boot is bounded.
    menu_config: Option<crate::menu::MenuConfig>,
    /// The Bluetooth associations, for the Bluetooth submenu.
    ///
    /// Cloned out of the published `Configuration` rather than borrowed from it: this task
    /// does not keep the `Configuration`, and the list is at most four entries of a name and
    /// a few scalars.
    bluetooth: Option<variegated_controller_types::bluetooth::BluetoothPeripheralList>,
    /// The schedule list, for the Schedules submenu.
    ///
    /// **Not fetched by this task**, unlike `menu_routines`. The button task is the sole
    /// builder and publishes it here, because a sparse-indexed list fetched independently on
    /// two sides is two chances to disagree about its length -- and this one changes under the
    /// user, since toggling `Enabled` rewrites it.
    schedules: Option<variegated_machine_menu::ScheduleRows>,
}

impl DisplayState {
    /// Create a new display state tracker
    pub fn new() -> Self {
        Self {
            status: Status::default(),
            last_brew_time: None,
            was_brewing: false,
            last_update: Instant::now(),
            menu: MenuSnapshot::closed(),
            menu_routines: None,
            machine_definition: None,
            menu_routine: None,
            previous_dose_weight: None,
            dose_tracking_initialized: false,
            dose_popup_until: None,
            menu_config: None,
            bluetooth: None,
            schedules: None,
        }
    }

    /// Take what the menu needs from a freshly published projection.
    pub fn update_menu_config(&mut self, snapshot: crate::menu::MenuConfigSnapshot) {
        self.menu_config = Some(snapshot.config);
        self.bluetooth = Some(snapshot.bluetooth);
        self.schedules = Some(snapshot.schedules);
    }

    /// The configuration projection, or its `Default` before the first one arrives.
    ///
    /// `Default` is deliberately the "nothing known" shape -- absent ceilings and a brew
    /// mode of `Off` -- so the rows it feeds grey out rather than showing invented numbers.
    pub fn menu_config(&self) -> crate::menu::MenuConfig {
        self.menu_config.unwrap_or_default()
    }

    /// What the menu needs in order to have rows, as this task has it cached.
    pub fn menu_data(&self) -> crate::menu::MenuData<'_> {
        let routine = self.menu_routine.as_ref().and_then(|(_, routine)| routine.as_ref());
        crate::menu::MenuData {
            routines: self.menu_routines.as_ref(),
            routine,
            // The edited values come over the watch rather than from the repository: they are
            // the one part of a parameter screen that no renderer could derive.
            values: self.menu.values,
            // Recomputed per frame from the live status rather than cached with the routine,
            // so a scale that drops while the parameter screen is open greys the Run row.
            // Cheap: a handful of map lookups over at most sixteen peripherals.
            routine_runnable: routine.is_none_or(|r| {
                crate::menu::routine_runnable(
                    r,
                    self.machine_definition,
                    &self.status.peripheral_status,
                )
            }),
            // Live for the same reason `routine_runnable` is: a scale can be switched off
            // while the submenu is open, and the calibration rows have to grey when it is.
            scale_calibration: crate::menu::scale_calibration(
                self.machine_definition,
                &self.status.peripheral_status,
            ),
            scale_present: crate::menu::scale_present(
                self.machine_definition,
                &self.status.peripheral_status,
            ),
            bluetooth: self.bluetooth.as_ref(),
            schedules: self.schedules.as_ref(),
            brew_target_unit: crate::menu::brew_target_unit(&self.menu_config()),
        }
    }

    /// What this task still has to fetch for the open menu. See `menu::pending_fetch`.
    pub fn menu_pending_fetch(&self) -> Option<crate::menu::MenuFetch> {
        crate::menu::pending_fetch(
            self.menu.stack.top().map(|frame| frame.id),
            self.menu_routines.is_some(),
            self.menu_routine.as_ref().map(|(index, _)| *index),
        )
    }

    /// Drop what was fetched for a menu that is no longer open.
    pub fn release_menu_data(&mut self) {
        self.menu_routines = None;
        self.menu_routine = None;
        // **`schedules` is deliberately not cleared.** It arrives on the config watch rather
        // than from a fetch here, so nothing in this task would ask for it again -- dropping
        // it would leave the Schedules list blank until the button task next happened to
        // republish, which on an unchanged machine is never.
    }

    /// Whether the dose popup is on screen right now.
    pub fn dose_popup_active(&self) -> bool {
        self.dose_popup_until.map(|until| Instant::now() < until).unwrap_or(false)
    }

    /// The dose it is showing.
    pub fn dose_popup_weight(&self) -> Option<f32> {
        self.previous_dose_weight
    }

    /// What the machine is doing right now, if it is worth an overlay.
    ///
    /// Read straight out of the live `Status` each frame rather than edge-detected the way
    /// the dose popup is: this has a duration of its own -- it is up exactly as long as the
    /// tap or the valve is -- where a dose capture is an instant and needs a timer to be
    /// visible at all.
    ///
    /// Suppressed while brewing or running a routine, exactly as the provisioning banner and
    /// the provisioning rows suppress themselves, and for the same reason: the box lands on
    /// the extraction numbers, and those are what the user is standing there watching. The
    /// tap can no longer run during a brew at all, so in practice this is the steam-during-a-
    /// shot case, which is the normal way to use a dual boiler.
    pub fn activity_overlay(&self) -> Option<ActivityOverlay> {
        match self.get_display_mode() {
            DisplayMode::Brewing | DisplayMode::RoutineExecution => return None,
            _ => {}
        }

        // Water first: the two are mutually exclusive by interlock in the direction that
        // matters, but steam can still be opened on top of a running tap, and one box holds
        // one label.
        if self.status.any_water_tap_dispensing() {
            Some(ActivityOverlay::HotWater)
        } else if self.status.any_steam_wand_steaming() {
            Some(ActivityOverlay::Steaming)
        } else {
            None
        }
    }

    /// Check if an update is needed (1Hz rate limiting)
    ///
    /// The five `format_*` helpers below and this one are the character LCD's; the TFT
    /// renderer formats its own values and paces itself. See the note on `last_update`
    /// for why these are `cfg_attr`-silenced rather than deleted or bare-`allow`ed.
    #[cfg_attr(not(feature = "character-display"), allow(dead_code))]
    pub fn should_update(&mut self) -> bool {
        let now = Instant::now();
        if now.saturating_duration_since(self.last_update).as_millis() >= 100 {
            self.last_update = now;
            true
        } else {
            false
        }
    }

    /// Update status and capture state transitions
    pub fn update_status(&mut self, new_status: Status) {
        // Check for brewing state transition to capture last brew time
        let current_brewing = self.is_currently_brewing(&new_status);
        if self.was_brewing && !current_brewing {
            // Brewing just stopped, capture the brew time
            if let Some(group_status) = new_status.get_group_status(SingleGroupControllerGroups::SingleGroup.as_index()) {
                self.last_brew_time = group_status.current_brew.as_ref().map(|b| b.brew_time);
            }
        }
        self.was_brewing = current_brewing;

        let new_dose = new_status.pending_shot_annotations.dose_weight();
        if !self.dose_tracking_initialized {
            self.dose_tracking_initialized = true;
        } else if let Some(grams) = new_dose {
            // Only a transition *to* a value, and only to a different one. `Some -> None` is the
            // post-shot clear (dual_boiler_single_group.rs:2806) and is not a capture; `Some(v) ->
            // Some(v)` is the same dose re-reported by the next status and is not one either.
            // `is_finite` because NaN never compares equal to itself, and a NaN dose would otherwise
            // re-arm this on every single status, forever.
            if grams.is_finite() && self.previous_dose_weight != Some(grams) {
                self.dose_popup_until = Some(
                    Instant::now() + embassy_time::Duration::from_millis(DOSE_POPUP_DURATION_MS),
                );
            }
        }
        self.previous_dose_weight = new_dose;

        self.status = new_status;
    }

    /// Check if the machine is currently brewing
    pub fn is_currently_brewing(&self, status: &Status) -> bool {
        status.get_group_status(SingleGroupControllerGroups::SingleGroup.as_index())
            .map(|group| group.is_brewing)
            .unwrap_or(false)
    }

    /// Check if we're in the post-brew display window (within 3 seconds after brew stopped)
    pub fn is_in_post_brew_window(&self) -> bool {
        if let Some(group_status) = self.status.get_group_status(SingleGroupControllerGroups::SingleGroup.as_index()) {
            if let Some(previous_brew) = &group_status.previous_brew {
                let now_millis = Instant::now().as_millis();
                let elapsed_since_stop = now_millis.saturating_sub(previous_brew.stopped_at_millis);
                return elapsed_since_stop < POST_BREW_DISPLAY_DURATION_MS;
            }
        }
        false
    }

    /// Determine the current display mode based on machine status
    pub fn get_display_mode(&self) -> DisplayMode {
        // Check machine mode first
        match self.status.mode {
            MachineMode::Off => return DisplayMode::Off,
            MachineMode::PowerSaveStandby => return DisplayMode::PowerSaveStandby,
            MachineMode::On => {
                // Continue with normal logic when machine is on
            }
        }

        // Check for routine execution
        if self.status.routine_execution.is_some() {
            return DisplayMode::RoutineExecution;
        }

        // Check for post-brew display window
        if self.is_in_post_brew_window() {
            return DisplayMode::PostBrew;
        }

        // Check if currently brewing
        if self.is_currently_brewing(&self.status) {
            return DisplayMode::Brewing;
        }

        // Default to idle
        DisplayMode::Idle
    }

    /// Format temperature as "XXX.XC" (6 chars)
    #[cfg_attr(not(feature = "character-display"), allow(dead_code))]
    pub fn format_temperature(&self, temp: Option<f32>) -> String {
        match temp {
            Some(t) => format!("{:5.1}C", t),
            None => "-----C".to_string(),
        }
    }

    /// Format pressure as "X.Xb" (4 chars)
    #[cfg_attr(not(feature = "character-display"), allow(dead_code))]
    pub fn format_pressure(&self, pressure: Option<f32>) -> String {
        match pressure {
            Some(p) => format!("{:3.1}b", p),
            None => "---b".to_string(),
        }
    }

    /// Format flow rate as "X.Xml/s" (7 chars)
    #[cfg_attr(not(feature = "character-display"), allow(dead_code))]
    pub fn format_flow_rate(&self, flow: Option<f32>) -> String {
        match flow {
            Some(f) => format!("{:3.1}ml/s", f),
            None => "---ml/s".to_string(),
        }
    }

    /// Format weight as "XX.Xg" (5 chars)
    #[cfg_attr(not(feature = "character-display"), allow(dead_code))]
    pub fn format_weight(&self, weight: Option<f32>) -> String {
        match weight {
            Some(w) => format!("{:4.1}g", w),
            None => "----g".to_string(),
        }
    }

    /// Format brew time as seconds "XXs" (3 chars max)
    #[cfg_attr(not(feature = "character-display"), allow(dead_code))]
    pub fn format_brew_time(&self, brew_time: Option<Duration>) -> String {
        match brew_time {
            Some(duration) => {
                let secs = duration.as_millis() as f32 / 1000.0;
                if secs < 100.0 {
                    format!("{:.1}s", secs)
                } else {
                    "99s".to_string() // Cap at 99s for display
                }
            }
            None => "0s".to_string(),
        }
    }

    /// Format tank level as "T0" (empty) or "T1" (non-empty) (2 chars)
    pub fn format_tank_level(&self) -> String {
        // Check the first available tank
        if let Some((_, tank_status)) = self.status.tank_statuses.iter().next() {
            match tank_status.water_level {
                Some(level) if level > 0 => "T1".to_string(),
                Some(_) => "T0".to_string(), // 0% water level
                None => "T?".to_string(), // No sensor data
            }
        } else {
            "T?".to_string() // No tank configured
        }
    }
}

impl Default for DisplayState {
    fn default() -> Self {
        Self::new()
    }
}
