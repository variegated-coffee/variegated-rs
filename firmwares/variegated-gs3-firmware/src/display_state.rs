//! Shared display state model for dual boiler espresso machines
//!
//! This module provides a shared state model that can be used by both
//! the LCD renderer and the graphical renderer. It determines the current
//! display mode based on machine status and provides common formatting utilities.

use alloc::format;
use alloc::string::{String, ToString};
use core::time::Duration;
use embassy_time::Instant;
use variegated_controller_types::{MachineMode, Status, SingleGroupControllerGroups};

/// Duration to display post-brew summary after brewing completes (milliseconds)
const POST_BREW_DISPLAY_DURATION_MS: u64 = 3000;

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
}

impl DisplayState {
    /// Create a new display state tracker
    pub fn new() -> Self {
        Self {
            status: Status::default(),
            last_brew_time: None,
            was_brewing: false,
            last_update: Instant::now(),
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
