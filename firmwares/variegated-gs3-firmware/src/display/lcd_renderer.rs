//! LCD renderer backend for 2x16 character HD44780 displays
//!
//! This module provides rendering functionality for the 2x16 character LCD display.
//! It takes the shared DisplayState and renders appropriate content to the LCD.

use alloc::string::{String, ToString};
use alloc::format;
use embassy_time::Instant;
use hd44780_controller::controller::{Controller, state::Init};
use variegated_controller_types::{DualBoilerSingleGroupControllerBoilers, Routine, RoutineExitCondition, SingleGroupControllerGroups, StateCondition, COMMS_STATUS_STALE_AFTER};
use variegated_controller_types::wifi::ImprovState;
use variegated_timekeeping::TimeKeeper;
use variegated_controller_lib::routine::RoutineRepository;

use crate::display_state::{DisplayState, DisplayMode};
use crate::menu::{self, MenuContext};
use crate::RoutineRepositoryMutex;

/// LCD-specific display state with buffer tracking
pub struct LcdDisplayState {
    /// Shared display state
    pub shared_state: DisplayState,
    /// Buffer tracking current display content (2 rows x 16 columns)
    display_buffer: [[char; 16]; 2],
    /// Track whether display has been initialized with content
    display_initialized: bool,
    /// Routine repository for looking up routine details
    routine_repository: &'static RoutineRepositoryMutex,
    /// Cached current routine being executed (fetched once per execution)
    pub current_routine: Option<Routine>,
    /// When the Improv identify flash ends, if one is running. Set by the display task.
    pub identify_until: Option<Instant>,
}

impl LcdDisplayState {
    /// Create a new LCD display state tracker
    pub fn new(routine_repository: &'static RoutineRepositoryMutex) -> Self {
        Self {
            shared_state: DisplayState::new(),
            display_buffer: [[' '; 16]; 2],
            display_initialized: false,
            routine_repository,
            current_routine: None,
            identify_until: None,
        }
    }

    /// Both rows, while the Improv provisioning window is open.
    ///
    /// The whole display, not the spare character at the end of
    /// [`Self::format_standby_row1`]. A 2x16 is full at all times, and one glyph tucked into
    /// the padding would be indistinguishable from a rendering fault. The window is a mode the
    /// user deliberately entered and which expires on its own, so taking the panel for its
    /// duration is proportionate.
    ///
    /// `None` means "not in a window, or no longer sure". The staleness check is the `W`
    /// icon's, for the reason documented on the graphical renderer: `comms_status` is a latch,
    /// and a dead comms processor must not leave a standing invitation to pair on screen.
    ///
    /// Every string here is at most 16 characters -- `pad_or_truncate_to_16` would cut a
    /// longer one silently, mid-word.
    pub fn provisioning_rows(&self) -> Option<(String, String)> {
        // Not while the screen is being used for something that cannot wait. This takes the
        // whole panel, and on 2x16 that means the shot timer and weight go with it. The window
        // cannot be *opened* while brewing, but it can already be open when brewing starts.
        match self.shared_state.get_display_mode() {
            DisplayMode::Brewing | DisplayMode::RoutineExecution => return None,
            _ => {}
        }

        let comms_stale = self.shared_state.status.comms_status_age
            .map(|age| age >= COMMS_STATUS_STALE_AFTER)
            .unwrap_or(true);
        if comms_stale {
            return None;
        }

        let improv = self.shared_state.status.comms_status.as_ref()?.improv;

        let second = match improv {
            ImprovState::Stopped => return None,
            ImprovState::AwaitingAuthorization | ImprovState::Authorized => "Ready to pair",
            ImprovState::Provisioning => "Connecting...",
            ImprovState::Provisioned => "Connected",
        };

        Some(("WiFi Setup".to_string(), second.to_string()))
    }

    /// Get the formatted text for the current display state
    pub async fn get_display_text(&self) -> (String, String) {
        // Ahead of the provisioning rows below: Identify is only ever sent from inside a
        // provisioning window, so anything checked after them would never be reached.
        //
        // Both rows filled and both rows blank, alternating at 4 Hz. The 2x16 has no other way
        // to be seen from across a room, which is the whole point of Improv Identify. The blank
        // phase relies on `pad_or_truncate_to_16` padding the empty string out with spaces --
        // a short write would leave the previous frame's characters on the panel.
        if let Some(until) = self.identify_until {
            let now = Instant::now();
            if now < until {
                let lit = (now.as_millis() / 250) % 2 == 0;
                let row = if lit { "*".repeat(16) } else { String::new() };
                return (row.clone(), row);
            }
        }

        // After the identify flash and ahead of the provisioning rows and the mode match, for the
        // reasons in `graphical_renderer::render`. Ahead of the provisioning rows specifically
        // because the menu's own value column says whether the window is open, and replacing a menu
        // the user is navigating with "Ready to pair" strands them.
        if self.shared_state.menu.stack.is_open() {
            return self.menu_rows();
        }

        // A dose the user just captured, for five seconds. Ahead of the provisioning rows because it
        // is direct feedback for an action taken a second ago.
        if self.shared_state.dose_popup_active() {
            if let Some(grams) = self.shared_state.dose_popup_weight() {
                return ("  Dose captured ".to_string(), format!("     {:.1} g", grams));
            }
        }

        // Ahead of the mode match rather than inside it: the window can be open in any mode,
        // and a copy of this check in each arm is a copy that will be missed when an arm is
        // added.
        if let Some(rows) = self.provisioning_rows() {
            return rows;
        }

        match self.shared_state.get_display_mode() {
            DisplayMode::Off => {
                (self.format_off_row1(), self.format_off_row2())
            }
            DisplayMode::PowerSaveStandby => {
                (self.format_power_save_standby_row1(), self.format_power_save_standby_row2())
            }
            DisplayMode::RoutineExecution => {
                (self.format_routine_row1().await, self.format_routine_row2().await)
            }
            DisplayMode::PostBrew => {
                (self.format_post_brew_row1(), self.format_post_brew_row2())
            }
            DisplayMode::Brewing => {
                (self.format_brewing_row1(), self.format_brewing_row2())
            }
            DisplayMode::Idle => {
                (self.format_standby_row1(), self.format_standby_row2())
            }
        }
    }

    /// Efficiently update the LCD display by only writing changed characters
    pub async fn update_display_efficient<D>(
        &mut self,
        lcd: &mut Controller<D, Init>
    ) -> Result<(), hd44780_controller::controller::Error>
    where
        D: hd44780_controller::device::AsyncDevice,
    {
        let (row1_text, row2_text) = self.get_display_text().await;

        // Ensure text is exactly 16 characters, padding or truncating as needed
        let row1_chars: [char; 16] = Self::pad_or_truncate_to_16(&row1_text);
        let row2_chars: [char; 16] = Self::pad_or_truncate_to_16(&row2_text);

        let new_content = [row1_chars, row2_chars];

        // If display hasn't been initialized, do a full write
        if !self.display_initialized {
            // Clear display and write everything
            lcd.clear().await?;

            // Write row 1
            lcd.set_cursor_position(0, 0).await?;
            for &c in &new_content[0] {
                lcd.write_char(c).await?;
            }

            // Write row 2
            lcd.set_cursor_position(1, 0).await?;
            for &c in &new_content[1] {
                lcd.write_char(c).await?;
            }

            self.display_buffer = new_content;
            self.display_initialized = true;

            return Ok(());
        }

        // Character-level diff update
        for row in 0..2 {
            let mut col = 0;
            while col < 16 {
                if self.display_buffer[row][col] != new_content[row][col] {
                    // Found a difference, set cursor position
                    lcd.set_cursor_position(row as u8, col as u8).await?;

                    // Write consecutive changed characters to minimize cursor movements
                    let _start_col = col;
                    while col < 16 && self.display_buffer[row][col] != new_content[row][col] {
                        lcd.write_char(new_content[row][col]).await?;
                        self.display_buffer[row][col] = new_content[row][col];
                        col += 1;
                    }
                } else {
                    col += 1;
                }
            }
        }

        Ok(())
    }

    /// Helper function to pad or truncate text to exactly 16 characters
    fn pad_or_truncate_to_16(text: &str) -> [char; 16] {
        let mut result = [' '; 16];
        let mut chars_iter = text.chars();

        for i in 0..16 {
            if let Some(c) = chars_iter.next() {
                result[i] = c;
            } else {
                break;
            }
        }

        result
    }

    /// Reset display state for clean takeover by efficient update method
    pub fn reset_display_state(&mut self) {
        self.display_initialized = false;
        self.display_buffer = [[' '; 16]; 2];
    }

    // ===== Text Formatting Helper Functions =====

    /// Format time as "HH:MM" (5 chars) from Unix timestamp
    fn format_current_time(&self) -> String {
        if let Some(now) = TimeKeeper::now_local() {
            format!("{:02}:{:02}", now.hour(), now.minute())
        } else {
            "--:--".to_string()
        }
    }

    /// Get heating indicator for a boiler ('X' = heating, 'O' = not heating)
    fn get_heating_indicator(&self, boiler_index: u8) -> char {
        self.shared_state.status.get_boiler_status(boiler_index)
            .map(|status| {
                if status.output.duty_cycle() > 0 {
                    'X'
                } else {
                    'O'
                }
            })
            .unwrap_or('?')
    }

    // ===== Display Layout Formatters =====

    /// Format off mode row 1: "Off" centered
    pub fn format_off_row1(&self) -> String {
        "      Off       ".to_string()
    }

    /// Format off mode row 2: Empty
    pub fn format_off_row2(&self) -> String {
        // The 2x16 equivalent of the TFT's Off-screen hint: with the any-button-wakes rule gone,
        // this is the one place that can say how to power the machine on. Exactly 16 characters.
        "Press 3+5 for on".to_string()
    }

    /// The button menu's two rows.
    ///
    /// Only one item row fits, so the selected item is the only one drawn and no `>` marker is
    /// needed. `"Wi-Fi Setup"` is 11 characters and `"OFF"` is 3, so the `{:<12}{:>4}` split fits
    /// exactly -- which is why `menu.rs` labels it `"Wi-Fi Setup"` rather than
    /// `"Wi-Fi Provisioning"`. Both rows must stay <= 16: `pad_or_truncate_to_16` truncates
    /// silently, mid-word.
    fn menu_rows(&self) -> (String, String) {
        let Some(frame) = self.shared_state.menu.stack.top() else {
            return ("                ".to_string(), "                ".to_string());
        };
        let items = menu::items(frame.id);
        let Some(item) = items.get(frame.nav.selected()) else {
            return ("                ".to_string(), "                ".to_string());
        };

        let ctx = MenuContext::from_status(
            &self.shared_state.status,
            self.shared_state.menu.wifi_pending,
        );
        let value = menu::value_text(item, &ctx).unwrap_or("");

        // ASCII for the same reason as the TFT: the HD44780 A00 ROM has no up/down triangle
        // glyphs (U+25B2/U+25BC), and `pad_or_truncate_to_16` would push a multi-byte char
        // through `write_char` unmodified.
        (
            format!("{:<12}{:>4}", item.label, value),
            "1v 2^ 3sel 4bck".to_string(),
        )
    }

    /// Format power save standby mode row 1: "Standby" centered
    pub fn format_power_save_standby_row1(&self) -> String {
        "    Standby     ".to_string()
    }

    /// Format power save standby mode row 2: Empty
    pub fn format_power_save_standby_row2(&self) -> String {
        "                ".to_string()
    }

    /// Format brewing mode row 1: "123.4C 8.5b XO  "
    pub fn format_brewing_row1(&self) -> String {
        let brew_temp = self.shared_state.status.get_boiler_status(DualBoilerSingleGroupControllerBoilers::BrewBoiler.as_index())
            .and_then(|status| status.temperature);
        let brew_pressure = self.shared_state.status.get_boiler_status(DualBoilerSingleGroupControllerBoilers::BrewBoiler.as_index())
            .and_then(|status| status.pressure);

        let brew_heating = self.get_heating_indicator(DualBoilerSingleGroupControllerBoilers::BrewBoiler.as_index());
        let steam_heating = self.get_heating_indicator(DualBoilerSingleGroupControllerBoilers::SteamBoiler.as_index());

        let temp_str = self.shared_state.format_temperature(brew_temp);
        let pressure_str = self.shared_state.format_pressure(brew_pressure);

        format!("{} {} {}{} ", temp_str, pressure_str, brew_heating, steam_heating)
    }

    /// Format brewing mode row 2: "1.2ml/s 45.6g 32"
    pub fn format_brewing_row2(&self) -> String {
        let group_status = self.shared_state.status.get_group_status(SingleGroupControllerGroups::SingleGroup.as_index());

        let flow_rate = group_status.and_then(|status| status.input_flow_rate);
        let weight = group_status.and_then(|status| status.output_weight);
        let brew_time = group_status.and_then(|status| status.current_brew.as_ref().map(|b| b.brew_time));

        let flow_str = self.shared_state.format_flow_rate(flow_rate);
        let weight_str = self.shared_state.format_weight(weight);
        let time_str = self.shared_state.format_brew_time(brew_time);

        format!("{} {} {}", flow_str, weight_str, time_str)
    }

    /// Format standby mode row 1: "123.4C XO 12:34 "
    pub fn format_standby_row1(&self) -> String {
        let brew_temp = self.shared_state.status.get_boiler_status(DualBoilerSingleGroupControllerBoilers::BrewBoiler.as_index())
            .and_then(|status| status.temperature);

        let brew_heating = self.get_heating_indicator(DualBoilerSingleGroupControllerBoilers::BrewBoiler.as_index());
        let steam_heating = self.get_heating_indicator(DualBoilerSingleGroupControllerBoilers::SteamBoiler.as_index());

        let temp_str = self.shared_state.format_temperature(brew_temp);
        let time_str = self.format_current_time();

        format!("{} {}{} {} ", temp_str, brew_heating, steam_heating, time_str)
    }

    /// Format standby mode row 2: "2.1b 123.4C T1"
    pub fn format_standby_row2(&self) -> String {
        let steam_pressure = self.shared_state.status.get_boiler_status(DualBoilerSingleGroupControllerBoilers::SteamBoiler.as_index())
            .and_then(|status| status.pressure);

        let steam_temp = self.shared_state.status.get_boiler_status(DualBoilerSingleGroupControllerBoilers::SteamBoiler.as_index())
            .and_then(|status| status.temperature);

        let pressure_str = self.shared_state.format_pressure(steam_pressure);
        let steam_temp_str = self.shared_state.format_temperature(steam_temp);
        let tank_str = self.shared_state.format_tank_level();

        format!("{} {} {}", pressure_str, steam_temp_str, tank_str)
    }

    /// Format post-brew mode row 1: "Time: X.X s"
    pub fn format_post_brew_row1(&self) -> String {
        if let Some(group_status) = self.shared_state.status.get_group_status(SingleGroupControllerGroups::SingleGroup.as_index()) {
            if let Some(previous_brew) = &group_status.previous_brew {
                let secs = previous_brew.brew_time.as_secs();
                let subsec = previous_brew.brew_time.subsec_millis() / 100; // tenths of a second
                return format!("Time: {}.{}s", secs, subsec);
            }
        }
        "Time: --.-s".to_string()
    }

    /// Format post-brew mode row 2: "Vol:XXml Wgt:XXg"
    pub fn format_post_brew_row2(&self) -> String {
        if let Some(group_status) = self.shared_state.status.get_group_status(SingleGroupControllerGroups::SingleGroup.as_index()) {
            if let Some(previous_brew) = &group_status.previous_brew {
                let volume_str = previous_brew.brew_input_volume
                    .map(|v| format!("{:.0}ml", v))
                    .unwrap_or_else(|| "---ml".to_string());

                let weight_str = previous_brew.output_weight
                    .map(|w| format!("{:.0}g", w))
                    .unwrap_or_else(|| "---g".to_string());

                return format!("Vol:{} Wgt:{}", volume_str, weight_str);
            }
        }
        "Vol:---ml Wgt:---g".to_string()
    }

    /// Format routine execution mode row 1: "X/Y StepDesc"
    pub async fn format_routine_row1(&self) -> String {
        if let Some(routine_execution) = &self.shared_state.status.routine_execution {
            // Use cached routine if available
            if let Some(routine) = &self.current_routine {
                if let Some(current_step_idx) = routine_execution.current_step {
                    let total_steps = routine.steps().len();
                    let step_num = current_step_idx + 1;

                    // Get step description if available
                    if let Some(step) = routine.steps().get(current_step_idx as usize) {
                        if let Some(description) = step.description() {
                            // Format: "X/Y Description"
                            let prefix = format!("{}/{} ", step_num, total_steps);
                            let remaining = 16 - prefix.len();

                            if description.len() <= remaining {
                                format!("{}{}", prefix, description)
                            } else {
                                // Truncate with ".."
                                let truncate_len = remaining.saturating_sub(2);
                                format!("{}{}...", prefix, &description[..truncate_len.min(description.len())])
                            }
                        } else {
                            format!("{}/{} Step {}", step_num, total_steps, step_num)
                        }
                    } else {
                        format!("{}/{}", step_num, total_steps)
                    }
                } else {
                    "Routine starting".to_string()
                }
            } else {
                // Fallback: query repository if cache is empty (shouldn't normally happen)
                let mut routine_repo = self.routine_repository.lock().await;
                if let Some(routine) = routine_repo.get_routine(routine_execution.routine_index).await {
                    if let Some(current_step_idx) = routine_execution.current_step {
                        let step_num = current_step_idx + 1;
                        let total_steps = routine.steps().len();
                        format!("{}/{}", step_num, total_steps)
                    } else {
                        "Routine starting".to_string()
                    }
                } else {
                    "Unknown routine".to_string()
                }
            }
        } else {
            "No routine".to_string()
        }
    }

    /// Format routine execution mode row 2: Shows first exit condition with current value
    pub async fn format_routine_row2(&self) -> String {
        if let Some(routine_execution) = &self.shared_state.status.routine_execution {
            // Use cached routine if available
            if let Some(routine) = &self.current_routine {
                if let Some(current_step_idx) = routine_execution.current_step {
                    if let Some(step) = routine.steps().get(current_step_idx as usize) {
                        // Get first exit condition
                        if let Some(exit) = step.exits().first() {
                            return self.format_exit_condition_with_value(&exit.condition, Some(routine));
                        }
                    }
                }
            } else {
                // Fallback: query repository if cache is empty (shouldn't normally happen)
                let mut routine_repo = self.routine_repository.lock().await;
                if let Some(routine) = routine_repo.get_routine(routine_execution.routine_index).await {
                    if let Some(current_step_idx) = routine_execution.current_step {
                        if let Some(step) = routine.steps().get(current_step_idx as usize) {
                            // Get first exit condition
                            if let Some(exit) = step.exits().first() {
                                return self.format_exit_condition_with_value(&exit.condition, Some(routine));
                            }
                        }
                    }
                }
            }
        }
        "".to_string()
    }

    /// Format exit condition with current process value (compact for 16 chars)
    ///
    /// The lookups are `variegated_controller_lib::routine_progress`; the abbreviations are
    /// this screen's, and it has sixteen columns to say it in. `routine` supplies the
    /// derived-parameter formulas, which `Status` does not carry -- without it a derived
    /// target reads as zero, which is what this renderer did for every one of them.
    fn format_exit_condition_with_value(
        &self,
        condition: &RoutineExitCondition,
        routine: Option<&Routine>,
    ) -> String {
        use variegated_controller_types::ParameterUnit;

        // The three conditions with no numeric progress get words rather than a reading.
        let progress = match variegated_controller_lib::routine_progress::exit_condition_progress(
            condition,
            &self.shared_state.status,
            routine,
        ) {
            Some(progress) => progress,
            None => {
                return match condition {
                    RoutineExitCondition::Always => "Ready".to_string(),
                    RoutineExitCondition::Never => "Manual".to_string(),
                    RoutineExitCondition::UserAction(_) => "Press button".to_string(),
                    RoutineExitCondition::StateConditionMet(StateCondition::Brewing(_)) => {
                        "Start brewing".to_string()
                    }
                    RoutineExitCondition::StateConditionMet(StateCondition::NotBrewing(_)) => {
                        "Stop brewing".to_string()
                    }
                    _ => "Waiting...".to_string(),
                };
            }
        };

        // Sixteen columns, so temperatures and seconds lose their decimal and the rest keep
        // one. Bar is "b" rather than "bar" for the same reason.
        let (unit, decimals) = match progress.unit {
            ParameterUnit::Seconds => ("", 0),
            ParameterUnit::Celsius => ("C", 0),
            ParameterUnit::Bar => ("b", 1),
            ParameterUnit::MillilitersPerSecond => ("ml/s", 1),
            ParameterUnit::Grams => ("g", 1),
            ParameterUnit::Percent => ("%", 0),
            ParameterUnit::Milliliters => ("ml", 1),
        };

        match (progress.current, decimals) {
            (Some(current), 0) => format!("{:.0}>{:.0}{}", current, progress.target, unit),
            (Some(current), _) => format!("{:.1}>{:.1}{}", current, progress.target, unit),
            (None, 0) => format!("?>{:.0}{}", progress.target, unit),
            (None, _) => format!("?>{:.1}{}", progress.target, unit),
        }
    }
}
