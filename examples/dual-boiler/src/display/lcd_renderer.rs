//! LCD renderer backend for 2x16 character HD44780 displays
//!
//! This module provides rendering functionality for the 2x16 character LCD display.
//! It takes the shared DisplayState and renders appropriate content to the LCD.

use alloc::string::{String, ToString};
use alloc::format;
use chrono::Timelike;
use core::time::Duration;
use hd44780_controller::controller::{Controller, state::Init};
use variegated_controller_types::{DualBoilerSingleGroupControllerBoilers, ParameterValue, RoutineExitCondition, SingleGroupControllerGroups, StateCondition};
use variegated_timekeeping::TimeKeeper;
use variegated_controller_lib::routine::RoutineRepository;

use crate::display_state::{DisplayState, DisplayMode};
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
}

impl LcdDisplayState {
    /// Create a new LCD display state tracker
    pub fn new(routine_repository: &'static RoutineRepositoryMutex) -> Self {
        Self {
            shared_state: DisplayState::new(),
            display_buffer: [[' '; 16]; 2],
            display_initialized: false,
            routine_repository,
        }
    }

    /// Get the formatted text for the current display state
    pub async fn get_display_text(&self) -> (String, String) {
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
                    let start_col = col;
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
            format!("{:02}:{:02}", now.time().hour(), now.time().minute())
        } else {
            "--:--".to_string()
        }
    }

    /// Format last brew time as "L:XXs" (5 chars max)
    fn format_last_brew_time(&self) -> String {
        match self.shared_state.last_brew_time {
            Some(duration) => {
                let secs = duration.as_secs();
                if secs < 100 {
                    format!("L:{}s", secs)
                } else {
                    "L:99s".to_string() // Cap at 99s for display
                }
            }
            None => "L:--s".to_string(),
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
        "                ".to_string()
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
        let brew_time = group_status.and_then(|status| status.brew_time);

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
            let mut routine_repo = self.routine_repository.lock().await;
            if let Some(routine) = routine_repo.get_routine(routine_execution.routine_index as usize).await {
                if let Some(current_step_idx) = routine_execution.current_step {
                    let total_steps = routine.steps().len();
                    let step_num = current_step_idx + 1;

                    // Get step description if available
                    if let Some(step) = routine.steps().get(current_step_idx) {
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
                "Unknown routine".to_string()
            }
        } else {
            "No routine".to_string()
        }
    }

    /// Format routine execution mode row 2: Shows first exit condition with current value
    pub async fn format_routine_row2(&self) -> String {
        if let Some(routine_execution) = &self.shared_state.status.routine_execution {
            let mut routine_repo = self.routine_repository.lock().await;
            if let Some(routine) = routine_repo.get_routine(routine_execution.routine_index as usize).await {
                if let Some(current_step_idx) = routine_execution.current_step {
                    if let Some(step) = routine.steps().get(current_step_idx) {
                        // Get first exit condition
                        if let Some(exit) = step.exits().first() {
                            return self.format_exit_condition_with_value(&exit.condition, &routine_execution);
                        }
                    }
                }
            }
        }
        "".to_string()
    }

    /// Helper method to resolve a ParameterValue using resolved parameters from routine execution
    fn resolve_parameter_value(&self, param_value: &ParameterValue, routine_execution: &variegated_controller_types::RoutineExecutionStatus) -> f32 {
        match param_value {
            ParameterValue::Static(value) => *value,
            ParameterValue::Parameter(index) => {
                routine_execution.resolved_parameters.get(index).copied().unwrap_or(0.0)
            }
            ParameterValue::DerivedParameter(_) => {
                // For display purposes, derived parameters aren't directly resolved here
                0.0
            }
        }
    }

    /// Format exit condition with current process value (compact for 16 chars)
    fn format_exit_condition_with_value(&self, condition: &RoutineExitCondition, routine_execution: &variegated_controller_types::RoutineExecutionStatus) -> String {
        match condition {
            RoutineExitCondition::StateConditionMet(state_condition) => {
                match state_condition {
                    StateCondition::BoilerTemperatureAbove(boiler_idx, target) |
                    StateCondition::BoilerTemperatureBelow(boiler_idx, target) => {
                        let current = self.shared_state.status.get_boiler_status(*boiler_idx)
                            .and_then(|s| s.temperature);
                        let target_value = self.resolve_parameter_value(target, routine_execution);
                        current.map(|temp| format!("{:.0}>{:.0}C", temp, target_value))
                            .unwrap_or_else(|| format!("?>{:.0}C", target_value))
                    }
                    StateCondition::BoilerPressureAbove(boiler_idx, target) |
                    StateCondition::BoilerPressureBelow(boiler_idx, target) => {
                        let current = self.shared_state.status.get_boiler_status(*boiler_idx)
                            .and_then(|s| s.pressure);
                        let target_value = self.resolve_parameter_value(target, routine_execution);
                        current.map(|press| format!("{:.1}>{:.1}b", press, target_value))
                            .unwrap_or_else(|| format!("?>{:.1}b", target_value))
                    }
                    StateCondition::GroupInputFlowRateAbove(group_idx, target) |
                    StateCondition::GroupInputFlowRateBelow(group_idx, target) => {
                        let current = self.shared_state.status.get_group_status(*group_idx)
                            .and_then(|s| s.input_flow_rate);
                        let target_value = self.resolve_parameter_value(target, routine_execution);
                        current.map(|flow| format!("{:.1}>{:.1}ml/s", flow, target_value))
                            .unwrap_or_else(|| format!("?>{:.1}ml/s", target_value))
                    }
                    StateCondition::GroupPressureAbove(group_idx, target) |
                    StateCondition::GroupPressureBelow(group_idx, target) => {
                        let current = self.shared_state.status.get_group_status(*group_idx)
                            .and_then(|s| s.pressure);
                        let target_value = self.resolve_parameter_value(target, routine_execution);
                        current.map(|press| format!("{:.1}>{:.1}b", press, target_value))
                            .unwrap_or_else(|| format!("?>{:.1}b", target_value))
                    }
                    StateCondition::OutputWeightAbove(group_idx, target) |
                    StateCondition::OutputWeightBelow(group_idx, target) => {
                        let current = self.shared_state.status.get_group_status(*group_idx)
                            .and_then(|s| s.output_weight);
                        let target_value = self.resolve_parameter_value(target, routine_execution);
                        current.map(|weight| format!("{:.1}>{:.1}g", weight, target_value))
                            .unwrap_or_else(|| format!("?>{:.1}g", target_value))
                    }
                    StateCondition::InputVolumeAboveRelativeToStart(group_idx, target) => {
                        let target_value = self.resolve_parameter_value(target, routine_execution);
                        if let Some(group_status) = self.shared_state.status.get_group_status(*group_idx) {
                            if let Some(relative) = group_status.brew_input_volume {
                                format!("{:.1}>{:.1}ml", relative, target_value)
                            } else {
                                format!("?>{:.1}ml", target_value)
                            }
                        } else {
                            format!("?>{:.1}ml", target_value)
                        }
                    }
                    StateCondition::Brewing(_) => "Start brewing".to_string(),
                    StateCondition::NotBrewing(_) => "Stop brewing".to_string(),
                    _ => "Waiting...".to_string()
                }
            }
            RoutineExitCondition::After(param_value) => {
                let target_secs = self.resolve_parameter_value(param_value, routine_execution) as u64;
                if let Some(step_elapsed) = routine_execution.step_elapsed_time {
                    let elapsed = step_elapsed.as_secs();
                    format!("{}>{}", elapsed, target_secs)
                } else {
                    format!("Wait {}s", target_secs)
                }
            }
            RoutineExitCondition::AfterDurationRelativeToStart(param_value) => {
                let target_secs = self.resolve_parameter_value(param_value, routine_execution) as u64;
                if let Some(group_status) = self.shared_state.status.get_group_status(SingleGroupControllerGroups::SingleGroup.as_index()) {
                    if let Some(brew_time) = group_status.brew_time {
                        let elapsed = brew_time.as_secs();
                        format!("{}>{}", elapsed, target_secs)
                    } else {
                        format!("Total {}s", target_secs)
                    }
                } else {
                    format!("Total {}s", target_secs)
                }
            }
            RoutineExitCondition::Always => "Ready".to_string(),
            RoutineExitCondition::Never => "Manual".to_string(),
            RoutineExitCondition::UserAction(_) => "Press button".to_string(),
        }
    }
}
