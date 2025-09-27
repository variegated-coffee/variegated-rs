//! TwoBySixteenLCDDisplayController for dual boiler espresso machines
//!
//! This module provides a display controller for 2x16 character LCD displays using
//! the HD44780 controller via MCP23017 I2C GPIO expander. The display shows different
//! information based on brewing state.
//!
//! Display states:
//! - **Brewing**: Shows brew boiler temp/pressure, heating indicators, flow rate, weight, brew time
//! - **Standby**: Shows brew boiler temp, heating indicators, current time, steam pressure, weight, last brew time

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use core::time::Duration;
use defmt;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_rp::i2c::{Async, I2c};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Receiver;
use embassy_time::{Instant, Timer};
use hd44780_controller::controller::{Controller, config::{InitialConfig, RuntimeConfig}, state::Init};
use hd44780_controller::command::function_set::{DataLength, NumberOfLines, CharacterFont};
use embassy_time::Delay;
use embedded_hal_async::delay::DelayNs;
use variegated_controller_types::{
    DualBoilerSingleGroupControllerBoilers, SingleGroupControllerGroups, Status,
};
use crate::{StatusSubscriber, mcp23017_hd44780::Mcp23017HD44780Device};

/// Simple data structure for tracking display state
pub struct DisplayState {
    /// Receiver for status updates from the dual boiler controller
    status_receiver: StatusSubscriber,
    /// Current system status
    status: Status,
    /// Last completed brew time for standby display
    last_brew_time: Option<Duration>,
    /// Track previous brewing state to detect transitions
    was_brewing: bool,
    /// Last display update time for rate limiting
    last_update: Instant,
    /// Buffer tracking current display content (2 rows x 16 columns)
    display_buffer: [[char; 16]; 2],
    /// Track whether display has been initialized with content
    display_initialized: bool,
}

impl DisplayState {
    /// Create a new display state tracker
    pub fn new(status_receiver: StatusSubscriber) -> Self {
        Self {
            status_receiver,
            status: Status::default(),
            last_brew_time: None,
            was_brewing: false,
            last_update: Instant::now(),
            display_buffer: [[' '; 16]; 2],
            display_initialized: false,
        }
    }

    /// Check if an update is needed (1Hz rate limiting)
    pub fn should_update(&mut self) -> bool {
        let now = Instant::now();
        if now.saturating_duration_since(self.last_update).as_millis() >= 1000 {
            self.last_update = now;
            true
        } else {
            false
        }
    }

    /// Update status from the receiver
    pub fn update_status(&mut self) {
        if let Some(new_status) = self.status_receiver.try_next_message_pure() {
            // Check for brewing state transition to capture last brew time
            let current_brewing = self.is_currently_brewing(&new_status);
            if self.was_brewing && !current_brewing {
                // Brewing just stopped, capture the brew time
                if let Some(group_status) = new_status.get_group_status(SingleGroupControllerGroups::SingleGroup.as_index()) {
                    self.last_brew_time = group_status.brew_time;
                }
            }
            self.was_brewing = current_brewing;

            self.status = new_status;
        }
    }

    /// Check if the machine is currently brewing
    pub fn is_currently_brewing(&self, status: &Status) -> bool {
        status.get_group_status(SingleGroupControllerGroups::SingleGroup.as_index())
            .map(|group| group.is_brewing)
            .unwrap_or(false)
    }

    /// Get the formatted text for the current display state
    pub fn get_display_text(&self) -> (String, String) {
        if self.is_currently_brewing(&self.status) {
            (self.format_brewing_row1(), self.format_brewing_row2())
        } else {
            (self.format_standby_row1(), self.format_standby_row2())
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
        let (row1_text, row2_text) = self.get_display_text();

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

    /// Format temperature as "XXX.XC" (6 chars)
    pub fn format_temperature(&self, temp: Option<f32>) -> String {
        match temp {
            Some(t) => format!("{:5.1}C", t),
            None => "-----C".to_string(),
        }
    }

    /// Format pressure as "X.Xb" (4 chars)
    fn format_pressure(&self, pressure: Option<f32>) -> String {
        match pressure {
            Some(p) => format!("{:3.1}b", p),
            None => "---b".to_string(),
        }
    }

    /// Format flow rate as "X.Xml/s" (7 chars)
    fn format_flow_rate(&self, flow: Option<f32>) -> String {
        match flow {
            Some(f) => format!("{:3.1}ml/s", f),
            None => "---ml/s".to_string(),
        }
    }

    /// Format weight as "XX.Xg" (5 chars)
    fn format_weight(&self, weight: Option<f32>) -> String {
        match weight {
            Some(w) => format!("{:4.1}g", w),
            None => "----g".to_string(),
        }
    }

    /// Format time as "HH:MM" (5 chars) from Unix timestamp
    fn format_current_time(&self) -> String {
        if let Some(comms) = &self.status.comms_status {
            if let Some(timestamp) = comms.timestamp {
                let hours = (timestamp / 3600) % 24;
                let minutes = (timestamp / 60) % 60;
                format!("{:02}:{:02}", hours, minutes)
            } else {
                "--:--".to_string()
            }
        } else {
            "--:--".to_string()
        }
    }

    /// Format brew time as seconds "XXs" (3 chars max)
    fn format_brew_time(&self, brew_time: Option<Duration>) -> String {
        match brew_time {
            Some(duration) => {
                let secs = duration.as_secs();
                if secs < 100 {
                    format!("{}s", secs)
                } else {
                    "99s".to_string() // Cap at 99s for display
                }
            }
            None => "0s".to_string(),
        }
    }

    /// Format tank level as "T0" (empty) or "T1" (non-empty) (2 chars)
    fn format_tank_level(&self) -> String {
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

    /// Format last brew time as "L:XXs" (5 chars max)
    fn format_last_brew_time(&self) -> String {
        match self.last_brew_time {
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
        self.status.get_boiler_status(boiler_index)
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

    /// Format brewing mode row 1: "123.4C 8.5b XO  "
    pub fn format_brewing_row1(&self) -> String {
        let brew_temp = self.status.get_boiler_status(DualBoilerSingleGroupControllerBoilers::BrewBoiler.as_index())
            .and_then(|status| status.temperature);
        let brew_pressure = self.status.get_boiler_status(DualBoilerSingleGroupControllerBoilers::BrewBoiler.as_index())
            .and_then(|status| status.pressure);

        let brew_heating = self.get_heating_indicator(DualBoilerSingleGroupControllerBoilers::BrewBoiler.as_index());
        let steam_heating = self.get_heating_indicator(DualBoilerSingleGroupControllerBoilers::SteamBoiler.as_index());

        let temp_str = self.format_temperature(brew_temp);
        let pressure_str = self.format_pressure(brew_pressure);

        format!("{} {} {}{} ", temp_str, pressure_str, brew_heating, steam_heating)
    }

    /// Format brewing mode row 2: "1.2ml/s 45.6g 32"
    pub fn format_brewing_row2(&self) -> String {
        let group_status = self.status.get_group_status(SingleGroupControllerGroups::SingleGroup.as_index());

        let flow_rate = group_status.and_then(|status| status.input_flow_rate);
        let weight = group_status.and_then(|status| status.output_weight);
        let brew_time = group_status.and_then(|status| status.brew_time);

        let flow_str = self.format_flow_rate(flow_rate);
        let weight_str = self.format_weight(weight);
        let time_str = self.format_brew_time(brew_time);

        format!("{} {} {}", flow_str, weight_str, time_str)
    }

    /// Format standby mode row 1: "123.4C XO 12:34 "
    pub fn format_standby_row1(&self) -> String {
        let brew_temp = self.status.get_boiler_status(DualBoilerSingleGroupControllerBoilers::BrewBoiler.as_index())
            .and_then(|status| status.temperature);

        let brew_heating = self.get_heating_indicator(DualBoilerSingleGroupControllerBoilers::BrewBoiler.as_index());
        let steam_heating = self.get_heating_indicator(DualBoilerSingleGroupControllerBoilers::SteamBoiler.as_index());

        let temp_str = self.format_temperature(brew_temp);
        let time_str = self.format_current_time();

        format!("{} {}{} {} ", temp_str, brew_heating, steam_heating, time_str)
    }

    /// Format standby mode row 2: "2.1b 123.4C T1"
    pub fn format_standby_row2(&self) -> String {
        let steam_pressure = self.status.get_boiler_status(DualBoilerSingleGroupControllerBoilers::SteamBoiler.as_index())
            .and_then(|status| status.pressure);

        let steam_temp = self.status.get_boiler_status(DualBoilerSingleGroupControllerBoilers::SteamBoiler.as_index())
            .and_then(|status| status.temperature);

        let pressure_str = self.format_pressure(steam_pressure);
        let steam_temp_str = self.format_temperature(steam_temp);
        let tank_str = self.format_tank_level();

        format!("{} {} {}", pressure_str, steam_temp_str, tank_str)
    }
}

/// Embassy task for running the LCD display controller
#[embassy_executor::task]
pub async fn lcd_display_task(
    lcd_device: Mcp23017HD44780Device<I2cDevice<'static, NoopRawMutex, I2c<'static, embassy_rp::peripherals::I2C1, Async>>, Delay>,
    status_receiver: StatusSubscriber,
) {
    // Initialize the HD44780 LCD controller configuration
    let initial_config = InitialConfig {
        data_length: DataLength::EightBit,
        lines: NumberOfLines::Two,
        font: CharacterFont::FiveByEight,
    };
    let runtime_config = RuntimeConfig::default(); // Display on, cursor off, backlight on

    // Create and initialize the controller (following main.rs pattern exactly)
    let lcd_controller = Controller::<Delay, _>::new_async(lcd_device, initial_config, runtime_config);
    let mut lcd = match lcd_controller.init().await {
        Ok(initialized_lcd) => initialized_lcd,
        Err(_) => {
            defmt::error!("Failed to initialize LCD controller");
            return;
        }
    };

    // Create display state tracker
    let mut display_state = DisplayState::new(status_receiver);

    // Show startup message (following main.rs pattern)
    if let Err(_) = lcd.clear().await {
        defmt::error!("Failed to clear LCD");
        return;
    }
    if let Err(_) = lcd.write_str("Dual Boiler".chars()).await {
        defmt::error!("Failed to write startup text");
        return;
    }
    if let Err(_) = lcd.write_line(1, "Starting...".chars()).await {
        defmt::error!("Failed to write startup line 2");
        return;
    }

    Timer::after(embassy_time::Duration::from_millis(2000)).await;

    // Reset display buffer state so efficient update can take over cleanly
    display_state.reset_display_state();

    defmt::info!("LCD display initialized successfully");

    // Main display loop
    loop {
        // Update status
        display_state.update_status();

        // Update display at 1Hz
        if display_state.should_update() {
            // Use efficient character-level update instead of clear-and-rewrite
            if let Err(_e) = display_state.update_display_efficient(&mut lcd).await {
                defmt::error!("Failed to update LCD efficiently");
                // Fallback: try the old method once as recovery
                let (row1, row2) = display_state.get_display_text();
                if lcd.clear().await.is_ok() {
                    let _ = lcd.write_str(row1.chars()).await;
                    let _ = lcd.write_line(1, row2.chars()).await;
                }
            }
        }

        // Small delay to prevent tight loop
        Timer::after(embassy_time::Duration::from_millis(10)).await;
    }
}