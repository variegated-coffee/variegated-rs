//! Button controller for dual boiler espresso machine
//!
//! This module provides button control functionality using MCP23017 I2C GPIO expander.
//! Handles 6 buttons with the following mappings:
//! - Button 1 (Pin 0): Toggle brewing (start/stop brewing for the single group)
//! - Button 6 (Pin 5): Toggle water dispensing (start/stop pumping to water tap)
//!
//! Features:
//! - Button debouncing to prevent spurious triggers
//! - State tracking via status subscription to determine toggle actions
//! - Integration with the machine command system

use alloc::format;
use core::time::Duration;
use defmt;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_rp::i2c::{Async, I2c};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Sender;
use embassy_time::{Instant, Timer};
use variegated_controller_types::{
    MachineCommand, SingleGroupControllerGroups, Status,
};
use variegated_mcp23017::Mcp23017;
use crate::StatusSubscriber;

/// Button debounce time in milliseconds
const DEBOUNCE_TIME_MS: u64 = 50;

/// Number of buttons on the controller
const NUM_BUTTONS: usize = 6;

/// Button indices for specific functions
const BREWING_BUTTON: usize = 0;      // Button 1 (Pin 0)
const WATER_TAP_BUTTON: usize = 5;    // Button 6 (Pin 5)

/// Button state tracking for debouncing and toggle logic
pub struct ButtonState {
    /// Current brewing state (from status subscription)
    brewing_active: bool,
    /// Current water dispensing state (from status subscription)
    water_dispensing_active: bool,
    /// Previous button states for edge detection (bit-packed)
    last_button_states: u8,
    /// Last debounce time for each button
    last_debounce_time: [Instant; NUM_BUTTONS],
    /// Debounced button states
    debounced_states: [bool; NUM_BUTTONS],
}

impl ButtonState {
    /// Create a new button state tracker
    pub fn new() -> Self {
        let now = Instant::now();
        Self {
            brewing_active: false,
            water_dispensing_active: false,
            last_button_states: 0xFF, // All buttons released (active low with pullups)
            last_debounce_time: [now; NUM_BUTTONS],
            debounced_states: [false; NUM_BUTTONS], // false = not pressed
        }
    }

    /// Update status from the status receiver
    pub fn update_status(&mut self, status: &Status) {
        // Update brewing state from status
        self.brewing_active = status.get_group_status(SingleGroupControllerGroups::SingleGroup.as_index())
            .map(|group| group.is_brewing)
            .unwrap_or(false);

        // Update water dispensing state from status
        self.water_dispensing_active = status.get_water_tap_status(0)
            .map(|water_tap| water_tap.is_dispensing)
            .unwrap_or(false);
    }

    /// Update button states with debouncing
    /// Returns true if any button state changed after debouncing
    pub fn update_buttons(&mut self, raw_button_states: u8) -> bool {
        let now = Instant::now();
        let mut state_changed = false;

        for i in 0..NUM_BUTTONS {
            let button_bit = 1 << i;
            let current_pressed = (raw_button_states & button_bit) == 0; // Active low
            let last_pressed = (self.last_button_states & button_bit) == 0;

            // Check if button state has changed
            if current_pressed != last_pressed {
                // State changed, update debounce timer
                self.last_debounce_time[i] = now;
            }

            // Check if enough time has passed for debouncing
            if now.saturating_duration_since(self.last_debounce_time[i]).as_millis() >= DEBOUNCE_TIME_MS {
                // Update debounced state if it differs
                if self.debounced_states[i] != current_pressed {
                    self.debounced_states[i] = current_pressed;
                    state_changed = true;
                }
            }
        }

        self.last_button_states = raw_button_states;
        state_changed
    }

    /// Check if a button was just pressed (rising edge after debouncing)
    pub fn is_button_just_pressed(&self, button_index: usize) -> bool {
        if button_index >= NUM_BUTTONS {
            return false;
        }
        self.debounced_states[button_index]
    }

    /// Get the appropriate command for button 1 (brewing toggle)
    pub fn get_brewing_toggle_command(&self) -> MachineCommand {
        let group_index = SingleGroupControllerGroups::SingleGroup.as_index();
        if self.brewing_active {
            MachineCommand::StopBrewing(group_index)
        } else {
            MachineCommand::StartBrewing(group_index)
        }
    }

    /// Get the appropriate command for button 6 (water tap toggle)
    pub fn get_water_tap_toggle_command(&self) -> MachineCommand {
        let water_tap_index = 0; // Single water tap at index 0
        if self.water_dispensing_active {
            MachineCommand::StopPumpingToWaterTap(water_tap_index)
        } else {
            MachineCommand::StartPumpingToWaterTap(water_tap_index)
        }
    }
}

/// Embassy task for running the button controller
#[embassy_executor::task]
pub async fn button_controller_task(
    mut mcp23017: Mcp23017<I2cDevice<'static, NoopRawMutex, I2c<'static, embassy_rp::peripherals::I2C1, Async>>, embassy_time::Delay>,
    command_sender: Sender<'static, embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, MachineCommand, 10>,
    mut status_receiver: StatusSubscriber,
) {
    let mut button_state = ButtonState::new();

    defmt::info!("Button controller task started");

    // Main button polling loop
    loop {
        // Update status if available
        if let Some(new_status) = status_receiver.try_next_message_pure() {
            button_state.update_status(&new_status);
        }

        // Read button states from MCP23017 port A
        match mcp23017.read_port_a().await {
            Ok(button_states) => {
                // Update button states with debouncing
                if button_state.update_buttons(button_states) {
                    // Check for button presses and send appropriate commands

                    // Button 1: Brewing toggle
                    if button_state.is_button_just_pressed(BREWING_BUTTON) {
                        let command = button_state.get_brewing_toggle_command();
                        defmt::info!("Button 1 pressed - sending brewing command: {:?}", command);

                        if let Err(_) = command_sender.try_send(command) {
                            defmt::warn!("Failed to send brewing command - channel full");
                        }
                    }

                    // Button 6: Water tap toggle
                    if button_state.is_button_just_pressed(WATER_TAP_BUTTON) {
                        let command = button_state.get_water_tap_toggle_command();
                        defmt::info!("Button 6 pressed - sending water tap command: {:?}", command);

                        if let Err(_) = command_sender.try_send(command) {
                            defmt::warn!("Failed to send water tap command - channel full");
                        }
                    }

                    // Log other button presses for debugging
                    for i in 0..NUM_BUTTONS {
                        if i != BREWING_BUTTON && i != WATER_TAP_BUTTON && button_state.is_button_just_pressed(i) {
                            defmt::info!("Button {} pressed (not mapped)", i + 1);
                        }
                    }
                }
            }
            Err(e) => {
                defmt::error!("Failed to read button states: {:?}", e);
            }
        }

        // Poll at 20Hz (50ms intervals) for responsive button handling
        Timer::after(embassy_time::Duration::from_millis(50)).await;
    }
}