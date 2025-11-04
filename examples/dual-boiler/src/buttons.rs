//! Button controller for dual boiler espresso machine
//!
//! This module provides button control functionality using MCP23017 I2C GPIO expander.
//! Handles 6 buttons with the following mappings:
//! - Buttons 1-4 (Pins 0-3): Trigger/cancel routines 0-3
//!   - When no routine is running: starts the corresponding routine
//!   - When a routine is running: any of these buttons cancels it
//! - Button 5 (Pin 4): Toggle brewing (start/stop brewing for the single group)
//! - Button 6 (Pin 5): Toggle water dispensing (start/stop pumping to water tap)
//!
//! # Architecture
//!
//! The button controller is split into three main components:
//!
//! 1. **Event Recognition** (`ButtonEventRecognizer`): Recognizes button events from raw GPIO state
//!    - Detects: Press, PressAndHoldStart, PressAndHoldChange, PressAndHoldStop
//!    - 50ms settling delay to group simultaneous button presses
//!    - 500ms threshold to distinguish press from hold
//!    - Press events are sent immediately on release (no artificial delay)
//!
//! 2. **Event Handling** (`ButtonEventHandler`): Translates events to machine commands
//!    - Maintains machine state for toggle behavior
//!    - Maps button events to appropriate machine commands
//!    - Supports single and multi-button combinations
//!
//! 3. **Main Task** (`button_controller_task`): Coordinates the components
//!    - Interrupt-driven button state reading
//!    - Updates recognizer with current state
//!    - Handles events and sends commands
//!
//! # Features
//!
//! - No debouncing - relies solely on hardware interrupts
//! - Simultaneous button press detection (50ms grouping window)
//! - Press-and-hold detection (500ms threshold)
//! - Button combination changes during hold
//! - State tracking via status subscription for toggle behavior
//! - Integration with the machine command system

use alloc::format;
use core::time::Duration;
use defmt;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_futures::select::{select, Either};
use embassy_rp::i2c::{Async, I2c};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Sender;
use embassy_time::{Instant, Timer};
use variegated_controller_types::{
    MachineCommand, MachineMode, RoutineIndex, SingleGroupControllerGroups, Status,
};
use variegated_mcp23017::{Mcp23017, Port, InterruptMode};
use crate::StatusSubscriber;

/// Number of buttons on the controller
const NUM_BUTTONS: usize = 6;

/// Button indices for routine control (buttons 0-3)
const ROUTINE_BUTTON_0: usize = 0;    // Button 1 (Pin 0) - Triggers routine 0
const ROUTINE_BUTTON_1: usize = 1;    // Button 2 (Pin 1) - Triggers routine 1
const ROUTINE_BUTTON_2: usize = 2;    // Button 3 (Pin 2) - Triggers routine 2
const ROUTINE_BUTTON_3: usize = 3;    // Button 4 (Pin 3) - Triggers routine 3

/// Button indices for specific functions
const BREWING_BUTTON: usize = 4;      // Button 5 (Pin 4)
const WATER_TAP_BUTTON: usize = 5;    // Button 6 (Pin 5)

/// Timing constants for event recognition
const SETTLING_DELAY_MS: u64 = 50;           // Time to group simultaneous button presses
const PRESS_AND_HOLD_THRESHOLD_MS: u64 = 500; // Time to distinguish press from hold

// ============================================================================
// Event Types
// ============================================================================

/// Represents a set of buttons as a bit-packed u8
/// Each bit corresponds to a button index (0-5)
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ButtonSet(u8);

impl ButtonSet {
    /// Create an empty button set
    pub const fn new() -> Self {
        Self(0)
    }

    /// Create a button set from raw bits
    pub const fn from_bits(bits: u8) -> Self {
        Self(bits & 0x3F) // Mask to 6 buttons
    }

    /// Create a button set from raw GPIO state (active low)
    pub const fn from_gpio_state(state: u8) -> Self {
        // Invert bits since buttons are active low, then mask to 6 buttons
        Self(!state & 0x3F)
    }

    /// Check if a specific button is in the set
    pub const fn contains(&self, button_index: usize) -> bool {
        if button_index >= NUM_BUTTONS {
            return false;
        }
        (self.0 & (1 << button_index)) != 0
    }

    /// Add a button to the set
    pub fn insert(&mut self, button_index: usize) {
        if button_index < NUM_BUTTONS {
            self.0 |= 1 << button_index;
        }
    }

    /// Remove a button from the set
    pub fn remove(&mut self, button_index: usize) {
        if button_index < NUM_BUTTONS {
            self.0 &= !(1 << button_index);
        }
    }

    /// Check if the set is empty
    pub const fn is_empty(&self) -> bool {
        self.0 == 0
    }

    /// Get the raw bits
    pub const fn bits(&self) -> u8 {
        self.0
    }

    /// Count the number of buttons in the set
    pub const fn count(&self) -> u32 {
        self.0.count_ones()
    }
}

/// Button events recognized by the event recognizer
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum ButtonEvent {
    /// Short press of button(s) - emitted on release if held <500ms
    Press(ButtonSet),
    /// Started holding button(s) - emitted at 500ms mark
    PressAndHoldStart(ButtonSet),
    /// Changed buttons while holding
    PressAndHoldChange { old: ButtonSet, new: ButtonSet },
    /// Released held button(s)
    PressAndHoldStop(ButtonSet),
}

// ============================================================================
// Event Recognition
// ============================================================================

/// State of the button event recognizer
#[derive(Debug, Clone, Copy)]
enum RecognizerState {
    /// No buttons pressed
    Idle,
    /// Buttons just pressed, waiting to group simultaneous presses
    Settling { buttons: ButtonSet, since: Instant },
    /// Tracking whether it becomes a press or hold
    Tracking { buttons: ButtonSet, since: Instant },
    /// Confirmed hold (≥500ms)
    Holding { buttons: ButtonSet },
}

/// Recognizes button events from raw button state changes
/// Separates event recognition from event handling
pub struct ButtonEventRecognizer {
    state: RecognizerState,
}

impl ButtonEventRecognizer {
    /// Create a new button event recognizer
    pub fn new() -> Self {
        Self {
            state: RecognizerState::Idle,
        }
    }

    /// Update the recognizer with current button state and time
    /// Returns an optional event if one should be emitted
    pub fn update(&mut self, current_buttons: ButtonSet, now: Instant) -> Option<ButtonEvent> {
        match self.state {
            RecognizerState::Idle => {
                if !current_buttons.is_empty() {
                    // Buttons pressed, enter settling state
                    self.state = RecognizerState::Settling {
                        buttons: current_buttons,
                        since: now,
                    };
                }
                None
            }

            RecognizerState::Settling { buttons, since } => {
                let elapsed = now.saturating_duration_since(since).as_millis();

                // Check if settling period has elapsed
                if elapsed >= SETTLING_DELAY_MS {
                    if current_buttons.is_empty() {
                        // Buttons were pressed and released during settling - emit Press
                        self.state = RecognizerState::Idle;
                        return Some(ButtonEvent::Press(buttons));
                    } else {
                        // Buttons still held after settling - move to tracking
                        self.state = RecognizerState::Tracking {
                            buttons: current_buttons,
                            since,
                        };
                        return None;
                    }
                }

                // Still within settling period
                if current_buttons != buttons {
                    // Button set changed during settling, restart settling period
                    self.state = RecognizerState::Settling {
                        buttons: current_buttons,
                        since: now,
                    };
                }
                None
            }

            RecognizerState::Tracking { buttons, since } => {
                if current_buttons.is_empty() {
                    // Released before hold threshold - it's a press!
                    self.state = RecognizerState::Idle;
                    return Some(ButtonEvent::Press(buttons));
                }

                if current_buttons != buttons {
                    // Button set changed during tracking - restart from settling
                    self.state = RecognizerState::Settling {
                        buttons: current_buttons,
                        since: now,
                    };
                    return None;
                }

                // Check if hold threshold has been reached
                let total_time = now.saturating_duration_since(since).as_millis();
                if total_time >= (SETTLING_DELAY_MS + PRESS_AND_HOLD_THRESHOLD_MS) {
                    // It's a hold!
                    self.state = RecognizerState::Holding { buttons };
                    return Some(ButtonEvent::PressAndHoldStart(buttons));
                }
                None
            }

            RecognizerState::Holding { buttons } => {
                if current_buttons.is_empty() {
                    // Released from hold
                    self.state = RecognizerState::Idle;
                    return Some(ButtonEvent::PressAndHoldStop(buttons));
                }

                if current_buttons != buttons {
                    // Button set changed during hold
                    let old_buttons = buttons;
                    self.state = RecognizerState::Holding {
                        buttons: current_buttons,
                    };
                    return Some(ButtonEvent::PressAndHoldChange {
                        old: old_buttons,
                        new: current_buttons,
                    });
                }
                None
            }
        }
    }
}

// ============================================================================
// Event Handling
// ============================================================================

/// Button event handler - translates events to machine commands
/// Maintains machine state for toggle behavior
pub struct ButtonEventHandler {
    /// Current brewing state (from status subscription)
    brewing_active: bool,
    /// Current water dispensing state (from status subscription)
    water_dispensing_active: bool,
    /// Current routine execution state (from status subscription)
    routine_executing: bool,
    /// Current machine mode (from status subscription)
    machine_mode: MachineMode,
    /// Tracks when button 5 hold started (for 3-second hold to turn off)
    button_5_hold_start: Option<Instant>,
}

impl ButtonEventHandler {
    /// Create a new button event handler
    pub fn new() -> Self {
        Self {
            brewing_active: false,
            water_dispensing_active: false,
            routine_executing: false,
            machine_mode: MachineMode::Off,
            button_5_hold_start: None,
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

        // Update routine execution state from status
        self.routine_executing = status.routine_execution.is_some();

        // Update machine mode from status
        self.machine_mode = status.mode;
    }

    /// Handle a button event and return the appropriate machine command
    pub fn handle_event(&mut self, event: ButtonEvent, now: Instant) -> Option<MachineCommand> {
        match event {
            ButtonEvent::Press(buttons) => self.handle_press(buttons),
            ButtonEvent::PressAndHoldStart(buttons) => {
                // Track button 5 hold for 3-second turn-off feature
                if buttons.contains(BREWING_BUTTON) {
                    self.button_5_hold_start = Some(now);
                    defmt::debug!("Button 5 hold started at {:?}", now);
                }
                None
            }
            ButtonEvent::PressAndHoldChange { .. } => {
                // Could be used for special functions in the future
                None
            }
            ButtonEvent::PressAndHoldStop(buttons) => {
                // Clear button 5 hold tracking when released
                if buttons.contains(BREWING_BUTTON) {
                    self.button_5_hold_start = None;
                    defmt::debug!("Button 5 hold stopped");
                }
                None
            }
        }
    }

    /// Handle a button press event
    fn handle_press(&self, buttons: ButtonSet) -> Option<MachineCommand> {
        // Check if machine is Off - any button press should turn it On
        if self.machine_mode == MachineMode::Off {
            return Some(MachineCommand::SetMachineMode(MachineMode::On));
        }

        // Single button presses for routine control (buttons 0-3)
        if buttons.count() == 1 {
            for button_idx in [ROUTINE_BUTTON_0, ROUTINE_BUTTON_1, ROUTINE_BUTTON_2, ROUTINE_BUTTON_3] {
                if buttons.contains(button_idx) {
                    let command = if self.routine_executing {
                        defmt::info!("Button {} pressed - cancelling routine", button_idx + 1);
                        MachineCommand::CancelRoutine
                    } else {
                        defmt::info!("Button {} pressed - starting routine {}", button_idx + 1, button_idx);
                        MachineCommand::RunRoutine(RoutineIndex::Function(button_idx), None)
                    };
                    return Some(command);
                }
            }

            // Button 5: Brewing toggle
            if buttons.contains(BREWING_BUTTON) {
                let group_index = SingleGroupControllerGroups::SingleGroup.as_index();
                let command = if self.brewing_active {
                    MachineCommand::StopBrewing(group_index)
                } else {
                    MachineCommand::StartBrewing(group_index)
                };
                defmt::info!("Button 5 pressed - sending brewing command: {:?}", command);
                return Some(command);
            }

            // Button 6: Water tap toggle
            if buttons.contains(WATER_TAP_BUTTON) {
                let water_tap_index = 0;
                let command = if self.water_dispensing_active {
                    MachineCommand::StopPumpingToWaterTap(water_tap_index)
                } else {
                    MachineCommand::StartPumpingToWaterTap(water_tap_index)
                };
                defmt::info!("Button 6 pressed - sending water tap command: {:?}", command);
                return Some(command);
            }
        }

        // Multi-button combinations can be added here in the future
        // For example: buttons 1+6 could trigger a specific routine or function

        None
    }

    /// Check for long hold conditions and return appropriate command
    /// Currently checks for button 5 held >= 3 seconds to turn machine off
    pub fn check_long_hold(&mut self, now: Instant) -> Option<MachineCommand> {
        const LONG_HOLD_THRESHOLD_MS: u64 = 3000; // 3 seconds

        if let Some(hold_start) = self.button_5_hold_start {
            let elapsed = now.saturating_duration_since(hold_start).as_millis();

            if elapsed >= LONG_HOLD_THRESHOLD_MS {
                // Clear the tracking to avoid repeated commands
                self.button_5_hold_start = None;
                defmt::info!("Button 5 held for {}ms - turning machine off", elapsed);
                return Some(MachineCommand::SetMachineMode(MachineMode::Off));
            }
        }

        None
    }
}

/// Embassy task for running the button controller
#[embassy_executor::task]
pub async fn button_controller_task(
    mut mcp23017: Mcp23017<I2cDevice<'static, NoopRawMutex, I2c<'static, embassy_rp::peripherals::I2C1, Async>>, embassy_time::Delay>,
    mut button_interrupt: embassy_rp::gpio::Input<'static>,
    command_sender: Sender<'static, embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, MachineCommand, 10>,
    mut status_receiver: StatusSubscriber,
) {
    let mut recognizer = ButtonEventRecognizer::new();
    let mut handler = ButtonEventHandler::new();

    defmt::info!("Button controller task started");

    // Configure all button pins (0-5 on Port A) for interrupt-on-change
    if let Err(e) = mcp23017.set_port_interrupt(Port::A, InterruptMode::OnChange).await {
        defmt::error!("Failed to configure button interrupts: {:?}", e);
    } else {
        defmt::info!("Button interrupts configured successfully");
    }

    // Main button event-driven loop
    loop {
        // Update status if available
        if let Some(new_status) = status_receiver.try_next_message_pure() {
            handler.update_status(&new_status);
        }

        // Wait for either button interrupt or timeout for state machine updates
        let button_state_opt = match select(
            button_interrupt.wait_for_falling_edge(),
            Timer::after(embassy_time::Duration::from_millis(10))
        ).await {
            Either::First(_) => {
                // Interrupt fired - button state changed
                // Read interrupt capture register (clears interrupt and gives captured GPIO state)
                match mcp23017.read_interrupt_capture().await {
                    Ok(captured) => {
                        // Port A is in lower 8 bits (buttons are on pins 0-5 of Port A)
                        let port_a_state = (captured & 0xFF) as u8;
                        defmt::debug!("Button interrupt: captured state=0x{:02x}", port_a_state);
                        Some(port_a_state)
                    }
                    Err(e) => {
                        defmt::error!("Failed to read interrupt capture: {:?}", e);
                        None
                    }
                }
            }
            Either::Second(_) => {
                // Timeout - periodic check for state machine updates
                match mcp23017.read_port_a().await {
                    Ok(state) => Some(state),
                    Err(e) => {
                        defmt::error!("Failed to read button states: {:?}", e);
                        None
                    }
                }
            }
        };

        // Process button state through recognizer and handler
        if let Some(raw_state) = button_state_opt {
            let button_set = ButtonSet::from_gpio_state(raw_state);
            let now = Instant::now();

            // Update recognizer and check for events
            if let Some(event) = recognizer.update(button_set, now) {
                defmt::debug!("Button event: {:?}", event);

                // Handle the event and get optional command
                if let Some(command) = handler.handle_event(event, now) {
                    if let Err(_) = command_sender.try_send(command) {
                        defmt::warn!("Failed to send command - channel full");
                    }
                }
            }

            // Check for long hold conditions (e.g., button 5 held for 3 seconds)
            if let Some(command) = handler.check_long_hold(now) {
                if let Err(_) = command_sender.try_send(command) {
                    defmt::warn!("Failed to send long hold command - channel full");
                }
            }
        }
    }
}