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
use alloc::vec;
use alloc::vec::Vec;
use core::time::Duration;
use defmt;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_futures::select::{select, Either};
use embassy_rp::i2c::{Async, I2c};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Sender;
use variegated_rp235x_atomic_raw_mutex::AtomicRawMutex;
use embassy_time::{Instant, Timer};
use variegated_controller_types::{
    MachineCommand, MachineMode, RoutineIndex, SingleGroupControllerGroups, Status,
};
use variegated_mcp23017::{Mcp23017, Port, InterruptMode};
use crate::StatusSubscriber;

/// Number of buttons on the controller
const NUM_BUTTONS: usize = 8;

/// Button indices for routine control (buttons 0-3)
const ROUTINE_BUTTON_0: usize = 0;    // Button 1 (Pin 0) - Triggers routine 0
const ROUTINE_BUTTON_1: usize = 1;    // Button 2 (Pin 1) - Triggers routine 1
const ROUTINE_BUTTON_2: usize = 2;    // Button 3 (Pin 2) - Triggers routine 2
const ROUTINE_BUTTON_3: usize = 3;    // Button 4 (Pin 3) - Triggers routine 3

/// Button indices for specific functions
const BREWING_BUTTON: usize = 4;      // Button 5 (Pin 4)
const WATER_TAP_BUTTON: usize = 5;    // Button 6 (Pin 5)

const MP_PADDLE_SWITCH_BUTTON: usize = 6;

const ON_BOARD_BUTTON: usize = 7;

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
        Self(bits & 0xFF) // Mask to 8 buttons
    }

    /// Create a button set from raw GPIO state (active low)
    pub const fn from_gpio_state(state: u8) -> Self {
        // Invert bits since buttons are active low, then mask to 6 buttons
        Self(!state & 0xFF)
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

#[cfg(feature = "pwm-steam-valve")]
#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
enum SteamValveState {
    Off,
    Low,      // 25%
    Medium,   // 50%
    High,     // 75%
    Full,     // 100%
}

#[cfg(feature = "pwm-steam-valve")]
impl SteamValveState {
    fn next(&self) -> Self {
        match self {
            Self::Off => Self::Low,
            Self::Low => Self::Medium,
            Self::Medium => Self::High,
            Self::High => Self::Full,
            Self::Full => Self::Off,
        }
    }

    fn to_valve_openness(&self) -> u8 {
        match self {
            Self::Off => 0,
            Self::Low => 25,
            Self::Medium => 50,
            Self::High => 75,
            Self::Full => 100,
        }
    }

    fn from_valve_openness(openness: u8, is_steaming: bool) -> Self {
        if !is_steaming || openness == 0 {
            Self::Off
        } else if openness <= 25 {
            Self::Low
        } else if openness <= 50 {
            Self::Medium
        } else if openness <= 75 {
            Self::High
        } else {
            Self::Full
        }
    }
}

/// Button event handler - translates events to machine commands
/// Maintains machine state for toggle behavior
pub struct ButtonEventHandler {
    /// Current brewing state (from status subscription)
    brewing_active: bool,
    /// Current water dispensing state (from status subscription)
    water_dispensing_active: bool,
    /// Steam valve state
    #[cfg(feature = "pwm-steam-valve")]
    steam_valve_state: SteamValveState,
    /// Current routine execution state (from status subscription)
    routine_executing: bool,
    /// Current machine mode (from status subscription)
    machine_mode: MachineMode,
    /// Tracks when button 5 hold started (for 3-second hold to turn off)
    button_5_hold_start: Option<Instant>,
    /// Tracks when button 6 hold started (for 5-second hold to open Wi-Fi setup)
    button_6_hold_start: Option<Instant>,
}

impl ButtonEventHandler {
    /// Create a new button event handler
    pub fn new() -> Self {
        Self {
            brewing_active: false,
            water_dispensing_active: false,
            #[cfg(feature = "pwm-steam-valve")]
            steam_valve_state: SteamValveState::Off,
            routine_executing: false,
            machine_mode: MachineMode::Off,
            button_5_hold_start: None,
            button_6_hold_start: None,
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

        #[cfg(feature = "pwm-steam-valve")]
        {
            self.steam_valve_state = status.get_steam_wand_status(0)
                .map(|steam| SteamValveState::from_valve_openness(steam.valve_openness, steam.is_steaming))
                .unwrap_or(SteamValveState::Off);
        }

        // Update routine execution state from status
        self.routine_executing = status.routine_execution.is_some();

        // Update machine mode from status
        self.machine_mode = status.mode;
    }

    /// Handle a button event and return the appropriate machine command
    pub fn handle_event(&mut self, event: ButtonEvent, now: Instant) -> Vec<MachineCommand> {
        match event {
            ButtonEvent::Press(buttons) => self.handle_press(buttons),
            ButtonEvent::PressAndHoldStart(buttons) => {
                // Track button 5 hold for 3-second turn-off feature
                if buttons.contains(BREWING_BUTTON) {
                    self.button_5_hold_start = Some(now);
                    defmt::debug!("Button 5 hold started at {:?}", now);
                }
                if buttons.contains(WATER_TAP_BUTTON) {
                    self.button_6_hold_start = Some(now);
                    defmt::debug!("Button 6 hold started at {:?}", now);
                }
                vec![]
            }
            ButtonEvent::PressAndHoldChange { .. } => {
                // Could be used for special functions in the future
                vec![]
            }
            ButtonEvent::PressAndHoldStop(buttons) => {
                // Clear button 5 hold tracking when released
                if buttons.contains(BREWING_BUTTON) {
                    self.button_5_hold_start = None;
                    defmt::debug!("Button 5 hold stopped");
                }
                if buttons.contains(WATER_TAP_BUTTON) {
                    self.button_6_hold_start = None;
                    defmt::debug!("Button 6 hold stopped");
                }
                vec![]
            }
        }
    }

    /// Handle a button press event
    fn handle_press(&mut self, buttons: ButtonSet) -> Vec<MachineCommand> {
        // Check if machine is Off - any button press should turn it On
        if self.machine_mode == MachineMode::Off {
            return vec![MachineCommand::SetMachineMode(MachineMode::On)];
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
                        MachineCommand::RunRoutine(RoutineIndex::Function(button_idx as u32), None)
                    };
                    return vec![command];
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
                return vec![command];
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
                return vec![command];
            }

            // Button 7: Steam valve cycling
            #[cfg(feature = "pwm-steam-valve")]
            if buttons.contains(ON_BOARD_BUTTON) {
                let next_state = self.steam_valve_state.next();
                defmt::info!("Steam button pressed - cycling to: {:?}", next_state);

                self.steam_valve_state = next_state;

                return if matches!(next_state, SteamValveState::Off) {
                    vec![MachineCommand::StopSteaming(0)]
                } else {
                    vec![
                        MachineCommand::StartSteaming(0),
                        MachineCommand::SetSteamValveOpenness(0, next_state.to_valve_openness())
                    ]
                };
            }

        }

        // Multi-button combinations can be added here in the future
        // For example: buttons 1+6 could trigger a specific routine or function

        vec![]
    }

    /// Check for long hold conditions and return appropriate command
    ///
    /// Button 5 held >= 3 seconds turns the machine off; button 6 held >= 5 seconds opens the
    /// Improv provisioning window.
    ///
    /// Neither can also fire the button's normal press action: the recognizer emits `Press`
    /// only out of `Tracking`, and once a hold is recognised the release produces
    /// `PressAndHoldStop` and nothing else (see `RecognizerState::Holding` above).
    pub fn check_long_hold(&mut self, now: Instant) -> Option<MachineCommand> {
        const LONG_HOLD_THRESHOLD_MS: u64 = 3000; // 3 seconds
        // Longer than button 5's, because this one is reached by holding the *water tap*
        // button, and someone who wanted water and held on a moment too long should not find
        // the machine advertising itself over Bluetooth.
        const PROVISIONING_HOLD_THRESHOLD_MS: u64 = 5000; // 5 seconds
        // Five minutes. Long enough to fetch a phone and type a password, short enough that a
        // window opened by accident closes itself long before anyone would notice it was open.
        const PROVISIONING_WINDOW_MS: u32 = 300_000;

        if let Some(hold_start) = self.button_5_hold_start {
            let elapsed = now.saturating_duration_since(hold_start).as_millis();

            if elapsed >= LONG_HOLD_THRESHOLD_MS {
                // Clear the tracking to avoid repeated commands
                self.button_5_hold_start = None;
                defmt::info!("Button 5 held for {}ms - turning machine off", elapsed);
                return Some(MachineCommand::SetMachineMode(MachineMode::Off));
            }
        }

        if let Some(hold_start) = self.button_6_hold_start {
            let elapsed = now.saturating_duration_since(hold_start).as_millis();

            if elapsed >= PROVISIONING_HOLD_THRESHOLD_MS {
                // Cleared for the same reason as above: this runs on a 10 ms poll, and without
                // it the request would be re-sent a hundred times a second until release.
                self.button_6_hold_start = None;
                defmt::info!(
                    "Button 6 held for {}ms - opening the Wi-Fi provisioning window",
                    elapsed
                );
                // Deliberately not gated on `machine_mode`. Provisioning a machine should not
                // require heating it, and a hold produces no `Press`, so this cannot collide
                // with the any-button-turns-it-on rule in `handle_press`. The controller
                // refuses the request while the machine is busy, which is the check that
                // matters and is the only place that knows coffee is being made.
                return Some(MachineCommand::OpenWifiProvisioningWindow {
                    duration_ms: PROVISIONING_WINDOW_MS,
                });
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
    command_sender: Sender<'static, AtomicRawMutex, MachineCommand, 10>,
    mut status_receiver: StatusSubscriber,
) {
    let mut recognizer = ButtonEventRecognizer::new();
    let mut handler = ButtonEventHandler::new();

    defmt::info!("Button controller task started");

    mcp23017.set_pin_pullup(ROUTINE_BUTTON_0 as u8, true).await.unwrap();
    mcp23017.set_pin_pullup(ROUTINE_BUTTON_1 as u8, true).await.unwrap();
    mcp23017.set_pin_pullup(ROUTINE_BUTTON_2 as u8, true).await.unwrap();
    mcp23017.set_pin_pullup(ROUTINE_BUTTON_3 as u8, true).await.unwrap();
    mcp23017.set_pin_pullup(BREWING_BUTTON as u8, true).await.unwrap();
    mcp23017.set_pin_pullup(WATER_TAP_BUTTON as u8, true).await.unwrap();
    mcp23017.set_pin_pullup(MP_PADDLE_SWITCH_BUTTON as u8, true).await.unwrap();
    mcp23017.set_pin_pullup(ON_BOARD_BUTTON as u8, true).await.unwrap();


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

                // Handle the event and get commands
                for command in handler.handle_event(event, now) {
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