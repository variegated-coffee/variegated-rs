//! The GS3's front panel.
//!
//! # The pins
//!
//! MCP23017 Port A. Pins 0-5 are the six panel buttons, numbered 1-6 on the machine. Pin 6
//! is the MP paddle switch and is **masked out** -- nothing in this firmware reads it, and
//! an engaged paddle used to land in every `ButtonSet` and silently turn every press into a
//! two-button set. Pin 7 is the on-board button, acted on only under `pwm-steam-valve`,
//! where it cycles the steam valve. See `ACTIVE_BUTTON_MASK`.
//!
//! # Normal mode
//!
//! | Input | Action |
//! |---|---|
//! | Tap 1-4 | Run routine 0-3; cancel the running one |
//! | Tap 5 | Toggle brew |
//! | Tap 6 | Toggle water dispensing |
//! | Tap 5 + 3 | Toggle machine power: `On -> Off`, anything else (incl. `PowerSaveStandby`) `-> On` |
//! | Hold 5, 1.5 s | Enter menu mode |
//! | Hold 6, 3 s | `TagDoseFromScale(GroupScale(SingleGroup))` |
//!
//! `{5,3}` is the *only* power control on the panel. The old "any button while Off turns it
//! On" rule is gone -- it made every button a power button, which is exactly what makes a
//! deliberate chord worth having.
//!
//! **Dispatch is on exact button sets**, so a chord is never a superset match: `{5,3}` powers
//! the machine and does not also start a brew, and `{5,6}` does nothing at all rather than
//! doing two things.
//!
//! Both long holds are measured **from finger-down**, not from the hold event. The recognizer
//! emits `PressAndHoldStart` at `SETTLING_DELAY_MS + PRESS_AND_HOLD_THRESHOLD_MS` (550 ms)
//! after the button goes down, so that offset is subtracted in `check_long_hold` and the
//! thresholds there are the numbers a user actually experiences.
//!
//! # Menu mode
//!
//! Entered by holding button 5. It captures the **six panel buttons**; the on-board
//! steam-valve button is not one of them and keeps working, because the menu can be opened
//! mid-steam and losing valve control behind a menu is not acceptable.
//!
//! | Input | Action |
//! |---|---|
//! | Tap 1 | Selection **down**, wrapping |
//! | Tap 2 | Selection **up**, wrapping |
//! | Tap 3 | Activate |
//! | Tap 4 | Pop; popping the root leaves menu mode |
//!
//! Buttons 5 and 6 are inert here, holds included. Entry is refused while brewing,
//! dispensing or running a routine, but is **not** gated on machine mode -- provisioning a
//! machine should not require heating it. The menu's content lives in [`crate::menu`]; this
//! module owns only the input half.
//!
//! # Architecture
//!
//! 1. **Event Recognition** lives in [`variegated_buttons`], not here. It turns samples into
//!    `Press` / `PressAndHoldStart` / `PressAndHoldChange` / `PressAndHoldStop`, groups
//!    near-simultaneous presses over 50 ms, and separates press from hold at 500 ms. It is in
//!    its own crate so it can be host-tested: it takes milliseconds as a `u64`, because a test
//!    binary that links `embassy-time` without a time driver fails at link on
//!    `_embassy_time_now`.
//!
//!    A recognised hold never also emits a `Press`, which is what lets a held button 5 open
//!    the menu without also toggling the brew.
//!
//! 2. **Event Handling** (`ButtonEventHandler`): events to machine commands, and the menu's
//!    position, which this task owns because it owns input
//!
//! 3. **Main Task** (`button_controller_task`): coordinates the two, and publishes the menu
//!    over `MENU_WATCH` for the read-only display tasks

use alloc::vec;
use alloc::vec::Vec;
use defmt;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_futures::select::{select, Either};
use embassy_rp::i2c::{Async, I2c};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Sender;
use variegated_rp235x_atomic_raw_mutex::AtomicRawMutex;
use embassy_time::{Instant, Timer};
use variegated_controller_types::{
    MachineCommand, MachineMode, RoutineIndex, ScaleSelector, SingleGroupControllerGroups, Status,
};
use variegated_mcp23017::{Mcp23017, Port, InterruptMode};
use variegated_buttons::{
    ButtonEvent, ButtonEventRecognizer, ButtonSet, PRESS_AND_HOLD_THRESHOLD_MS, SETTLING_DELAY_MS,
};
use crate::StatusSubscriber;
use crate::menu::{self, GsMenu, MenuActivation, MenuContext, MenuId, MenuSender};

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

const SET_ROUTINE_0: ButtonSet = ButtonSet::from_bits(1 << ROUTINE_BUTTON_0);
const SET_ROUTINE_1: ButtonSet = ButtonSet::from_bits(1 << ROUTINE_BUTTON_1);
const SET_ROUTINE_2: ButtonSet = ButtonSet::from_bits(1 << ROUTINE_BUTTON_2);
const SET_ROUTINE_3: ButtonSet = ButtonSet::from_bits(1 << ROUTINE_BUTTON_3);
const SET_BREW: ButtonSet = ButtonSet::from_bits(1 << BREWING_BUTTON);
const SET_WATER_TAP: ButtonSet = ButtonSet::from_bits(1 << WATER_TAP_BUTTON);
/// Buttons 5 and 3 together: the only way to wake or sleep the machine from the panel.
const SET_POWER: ButtonSet =
    ButtonSet::from_bits((1 << BREWING_BUTTON) | (1 << ROUTINE_BUTTON_2));
#[cfg(feature = "pwm-steam-valve")]
const SET_STEAM_VALVE: ButtonSet = ButtonSet::from_bits(1 << ON_BOARD_BUTTON);

/// The pins `handle_press` acts on.
///
/// Pin 6 is the MP paddle switch and pin 7 is the on-board button. Both were landing in
/// every `ButtonSet`, because this masked with `0xFF` while its own comment said six
/// buttons. With `count() == 1` gating every action, an engaged paddle switch turned every
/// button press into a two-button set and dropped it in silence -- and now that chords mean
/// something, `{paddle, 5}` would be one bit away from a real one.
///
/// Nothing else in the firmware reads pin 6; the only other mention of it is its pull-up
/// below, which stays, because a floating input on an interrupt-on-change port is noise.
#[cfg(not(feature = "pwm-steam-valve"))]
const ACTIVE_BUTTON_MASK: u8 = 0b0011_1111;
/// Bit 7 is the on-board button, which cycles the steam valve in this configuration and only
/// in this one.
#[cfg(feature = "pwm-steam-valve")]
const ACTIVE_BUTTON_MASK: u8 = 0b1011_1111;

/// Read a sample from Port A, dropping the pins this build does not act on.
fn button_set_from_port_a(sample: u8) -> ButtonSet {
    ButtonSet::from_active_low(sample, ACTIVE_BUTTON_MASK)
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
    /// The projection of `Status` the menu reads, refreshed by `update_status`.
    menu_context: MenuContext,
    /// Where the menu is. Owned here rather than by the display because this task owns
    /// input, and a selection that lived on the far side of a channel would move a frame
    /// after the button that moved it.
    menu: GsMenu,
    /// When the current hold of exactly button 5 started, for the menu long hold.
    button_5_hold_start: Option<Instant>,
    /// When the current hold of exactly button 6 started, for the dose-tag long hold.
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
            menu_context: MenuContext::default(),
            menu: GsMenu::closed(),
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

        self.menu_context = MenuContext::from_status(status);

        // The menu is a full-screen takeover, and the machine can become busy underneath it --
        // a schedule can start a routine, and so can the comms processor. The busy condition is
        // not only an entry gate.
        if self.menu.is_open() && self.machine_is_busy() {
            defmt::info!("Menu: closing, the machine became busy");
            self.menu.close();
        }
    }

    /// Brewing, dispensing or running a routine. Not a mode check: the menu is reachable while
    /// the machine is Off, because provisioning it should not require heating it.
    fn machine_is_busy(&self) -> bool {
        self.brewing_active || self.water_dispensing_active || self.routine_executing
    }

    fn clear_hold_deadlines(&mut self) {
        self.button_5_hold_start = None;
        self.button_6_hold_start = None;
    }

    /// Where the menu is, for publication.
    pub fn menu_nav(&self) -> GsMenu {
        self.menu
    }

    /// Handle a button event and return the appropriate machine command
    pub fn handle_event(&mut self, event: ButtonEvent, now: Instant) -> Vec<MachineCommand> {
        // The menu captures the six panel buttons. Routed here rather than inside `handle_press`
        // so that hold events cannot leak past it: with the menu open, a hold of button 5 must
        // not re-open it and a hold of button 6 must not tag a dose.
        if self.menu.is_open() {
            return match event {
                ButtonEvent::Press(buttons) => self.handle_menu_press(buttons),
                // A hold that began before the menu opened ends up here. Both deadlines have to
                // be cleared, or a hold that straddled the transition fires out of
                // `check_long_hold`.
                _ => {
                    self.clear_hold_deadlines();
                    vec![]
                }
            };
        }

        match event {
            ButtonEvent::Press(buttons) => self.handle_press(buttons),
            ButtonEvent::PressAndHoldStart(buttons) => {
                // Exact sets, so holding {5,6} arms neither.
                if buttons == SET_BREW {
                    self.button_5_hold_start = Some(now);
                } else if buttons == SET_WATER_TAP {
                    self.button_6_hold_start = Some(now);
                }
                vec![]
            }
            // The held set changed, so it is no longer a hold of exactly one of them. Cancelled
            // rather than restarted: {6} -> {6,1} -> {6} must not accumulate into a tag.
            ButtonEvent::PressAndHoldChange { .. } => {
                self.clear_hold_deadlines();
                vec![]
            }
            ButtonEvent::PressAndHoldStop(_) => {
                self.clear_hold_deadlines();
                vec![]
            }
        }
    }

    fn handle_menu_press(&mut self, buttons: ButtonSet) -> Vec<MachineCommand> {
        let Some(frame) = self.menu.top() else { return vec![] };
        let geo = menu::geometry(frame.id);

        match buttons {
            SET_ROUTINE_0 => {
                if let Some(frame) = self.menu.top_mut() {
                    frame.nav.down(geo);
                }
                vec![]
            }
            SET_ROUTINE_1 => {
                if let Some(frame) = self.menu.top_mut() {
                    frame.nav.up(geo);
                }
                vec![]
            }
            SET_ROUTINE_2 => self.activate_selected(),
            SET_ROUTINE_3 => {
                self.menu.pop();
                vec![]
            }
            // The on-board button is not one of the six panel buttons. The menu can be opened
            // mid-steam, and losing valve control behind a menu is not acceptable.
            #[cfg(feature = "pwm-steam-valve")]
            SET_STEAM_VALVE => self.cycle_steam_valve(),
            // Buttons 5 and 6, and every chord, are inert here.
            _ => vec![],
        }
    }

    fn activate_selected(&mut self) -> Vec<MachineCommand> {
        let Some(frame) = self.menu.top() else { return vec![] };
        // `.get`, not `[..]`: the selection cannot be out of range here, but a renderer and a
        // handler reading the same table at different moments is exactly where that stops being
        // true.
        let Some(item) = menu::items(frame.id).get(frame.nav.selected()) else { return vec![] };

        match menu::activate(item, &self.menu_context) {
            MenuActivation::Command(command) => {
                defmt::info!("Menu: activated {}", item.label);
                vec![command]
            }
            MenuActivation::Pop => {
                self.menu.pop();
                vec![]
            }
        }
    }

    /// Handle a button press event
    fn handle_press(&mut self, buttons: ButtonSet) -> Vec<MachineCommand> {
        // The "any button while Off turns it On" rule is gone: it made every button on the panel
        // a power button, which is exactly what makes `{5,3}` worth having. The controller
        // already refuses `RunRoutine`, `StartBrewing` and `StartPumpingToWaterTap` while not in
        // `On` (dual_boiler_single_group.rs:1787, 1811, 1832), so a press on a cold machine costs
        // a log line and nothing else, and a mode gate here would be the same rule in two places.
        match buttons {
            SET_ROUTINE_0 => self.routine_command(ROUTINE_BUTTON_0),
            SET_ROUTINE_1 => self.routine_command(ROUTINE_BUTTON_1),
            SET_ROUTINE_2 => self.routine_command(ROUTINE_BUTTON_2),
            SET_ROUTINE_3 => self.routine_command(ROUTINE_BUTTON_3),
            SET_BREW => {
                let group_index = SingleGroupControllerGroups::SingleGroup.as_index();
                vec![if self.brewing_active {
                    MachineCommand::StopBrewing(group_index)
                } else {
                    MachineCommand::StartBrewing(group_index)
                }]
            }
            SET_WATER_TAP => vec![if self.water_dispensing_active {
                MachineCommand::StopPumpingToWaterTap(0)
            } else {
                MachineCommand::StartPumpingToWaterTap(0)
            }],
            SET_POWER => vec![MachineCommand::SetMachineMode(match self.machine_mode {
                // Anything that is not On becomes On -- `PowerSaveStandby` included, so the one
                // chord is the one way back from either resting state.
                MachineMode::On => MachineMode::Off,
                _ => MachineMode::On,
            })],
            #[cfg(feature = "pwm-steam-valve")]
            SET_STEAM_VALVE => self.cycle_steam_valve(),
            _ => vec![],
        }
    }

    fn routine_command(&self, button_idx: usize) -> Vec<MachineCommand> {
        vec![if self.routine_executing {
            MachineCommand::CancelRoutine
        } else {
            MachineCommand::RunRoutine(RoutineIndex::Function(button_idx as u32), None)
        }]
    }

    #[cfg(feature = "pwm-steam-valve")]
    fn cycle_steam_valve(&mut self) -> Vec<MachineCommand> {
        let next_state = self.steam_valve_state.next();
        defmt::info!("Steam button pressed - cycling to: {:?}", next_state);

        self.steam_valve_state = next_state;

        if matches!(next_state, SteamValveState::Off) {
            vec![MachineCommand::StopSteaming(0)]
        } else {
            vec![
                MachineCommand::StartSteaming(0),
                MachineCommand::SetSteamValveOpenness(0, next_state.to_valve_openness())
            ]
        }
    }

    /// Fire the two long holds.
    ///
    /// Button 5 held opens the menu; button 6 held tags the group scale's reading as the dose
    /// for the next shot.
    ///
    /// **Both are measured from finger-down, not from the hold event.** The deadlines are armed
    /// at `PressAndHoldStart`, which the recognizer emits `SETTLING_DELAY_MS +
    /// PRESS_AND_HOLD_THRESHOLD_MS` after the button went down, so that offset is subtracted
    /// here and the constants below are the numbers a user experiences. The hold this replaces
    /// had the same off-by-550 and nobody noticed, because nobody was timing five seconds.
    ///
    /// 1.5 s on button 5 rather than the recognizer's own 550 ms hold event, because button 5 is
    /// the *brew* button: `RecognizerState::Holding` never also emits a `Press`, so a 550 ms
    /// menu would mean a 0.6 s press opens the menu instead of starting a shot, and 0.6 s is an
    /// ordinary press for someone reaching across a machine. 3 s on button 6 because a wrong
    /// dose silently replaces one that may have been set from the app and is not correctable
    /// from the panel.
    pub fn check_long_hold(&mut self, now: Instant) -> Option<MachineCommand> {
        const MENU_HOLD_MS: u64 = 1500;
        const DOSE_TAG_HOLD_MS: u64 = 3000;
        const HOLD_EVENT_OFFSET_MS: u64 = SETTLING_DELAY_MS + PRESS_AND_HOLD_THRESHOLD_MS;

        // With the menu open the six panel buttons belong to the menu, holds included.
        if self.menu.is_open() {
            self.clear_hold_deadlines();
            return None;
        }

        if let Some(started) = self.button_5_hold_start {
            let held = now.saturating_duration_since(started).as_millis();
            if held >= MENU_HOLD_MS - HOLD_EVENT_OFFSET_MS {
                // Cleared as it fires, or this re-runs on every 10 ms poll until release.
                self.button_5_hold_start = None;
                // Checked here rather than when the deadline was armed: 1.5 s is long enough for
                // a schedule to have started a routine in the meantime.
                if self.machine_is_busy() {
                    defmt::info!("Menu: refused, the machine is busy");
                } else {
                    defmt::info!("Menu: opened");
                    self.menu = GsMenu::open(MenuId::Root);
                }
                return None;
            }
        }

        if let Some(started) = self.button_6_hold_start {
            let held = now.saturating_duration_since(started).as_millis();
            if held >= DOSE_TAG_HOLD_MS - HOLD_EVENT_OFFSET_MS {
                self.button_6_hold_start = None;
                defmt::info!("Button 6 held - tagging the dose from the group scale");
                return Some(MachineCommand::TagDoseFromScale(ScaleSelector::GroupScale(
                    SingleGroupControllerGroups::SingleGroup.as_index(),
                )));
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
    checkin: variegated_checkin::CheckinHandle,
    menu_sender: MenuSender,
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
        // Reports the body ran. This loop shares I2C1 with the LEDs and the LCD, and it is
        // the machine's only front-panel input, so a wedged expander here is the difference
        // between a working machine and one that ignores its buttons.
        checkin.good();

        let menu_before = handler.menu_nav();

        // Update status if available
        if let Some(new_status) = status_receiver.try_next_message_pure() {
            // Can close the menu, if the machine became busy underneath it.
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
            let button_set = button_set_from_port_a(raw_state);
            let now = Instant::now();

            // Update recognizer and check for events
            // Milliseconds, not an `Instant`: the recognizer is host-testable precisely
            // because it never sees an `embassy_time` type.
            if let Some(event) = recognizer.update(button_set, now.as_millis()) {
                defmt::debug!("Button event: {:?}", event);

                // Handle the event and get commands
                for command in handler.handle_event(event, now) {
                    if let Err(_) = command_sender.try_send(command) {
                        defmt::warn!("Failed to send command - channel full");
                    }
                }
            }

            // Check for long hold conditions (button 5 opens the menu, button 6 tags the dose)
            if let Some(command) = handler.check_long_hold(now) {
                if let Err(_) = command_sender.try_send(command) {
                    defmt::warn!("Failed to send long hold command - channel full");
                }
            }
        }

        // Outside the `if let Some(raw_state)` block above, so a menu closed by `update_status`
        // is published even on an iteration with no button sample. Only on change: a redundant
        // send makes every display's `try_changed()` fire for nothing, and `MenuStack: PartialEq`
        // makes the test free.
        let menu_after = handler.menu_nav();
        if menu_after != menu_before {
            menu_sender.send(menu_after);
        }
    }
}