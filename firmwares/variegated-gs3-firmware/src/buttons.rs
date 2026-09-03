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
//! **Buttons 1 and 2 are marked `-` and `+` on the panel**, and that is the whole rule for
//! what they do here: 1 is *less* or *previous*, 2 is *more* or *next*. Every menu screen is
//! that one rule read against whatever it is showing.
//!
//! | Input | In a list | In a value editor |
//! |---|---|---|
//! | Tap 1 (`-`) | Selection **up**, wrapping | **Decrease** |
//! | Tap 2 (`+`) | Selection **down**, wrapping | **Increase** |
//! | Tap 3 | Activate | Confirm |
//! | Tap 4 | Pop; popping the root leaves menu mode | Cancel |
//! | Hold 3, 1.5 s | **Run the selected routine**, skipping its parameter screen | — |
//! | Hold 6, 3 s | `TagDoseFromScale` -- the same meaning it has outside the menu | (same) |
//!
//! A list is not a third direction to learn: `-` moves towards the top of it because the top
//! is the previous item, which is the same thing `-` means to a number. Each screen's hint row
//! names what its own buttons do, so nobody has to derive it.
//!
//! **The two holds are meanings the menu has, not holes in its capture.** Tapping a routine
//! deliberately never runs it -- it always opens the parameter screen, so that what a press
//! does is predictable before it is made. A hold is a separate, deliberate gesture, so it can
//! carry the shortcut without weakening that rule; see [`crate::menu::activate_hold`], which
//! also owns the two conditions under which it refuses. Button 5's hold is dropped here --
//! re-opening an open menu is meaningless -- and every other hold and chord stays inert.
//!
//! Entry is refused while brewing, dispensing or running a routine, but is **not** gated on
//! machine mode -- provisioning a machine should not require heating it. The menu's content
//! lives in [`crate::menu`]; this module owns only the input half.
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
    InputCommand, MachineCommand, MachineMode, RoutineIndex, ScaleSelector,
    SingleGroupControllerGroups, Status,
};
use variegated_mcp23017::{Mcp23017, Port, InterruptMode};
use variegated_buttons::{
    ButtonEvent, ButtonEventRecognizer, ButtonSet, PRESS_AND_HOLD_THRESHOLD_MS, SETTLING_DELAY_MS,
};
use crate::StatusSubscriber;
use crate::menu::{
    self, EditorState, GsMenu, MenuActivation, MenuContext, MenuData, MenuFetch, MenuId,
    MenuItemKind, MenuKind, MenuRow, MenuSender, MenuSnapshot, WifiRequest,
    LIST_FUNCTION_ROUTINES,
};
use variegated_controller_lib::routine::{Routine, RoutineRepository as RoutineRepositoryTrait};
use variegated_controller_lib::schedule::ScheduleStore as ScheduleStoreTrait;
use variegated_controller_types::Configuration;
use variegated_controller_types::panel::{PanelDataPoints, PanelOrigin};
// The panel origin's store is reached through the trait, like every other settings store.
use variegated_controller_lib::settings::SettingsStorage;
use variegated_machine_menu::{
    apply_schedule_change, parameter_adjustable, routine_rows, schedule_rows, ParameterValues,
    RoutineRows, ScheduleChange,
};

/// Button indices for routine control (buttons 0-3)
const ROUTINE_BUTTON_0: usize = 0;    // Button 1 (Pin 0) - Triggers routine 0
const ROUTINE_BUTTON_1: usize = 1;    // Button 2 (Pin 1) - Triggers routine 1
const ROUTINE_BUTTON_2: usize = 2;    // Button 3 (Pin 2) - Triggers routine 2
const ROUTINE_BUTTON_3: usize = 3;    // Button 4 (Pin 3) - Triggers routine 3

/// Button indices for specific functions
const BREWING_BUTTON: usize = 4;      // Button 5 (Pin 4)
const WATER_TAP_BUTTON: usize = 5;    // Button 6 (Pin 5)

/// How long a refused gesture is announced for.
///
/// The same five seconds `DISPLAY_POPUP_DURATION_MS` gives a captured dose, and deliberately
/// the same: the two are the two possible outcomes of one gesture, and an operator who has
/// learned how long the confirmation stays up should not find the refusal behaving differently.
const NOTICE_DURATION: embassy_time::Duration = embassy_time::Duration::from_millis(5000);

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
    /// A provisioning command that has been sent and not yet confirmed by a `Status`.
    ///
    /// Owned here for the same reason the menu is: this task is the one that sent it.
    wifi_request: Option<WifiRequest>,
    /// The routine list backing `MenuId::Routines`, once fetched.
    ///
    /// Fetched by the task loop when that menu opens and dropped when it closes, rather than
    /// rebuilt per iteration: the repository is RAM-cached after its first load, but the ESP
    /// transceiver holds its lock for the length of a chunked routine read, and stalling the
    /// input loop behind that would make the brew button unresponsive.
    ///
    /// `Option`, not an empty `Vec` standing in for "not yet": a machine with no custom
    /// routines has an empty list and it is a correct answer.
    routines: Option<RoutineRows>,
    /// The routine a parameter screen is open on, and which one it is.
    ///
    /// Cloned out of the repository, because the screen needs its parameter names and units
    /// on every frame and re-locking for them is what the Silvia does and should not.
    ///
    /// Doubly optional: the outer says whether the fetch has happened, the inner is its
    /// result. A routine deleted over HTTP while its screen was open must still count as
    /// fetched, or `pending_fetch` asks for it again on every iteration and takes the
    /// repository lock in the input loop, forever.
    routine: Option<(RoutineIndex, Option<Routine>)>,
    /// What has been dialled into that routine's parameters.
    values: ParameterValues,
    /// What an editor frame is editing. See [`EditorState`].
    editor: Option<EditorState>,
    /// The schedule list backing `MenuId::Schedules`, once fetched.
    ///
    /// **This task is the sole builder**, and publishes it to both displays in
    /// [`MenuConfigSnapshot`] rather than letting each side fetch its own. Two sides fetching
    /// a sparse-indexed list independently is two chances to disagree about its length, and
    /// this list changes under the user: toggling `Enabled` rewrites it.
    ///
    /// `Option`, not an empty `Vec` standing in for "not yet": a machine with no schedules has
    /// an empty list and that is a correct answer, drawn as `menu::empty_label`.
    schedules: Option<variegated_machine_menu::ScheduleRows>,
    /// A schedule rewrite the user has asked for and this task has not yet resolved.
    ///
    /// Deferred rather than turned into a command on the spot, because building the command
    /// needs the stored `ScheduleItem` and the press path is deliberately synchronous -- the
    /// store's lock must not be taken in the event handler, for the reason the routine
    /// repository's is not. Resolved in the task loop, at most one iteration later.
    pending_schedule_change: Option<(u32, ScheduleChange)>,
    /// Whether `schedules` has changed since the display tasks were last told.
    ///
    /// A toggle has to reach the panels without waiting for the next `Configuration`, which
    /// the controller republishes only every ten seconds -- and which does not currently
    /// republish on a schedule change at all.
    schedules_dirty: bool,
    /// What the menu reads out of `Configuration`, from that channel.
    ///
    /// A projection, not the `Configuration` it came from: that struct is far too large to
    /// hold on this task. It was a single `Option<f32>` -- the brew boiler's ceiling -- until
    /// the Settings menu grew rows backed by more of it.
    menu_config: crate::menu::MenuConfig,
    /// Where the panel's content sits inside the bezel's aperture.
    ///
    /// **This task owns it**, unlike everything else on this screen: it has a settings key of
    /// its own, no `MachineCommand`, and no reason to reach the controller. Loaded from flash
    /// at startup, written on confirm, and published to the display tasks on the config watch.
    ///
    /// Before flash has been read it is the shipped default, which is what the firmware has
    /// always drawn at -- so a machine that has never been trimmed never moves.
    panel_origin: PanelOrigin,
    /// Whether [`Self::panel_origin`] has changed and has not been written to flash yet.
    ///
    /// The same shape as `schedules_dirty` and for the same reason: the press is handled in a
    /// synchronous function and the store is behind an async mutex.
    panel_origin_dirty: bool,
    /// Which optional data points the routine screen may draw.
    ///
    /// Owned here for [`Self::panel_origin`]'s reasons exactly: its own settings key, no
    /// `MachineCommand`, and nothing the controller needs to know. Before flash has been read
    /// it is the shipped default, which draws everything measurable -- so a machine that has
    /// never been told otherwise shows the same panel it always did.
    panel_data_points: PanelDataPoints,
    /// Whether [`Self::panel_data_points`] has changed and has not been written to flash yet.
    panel_data_points_dirty: bool,
    /// The group scale's live reading, for the dose-capture gate.
    ///
    /// Kept here rather than reached for, like every other projection on this task: a `Status`
    /// is far too large to hold, and this is the one number the button 6 hold needs in order
    /// to know whether the gesture can work at all.
    group_weight: Option<variegated_controller_types::WeightType>,
    /// A refusal waiting to be published to the displays. See [`crate::menu::Notice`].
    notice: Option<crate::menu::Notice>,
    /// The Bluetooth associations, for the Bluetooth submenu's rows.
    ///
    /// Kept beside the projection rather than in it because it is a list of rows rather than
    /// a scalar, and `MenuConfig` is `Copy` on purpose. At most four short entries.
    bluetooth: Option<variegated_controller_types::bluetooth::BluetoothPeripheralList>,
    /// When the current hold of exactly button 3 started, for run-a-routine-from-the-list.
    ///
    /// **Armed only while the menu is open.** Outside it button 3 taps to run function
    /// routine 2, and a hold there has never meant anything; giving it a meaning in one mode
    /// only is what keeps it from colliding with that.
    button_3_hold_start: Option<Instant>,
    /// When the current hold of exactly button 5 started, for the menu long hold.
    button_5_hold_start: Option<Instant>,
    /// When the current hold of exactly button 6 started, for the dose-tag long hold.
    button_6_hold_start: Option<Instant>,
    /// The last dose seen in `Status.pending_shot_annotations`.
    ///
    /// Kept so the re-seed below can be **edge-triggered**. `Status` republishes at ~10 Hz
    /// with the same value in it, and re-seeding on presence rather than on change would
    /// overwrite a hand-dialled parameter ten times a second.
    ///
    /// A projection of the annotation block rather than a copy of it, for the reason
    /// `MenuContext` is a projection of `Status`: this task holds neither.
    pending_dose: Option<f32>,
    /// Which peripherals are answering, for deciding whether a routine can run.
    ///
    /// Kept in full, unlike `pending_dose`, because the question is per-capability and the
    /// answer needs the whole map. It is a bounded `FnvIndexMap` of sixteen small entries,
    /// which is the one part of `Status` cheap enough to hold here.
    peripheral_status: variegated_controller_types::PeripheralStatus,
    /// What this machine declares it can sense.
    ///
    /// `None` until the first menu fetch latches it from `MACHINE_DEFINITION_REF`, which is
    /// behind an async mutex where `menu_data` is sync. It never changes after boot.
    machine_definition: Option<&'static variegated_controller_types::MachineDefinition>,
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
            wifi_request: None,
            routines: None,
            routine: None,
            values: ParameterValues::default(),
            editor: None,
            schedules: None,
            pending_schedule_change: None,
            schedules_dirty: false,
            menu_config: crate::menu::MenuConfig::default(),
            panel_origin: PanelOrigin::DEFAULT,
            panel_origin_dirty: false,
            panel_data_points: PanelDataPoints::DEFAULT,
            panel_data_points_dirty: false,
            group_weight: None,
            notice: None,
            bluetooth: None,
            button_3_hold_start: None,
            button_5_hold_start: None,
            button_6_hold_start: None,
            pending_dose: None,
            peripheral_status: Default::default(),
            machine_definition: None,
        }
    }

    /// What the open menu is missing, or `None` if it has everything it needs.
    ///
    /// Called every loop iteration; it answers `None` once the fetch has landed, so a settled
    /// menu costs one comparison and no lock.
    pub fn pending_fetch(&self) -> Option<MenuFetch> {
        menu::pending_fetch(
            self.menu.top().map(|frame| frame.id),
            self.routines.is_some(),
            self.routine.as_ref().map(|(index, _)| *index),
        )
    }

    /// Hand over a fetched routine list.
    pub fn provide_routines(&mut self, routines: RoutineRows) {
        self.routines = Some(routines);
    }

    /// Hand over a fetched routine, and seed its parameters.
    ///
    /// From their defaults, except that a parameter linked to a numeric shot attribute takes
    /// the pending annotation's value instead. That is what makes "capture a dose, then open
    /// the routine" show the captured dose rather than the routine's default.
    ///
    /// `None` for a routine that has gone -- deleted over HTTP while its screen was open.
    /// The screen then has no rows at all, which is the honest rendering of it.
    pub fn provide_routine(&mut self, index: RoutineIndex, routine: Option<Routine>) {
        self.values = routine.as_ref().map(ParameterValues::from_defaults).unwrap_or_default();
        if let Some(routine) = routine.as_ref() {
            self.apply_pending_dose(routine);
        }
        self.routine = Some((index, routine));
    }

    /// Write the pending dose into every parameter linked to it.
    ///
    /// Positional, because `ParameterValues` is: the position in `routine.parameters()`, not
    /// `RoutineParameter::index`. Confusing the two has been a real bug here before.
    fn apply_pending_dose(&mut self, routine: &Routine) {
        let Some(dose) = self.pending_dose else { return };

        for (position, parameter) in routine.parameters().iter().enumerate() {
            if parameter.linked_attribute.as_ref()
                == Some(&variegated_controller_types::ShotAnnotationKey::DoseWeight)
            {
                self.values.set(position, dose);
                // The editor, if one is open on this very parameter, is reset too. Last
                // change wins, and a capture is the later change -- leaving the editor
                // showing the old number would mean confirming it silently undid the capture.
                if let Some(MenuId::EditParameter { position: editing, .. }) =
                    self.menu.top().map(|frame| frame.id)
                {
                    if editing as usize == position {
                        if let Some(EditorState::Number(editor)) = self.editor.as_mut() {
                            *editor = parameter_adjustable(parameter, dose);
                        }
                    }
                }
            }
        }
    }

    /// The menu's view of what has been fetched.
    fn menu_data(&self) -> MenuData<'_> {
        let routine = self.routine.as_ref().and_then(|(_, r)| r.as_ref());
        MenuData {
            routines: self.routines.as_ref(),
            routine,
            values: self.values,
            // Live, not latched with the routine: a scale can drop while the parameter
            // screen is open, and the Run row has to grey when it does.
            routine_runnable: routine.is_none_or(|r| {
                crate::menu::routine_runnable(r, self.machine_definition, &self.peripheral_status)
            }),
            // Live for the same reason: a scale switched off while the Scale submenu is
            // open has to grey its calibration rows.
            scale_calibration: crate::menu::scale_calibration(
                self.machine_definition,
                &self.peripheral_status,
            ),
            scale_present: crate::menu::scale_present(
                self.machine_definition,
                &self.peripheral_status,
            ),
            // Not live -- see the note at the display task's matching call.
            scale_timer: crate::menu::scale_timer(self.machine_definition),
            bluetooth: self.bluetooth.as_ref(),
            schedules: self.schedules.as_ref(),
            brew_target_unit: crate::menu::brew_target_unit(&self.menu_config),
        }
    }

    /// Drop everything fetched for a menu that is no longer open.
    ///
    /// A `Routine` clone is not small, and holding one after the menu closed would keep it
    /// for as long as the machine stays up.
    ///
    /// The schedule list goes too, so that one added or deleted from the web while the menu
    /// was shut is picked up on the next entry rather than never.
    fn release_menu_data(&mut self) {
        self.routines = None;
        self.routine = None;
        self.values = ParameterValues::default();
        self.editor = None;
        self.schedules = None;
        self.schedules_dirty = true;
    }

    /// Whether a schedule menu is open and its list has not been fetched.
    fn needs_schedules(&self) -> bool {
        self.schedules.is_none()
            && matches!(
                self.menu.top().map(|frame| frame.id),
                Some(MenuId::Schedules)
                    | Some(MenuId::ScheduleItem(_))
                    | Some(MenuId::EditScheduleTime(_))
            )
    }

    /// Hand the fetched schedule list over, and tell the displays.
    fn provide_schedules(&mut self, rows: variegated_machine_menu::ScheduleRows) {
        if self.schedules.as_ref() != Some(&rows) {
            self.schedules_dirty = true;
        }
        self.schedules = Some(rows);
    }

    /// Take the outstanding schedule rewrite, if there is one.
    fn take_pending_schedule_change(&mut self) -> Option<(u32, ScheduleChange)> {
        self.pending_schedule_change.take()
    }

    /// Whether the displays need a fresh `MenuConfigSnapshot` because the schedules moved.
    fn take_schedules_dirty(&mut self) -> bool {
        core::mem::take(&mut self.schedules_dirty)
    }

    /// The panel's trim, if it has changed since it was last written to flash.
    fn take_dirty_panel_origin(&mut self) -> Option<PanelOrigin> {
        core::mem::take(&mut self.panel_origin_dirty).then_some(self.panel_origin)
    }

    /// Seed the trim from flash, at startup.
    fn set_panel_origin(&mut self, origin: PanelOrigin) {
        self.panel_origin = origin;
    }

    /// The data-point switches, if they have changed since they were last written to flash.
    fn take_dirty_panel_data_points(&mut self) -> Option<PanelDataPoints> {
        core::mem::take(&mut self.panel_data_points_dirty).then_some(self.panel_data_points)
    }

    /// Seed the switches from flash, at startup.
    fn set_panel_data_points(&mut self, points: PanelDataPoints) {
        self.panel_data_points = points;
    }

    /// Apply a change to this task's own copy of a schedule row.
    ///
    /// **Applied locally as well as sent**, because the controller takes up to a status period
    /// to act on the command and republishes `Configuration` only every ten seconds -- and, as
    /// it happens, not at all on a schedule change. Without this the row the user just pressed
    /// would keep reading its old value until they left the menu, which reads as a dead button.
    fn apply_schedule_change_locally(&mut self, index: u32, change: ScheduleChange) {
        let Some(rows) = self.schedules.as_mut() else { return };
        let Some(row) = rows.iter_mut().find(|row| row.index == index) else { return };

        match change {
            ScheduleChange::Enabled(enabled) => row.enabled = enabled,
            ScheduleChange::Time { hour, minute } => {
                row.hour = hour;
                row.minute = minute;
            }
        }

        self.schedules_dirty = true;
    }

    /// Take what the menu needs out of a configuration.
    pub fn update_configuration(&mut self, configuration: &Configuration) {
        self.menu_config = crate::menu::MenuConfig::from_configuration(configuration);
        self.bluetooth = Some(configuration.bluetooth_peripherals.clone());
    }

    /// The same projection, for the display tasks. See `menu::MenuConfigSnapshot`.
    pub fn menu_config_snapshot(&self) -> crate::menu::MenuConfigSnapshot {
        crate::menu::MenuConfigSnapshot {
            config: self.menu_config,
            panel_origin: self.panel_origin,
            panel_data_points: self.panel_data_points,
            bluetooth: self.bluetooth.clone().unwrap_or_default(),
            schedules: self.schedules.clone().unwrap_or_default(),
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

        // A dose captured while a parameter screen is already open -- by the button-6 hold
        // below, or from the web -- moves the linked parameter under the cursor.
        //
        // **Edge-triggered.** `Status` arrives at ~10 Hz carrying the same dose every time;
        // acting on its presence rather than on its change would overwrite a hand-dialled
        // value ten times a second and make the parameter uneditable.
        let dose = status.pending_shot_annotations.dose_weight();
        if dose != self.pending_dose {
            self.pending_dose = dose;
            if let Some(routine) = self.routine.as_ref().and_then(|(_, r)| r.clone()) {
                self.apply_pending_dose(&routine);
            }
        }

        // Update machine mode from status
        self.machine_mode = status.mode;

        // Cloned rather than projected, because a prerequisite names a capability and only
        // the whole map can answer which peripheral has it. Read when a routine list is
        // rebuilt, not per frame.
        self.peripheral_status = status.peripheral_status.clone();

        // The live weight, for the dose-capture gate. `None` covers both "no scale paired"
        // and "paired and not answering", which want the same message: there is nothing to
        // weigh, go and look at the scale.
        self.group_weight = status
            .get_group_status(SingleGroupControllerGroups::SingleGroup.as_index())
            .and_then(|group| group.output_weight);

        // Read `improv` first, then let it retire an outstanding request: a change in either
        // direction is the confirmation we were waiting for.
        let improv = MenuContext::from_status(
            status,
            false,
            self.menu_config,
            self.panel_origin,
            self.panel_data_points,
        )
        .improv;
        if let Some(request) = self.wifi_request {
            if !request.is_outstanding(improv, Instant::now()) {
                self.wifi_request = None;
            }
        }
        self.menu_context = MenuContext::from_status(
            status,
            self.wifi_request.is_some(),
            self.menu_config,
            self.panel_origin,
            self.panel_data_points,
        );

        // The menu is a full-screen takeover, and the machine can become busy underneath it --
        // a schedule can start a routine, and so can the comms processor. The busy condition is
        // not only an entry gate.
        //
        // This also covers a routine started *from* the menu, but it is not what closes it:
        // `activate_selected` closes first and sends second, so the menu is gone before the
        // command leaves rather than up to a status period later.
        if self.menu.is_open() && self.machine_is_busy() {
            defmt::info!("Menu: closing, the machine became busy");
            self.close_menu();
        }
    }

    /// Close the menu and drop everything that was fetched for it.
    fn close_menu(&mut self) {
        self.menu.close();
        self.release_menu_data();
    }

    /// Brewing, dispensing or running a routine. Not a mode check: the menu is reachable while
    /// the machine is Off, because provisioning it should not require heating it.
    fn machine_is_busy(&self) -> bool {
        self.brewing_active || self.water_dispensing_active || self.routine_executing
    }

    fn clear_hold_deadlines(&mut self) {
        self.button_3_hold_start = None;
        self.button_5_hold_start = None;
        self.button_6_hold_start = None;
    }

    /// Retire an outstanding provisioning request that nothing is going to confirm.
    ///
    /// Called every loop iteration rather than only when a `Status` arrives, because the
    /// deadline exists precisely for the case where no `Status` ever reports the change --
    /// `update_status` would be the one place guaranteed not to run.
    pub fn tick(&mut self, now: Instant) {
        if let Some(request) = self.wifi_request {
            // A closed menu draws nothing, so a request outliving it would only surface as a
            // stale "..." on the next entry.
            if !self.menu.is_open() || !request.is_outstanding(self.menu_context.improv, now) {
                self.wifi_request = None;
                self.menu_context.wifi_pending = false;
            }
        }
    }

    /// Where the menu is and what it is waiting for, for publication.
    pub fn menu_snapshot(&self) -> MenuSnapshot {
        MenuSnapshot {
            stack: self.menu,
            wifi_pending: self.wifi_request.is_some(),
            editor: self.editor,
            values: self.values,
            notice: self.notice,
        }
    }

    /// Open the menu, if the machine is in a state to have one opened.
    ///
    /// Shared by the button-5 hold and by [`InputCommand::Menu`] from a Bluetooth input
    /// device, so that a dial cannot reach a menu the panel would have refused. The busy
    /// check is deliberately here rather than at either call site: it is checked when the
    /// menu is actually opened, because 1.5 s is long enough for a schedule to have started
    /// a routine since the gesture began.
    pub fn open_menu(&mut self) {
        if self.machine_is_busy() {
            defmt::info!("Menu: refused, the machine is busy");
        } else {
            defmt::info!("Menu: opened");
            self.menu = GsMenu::open(MenuId::Root);
        }
    }

    /// Handle a UI command from an input device the comms processor owns.
    ///
    /// Synthesizes the panel event that means the same thing and feeds it to
    /// [`Self::handle_event`], rather than acting on the menu directly. That is what makes
    /// a dial mean on each screen exactly what the panel means on it -- including on the
    /// idle screen, where `-` and `+` run routines 0 and 1, because that is what those
    /// buttons do there. Reimplementing the dispatch here would be a second UI that drifts
    /// from the first.
    ///
    /// Nothing in here asks whether the menu is open, and nothing should: the panel's
    /// vocabulary is the same on every screen, and which screen is showing is
    /// `handle_event`'s business.
    pub fn handle_input_command(
        &mut self,
        command: InputCommand,
        now: Instant,
    ) -> Vec<MachineCommand> {
        let (buttons, repeats) = match command {
            // Buttons 1 and 2 are marked `-` and `+` on the panel, which is the whole
            // mapping: less/previous and more/next.
            InputCommand::Decrement(steps) => (SET_ROUTINE_0, steps),
            InputCommand::Increment(steps) => (SET_ROUTINE_1, steps),
            InputCommand::Activate => (SET_ROUTINE_2, 1),
            InputCommand::Return => (SET_ROUTINE_3, 1),
            InputCommand::Menu => {
                // Not a press: the menu opens on a *hold*, which is why this command exists
                // at all. Synthesizing a press of button 5 would toggle the brew instead.
                self.open_menu();
                return Vec::new();
            }
            InputCommand::Custom(index) => {
                // Reserved, and carried this far so that giving it a meaning later is not a
                // wire-format change. Logged so a key that does nothing is still visibly
                // arriving rather than looking like a dead device.
                defmt::info!("Input: custom {} is not bound to anything", index);
                return Vec::new();
            }
        };

        // One event per step. The far side batches a turn into a count precisely so this
        // link carries one message instead of many -- but the menu moves one row per press,
        // so the count has to be spent here.
        let mut commands = Vec::new();
        for _ in 0..repeats {
            commands.extend(self.handle_event(ButtonEvent::Press(buttons), now));
        }
        commands
    }

    /// Handle a button event and return the appropriate machine command
    pub fn handle_event(&mut self, event: ButtonEvent, now: Instant) -> Vec<MachineCommand> {
        // The menu captures the six panel buttons. Routed here rather than inside `handle_press`
        // so that hold events cannot leak past it: with the menu open, a hold of button 5 must
        // not re-open it.
        //
        // **Two holds mean something here, and both are meanings the menu has rather than
        // holes in the capture.**
        //
        // Button 6 is "capture a dose", on every screen. That is exactly what a user is doing
        // while standing at a routine's parameter screen, and requiring them to back out of
        // the menu to do it -- then come back in, by which point the screen has re-seeded --
        // is a worse gesture than the one that already exists.
        //
        // Button 3 is "run the selected routine", and only on a row that is one. It is the
        // same button that selects, which is the point: hold what you would have pressed, and
        // skip the parameter screen. See `menu::activate_hold` for why the shortcut is a hold
        // rather than a second row, and for the two gates it still applies.
        //
        // Every *other* hold and chord stays inert here, which is what `handle_menu_press`
        // and `check_long_hold` still enforce.
        if self.menu.is_open() {
            return match event {
                ButtonEvent::Press(buttons) => self.handle_menu_press(buttons),
                ButtonEvent::PressAndHoldStart(buttons) if buttons == SET_WATER_TAP => {
                    self.button_6_hold_start = Some(now);
                    vec![]
                }
                ButtonEvent::PressAndHoldStart(buttons) if buttons == SET_ROUTINE_2 => {
                    self.button_3_hold_start = Some(now);
                    vec![]
                }
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

        // An editor frame has no rows, and buttons 1 and 2 move the value instead of the
        // selection. 1 and 2 still mean "previous / next", whether what they step through is a
        // list, a number or a pair of clock fields.
        match frame.id.kind() {
            MenuKind::NumberEditor => return self.handle_editor_press(buttons),
            MenuKind::TimeEditor => return self.handle_time_editor_press(buttons),
            MenuKind::List => {}
        }

        // `geometry` borrows the fetched data; take the value out before touching `self` again.
        let geo = menu::geometry(frame.id, &self.menu_data());

        match buttons {
            SET_ROUTINE_0 => {
                if let Some(frame) = self.menu.top_mut() {
                    frame.nav.up(geo);
                }
                vec![]
            }
            SET_ROUTINE_1 => {
                if let Some(frame) = self.menu.top_mut() {
                    frame.nav.down(geo);
                }
                vec![]
            }
            SET_ROUTINE_2 => self.activate_selected(),
            SET_ROUTINE_3 => {
                self.pop_menu();
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

    /// Buttons on an editor frame: 1 decreases, 2 increases, 3 confirms, 4 cancels.
    ///
    /// 1 and 2 are doing exactly what they do in a list -- the panel marks them `-` and `+`,
    /// and *less* applied to a number is a smaller number the same way *previous* applied to a
    /// list is the row above. Nothing is inverted between the two screens; see the module docs.
    ///
    /// 3 and 4 do change, from Select/Back to Confirm/Cancel, because leaving an editor
    /// without committing is a real choice here rather than the only one.
    fn handle_editor_press(&mut self, buttons: ButtonSet) -> Vec<MachineCommand> {
        let Some(frame) = self.menu.top() else { return vec![] };
        let id = frame.id;

        match buttons {
            SET_ROUTINE_0 => {
                if let Some(EditorState::Number(editor)) = self.editor.as_mut() {
                    editor.decrease();
                }
                vec![]
            }
            SET_ROUTINE_1 => {
                if let Some(EditorState::Number(editor)) = self.editor.as_mut() {
                    editor.increase();
                }
                vec![]
            }
            SET_ROUTINE_2 => {
                let Some(value) = self.editor.and_then(EditorState::number).map(|e| e.value())
                else {
                    return vec![];
                };
                // A parameter's value never leaves this task until the routine runs; only the
                // brew setpoint produces a command, and the controller persists that itself.
                let command = menu::confirm_editor(id, value, &self.menu_config);
                if let MenuId::EditParameter { position, .. } = id {
                    self.values.set(position as usize, value);
                }
                // The panel's trim is this task's too, and unlike a parameter it is written to
                // flash. Not from here, which is not async: the flag is what the task loop
                // acts on, exactly as `schedules_dirty` is. Applied locally in the same breath
                // so the row and the panel do not disagree in the frame after the press.
                match id {
                    MenuId::EditPanelOriginX => {
                        self.panel_origin.x = value as u8;
                        self.panel_origin_dirty = true;
                    }
                    MenuId::EditPanelOriginY => {
                        self.panel_origin.y = value as u8;
                        self.panel_origin_dirty = true;
                    }
                    _ => {}
                }
                defmt::info!("Menu: confirmed editor at {}", value);
                self.pop_menu();
                command.into_iter().collect()
            }
            SET_ROUTINE_3 => {
                self.pop_menu();
                vec![]
            }
            #[cfg(feature = "pwm-steam-valve")]
            SET_STEAM_VALVE => self.cycle_steam_valve(),
            _ => vec![],
        }
    }

    /// Buttons on the time editor: 1 less, 2 more, 3 switches field, **4 commits**.
    ///
    /// 1 and 2 mean what they mean everywhere else on this panel -- `-` and `+` applied to
    /// whichever half of `HH:MM` is selected.
    ///
    /// **There is no cancel, and that is the deliberate difference from every other editor
    /// here.** A time has two fields and this panel has four buttons, so button 3 is spent on
    /// switching between them -- which leaves 4 as the only button that can leave the screen.
    /// A 4 that discarded the edit would make the screen a dead end with no way to keep a
    /// change, so it commits. The hint row reads `4 Done` rather than `4 Back` for exactly
    /// that reason: this is the one screen in the menu where 4 does not mean "back".
    ///
    /// The command itself is not built here. `UpdateScheduleItem` replaces the stored item
    /// wholesale, so it needs the schedule's `on_days`, `on_date`, `once` and `commands` --
    /// which only the store has, and reading it means awaiting a lock this synchronous path
    /// must not take. The change is recorded and the task loop resolves it.
    fn handle_time_editor_press(&mut self, buttons: ButtonSet) -> Vec<MachineCommand> {
        let Some(frame) = self.menu.top() else { return vec![] };
        let MenuId::EditScheduleTime(index) = frame.id else { return vec![] };

        match buttons {
            SET_ROUTINE_0 => {
                if let Some(EditorState::Time(time)) = self.editor.as_mut() {
                    time.decrease();
                }
                vec![]
            }
            SET_ROUTINE_1 => {
                if let Some(EditorState::Time(time)) = self.editor.as_mut() {
                    time.increase();
                }
                vec![]
            }
            SET_ROUTINE_2 => {
                if let Some(EditorState::Time(time)) = self.editor.as_mut() {
                    time.next_field();
                }
                vec![]
            }
            SET_ROUTINE_3 => {
                if let Some(time) = self.editor.and_then(EditorState::time) {
                    defmt::info!(
                        "Menu: committing schedule {} at {}:{}",
                        index,
                        time.hour(),
                        time.minute()
                    );
                    self.pending_schedule_change = Some((
                        index,
                        ScheduleChange::Time { hour: time.hour(), minute: time.minute() },
                    ));
                }
                self.pop_menu();
                vec![]
            }
            #[cfg(feature = "pwm-steam-valve")]
            SET_STEAM_VALVE => self.cycle_steam_valve(),
            _ => vec![],
        }
    }

    /// Leave the current menu, clearing anything that belonged only to it.
    ///
    /// Leaving a routine's parameter screen drops the routine *and the values dialled into
    /// it*, so re-entering it starts from the routine's own defaults again. Keeping them
    /// would mean a screen that looks identical on two visits but runs different numbers, and
    /// nothing on it says which visit you are on. It also matches what selecting a routine
    /// does on the Silvia, where the edit state is built fresh every time.
    fn pop_menu(&mut self) {
        self.menu.pop();
        self.editor = None;

        if !self.menu.is_open() {
            self.release_menu_data();
        } else if !matches!(
            self.menu.top().map(|frame| frame.id),
            Some(MenuId::RoutineParameters(_)) | Some(MenuId::EditParameter { .. })
        ) {
            self.routine = None;
            self.values = ParameterValues::default();
        }
    }

    fn activate_selected(&mut self) -> Vec<MachineCommand> {
        let Some(frame) = self.menu.top() else { return vec![] };
        let (id, selected) = (frame.id, frame.nav.selected());

        // Resolve the row and decide what to do with it in one borrow, so that nothing below
        // can act on a row the data no longer has. `menu::row` returns `None` rather than
        // indexing: the selection cannot be out of range here, but a renderer and a handler
        // reading the same list at different moments is exactly where that stops being true.
        let (activation, is_wifi) = {
            let data = self.menu_data();
            let Some(row) = menu::row(id, selected, &data) else { return vec![] };
            let is_wifi = matches!(
                &row,
                MenuRow::Item(item) if item.kind == MenuItemKind::WifiProvisioning
            );
            (menu::activate(&row, &self.menu_context), is_wifi)
        };

        match activation {
            MenuActivation::Command(command) => {
                if is_wifi {
                    // The window takes about a second to open or close. Until it does, the
                    // value column reads "..." rather than the state we just asked to leave.
                    self.wifi_request =
                        Some(WifiRequest::new(self.menu_context.improv, Instant::now()));
                    self.menu_context.wifi_pending = true;
                }
                vec![command]
            }
            MenuActivation::CommandAndClose(command) => {
                // Close first, send second -- the order `RunRoutine` below uses, and for the
                // same reason: the busy gate in `update_status` would also close the menu
                // once the new mode showed up in a `Status`, but that is up to a status
                // period later.
                self.close_menu();
                vec![command]
            }
            MenuActivation::Enter(submenu) => {
                if !self.menu.push(submenu) {
                    // Only reachable if `MENU_MAX_DEPTH` stops matching the deepest path.
                    defmt::warn!("Menu: stack full, cannot enter {}", submenu);
                }
                vec![]
            }
            MenuActivation::Edit { menu: editor_menu, value } => {
                if self.menu.push(editor_menu) {
                    self.editor = Some(EditorState::Number(value));
                } else {
                    defmt::warn!("Menu: stack full, cannot edit");
                }
                vec![]
            }
            MenuActivation::EditTime { menu: editor_menu, value } => {
                if self.menu.push(editor_menu) {
                    self.editor = Some(EditorState::Time(value));
                } else {
                    defmt::warn!("Menu: stack full, cannot edit time");
                }
                vec![]
            }
            // Flipped here and now, unlike a schedule change: the whole value is three bytes
            // this task already owns, so there is no store to read before applying it. The
            // dirty flag is what defers the *write*, for the reason `panel_origin_dirty`
            // exists -- this is a synchronous function and the store is behind an async mutex.
            //
            // `menu_context` is updated too, not just the field. The context is rebuilt from
            // `Status`, which arrives at 10 Hz and knows nothing about this setting, so
            // without this the row the user just pressed would keep reading its old value for
            // up to a status period -- which is exactly the dead-button reading this menu's
            // "announce the refusal before the press" rule exists to avoid.
            MenuActivation::ToggleDataPoint(switch) => {
                switch.toggle(&mut self.panel_data_points);
                self.panel_data_points_dirty = true;
                self.menu_context.data_points = self.panel_data_points;
                vec![]
            }
            // Resolved in the task loop, which can await the store. Nothing is sent from here:
            // building the command needs the stored `ScheduleItem`, and taking the store's
            // lock in the event path is what this defers.
            MenuActivation::UpdateSchedule { index, change } => {
                self.pending_schedule_change = Some((index, change));
                vec![]
            }
            MenuActivation::RunRoutine(index) => {
                // Build the parameters *before* closing, because closing drops the routine the
                // positions are resolved against.
                let parameters = self
                    .routine
                    .as_ref()
                    .filter(|(cached, _)| *cached == index)
                    .and_then(|(_, routine)| routine.as_ref())
                    .and_then(|routine| self.values.to_runtime(routine));

                // Close first, send second. The busy gate in `update_status` would also close
                // it once the routine showed up in a `Status`, but that is up to a status
                // period after the user asked for the menu to go, and this menu is a
                // full-screen takeover sitting over a machine about to pump hot water.
                defmt::info!("Menu: running routine, closing");
                self.close_menu();
                vec![MachineCommand::RunRoutine(index, parameters)]
            }
            MenuActivation::Pop => {
                self.pop_menu();
                vec![]
            }
            MenuActivation::Refuse => {
                defmt::info!("Menu: row refused");
                vec![]
            }
        }
    }

    /// Run the routine the selection is sitting on, skipping its parameter screen.
    ///
    /// **With no parameters at all**, not with the ones a screen would have shown. The
    /// screen is what collects them, and this gesture is the choice not to open it; sending
    /// `None` lets the controller merge the routine's own defaults and seed any linked
    /// parameter from the pending shot annotations. That is the same path the four hardware
    /// buttons take, so a dose captured with button 6 beforehand still lands on the shot --
    /// which is the case that makes running without the screen useful rather than lossy.
    ///
    /// `None` from `activate_hold` is impossible to distinguish from a refusal here, and
    /// both correctly do nothing: the row is not a routine, or it is one that cannot run.
    fn run_selected_routine(&mut self) -> Option<MachineCommand> {
        let frame = self.menu.top()?;
        let index = {
            let data = self.menu_data();
            let row = menu::row(frame.id, frame.nav.selected(), &data)?;
            match menu::activate_hold(&row, &self.menu_context) {
                MenuActivation::RunRoutine(index) => index,
                _ => {
                    defmt::info!("Menu: hold to run refused");
                    return None;
                }
            }
        };

        // Close first, send second, for the reason `activate_selected`'s `RunRoutine` arm
        // gives: this menu is a full-screen takeover sitting over a machine about to pump
        // hot water.
        defmt::info!("Menu: held to run a routine, closing");
        self.close_menu();
        Some(MachineCommand::RunRoutine(index, None))
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

    /// **This is the one start surface with no prerequisite check.**
    ///
    /// Every other one greys out a routine the machine cannot run. These four buttons send
    /// unconditionally, and the controller's backstop refuses -- safely, but silently: no
    /// LED, no display, nothing but a `DebugEvent::RoutineRefused` in the log. A dead button
    /// reads as a broken machine, which is exactly what "prevent, don't refuse" exists to
    /// avoid.
    ///
    /// It is not an oversight, it is a gap with a cause. This task holds no `Routine` for a
    /// `Function` index: `LIST_FUNCTION_ROUTINES` is false, so `provide_routines` never
    /// fetches them, and looking one up here would take the routine repository lock *in the
    /// input path* -- which the fetch below is deliberately structured to avoid, because the
    /// ESP transceiver holds that lock for the length of a chunked routine read and the brew
    /// button must not queue behind one.
    ///
    /// Closing it properly means caching the four routines' prerequisites on this handler,
    /// refreshed off `ROUTINES_CHANGED` at the existing lock point rather than per press.
    /// Until then the refusal is real but invisible here.
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

        /// How long button 3 must be held on a routine row to start it without the parameter
        /// screen.
        ///
        /// The same 1.5 s as the hold that opens the menu, and deliberately so: those are the
        /// panel's two deliberate "I mean this" gestures, and a user who has learned one
        /// should not have to learn a second duration for the other. Long enough that it
        /// cannot be reached by pressing select firmly -- the recognizer already stops calling
        /// it a press at 550 ms, so the two are nowhere near each other.
        const RUN_ROUTINE_HOLD_MS: u64 = 1500;

        // With the menu open the six panel buttons belong to the menu -- holds included,
        // except the two the menu gives its own meaning: button 6 captures a dose anywhere,
        // and button 3 runs the selected routine. See `handle_event` for why those are
        // meanings the menu has rather than holes in the capture. Button 5's deadline is
        // still dropped here: re-opening an open menu is meaningless.
        if self.menu.is_open() {
            self.button_5_hold_start = None;
        } else {
            // And the converse: button 3's meaning is the menu's alone. A hold that began
            // inside the menu and outlived it must not fire against a closed one.
            self.button_3_hold_start = None;
        }

        if let Some(started) = self.button_3_hold_start {
            let held = now.saturating_duration_since(started).as_millis();
            if held >= RUN_ROUTINE_HOLD_MS - HOLD_EVENT_OFFSET_MS {
                // Cleared as it fires, or this re-runs on every 10 ms poll until release --
                // which for this one would be a routine started repeatedly.
                self.button_3_hold_start = None;
                return self.run_selected_routine();
            }
        }

        if let Some(started) = self.button_5_hold_start {
            let held = now.saturating_duration_since(started).as_millis();
            if held >= MENU_HOLD_MS - HOLD_EVENT_OFFSET_MS {
                // Cleared as it fires, or this re-runs on every 10 ms poll until release.
                self.button_5_hold_start = None;
                // Checked here rather than when the deadline was armed: 1.5 s is long enough for
                // a schedule to have started a routine in the meantime.
                self.open_menu();
                return None;
            }
        }

        if let Some(started) = self.button_6_hold_start {
            let held = now.saturating_duration_since(started).as_millis();
            if held >= DOSE_TAG_HOLD_MS - HOLD_EVENT_OFFSET_MS {
                self.button_6_hold_start = None;

                // Refused here rather than at the controller, which is the only place it can
                // be *said*. The controller applies the same rule -- see
                // `command::shot::dose_refusal` -- but a refusal there stores nothing, and
                // storing nothing produces no `Status` change for a display to notice. A
                // three-second hold that leaves the panel exactly as it was is
                // indistinguishable from a broken button.
                if let Some(refusal) =
                    variegated_controller_lib::command::shot::dose_refusal(self.group_weight)
                {
                    defmt::info!("Button 6 held - dose refused: {}", refusal.reason());
                    self.notice = Some(crate::menu::Notice {
                        refusal,
                        until: now + NOTICE_DURATION,
                    });
                    return None;
                }

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
    mut configuration_receiver: crate::ConfigurationSubscriber,
    routine_repository: &'static crate::RoutineRepositoryMutex,
    // Passed rather than read from `SCHEDULE_STORE_REF`, because this task is spawned from
    // `main_task`, which is where the store is created -- exactly as `routine_repository` is.
    // The global stays for the TFT task, which runs on core 1 and is spawned before that.
    schedule_store: &'static crate::ScheduleStoreMutex,
    // The panel's trim. Owned here rather than by the controller: it has a settings key of
    // its own, no `MachineCommand`, and nothing outside this firmware's own screens has any
    // use for it. Passed like the two stores above, and created in the same place.
    panel_origin_store: &'static crate::PanelOriginStoreMutex,
    // Which data points the routine screen may draw. Owned here for the trim's reasons, and
    // created in the same place.
    panel_data_points_store: &'static crate::PanelDataPointsStoreMutex,
    checkin: variegated_checkin::CheckinHandle,
    menu_sender: MenuSender,
    menu_config_sender: crate::menu::MenuConfigSender,
) {
    let mut recognizer = ButtonEventRecognizer::new();
    let mut handler = ButtonEventHandler::new();

    defmt::info!("Button controller task started");

    // The panel's trim, and one publish of it before anything else.
    //
    // Published here rather than left to the first `Configuration`, which is up to ten
    // seconds away: without it the panel would come up at the compiled default and then jump
    // to the trimmed position once the controller got around to republishing. A machine that
    // has been trimmed should come up trimmed.
    //
    // A read failure is the store's own "nothing stored yet", which `load_settings` reports
    // as the default -- the value the firmware has always drawn at.
    let origin = panel_origin_store
        .lock()
        .await
        .load_settings()
        .await
        .unwrap_or_default();
    defmt::info!("Panel origin: {}, {}", origin.x, origin.y);
    handler.set_panel_origin(origin);

    // The data-point switches, read the same way and for the same reason: a machine that has
    // been told to stop drawing its weight should not draw it for the first ten seconds after
    // every boot. A read failure is "nothing stored yet", which is the default -- everything
    // on.
    let data_points = panel_data_points_store
        .lock()
        .await
        .load_settings()
        .await
        .unwrap_or_default();
    handler.set_panel_data_points(data_points);

    // The last projection sent to the displays, so an unchanged republish costs nothing. It
    // starts at the trim's publish above rather than at `None`, which is what makes that
    // publish the *first* rather than one that is immediately repeated.
    let mut menu_config_published = {
        let snapshot = handler.menu_config_snapshot();
        menu_config_sender.send(snapshot.clone());
        Some(snapshot)
    };

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

        let menu_before = handler.menu_snapshot();

        // Update status if available
        if let Some(new_status) = status_receiver.try_next_message_pure() {
            // Can close the menu, if the machine became busy underneath it.
            handler.update_status(&new_status);
        }

        // What the Settings rows read: both boiler ceilings, the group's brew mode and its
        // targets, and the Bluetooth associations. The controller republishes every ten
        // seconds whether or not anything changed, so this arrives shortly after boot
        // without anything here having to ask.
        //
        // Forwarded to the display tasks as well as kept, because this task is the only
        // consumer of `Configuration` that both display tasks can reach -- one of them runs
        // on core 1 and is spawned before the configuration channel exists. Sent on change
        // only, so a ten-second republish of an unchanged configuration does not wake a
        // render loop. See `menu::MenuConfigSnapshot`.
        if let Some(configuration) = configuration_receiver.try_next_message_pure() {
            handler.update_configuration(&configuration);
            let snapshot = handler.menu_config_snapshot();
            if Some(&snapshot) != menu_config_published.as_ref() {
                menu_config_sender.send(snapshot.clone());
                menu_config_published = Some(snapshot);
            }
        }

        // The machine's own definition, latched onto the handler because `menu_data` needs it
        // every frame and is sync. It never changes after boot, so this is exact rather than
        // a cache, and the lock is taken once.
        //
        // **Outside the fetch match, not inside its `Routines` arm.** It used to sit there,
        // where it was reached only when the routines list needed fetching -- so a user who
        // opened Settings -> Scale without ever visiting Routines still had `None` here, and
        // `scale_calibration` reads that as "this machine cannot calibrate" and offers no
        // rows. The calibration rows would then appear only after an unrelated visit to the
        // routines list.
        if handler.menu.is_open() && handler.machine_definition.is_none() {
            handler.machine_definition = *crate::MACHINE_DEFINITION_REF.lock().await;
        }

        // Fetch what the open menu needs. Answers `None` for a settled menu, so this is a
        // comparison and no lock on all but the first iteration after a screen opens.
        //
        // Awaiting the repository here rather than inside the event path is deliberate: the
        // ESP transceiver holds this lock for the length of a chunked routine read, and the
        // brew button must not queue behind one.
        match handler.pending_fetch() {
            Some(MenuFetch::Routines) => {
                let definition = handler.machine_definition;
                let peripherals = handler.peripheral_status.clone();
                let mut repository = routine_repository.lock().await;
                let rows = routine_rows(
                    repository.iterate_routines_with_indices().await,
                    LIST_FUNCTION_ROUTINES,
                    |routine| crate::menu::routine_runnable(routine, definition, &peripherals),
                );
                handler.provide_routines(rows);
            }
            Some(MenuFetch::Routine(index)) => {
                let mut repository = routine_repository.lock().await;
                let routine = repository.get_routine(index).await.cloned();
                handler.provide_routine(index, routine);
            }
            None => {}
        }

        // The schedule rows, when a menu that lists them opens. Fetched here rather than in
        // the event path for the routine list's reason: this lock is held across a flash read.
        //
        // **With indices**, because `UpdateScheduleItem` names the storage index and those are
        // sparse -- `add_schedule` fills holes left by `remove_schedule` -- so a row's position
        // in the list is not the index it is stored under.
        if handler.needs_schedules() {
            let mut store = schedule_store.lock().await;
            let rows = schedule_rows(store.iterate_schedules_with_indices().await);
            drop(store);
            handler.provide_schedules(rows);
        }

        // A toggled `Enabled`, or a committed time. Resolved against the store rather than a
        // cached item, so a schedule whose days or actions were changed from the web a moment
        // ago is not silently rewritten back to what this panel last saw.
        if let Some((index, change)) = handler.take_pending_schedule_change() {
            let mut store = schedule_store.lock().await;
            let stored = store
                .iterate_schedules_with_indices()
                .await
                .find(|(stored_index, _)| *stored_index as u32 == index)
                .map(|(_, item)| item.clone());
            drop(store);

            match stored {
                Some(item) => {
                    let updated = apply_schedule_change(&item, change);
                    // Locally too, and in the same breath, so the row the user just pressed
                    // does not keep reading its old value while the controller catches up.
                    handler.apply_schedule_change_locally(index, change);
                    if command_sender
                        .try_send(MachineCommand::UpdateScheduleItem(index, updated))
                        .is_err()
                    {
                        defmt::warn!("Failed to send UpdateScheduleItem - channel full");
                    }
                }
                // Deleted from the web while its screen was open. Refused rather than sent,
                // since the controller would only log "index out of bounds" and the row would
                // appear to have done nothing either way.
                None => defmt::warn!("Menu: schedule {} is gone, not updating", index),
            }
        }

        // A schedule change has to reach the panels without waiting for a `Configuration` --
        // the controller republishes that only every ten seconds, and not at all on a schedule
        // command. Gated on the dirty flag rather than run every iteration, because building
        // the snapshot clones two lists and this loop runs at 100 Hz.
        if handler.take_schedules_dirty() {
            let snapshot = handler.menu_config_snapshot();
            if Some(&snapshot) != menu_config_published.as_ref() {
                menu_config_sender.send(snapshot.clone());
                menu_config_published = Some(snapshot);
            }
        }

        // The panel's trim, on confirm. One write per confirm rather than per press: a press
        // moves the picture, which the display task is already showing live from the editor's
        // value on the menu watch, and only confirming makes it survive a reboot.
        if let Some(origin) = handler.take_dirty_panel_origin() {
            let mut store = panel_origin_store.lock().await;
            if let Err(e) = store.save_settings(&origin).await {
                defmt::error!("Failed to save panel origin: {}", e);
            }
            let snapshot = handler.menu_config_snapshot();
            if Some(&snapshot) != menu_config_published.as_ref() {
                menu_config_sender.send(snapshot.clone());
                menu_config_published = Some(snapshot);
            }
        }

        // The data-point switches, on the press that flipped one. There is no confirm step to
        // wait for -- the row acts in place, like a Bluetooth toggle -- so the press is the
        // commit, and a flash write per press is what a toggle costs.
        if let Some(points) = handler.take_dirty_panel_data_points() {
            let mut store = panel_data_points_store.lock().await;
            if let Err(e) = store.save_settings(&points).await {
                defmt::error!("Failed to save panel data points: {}", e);
            }
            let snapshot = handler.menu_config_snapshot();
            if Some(&snapshot) != menu_config_published.as_ref() {
                menu_config_sender.send(snapshot.clone());
                menu_config_published = Some(snapshot);
            }
        }

        // Every iteration, not only the ones with a button sample: a provisioning command
        // that is refused outright is never confirmed by any `Status`, and its deadline is
        // the only thing that retires it.
        handler.tick(Instant::now());

        // UI commands from a Bluetooth input device, drained rather than selected on.
        //
        // The loop below already wakes at least every 10 ms, so a non-blocking drain here
        // costs a dial at most that much latency -- imperceptible against a gesture -- and
        // keeps this task's single `select` reading as "the panel", which is what it is
        // about. Widening it to a `select3` would mean a third arm that has nothing to do
        // with the MCP23017 either arm is there to read.
        //
        // Drained to empty rather than one per iteration: a fast turn arrives as one
        // message carrying several steps, and `handle_input_command` spends them all, so
        // leaving any queued would show up as the menu still scrolling after the dial has
        // stopped.
        while let Ok(command) = crate::INPUT_COMMAND_CHANNEL.try_receive() {
            defmt::debug!("Input command: {:?}", command);
            for machine_command in handler.handle_input_command(command, Instant::now()) {
                if command_sender.try_send(machine_command).is_err() {
                    defmt::warn!("Failed to send command - channel full");
                }
            }
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

            // Check for long hold conditions: button 5 opens the menu, button 6 tags the dose,
            // and button 3 runs the selected routine from inside it.
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
        let menu_after = handler.menu_snapshot();
        if menu_after != menu_before {
            menu_sender.send(menu_after);
        }
    }
}