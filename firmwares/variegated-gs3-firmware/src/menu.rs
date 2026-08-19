//! What the GS3's button menu *is*.
//!
//! Three layers meet here, and only the third is this file's:
//!
//! - `variegated-menu` has the navigation arithmetic -- selection, scrolling, the stack --
//!   and knows nothing about what a row means.
//! - `variegated-machine-menu` has the routine and value model -- listing routines in an
//!   agreed order, seeding and editing their parameters, turning a number and a unit into
//!   text -- shared with the Silvia, which asks all the same questions with an encoder.
//! - **This file is the content**: which menus exist, what is on them, and what activating a
//!   row does.
//!
//! It is deliberately the only copy of that content. The button task resolves an activation
//! into a `MachineCommand` and needs row counts; the two renderers need labels and value
//! text. Neither is the other's, and a copy on each side is a copy that drifts -- a selection
//! index that means row 2 on one side and row 3 on the other is a menu that runs the wrong
//! routine.

use embassy_time::{Duration, Instant};
use variegated_controller_lib::routine::{ParameterUnit, Routine, RoutineParameter};
use variegated_controller_types::{
    BoilerIndex, ImprovState, MachineCommand, MachineMode, RoutineIndex, Status, TemperatureType,
};
use variegated_machine_menu::{
    boiler_temperature_adjustable, format_value, parameter_adjustable, parameter_geometry,
    parameter_row, ParameterListChrome, ParameterRow, ParameterValues, RoutineRows, UnitStyle,
    VALUE_TEXT_LEN,
};
use variegated_menu::{Adjustable, ListGeometry, MenuStack};

/// How deep the menu stack can go.
///
/// The deepest path is `Root -> Routines -> RoutineParameters -> EditParameter`, which is
/// **exactly four**. There is no spare level: `MenuStack::push` returns `false` on a full
/// stack and changes nothing, so a fifth would read as a button that does nothing, which is
/// the failure mode `geometry`'s `wrap` exists to avoid. Add a level and raise this.
pub const MENU_MAX_DEPTH: usize = 4;

/// Rows on screen at once on the 428x168 TFT. See `render_menu`.
pub const MENU_VISIBLE_ROWS: usize = 4;

/// Whether the routines list includes `RoutineIndex::Function(0..3)`.
///
/// It does not. This machine binds those four to panel buttons 1-4, so they are already
/// reachable with one press and do not belong in a list of things to go and find. Here
/// rather than in either caller, because the button task and both renderers build the list
/// separately and a list that disagreed about its own length would put the selection on a
/// different row on each side.
pub const LIST_FUNCTION_ROUTINES: bool = false;

/// This panel's parameter-screen chrome.
///
/// No back row: button 4 pops and the hint row on every menu screen says so, and on a
/// four-row panel a row spent on "Back" is a routine the user cannot see. The Silvia draws
/// one because a rotary encoder has no dedicated back control.
const PARAMETER_CHROME: ParameterListChrome =
    ParameterListChrome { back_row: false, visible_rows: MENU_VISIBLE_ROWS, wrap: true };

/// Which menu a stack frame is showing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum MenuId {
    /// The menu a long press of button 5 opens.
    Root,
    /// Machine settings.
    Settings,
    /// Every routine in the repository, Custom first and Internal last.
    Routines,
    /// One routine's parameters, plus the row that runs it.
    ///
    /// Carries the index rather than leaving it to be recovered from the parent's selected
    /// row. A `RoutineIndex` cannot be recovered from a row number -- it is sparse and
    /// ordered differently -- and carrying it means the button task and the renderers cannot
    /// disagree about which routine is open.
    RoutineParameters(RoutineIndex),
    /// Editing one of that routine's parameters.
    EditParameter {
        /// Which routine's.
        routine: RoutineIndex,
        /// Position in `routine.parameters()`, the way `ParameterValues` is indexed. **Not**
        /// `RoutineParameter::index`; a routine's indices need not be contiguous.
        position: u8,
    },
    /// Editing the brew boiler setpoint.
    EditBrewTemperature,
}

impl MenuId {
    /// Whether this frame edits a value rather than showing a list.
    ///
    /// An editor has no rows, and buttons 1 and 2 move the *value* rather than the selection.
    pub const fn is_editor(&self) -> bool {
        matches!(self, MenuId::EditParameter { .. } | MenuId::EditBrewTemperature)
    }
}

/// What activating a fixed row does, and what its value column reads.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum MenuItemKind {
    /// Opens the settings submenu.
    OpenSettings,
    /// Opens the routines submenu.
    OpenRoutines,
    /// Opens or closes the Improv provisioning window.
    WifiProvisioning,
    /// Edits the brew boiler setpoint.
    BrewTemperature,
    /// Leaves the current menu; at the root that closes it.
    Exit,
}

/// One fixed row.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct MenuItem {
    /// Drawn on the left.
    ///
    /// **Keep it to twelve characters.** The character LCD splits its sixteen columns
    /// `{:<12}{:>4}`, and `pad_or_truncate_to_16` truncates the row silently -- from the
    /// right, so an over-long label eats the value rather than itself.
    pub label: &'static str,
    /// What it does.
    pub kind: MenuItemKind,
}

const ROOT_ITEMS: &[MenuItem] = &[
    MenuItem { label: "Settings", kind: MenuItemKind::OpenSettings },
    MenuItem { label: "Routines", kind: MenuItemKind::OpenRoutines },
    MenuItem { label: "Exit menu", kind: MenuItemKind::Exit },
];

const SETTINGS_ITEMS: &[MenuItem] = &[
    MenuItem { label: "Wi-Fi Setup", kind: MenuItemKind::WifiProvisioning },
    MenuItem { label: "Brew temp", kind: MenuItemKind::BrewTemperature },
];

/// The fixed rows of a menu, or an empty slice for one whose rows come from data.
const fn fixed_items(menu: MenuId) -> &'static [MenuItem] {
    match menu {
        MenuId::Root => ROOT_ITEMS,
        MenuId::Settings => SETTINGS_ITEMS,
        _ => &[],
    }
}

/// What the data-driven menus need in order to have rows at all.
///
/// Passed in rather than reached for, because the two sides get it from different places: the
/// button task keeps it beside the menu it owns, and each renderer caches its own copy out of
/// the routine repository. Both build it the same way, through
/// `variegated_machine_menu::routine_rows`, so the ordering rule has one implementation.
pub struct MenuData<'a> {
    /// Every routine, in display order. `None` until the list has been fetched.
    pub routines: Option<&'a RoutineRows>,
    /// The routine a `RoutineParameters` or `EditParameter` frame is about.
    ///
    /// `None` while it is still being fetched, which makes the screen briefly empty rather
    /// than wrong.
    pub routine: Option<&'a Routine>,
    /// What has been dialled into that routine's parameters so far.
    pub values: ParameterValues,
}

impl MenuData<'_> {
    /// A parameter screen's row count is its routine's parameter count. Zero without one.
    fn param_count(&self) -> usize {
        self.routine.map_or(0, |r| r.parameters().len())
    }
}

/// Data an open menu needs that only an async fetch can supply.
///
/// Named rather than fetched on the spot because the two consumers are both in the wrong
/// place to `await`: the button handler is deliberately synchronous, so the repository lock
/// stays out of the event path, and a renderer must not block a frame on it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum MenuFetch {
    /// The routine list, for [`MenuId::Routines`].
    Routines,
    /// One routine's definition, for a parameter or editor frame.
    Routine(RoutineIndex),
}

/// What the open menu is missing, given what the caller already holds.
///
/// One implementation for three callers -- the button task and both display tasks -- because
/// a renderer that decided to fetch on different terms from the task that owns the selection
/// is a renderer drawing a different list from the one being navigated.
///
/// `menu` is the top frame's id, or `None` for a closed menu. `held_routines` says whether a
/// list has been fetched (an *empty* list is a legitimate answer, so this cannot be inferred
/// from its length). `held_routine` is which routine is already loaded, if any.
pub fn pending_fetch(
    menu: Option<MenuId>,
    held_routines: bool,
    held_routine: Option<RoutineIndex>,
) -> Option<MenuFetch> {
    match menu? {
        MenuId::Routines if !held_routines => Some(MenuFetch::Routines),
        MenuId::RoutineParameters(index) | MenuId::EditParameter { routine: index, .. }
            if held_routine != Some(index) =>
        {
            Some(MenuFetch::Routine(index))
        }
        _ => None,
    }
}

/// One row of a menu, resolved.
///
/// Looking a row up once and then asking it for its label, its value and its action is what
/// keeps those three from disagreeing about what row 2 is.
pub enum MenuRow<'a> {
    /// A fixed row.
    Item(&'a MenuItem),
    /// A routine in the routines list.
    Routine {
        /// What running it would run.
        index: RoutineIndex,
        /// Its name, already truncated to a sane length.
        name: &'a str,
    },
    /// One of the open routine's parameters.
    Parameter {
        /// Whose. Carried so activation can name the editor frame it pushes.
        routine: RoutineIndex,
        /// Position in `routine.parameters()`.
        position: usize,
        /// Its declaration, for the name and the unit.
        param: &'a RoutineParameter,
        /// What is currently dialled in.
        value: f32,
    },
    /// The row at the bottom of a parameter screen that runs the routine.
    Run(RoutineIndex),
}

/// How many rows a menu has, including every piece of chrome.
pub fn row_count(menu: MenuId, data: &MenuData) -> usize {
    match menu {
        MenuId::Root | MenuId::Settings => fixed_items(menu).len(),
        MenuId::Routines => data.routines.map_or(0, |rows| rows.len()),
        // The parameters, then Run. A routine with no parameters still gets the screen, and
        // it is one row long -- see `activate`.
        MenuId::RoutineParameters(_) => data.param_count() + 1,
        // An editor has no rows.
        MenuId::EditParameter { .. } | MenuId::EditBrewTemperature => 0,
    }
}

/// The list geometry for a menu, so the button task and the renderers cannot disagree.
pub fn geometry(menu: MenuId, data: &MenuData) -> ListGeometry {
    match menu {
        MenuId::RoutineParameters(_) => parameter_geometry(data.param_count(), PARAMETER_CHROME),
        _ => ListGeometry {
            total_rows: row_count(menu, data),
            visible_rows: MENU_VISIBLE_ROWS,
            // One physical button per direction: clamped, button 1 would do nothing on the
            // first row, and a button that does nothing reads as a broken machine.
            wrap: true,
        },
    }
}

/// The row at `index`, or `None` past the end.
pub fn row<'a>(menu: MenuId, index: usize, data: &MenuData<'a>) -> Option<MenuRow<'a>> {
    match menu {
        MenuId::Root | MenuId::Settings => fixed_items(menu).get(index).map(MenuRow::Item),
        MenuId::Routines => data
            .routines?
            .get(index)
            .map(|r| MenuRow::Routine { index: r.index, name: r.name.as_str() }),
        MenuId::RoutineParameters(routine_index) => {
            let routine = data.routine?;
            match parameter_row(index, routine.parameters().len(), PARAMETER_CHROME) {
                ParameterRow::Parameter(position) => {
                    let param = routine.parameters().get(position)?;
                    Some(MenuRow::Parameter {
                        routine: routine_index,
                        position,
                        param,
                        value: data.values.get(position).unwrap_or(param.default),
                    })
                }
                ParameterRow::Execute => Some(MenuRow::Run(routine_index)),
                // This panel draws no back row.
                ParameterRow::Back => None,
            }
        }
        MenuId::EditParameter { .. } | MenuId::EditBrewTemperature => None,
    }
}

/// What a row is drawn as, on the left.
///
/// Borrowed rather than owned: a routine's name lives in the fetched list and a parameter's
/// in the fetched routine, and copying either per frame would allocate on a render loop.
pub fn label<'a>(row: &MenuRow<'a>) -> &'a str {
    match row {
        MenuRow::Item(item) => item.label,
        MenuRow::Routine { name, .. } => name,
        MenuRow::Parameter { param, .. } => param.name.as_str(),
        MenuRow::Run(_) => "Run routine",
    }
}

/// The heading above the rows.
///
/// A routine's own name where there is one, because "Routine" above a parameter list says
/// nothing a user did not just choose.
pub fn title<'a>(menu: MenuId, data: &MenuData<'a>) -> &'a str {
    match menu {
        MenuId::Root => "Menu",
        MenuId::Settings => "Settings",
        MenuId::Routines => "Routines",
        MenuId::RoutineParameters(_) => data.routine.map_or("Routine", |r| r.name()),
        MenuId::EditParameter { position, .. } => data
            .routine
            .and_then(|r| r.parameters().get(position as usize))
            .map_or("Parameter", |p| p.name.as_str()),
        MenuId::EditBrewTemperature => "Brew temp",
    }
}

/// The unit an editor frame's value carries, for the renderers to draw beside it.
pub fn editor_unit(menu: MenuId, data: &MenuData) -> Option<ParameterUnit> {
    match menu {
        MenuId::EditBrewTemperature => Some(ParameterUnit::Celsius),
        MenuId::EditParameter { position, .. } => data
            .routine
            .and_then(|r| r.parameters().get(position as usize))
            .and_then(|p| p.unit),
        _ => None,
    }
}

/// The subset of `Status` and `Configuration` the menu reads.
///
/// A projection rather than borrowing either, because the button task deliberately keeps
/// derived values rather than copies -- a `Status` is far too large to clone onto that task
/// for one enum, and a `Configuration` for one float.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct MenuContext {
    /// Where Improv provisioning currently is.
    pub improv: ImprovState,
    /// Whether a provisioning command has been sent and `improv` has not caught up yet.
    ///
    /// Not derivable from `Status`: it says a *request* is outstanding, and only the task
    /// that sent it knows that. See `WIFI_REQUEST_TIMEOUT_MS`.
    pub wifi_pending: bool,
    /// Whether the machine is on.
    ///
    /// The menu is reachable while it is not, deliberately -- provisioning a machine should
    /// not require heating it -- but the controller refuses `RunRoutine` outright unless the
    /// mode is `On`. See `activate`.
    pub mode: MachineMode,
    /// The brew boiler's current setpoint, for the value column and to seed its editor.
    ///
    /// `None` until the first `Status` arrives.
    pub brew_target: Option<TemperatureType>,
    /// The brew boiler's configured ceiling.
    ///
    /// `None` until the first `Configuration` arrives, which the controller republishes every
    /// ten seconds whether or not it changed. Read from the machine rather than written down
    /// here: a setpoint above the interlock can only produce an element that runs to the
    /// limit and shuts off, so a ceiling this file invented would be a control problem
    /// wearing a display problem's clothes.
    pub brew_max: Option<TemperatureType>,
}

impl MenuContext {
    /// Read the projection out of a status, plus the two things a status does not carry.
    pub fn from_status(
        status: &Status,
        wifi_pending: bool,
        brew_max: Option<TemperatureType>,
    ) -> Self {
        Self {
            improv: status.comms_status.as_ref().map(|c| c.improv).unwrap_or_default(),
            wifi_pending,
            mode: status.mode,
            brew_target: status
                .get_boiler_status(BREW_BOILER)
                .map(|b| b.control_state.values.target_temperature),
            brew_max,
        }
    }

    /// Whether the brew setpoint can be edited: both ends of its range have to be known.
    fn brew_editable(&self) -> Option<(TemperatureType, TemperatureType)> {
        Some((self.brew_target?, self.brew_max?))
    }
}

/// Boiler 0. Named because `SetBoilerControlTargetValues(0, ..)` reads as a magic number.
const BREW_BOILER: BoilerIndex = 0;

/// How long the value column waits for `Status` to confirm a provisioning command.
///
/// The confirmation normally arrives in about a second, on the next `CommsStatus`. This is
/// the escape hatch for the case where it never arrives at all: the controller refuses
/// `OpenWifiProvisioningWindow` outright while the machine is busy -- steaming counts, and
/// the menu is *not* gated on steaming -- and can also drop it when the channel is full or
/// when the machine has no provisioning path. None of those reply, so without a deadline the
/// row would read "..." until the user left the menu.
///
/// Falling back to the real state is the correct reading of a refused command, so the cost
/// of this firing is nil. Five seconds rather than the ten a pure failsafe would use, at
/// Magnus's call: the recovery is worth more here than the margin.
const WIFI_REQUEST_TIMEOUT_MS: u64 = 5_000;

/// The right-hand column of a row.
///
/// Not pre-rendered text, because the two panels cannot share one rendering: every font on
/// the TFT is a u8g2 `_tr` and has no degree sign, and the character LCD has four columns for
/// a value and no room for a unit at all. Each renderer formats this with the `UnitStyle` it
/// can actually draw.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MenuValue {
    /// A fixed word.
    Text(&'static str),
    /// A quantity.
    Number {
        /// Its value.
        value: f32,
        /// Its unit, if it has one.
        unit: Option<ParameterUnit>,
    },
}

impl MenuValue {
    /// Render for a panel that can draw a unit.
    pub fn text(&self, style: UnitStyle) -> heapless::String<VALUE_TEXT_LEN> {
        match self {
            MenuValue::Text(s) => {
                let mut out = heapless::String::new();
                let _ = out.push_str(s);
                out
            }
            MenuValue::Number { value, unit } => format_value(*value, *unit, style),
        }
    }
}

/// What a row that cannot be acted on right now reads instead of its value.
///
/// Three characters, so it fits the character LCD's four-column value field.
const UNAVAILABLE: &str = "n/a";

/// The value column, or `None` for a row that has no value.
pub fn value(row: &MenuRow, ctx: &MenuContext) -> Option<MenuValue> {
    match row {
        MenuRow::Item(item) => match item.kind {
            MenuItemKind::WifiProvisioning => Some(MenuValue::Text(if ctx.wifi_pending {
                // The window takes about a second to open or close, and until it does
                // `comms_status.improv` still reports the old state. Showing that reads as
                // "nothing happened" and invites a second press.
                "..."
            } else {
                match ctx.improv {
                    ImprovState::Stopped => "OFF",
                    _ => "ON",
                }
            })),
            MenuItemKind::BrewTemperature => Some(match ctx.brew_target {
                Some(value) => MenuValue::Number { value, unit: Some(ParameterUnit::Celsius) },
                None => MenuValue::Text(UNAVAILABLE),
            }),
            MenuItemKind::OpenSettings | MenuItemKind::OpenRoutines | MenuItemKind::Exit => None,
        },
        MenuRow::Parameter { param, value, .. } => {
            Some(MenuValue::Number { value: *value, unit: param.unit })
        }
        // Say why the row will not do anything before it is pressed, rather than closing the
        // menu and leaving the machine cold. See `activate`.
        MenuRow::Run(_) if ctx.mode != MachineMode::On => Some(MenuValue::Text(UNAVAILABLE)),
        MenuRow::Run(_) | MenuRow::Routine { .. } => None,
    }
}

/// What activating a row asks the caller to do.
pub enum MenuActivation {
    /// Send this command. The menu stays where it is and its value column updates from the
    /// next `Status` -- which is the whole reason the `Watch` carries navigation rather than
    /// a rendered view.
    Command(MachineCommand),
    /// Open a submenu.
    Enter(MenuId),
    /// Open an editor frame, seeded with this value.
    ///
    /// The frame is named here rather than inferred by the caller, because the row is what
    /// knows which quantity it edits -- and for a parameter, which routine's.
    Edit {
        /// The editor frame to push.
        menu: MenuId,
        /// What it starts at, already clamped into range.
        value: Adjustable,
    },
    /// Close the menu, then run this routine. In that order, and the order is the point:
    /// see the note on the variant's only caller.
    RunRoutine(RoutineIndex),
    /// Leave this menu.
    Pop,
    /// Refused. The menu stays exactly where it is.
    Refuse,
}

/// Five minutes -- unchanged from the button-6 hold this replaces. Long enough to fetch a
/// phone and type a password, short enough that a window opened by accident closes itself
/// long before anyone would notice it was open.
const PROVISIONING_WINDOW_MS: u32 = 300_000;

/// Decide what activating a row does.
pub fn activate(row: &MenuRow, ctx: &MenuContext) -> MenuActivation {
    match row {
        MenuRow::Item(item) => match item.kind {
            MenuItemKind::OpenSettings => MenuActivation::Enter(MenuId::Settings),
            MenuItemKind::OpenRoutines => MenuActivation::Enter(MenuId::Routines),
            MenuItemKind::WifiProvisioning => MenuActivation::Command(match ctx.improv {
                ImprovState::Stopped => MachineCommand::OpenWifiProvisioningWindow {
                    duration_ms: PROVISIONING_WINDOW_MS,
                },
                _ => MachineCommand::CloseWifiProvisioningWindow,
            }),
            MenuItemKind::BrewTemperature => match ctx.brew_editable() {
                Some((current, max)) => MenuActivation::Edit {
                    menu: MenuId::EditBrewTemperature,
                    value: boiler_temperature_adjustable(current, max),
                },
                // Neither end of the range is guessable, and an editor seeded from a guess
                // would write that guess to flash on confirm.
                None => MenuActivation::Refuse,
            },
            MenuItemKind::Exit => MenuActivation::Pop,
        },

        // Selecting a routine **never runs it** -- it always opens the parameter screen, even
        // for a routine with no parameters, where that screen is a single "Run routine" row.
        //
        // The consistency is the safety property, not a keystroke cost. Running a routine
        // starts pumping hot water through the group, and a user has to be able to predict
        // what a press does *before* making it. A rule of "selecting runs it, unless it has
        // parameters, in which case it does not" is one nobody can hold, and the routine most
        // likely to have no parameters is a cleaning cycle.
        MenuRow::Routine { index, .. } => MenuActivation::Enter(MenuId::RoutineParameters(*index)),

        MenuRow::Parameter { routine, position, param, value } => MenuActivation::Edit {
            menu: MenuId::EditParameter { routine: *routine, position: *position as u8 },
            value: parameter_adjustable(param, *value),
        },

        // The controller refuses `RunRoutine` outright unless the machine is On, and the menu
        // is deliberately reachable while it is not. Sending anyway would close the menu and
        // do nothing, which is exactly the "reads as a broken machine" failure that the value
        // column's `n/a` is there to pre-empt.
        MenuRow::Run(_) if ctx.mode != MachineMode::On => MenuActivation::Refuse,
        MenuRow::Run(index) => MenuActivation::RunRoutine(*index),
    }
}

/// What confirming an editor frame does.
///
/// A parameter's value never leaves the button task until the routine runs, so this is only
/// ever a command for the brew setpoint. Returning `None` for a parameter rather than taking
/// two functions keeps the two editors' confirm paths one shape.
pub fn confirm_editor(menu: MenuId, value: f32) -> Option<MachineCommand> {
    match menu {
        // Values only, not `SetBoilerControlTarget`: changing the setpoint must not also
        // switch the boiler's mode. The controller persists this to flash itself, so there is
        // nothing to store on this side.
        MenuId::EditBrewTemperature => Some(MachineCommand::SetBoilerControlTargetValues(
            BREW_BOILER,
            variegated_controller_types::BoilerControlTargetValuesUpdate {
                temperature: Some(value),
                pressure: None,
            },
        )),
        _ => None,
    }
}

/// The GS3's menu stack, as published and as drawn.
pub type GsMenu = MenuStack<MenuId, MENU_MAX_DEPTH>;

/// What the button task publishes for the displays to draw.
///
/// Navigation plus the state the renderers cannot derive. It is deliberately *not* a rendered
/// view: the value column is still computed per frame from live `Status`, which is what lets
/// `OFF` become `ON` about a second after activation with no button pressed in between.
///
/// **`Eq` is gone and `PartialEq` stays**, because an in-progress edit is an `f32`. The
/// publish-on-change test in `button_controller_task` is `!=`, so `PartialEq` is all it ever
/// needed; and `Adjustable` maps NaN to its minimum, so a value here can never fail to
/// compare equal to itself and pin the watch into republishing every iteration.
#[derive(Debug, Clone, Copy, PartialEq, Default, defmt::Format)]
pub struct MenuSnapshot {
    /// Where the user is.
    pub stack: GsMenu,
    /// Whether a provisioning command is still unconfirmed. See [`MenuContext::wifi_pending`].
    pub wifi_pending: bool,
    /// The value being edited, when the top frame is an editor.
    pub editor: Option<Adjustable>,
    /// What has been dialled into the open routine's parameters.
    ///
    /// Carried rather than left in the button task because the parameter *list* has to draw
    /// every value, not just the one being edited. Positional and `Copy`, which is what lets
    /// this type stay a `Watch` payload.
    pub values: ParameterValues,
}

impl MenuSnapshot {
    /// Nothing open, nothing outstanding.
    pub fn closed() -> Self {
        Self {
            stack: GsMenu::closed(),
            wifi_pending: false,
            editor: None,
            values: ParameterValues::default(),
        }
    }
}

/// A provisioning command that has been sent and not yet confirmed by a `Status`.
#[derive(Debug, Clone, Copy)]
pub struct WifiRequest {
    /// What `improv` read when the command went out. Any change to it is the confirmation --
    /// which direction does not matter, and reading it this way means an unexpected state
    /// clears the row rather than pinning it.
    improv_at_request: ImprovState,
    /// When to give up waiting.
    deadline: Instant,
}

impl WifiRequest {
    /// Start waiting, from the state the machine was in when the command was sent.
    pub fn new(improv_at_request: ImprovState, now: Instant) -> Self {
        Self {
            improv_at_request,
            deadline: now + Duration::from_millis(WIFI_REQUEST_TIMEOUT_MS),
        }
    }

    /// Whether this request is still waiting, given the latest `improv` and the time.
    pub fn is_outstanding(&self, improv: ImprovState, now: Instant) -> bool {
        improv == self.improv_at_request && now < self.deadline
    }
}

/// Two: the TFT task on core 1 and the character LCD task on core 0.
///
/// `character-display` is off by default, so in three of the four gate configurations one
/// slot goes untaken -- exactly as `IDENTIFY_WATCH` already does, since its LCD receiver is
/// taken inside a `#[cfg]`. An untaken slot costs one `WakerRegistration` and cannot cause a
/// missed wake. Making the count feature-dependent would save eight bytes and cost a
/// `#[cfg]`'d constant that has to track which receiver takes are `#[cfg]`'d.
pub const MENU_WATCH_RECEIVERS: usize = 2;

/// Named because it is written out in a task signature and is unreadable inline.
pub type MenuSender = embassy_sync::watch::Sender<
    'static,
    variegated_hal::SyncSendRawMutex,
    MenuSnapshot,
    MENU_WATCH_RECEIVERS,
>;
