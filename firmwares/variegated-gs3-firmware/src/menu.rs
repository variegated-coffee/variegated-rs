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
use variegated_controller_lib::scale_calibration::ScaleCalibration;
use variegated_controller_types::bluetooth::BluetoothPeripheralList;
use variegated_controller_types::{
    BoilerIndex, GroupBrewControlMode, ImprovState, MachineCommand, MachineMode, RoutineIndex,
    Status, TemperatureType,
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
    /// Editing the steam boiler setpoint.
    EditSteamTemperature,
    /// Editing whichever brew target the group's current mode selects.
    ///
    /// One frame for pressure, flow and duty rather than three, because the mode decides
    /// which quantity exists -- see [`MenuItemKind::BrewTarget`].
    EditBrewTarget,
    /// Scale actions: tare, and calibration where the fitted scale supports it.
    Scale,
    /// What the radio is connected to. Read-only.
    WifiInfo,
    /// One row per Bluetooth association.
    Bluetooth,
}

impl MenuId {
    /// Whether this frame edits a value rather than showing a list.
    ///
    /// An editor has no rows, and buttons 1 and 2 move the *value* rather than the selection.
    ///
    /// An **exhaustive match rather than a `matches!`**, so that a new variant is a compile
    /// error here rather than silently answering `false` -- which would put buttons 1 and 2
    /// on a selection, on a screen that has no rows to select. This was the same trap
    /// `confirm_editor` carried in its `_ => None`, and it is worth closing the same way.
    pub const fn is_editor(&self) -> bool {
        match self {
            MenuId::EditParameter { .. }
            | MenuId::EditBrewTemperature
            | MenuId::EditSteamTemperature
            | MenuId::EditBrewTarget => true,
            MenuId::Root
            | MenuId::Settings
            | MenuId::Routines
            | MenuId::RoutineParameters(_)
            | MenuId::Scale
            | MenuId::WifiInfo
            | MenuId::Bluetooth => false,
        }
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
    /// Edits the steam boiler setpoint.
    SteamTemperature,
    /// Cycles the group's brew control mode.
    ///
    /// Through a curated four of [`GroupBrewControlMode`]'s ten, not all of them: the six
    /// curve and full-on variants are not editable from four buttons -- a `ControlCurve` is
    /// five numbers -- and offering a mode whose target this panel cannot then set would be
    /// worse than not offering it. See [`OFFERED_BREW_MODES`].
    BrewMode,
    /// Edits whichever brew target [`Self::BrewMode`] currently selects.
    ///
    /// **One row that renames itself**, rather than three of which two are always
    /// irrelevant. On a four-row panel three fixed rows would be most of the screen spent
    /// on quantities that do not apply.
    BrewTarget,
    /// Puts the machine into power-save standby.
    ///
    /// The one machine state with no other route from this panel: the `{5,3}` chord reaches
    /// `On` and `Off` only.
    Standby,
    /// Opens the scale submenu.
    OpenScale,
    /// Opens the Wi-Fi info submenu.
    OpenWifiInfo,
    /// Opens the Bluetooth submenu.
    OpenBluetooth,
    /// Tares the group scale.
    ScaleTare,
    /// Zero-calibrates the group scale. Only offered where the fitted scale supports it.
    ScaleZeroCalibrate,
    /// Calibrates the group scale against a 100 g reference. Only offered where supported.
    ScaleCalibrate100g,
    /// The network the radio is associated with. Read-only.
    WifiSsid,
    /// Signal strength, in dBm. Read-only.
    WifiRssi,
    /// The DHCP address. Read-only.
    WifiIp,
    /// Leaves the current menu; at the root that closes it.
    Exit,
}

impl MenuItemKind {
    /// Whether this row reports something rather than doing something.
    ///
    /// Info rows are drawn differently on the character LCD -- they take both of its rows,
    /// because an address does not fit in a four-column value field -- and they refuse
    /// activation rather than doing nothing quietly.
    const fn is_info(self) -> bool {
        matches!(self, Self::WifiSsid | Self::WifiRssi | Self::WifiIp)
    }

    /// Whether this row is one of the two conditional scale calibration actions.
    const fn is_calibration(self) -> bool {
        matches!(self, Self::ScaleZeroCalibrate | Self::ScaleCalibrate100g)
    }
}

/// What the brew target row is called, what unit it carries and what it currently reads.
///
/// `None` when the mode has no editable target, which is what greys the row. Delegates to
/// `variegated_machine_menu`, which owns the mode-to-quantity mapping and is host-tested --
/// this crate cannot host a test binary at all.
fn brew_target(config: &MenuConfig) -> Option<(&'static str, ParameterUnit, f32)> {
    let brew = config.brew?;
    variegated_machine_menu::brew_target(brew.mode, &brew.values)
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

/// Routines first: it is what the menu gets opened for. Settings is the thing you visit when
/// something has changed, and on a panel that shows one row at a time on the character LCD,
/// the order *is* the number of presses.
const ROOT_ITEMS: &[MenuItem] = &[
    MenuItem { label: "Routines", kind: MenuItemKind::OpenRoutines },
    MenuItem { label: "Settings", kind: MenuItemKind::OpenSettings },
    MenuItem { label: "Exit menu", kind: MenuItemKind::Exit },
];

/// Settings, in the order the tree gives: the numbers you change while tasting first, then
/// the machine state, then the submenus.
///
/// `Brew press` carries no label of its own -- [`MenuItemKind::BrewTarget`] renames it with
/// the mode above it -- but a placeholder is needed for the table, and it is what the row
/// reads when there is no mode to name it.
const SETTINGS_ITEMS: &[MenuItem] = &[
    MenuItem { label: "Brew temp", kind: MenuItemKind::BrewTemperature },
    MenuItem { label: "Steam temp", kind: MenuItemKind::SteamTemperature },
    MenuItem { label: "Brew mode", kind: MenuItemKind::BrewMode },
    MenuItem { label: "Brew press", kind: MenuItemKind::BrewTarget },
    MenuItem { label: "Standby", kind: MenuItemKind::Standby },
    MenuItem { label: "Scale", kind: MenuItemKind::OpenScale },
    MenuItem { label: "Wi-Fi Setup", kind: MenuItemKind::WifiProvisioning },
    MenuItem { label: "Wi-Fi Info", kind: MenuItemKind::OpenWifiInfo },
    MenuItem { label: "Bluetooth", kind: MenuItemKind::OpenBluetooth },
];

/// Scale actions. **Tare first, and it is the only one every scale can do.**
///
/// The two calibration rows are sliced off this table when the fitted scale cannot perform
/// them -- see [`scale_items`]. They are last precisely so that this is a truncation rather
/// than a filter: dropping a row from the middle would renumber the ones after it, and the
/// button task and both renderers resolve a selection index against this list independently.
const SCALE_ITEMS: &[MenuItem] = &[
    MenuItem { label: "Tare", kind: MenuItemKind::ScaleTare },
    MenuItem { label: "Zero cal", kind: MenuItemKind::ScaleZeroCalibrate },
    MenuItem { label: "Cal 100 g", kind: MenuItemKind::ScaleCalibrate100g },
];

/// Read-only. Every value here comes from `comms_status`, which is a latch.
const WIFI_INFO_ITEMS: &[MenuItem] = &[
    MenuItem { label: "SSID", kind: MenuItemKind::WifiSsid },
    MenuItem { label: "RSSI", kind: MenuItemKind::WifiRssi },
    MenuItem { label: "IP", kind: MenuItemKind::WifiIp },
];

/// Every fixed row in this file, for the width assertion below.
const ALL_ITEM_TABLES: &[&[MenuItem]] =
    &[ROOT_ITEMS, SETTINGS_ITEMS, SCALE_ITEMS, WIFI_INFO_ITEMS];

/// Twelve characters, and this is checked at compile time rather than trusted.
///
/// `pad_or_truncate_to_16` cuts the character LCD's row from the right, silently -- so an
/// over-long label does not truncate *itself*, it eats the value column beside it. A row
/// reading `Wi-Fi Provisio` with no `ON` after it looks like a value that failed to load,
/// which is the hardest kind of bug to attribute.
///
/// A `const` block rather than a test because this crate cannot host a test binary at all:
/// it sets `test = false` and depends on `embassy-rp`. The build is the only gate available,
/// and this makes it one.
const _: () = {
    let mut table = 0;
    while table < ALL_ITEM_TABLES.len() {
        let items = ALL_ITEM_TABLES[table];
        let mut item = 0;
        while item < items.len() {
            // `len()` is bytes, not characters. Every label here is ASCII, and that is worth
            // holding to: the HD44780's A00 ROM and the TFT's `_tr` fonts both cover 32..127
            // and nothing else, so a non-ASCII label would not draw either way.
            assert!(items[item].label.len() <= 12, "menu label longer than twelve columns");
            item += 1;
        }
        table += 1;
    }
};

/// The scale rows this machine has.
///
/// One row on a machine whose scale cannot calibrate, three where it can. **Hidden rather
/// than greyed**, and that is the deliberate half: a scale that cannot zero-calibrate will
/// never be able to, so a permanently disabled row is a control that can only ever
/// disappoint. Contrast the scale merely being switched off, which *is* greyed -- that one
/// has a fix, and the row is where it gets said.
fn scale_items(data: &MenuData) -> &'static [MenuItem] {
    if data.scale_calibration.is_offered() {
        SCALE_ITEMS
    } else {
        &SCALE_ITEMS[..1]
    }
}

/// The fixed rows of a menu, or an empty slice for one whose rows come from data.
fn fixed_items(menu: MenuId, data: &MenuData) -> &'static [MenuItem] {
    match menu {
        MenuId::Root => ROOT_ITEMS,
        MenuId::Settings => SETTINGS_ITEMS,
        MenuId::Scale => scale_items(data),
        MenuId::WifiInfo => WIFI_INFO_ITEMS,
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
    /// Whether `routine`'s prerequisites are currently met.
    ///
    /// Supplied rather than computed here for the same reason `routines` is: answering it
    /// needs a `MachineDefinition` and a live `PeripheralStatus`, and this module sees a
    /// deliberately narrow projection of `Status` and neither of those. Kept live rather
    /// than only evaluated at fetch time -- a scale can drop while the screen is open.
    pub routine_runnable: bool,
    /// Whether the scale submenu offers calibration, and whether it can be used now.
    ///
    /// Supplied for `routine_runnable`'s reason -- it needs the same two inputs. This one
    /// decides how many rows [`MenuId::Scale`] *has*, which is why it must come from one
    /// implementation: two sides disagreeing about a list's length is a selection index
    /// pointing at different rows on each.
    pub scale_calibration: ScaleCalibration,
    /// Whether any scale is answering, for the `Tare` row.
    ///
    /// A different question from [`Self::scale_calibration`]: every scale in this tree can
    /// tare, so this asks only about liveness. See [`MenuRow::ScaleAction`].
    pub scale_present: bool,
    /// Every Bluetooth association, for [`MenuId::Bluetooth`].
    ///
    /// Borrowed from `Configuration` rather than projected into [`MenuContext`], because it
    /// is a list of *rows* -- which is what this struct is for, and what `routines` already
    /// does. `None` before the first `Configuration` arrives, which makes the screen briefly
    /// empty rather than wrong.
    pub bluetooth: Option<&'a BluetoothPeripheralList>,
    /// The unit [`MenuId::EditBrewTarget`] is currently editing. See [`editor_unit`].
    pub brew_target_unit: Option<ParameterUnit>,
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
        /// Whether the machine can currently satisfy its prerequisites.
        ///
        /// Greyed rather than hidden when it cannot: a routine missing from the machine's
        /// list but present in the browser sends a user hunting, where a greyed row with
        /// "no sensor" beside it says what to fix.
        runnable: bool,
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
    /// A scale action: tare, or one of the two calibrations.
    ///
    /// Its own variant rather than a plain [`MenuRow::Item`] because it carries liveness,
    /// exactly as [`MenuRow::Routine`] does and for the same reason: `value` and `activate`
    /// see only a row and a [`MenuContext`], and this answer comes from [`MenuData`].
    ///
    /// **All three scale rows need it, not just the calibrations.** `Group::scale_tare`
    /// returns `Ok(())` when there is no controller, and a tare sent to a disconnected
    /// Bluetooth scale is dropped on the far side -- the same silent success that made the
    /// calibration rows need a gate. A screen where `Zero cal` says `sens` and `Tare` sits
    /// there looking live, while both do nothing, is worse than one where neither is gated:
    /// it tells the user the working thing is broken and the broken thing is fine.
    ///
    /// `available: false` never means "not supported" here. A scale that cannot calibrate
    /// has no calibration row at all -- see [`scale_items`].
    ScaleAction {
        /// Its table entry, for the label and for what activating it sends.
        item: &'a MenuItem,
        /// Whether the scale is answering right now.
        available: bool,
    },
    /// One Bluetooth association, as a switch.
    ///
    /// Switching one off keeps the pairing -- that is the whole point of
    /// `SetBluetoothPeripheralEnabled` existing separately from `RemoveBluetoothPeripheral`.
    BluetoothPeripheral {
        /// Which association. The identity, and what the command names.
        id: variegated_controller_types::PeripheralId,
        /// Its user-editable name, already reduced to what these panels can draw.
        ///
        /// Owned rather than borrowed from the association, because it is one of the two
        /// strings on this panel that comes from outside the firmware -- a BLE advertised
        /// name, or something typed into the web UI -- and both panels fail badly on a
        /// character outside 32..=126. See `variegated_machine_menu::push_drawable`.
        name: heapless::String<{ variegated_controller_types::bluetooth::BLUETOOTH_NAME_LEN }>,
        /// Whether it is currently switched on.
        enabled: bool,
    },
    /// The row at the bottom of a parameter screen that runs the routine.
    Run {
        index: RoutineIndex,
        /// Whether the machine can currently satisfy the open routine's prerequisites.
        ///
        /// Carried here as well as on the list row, because a scale can drop while the user
        /// is already standing on this screen. Without it the row stays live, the press is
        /// refused by the controller, and the menu closes with nothing to show for it.
        runnable: bool,
    },
}

/// How many rows a menu has, including every piece of chrome.
pub fn row_count(menu: MenuId, data: &MenuData) -> usize {
    match menu {
        MenuId::Root | MenuId::Settings | MenuId::Scale | MenuId::WifiInfo => {
            fixed_items(menu, data).len()
        }
        MenuId::Routines => data.routines.map_or(0, |rows| rows.len()),
        MenuId::Bluetooth => data.bluetooth.map_or(0, |list| list.len()),
        // The parameters, then Run. A routine with no parameters still gets the screen, and
        // it is one row long -- see `activate`.
        MenuId::RoutineParameters(_) => data.param_count() + 1,
        // An editor has no rows.
        MenuId::EditParameter { .. }
        | MenuId::EditBrewTemperature
        | MenuId::EditSteamTemperature
        | MenuId::EditBrewTarget => 0,
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
        MenuId::Root | MenuId::Settings | MenuId::WifiInfo => {
            fixed_items(menu, data).get(index).map(MenuRow::Item)
        }
        MenuId::Scale => fixed_items(menu, data).get(index).map(|item| MenuRow::ScaleAction {
            item,
            // Two different questions, and conflating them would be wrong in both
            // directions. Calibration asks whether *this* scale can calibrate and is live;
            // tare asks only whether a scale is answering, because every scale in this tree
            // can tare. Gating tare on the calibration answer would grey it on the GS3's
            // default Bluetooth scale, which tares perfectly well.
            available: if item.kind.is_calibration() {
                data.scale_calibration.is_available()
            } else {
                data.scale_present
            },
        }),
        MenuId::Bluetooth => data.bluetooth?.get(index).map(|association| {
            let mut name = heapless::String::new();
            variegated_machine_menu::push_drawable(&mut name, association.name.as_str());
            MenuRow::BluetoothPeripheral {
                id: association.id,
                name,
                enabled: association.enabled,
            }
        }),
        MenuId::Routines => data
            .routines?
            .get(index)
            .map(|r| MenuRow::Routine {
                index: r.index,
                name: r.name.as_str(),
                runnable: r.runnable,
            }),
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
                ParameterRow::Execute => Some(MenuRow::Run {
                    index: routine_index,
                    runnable: data.routine_runnable,
                }),
                // This panel draws no back row.
                ParameterRow::Back => None,
            }
        }
        MenuId::EditParameter { .. }
        | MenuId::EditBrewTemperature
        | MenuId::EditSteamTemperature
        | MenuId::EditBrewTarget => None,
    }
}

/// What a row is drawn as, on the left.
///
/// Borrowed rather than owned: a routine's name lives in the fetched list and a parameter's
/// in the fetched routine, and copying either per frame would allocate on a render loop.
/// `ctx` is here for exactly one row: [`MenuItemKind::BrewTarget`] is named by the mode
/// above it, so its label is not a property of the table. Every label it can return is
/// still `&'static str`, so nothing allocates on a render loop.
/// The returned reference borrows from `row`, not from the data behind it, because one row
/// owns its label: a Bluetooth name is sanitised into the row when it is built. Every caller
/// uses this while the row is still in scope, so the shorter lifetime costs nothing.
pub fn label<'r>(row: &'r MenuRow<'_>, ctx: &MenuContext) -> &'r str {
    match row {
        MenuRow::Item(item) if item.kind == MenuItemKind::BrewTarget => {
            brew_target(&ctx.config).map_or(item.label, |(label, _, _)| label)
        }
        MenuRow::Item(item) => item.label,
        MenuRow::Routine { name, .. } => name,
        MenuRow::Parameter { param, .. } => param.name.as_str(),
        // A device that advertised no name, or whose name was cleared from the web. The row
        // still has a working toggle, so drawing it blank would be a switch with nothing
        // beside it -- worse than a placeholder, because there is no way to tell it from a
        // rendering failure.
        MenuRow::BluetoothPeripheral { name, .. } if name.is_empty() => "(unnamed)",
        MenuRow::BluetoothPeripheral { name, .. } => name,
        MenuRow::ScaleAction { item, .. } => item.label,
        MenuRow::Run { .. } => "Run routine",
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
        MenuId::EditSteamTemperature => "Steam temp",
        // Named by the mode, like the row it was opened from. `title` has no `MenuContext`,
        // and threading one in for this alone would change three call sites -- the editor is
        // seeded from the row, so if it is open at all the mode had a target when it opened.
        MenuId::EditBrewTarget => "Brew target",
        MenuId::Scale => "Scale",
        MenuId::WifiInfo => "Wi-Fi Info",
        MenuId::Bluetooth => "Bluetooth",
    }
}

/// What a menu with no rows says instead of nothing.
///
/// Both data-driven lists can legitimately be empty, and [`MenuId::Bluetooth`] is empty on
/// every machine that has not paired anything -- which is all of them, out of the box. A
/// screen drawn blank in that state is indistinguishable from a crashed renderer.
///
/// Twelve characters, like any other label: this goes through the same `{:<12.12}` split.
pub fn empty_label(menu: MenuId) -> &'static str {
    match menu {
        MenuId::Bluetooth => "None paired",
        MenuId::Routines => "No routines",
        // Every other menu has fixed rows and cannot be empty. Reached only if one grows a
        // data-driven list without coming here, so it says nothing rather than guessing.
        _ => "",
    }
}

/// The unit an editor frame's value carries, for the renderers to draw beside it.
pub fn editor_unit(menu: MenuId, data: &MenuData) -> Option<ParameterUnit> {
    match menu {
        MenuId::EditBrewTemperature | MenuId::EditSteamTemperature => {
            Some(ParameterUnit::Celsius)
        }
        // Whatever the mode names right now. In `MenuData` rather than derived here because
        // this function has no `MenuContext`, and both sides build that struct from the same
        // config projection, so they cannot disagree about it.
        MenuId::EditBrewTarget => data.brew_target_unit,
        MenuId::EditParameter { position, .. } => data
            .routine
            .and_then(|r| r.parameters().get(position as usize))
            .and_then(|p| p.unit),
        _ => None,
    }
}

/// The subset of `Configuration` the menu reads.
///
/// Split out of [`MenuContext`] once more than one field came from here, so that the two
/// sources stay distinguishable: everything in `MenuContext` proper is read from a `Status`
/// that arrives ten times a second, and everything here from a `Configuration` republished
/// every ten seconds whether or not it changed.
///
/// **`Copy`, and every field a scalar.** The Bluetooth associations are the one part of
/// `Configuration` the menu needs that is not, and they live in [`MenuData`] instead --
/// they are a list of rows, which is what that struct is for.
///
/// **Every field is an `Option`, and `Default` is "nothing known yet".** The first
/// `Configuration` can be up to ten seconds after boot, and the menu is reachable before it
/// arrives -- so every row backed by this has to be able to say it does not know, the way
/// `Brew temp` already does with `n/a`.
///
/// The alternative was tried and is worse: a `brew_mode` defaulting to
/// `GroupBrewControlMode::Off` makes the row *claim* the group is off. That is not a missing
/// value, it is a wrong one, and on a machine that was mid-shot it would be a wrong one
/// about what the pump is doing.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct MenuConfig {
    /// The brew boiler's configured ceiling. `None` until the first `Configuration`.
    ///
    /// Read from the machine rather than written down here: a setpoint above the interlock
    /// can only produce an element that runs to the limit and shuts off, so a ceiling this
    /// file invented would be a control problem wearing a display problem's clothes.
    pub brew_max: Option<TemperatureType>,
    /// The steam boiler's configured ceiling. `None` until the first `Configuration`.
    pub steam_max: Option<TemperatureType>,
    /// The group's brew control mode and its targets. `None` until the first
    /// `Configuration`.
    ///
    /// The pair rather than the mode alone, because the mode is what says which of the
    /// targets is live -- see `variegated_machine_menu::brew_target` -- and separating them
    /// would allow a state where one is known and the other is not. `GroupBrewControlState`
    /// is `Copy`, so carrying it whole costs nothing.
    pub brew: Option<variegated_controller_types::GroupBrewControlState>,
}

impl MenuConfig {
    /// Read the projection out of a published `Configuration`.
    ///
    /// One implementation, because the button task and the display task both need it and a
    /// second would be a second thing to get wrong. Boiler 0 is the brew boiler and 1 the
    /// steam boiler, matching `SetBoilerControlTargetValues`.
    pub fn from_configuration(configuration: &variegated_controller_types::Configuration) -> Self {
        Self {
            brew_max: configuration
                .boiler_configurations
                .get(&BREW_BOILER)
                .and_then(|b| b.max_temperature),
            steam_max: configuration
                .boiler_configurations
                .get(&STEAM_BOILER)
                .and_then(|b| b.max_temperature),
            // The published `Configuration` folds the controller's *ephemeral* brew control
            // state into this field, so it is the live mode rather than the stored default.
            // That is also why editing it does not survive a reboot -- see `activate`.
            brew: configuration
                .group_configurations
                .get(&GROUP)
                .map(|g| g.brew_control_state),
        }
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
    /// The steam boiler's current setpoint. `None` until the first `Status` arrives.
    pub steam_target: Option<TemperatureType>,
    /// What the radio reports, or `None` when nothing recent has arrived.
    ///
    /// **Gated on age, not merely on presence.** `comms_status` is a latch: the controller
    /// republishes the last one it received forever, with the timestamp extrapolated, so an
    /// ungated read shows a network that went away minutes ago as though it were live. The
    /// only thing in `Status` that can say otherwise is `comms_status_age`.
    pub wifi: Option<WifiInfo>,
    /// Everything the menu reads out of `Configuration`.
    pub config: MenuConfig,
}

/// What the Wi-Fi info screen reports, once staleness has been ruled out.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WifiInfo {
    /// Whether the station is associated.
    pub connected: bool,
    /// Signal strength in dBm, or `None` when the link is down.
    pub rssi: Option<i8>,
    /// The DHCP address, or `None` before the lease is granted.
    pub ip: Option<[u8; 4]>,
}

impl MenuContext {
    /// Read the projection out of a status, plus the two things a status does not carry.
    pub fn from_status(status: &Status, wifi_pending: bool, config: MenuConfig) -> Self {
        // Absent *or* stale reads as "nothing to report" -- see the note on `wifi`. A
        // `comms_status` with no age has never been received at all.
        let fresh = status
            .comms_status_age
            .is_some_and(|age| age < variegated_controller_types::COMMS_STATUS_STALE_AFTER);

        Self {
            improv: status.comms_status.as_ref().map(|c| c.improv).unwrap_or_default(),
            wifi_pending,
            mode: status.mode,
            brew_target: status
                .get_boiler_status(BREW_BOILER)
                .map(|b| b.control_state.values.target_temperature),
            steam_target: status
                .get_boiler_status(STEAM_BOILER)
                .map(|b| b.control_state.values.target_temperature),
            wifi: status.comms_status.as_ref().filter(|_| fresh).map(|c| WifiInfo {
                connected: c.wifi_connected,
                rssi: c.wifi_rssi,
                ip: c.wifi_ip,
            }),
            config,
        }
    }

    /// The SSID to display, borrowed from the status rather than copied into the projection.
    ///
    /// A `heapless::String<32>` in [`MenuContext`] would cost it `Copy`, which it is worth
    /// keeping; and this is read by one row on one screen. Same freshness gate as [`Self::wifi`].
    pub fn wifi_ssid(status: &Status) -> &str {
        let fresh = status
            .comms_status_age
            .is_some_and(|age| age < variegated_controller_types::COMMS_STATUS_STALE_AFTER);
        status
            .comms_status
            .as_ref()
            .filter(|_| fresh)
            .map_or("", |c| c.wifi_ssid.as_str())
    }

    /// Whether the brew setpoint can be edited: both ends of its range have to be known.
    fn brew_editable(&self) -> Option<(TemperatureType, TemperatureType)> {
        Some((self.brew_target?, self.config.brew_max?))
    }

    /// The same, for the steam setpoint.
    fn steam_editable(&self) -> Option<(TemperatureType, TemperatureType)> {
        Some((self.steam_target?, self.config.steam_max?))
    }
}

/// Boiler 0. Named because `SetBoilerControlTargetValues(0, ..)` reads as a magic number.
const BREW_BOILER: BoilerIndex = 0;

/// Boiler 1, the steam boiler. Same reason as [`BREW_BOILER`].
const STEAM_BOILER: BoilerIndex = 1;

/// Group 0. This machine has one.
const GROUP: variegated_controller_types::GroupIndex = 0;

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

/// What a routine reads when the machine cannot sense something it needs.
///
/// Distinct from [`UNAVAILABLE`] because the fix is different: `n/a` means turn the machine
/// on, this means connect the scale. Four characters, which is the value field's width.
const NO_SENSOR: &str = "sens";

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
            // Both read `brew_editable`/`steam_editable` rather than the setpoint alone, so
            // the value column and `activate` agree about whether the row will do anything.
            //
            // They disagreed before, and the window was real: the setpoint comes from
            // `Status` at 10 Hz and the ceiling from `Configuration`, which can be ten
            // seconds behind after boot. In between, the row showed a correct `94.0` and
            // button 3 did nothing -- exactly the dead-button reading `n/a` exists to
            // pre-empt, and the reason this file's contract is that a refusal is always
            // pre-announced.
            MenuItemKind::BrewTemperature => Some(match ctx.brew_editable() {
                Some((value, _)) => {
                    MenuValue::Number { value, unit: Some(ParameterUnit::Celsius) }
                }
                None => MenuValue::Text(UNAVAILABLE),
            }),
            MenuItemKind::SteamTemperature => Some(match ctx.steam_editable() {
                Some((value, _)) => {
                    MenuValue::Number { value, unit: Some(ParameterUnit::Celsius) }
                }
                None => MenuValue::Text(UNAVAILABLE),
            }),
            // `n/a` rather than `Off` before the first `Configuration`: this row says what
            // the pump is doing, and "off" is a claim rather than an absence.
            MenuItemKind::BrewMode => Some(match ctx.config.brew {
                Some(brew) => {
                    MenuValue::Text(variegated_machine_menu::brew_mode_label(brew.mode))
                }
                None => MenuValue::Text(UNAVAILABLE),
            }),
            // Greyed rather than hidden when the mode has no target. Hiding it would make
            // the Settings list change length under the user, and both this task and the
            // renderers resolve a selection index against that length independently.
            MenuItemKind::BrewTarget => Some(match brew_target(&ctx.config) {
                Some((_, unit, value)) => MenuValue::Number { value, unit: Some(unit) },
                None => MenuValue::Text(UNAVAILABLE),
            }),
            // Info rows carry their real value through `info_value` -- nothing here can
            // render a 15-character address in a four-column field. All this decides is the
            // no-recent-report case, where `ctx.wifi` is `None` and the row reads `n/a`.
            MenuItemKind::WifiSsid | MenuItemKind::WifiRssi | MenuItemKind::WifiIp => {
                ctx.wifi.is_none().then_some(MenuValue::Text(UNAVAILABLE))
            }
            MenuItemKind::OpenSettings
            | MenuItemKind::OpenRoutines
            | MenuItemKind::OpenScale
            | MenuItemKind::OpenWifiInfo
            | MenuItemKind::OpenBluetooth
            | MenuItemKind::Standby
            | MenuItemKind::ScaleTare
            | MenuItemKind::ScaleZeroCalibrate
            | MenuItemKind::ScaleCalibrate100g
            | MenuItemKind::Exit => None,
        },
        MenuRow::Parameter { param, value, .. } => {
            Some(MenuValue::Number { value: *value, unit: param.unit })
        }
        MenuRow::BluetoothPeripheral { enabled, .. } => {
            Some(MenuValue::Text(if *enabled { "ON" } else { "OFF" }))
        }
        // Only ever "switched off" -- an unsupporting scale has no calibration row at all.
        MenuRow::ScaleAction { available: false, .. } => Some(MenuValue::Text(NO_SENSOR)),
        MenuRow::ScaleAction { .. } => None,
        // Say why the row will not do anything before it is pressed, rather than closing the
        // menu and leaving the machine cold. See `activate`.
        MenuRow::Run { .. } if ctx.mode != MachineMode::On => Some(MenuValue::Text(UNAVAILABLE)),
        // Same rule, different reason: the machine cannot sense something this routine needs.
        // On the list row so a user reading the list knows which routines are available
        // without opening each one, and on the Run row so a sensor that drops while they are
        // already on the parameter screen shows up there too.
        MenuRow::Routine { runnable: false, .. } | MenuRow::Run { runnable: false, .. } => {
            Some(MenuValue::Text(NO_SENSOR))
        }
        MenuRow::Run { .. } | MenuRow::Routine { .. } => None,
    }
}

/// How wide an info row's value can be.
///
/// An IPv4 address in dotted quad is fifteen characters and an SSID is up to
/// [`WIFI_SSID_LEN`](variegated_controller_types::wifi::WIFI_SSID_LEN). Sized for the
/// larger; the character LCD truncates to its own sixteen columns, and the TFT draws what
/// fits beside the label.
pub const INFO_TEXT_LEN: usize = 32;

/// An info row's value, or `None` when nothing recent has been reported.
///
/// Separate from [`value`] because these do not fit a value column: an address is fifteen
/// characters against four. The character LCD gives info rows both of its rows and the TFT
/// draws them right-aligned like any other value, so each renderer decides what to do with
/// the width -- which is the same split [`MenuValue`] exists for.
///
/// `None` means the radio has said nothing recently, and the row falls back to `n/a`
/// through [`value`].
///
/// `ssid` is passed in rather than read from [`MenuContext`], which does not carry it: a
/// `heapless::String<32>` there would cost that struct its `Copy`, for one row on one
/// screen. Both renderers hold the `Status` and get it from [`MenuContext::wifi_ssid`].
pub fn info_value(
    row: &MenuRow,
    ctx: &MenuContext,
    ssid: &str,
) -> Option<heapless::String<INFO_TEXT_LEN>> {
    use core::fmt::Write;

    let MenuRow::Item(item) = row else { return None };
    if !item.kind.is_info() {
        return None;
    }

    let wifi = ctx.wifi?;
    let mut out = heapless::String::new();

    match item.kind {
        MenuItemKind::WifiSsid => {
            // The station reports the network it was configured with, and the comms
            // processor blanks it while the link is down -- so an empty string here is
            // "not on a network", not "no name". Say which, rather than drawing nothing.
            if wifi.connected && !ssid.is_empty() {
                // Through `drawable`, because this is one of exactly two strings on these
                // panels chosen by someone outside this firmware. A non-ASCII SSID is
                // ordinary, and undrawn it would render as an empty row -- which on this
                // screen reads as "not connected", the opposite of the truth.
                for c in variegated_machine_menu::drawable(ssid).chars() {
                    let _ = out.push(c);
                }
            } else {
                let _ = out.push_str("not connected");
            }
        }
        MenuItemKind::WifiRssi => match wifi.rssi {
            Some(rssi) => {
                let _ = write!(out, "{} dBm", rssi);
            }
            None => {
                let _ = out.push_str("--");
            }
        },
        MenuItemKind::WifiIp => match wifi.ip {
            Some([a, b, c, d]) => {
                let _ = write!(out, "{}.{}.{}.{}", a, b, c, d);
            }
            // Before DHCP completes. Not `0.0.0.0`, which is a real-looking address that
            // means "this host on this network" and never an interface's own.
            None => {
                let _ = out.push_str("--");
            }
        },
        _ => return None,
    }

    Some(out)
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
    /// Close the menu, then send this command. In that order, for the reason
    /// [`Self::RunRoutine`] gives.
    ///
    /// For a row that changes what the *panel* should be showing. `Standby` is the one so
    /// far: the menu draws over every other display mode, so leaving it up would put the
    /// user on a Settings list belonging to a machine that had just shut its boilers down,
    /// with no sign anything had happened. Every other state transition on this panel --
    /// the `{5,3}` power chord, running a routine -- also leaves the menu behind.
    CommandAndClose(MachineCommand),
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
/// Whether the machine can currently satisfy a routine's prerequisites.
///
/// Wraps `variegated_controller_lib::routine_prerequisites` so both places that build a
/// routine list -- the button task and the display task -- ask the same question the same
/// way, and so that neither has to remember the `None`-definition case.
///
/// `definition` is `None` only before boot has finished populating it. Answering "runnable"
/// there is deliberate: the controller's own backstop still refuses anything that really
/// cannot run, and greying every routine on a machine that is merely still starting up
/// would be a worse lie than the momentary opposite.
pub fn routine_runnable(
    routine: &Routine,
    definition: Option<&variegated_controller_types::MachineDefinition>,
    peripherals: &variegated_controller_types::PeripheralStatus,
) -> bool {
    let Some(definition) = definition else { return true };
    variegated_controller_lib::routine_prerequisites::prerequisites_satisfied(
        &routine.prerequisites,
        definition,
        peripherals,
    )
}

/// Whether this machine's scale can be calibrated, and whether it can be right now.
///
/// Wraps `variegated_controller_lib::scale_calibration` for the same reason
/// [`routine_runnable`] wraps its own module: both places that build the Scale submenu ask
/// the same question the same way, and neither has to remember the `None`-definition case.
///
/// **`None` here means `Unsupported`, unlike `routine_runnable`'s optimistic default.** A
/// definition arrives before any menu can be opened, so this is a boot-time transient; and
/// the two directions of being wrong are not symmetrical. Guessing "supported" would draw
/// two rows and then remove them, moving the selection under the user's finger. Guessing
/// "unsupported" only delays their appearance.
pub fn scale_calibration(
    definition: Option<&variegated_controller_types::MachineDefinition>,
    peripherals: &variegated_controller_types::PeripheralStatus,
) -> ScaleCalibration {
    let Some(definition) = definition else { return ScaleCalibration::Unsupported };
    variegated_controller_lib::scale_calibration::scale_calibration(definition, peripherals)
}

/// Whether any scale is currently answering, for [`MenuData::scale_present`].
///
/// The same question `routine_prerequisites` asks of a routine that needs a weight, asked
/// the same way -- so the `Tare` row and a brew-by-weight routine cannot disagree about
/// whether this machine can weigh anything right now.
///
/// `None` definition greys the row, matching [`scale_calibration`]'s choice and for the same
/// reason: a definition arrives before any menu can be opened, and greying briefly is a
/// smaller lie than offering an action that will silently do nothing.
///
/// Note that the button task latches its definition only while a menu is open, so on the
/// single iteration a menu opens it can still hold `None` while the renderers already have
/// one. That costs a frame of greying, and cannot cost a row: this answer feeds a row's
/// *value*, never [`row_count`].
pub fn scale_present(
    definition: Option<&variegated_controller_types::MachineDefinition>,
    peripherals: &variegated_controller_types::PeripheralStatus,
) -> bool {
    let Some(definition) = definition else { return false };
    variegated_controller_lib::routine_prerequisites::capability_available(
        variegated_controller_types::SensorCapability::Weight,
        definition,
        peripherals,
    )
}

/// The unit the brew target row is currently editing, for [`MenuData::brew_target_unit`].
pub fn brew_target_unit(config: &MenuConfig) -> Option<ParameterUnit> {
    brew_target(config).map(|(_, unit, _)| unit)
}

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
            MenuItemKind::SteamTemperature => match ctx.steam_editable() {
                Some((current, max)) => MenuActivation::Edit {
                    menu: MenuId::EditSteamTemperature,
                    value: boiler_temperature_adjustable(current, max),
                },
                None => MenuActivation::Refuse,
            },
            // Mode only, with no values update: this row changes *which* target the group
            // uses, and carrying a value would also overwrite the one the row below edits.
            //
            // **This does not survive a reboot.** `SetGroupBrewControlTarget` writes the
            // controller's ephemeral state, and the persistent
            // `default_group_brew_control_state` it is restored from at boot has no command
            // at all -- so this row and `Brew temp` two rows above disagree about
            // permanence. Fixing that is one appended `MachineCommand` and an arm.
            // Refused until the current mode is known. "Next" is meaningless without a
            // "this", and guessing would send the group into a mode chosen from nothing.
            MenuItemKind::BrewMode => match ctx.config.brew {
                Some(brew) => MenuActivation::Command(
                    MachineCommand::SetGroupBrewControlTarget(
                        GROUP,
                        variegated_machine_menu::next_brew_mode(brew.mode),
                        None,
                    ),
                ),
                None => MenuActivation::Refuse,
            },
            MenuItemKind::BrewTarget => match brew_target(&ctx.config) {
                Some((_, unit, current)) => MenuActivation::Edit {
                    menu: MenuId::EditBrewTarget,
                    value: unit_adjustable(unit, current),
                },
                // The mode has no editable target. The value column already reads `n/a`.
                None => MenuActivation::Refuse,
            },
            MenuItemKind::Standby => MenuActivation::CommandAndClose(
                MachineCommand::SetMachineMode(MachineMode::PowerSaveStandby),
            ),
            MenuItemKind::OpenScale => MenuActivation::Enter(MenuId::Scale),
            MenuItemKind::OpenWifiInfo => MenuActivation::Enter(MenuId::WifiInfo),
            MenuItemKind::OpenBluetooth => MenuActivation::Enter(MenuId::Bluetooth),
            MenuItemKind::ScaleTare => {
                MenuActivation::Command(MachineCommand::TareGroupScale(GROUP))
            }
            // Reached only through `MenuRow::ScaleCalibration`, which carries the liveness
            // gate. A bare `Item` with one of these kinds would be a row this file did not
            // build, so refusing is the safe reading.
            MenuItemKind::ScaleZeroCalibrate | MenuItemKind::ScaleCalibrate100g => {
                MenuActivation::Refuse
            }
            // Nothing to activate. Refused rather than silently ignored so the press is at
            // least logged, and so a row that grows an action later has to say so here.
            MenuItemKind::WifiSsid | MenuItemKind::WifiRssi | MenuItemKind::WifiIp => {
                MenuActivation::Refuse
            }
            MenuItemKind::Exit => MenuActivation::Pop,
        },

        // Switching an association off keeps the pairing -- which is why
        // `SetBluetoothPeripheralEnabled` exists separately from `RemoveBluetoothPeripheral`,
        // and why this row is a toggle rather than a way to forget a scale.
        MenuRow::BluetoothPeripheral { id, enabled, .. } => MenuActivation::Command(
            MachineCommand::SetBluetoothPeripheralEnabled(*id, !*enabled),
        ),

        // The scale is fitted but not answering. Refused here rather than sent and dropped:
        // every `Group::scale_*` wrapper returns `Ok(())` when there is no controller, and a
        // command sent to a disconnected Bluetooth scale is discarded on the far side -- so
        // the press would report nothing and do nothing.
        MenuRow::ScaleAction { available: false, .. } => MenuActivation::Refuse,
        MenuRow::ScaleAction { item, .. } => MenuActivation::Command(match item.kind {
            MenuItemKind::ScaleZeroCalibrate => MachineCommand::ZeroCalibrateGroupScale(GROUP),
            // 100 g is the only reference weight the hardware supports -- see
            // `GravityController::get_capabilities`, whose `supported_reference_weights` is
            // `&[100]` and which refuses anything else.
            MenuItemKind::ScaleCalibrate100g => MachineCommand::CalibrateGroupScale100g(GROUP),
            // `ScaleTare`, and anything else that ever joins `SCALE_ITEMS`.
            _ => MachineCommand::TareGroupScale(GROUP),
        }),

        // Selecting a routine **never runs it** -- it always opens the parameter screen, even
        // for a routine with no parameters, where that screen is a single "Run routine" row.
        //
        // The consistency is the safety property, not a keystroke cost. Running a routine
        // starts pumping hot water through the group, and a user has to be able to predict
        // what a press does *before* making it. A rule of "selecting runs it, unless it has
        // parameters, in which case it does not" is one nobody can hold, and the routine most
        // likely to have no parameters is a cleaning cycle.
        // Refused at the list rather than at the Run row inside it, so a user is not walked
        // into a parameter screen for a routine that cannot be started from it.
        MenuRow::Routine { runnable: false, .. } => MenuActivation::Refuse,
        MenuRow::Routine { index, .. } => MenuActivation::Enter(MenuId::RoutineParameters(*index)),

        MenuRow::Parameter { routine, position, param, value } => MenuActivation::Edit {
            menu: MenuId::EditParameter { routine: *routine, position: *position as u8 },
            value: parameter_adjustable(param, *value),
        },

        // The controller refuses `RunRoutine` outright unless the machine is On, and the menu
        // is deliberately reachable while it is not. Sending anyway would close the menu and
        // do nothing, which is exactly the "reads as a broken machine" failure that the value
        // column's `n/a` is there to pre-empt.
        MenuRow::Run { .. } if ctx.mode != MachineMode::On => MenuActivation::Refuse,
        // A prerequisite lost while this screen was open. Refused here rather than left to
        // the controller, which would close the menu and do nothing visible.
        MenuRow::Run { runnable: false, .. } => MenuActivation::Refuse,
        MenuRow::Run { index, .. } => MenuActivation::RunRoutine(*index),
    }
}

/// What confirming an editor frame does.
///
/// A parameter's value never leaves the button task until the routine runs, so this is only
/// ever a command for the brew setpoint. Returning `None` for a parameter rather than taking
/// two functions keeps the two editors' confirm paths one shape.
/// What *holding* the select button on a row does.
///
/// One thing: a routine in the list runs, with no parameter screen in between. Everything
/// else refuses, so the gesture means exactly one thing wherever it is made.
///
/// **Why a hold rather than another row.** Tapping a routine deliberately never runs it --
/// it always opens the parameter screen, even for a routine with no parameters, because a
/// user has to be able to predict what a press does *before* making it and "selecting runs
/// it, unless it has parameters" is a rule nobody can hold. That argument is about the
/// *tap*. A hold is a distinct, deliberate gesture that cannot be made by accident, so it
/// can carry the shortcut without weakening the rule the tap follows.
///
/// **Both gates, and they are the two `activate` applies at the two ends of the same
/// journey.** Prerequisites are checked on the list row and the machine mode on the `Run`
/// row inside it; a hold skips the screen between them, so it has to apply both itself.
///
/// A refusal here is silent, and that is a considered difference from the rule that a
/// refusal is always pre-announced in the value column. That rule exists because a *row*
/// looks pressable. A hold has no visual affordance on this panel to be inconsistent with,
/// and the alternative -- greying every routine while the machine is off -- would be a
/// worse lie, because tapping one still usefully opens its parameter screen. The unrunnable
/// case is announced anyway: those rows already read `sens`.
pub fn activate_hold(row: &MenuRow, ctx: &MenuContext) -> MenuActivation {
    match row {
        // Refused rather than sent for the controller to drop: the same reason the `Run` row
        // refuses instead of closing the menu and leaving the machine cold.
        MenuRow::Routine { .. } if ctx.mode != MachineMode::On => MenuActivation::Refuse,
        MenuRow::Routine { runnable: false, .. } => MenuActivation::Refuse,
        MenuRow::Routine { index, .. } => MenuActivation::RunRoutine(*index),
        // Every other row, including `Run` -- where a tap already does this and a second
        // gesture for it would be one more thing that has to keep agreeing.
        _ => MenuActivation::Refuse,
    }
}

pub fn confirm_editor(menu: MenuId, value: f32, config: &MenuConfig) -> Option<MachineCommand> {
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
        MenuId::EditSteamTemperature => Some(MachineCommand::SetBoilerControlTargetValues(
            STEAM_BOILER,
            variegated_controller_types::BoilerControlTargetValuesUpdate {
                temperature: Some(value),
                pressure: None,
            },
        )),
        // Values only, like the boilers above and for the same reason: the mode belongs to
        // the row above this one. Which field is set follows the mode, so an editor opened
        // on pressure cannot write a flow rate.
        //
        // **Unlike the boilers, this does not reach flash.** See `activate`'s note on
        // `BrewMode`.
        MenuId::EditBrewTarget => {
            let mut update =
                variegated_controller_types::GroupBrewControlTargetValuesUpdate::default();
            // The editor cannot have been opened without a known mode -- `activate` refuses
            // -- but confirming into a guessed one would write a target the user never saw.
            let brew = config.brew?;
            match brew.mode {
                GroupBrewControlMode::Pressure => update.pressure = Some(value),
                GroupBrewControlMode::GroupFlowRate => update.flow_rate = Some(value),
                // `duty_cycle` is a `u8` and the editor works in `f32`. Rounded, not
                // truncated: the step is 1.0 from a whole number, so truncation would turn
                // a 99.999999 into 99 and make the row appear to skip a value.
                GroupBrewControlMode::FixedDutyCycle => {
                    update.duty_cycle =
                        Some(variegated_machine_menu::duty_cycle_from_editor(value))
                }
                // The mode changed while the editor was open. Writing a value into whatever
                // field the new mode uses would set a quantity the user never looked at.
                _ => return None,
            }
            Some(MachineCommand::SetGroupBrewControlTargetValues(GROUP, update))
        }
        // A routine parameter's value never leaves the button task until the routine runs.
        MenuId::EditParameter { .. } => None,
        // Not an editor. Listed rather than wildcarded so that a new editor frame is a
        // compile error here -- this was the one non-exhaustive match in the file, and a
        // missing arm is a confirm that silently does nothing.
        MenuId::Root
        | MenuId::Settings
        | MenuId::Routines
        | MenuId::RoutineParameters(_)
        | MenuId::Scale
        | MenuId::WifiInfo
        | MenuId::Bluetooth => None,
    }
}

/// An editor for a quantity identified by its unit.
///
/// Reuses `parameter_bounds`, which is keyed on [`ParameterUnit`] rather than on any
/// particular setting -- so the brew targets get the same ranges and steps a routine
/// parameter of the same unit would, and there is one table rather than two.
fn unit_adjustable(unit: ParameterUnit, current: f32) -> Adjustable {
    let (min, max, step) = variegated_machine_menu::parameter_bounds(Some(unit));
    Adjustable::new(current, min, max, step)
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

/// Everything the menu needs from `Configuration`, as published to the display tasks.
///
/// **Not part of [`MenuSnapshot`]**, which is `Copy` and must stay so -- it is a `Watch`
/// payload read on a render loop, and the associations are a `heapless::Vec` of names.
/// Separate watches also mean a configuration republished every ten seconds does not wake
/// the displays for a menu position that has not moved.
///
/// A `Watch` rather than a fourth `Configuration` subscriber because of *where* the two
/// display tasks are spawned: the character LCD's is created in `main_task`, which owns the
/// configuration channel, but the TFT's runs on core 1 and is spawned from `main` before
/// that channel exists. `MENU_WATCH` already crosses that boundary for the same pair of
/// tasks, and this mirrors it.
///
/// **The displays are one hop behind the button task**, which applies a `Configuration` to
/// itself and then publishes this. That is deliberate and bounded: it affects the brew
/// target row's *label* and the editor's unit, never a row count -- [`MenuData`]'s
/// `scale_calibration` and `bluetooth` are what decide how many rows a menu has, and the
/// first of those comes from the immutable `MachineDefinition`. A label that lags one status
/// period is a cosmetic difference; a row count that lags would put the selection on a
/// different row on each side.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct MenuConfigSnapshot {
    /// The scalars. See [`MenuConfig`].
    pub config: MenuConfig,
    /// The Bluetooth associations, for that submenu's rows. At most four.
    pub bluetooth: BluetoothPeripheralList,
}

/// Named for [`MenuSender`]'s reason.
pub type MenuConfigSender = embassy_sync::watch::Sender<
    'static,
    variegated_hal::SyncSendRawMutex,
    MenuConfigSnapshot,
    MENU_WATCH_RECEIVERS,
>;

/// Named for [`MenuSender`]'s reason.
pub type MenuConfigReceiver = embassy_sync::watch::Receiver<
    'static,
    variegated_hal::SyncSendRawMutex,
    MenuConfigSnapshot,
    MENU_WATCH_RECEIVERS,
>;
