//! What the GS3's button menu *is*.
//!
//! The mechanism -- selection, scrolling, the stack -- lives in `variegated-menu`, shared
//! with the Silvia. This file is the content, and it is deliberately the only copy of it:
//! the button task resolves an activation into a `MachineCommand` and needs row counts, the
//! two renderers need labels and value text. Neither is the other's, and a copy on each side
//! is a copy that drifts -- a selection index that means row 2 on one side and row 3 on the
//! other is a menu that runs the wrong command.

use embassy_time::{Duration, Instant};
use variegated_controller_types::{ImprovState, MachineCommand, Status};
use variegated_menu::{ListGeometry, MenuStack};

/// How deep the menu stack can go.
///
/// Only the root exists today. The depth is headroom in the *navigation*, not a claim about
/// the *definition*, and it costs a few bytes.
pub const MENU_MAX_DEPTH: usize = 4;

/// Rows on screen at once on the 428x168 TFT. See `render_menu`.
pub const MENU_VISIBLE_ROWS: usize = 4;

/// Which menu a stack frame is showing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum MenuId {
    /// The menu a long press of button 5 opens.
    Root,
}

/// What activating an item does, and what its value column reads.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum MenuItemKind {
    /// Opens or closes the Improv provisioning window.
    WifiProvisioning,
    /// Leaves the current menu; at the root that closes it.
    Exit,
}

/// One row.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct MenuItem {
    /// Drawn on the left. Keep it short: the character LCD has sixteen columns.
    pub label: &'static str,
    /// What it does.
    pub kind: MenuItemKind,
}

const ROOT_ITEMS: &[MenuItem] = &[
    MenuItem { label: "Wi-Fi Setup", kind: MenuItemKind::WifiProvisioning },
    MenuItem { label: "Exit menu", kind: MenuItemKind::Exit },
];

/// The rows of a menu, in order. The one function both sides call.
pub const fn items(menu: MenuId) -> &'static [MenuItem] {
    match menu {
        MenuId::Root => ROOT_ITEMS,
    }
}

/// The list geometry for a menu, so the button task and the renderers cannot disagree.
pub fn geometry(menu: MenuId) -> ListGeometry {
    ListGeometry {
        total_rows: items(menu).len(),
        visible_rows: MENU_VISIBLE_ROWS,
        // One physical button per direction: clamped, button 2 would do nothing on the first
        // row, and a button that does nothing reads as a broken machine.
        wrap: true,
    }
}

/// The subset of `Status` the menu reads.
///
/// A projection rather than a `&Status`, because the button task deliberately keeps derived
/// booleans rather than a `Status` copy, and a `Status` is far too large to clone onto that
/// task for one enum.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct MenuContext {
    /// Where Improv provisioning currently is.
    pub improv: ImprovState,
    /// Whether a provisioning command has been sent and `improv` has not caught up yet.
    ///
    /// Not derivable from `Status`: it says a *request* is outstanding, and only the task
    /// that sent it knows that. See `WIFI_REQUEST_TIMEOUT_MS`.
    pub wifi_pending: bool,
}

impl MenuContext {
    /// Read the projection out of a status.
    ///
    /// `wifi_pending` comes from the button task rather than from the status, for the reason
    /// on the field.
    pub fn from_status(status: &Status, wifi_pending: bool) -> Self {
        Self {
            improv: status.comms_status.as_ref().map(|c| c.improv).unwrap_or_default(),
            wifi_pending,
        }
    }
}

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

/// The value column, or `None` for an item that has no value.
pub fn value_text(item: &MenuItem, ctx: &MenuContext) -> Option<&'static str> {
    match item.kind {
        MenuItemKind::WifiProvisioning => Some(if ctx.wifi_pending {
            // The window takes about a second to open or close, and until it does
            // `comms_status.improv` still reports the old state. Showing that reads as
            // "nothing happened" and invites a second press.
            "..."
        } else {
            match ctx.improv {
                ImprovState::Stopped => "OFF",
                _ => "ON",
            }
        }),
        MenuItemKind::Exit => None,
    }
}

/// What activating an item asks the caller to do.
///
/// When adjustable settings arrive this gains an `Enter(MenuId)`, and `MenuStack::push`
/// and `variegated_menu::Adjustable` are already there to serve it. It does not gain one
/// now: an unused variant in a binary crate is a warning, and zero warnings is not optional
/// here.
pub enum MenuActivation {
    /// Send this command. The menu stays where it is and its value column updates from the
    /// next `Status` -- which is the whole reason the `Watch` carries navigation rather than
    /// a rendered view.
    Command(MachineCommand),
    /// Leave this menu.
    Pop,
}

/// Five minutes -- unchanged from the button-6 hold this replaces. Long enough to fetch a
/// phone and type a password, short enough that a window opened by accident closes itself
/// long before anyone would notice it was open.
const PROVISIONING_WINDOW_MS: u32 = 300_000;

/// Decide what activating an item does.
pub fn activate(item: &MenuItem, ctx: &MenuContext) -> MenuActivation {
    match item.kind {
        MenuItemKind::WifiProvisioning => MenuActivation::Command(match ctx.improv {
            ImprovState::Stopped => MachineCommand::OpenWifiProvisioningWindow {
                duration_ms: PROVISIONING_WINDOW_MS,
            },
            _ => MachineCommand::CloseWifiProvisioningWindow,
        }),
        MenuItemKind::Exit => MenuActivation::Pop,
    }
}

/// The GS3's menu stack, as published and as drawn.
pub type GsMenu = MenuStack<MenuId, MENU_MAX_DEPTH>;

/// What the button task publishes for the displays to draw.
///
/// Navigation plus the one piece of state the renderers cannot derive. It is deliberately
/// *not* a rendered view: the value column is still computed per frame from live `Status`,
/// which is what lets `OFF` become `ON` about a second after activation with no button
/// pressed in between.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, defmt::Format)]
pub struct MenuSnapshot {
    /// Where the user is.
    pub stack: GsMenu,
    /// Whether a provisioning command is still unconfirmed. See [`MenuContext::wifi_pending`].
    pub wifi_pending: bool,
}

impl MenuSnapshot {
    /// Nothing open, nothing outstanding.
    pub fn closed() -> Self {
        Self { stack: GsMenu::closed(), wifi_pending: false }
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
