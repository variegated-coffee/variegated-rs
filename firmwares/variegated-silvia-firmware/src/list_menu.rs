use alloc::vec::Vec;
use alloc::vec;
use alloc::{string::{String, ToString}};
use defmt::Format;
use variegated_controller_types::{Status, RoutineIndex};
use variegated_controller_lib::routine::RoutineRepository as RoutineRepositoryTrait;
use variegated_machine_menu::routine_rows;
use variegated_menu::{ListGeometry, ListNav};
use crate::RoutineRepository;

#[derive(Debug, Clone, Copy, PartialEq, Format)]
pub enum ListMenuType {
    Routines,
    Settings,
    PidConfig(PidConfigType),           // PID main menu (kP, kI, kD)
    PidTermConfig(PidConfigType, PidTermType), // PID term submenu
}

#[derive(Debug, Clone, Copy, PartialEq, Format)]
pub enum PidConfigType {
    BoilerTemperature,
    PumpFlowRate,
    PumpOutputFlowRate,
    PumpPressure,
}

#[derive(Debug, Clone, Copy, PartialEq, Format)]
pub enum PidTermType {
    Kp,
    Ki,
    Kd,
}

#[derive(Debug, Clone, Copy, PartialEq, Format)]
pub enum PidComponentType {
    PositiveScale,
    NegativeScale,
    UpperLimit,
    LowerLimit,
}

#[derive(Debug, Clone, Copy, PartialEq, Format)]
pub enum MenuItemId {
    Routine(RoutineIndex),
    SettingsInformation,
    SettingsScaleSettings,
    SettingsWifiProvisioning,
    SettingsManualBrew,
    SettingsDebugInfo,
    SettingsBoilerTemperature,
    /// The target the single element holds in steam mode — boiler index 1, the virtual
    /// steam boiler. Nothing on this display could reach it before, which is how it stayed
    /// at its `Off` default and made the steam switch a no-op.
    SettingsSteamTemperature,
    SettingsBoilerTemperaturePID,
    SettingsPumpFlowRatePID,
    SettingsPumpOutputFlowRatePID,
    SettingsPumpPressurePID,
    PidTerm(PidTermType),
    PidComponent(PidComponentType),
    PidResetParameters,
}

struct SettingsMenuDefinition {
    label: &'static str,
    id: MenuItemId,
}

const SETTINGS_MENU_ITEMS: &[SettingsMenuDefinition] = &[
    SettingsMenuDefinition {
        label: "Information",
        id: MenuItemId::SettingsInformation,
    },
    SettingsMenuDefinition {
        label: "Scale Settings", 
        id: MenuItemId::SettingsScaleSettings,
    },
    SettingsMenuDefinition {
        label: "WiFi Setup",
        id: MenuItemId::SettingsWifiProvisioning,
    },
    SettingsMenuDefinition {
        label: "Manual Brew",
        id: MenuItemId::SettingsManualBrew,
    },
    SettingsMenuDefinition {
        label: "Debug info",
        id: MenuItemId::SettingsDebugInfo,
    },
    SettingsMenuDefinition {
        label: "Boiler Temperature",
        id: MenuItemId::SettingsBoilerTemperature,
    },
    SettingsMenuDefinition {
        label: "Steam Temperature",
        id: MenuItemId::SettingsSteamTemperature,
    },
    SettingsMenuDefinition {
        label: "Boiler Temp PID",
        id: MenuItemId::SettingsBoilerTemperaturePID,
    },
    SettingsMenuDefinition {
        label: "Pump Flow PID",
        id: MenuItemId::SettingsPumpFlowRatePID,
    },
    SettingsMenuDefinition {
        label: "Pump Output PID",
        id: MenuItemId::SettingsPumpOutputFlowRatePID,
    },
    SettingsMenuDefinition {
        label: "Pump Pressure PID",
        id: MenuItemId::SettingsPumpPressurePID,
    },
];

#[derive(Debug, Clone)]
pub struct ListMenuItem {
    pub label: String,
    /// What activating this row does.
    ///
    /// This is what activation resolves through, and it is the only way a routine row can be
    /// resolved at all: [`ListMenuType::get_menu_item_id`] maps a *position* to an id from
    /// static tables, and for `Routines` there is nothing to map to -- a `RoutineIndex` cannot
    /// be recovered from a row number, so it returns `None` unconditionally. The id is captured
    /// here when the items are fetched, while the index is still in hand.
    pub id: MenuItemId,
    /// Whether activating this row will do anything.
    ///
    /// Only routines can be un-runnable -- everything else in these menus is a setting -- so
    /// every other row builds this as `true`. Captured at fetch time alongside the id, and
    /// for the same reason: the row is what activation and drawing both see, and they must
    /// agree.
    pub runnable: bool,
}

/// Rows on screen at once on the 128x64 panel.
pub const VISIBLE_ROWS: usize = 5;

impl ListMenuType {
    /// Every row the user can land on: the back row plus one per item.
    ///
    /// **This is the whole fix.** The old code counted the back button in the bounds check
    /// and excluded it from the render window, which over-scrolled by one at the end of every
    /// list, left a permanently blank bottom row, and drove the scrollbar thumb off the
    /// bottom of a 64-pixel panel. One index space, counted once.
    pub fn geometry(&self, item_count: usize) -> ListGeometry {
        ListGeometry {
            total_rows: item_count + if self.has_back_button() { 1 } else { 0 },
            visible_rows: VISIBLE_ROWS,
            // A knob does not need wrapping, and wrapping a long settings list on one is
            // disorienting. The GS3 wraps because it has one button per direction.
            wrap: false,
        }
    }

    /// The item a row refers to, or `None` for the back row.
    pub fn item_index(&self, row: usize) -> Option<usize> {
        if self.has_back_button() {
            row.checked_sub(1)
        } else {
            Some(row)
        }
    }
}

impl ListMenuType {
    pub fn get_title(&self) -> &'static str {
        match self {
            ListMenuType::Routines => "Routines",
            ListMenuType::Settings => "Settings",
            ListMenuType::PidConfig(pid_type) => match pid_type {
                PidConfigType::BoilerTemperature => "Boiler Temp PID",
                PidConfigType::PumpFlowRate => "Pump Flow PID",
                PidConfigType::PumpOutputFlowRate => "Pump Output PID",
                PidConfigType::PumpPressure => "Pump Pressure PID",
            },
            ListMenuType::PidTermConfig(pid_type, term) => match (pid_type, term) {
                (PidConfigType::BoilerTemperature, PidTermType::Kp) => "Boiler Temp kP",
                (PidConfigType::BoilerTemperature, PidTermType::Ki) => "Boiler Temp kI",
                (PidConfigType::BoilerTemperature, PidTermType::Kd) => "Boiler Temp kD",
                (PidConfigType::PumpFlowRate, PidTermType::Kp) => "Flow Rate kP",
                (PidConfigType::PumpFlowRate, PidTermType::Ki) => "Flow Rate kI",
                (PidConfigType::PumpFlowRate, PidTermType::Kd) => "Flow Rate kD",
                (PidConfigType::PumpOutputFlowRate, PidTermType::Kp) => "Output Flow kP",
                (PidConfigType::PumpOutputFlowRate, PidTermType::Ki) => "Output Flow kI",
                (PidConfigType::PumpOutputFlowRate, PidTermType::Kd) => "Output Flow kD",
                (PidConfigType::PumpPressure, PidTermType::Kp) => "Pressure kP",
                (PidConfigType::PumpPressure, PidTermType::Ki) => "Pressure kI",
                (PidConfigType::PumpPressure, PidTermType::Kd) => "Pressure kD",
            },
        }
    }
    
    pub fn has_back_button(&self) -> bool {
        true // All list menus have back buttons for now
    }
    
    pub fn get_back_state(&self) -> crate::rotary::UIState {
        match self {
            ListMenuType::Routines => crate::rotary::UIState::Idle(crate::rotary::IdleSubState::RoutineMenuSelected),
            ListMenuType::Settings => crate::rotary::UIState::Idle(crate::rotary::IdleSubState::SettingsMenuSelected),
            ListMenuType::PidConfig(_) => {
                // Go back to Settings menu
                crate::rotary::UIState::ListMenu(ListMenuType::Settings, ListNav::new(), None, None)
            },
            ListMenuType::PidTermConfig(pid_type, _) => {
                // Go back to PID Config menu for this PID type
                crate::rotary::UIState::ListMenu(ListMenuType::PidConfig(*pid_type), ListNav::new(), None, None)
            },
        }
    }
    
    /// `status` is what decides whether each routine can currently run. It has always been
    /// in this signature and always been ignored; it is now used. `None` means "cannot tell",
    /// which reads as runnable -- the controller's own backstop still refuses anything that
    /// really cannot run, and greying every routine because a caller passed `None` would be
    /// the worse lie.
    pub async fn get_items(&self, routine_repository: Option<&RoutineRepository>, status: Option<&Status>) -> Vec<ListMenuItem> {
        match self {
            ListMenuType::Routines => {
                if let Some(rr) = routine_repository {
                    let definition = *crate::MACHINE_DEFINITION_REF.lock().await;
                    let mut repo = rr.lock().await;
                    // Ordering, filtering and name truncation all live in
                    // `variegated-machine-menu`, shared with the GS3 and host-tested there.
                    // `include_function: true` keeps this board's behaviour: it has no
                    // hardware routine buttons, so it shows everything it has.
                    routine_rows(repo.iterate_routines_with_indices().await, true, |routine| {
                        match (definition, status) {
                            (Some(definition), Some(status)) => {
                                variegated_controller_lib::routine_prerequisites::prerequisites_satisfied(
                                    &routine.prerequisites,
                                    definition,
                                    &status.peripheral_status,
                                )
                            }
                            _ => true,
                        }
                    })
                        .into_iter()
                        .map(|row| ListMenuItem {
                            label: row.name.as_str().to_string(),
                            id: MenuItemId::Routine(row.index),
                            runnable: row.runnable,
                        })
                        .collect::<Vec<_>>()
                } else {
                    Vec::new()
                }
            }
            ListMenuType::Settings => {
                SETTINGS_MENU_ITEMS.iter()
                    .map(|item| ListMenuItem {
                        label: item.label.to_string(),
                        id: item.id,
                        // Settings are always actionable; only routines can be gated.
                        runnable: true,
                    })
                    .collect()
            }
            ListMenuType::PidConfig(_pid_type) => {
                vec![
                    ListMenuItem { label: "kP".to_string(), id: MenuItemId::PidTerm(PidTermType::Kp), runnable: true },
                    ListMenuItem { label: "kI".to_string(), id: MenuItemId::PidTerm(PidTermType::Ki), runnable: true },
                    ListMenuItem { label: "kD".to_string(), id: MenuItemId::PidTerm(PidTermType::Kd), runnable: true },
                    ListMenuItem { label: "Reset parameters".to_string(), id: MenuItemId::PidResetParameters, runnable: true },
                ]
            }
            ListMenuType::PidTermConfig(_pid_type, _term) => {
                // TODO: Get current values from status and display them
                vec![
                    ListMenuItem { label: "Positive Scale".to_string(), id: MenuItemId::PidComponent(PidComponentType::PositiveScale), runnable: true },
                    ListMenuItem { label: "Negative Scale".to_string(), id: MenuItemId::PidComponent(PidComponentType::NegativeScale), runnable: true },
                    ListMenuItem { label: "Upper Limit".to_string(), id: MenuItemId::PidComponent(PidComponentType::UpperLimit), runnable: true },
                    ListMenuItem { label: "Lower Limit".to_string(), id: MenuItemId::PidComponent(PidComponentType::LowerLimit), runnable: true },
                ]
            }
        }
    }
    
    pub async fn get_item_count(&self, routine_repository: Option<&RoutineRepository>) -> usize {
        match self {
            ListMenuType::Routines => {
                if let Some(rr) = routine_repository {
                    let mut repo = rr.lock().await;
                    // Counted the same way `get_items` builds the list, not with
                    // `get_routine_count`. The two disagree once anything is filtered or
                    // capped -- and a count that exceeds the rows drawn is exactly the
                    // over-scroll this file's `geometry` comment is about.
                    // `|_| true`: this counts rows, and an unrunnable routine still has one.
                    // Greying is what marks it, not omission -- see `RoutineRow::runnable`.
                    routine_rows(repo.iterate_routines_with_indices().await, true, |_| true).len()
                } else {
                    0
                }
            }
            ListMenuType::Settings => SETTINGS_MENU_ITEMS.len(),
            ListMenuType::PidConfig(_) => 4, // kP, kI, kD, Reset parameters
            ListMenuType::PidTermConfig(_, _) => 4, // Positive Scale, Negative Scale, Upper Limit, Lower Limit
        }
    }
    
    pub fn get_menu_item_id(&self, item_index: usize) -> Option<MenuItemId> {
        match self {
            ListMenuType::Routines => {
                // Cannot determine RoutineIndex from item position alone
                // Caller should use cached menu items instead
                None
            }
            ListMenuType::Settings => {
                SETTINGS_MENU_ITEMS.get(item_index).map(|item| item.id)
            }
            ListMenuType::PidConfig(_) => {
                match item_index {
                    0 => Some(MenuItemId::PidTerm(PidTermType::Kp)),
                    1 => Some(MenuItemId::PidTerm(PidTermType::Ki)),
                    2 => Some(MenuItemId::PidTerm(PidTermType::Kd)),
                    3 => Some(MenuItemId::PidResetParameters),
                    _ => None,
                }
            }
            ListMenuType::PidTermConfig(_, _) => {
                match item_index {
                    0 => Some(MenuItemId::PidComponent(PidComponentType::PositiveScale)),
                    1 => Some(MenuItemId::PidComponent(PidComponentType::NegativeScale)),
                    2 => Some(MenuItemId::PidComponent(PidComponentType::UpperLimit)),
                    3 => Some(MenuItemId::PidComponent(PidComponentType::LowerLimit)),
                    _ => None,
                }
            }
        }
    }
}