use alloc::vec::Vec;
use alloc::vec;
use alloc::{string::{String, ToString}};
use defmt::Format;
use variegated_controller_types::Status;
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
    Routine(usize),
    SettingsInformation,
    SettingsScaleSettings,
    SettingsManualBrew,
    SettingsDebugInfo,
    SettingsBoilerTemperature,
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
    pub id: MenuItemId,
}

// State only contains navigation state, not the items
#[derive(Debug, Clone, Copy, Default, Format, PartialEq)]
pub struct ListMenuState {
    pub selected_index: usize,
    pub scroll_offset: usize,
}

impl ListMenuState {
    pub const VISIBLE_ITEMS: usize = 5;
    
    pub fn new() -> Self {
        Self {
            selected_index: 0,
            scroll_offset: 0,
        }
    }
    
    pub fn navigate_up(&mut self) {
        if self.selected_index > 0 {
            self.selected_index -= 1;
            
            // Adjust scroll offset if needed
            if self.selected_index < self.scroll_offset + 1 && self.scroll_offset > 0 {
                self.scroll_offset -= 1;
            }
        }
    }
    
    pub fn navigate_down(&mut self, total_items: usize) {
        if self.selected_index < total_items - 1 {
            self.selected_index += 1;
            
            // Adjust scroll offset if needed
            if self.selected_index >= self.scroll_offset + Self::VISIBLE_ITEMS - 1 
               && self.scroll_offset + Self::VISIBLE_ITEMS < total_items {
                self.scroll_offset += 1;
            }
        }
    }
    
    pub fn is_back_button_selected(&self) -> bool {
        self.selected_index == 0
    }
    
    pub fn get_selected_item_index(&self, has_back_button: bool) -> Option<usize> {
        if has_back_button && self.selected_index == 0 {
            None // Back button is selected
        } else {
            Some(self.selected_index - (if has_back_button { 1 } else { 0 }))
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
                crate::rotary::UIState::ListMenu(ListMenuType::Settings, ListMenuState::new(), None)
            },
            ListMenuType::PidTermConfig(pid_type, _) => {
                // Go back to PID Config menu for this PID type
                crate::rotary::UIState::ListMenu(ListMenuType::PidConfig(*pid_type), ListMenuState::new(), None)
            },
        }
    }
    
    pub async fn get_items(&self, routine_repository: Option<&RoutineRepository>, _status: Option<&Status>) -> Vec<ListMenuItem> {
        match self {
            ListMenuType::Routines => {
                if let Some(rr) = routine_repository {
                    let repo = rr.lock().await;
                    repo.iterate_routines()
                        .enumerate()
                        .map(|(index, routine)| ListMenuItem {
                            label: routine.name().to_string(),
                            id: MenuItemId::Routine(index),
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
                    })
                    .collect()
            }
            ListMenuType::PidConfig(_pid_type) => {
                vec![
                    ListMenuItem { label: "kP".to_string(), id: MenuItemId::PidTerm(PidTermType::Kp) },
                    ListMenuItem { label: "kI".to_string(), id: MenuItemId::PidTerm(PidTermType::Ki) },
                    ListMenuItem { label: "kD".to_string(), id: MenuItemId::PidTerm(PidTermType::Kd) },
                    ListMenuItem { label: "Reset parameters".to_string(), id: MenuItemId::PidResetParameters },
                ]
            }
            ListMenuType::PidTermConfig(_pid_type, _term) => {
                // TODO: Get current values from status and display them
                vec![
                    ListMenuItem { label: "Positive Scale".to_string(), id: MenuItemId::PidComponent(PidComponentType::PositiveScale) },
                    ListMenuItem { label: "Negative Scale".to_string(), id: MenuItemId::PidComponent(PidComponentType::NegativeScale) },
                    ListMenuItem { label: "Upper Limit".to_string(), id: MenuItemId::PidComponent(PidComponentType::UpperLimit) },
                    ListMenuItem { label: "Lower Limit".to_string(), id: MenuItemId::PidComponent(PidComponentType::LowerLimit) },
                ]
            }
        }
    }
    
    pub async fn get_item_count(&self, routine_repository: Option<&RoutineRepository>) -> usize {
        match self {
            ListMenuType::Routines => {
                if let Some(rr) = routine_repository {
                    let repo = rr.lock().await;
                    repo.get_routine_count()
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
                Some(MenuItemId::Routine(item_index))
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