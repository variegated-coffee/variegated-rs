use alloc::vec::Vec;
use alloc::vec;
use alloc::{string::{String, ToString}};
use defmt::Format;
use crate::RoutineRepository;

#[derive(Debug, Clone, Copy, PartialEq, Format)]
pub enum ListMenuType {
    Routines,
    Settings,
}

#[derive(Debug, Clone, Copy, PartialEq, Format)]
pub enum MenuItemId {
    Routine(usize),
    SettingsInformation,
    SettingsScaleSettings,
    SettingsManualBrew,
    SettingsDebugInfo,
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
        }
    }
    
    pub fn has_back_button(&self) -> bool {
        true // All list menus have back buttons for now
    }
    
    pub fn get_back_state(&self) -> crate::rotary::IdleSubState {
        match self {
            ListMenuType::Routines => crate::rotary::IdleSubState::RoutineMenuSelected,
            ListMenuType::Settings => crate::rotary::IdleSubState::SettingsMenuSelected,
        }
    }
    
    pub async fn get_items(&self, routine_repository: Option<&RoutineRepository>) -> Vec<ListMenuItem> {
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
        }
    }
}