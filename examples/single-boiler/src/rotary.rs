use core::cmp::{max, min};
use defmt::{info, Format};
use embassy_futures::select::Either::First;
use embassy_futures::select::select;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio_programs::rotary_encoder::{Direction, PioEncoder};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::channel::Sender;
use embassy_time::Timer;
use embedded_hal::digital::InputPin;
use embedded_hal_async::digital::Wait;
use variegated_controller_types::{BoilerControlTarget, DutyCycleType, GroupBrewControlTarget, MachineCommand, PidLimits, PidParameters, PidTerm, RoutineIndex, TemperatureType};
use crate::RoutineRepository;
use crate::list_menu::{ListMenuType, ListMenuState, MenuItemId};
use alloc::string::ToString;
use variegated_controller_types::SingleGroupControllerGroups::SingleGroup;

#[derive(Debug, Format, Default, Copy, Clone, PartialEq)]
pub(crate) enum ControlMode {
    #[default]
    PumpDutyCycle,
    PumpFlowRate,
    PumpPressure,
}

#[derive(Debug, Format, Default, Copy, Clone, PartialEq)]
pub(crate) enum IdleSubState {
    #[default]
    NoMenuItemSelected,
    RoutineMenuSelected,
    SettingsMenuSelected,
}

impl IdleSubState {
    pub fn rotate_clockwise(&self) -> IdleSubState {
        match self {
            IdleSubState::NoMenuItemSelected => IdleSubState::SettingsMenuSelected,
            IdleSubState::RoutineMenuSelected => IdleSubState::NoMenuItemSelected,
            IdleSubState::SettingsMenuSelected => IdleSubState::SettingsMenuSelected,
        }
    }

    pub fn rotate_counterclockwise(&self) -> IdleSubState {
        match self {
            IdleSubState::NoMenuItemSelected => IdleSubState::RoutineMenuSelected,
            IdleSubState::RoutineMenuSelected => IdleSubState::RoutineMenuSelected,
            IdleSubState::SettingsMenuSelected => IdleSubState::NoMenuItemSelected,
        }
    }
}

#[derive(Debug, Format, Default, Copy, Clone, PartialEq)]
pub(crate) enum ScaleSettingsSubState {
    #[default]
    NoneSelected,
    BackSelected,
    TareSelected,
    CalibrateZeroSelected,
    Calibrate100gSelected,
}

impl ScaleSettingsSubState {
    pub fn rotate_clockwise(&self) -> ScaleSettingsSubState {
        match self {
            ScaleSettingsSubState::NoneSelected => ScaleSettingsSubState::BackSelected,
            ScaleSettingsSubState::TareSelected => ScaleSettingsSubState::BackSelected,
            ScaleSettingsSubState::CalibrateZeroSelected => ScaleSettingsSubState::TareSelected,
            ScaleSettingsSubState::Calibrate100gSelected => ScaleSettingsSubState::CalibrateZeroSelected,
            ScaleSettingsSubState::BackSelected => ScaleSettingsSubState::BackSelected,
        }
    }

    pub fn rotate_counterclockwise(&self) -> ScaleSettingsSubState {
        match self {
            ScaleSettingsSubState::NoneSelected => ScaleSettingsSubState::TareSelected,
            ScaleSettingsSubState::TareSelected => ScaleSettingsSubState::CalibrateZeroSelected,
            ScaleSettingsSubState::CalibrateZeroSelected => ScaleSettingsSubState::Calibrate100gSelected,
            ScaleSettingsSubState::Calibrate100gSelected => ScaleSettingsSubState::TareSelected,
            ScaleSettingsSubState::BackSelected => ScaleSettingsSubState::TareSelected,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Format)]
pub(crate) enum UIState {
    Idle(IdleSubState),
    Steaming,
    ManualBrew(ControlMode),
    DispensingWater,
    RoutineExecution,
    ListMenu(ListMenuType, ListMenuState),
    SettingsInformation,
    SettingsDebugInfo,
    ScaleSettings(ScaleSettingsSubState),
}

impl Default for UIState {
    fn default() -> Self {
        UIState::Idle(IdleSubState::NoMenuItemSelected)
    }
}

#[derive(Debug, Default, Clone)]
pub(crate) struct UIStatus {
    pub(crate) state: UIState,
}

// Menu item activation handler
pub async fn handle_menu_item_activation<const N: usize>(
    item_id: MenuItemId,
    command_sender: &Sender<'_, NoopRawMutex, MachineCommand, N>,
) -> Option<UIState> {
    info!("Menu item activated: {:?}", item_id);
    let new_state = match item_id {
        MenuItemId::Routine(index) => {
            command_sender.send(MachineCommand::RunRoutine(index as RoutineIndex)).await;
            Some(UIState::RoutineExecution)
        }
        MenuItemId::SettingsInformation => Some(UIState::SettingsInformation),
        MenuItemId::SettingsDebugInfo => Some(UIState::SettingsDebugInfo),
        MenuItemId::SettingsScaleSettings => Some(UIState::ScaleSettings(ScaleSettingsSubState::default())),
    };
    
    info!("New state: {:?}", new_state);
    
    new_state
}

pub(crate) struct RotaryController<'a, C, const N: usize> where
    C: InputPin + Wait,
{
    rotary: PioEncoder<'a, PIO0, 0>,
    button: C,
    command_sender: Sender<'a, NoopRawMutex, MachineCommand, N>,
    ui_status_sender: Sender<'a, NoopRawMutex, UIStatus, N>,
    status: UIStatus,
    routine_repository: &'static RoutineRepository
}

impl<'a, C, const N: usize> RotaryController<'a, C, N>
where
    C: InputPin + Wait,
{
    pub fn new(
        rotary: PioEncoder<'a, PIO0, 0>,
        button: C,
        command_sender: Sender<'a, NoopRawMutex, MachineCommand, N>,
        ui_status_sender: Sender<'a, NoopRawMutex, UIStatus, N>,
        routine_repository: &'static RoutineRepository,
    ) -> Self {
        Self {
            rotary,
            button,
            command_sender,
            ui_status_sender,
            status: UIStatus::default(),
            routine_repository,
        }
    }

    pub async fn task(&mut self) {
        self.ui_status_sender.send(self.status.clone()).await;

        loop {
            let either = select(self.rotary.read(), self.button.wait_for_falling_edge()).await;
            if let First(direction) = either {
                match &mut self.status.state {
                    UIState::Idle(substate) => {
                        *substate = match direction {
                            Direction::Clockwise => substate.rotate_counterclockwise(),
                            Direction::CounterClockwise => substate.rotate_clockwise(),
                        };
                    }
                    UIState::ListMenu(menu_type, menu_state) => {
                        // Get total items count from centralized location
                        let item_count = menu_type.get_item_count(Some(self.routine_repository)).await;
                        let total_items = item_count + (if menu_type.has_back_button() { 1 } else { 0 });
                        
                        match direction {
                            Direction::Clockwise => menu_state.navigate_up(),
                            Direction::CounterClockwise => menu_state.navigate_down(total_items),
                        }
                    }
                    UIState::ScaleSettings(substate) => {
                        *substate = match direction {
                            Direction::Clockwise => substate.rotate_clockwise(),
                            Direction::CounterClockwise => substate.rotate_counterclockwise(),
                        };
                    }
                    _ => {}
                }

                self.ui_status_sender.send(self.status.clone()).await;
            } else {
                // Button was pressed
                match &self.status.state {
                    UIState::Idle(substate) => {
                        match substate {
                            IdleSubState::RoutineMenuSelected => {
                                let menu_state = ListMenuState::new();
                                self.status.state = UIState::ListMenu(ListMenuType::Routines, menu_state);
                            }
                            IdleSubState::SettingsMenuSelected => {
                                let menu_state = ListMenuState::new();
                                self.status.state = UIState::ListMenu(ListMenuType::Settings, menu_state);
                            }
                            _ => {}
                        }
                    }
                    UIState::ListMenu(menu_type, menu_state) => {
                        let menu_type = *menu_type;
                        let has_back_button = menu_type.has_back_button();
                        
                        // Check if back button is selected
                        if has_back_button && menu_state.is_back_button_selected() {
                            self.status.state = UIState::Idle(menu_type.get_back_state());
                        } else if let Some(item_index) = menu_state.get_selected_item_index(has_back_button) {
                            // Get MenuItemId from centralized location
                            let Some(menu_item_id) = menu_type.get_menu_item_id(item_index) else {
                                return; // Invalid index
                            };
                            
                            // Handle menu item activation
                            if let Some(new_state) = handle_menu_item_activation(
                                menu_item_id,
                                &self.command_sender
                            ).await {
                                self.status.state = new_state;
                            }
                        }
                    }
                    UIState::ScaleSettings(substate) => {
                        match substate {
                            ScaleSettingsSubState::TareSelected => {
                                self.command_sender.send(MachineCommand::TareGroupScale(SingleGroup.as_index())).await;
                            }
                            ScaleSettingsSubState::CalibrateZeroSelected => {
                                self.command_sender.send(MachineCommand::ZeroCalibrateGroupScale(SingleGroup.as_index())).await;
                            }
                            ScaleSettingsSubState::Calibrate100gSelected => {
                                self.command_sender.send(MachineCommand::CalibrateGroupScale100g(SingleGroup.as_index())).await;
                            }
                            ScaleSettingsSubState::BackSelected | ScaleSettingsSubState::NoneSelected => {
                                // Go back to Settings menu
                                self.status.state = UIState::ListMenu(ListMenuType::Settings, ListMenuState::default());
                            }
                        }
                    }
                    UIState::SettingsInformation | UIState::SettingsDebugInfo => {
                        // Go back to settings menu
                        let menu_state = ListMenuState::new();
                        self.status.state = UIState::ListMenu(ListMenuType::Settings, menu_state);
                    }
                    _ => {
                        self.status.state = UIState::Idle(IdleSubState::NoMenuItemSelected);
                    }
                }
                self.ui_status_sender.send(self.status.clone()).await;

                Timer::after_millis(300).await;
            }
        }
    }
}