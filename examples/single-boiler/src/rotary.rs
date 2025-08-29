use core::cmp::{max, min};
use defmt::{info, warn, Format};
use embassy_futures::select::Either::{First, Second};
use embassy_futures::select::{select, select3, Either3};
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio_programs::rotary_encoder::{Direction, PioEncoder};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::channel::Sender;
use embassy_time::Timer;
use embedded_hal::digital::InputPin;
use embedded_hal_async::digital::Wait;
use variegated_controller_types::{BoilerControlTarget, DutyCycleType, GroupBrewControlTarget, MachineCommand, PidLimits, PidParameters, PidTerm, RoutineIndex, TemperatureType, Status};
use crate::{RoutineRepository, StatusSubscriber};
use crate::list_menu::{ListMenuType, ListMenuState, MenuItemId};
use alloc::string::ToString;
use variegated_controller_types::SingleGroupControllerGroups::SingleGroup;
use variegated_controller_lib::routine::{ParameterUnit, Routine, RoutineParameters};
use alloc::string::String;

#[derive(Debug, Format, Default, Copy, Clone, PartialEq)]
pub(crate) enum ControlMode {
    #[default]
    PumpDutyCycle,
    PumpFlowRate,
    PumpPressure,
}

impl ControlMode {
    pub fn next(&self) -> ControlMode {
        match self {
            ControlMode::PumpDutyCycle => ControlMode::PumpFlowRate,
            ControlMode::PumpFlowRate => ControlMode::PumpPressure,
            ControlMode::PumpPressure => ControlMode::PumpDutyCycle,
        }
    }

    pub fn display_name(&self) -> &'static str {
        match self {
            ControlMode::PumpDutyCycle => "Duty Cycle",
            ControlMode::PumpFlowRate => "Flow Rate", 
            ControlMode::PumpPressure => "Pressure",
        }
    }

    pub fn unit(&self) -> &'static str {
        match self {
            ControlMode::PumpDutyCycle => "%",
            ControlMode::PumpFlowRate => "ml/s",
            ControlMode::PumpPressure => "bar",
        }
    }
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

#[derive(Debug, Clone)]
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
    RoutineParameters(RoutineIndex, RoutineParameterEditState),
    ParameterManipulation {
        edit_state: RoutineParameterEditState,
        routine_index: RoutineIndex,
        param_index: u8,
        current_value: f32,
        param_name: String,
        param_unit: Option<ParameterUnit>,
    },
}

impl Default for UIState {
    fn default() -> Self {
        UIState::Idle(IdleSubState::NoMenuItemSelected)
    }
}

#[derive(Debug, Default, Clone)]
pub(crate) struct ManualBrewParameters {
    pub duty_cycle: u8,    // 0-100%
    pub flow_rate: f32,    // ml/s
    pub pressure: f32,     // bar
}

impl ManualBrewParameters {
    pub fn new() -> Self {
        Self {
            duty_cycle: 0,
            flow_rate: 5.0,
            pressure: 9.0,
        }
    }

    pub fn get_value(&self, mode: ControlMode) -> f32 {
        match mode {
            ControlMode::PumpDutyCycle => self.duty_cycle as f32,
            ControlMode::PumpFlowRate => self.flow_rate,
            ControlMode::PumpPressure => self.pressure,
        }
    }

    pub fn adjust_value(&mut self, mode: ControlMode, increment: bool) {
        match mode {
            ControlMode::PumpDutyCycle => {
                if increment {
                    self.duty_cycle = (self.duty_cycle + 5).min(100);
                } else {
                    self.duty_cycle = self.duty_cycle.saturating_sub(5);
                }
            }
            ControlMode::PumpFlowRate => {
                if increment {
                    self.flow_rate = (self.flow_rate + 0.5).min(50.0);
                } else {
                    self.flow_rate = (self.flow_rate - 0.5).max(0.0);
                }
            }
            ControlMode::PumpPressure => {
                if increment {
                    self.pressure = (self.pressure + 0.5).min(15.0);
                } else {
                    self.pressure = (self.pressure - 0.5).max(0.0);
                }
            }
        }
    }

    pub fn to_group_brew_control_target(&self, mode: ControlMode) -> GroupBrewControlTarget {
        match mode {
            ControlMode::PumpDutyCycle => {
                if self.duty_cycle == 0 {
                    GroupBrewControlTarget::Off
                } else {
                    GroupBrewControlTarget::FixedDutyCycle(self.duty_cycle)
                }
            }
            ControlMode::PumpFlowRate => {
                if self.flow_rate <= 0.0 {
                    GroupBrewControlTarget::Off
                } else {
                    GroupBrewControlTarget::GroupFlowRate(self.flow_rate)
                }
            }
            ControlMode::PumpPressure => {
                if self.pressure <= 0.0 {
                    GroupBrewControlTarget::Off
                } else {
                    GroupBrewControlTarget::Pressure(self.pressure)
                }
            }
        }
    }

    pub fn sync_from_process_values(&mut self, group_status: &variegated_controller_types::GroupStatus) {
        // Update duty cycle from current pump output (clamp to 0-100%)
        let current_duty = group_status.pump_output.duty_cycle();
        // Round to nearest 5% increment
        self.duty_cycle = ((current_duty + 2) / 5) * 5; // Integer rounding to 5% increments
        
        // Update flow rate from current process value (prefer output flow rate, fallback to input)
        if let Some(flow) = group_status.output_flow_rate.or(group_status.input_flow_rate) {
            // Clamp to valid range (0.0 to 50.0 ml/s) and round to nearest 0.5 increment
            let clamped_flow = flow.max(0.0).min(50.0);
            self.flow_rate = ((clamped_flow + 0.25) / 0.5) as u32 as f32 * 0.5; // Round to 0.5 ml/s increments
        }
        
        // Update pressure from current process value  
        if let Some(pressure) = group_status.pressure {
            // Clamp to valid range (0.0 to 15.0 bar) and round to nearest 0.5 increment
            let clamped_pressure = pressure.max(0.0).min(15.0);
            self.pressure = ((clamped_pressure + 0.25) / 0.5) as u32 as f32 * 0.5; // Round to 0.5 bar increments
        }
    }
}

#[derive(Debug, Clone)]
pub(crate) struct RoutineParameterEditState {
    pub selected_index: usize,
    pub scroll_offset: usize,
    pub parameter_values: RoutineParameters,
    pub routine_name: String,
}

impl RoutineParameterEditState {
    pub const VISIBLE_ITEMS: usize = 5; // Same as ListMenuState
    
    pub fn new(routine: &Routine) -> Self {
        // Initialize with default values from routine parameters
        let mut parameter_values = RoutineParameters::new();
        for param in routine.parameters() {
            let _ = parameter_values.insert(param.index, param.default);
        }
        
        Self {
            selected_index: 0, // Start with back button selected
            scroll_offset: 0,
            parameter_values,
            routine_name: routine.name().to_string(),
        }
    }
    
    pub fn is_back_button_selected(&self) -> bool {
        self.selected_index == 0
    }
    
    pub fn is_execute_selected(&self, routine: &Routine) -> bool {
        self.selected_index == routine.parameters().len() + 1 // After back + parameters
    }
    
    pub fn get_selected_param_index(&self, routine: &Routine) -> Option<u8> {
        if self.selected_index == 0 || self.is_execute_selected(routine) {
            None // Back button or Execute selected
        } else {
            // Adjust index for back button offset (same logic as ListMenuState)
            routine.parameters().get(self.selected_index - 1).map(|p| p.index)
        }
    }
    
    pub fn get_total_items(&self, routine: &Routine) -> usize {
        1 + routine.parameters().len() + 1 // Back + Parameters + Execute
    }
    
    pub fn navigate_up(&mut self) {
        if self.selected_index > 0 {
            self.selected_index -= 1;
            
            // Adjust scroll offset if needed (same logic as ListMenuState)
            if self.selected_index < self.scroll_offset + 1 && self.scroll_offset > 0 {
                self.scroll_offset -= 1;
            }
        }
    }
    
    pub fn navigate_down(&mut self, total_items: usize) {
        if self.selected_index < total_items - 1 {
            self.selected_index += 1;
            
            // Adjust scroll offset if needed (same logic as ListMenuState)
            if self.selected_index >= self.scroll_offset + Self::VISIBLE_ITEMS - 1 
               && self.scroll_offset + Self::VISIBLE_ITEMS < total_items {
                self.scroll_offset += 1;
            }
        }
    }
}

#[derive(Debug, Default, Clone)]
pub(crate) struct UIStatus {
    pub(crate) state: UIState,
    pub(crate) manual_brew_parameters: ManualBrewParameters,
}

// Menu item activation handler
pub async fn handle_menu_item_activation(
    item_id: MenuItemId,
    routine_repository: &RoutineRepository,
) -> Option<UIState> {
    info!("Menu item activated: {:?}", item_id);
    let new_state = match item_id {
        MenuItemId::Routine(index) => {
            // Load routine definition and transition to parameter view
            let repo = routine_repository.lock().await;
            if let Some(routine) = repo.get_routine(index) {
                let edit_state = RoutineParameterEditState::new(routine);
                Some(UIState::RoutineParameters(index as RoutineIndex, edit_state))
            } else {
                // Handle missing routine gracefully
                None
            }
        }
        MenuItemId::SettingsInformation => Some(UIState::SettingsInformation),
        MenuItemId::SettingsDebugInfo => Some(UIState::SettingsDebugInfo),
        MenuItemId::SettingsScaleSettings => Some(UIState::ScaleSettings(ScaleSettingsSubState::default())),
        MenuItemId::SettingsManualBrew => Some(UIState::ManualBrew(ControlMode::default())),
    };
    
    info!("New state set");
    
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
    routine_repository: &'static RoutineRepository,
    status_receiver: StatusSubscriber,
    previous_brewing_state: bool,
    current_status: Status,
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
        status_receiver: StatusSubscriber,
    ) -> Self {
        let mut status = UIStatus::default();
        status.manual_brew_parameters = ManualBrewParameters::new();
        
        Self {
            rotary,
            button,
            command_sender,
            ui_status_sender,
            status,
            routine_repository,
            status_receiver,
            previous_brewing_state: false,
            current_status: Status::default(),
        }
    }


    pub async fn task(&mut self) {
        self.ui_status_sender.send(self.status.clone()).await;

        loop {
            let either3 = select3(
                self.rotary.read(),
                self.button.wait_for_falling_edge(),
                self.status_receiver.next_message_pure()
            ).await;
            match either3 {
                Either3::First(direction) => {
                    // Rotary encoder was turned
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
                    UIState::ManualBrew(control_mode) => {
                        // Adjust the parameter value for the current control mode
                        let increment = match direction {
                            Direction::CounterClockwise => true,  // CounterClockwise increments
                            Direction::Clockwise => false,        // Clockwise decrements
                        };
                        
                        self.status.manual_brew_parameters.adjust_value(*control_mode, increment);
                        
                        // Send command to update the group brew control target
                        let target = self.status.manual_brew_parameters.to_group_brew_control_target(*control_mode);
                        self.command_sender.send(
                            MachineCommand::SetGroupBrewControlTarget(SingleGroup.as_index(), target)
                        ).await;
                    }
                    UIState::RoutineParameters(routine_index, edit_state) => {
                        // Navigate through parameter list (same logic as ListMenu)
                        let repo = self.routine_repository.lock().await;
                        if let Some(routine) = repo.get_routine(*routine_index) {
                            let total_items = edit_state.get_total_items(routine);
                            match direction {
                                Direction::Clockwise => edit_state.navigate_up(),
                                Direction::CounterClockwise => edit_state.navigate_down(total_items),
                            }
                        }
                    }
                    UIState::ParameterManipulation { current_value, .. } => {
                        // Adjust parameter value in 0.5 increments
                        let increment = match direction {
                            Direction::CounterClockwise => true,  // CounterClockwise increments
                            Direction::Clockwise => false,        // Clockwise decrements
                        };
                        
                        if increment {
                            *current_value += 0.5;
                        } else {
                            *current_value = (*current_value - 0.5).max(0.0); // Don't go below 0
                        }
                    }
                    _ => {}
                }

                self.ui_status_sender.send(self.status.clone()).await;
                }
                Either3::Second(_) => {
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
                                self.routine_repository
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
                    UIState::ManualBrew(control_mode) => {
                        // Sync parameters from current process values before switching modes
                        if let Some(group_status) = self.current_status.get_group_status(SingleGroup.as_index()) {
                            self.status.manual_brew_parameters.sync_from_process_values(group_status);
                        }
                        
                        // Cycle to the next control mode
                        let new_mode = control_mode.next();
                        self.status.state = UIState::ManualBrew(new_mode);
                        
                        // Send command to update the group brew control target with new mode
                        let target = self.status.manual_brew_parameters.to_group_brew_control_target(new_mode);
                        self.command_sender.send(
                            MachineCommand::SetGroupBrewControlTarget(SingleGroup.as_index(), target)
                        ).await;
                    }
                    UIState::SettingsInformation | UIState::SettingsDebugInfo => {
                        // Go back to settings menu
                        let menu_state = ListMenuState::new();
                        self.status.state = UIState::ListMenu(ListMenuType::Settings, menu_state);
                    }
                    UIState::RoutineExecution => {
                        // Cancel the currently running routine
                        self.command_sender.send(MachineCommand::CancelRoutine).await;
                        self.status.state = UIState::Idle(IdleSubState::NoMenuItemSelected);
                    }
                    UIState::RoutineParameters(routine_index, edit_state) => {
                        let routine_index = *routine_index;
                        let repo = self.routine_repository.lock().await;
                        if let Some(routine) = repo.get_routine(routine_index) {
                            if edit_state.is_back_button_selected() {
                                // Back button selected - return to routine menu
                                let menu_state = ListMenuState::new();
                                self.status.state = UIState::ListMenu(ListMenuType::Routines, menu_state);
                            } else if edit_state.is_execute_selected(routine) {
                                // Execute button selected - run routine with current parameters
                                let runtime_params = if edit_state.parameter_values.is_empty() {
                                    None
                                } else {
                                    Some(edit_state.parameter_values.clone())
                                };
                                self.command_sender.send(MachineCommand::RunRoutine(routine_index, runtime_params)).await;
                                self.status.state = UIState::RoutineExecution;
                            } else if let Some(param_index) = edit_state.get_selected_param_index(routine) {
                                // Parameter selected - enter manipulation mode
                                if let Some(param) = routine.parameters().iter().find(|p| p.index == param_index) {
                                    let current_value = edit_state.parameter_values.get(&param_index).copied().unwrap_or(param.default);
                                    self.status.state = UIState::ParameterManipulation {
                                        edit_state: edit_state.clone(),
                                        routine_index,
                                        param_index,
                                        current_value,
                                        param_name: param.name.clone(),
                                        param_unit: param.unit,
                                    };
                                }
                            }
                        }
                    }
                    UIState::ParameterManipulation { edit_state, routine_index, param_index, current_value, .. } => {
                        // Return to parameter list with updated value
                        let routine_index = *routine_index;
                        let param_index = *param_index;
                        let current_value = *current_value;
                        
                        // Use the preserved edit state and update only the current parameter
                        let mut preserved_edit_state = edit_state.clone();
                        preserved_edit_state.parameter_values.insert(param_index, current_value);
                        
                        self.status.state = UIState::RoutineParameters(routine_index, preserved_edit_state);
                    }
                    _ => {
                        self.status.state = UIState::Idle(IdleSubState::NoMenuItemSelected);
                    }
                }
                self.ui_status_sender.send(self.status.clone()).await;

                Timer::after_millis(300).await;
                }
                Either3::Third(status_update) => {
                    // Status update received - handle automatic UI switching
                    self.current_status = status_update;
                    
                    // Get current brewing state
                    let current_brewing = self.current_status
                        .get_group_status(SingleGroup.as_index())
                        .map_or(false, |s| s.is_brewing);

                    let routine_running = self.current_status.routine_execution.is_some();

                    // Handle automatic UI switching for Manual Brew entry
                    if !self.previous_brewing_state && current_brewing && 
                       !routine_running &&
                       !matches!(self.status.state, UIState::ManualBrew(_) | UIState::RoutineExecution) {
                        warn!("Switching to Manual Brew mode due to brewing start");
                        // Brewing just started, no routine running, switch to Manual Brew
                        self.status.state = UIState::ManualBrew(ControlMode::default());
                        
                        // Set group control target to safe default (duty cycle 0 = Off)
                        let target = self.status.manual_brew_parameters.to_group_brew_control_target(ControlMode::default());
                        self.command_sender.send(
                            MachineCommand::SetGroupBrewControlTarget(SingleGroup.as_index(), target)
                        ).await;
                        
                        self.ui_status_sender.send(self.status.clone()).await;
                    }
                    
                    // Handle automatic UI switching for Manual Brew exit
                    else if self.previous_brewing_state && !current_brewing &&
                            matches!(self.status.state, UIState::ManualBrew(_)) &&
                            !routine_running {
                        warn!("Switching from Manual Brew mode due to brewing stop");

                        // Brewing just stopped, currently in Manual Brew, no routine running, return to idle
                        self.status.state = UIState::Idle(IdleSubState::NoMenuItemSelected);
                        self.ui_status_sender.send(self.status.clone()).await;
                    }

                    // Update previous brewing state for next iteration
                    self.previous_brewing_state = current_brewing;
                }
            }
        }
    }
}