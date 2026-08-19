use defmt::{info, warn, Format};
use embassy_futures::select::{select4, Either4};
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio_programs::rotary_encoder::{Direction, PioEncoder};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Sender;
use embassy_time::Timer;
use embedded_hal::digital::InputPin;
use embedded_hal_async::digital::Wait;
use variegated_controller_types::{BoilerControlMode, BoilerControlTargetValuesUpdate, Configuration, GroupBrewControlMode, GroupBrewControlTargetValuesUpdate, MachineCommand, MachineMode, PidParameters, PidParameterTarget, RoutineIndex, Status};
use crate::{RoutineRepository, StatusSubscriber, ConfigurationSubscriber};
use crate::list_menu::{ListMenuType, ListMenuItem, MenuItemId, PidConfigType, PidTermType, PidComponentType};
use variegated_machine_menu::{
    boiler_temperature_adjustable, parameter_bounds, parameter_geometry, parameter_row,
    ParameterListChrome, ParameterRow, ParameterValues,
};
use variegated_menu::{Adjustable, ListGeometry, ListNav};
use alloc::string::ToString;
use alloc::vec::Vec;
use alloc::boxed::Box;
use variegated_controller_types::SingleGroupControllerGroups::SingleGroup;
use variegated_controller_lib::routine::{ParameterUnit, Routine, RoutineRepository as RoutineRepositoryTrait};
use variegated_controller_lib::single_boiler_state::{
    DEFAULT_STEAM_TARGET_TEMPERATURE, MAX_BREW_TEMPERATURE, MAX_STEAM_TEMPERATURE,
};
use alloc::string::String;

#[derive(Debug, Format, Default, Copy, Clone, PartialEq)]
pub(crate) enum ControlMode {
    #[default]
    PumpDutyCycle,
    PumpFlowRate,
    PumpPressure,
}

#[derive(Debug, Format, Copy, Clone, PartialEq)]
pub(crate) enum ConfigEditType {
    BoilerTemperature,
    /// Boiler index 1 — the virtual steam boiler, i.e. what the one element holds while the
    /// machine is in steam mode.
    SteamTemperature,
    PidParameter(PidConfigType, PidTermType, PidComponentType),
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
    /// Never constructed: steaming and hot-water dispensing are shown through
    /// `Idle(IdleSubState)` and the machine's own mode rather than by entering a
    /// dedicated UI state. Kept because they name real machine activities and the screens
    /// may yet want their own state; nothing transitions into them today.
    #[allow(dead_code)]
    Steaming,
    ManualBrew(ControlMode),
    #[allow(dead_code)]
    DispensingWater,
    RoutineExecution,
    /// The fourth field is the cached item list, populated by `enter_list_menu`. Activation
    /// resolves the selected row through it -- see the note on [`ListMenuItem::id`] -- and the
    /// renderer reads it instead of re-fetching every frame.
    ///
    /// `None` only where a menu is entered from a synchronous context that has no repository
    /// to fetch from: `ListMenuType::get_back_state`. Those destinations are `Settings` and
    /// `PidConfig`, whose rows resolve positionally, so the fallback is correct there.
    ListMenu(
        ListMenuType,
        ListNav,
        Option<Box<(ListMenuType, ListNav)>>,
        Option<Vec<ListMenuItem>>,
    ),
    SettingsInformation,
    SettingsDebugInfo,
    /// The Improv provisioning window: opened on entry, closed on exit.
    ///
    /// Carries no state of its own. What it renders comes from `Status.comms_status.improv`,
    /// which the comms processor reports once a second, so the screen tracks the radio rather
    /// than tracking what this processor last asked for.
    WifiProvisioning,
    ScaleSettings(ScaleSettingsSubState),
    RoutineParameters(RoutineIndex, RoutineParameterEditState),
    ParameterManipulation {
        edit_state: RoutineParameterEditState,
        routine_index: RoutineIndex,
        /// Position in `routine.parameters()`, **not** `RoutineParameter::index`.
        ///
        /// The same thing [`variegated_machine_menu::ParameterValues`] is indexed by. The two
        /// are not interchangeable: a routine's parameter indices are not required to be
        /// contiguous or to start at zero.
        position: usize,
        current_value: f32,
        param_name: String,
        param_unit: Option<ParameterUnit>,
    },
    ConfigValueEdit {
        config_type: ConfigEditType,
        current_value: f32,
        previous_menu_type: ListMenuType,
        previous_menu_state: ListNav,
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

    /// The bounds and step for each mode, stated exactly once.
    ///
    /// They used to be per-mode literals inside `adjust_value` *and* again as rounding clamps
    /// inside `sync_from_process_values`, which is two places for one fact.
    fn limits(mode: ControlMode) -> (f32, f32, f32) {
        match mode {
            //             min    max     step
            ControlMode::PumpDutyCycle => (0.0, 100.0, 5.0),
            ControlMode::PumpFlowRate => (0.0, 50.0, 0.5),
            ControlMode::PumpPressure => (0.0, 15.0, 0.5),
        }
    }

    fn adjustable(&self, mode: ControlMode) -> Adjustable {
        let (min, max, step) = Self::limits(mode);
        Adjustable::new(self.get_value(mode), min, max, step)
    }

    pub fn adjust_value(&mut self, mode: ControlMode, increment: bool) {
        let mut a = self.adjustable(mode);
        if increment { a.increase() } else { a.decrease() }

        match mode {
            // Back through `u8`. The value is clamped to 0..=100 by `Adjustable`, so this cast
            // cannot saturate.
            ControlMode::PumpDutyCycle => self.duty_cycle = a.value() as u8,
            ControlMode::PumpFlowRate => self.flow_rate = a.value(),
            ControlMode::PumpPressure => self.pressure = a.value(),
        }
    }

    pub fn to_group_brew_control_command(&self, mode: ControlMode) -> (GroupBrewControlMode, Option<GroupBrewControlTargetValuesUpdate>) {
        match mode {
            ControlMode::PumpDutyCycle => {
                if self.duty_cycle == 0 {
                    (GroupBrewControlMode::Off, None)
                } else {
                    (GroupBrewControlMode::FixedDutyCycle, Some(GroupBrewControlTargetValuesUpdate {
                        flow_rate: None,
                        flow_rate_curve: None,
                        pressure: None,
                        pressure_curve: None,
                        output_flow_rate: None,
                        output_flow_rate_curve: None,
                        duty_cycle: Some(self.duty_cycle),
                        duty_cycle_curve: None
                    }))
                }
            }
            ControlMode::PumpFlowRate => {
                if self.flow_rate <= 0.0 {
                    (GroupBrewControlMode::Off, None)
                } else {
                    (GroupBrewControlMode::GroupFlowRate, Some(GroupBrewControlTargetValuesUpdate {
                        flow_rate: Some(self.flow_rate),
                        flow_rate_curve: None,
                        pressure: None,
                        pressure_curve: None,
                        output_flow_rate: None,
                        output_flow_rate_curve: None,
                        duty_cycle: None,
                        duty_cycle_curve: None
                    }))
                }
            }
            ControlMode::PumpPressure => {
                if self.pressure <= 0.0 {
                    (GroupBrewControlMode::Off, None)
                } else {
                    (GroupBrewControlMode::Pressure, Some(GroupBrewControlTargetValuesUpdate {
                        flow_rate: None,
                        flow_rate_curve: None,
                        pressure: Some(self.pressure),
                        pressure_curve: None,
                        output_flow_rate: None,
                        output_flow_rate_curve: None,
                        duty_cycle: None,
                        duty_cycle_curve: None
                    }))
                }
            }
        }
    }

    pub fn sync_from_process_values(&mut self, group_status: &variegated_controller_types::GroupStatus) {
        // Update duty cycle from current pump output (clamp to 0-100%)
        let current_duty = group_status.pump_output.duty_cycle();
        // Round to nearest 5% increment, in `u16`. `current_duty` is a `DutyCycleType = u8`
        // produced by a saturating `as u8` cast, so a duty of 254 or 255 overflowed the `+ 2` --
        // a debug panic, and a wrap to 0 in release.
        let (_, duty_max, _) = Self::limits(ControlMode::PumpDutyCycle);
        self.duty_cycle = (((current_duty as u16 + 2) / 5) * 5).min(duty_max as u16) as u8;

        // Update flow rate from the current process value. It must be the *input* flow rate:
        // `ControlMode::PumpFlowRate` maps to `GroupBrewControlMode::GroupFlowRate`, whose PID
        // reads `get_input_flow_rate`. Preferring the output flow rate -- which this did --
        // handed the PID a setpoint measured on the other side of the puck, so the transfer
        // started with a real error however well the integral was seeded. If there is no
        // input reading, leave the previous target alone rather than substitute the one the
        // loop cannot see, exactly as the pressure branch below does.
        if let Some(flow) = group_status.input_flow_rate {
            // Bounds and step from the same table `adjust_value` uses, so the rounding grid a
            // synced value lands on is the grid the knob then steps along.
            let (min, max, step) = Self::limits(ControlMode::PumpFlowRate);
            let clamped_flow = flow.max(min).min(max);
            self.flow_rate = ((clamped_flow + step / 2.0) / step) as u32 as f32 * step;
        }

        // Update pressure from current process value
        if let Some(pressure) = group_status.pressure {
            let (min, max, step) = Self::limits(ControlMode::PumpPressure);
            let clamped_pressure = pressure.max(min).min(max);
            self.pressure = ((clamped_pressure + step / 2.0) / step) as u32 as f32 * step;
        }
    }
}

/// This panel's parameter-screen chrome: a `<-` row, five rows visible, no wrapping.
///
/// A rotary encoder has no dedicated back control, so the row has to exist here; the GS3 has
/// button 4 and a permanent hint row, and spends its four rows on routines instead.
pub(crate) const PARAMETER_CHROME: ParameterListChrome = ParameterListChrome {
    back_row: true,
    visible_rows: crate::list_menu::VISIBLE_ROWS,
    wrap: false,
};

/// Where a routine's parameter screen is, and what has been dialled into it.
///
/// The row layout and the value store are both [`variegated_machine_menu`]'s, shared with the
/// GS3 and host-tested there. What stays here is what is this panel's: the chrome above, and
/// the routine name, which is cached so the renderer does not re-lock the repository for it.
#[derive(Debug, Clone)]
pub(crate) struct RoutineParameterEditState {
    pub nav: ListNav,
    pub values: ParameterValues,
    pub routine_name: String,
}

impl RoutineParameterEditState {
    /// Seed the screen: defaults, except that a parameter linked to the dose takes whatever
    /// dose has been captured for the next shot.
    ///
    /// `pending_dose` is `Status.pending_shot_annotations.dose_weight()`. This board has no
    /// way to *capture* a dose of its own -- nothing here emits `TagDoseFromScale`, and the
    /// encoder has a single falling edge and no hold gesture to hang one on -- so in practice
    /// the value arrives from the web. Seeding from it anyway is what makes that capture
    /// reach the routine.
    pub fn new(routine: &Routine, pending_dose: Option<f32>) -> Self {
        let mut values = ParameterValues::from_defaults(routine);

        if let Some(dose) = pending_dose {
            for (position, parameter) in routine.parameters().iter().enumerate() {
                // Positional, because `ParameterValues` is -- not `RoutineParameter::index`.
                if parameter.linked_attribute.as_ref()
                    == Some(&variegated_controller_types::ShotAnnotationKey::DoseWeight)
                {
                    values.set(position, dose);
                }
            }
        }

        Self {
            nav: ListNav::new(), // Starts on the back row
            values,
            routine_name: routine.name().to_string(),
        }
    }

    /// Back, then one row per parameter, then Execute. One index space, same as the list menu.
    ///
    /// `param_count` is passed in rather than read off `values`, because the renderer draws
    /// `routine.parameters()` and a row count taken from anywhere else is a second source of
    /// truth for the same number -- which is the class of bug this whole change removes.
    pub fn geometry(&self, param_count: usize) -> ListGeometry {
        parameter_geometry(param_count, PARAMETER_CHROME)
    }

    /// What a row is.
    pub fn row_kind(&self, row: usize, param_count: usize) -> ParameterRow {
        parameter_row(row, param_count, PARAMETER_CHROME)
    }
}

#[derive(Debug, Default, Clone)]
pub(crate) struct UIStatus {
    pub(crate) state: UIState,
    pub(crate) manual_brew_parameters: ManualBrewParameters,
}

// Menu item activation handler
/// Enter a list menu with its items already fetched.
///
/// **Populating the cache is half the routine-selection fix.** `get_menu_item_id` maps a row
/// number to an id from static tables and returns `None` unconditionally for
/// `ListMenuType::Routines`, because a `RoutineIndex` cannot be recovered from a position. The
/// cached `ListMenuItem` carries one, so activation can resolve through it.
///
/// It also spares the renderer a `get_items` call per frame -- it runs on a 1 us delay and was
/// taking the routine repository mutex and re-allocating a `Vec<String>` every time round.
/// `status` is what decides which routines are runnable. It has been in `get_items`'s
/// signature all along and was ignored, which is why an unrunnable routine has always looked
/// exactly like a runnable one here.
///
/// `None` where the menu being entered contains no routines -- the PID screens -- rather
/// than manufacturing a `Status` to satisfy the type. Nothing in those menus can be gated.
async fn enter_list_menu(
    menu_type: ListMenuType,
    routine_repository: &RoutineRepository,
    parent: Option<Box<(ListMenuType, ListNav)>>,
    status: Option<&Status>,
) -> UIState {
    let items = menu_type.get_items(Some(routine_repository), status).await;
    UIState::ListMenu(menu_type, ListNav::new(), parent, Some(items))
}

/// `pending_dose` seeds a dose-linked parameter; see [`RoutineParameterEditState::new`].
pub async fn handle_menu_item_activation(
    item_id: MenuItemId,
    routine_repository: &RoutineRepository,
    pending_dose: Option<f32>,
) -> Option<UIState> {
    info!("Menu item activated: {:?}", item_id);
    let new_state = match item_id {
        MenuItemId::Routine(index) => {
            // Load routine definition and transition to parameter view
            let mut repo = routine_repository.lock().await;
            if let Some(routine) = repo.get_routine(index).await {
                let edit_state = RoutineParameterEditState::new(routine, pending_dose);
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
        MenuItemId::SettingsWifiProvisioning => {
            // Handled at the call site, which has the command sender this function does not:
            // entering the screen has to *open* the window, not merely display it. Same shape
            // as `SettingsBoilerTemperature` below.
            None
        },
        MenuItemId::SettingsBoilerTemperature | MenuItemId::SettingsSteamTemperature => {
            // This is now handled inline in the match statement to have access to configuration
            None
        },
        MenuItemId::SettingsBoilerTemperaturePID => {
            Some(enter_list_menu(ListMenuType::PidConfig(PidConfigType::BoilerTemperature), routine_repository, None, None).await)
        },
        MenuItemId::SettingsPumpFlowRatePID => {
            Some(enter_list_menu(ListMenuType::PidConfig(PidConfigType::PumpFlowRate), routine_repository, None, None).await)
        },
        MenuItemId::SettingsPumpOutputFlowRatePID => {
            Some(enter_list_menu(ListMenuType::PidConfig(PidConfigType::PumpOutputFlowRate), routine_repository, None, None).await)
        },
        MenuItemId::SettingsPumpPressurePID => {
            Some(enter_list_menu(ListMenuType::PidConfig(PidConfigType::PumpPressure), routine_repository, None, None).await)
        },
        MenuItemId::PidTerm(_term) => {
            // This will be called from PID config menu, need to get the PID type from context
            // For now, we'll handle this in the button press logic where we have more context
            None
        },
        MenuItemId::PidComponent(_component) => {
            // This will be handled in button press logic where we have the full context
            None
        },
        MenuItemId::PidResetParameters => {
            // This will be handled in button press logic where we have the full context
            None
        },
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
    configuration_receiver: ConfigurationSubscriber,
    previous_brewing_state: bool,
    current_status: Status,
    current_configuration: Option<Configuration>,
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
        configuration_receiver: ConfigurationSubscriber,
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
            configuration_receiver,
            previous_brewing_state: false,
            current_status: Status::default(),
            current_configuration: None,
        }
    }


    pub async fn task(&mut self) {
        self.ui_status_sender.send(self.status.clone()).await;

        loop {
            let either4 = select4(
                self.rotary.read(),
                self.button.wait_for_falling_edge(),
                self.status_receiver.next_message_pure(),
                self.configuration_receiver.next_message_pure()
            ).await;
            match either4 {
                Either4::First(direction) => {
                    // Rotary encoder was turned
                match &mut self.status.state {
                    UIState::Idle(substate) => {
                        *substate = match direction {
                            Direction::Clockwise => substate.rotate_counterclockwise(),
                            Direction::CounterClockwise => substate.rotate_clockwise(),
                        };
                    }
                    UIState::ListMenu(menu_type, nav, _, _) => {
                        let item_count = menu_type.get_item_count(Some(self.routine_repository)).await;
                        let geo = menu_type.geometry(item_count);
                        match direction {
                            Direction::Clockwise => nav.up(geo),
                            Direction::CounterClockwise => nav.down(geo),
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
                        let (mode, values) = self.status.manual_brew_parameters.to_group_brew_control_command(*control_mode);
                        self.command_sender.send(
                            MachineCommand::SetGroupBrewControlTarget(SingleGroup.as_index(), mode, values)
                        ).await;
                    }
                    UIState::RoutineParameters(routine_index, edit_state) => {
                        // Navigate through parameter list (same logic as ListMenu)
                        let mut repo = self.routine_repository.lock().await;
                        if let Some(routine) = repo.get_routine(*routine_index).await {
                            let geo = edit_state.geometry(routine.parameters().len());
                            match direction {
                                Direction::Clockwise => edit_state.nav.up(geo),
                                Direction::CounterClockwise => edit_state.nav.down(geo),
                            }
                        }
                    }
                    UIState::ParameterManipulation { current_value, param_unit, .. } => {
                        // The range now comes from the parameter's unit rather than being
                        // `(0.0, f32::INFINITY, 0.5)` for everything. `RoutineParameter` still
                        // carries no min, max or step -- and must not gain any, its postcard
                        // encoding being positional and unversioned -- so the table lives in
                        // `variegated-machine-menu` and is keyed on the unit.
                        //
                        // A parameter with no declared unit keeps exactly the old behaviour.
                        let (min, max, step) = parameter_bounds(*param_unit);
                        let mut a = Adjustable::new(*current_value, min, max, step);
                        match direction {
                            Direction::CounterClockwise => a.increase(),
                            Direction::Clockwise => a.decrease(),
                        }
                        *current_value = a.value();
                    }
                    UIState::ConfigValueEdit { config_type, current_value, .. } => {

                        // The temperatures stop where the controller's interlock would cut
                        // heating anyway; a target above it can only produce an element that
                        // runs to the limit and shuts off. PID terms keep the old open-ended
                        // behaviour -- there is no principled ceiling for a gain.
                        //
                        // The lower bounds are the substantive change. `-100` used to be the
                        // floor for *every* quantity edited here, which made it possible to dial
                        // a brew setpoint to -100 degrees C. It now applies only to the PID
                        // components, which can legitimately be negative; picking real PID limits
                        // is a domain question and a separate conversation.
                        // The half-degree step and the zero floor are shared with the GS3's
                        // brew-temperature editor; only the ceiling is this machine's.
                        let mut a = match config_type {
                            ConfigEditType::BoilerTemperature => {
                                boiler_temperature_adjustable(*current_value, MAX_BREW_TEMPERATURE)
                            }
                            ConfigEditType::SteamTemperature => {
                                boiler_temperature_adjustable(*current_value, MAX_STEAM_TEMPERATURE)
                            }
                            ConfigEditType::PidParameter(_, _, component) => {
                                let step = match component {
                                    PidComponentType::PositiveScale
                                    | PidComponentType::NegativeScale => 0.1,
                                    PidComponentType::UpperLimit
                                    | PidComponentType::LowerLimit => 1.0,
                                };
                                Adjustable::new(*current_value, -100.0, f32::INFINITY, step)
                            }
                        };
                        match direction {
                            Direction::CounterClockwise => a.increase(),
                            Direction::Clockwise => a.decrease(),
                        }
                        *current_value = a.value();
                    }
                    _ => {}
                }

                self.ui_status_sender.send(self.status.clone()).await;
                }
                Either4::Second(_) => {
                    // Button was pressed
                match &self.status.state {
                    UIState::Idle(substate) => {
                        match substate {
                            IdleSubState::RoutineMenuSelected => {
                                self.status.state = enter_list_menu(ListMenuType::Routines, self.routine_repository, None, Some(&self.current_status)).await;
                            }
                            IdleSubState::SettingsMenuSelected => {
                                self.status.state = enter_list_menu(ListMenuType::Settings, self.routine_repository, None, Some(&self.current_status)).await;
                            }
                            // Nothing selected, and the machine is not on: the press turns
                            // it on.
                            //
                            // This is the only way out of "Machine Off" from the front of
                            // the machine. Without it the mode could be left but never
                            // returned to without a browser or a debug link, on a screen
                            // whose entire content is the word "Off" -- and the press was
                            // doing nothing at all, so there was nothing to discover.
                            //
                            // `PowerSaveStandby` is included for the same reason: it renders
                            // the same kind of dead-end screen, and the same press is the
                            // obvious way out of it.
                            IdleSubState::NoMenuItemSelected
                                if self.current_status.mode != MachineMode::On =>
                            {
                                let _ = self
                                    .command_sender
                                    .try_send(MachineCommand::SetMachineMode(MachineMode::On));
                            }
                            // Nothing selected on a machine that is already on. Left alone
                            // deliberately: the symmetric "press to turn off" would fire on
                            // the same press a user makes to dismiss the idle screen, and
                            // switching a machine off mid-session is not something to do by
                            // accident. Turning off stays in the settings menu.
                            _ => {}
                        }
                    }
                    UIState::ListMenu(menu_type, nav, parent_state, cached_items) => {
                        let menu_type = *menu_type;
                        let selected_row = nav.selected();

                        // Prefer the cached item's own id, falling back to the positional table.
                        //
                        // `get_menu_item_id` returns `None` **unconditionally** for
                        // `ListMenuType::Routines`, because a `RoutineIndex` cannot be recovered
                        // from a row number -- and the cached `ListMenuItem` carries one. This is
                        // the fix `ListMenuItem::id`'s doc comment describes and that nobody
                        // connected; it is what makes `handle_menu_item_activation`'s
                        // `MenuItemId::Routine` arm reachable at all.
                        let resolved = menu_type.item_index(selected_row).and_then(|item_index| {
                            cached_items
                                .as_ref()
                                .and_then(|items| items.get(item_index))
                                .map(|item| item.id)
                                .or_else(|| menu_type.get_menu_item_id(item_index))
                        });

                        // Whether the selected row can be acted on at all. Only routines can
                        // answer `false` -- see `ListMenuItem::runnable`. Read from the same
                        // cached item the renderer drew the "!" from, so what a press does and
                        // what the screen said agree by construction rather than by both
                        // recomputing it.
                        let selected_runnable = menu_type
                            .item_index(selected_row)
                            .and_then(|item_index| {
                                cached_items
                                    .as_ref()
                                    .and_then(|items| items.get(item_index))
                                    .map(|item| item.runnable)
                            })
                            .unwrap_or(true);

                        // Row space throughout: `item_index` returns `None` for the back row and
                        // `Some(i)` for an item, which is the same mapping the renderer uses.
                        if menu_type.item_index(selected_row).is_none() {
                            // Use stored parent state if available, otherwise use default back state
                            if let Some(parent) = parent_state {
                                let (parent_menu_type, parent_menu_state) = *parent.clone();
                                self.status.state = UIState::ListMenu(parent_menu_type, parent_menu_state, None, None);
                            } else {
                                self.status.state = menu_type.get_back_state();
                            }
                        } else if !selected_runnable {
                            // The machine cannot sense something this routine needs. Refused
                            // at the list rather than at the Execute row inside the parameter
                            // screen, so a user is not walked into a screen they cannot leave
                            // by running anything.
                            warn!("Routine refused: its prerequisites are not met");
                        } else if let Some(menu_item_id) = resolved {

                            // Handle menu item activation
                            match menu_item_id {
                                MenuItemId::SettingsWifiProvisioning => {
                                    // Five minutes, matching the dual boiler's button hold.
                                    // The controller refuses this outright while the machine is
                                    // busy; the screen then shows "Not open" and the user finds
                                    // out by reading it rather than by being told twice.
                                    self.command_sender.send(
                                        MachineCommand::OpenWifiProvisioningWindow {
                                            duration_ms: 300_000,
                                        }
                                    ).await;
                                    self.status.state = UIState::WifiProvisioning;
                                },
                                MenuItemId::SettingsBoilerTemperature => {
                                    // Get current boiler temperature from configuration
                                    let current_temp = if let Some(ref config) = self.current_configuration {
                                        config.get_boiler_configuration(0)
                                            .map(|bc| {
                                                bc.control_state.values.target_temperature
                                            })
                                            .unwrap_or(110.0)
                                    } else {
                                        110.0 // Default
                                    };
                                    
                                    self.status.state = UIState::ConfigValueEdit {
                                        config_type: ConfigEditType::BoilerTemperature,
                                        current_value: current_temp,
                                        previous_menu_type: menu_type,
                                        previous_menu_state: *nav,
                                    };
                                },
                                MenuItemId::SettingsSteamTemperature => {
                                    // Boiler 1 is the virtual steam boiler: the same element
                                    // under a second control state, selected while the
                                    // machine is in steam mode.
                                    let current_temp = self.current_configuration
                                        .as_ref()
                                        .and_then(|config| config.get_boiler_configuration(1))
                                        .map(|bc| bc.control_state.values.target_temperature)
                                        .unwrap_or(DEFAULT_STEAM_TARGET_TEMPERATURE);

                                    self.status.state = UIState::ConfigValueEdit {
                                        config_type: ConfigEditType::SteamTemperature,
                                        current_value: current_temp,
                                        previous_menu_type: menu_type,
                                        previous_menu_state: *nav,
                                    };
                                },
                                MenuItemId::PidTerm(term) => {
                                    // We need to know which PID type we're in
                                    if let ListMenuType::PidConfig(pid_type) = menu_type {
                                        // Store parent menu state, so backing out lands on the
                                        // row that was left rather than on row 0.
                                        let parent_state = Some(Box::new((menu_type, *nav)));
                                        self.status.state = enter_list_menu(
                                            ListMenuType::PidTermConfig(pid_type, term),
                                            self.routine_repository,
                                            parent_state,
                                            Some(&self.current_status),
                                        ).await;
                                    }
                                },
                                MenuItemId::PidResetParameters => {
                                    // We need to know which PID type we're in
                                    if let ListMenuType::PidConfig(pid_type) = menu_type {
                                        // Get default PID parameters based on type
                                        use variegated_controller_types::{PidParameters, PidTerm, PidLimits};
                                        
                                        let default_params = match pid_type {
                                            PidConfigType::BoilerTemperature => PidParameters {
                                                kp: PidTerm::new(3.0, PidLimits::default()),
                                                ki: PidTerm::new(0.01, PidLimits::new_with_limits(-10.0, 10.0).unwrap()),
                                                kd: PidTerm::new(30.0, PidLimits::new_with_limits(-10.0, 10.0).unwrap())
                                            },
                                            PidConfigType::PumpFlowRate | PidConfigType::PumpOutputFlowRate | PidConfigType::PumpPressure => PidParameters {
                                                kp: PidTerm::new(10.0, PidLimits::default()),
                                                ki: PidTerm::new(0.01, PidLimits::new_with_limits(-50.0, 80.0).unwrap()),
                                                kd: PidTerm::new(30.0, PidLimits::new_with_limits(-10.0, 10.0).unwrap())
                                            },
                                        };
                                        
                                        // Send command to reset PID parameters
                                        let target = match pid_type {
                                            PidConfigType::BoilerTemperature => PidParameterTarget::BoilerTemperature(0),
                                            PidConfigType::PumpFlowRate => PidParameterTarget::GroupFlowRate(0),
                                            PidConfigType::PumpOutputFlowRate => PidParameterTarget::GroupOutputFlowRate(0),
                                            PidConfigType::PumpPressure => PidParameterTarget::GroupPressure(0),
                                        };
                                        
                                        self.command_sender.send(
                                            MachineCommand::SetPidParameters(target, default_params)
                                        ).await;
                                        
                                        info!("Reset PID parameters for {:?} to defaults", pid_type);
                                    }
                                },
                                MenuItemId::PidComponent(component) => {
                                    // We need to know which PID type and term we're in
                                    if let ListMenuType::PidTermConfig(pid_type, term) = menu_type {
                                        // Get current value from configuration
                                        let current_value = if let Some(ref config) = self.current_configuration {
                                            let params = match pid_type {
                                                PidConfigType::BoilerTemperature => {
                                                    config.get_boiler_configuration(0)
                                                        .map(|bc| &bc.temperature_pid_parameters)
                                                },
                                                PidConfigType::PumpFlowRate => {
                                                    config.get_group_configuration(0)
                                                        .map(|gc| &gc.flow_rate_pid_parameters)
                                                },
                                                PidConfigType::PumpOutputFlowRate => {
                                                    config.get_group_configuration(0)
                                                        .map(|gc| &gc.output_flow_rate_pid_parameters)
                                                },
                                                PidConfigType::PumpPressure => {
                                                    config.get_group_configuration(0)
                                                        .map(|gc| &gc.pressure_pid_parameters)
                                                },
                                            };

                                            if let Some(params) = params {
                                                let term_value = match term {
                                                    PidTermType::Kp => &params.kp,
                                                    PidTermType::Ki => &params.ki,
                                                    PidTermType::Kd => &params.kd,
                                                };

                                                match component {
                                                    PidComponentType::PositiveScale => term_value.positive_scale,
                                                    PidComponentType::NegativeScale => term_value.negative_scale,
                                                    PidComponentType::UpperLimit => 100.0, // TODO: Need getter methods in PID lib
                                                    PidComponentType::LowerLimit => -100.0, // TODO: Need getter methods in PID lib
                                                }
                                            } else {
                                                1.0 // Default fallback
                                            }
                                        } else {
                                            1.0 // No configuration available yet
                                        };
                                        self.status.state = UIState::ConfigValueEdit {
                                            config_type: ConfigEditType::PidParameter(pid_type, term, component),
                                            current_value,
                                            previous_menu_type: menu_type,
                                            previous_menu_state: *nav,
                                        };
                                    }
                                },
                                _ => {
                                    if let Some(new_state) = handle_menu_item_activation(
                                        menu_item_id,
                                        self.routine_repository,
                                        self.current_status.pending_shot_annotations.dose_weight(),
                                    ).await {
                                        self.status.state = new_state;
                                    }
                                }
                            }
                        } else {
                            // Deliberately not a `return`. This is inside `task`'s loop, and
                            // returning from here is what made selecting a routine kill the
                            // encoder until the next power cycle -- `main.rs` joins these futures,
                            // so a completed one is an input task that never runs again. With the
                            // item cache unpopulated, `get_menu_item_id` returned `None`
                            // unconditionally for `Routines`, so that path was reached every
                            // single time.
                            warn!("Menu: row {} resolved to no item; ignoring", selected_row);
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
                                self.status.state = enter_list_menu(ListMenuType::Settings, self.routine_repository, None, Some(&self.current_status)).await;
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
                        let (mode, values) = self.status.manual_brew_parameters.to_group_brew_control_command(new_mode);
                        self.command_sender.send(
                            MachineCommand::SetGroupBrewControlTarget(SingleGroup.as_index(), mode, values)
                        ).await;
                    }
                    UIState::SettingsInformation | UIState::SettingsDebugInfo => {
                        // Go back to settings menu
                        self.status.state = enter_list_menu(ListMenuType::Settings, self.routine_repository, None, Some(&self.current_status)).await;
                    }
                    UIState::WifiProvisioning => {
                        // Closed explicitly rather than left to expire. Leaving the screen is
                        // the clearest statement a user can make that they are done, and five
                        // more minutes of connectable advertising shares one antenna with
                        // Wi-Fi and with the live link to the scale.
                        self.command_sender.send(MachineCommand::CloseWifiProvisioningWindow).await;
                        self.status.state = enter_list_menu(ListMenuType::Settings, self.routine_repository, None, Some(&self.current_status)).await;
                    }
                    UIState::RoutineExecution => {
                        // Cancel the currently running routine
                        self.command_sender.send(MachineCommand::CancelRoutine).await;
                        self.status.state = UIState::Idle(IdleSubState::NoMenuItemSelected);
                    }
                    UIState::RoutineParameters(routine_index, edit_state) => {
                        let routine_index = *routine_index;
                        let mut repo = self.routine_repository.lock().await;
                        if let Some(routine) = repo.get_routine(routine_index).await {
                            let params = routine.parameters();
                            match edit_state.row_kind(edit_state.nav.selected(), params.len()) {
                                ParameterRow::Back => {
                                    // Back button selected - return to routine menu
                                    self.status.state = enter_list_menu(ListMenuType::Routines, self.routine_repository, None, Some(&self.current_status)).await;
                                }
                                ParameterRow::Execute => {
                                    // Execute button selected - run routine with current parameters.
                                    // `to_runtime` re-keys the positional values by
                                    // `RoutineParameter::index` and returns `None` for a routine
                                    // with no parameters, which is what this sent before.
                                    let runtime_params = edit_state.values.to_runtime(routine);
                                    self.command_sender.send(MachineCommand::RunRoutine(routine_index, runtime_params)).await;
                                    self.status.state = UIState::RoutineExecution;
                                }
                                ParameterRow::Parameter(position) => {
                                    // Parameter selected - enter manipulation mode
                                    if let Some(param) = params.get(position) {
                                        let current_value = edit_state
                                            .values
                                            .get(position)
                                            .unwrap_or(param.default);
                                        self.status.state = UIState::ParameterManipulation {
                                            edit_state: edit_state.clone(),
                                            routine_index,
                                            position,
                                            current_value,
                                            param_name: param.name.clone(),
                                            param_unit: param.unit,
                                        };
                                    }
                                }
                            }
                        }
                    }
                    UIState::ParameterManipulation { edit_state, routine_index, position, current_value, .. } => {
                        // Return to parameter list with updated value.
                        //
                        // The write cannot fail and cannot be dropped. `ParameterValues` is a
                        // fixed positional array, and `position` came from the row layout over
                        // the same routine, so it is in range by construction. The map-keyed
                        // version this replaced could refuse the write on a routine with more
                        // than eight parameters, and its only recourse was to log and discard
                        // an edit the operator had just made.
                        let routine_index = *routine_index;
                        let mut preserved_edit_state = edit_state.clone();
                        preserved_edit_state.values.set(*position, *current_value);

                        self.status.state = UIState::RoutineParameters(routine_index, preserved_edit_state);
                    }
                    UIState::ConfigValueEdit { config_type, current_value, previous_menu_type, previous_menu_state, .. } => {
                        // Send command to update configuration and return to previous menu
                        let config_type = *config_type;
                        let current_value = *current_value;
                        
                        // Send appropriate command based on config type
                        match config_type {
                            ConfigEditType::BoilerTemperature => {
                                self.command_sender.send(
                                    MachineCommand::SetBoilerControlTarget(
                                        0, // Boiler index 0 for single boiler
                                        BoilerControlMode::Temperature,
                                        Some(BoilerControlTargetValuesUpdate {
                                            temperature: Some(current_value),
                                            pressure: None
                                        })
                                    )
                                ).await;
                            },
                            ConfigEditType::SteamTemperature => {
                                // Sends `Temperature` alongside the value, which is what
                                // clears a stored `Off` for good rather than relying on the
                                // substitution at load.
                                self.command_sender.send(
                                    MachineCommand::SetBoilerControlTarget(
                                        1, // The virtual steam boiler
                                        BoilerControlMode::Temperature,
                                        Some(BoilerControlTargetValuesUpdate {
                                            temperature: Some(current_value),
                                            pressure: None
                                        })
                                    )
                                ).await;
                            },
                            ConfigEditType::PidParameter(pid_type, term, component) => {
                                use crate::list_menu::{PidConfigType, PidTermType, PidComponentType};
                                use variegated_controller_types::PidParameterTarget;
                                
                                // Get current PID parameters from configuration
                                let mut updated_params = if let Some(ref config) = self.current_configuration {
                                    match pid_type {
                                        PidConfigType::BoilerTemperature => {
                                            config.get_boiler_configuration(0)
                                                .map(|bc| bc.temperature_pid_parameters.clone())
                                                .unwrap_or_default()
                                        },
                                        PidConfigType::PumpFlowRate => {
                                            config.get_group_configuration(0)
                                                .map(|gc| gc.flow_rate_pid_parameters.clone())
                                                .unwrap_or_default()
                                        },
                                        PidConfigType::PumpOutputFlowRate => {
                                            config.get_group_configuration(0)
                                                .map(|gc| gc.output_flow_rate_pid_parameters.clone())
                                                .unwrap_or_default()
                                        },
                                        PidConfigType::PumpPressure => {
                                            config.get_group_configuration(0)
                                                .map(|gc| gc.pressure_pid_parameters.clone())
                                                .unwrap_or_default()
                                        },
                                    }
                                } else {
                                    // No configuration available yet, use defaults
                                    PidParameters::default()
                                };
                                
                                // Modify the specific parameter component
                                let term_ref = match term {
                                    PidTermType::Kp => &mut updated_params.kp,
                                    PidTermType::Ki => &mut updated_params.ki, 
                                    PidTermType::Kd => &mut updated_params.kd,
                                };
                                
                                match component {
                                    PidComponentType::PositiveScale => term_ref.positive_scale = current_value,
                                    PidComponentType::NegativeScale => term_ref.negative_scale = current_value,
                                    PidComponentType::UpperLimit => {
                                        let _ = term_ref.limits.try_set_upper(current_value);
                                    },
                                    PidComponentType::LowerLimit => {
                                        let _ = term_ref.limits.try_set_lower(current_value);
                                    },
                                }
                                
                                // Send the updated parameters
                                let target = match pid_type {
                                    PidConfigType::BoilerTemperature => PidParameterTarget::BoilerTemperature(0),
                                    PidConfigType::PumpFlowRate => PidParameterTarget::GroupFlowRate(0), 
                                    PidConfigType::PumpOutputFlowRate => PidParameterTarget::GroupOutputFlowRate(0),
                                    PidConfigType::PumpPressure => PidParameterTarget::GroupPressure(0),
                                };
                                
                                self.command_sender.send(
                                    MachineCommand::SetPidParameters(target, updated_params)
                                ).await;
                                
                                info!("Updated PID parameter: {:?} {:?} {:?} = {}", pid_type, term, component, current_value);
                            }
                        }
                        
                        // Return to previous menu, on the row it was left on -- so not
                        // `enter_list_menu`, which starts a fresh menu at the top.
                        let previous_menu_type = *previous_menu_type;
                        let previous_nav = *previous_menu_state;
                        let items = previous_menu_type.get_items(Some(self.routine_repository), None).await;
                        self.status.state = UIState::ListMenu(previous_menu_type, previous_nav, None, Some(items));
                    }
                    _ => {
                        self.status.state = UIState::Idle(IdleSubState::NoMenuItemSelected);
                    }
                }
                self.ui_status_sender.send(self.status.clone()).await;

                Timer::after_millis(300).await;
                }
                Either4::Third(status_update) => {
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
                        let (mode, values) = self.status.manual_brew_parameters.to_group_brew_control_command(ControlMode::default());
                        self.command_sender.send(
                            MachineCommand::SetGroupBrewControlTarget(SingleGroup.as_index(), mode, values)
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
                Either4::Fourth(configuration_update) => {
                    // Configuration update received - store the latest configuration
                    self.current_configuration = Some(configuration_update);
                    info!("Configuration updated");
                }
            }
        }
    }
}