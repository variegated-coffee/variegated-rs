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
use variegated_controller_types::{BoilerControlTarget, DutyCycleType, GroupBrewControlTarget, MachineCommand, PidLimits, PidParameters, PidTerm, TemperatureType};
use variegated_hal::scale::gravity::GravityCommand;

#[derive(Debug, Format, Default, Copy, Clone)]
pub(crate) enum UIEditMode {
    #[default]
    PumpDutyCycle,
    BoilerTemperature,
    PumpFlowRate,
    PumpPressure,
    ScaleTare,
}

impl UIEditMode {
    pub fn next(&mut self) -> UIEditMode {
        match self {
            UIEditMode::PumpDutyCycle => UIEditMode::BoilerTemperature,
            UIEditMode::BoilerTemperature => UIEditMode::PumpFlowRate,
            UIEditMode::PumpFlowRate => UIEditMode::PumpPressure,
            UIEditMode::PumpPressure => UIEditMode::ScaleTare,
            UIEditMode::ScaleTare => UIEditMode::PumpDutyCycle,
        }
    }

    pub fn min_value(&self) -> f32 {
        match self {
            UIEditMode::PumpDutyCycle => 0.0,
            UIEditMode::BoilerTemperature => 10.0,
            UIEditMode::PumpFlowRate => 0.0,
            UIEditMode::PumpPressure => 0.0,
            UIEditMode::ScaleTare => 0.0, //Value doesn't matter for tare
        }
    }

    pub fn max_value(&self) -> f32 {
        match self {
            UIEditMode::PumpDutyCycle => 100.0,
            UIEditMode::BoilerTemperature => 120.0,
            UIEditMode::PumpFlowRate => 10.0,
            UIEditMode::PumpPressure => 10.0,
            UIEditMode::ScaleTare => 10.0, //Value doesn't matter for tare
        }
    }

    pub fn step(&self) -> f32 {
        match self {
            UIEditMode::PumpDutyCycle => 5.0,
            UIEditMode::BoilerTemperature => 5.0,
            UIEditMode::PumpFlowRate => 0.25,
            UIEditMode::PumpPressure => 0.5,
            UIEditMode::ScaleTare => 1.0, //Value doesn't matter for tare
        }
    }
}

#[derive(Debug, Format, Default, Copy, Clone)]
pub(crate) enum ControlMode {
    #[default]
    PumpDutyCycle,
    PumpFlowRate,
    PumpPressure,
}

#[derive(Debug, Format, Default, Copy, Clone)]
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

#[derive(Debug, Format, Copy, Clone)]
pub(crate) enum UIState {
    Idle(IdleSubState),
    Steaming,
    ManualBrew(ControlMode),
    DispensingWater,
    RoutineExecution,
    RoutineSelection,
    Settings,
}

impl Default for UIState {
    fn default() -> Self {
        UIState::Idle(IdleSubState::NoMenuItemSelected)
    }
}

#[derive(Debug, Format, Default, Copy, Clone)]
pub(crate) struct UIStatus {
    pub(crate) state: UIState,
    pub(crate) edit_mode: UIEditMode,
    pub(crate) current_duty_cycle: f32,
    pub(crate) current_boiler_temp: f32,
    pub(crate) current_flow_rate: f32,
    pub(crate) current_pressure: f32,
}

pub(crate) struct RotaryController<'a, C, const N: usize> where
    C: InputPin + Wait,
{
    rotary: PioEncoder<'a, PIO0, 0>,
    button: C,
    command_sender: Sender<'a, NoopRawMutex, MachineCommand, N>,
    ui_status_sender: Sender<'a, NoopRawMutex, UIStatus, N>,
    status: UIStatus,
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
    ) -> Self {
        Self {
            rotary,
            button,
            command_sender,
            ui_status_sender,
            status: UIStatus::default(),
        }
    }

    pub async fn task(&mut self) {
        self.ui_status_sender.send(self.status).await;

        loop {
            let either = select(self.rotary.read(), self.button.wait_for_falling_edge()).await;
            if let First(direction) = either {
                match self.status.state {
                    UIState::Idle(substate) => {
                        self.status.state = UIState::Idle(match direction {
                            Direction::Clockwise => substate.rotate_counterclockwise(),
                            Direction::CounterClockwise => substate.rotate_clockwise(),
                        });
                    }
                    _ => {}
                }
                
                self.ui_status_sender.send(self.status).await;
            } else {
                match self.status.state {
                    UIState::Idle(substate) => {
                        match substate {
                            IdleSubState::RoutineMenuSelected => {
                                self.status.state = UIState::RoutineSelection;
                            }
                            IdleSubState::SettingsMenuSelected => {
                                self.status.state = UIState::Settings;
                            }
                            _ => {}
                        }
                    }
                    _ => {
                        self.status.state = UIState::Idle(IdleSubState::NoMenuItemSelected);
                    }
                }
                self.ui_status_sender.send(self.status).await;

                Timer::after_millis(300).await;
            }
        }
    }
}