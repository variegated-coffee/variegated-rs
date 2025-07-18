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
pub(crate) struct UIStatus {
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
                let current_value = match self.status.edit_mode {
                    UIEditMode::PumpDutyCycle => self.status.current_duty_cycle,
                    UIEditMode::BoilerTemperature => self.status.current_boiler_temp,
                    UIEditMode::PumpFlowRate => self.status.current_flow_rate,
                    UIEditMode::PumpPressure => self.status.current_pressure,
                    UIEditMode::ScaleTare => 0.0, // Tare doesn't have a value
                };
                
                let new_value = match direction {
                    Direction::CounterClockwise => self.status.edit_mode.max_value().min(current_value + self.status.edit_mode.step()),
                    Direction::Clockwise => self.status.edit_mode.min_value().max(current_value - self.status.edit_mode.step()),
                };

                if new_value == current_value {
                    continue;
                }

                match self.status.edit_mode {
                    UIEditMode::PumpDutyCycle => {
                        info!("Pump duty cycle: {}", new_value);
                        self.status.current_duty_cycle = new_value;
                        self.command_sender.send(MachineCommand::SetGroupBrewControlTarget(0, GroupBrewControlTarget::FixedDutyCycle(self.status.current_duty_cycle as DutyCycleType))).await;
                    },
                    UIEditMode::BoilerTemperature => {
                        let _boiler_params = PidParameters {
                            kp: PidTerm::new(3.0, PidLimits::default()),
                            ki: PidTerm::new(0.01, PidLimits::new_with_limits(-10.0, 10.0).unwrap()),
                            kd: PidTerm::new(30.0, PidLimits::new_with_limits(-10.0, 10.0).unwrap()),
                        };

                        info!("Boiler temperature: {}", new_value);
                        self.status.current_boiler_temp = new_value;
                        self.command_sender.send(MachineCommand::SetBoilerControlTarget(0, BoilerControlTarget::Temperature(self.status.current_boiler_temp as TemperatureType))).await;
                    },
                    UIEditMode::PumpFlowRate => {
                        let _flow_params = PidParameters {
                            kp: PidTerm::new(10.0, PidLimits::default()),
                            ki: PidTerm::new(0.01, PidLimits::new_with_limits(-50.0, 80.0).unwrap()),
                            kd: PidTerm::new(30.0, PidLimits::new_with_limits(-10.0, 10.0).unwrap()),
                        };


                        info!("Pump flow rate: {}", new_value);
                        self.status.current_flow_rate = new_value;
                        self.command_sender.send(MachineCommand::SetGroupBrewControlTarget(0, GroupBrewControlTarget::GroupFlowRate(self.status.current_flow_rate as f32))).await;
                    },
                    UIEditMode::PumpPressure => {
                        let _pressure_params = PidParameters {
                            kp: PidTerm::new(10.0, PidLimits::default()),
                            ki: PidTerm::new(0.01, PidLimits::new_with_limits(-50.0, 80.0).unwrap()),
                            kd: PidTerm::new(30.0, PidLimits::new_with_limits(-10.0, 10.0).unwrap()),
                        };

                        info!("Pump pressure: {}", new_value);
                        self.status.current_pressure = new_value;
                        self.command_sender.send(MachineCommand::SetGroupBrewControlTarget(0, GroupBrewControlTarget::Pressure(self.status.current_pressure as f32))).await;
                    },
                    UIEditMode::ScaleTare => {
                        info!("Scale tare");
                        self.command_sender.send(MachineCommand::TareGroupScale(0)).await;
                    },
                }

                self.ui_status_sender.send(self.status).await;
            } else {
                self.status.edit_mode = self.status.edit_mode.next();
                info!("Edit mode: {:?}", self.status.edit_mode);
                self.ui_status_sender.send(self.status).await;

                Timer::after_millis(300).await;
            }
        }
    }
}