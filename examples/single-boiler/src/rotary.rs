use core::cmp::{max, min};
use defmt::{info, Format};
use embassy_futures::select::Either::First;
use embassy_futures::select::select;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Sender;
use embassy_time::Timer;
use embedded_hal::digital::InputPin;
use embedded_hal_async::digital::Wait;
use rotary_encoder_hal::{DefaultPhase, Rotary};
use variegated_controller_types::{BoilerControlTarget, DutyCycleType, GroupBrewControlTarget, MachineCommand, PidLimits, PidParameters, PidTerm, TemperatureType};

#[derive(Debug, Format, Default, Copy, Clone)]
pub(crate) enum UIEditMode {
    #[default]
    PumpDutyCycle,
    BoilerTemperature,
    PumpFlowRate,
    PumpPressure,
}

impl UIEditMode {
    pub fn next(&mut self) -> UIEditMode {
        match self {
            UIEditMode::PumpDutyCycle => UIEditMode::BoilerTemperature,
            UIEditMode::BoilerTemperature => UIEditMode::PumpFlowRate,
            UIEditMode::PumpFlowRate => UIEditMode::PumpPressure,
            UIEditMode::PumpPressure => UIEditMode::PumpDutyCycle,
        }
    }

    pub fn min_value(&self) -> i32 {
        match self {
            UIEditMode::PumpDutyCycle => 0,
            UIEditMode::BoilerTemperature => 10,
            UIEditMode::PumpFlowRate => 0,
            UIEditMode::PumpPressure => 0,
        }
    }

    pub fn max_value(&self) -> i32 {
        match self {
            UIEditMode::PumpDutyCycle => 100,
            UIEditMode::BoilerTemperature => 120,
            UIEditMode::PumpFlowRate => 10,
            UIEditMode::PumpPressure => 10,
        }
    }

    pub fn step(&self) -> i32 {
        match self {
            UIEditMode::PumpDutyCycle => 5,
            UIEditMode::BoilerTemperature => 5,
            UIEditMode::PumpFlowRate => 1,
            UIEditMode::PumpPressure => 1,
        }
    }
}

#[derive(Debug, Format, Default, Copy, Clone)]
pub(crate) struct UIStatus {
    pub(crate) edit_mode: UIEditMode,
    pub(crate) current_duty_cycle: i32,
    pub(crate) current_boiler_temp: i32,
    pub(crate) current_flow_rate: i32,
    pub(crate) current_pressure: i32,
}

pub(crate) struct RotaryController<'a, A, B, C, const N: usize> where
    A: InputPin + Wait,
    B: InputPin + Wait,
    C: InputPin + Wait,
{
    rotary: Rotary<A, B, DefaultPhase>,
    button: C,
    command_sender: Sender<'a, CriticalSectionRawMutex, MachineCommand, N>,
    ui_status_sender: Sender<'a, CriticalSectionRawMutex, UIStatus, N>,
    status: UIStatus,
}

impl<'a, A, B, C, const N: usize> RotaryController<'a, A, B, C, N>
where
    A: InputPin + Wait,
    B: InputPin + Wait,
    C: InputPin + Wait,
{
    pub fn new(
        rotary: Rotary<A, B, DefaultPhase>,
        button: C,
        command_sender: Sender<'a, CriticalSectionRawMutex, MachineCommand, N>,
        ui_status_sender: Sender<'a, CriticalSectionRawMutex, UIStatus, N>,
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
            let (pin_a,pin_b) = self.rotary.pins();
            let either = select(select(pin_a.wait_for_any_edge(),pin_b.wait_for_any_edge()), self.button.wait_for_falling_edge()).await;
            if let First(_) = either {
                let current_value = match self.status.edit_mode {
                    UIEditMode::PumpDutyCycle => self.status.current_duty_cycle,
                    UIEditMode::BoilerTemperature => self.status.current_boiler_temp,
                    UIEditMode::PumpFlowRate => self.status.current_flow_rate,
                    UIEditMode::PumpPressure => self.status.current_pressure,
                };

                let direction = self.rotary.update().unwrap();
                let new_value = match direction {
                    rotary_encoder_hal::Direction::Clockwise => min(self.status.edit_mode.max_value(), current_value + self.status.edit_mode.step()),
                    rotary_encoder_hal::Direction::CounterClockwise => max(self.status.edit_mode.min_value(), current_value - self.status.edit_mode.step()),
                    rotary_encoder_hal::Direction::None => current_value,
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
                            kp: PidTerm { scale: 3.0, limits: PidLimits::default() },
                            ki: PidTerm { scale: 0.01, limits: PidLimits::new_with_limits(-10.0, 10.0).unwrap() },
                            kd: PidTerm { scale: 30.0, limits: PidLimits::new_with_limits(-10.0, 10.0).unwrap() }
                        };

                        info!("Boiler temperature: {}", new_value);
                        self.status.current_boiler_temp = new_value;
                        self.command_sender.send(MachineCommand::SetBoilerControlTarget(0, BoilerControlTarget::Temperature(self.status.current_boiler_temp as TemperatureType))).await;
                    },
                    UIEditMode::PumpFlowRate => {
                        let _flow_params = PidParameters {
                            kp: PidTerm { scale: 10.0, limits: PidLimits::default() },
                            ki: PidTerm { scale: 0.01, limits: PidLimits::new_with_limits(-50.0, 80.0).unwrap() },
                            kd: PidTerm { scale: 30.0, limits: PidLimits::new_with_limits(-10.0, 10.0).unwrap() }
                        };


                        info!("Pump flow rate: {}", new_value);
                        self.status.current_flow_rate = new_value;
                        self.command_sender.send(MachineCommand::SetGroupBrewControlTarget(0, GroupBrewControlTarget::GroupFlowRate(self.status.current_flow_rate as f32))).await;
                    },
                    UIEditMode::PumpPressure => {
                        let _pressure_params = PidParameters {
                            kp: PidTerm { scale: 10.0, limits: PidLimits::default() },
                            ki: PidTerm { scale: 0.01, limits: PidLimits::new_with_limits(-50.0, 80.0).unwrap() },
                            kd: PidTerm { scale: 30.0, limits: PidLimits::new_with_limits(-10.0, 10.0).unwrap() }
                        };

                        info!("Pump pressure: {}", new_value);
                        self.status.current_pressure = new_value;
                        self.command_sender.send(MachineCommand::SetGroupBrewControlTarget(0, GroupBrewControlTarget::Pressure(self.status.current_pressure as f32))).await;
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