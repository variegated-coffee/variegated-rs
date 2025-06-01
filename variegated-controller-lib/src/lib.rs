#![no_std]

mod routine;

extern crate alloc;

use defmt::{error, info, warn, Format};
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::channel::{Receiver};
use embassy_sync::pubsub::Publisher;
use embassy_time::{Instant, Timer};
use variegated_control_algorithm::pid::{PidCtrl, PidIn, PidOut};
use variegated_hal::{Boiler, Group};
use variegated_controller_types::{BoilerControlTarget, FlowRateType, GroupBrewControlTarget, MachineCommand, PidLimits, PidParameterTarget, PidParameters, PidTerm, PressureType, SingleBoilerSingleGroupControllerState, Status};
use crate::routine::RoutineExecutionContext;

fn limited_pid() -> PidCtrl<f32> {
    let mut pid = PidCtrl::default();
    pid.limits.try_set_lower(0.0).unwrap();
    pid.limits.try_set_upper(100.0).unwrap();
    pid
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct SingleBoilerSingleGroupPidParameters {
    boiler_pressure_params: PidParameters,
    boiler_temperature_params: PidParameters,
    pump_flow_rate_params: PidParameters,
    pump_pressure_params: PidParameters,
    pump_output_flow_rate_params: PidParameters,
}

#[derive(Clone, Copy, Debug, Default)]
struct SingleBoilerSingleGroupConfiguration {
    pub brew_boiler_control_target: BoilerControlTarget,
    pub steam_boiler_control_target: BoilerControlTarget,
    pub group_brew_control_target: GroupBrewControlTarget,
    pub pid_parameters: SingleBoilerSingleGroupPidParameters,
}

pub struct SingleBoilerSingleGroupController<'a, ChannelM: RawMutex, M: RawMutex, const N_CHANNEL: usize, const N_WATCH: usize, const N_SUBS: usize> {
    command_channel_receiver: Receiver<'a, ChannelM, MachineCommand, N_CHANNEL>,
    status_channel_sender: Publisher<'a, ChannelM, Status, 1, N_SUBS, 1>,
    boiler: Boiler<'a, M, N_WATCH>,
    group: Group<'a, M, N_WATCH>,
    state: SingleBoilerSingleGroupControllerState,
    boiler_pid: PidCtrl<f32>,
    pump_pid: PidCtrl<f32>,
    configuration: SingleBoilerSingleGroupConfiguration,
    current_routine: Option<RoutineExecutionContext>,
    routine_saved_configuration: Option<SingleBoilerSingleGroupConfiguration>,
    routine_saved_state: Option<SingleBoilerSingleGroupControllerState>,
    previous_status: Option<Status>,
}

impl <'a, ChannelM: RawMutex, M: RawMutex, const N_CHANNEL: usize, const N_WATCH: usize, const N_SUBS: usize> SingleBoilerSingleGroupController<'a, ChannelM, M, N_CHANNEL, N_WATCH, N_SUBS> {
    pub fn new(
        command_channel_receiver: Receiver<'a, ChannelM, MachineCommand, N_CHANNEL>,
        status_channel_sender: Publisher<'a, ChannelM, Status, 1, N_SUBS, 1>,
        boiler: Boiler<'a, M, N_WATCH>,
        group: Group<'a, M, N_WATCH>,
        brew_boiler_control_target: BoilerControlTarget,
        steam_boiler_control_target: BoilerControlTarget,
        group_brew_control_target: GroupBrewControlTarget,
    ) -> Self {
        let mut pid_parameters = SingleBoilerSingleGroupPidParameters::default();
        pid_parameters.boiler_temperature_params = PidParameters {
            kp: PidTerm { scale: 3.0, limits: PidLimits::default() },
            ki: PidTerm { scale: 0.01, limits: PidLimits::new_with_limits(-10.0, 10.0).unwrap() },
            kd: PidTerm { scale: 30.0, limits: PidLimits::new_with_limits(-10.0, 10.0).unwrap() }
        };
        pid_parameters.pump_flow_rate_params = PidParameters {
            kp: PidTerm { scale: 10.0, limits: PidLimits::default() },
            ki: PidTerm { scale: 0.01, limits: PidLimits::new_with_limits(-50.0, 80.0).unwrap() },
            kd: PidTerm { scale: 30.0, limits: PidLimits::new_with_limits(-10.0, 10.0).unwrap() }
        };
        pid_parameters.pump_pressure_params = PidParameters {
            kp: PidTerm { scale: 10.0, limits: PidLimits::default() },
            ki: PidTerm { scale: 0.01, limits: PidLimits::new_with_limits(-50.0, 80.0).unwrap() },
            kd: PidTerm { scale: 30.0, limits: PidLimits::new_with_limits(-10.0, 10.0).unwrap() }
        };

        Self {
            command_channel_receiver,
            status_channel_sender,
            boiler,
            group,
            state: SingleBoilerSingleGroupControllerState::default(),
            boiler_pid: limited_pid(),
            pump_pid: limited_pid(),
            configuration: SingleBoilerSingleGroupConfiguration {
                brew_boiler_control_target,
                steam_boiler_control_target,
                group_brew_control_target,
                pid_parameters,
            },
            current_routine: None,
            routine_saved_configuration: None,
            previous_status: None,
            routine_saved_state: None,
        }
    }

    pub async fn task(&mut self) {
        let mut last_pid_update = Instant::now();

        loop {
            while !self.command_channel_receiver.is_empty() {
                let command = self.command_channel_receiver.try_receive();
                if let Ok(command) = command {
                    self.handle_command(command).await;
                }
            }

            if let Some(routine) = &mut self.current_routine {
                if routine.finished_executing {
                    info!("Routine finished executing");
                    self.handle_routine_exit();
                } else if let Some(status) = self.previous_status.as_ref() {
                    if let Some(command) = routine.step(status) {
                        self.handle_command(command).await;
                    }
                }
            }

            let (actual_boiler_control_target, actual_pump_control_target) = self.get_control_targets();

            let next_pid_update = Instant::now();
            let delta_t = (next_pid_update - last_pid_update).as_millis() as f32;
            last_pid_update = next_pid_update;

            let boiler_pid_out = self.update_boiler(actual_boiler_control_target, delta_t).await;
            let pump_pid_out = self.update_pump(actual_pump_control_target, delta_t).await;

            self.send_status(boiler_pid_out, pump_pid_out).await;

            Timer::after_millis(100).await;
        }
    }

    async fn update_pump(&mut self, actual_pump_control_target: GroupBrewControlTarget, delta_t: f32) -> PidOut<f32> {
        let pump_pv = match actual_pump_control_target {
            GroupBrewControlTarget::GroupFlowRate(target) => {
                self.pump_pid.setpoint = target as f32;
                self.pump_pid.set_parameters(self.configuration.pid_parameters.pump_flow_rate_params);

                self.group.get_flow_rate().unwrap_or(0.0) as f32
            },
            GroupBrewControlTarget::Pressure(target) => {
                self.pump_pid.setpoint = target as f32;
                self.pump_pid.set_parameters(self.configuration.pid_parameters.pump_pressure_params);

                self.group.get_pressure().unwrap_or(0.0) as f32
            },
            GroupBrewControlTarget::OutputFlowRate(target) => {
                self.pump_pid.setpoint = target as f32;
                self.pump_pid.set_parameters(self.configuration.pid_parameters.pump_output_flow_rate_params);

                0.0
            },
            _ => 0.0,
        };

        let pump_pid_out = self.pump_pid.step(PidIn::new(pump_pv, delta_t));

        match actual_pump_control_target {
            GroupBrewControlTarget::Off => self.group.set_pump_duty_cycle(0).await,
            GroupBrewControlTarget::FullOn => self.group.set_pump_duty_cycle(100).await,
            GroupBrewControlTarget::FixedDutyCycle(duty_cycle) => {
                self.group.set_pump_duty_cycle(duty_cycle).await
            }
            _ => self.group.set_pump_duty_cycle(pump_pid_out.out as u8).await,
        }
        pump_pid_out
    }

    async fn update_boiler(&mut self, actual_boiler_control_target: BoilerControlTarget, delta_t: f32) -> PidOut<f32> {
        let boiler_pv = match actual_boiler_control_target {
            BoilerControlTarget::Temperature(target) => {
                self.boiler_pid.setpoint = target as f32;
                self.boiler_pid.set_parameters(self.configuration.pid_parameters.boiler_pressure_params);

                self.boiler.get_temperature().unwrap_or(0.0) as f32
            }
            BoilerControlTarget::Pressure(target) => {
                self.boiler_pid.setpoint = target as f32;
                self.boiler_pid.set_parameters(self.configuration.pid_parameters.boiler_pressure_params);

                self.boiler.get_pressure().unwrap_or(0.0) as f32
            }
            _ => 0.0,
        };

        let boiler_pid_out = self.boiler_pid.step(PidIn::new(boiler_pv, delta_t));

        match actual_boiler_control_target {
            BoilerControlTarget::Off => self.boiler.set_heating_element_duty_cycle(0).await,
            _ => self.boiler.set_heating_element_duty_cycle(boiler_pid_out.out as u8).await,
        }
        boiler_pid_out
    }

    fn get_control_targets(&mut self) -> (BoilerControlTarget, GroupBrewControlTarget) {
        let (actual_boiler_control_target, actual_pump_control_target) = match self.state {
            SingleBoilerSingleGroupControllerState::Brewing => {
                (self.configuration.brew_boiler_control_target, self.configuration.group_brew_control_target)
            }
            SingleBoilerSingleGroupControllerState::PumpingToWaterTap => {
                (self.configuration.brew_boiler_control_target, GroupBrewControlTarget::FullOn)
            },
            SingleBoilerSingleGroupControllerState::BrewModeIdle => {
                (self.configuration.brew_boiler_control_target, GroupBrewControlTarget::Off)
            }
            SingleBoilerSingleGroupControllerState::SteamModeIdle => {
                (self.configuration.steam_boiler_control_target, GroupBrewControlTarget::Off)
            }
            SingleBoilerSingleGroupControllerState::PowerSave => {
                (BoilerControlTarget::Off, GroupBrewControlTarget::Off)
            }
        };
        (actual_boiler_control_target, actual_pump_control_target)
    }

    async fn send_status(&mut self, boiler_pid_out: PidOut<f32>, pump_pid_out: PidOut<f32>) {
        //info!("Pump PID out: {:?}", pump_pid_out);

        let status = Status {
            boiler_temp: self.boiler.get_temperature().unwrap_or(0.0) as f32,
            boiler_pressure: self.boiler.get_pressure(),
            brew_boiler_duty_cycle: self.boiler.get_heating_element_duty_cycle().await,
            pump_duty_cycle: self.group.get_pump_duty_cycle().unwrap_or(0),
            is_brewing: self.state == SingleBoilerSingleGroupControllerState::Brewing,
            group_flow_rate: self.group.get_flow_rate(),
            boiler_pid_output: Some(boiler_pid_out),
            pump_pid_output: Some(pump_pid_out),
            config_brew_boiler_control_target: self.configuration.brew_boiler_control_target,
            config_steam_boiler_control_target: self.configuration.steam_boiler_control_target,
            config_group_brew_control_target: self.configuration.group_brew_control_target,
            routine_running: self.current_routine.is_some(),
            routine_step: self.current_routine.as_ref().and_then(|r| r.current_step),
        };

        if self.status_channel_sender.is_full() {
            warn!("Status channel is full, dropping status");
        } else {
            self.status_channel_sender.try_publish(status).ok();
        }

        self.previous_status = Some(status);
    }

    async fn handle_command(&mut self, command: MachineCommand) {
        match command {
            MachineCommand::StartBrewing(_) => {
                self.transition_to_state(SingleBoilerSingleGroupControllerState::Brewing).await;
            }
            MachineCommand::StopBrewing(_) => {
                self.transition_to_state(SingleBoilerSingleGroupControllerState::BrewModeIdle).await;
            }
            MachineCommand::StartPumpingToWaterTap(_) => {
                self.transition_to_state(SingleBoilerSingleGroupControllerState::PumpingToWaterTap).await;
            }
            MachineCommand::StopPumpingToWaterTap(_) => {
                self.transition_to_state(SingleBoilerSingleGroupControllerState::BrewModeIdle).await;
            }
            MachineCommand::SetBoilerControlTarget(boiler_index, control_target) => {
                info!("Setting boiler control target for boiler {} to {:?}", boiler_index, control_target);
                match boiler_index {
                    0 => self.configuration.brew_boiler_control_target = control_target,
                    1 => self.configuration.steam_boiler_control_target = control_target,
                    _ => {
                        error!("Invalid boiler index: {}", boiler_index);
                    }
                }
            }
            MachineCommand::SetGroupBrewControlTarget(group_index, control_target) => {
                info!("Setting group brew control target for group {} to {:?}", group_index, control_target);
                if group_index == 0 {
                    self.configuration.group_brew_control_target = control_target;
                } else {
                    error!("Invalid group index: {}", group_index);
                }
            }
            MachineCommand::SetPidParameters(target, params) => {
                match target {
                    PidParameterTarget::BoilerPressure(_) => {
                        self.configuration.pid_parameters.boiler_pressure_params = params;
                    }
                    PidParameterTarget::BoilerTemperature(_) => {
                        self.configuration.pid_parameters.boiler_temperature_params = params;
                    }
                    PidParameterTarget::GroupFlowRate(_) => {
                        self.configuration.pid_parameters.pump_flow_rate_params = params;
                    }
                    PidParameterTarget::GroupPressure(_) => {
                        self.configuration.pid_parameters.pump_pressure_params = params;
                    }
                    PidParameterTarget::GroupOutputFlowRate(_) => {
                        self.configuration.pid_parameters.pump_output_flow_rate_params = params;
                    }
                }
            }
            MachineCommand::RunRoutine => {
                if self.current_routine.is_some() {
                    //warn!("Cannot run routine, already executing a routine");
                    return;
                }
                info!("Starting routine");
                self.routine_saved_configuration = Some(self.configuration);
                self.routine_saved_state = Some(self.state);
                self.current_routine = Some(RoutineExecutionContext::new(routine::create_shot_routine(0)));
                info!("Routine started");
            }
            MachineCommand::CancelRoutine => {
                info!("Cancelling routine");
                self.handle_routine_exit();
            }
        }
    }

    async fn transition_to_state(&mut self, new_state: SingleBoilerSingleGroupControllerState) {
        if self.state != new_state {
            info!("Transitioning from {:?} to {:?}", self.state, new_state);
            self.state = new_state;
            
            match new_state {
                SingleBoilerSingleGroupControllerState::Brewing => {
                    info!("Starting brewing");
                    self.group.set_brew_state(true).await;
                }
                SingleBoilerSingleGroupControllerState::BrewModeIdle => {
                    info!("Stopping brewing");
                    self.group.set_brew_state(false).await;
                }
                _ => {}
            }
        }
    }
    
    fn handle_routine_exit(&mut self) {
        self.current_routine = None;
        self.configuration = self.routine_saved_configuration.take().unwrap_or(self.configuration);
        self.state = self.routine_saved_state.take().unwrap_or(SingleBoilerSingleGroupControllerState::BrewModeIdle);
    }
}