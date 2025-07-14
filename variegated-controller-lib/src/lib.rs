#![no_std]

pub mod routine;

extern crate alloc;

use defmt::{error, info, warn, Format};
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, RawMutex};
use embassy_sync::channel::{Receiver};
use embassy_sync::mutex::Mutex;
use embassy_sync::pubsub::Publisher;
use embassy_time::{Instant, Timer};
use heapless::FnvIndexMap;
use movavg::MovAvg;
use variegated_control_algorithm::pid::{PidCtrl, PidIn, PidOut};
use variegated_hal::{Boiler, Group};
use variegated_controller_types::{BoilerControlTarget, BoilerStatus, FlowRateType, GroupBrewControlTarget, GroupStatus, MachineCommand, Output, PidLimits, PidParameterTarget, PidParameters, PidTerm, PressureType, RoutineIndex, SingleBoilerSingleGroupControllerState, Status};
use variegated_controller_types::SingleBoilerSingleGroupControllerBoilers::{BrewBoiler, VirtualSteamBoiler};
use variegated_controller_types::SingleGroupControllerGroups::SingleGroup;
use crate::routine::{InMemoryRoutineRepository, RoutineExecutionContext};

fn limited_pid() -> PidCtrl<f32> {
    let mut pid = PidCtrl::default();
    pid.limits.try_set_lower(0.0).unwrap();
    pid.limits.try_set_upper(100.0).unwrap();
    pid
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SingleBoilerSingleGroupPidParameters {
    pub boiler_pressure_params: PidParameters,
    pub boiler_temperature_params: PidParameters,
    pub pump_flow_rate_params: PidParameters,
    pub pump_pressure_params: PidParameters,
    pub pump_output_flow_rate_params: PidParameters,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SingleBoilerSingleGroupConfiguration {
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
    routine_repository: &'static Mutex<NoopRawMutex, InMemoryRoutineRepository>,
    current_routine: Option<RoutineExecutionContext<SingleBoilerSingleGroupControllerState, SingleBoilerSingleGroupConfiguration>>,
    previous_status: Option<Status>,
    temperature_movavg: MovAvg<f32, f32, 10>,
}

impl <'a, ChannelM: RawMutex, M: RawMutex, const N_CHANNEL: usize, const N_WATCH: usize, const N_SUBS: usize> SingleBoilerSingleGroupController<'a, ChannelM, M, N_CHANNEL, N_WATCH, N_SUBS> {
    pub fn new(
        command_channel_receiver: Receiver<'a, ChannelM, MachineCommand, N_CHANNEL>,
        status_channel_sender: Publisher<'a, ChannelM, Status, 1, N_SUBS, 1>,
        boiler: Boiler<'a, M, N_WATCH>,
        group: Group<'a, M, N_WATCH>,
        configuration: SingleBoilerSingleGroupConfiguration,
        routine_repository: &'static Mutex<NoopRawMutex, InMemoryRoutineRepository>,
    ) -> Self {
        Self {
            command_channel_receiver,
            status_channel_sender,
            boiler,
            group,
            state: SingleBoilerSingleGroupControllerState::default(),
            boiler_pid: limited_pid(),
            pump_pid: limited_pid(),
            configuration,
            routine_repository,
            current_routine: None,
            previous_status: None,
            temperature_movavg: MovAvg::default(),
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
                    self.handle_routine_exit().await;
                } else if let Some(status) = self.previous_status.as_ref() {
                    if let Some(command) = routine.step(status, None) {
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

    async fn update_pump(&mut self, actual_pump_control_target: GroupBrewControlTarget, delta_t: f32) -> Output {
        let pump_pv = match actual_pump_control_target {
            GroupBrewControlTarget::GroupFlowRate(target) => {
                self.pump_pid.setpoint = target as f32;
                self.pump_pid.set_parameters(self.configuration.pid_parameters.pump_flow_rate_params);

                self.group.get_input_flow_rate().unwrap_or(0.0) as f32
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
            GroupBrewControlTarget::Off => {
                self.group.set_pump_duty_cycle(0).await;
                Output::Off
            },
            GroupBrewControlTarget::FullOn => {
                self.group.set_pump_duty_cycle(100).await;
                Output::FixedDutyCycle(100)
            },
            GroupBrewControlTarget::FixedDutyCycle(duty_cycle) => {
                self.group.set_pump_duty_cycle(duty_cycle).await;
                Output::FixedDutyCycle(duty_cycle)
            }
            _ => {
                self.group.set_pump_duty_cycle(pump_pid_out.out as u8).await;
                Output::PidOutput(pump_pid_out)
            },
        }
    }

    async fn update_boiler(&mut self, actual_boiler_control_target: BoilerControlTarget, delta_t: f32) -> Output {
        let mut boiler_pv = match actual_boiler_control_target {
            BoilerControlTarget::Temperature(target) => {
                self.boiler_pid.setpoint = target as f32;
                self.boiler_pid.set_parameters(self.configuration.pid_parameters.boiler_temperature_params);

                self.boiler.get_temperature().unwrap_or(0.0) as f32
            }
            BoilerControlTarget::Pressure(target) => {
                self.boiler_pid.setpoint = target as f32;
                self.boiler_pid.set_parameters(self.configuration.pid_parameters.boiler_pressure_params);

                self.boiler.get_pressure().unwrap_or(0.0) as f32
            }
            _ => 0.0,
        };
        
        if let Ok(pv) = self.temperature_movavg.try_feed(boiler_pv) {
            boiler_pv = pv;
        }

        let boiler_pid_out = self.boiler_pid.step(PidIn::new(boiler_pv, delta_t));

        match actual_boiler_control_target {
            BoilerControlTarget::Off => {
                self.boiler.set_heating_element_duty_cycle(0).await;
                Output::Off
            },
            _ => {
                self.boiler.set_heating_element_duty_cycle(boiler_pid_out.out as u8).await;
                Output::PidOutput(boiler_pid_out)
            },
        }
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

    async fn send_status(&mut self, boiler_output: Output, pump_output: Output) {
        let (brew_boiler_output, steam_boiler_output) = match self.state {
            SingleBoilerSingleGroupControllerState::SteamModeIdle => (Output::Off, boiler_output.clone()),
            _ => (boiler_output.clone(), Output::Off),
        };

        let brew_boiler_status = BoilerStatus {
            temperature: self.boiler.get_temperature(),
            pressure: self.boiler.get_pressure(),
            output: brew_boiler_output,
            control_target: self.configuration.brew_boiler_control_target,
        };

        let virtual_steam_boiler_status = BoilerStatus {
            temperature: self.boiler.get_temperature(),
            pressure: self.boiler.get_pressure(),
            output: steam_boiler_output,
            control_target: self.configuration.steam_boiler_control_target,
        };

        let group_status = GroupStatus {
            is_brewing: self.state == SingleBoilerSingleGroupControllerState::Brewing,
            three_way_valve_open: self.group.get_three_way_valve_open(),
            brew_time: None,
            input_flow_rate: self.group.get_input_flow_rate(),
            output_flow_rate: self.group.get_output_flow_rate(),
            output_weight: self.group.get_output_weight(),
            pressure: self.group.get_pressure(),
            temperature: self.group.get_temperature(),
            pump_output: pump_output.clone(),
            control_target: self.configuration.group_brew_control_target,
        };

        let status = Status {
            boiler_statuses: FnvIndexMap::from_iter([(BrewBoiler.as_index(), brew_boiler_status), (VirtualSteamBoiler.as_index(), virtual_steam_boiler_status)]),
            group_statuses: FnvIndexMap::from_iter([(SingleGroup.as_index(), group_status)]),
            mode: Default::default(),
            current_routine: self.current_routine.as_ref().and_then(|rxc| Some(rxc.routine_index)),
            routine_step: self.current_routine.as_ref().and_then(|rxc| rxc.current_step),
        };

        self.status_channel_sender.publish_immediate(status.clone());

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
            MachineCommand::RunRoutine(usize) => {
                info!("Received command to run routine with index: {}", usize);
                self.handle_routine_start(usize).await;
            }
            MachineCommand::CancelRoutine => {
                info!("Cancelling routine");
                self.handle_routine_exit().await;
            }
            MachineCommand::EnableBoiler(boiler_index) => {
                if boiler_index == 0 && self.state == SingleBoilerSingleGroupControllerState::SteamModeIdle {
                    info!("Enabling brew boiler");
                    self.transition_to_state(SingleBoilerSingleGroupControllerState::BrewModeIdle).await;
                } else if boiler_index == 1 && self.state == SingleBoilerSingleGroupControllerState::BrewModeIdle {
                    info!("Enabling steam boiler");
                    self.transition_to_state(SingleBoilerSingleGroupControllerState::SteamModeIdle).await;
                } else {
                    warn!("Invalid boiler index or state for enabling boiler: {} Current state: {:?}", boiler_index, self.state);
                }
            }
            MachineCommand::DisableBoiler(boiler_index) => {
                if boiler_index == 1 && self.state == SingleBoilerSingleGroupControllerState::SteamModeIdle {
                    info!("Going back to brew mode");
                    self.transition_to_state(SingleBoilerSingleGroupControllerState::BrewModeIdle).await;
                } else if boiler_index == 0 && self.state == SingleBoilerSingleGroupControllerState::BrewModeIdle {
                    info!("Going in to power save mode");
                    self.transition_to_state(SingleBoilerSingleGroupControllerState::PowerSave).await;
                } else {
                    warn!("Invalid boiler index or state for disabling boiler: {} Current state: {:?}", boiler_index, self.state);
                }
            }
        }
    }

    async fn transition_to_state(&mut self, new_state: SingleBoilerSingleGroupControllerState) {
        if self.state != new_state {
            info!("Transitioning from {:?} to {:?}", self.state, new_state);
            let old_state = self.state;
            self.state = new_state;

            match (old_state, new_state) {
                (SingleBoilerSingleGroupControllerState::BrewModeIdle, SingleBoilerSingleGroupControllerState::Brewing) => {
                    info!("Starting brewing");
                    self.group.set_brew_state(true).await;
                    self.started_brewing().await;
                }
                (SingleBoilerSingleGroupControllerState::Brewing, SingleBoilerSingleGroupControllerState::BrewModeIdle) => {
                    info!("Stopping brewing");
                    self.group.set_brew_state(false).await;
                    self.stopped_brewing().await;
                }
                _ => {}
            }
        }
    }
    
    async fn started_brewing(&mut self) {
        self.boiler_pid.ki.accumulate += 50.0; // Initial accumulation to compensate for initial temperature drop
    }
    
    async fn stopped_brewing(&mut self) {
    }

    async fn handle_routine_start(&mut self, routine_index: RoutineIndex) {
        if self.current_routine.is_some() {
            //warn!("Cannot run routine, already executing a routine");
            return;
        }
        let repo = self.routine_repository.lock().await;
        let routine = repo.get_routine(routine_index);

        if let Some(routine) = routine {
            info!("Running routine");
            self.current_routine = Some(RoutineExecutionContext::new(routine_index, routine.clone(), self.state, self.configuration));
            info!("Routine started");
        } else {
            error!("Routine not found: {}", routine_index);
        }
    }

    async fn handle_routine_exit(&mut self) {
        if let Some(routine) = self.current_routine.take() {
            info!("Routine execution finished, saving state and configuration");
            self.configuration = routine.saved_configuration;
            self.transition_to_state(routine.saved_state).await;
        } else {
            warn!("No routine to exit");
        }
    }
}