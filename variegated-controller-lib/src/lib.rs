#![no_std]

extern crate alloc;

use core::cmp::PartialEq;
use defmt::{info, warn, Format};
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_time::{Instant, Timer};
use pid_ctrl::{PidCtrl, PidIn, PidOut};
use variegated_hal::{Boiler, FlowRateType, Group, PressureType, TemperatureType};
use num_traits::float::FloatCore;

type BoilerIndex = u8;
type GroupIndex = u8;

#[derive(Clone, Copy, Debug, Format)]
pub enum MachineCommand {
    StartBrewing,
    StopBrewing,
    StartPumpingToWaterTap,
    StopPumpingToWaterTap,
    SetBoilerControlTarget(BoilerIndex, BoilerControlTarget),
    SetGroupBrewControlTarget(GroupIndex, GroupBrewControlTarget),
}

#[derive(Clone, Copy, Debug, Format, Default)]
pub enum BoilerControlTarget {
    Temperature(TemperatureType, PidParameters),
    Pressure(PressureType, PidParameters),
    #[default]
    Off
}

#[derive(Clone, Copy, Debug, Format, PartialEq, Default)]
pub struct PidTerm {
    pub scale: f32,
    pub min: Option<f32>,
    pub max: Option<f32>,
}

#[derive(Clone, Copy, Debug, Format, PartialEq, Default)]
pub struct PidParameters {
    pub kp: PidTerm,
    pub ki: PidTerm,
    pub kd: PidTerm,
}

#[derive(Clone, Copy, Debug, Format, Default)]
pub enum GroupBrewControlTarget {
    GroupFlowRate(FlowRateType, PidParameters),
    Pressure(PressureType, PidParameters),
    OutputFlowRate(FlowRateType, PidParameters),
    FixedDutyCycle(u8),
    FullOn,
    #[default]
    Off
}

#[derive(Clone, Copy, Default, Debug, Format, PartialEq)]
pub enum SingleBoilerSingleGroupControllerState {
    #[default]
    BrewModeIdle,
    SteamModeIdle,
    Brewing,
    PumpingToWaterTap,
    PowerSave,
}

#[derive(Clone, Copy, Debug, Format)]
#[repr(u8)]
pub enum SingleBoilerSingleGroupControllerBoilers {
    BrewBoiler = 0,
    VirtualSteamBoiler = 1,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Status {
    pub boiler_temp: TemperatureType,
    pub boiler_pressure: Option<PressureType>,
    pub brew_boiler_duty_cycle: u8,
    pub pump_duty_cycle: u8,
    pub is_brewing: bool,
    pub group_flow_rate: Option<FlowRateType>,
    pub boiler_pid_output: Option<PidOut<f32>>,
    pub pump_pid_output: Option<PidOut<f32>>,
    pub config_brew_boiler_control_target: BoilerControlTarget,
    pub config_steam_boiler_control_target: BoilerControlTarget,
    pub config_group_brew_control_target: GroupBrewControlTarget,
}

fn limited_pid() -> PidCtrl<f32> {
    let mut pid = PidCtrl::default();
    pid.limits.try_set_lower(0.0).unwrap();
    pid.limits.try_set_upper(100.0).unwrap();
    pid
}

pub struct SingleBoilerSingleGroupController<'a, ChannelM: RawMutex, M: RawMutex, const N_CHANNEL: usize, const N_WATCH: usize> {
    command_channel_receiver: Receiver<'a, ChannelM, MachineCommand, N_CHANNEL>,
    status_channel_sender: Sender<'a, ChannelM, Status, N_CHANNEL>,
    boiler: Boiler<'a, M, N_WATCH>,
    group: Group<'a, M, N_WATCH>,
    brew_boiler_control_target: BoilerControlTarget,
    steam_boiler_control_target: BoilerControlTarget,
    group_brew_control_target: GroupBrewControlTarget,
    state: SingleBoilerSingleGroupControllerState,
    boiler_pid: PidCtrl<f32>,
    pump_pid: PidCtrl<f32>,
    boiler_pid_params: PidParameters,
    pump_pid_params: PidParameters,
}

impl <'a, ChannelM: RawMutex, M: RawMutex, const N_CHANNEL: usize, const N_WATCH: usize> SingleBoilerSingleGroupController<'a, ChannelM, M, N_CHANNEL, N_WATCH> {
    pub fn new(
        command_channel_receiver: Receiver<'a, ChannelM, MachineCommand, N_CHANNEL>,
        status_channel_sender: Sender<'a, ChannelM, Status, N_CHANNEL>,
        boiler: Boiler<'a, M, N_WATCH>,
        group: Group<'a, M, N_WATCH>,
        brew_boiler_control_target: BoilerControlTarget,
        steam_boiler_control_target: BoilerControlTarget,
        group_brew_control_target: GroupBrewControlTarget,
    ) -> Self {
        Self {
            command_channel_receiver,
            status_channel_sender,
            boiler,
            group,
            brew_boiler_control_target,
            steam_boiler_control_target,
            group_brew_control_target,
            state: SingleBoilerSingleGroupControllerState::default(),
            boiler_pid: limited_pid(),
            pump_pid: limited_pid(),
            boiler_pid_params: PidParameters::default(),
            pump_pid_params: PidParameters::default(),
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

            let (actual_boiler_control_target, actual_pump_control_target) = match self.state {
                SingleBoilerSingleGroupControllerState::Brewing => {
                    (self.brew_boiler_control_target, self.group_brew_control_target)
                }
                SingleBoilerSingleGroupControllerState::PumpingToWaterTap => {
                    (self.brew_boiler_control_target, GroupBrewControlTarget::FullOn)
                },
                SingleBoilerSingleGroupControllerState::BrewModeIdle => {
                    (self.brew_boiler_control_target, GroupBrewControlTarget::Off)
                }
                SingleBoilerSingleGroupControllerState::SteamModeIdle => {
                    (self.steam_boiler_control_target, GroupBrewControlTarget::Off)
                }
                SingleBoilerSingleGroupControllerState::PowerSave => {
                    (BoilerControlTarget::Off, GroupBrewControlTarget::Off)
                }
            };

            //info!("State: {:?}", self.state);

            let next_pid_update = Instant::now();
            let delta_t = (next_pid_update - last_pid_update).as_millis() as f32;
            last_pid_update = next_pid_update;

            let boiler_pv = match actual_boiler_control_target {
                BoilerControlTarget::Temperature(_, _) => {
                    self.boiler.get_temperature().unwrap_or(0.0) as f32
                }
                BoilerControlTarget::Pressure(_, _) => {
                    self.boiler.get_pressure().unwrap_or(0.0) as f32
                }
                _ => 0.0,
            };

            match actual_boiler_control_target {
                BoilerControlTarget::Temperature(target, params) => {
                    self.boiler_pid.setpoint = target as f32;
                    self.update_boiler_pid_params(params);
                },
                BoilerControlTarget::Pressure(target, params) => {
                    self.boiler_pid.setpoint = target as f32;
                    self.update_boiler_pid_params(params);
                },
                _ => {},
            };

            //info!("Boiler setpoint: {:?}", self.boiler_pid.setpoint);
            //info!("Boiler PV: {:?}, dT: {:?}", boiler_pv, delta_t);

            let boiler_pid_out = self.boiler_pid.step(PidIn::new(boiler_pv, delta_t));

            match actual_boiler_control_target {
                BoilerControlTarget::Off => self.boiler.set_heating_element_duty_cycle(0).await,
                _ => self.boiler.set_heating_element_duty_cycle(boiler_pid_out.out as u8).await,
            }

            //info!("Boiler P: {:?}, I: {:?}, D: {:?}", boiler_pid_out.p, boiler_pid_out.i, boiler_pid_out.d);


            let pump_pv = match actual_pump_control_target {
                GroupBrewControlTarget::GroupFlowRate(_, _) => {
                    self.group.get_flow_rate().unwrap_or(0.0) as f32
                },
                GroupBrewControlTarget::Pressure(_, _) => {
                    self.group.get_pressure().unwrap_or(0.0) as f32
                },
                GroupBrewControlTarget::OutputFlowRate(_, _) => 0.0,
                _ => 0.0,
            };

            match actual_pump_control_target {
                GroupBrewControlTarget::GroupFlowRate(target, params) => {
                    self.pump_pid.setpoint = target as f32;
                    self.update_pump_pid_params(params);
                }
                GroupBrewControlTarget::Pressure(target, params) => {
                    self.pump_pid.setpoint = target as f32;
                    self.update_pump_pid_params(params);
                }
                GroupBrewControlTarget::OutputFlowRate(target, params) => {
                    self.pump_pid.setpoint = target as f32;
                    self.update_pump_pid_params(params);
                }
                _ => {}
            }

            let pump_pid_out = self.pump_pid.step(PidIn::new(pump_pv, delta_t));
            //info!("Pump PV: {:?}, P: {:?}, I: {:?}, D: {:?}, OV: {:?}", pump_pv, pump_pid_out.p, pump_pid_out.i, pump_pid_out.d, pump_pid_out.out);

            match actual_pump_control_target {
                GroupBrewControlTarget::Off => self.group.set_pump_duty_cycle(0).await,
                GroupBrewControlTarget::FullOn => self.group.set_pump_duty_cycle(100).await,
                GroupBrewControlTarget::FixedDutyCycle(duty_cycle) => {
                    self.group.set_pump_duty_cycle(duty_cycle).await
                }
                _ => self.group.set_pump_duty_cycle(pump_pid_out.out as u8).await,
            }

            let status = Status {
                boiler_temp: self.boiler.get_temperature().unwrap_or(0.0) as f32,
                boiler_pressure: self.boiler.get_pressure(),
                brew_boiler_duty_cycle: self.boiler.get_heating_element_duty_cycle().await,
                pump_duty_cycle: self.group.get_pump_duty_cycle().unwrap_or(0),
                is_brewing: self.state == SingleBoilerSingleGroupControllerState::Brewing,
                group_flow_rate: self.group.get_flow_rate(),
                boiler_pid_output: Some(boiler_pid_out),
                pump_pid_output: Some(pump_pid_out),
                config_brew_boiler_control_target: self.brew_boiler_control_target,
                config_steam_boiler_control_target: self.steam_boiler_control_target,
                config_group_brew_control_target: self.group_brew_control_target,
            };

            if self.status_channel_sender.is_full() {
                warn!("Status channel is full, dropping status");
            } else {
                self.status_channel_sender.try_send(status).ok();
            }

            // Implement the control logic here
            Timer::after_millis(100).await;
        }
    }

    async fn handle_command(&mut self, command: MachineCommand) {
        match command {
            MachineCommand::StartBrewing => {
                self.state = SingleBoilerSingleGroupControllerState::Brewing;
                info!("Starting brewing");
                self.group.set_brew_state(true).await;
            }
            MachineCommand::StopBrewing => {
                self.state = SingleBoilerSingleGroupControllerState::BrewModeIdle;
                info!("Stopping brewing");
                self.group.set_brew_state(false).await;
            }
            MachineCommand::StartPumpingToWaterTap => {
                self.state = SingleBoilerSingleGroupControllerState::PumpingToWaterTap;
                info!("Starting pumping to water tap");
            }
            MachineCommand::StopPumpingToWaterTap => {
                self.state = SingleBoilerSingleGroupControllerState::BrewModeIdle;
                info!("Stopping pumping to water tap");
            }
            MachineCommand::SetBoilerControlTarget(boiler_index, control_target) => {
                match boiler_index {
                    0 => self.brew_boiler_control_target = control_target,
                    1 => self.steam_boiler_control_target = control_target,
                    _ => {}
                }
            }
            MachineCommand::SetGroupBrewControlTarget(group_index, control_target) => {
                if group_index == 0 {
                    info!("Setting group brew control target to {:?}", control_target);
                    self.group_brew_control_target = control_target;
                }
            }
        }
    }

    fn update_boiler_pid_params(&mut self, params: PidParameters) {
        if self.boiler_pid_params == params {
            return;
        }

        self.boiler_pid.kp.set_scale(params.kp.scale);
        self.boiler_pid.ki.set_scale(params.ki.scale);
        self.boiler_pid.kd.set_scale(params.kd.scale);

        if let Some(min) = params.kp.min {
            self.boiler_pid.kp.limits.try_set_lower(min).unwrap();
        } else {
            self.boiler_pid.kp.limits.try_set_lower(f32::neg_infinity()).unwrap();
        }

        if let Some(max) = params.kp.max {
            self.boiler_pid.kp.limits.try_set_upper(max).unwrap();
        } else {
            self.boiler_pid.kp.limits.try_set_upper(f32::infinity()).unwrap();
        }

        if let Some(min) = params.ki.min {
            self.boiler_pid.ki.limits.try_set_lower(min).unwrap();
        } else {
            self.boiler_pid.ki.limits.try_set_lower(f32::neg_infinity()).unwrap();
        }

        if let Some(max) = params.ki.max {
            self.boiler_pid.ki.limits.try_set_upper(max).unwrap();
        } else {
            self.boiler_pid.ki.limits.try_set_upper(f32::infinity()).unwrap();
        }

        if let Some(min) = params.kd.min {
            self.boiler_pid.kd.limits.try_set_lower(min).unwrap();
        } else {
            self.boiler_pid.kd.limits.try_set_lower(f32::neg_infinity()).unwrap();
        }

        if let Some(max) = params.kd.max {
            self.boiler_pid.kd.limits.try_set_upper(max).unwrap();
        } else {
            self.boiler_pid.kd.limits.try_set_upper(f32::infinity()).unwrap();
        }

        self.boiler_pid_params = params;
        self.reset_boiler_pid();
    }

    fn update_pump_pid_params(&mut self, params: PidParameters) {
        if self.pump_pid_params == params {
            return;
        }

        self.pump_pid.kp.set_scale(params.kp.scale);
        self.pump_pid.ki.set_scale(params.ki.scale);
        self.pump_pid.kd.set_scale(params.kd.scale);

        if let Some(min) = params.kp.min {
            self.pump_pid.kp.limits.try_set_lower(min).unwrap();
        } else {
            self.pump_pid.kp.limits.try_set_lower(f32::neg_infinity()).unwrap();
        }

        if let Some(max) = params.kp.max {
            self.pump_pid.kp.limits.try_set_upper(max).unwrap();
        } else {
            self.pump_pid.kp.limits.try_set_upper(f32::infinity()).unwrap();
        }

        if let Some(min) = params.ki.min {
            self.pump_pid.ki.limits.try_set_lower(min).unwrap();
        } else {
            self.pump_pid.ki.limits.try_set_lower(f32::neg_infinity()).unwrap();
        }

        if let Some(max) = params.ki.max {
            self.pump_pid.ki.limits.try_set_upper(max).unwrap();
        } else {
            self.pump_pid.ki.limits.try_set_upper(f32::infinity()).unwrap();
        }

        if let Some(min) = params.kd.min {
            self.pump_pid.kd.limits.try_set_lower(min).unwrap();
        } else {
            self.pump_pid.kd.limits.try_set_lower(f32::neg_infinity()).unwrap();
        }

        if let Some(max) = params.kd.max {
            self.pump_pid.kd.limits.try_set_upper(max).unwrap();
        } else {
            self.pump_pid.kd.limits.try_set_upper(f32::infinity()).unwrap();
        }

        self.pump_pid_params = params;
        self.reset_pump_pid();
    }

    fn reset_boiler_pid(&mut self) {
        self.boiler_pid.ki.accumulate = 0.0;
        self.boiler_pid.kd.prev_measurement = 0.0;
    }

    fn reset_pump_pid(&mut self) {
        self.pump_pid.ki.accumulate = 0.0;
        self.pump_pid.kd.prev_measurement = 0.0;
    }
}