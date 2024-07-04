use defmt::Format;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::Instant;
use pid_ctrl::{PidCtrl, PidIn};
use variegated_controller_lib::{Controller, ControllerError};
use crate::state::{SilviaSystemActuatorState, SilviaSystemConfiguration, SilviaSystemGeneralState, SilviaSystemSensorState};
use num_traits::ToPrimitive;
use variegated_log::log_info;

#[derive(Debug, Format)]
pub enum SilviaCommands {
    SetBoilerSetPointCelsius(f32),
    SetBrewPressureBar(f32),
}

pub struct SilviaController<'a> {
    command_channel: &'a Channel<NoopRawMutex, SilviaCommands, 10>,

    prev_update: Instant,
    brew_boiler_pid: PidCtrl<f32>,
    brew_pressure_pid: PidCtrl<f32>,
    system_configuration: SilviaSystemConfiguration,
}

impl<'a> SilviaController<'a> {
    pub fn new(command_channel: &'a Channel<NoopRawMutex, SilviaCommands, 10>) -> Self {
        let mut brew_temp_pid = PidCtrl::new_with_pid(5.0, 5.0, 5.0);
        brew_temp_pid.limits.try_set_lower(0.0);
        brew_temp_pid.limits.try_set_upper(100.0);
        brew_temp_pid.ki.limits.set_limit(1000.0);
        let mut brew_pressure_pid = PidCtrl::new_with_pid(5.0, 0.1, 5.0);
        brew_pressure_pid.limits.try_set_lower(0.0);
        brew_pressure_pid.limits.try_set_upper(100.0);
        brew_pressure_pid.kp.limits.set_limit(10.0);
        brew_pressure_pid.ki.limits.try_set_lower(0.0);
        brew_pressure_pid.ki.limits.try_set_upper(100.0);

        let mut system_configuration = SilviaSystemConfiguration::default();
        system_configuration.boiler_temp_setpoint_c = 95.0;

        Self {
            command_channel,
            prev_update: Instant::now(),
            brew_boiler_pid: brew_temp_pid,
            brew_pressure_pid,
            system_configuration
        }
    }
    pub async fn update_actuator_state_from_sensor_state(&mut self, system_sensor_state: &SilviaSystemSensorState, system_actuator_state: &mut SilviaSystemActuatorState, general_state: &mut SilviaSystemGeneralState) -> Result<(), ControllerError> {
        let now = Instant::now();
        
        if system_sensor_state.brew_switch {
            general_state.is_brewing = true;
        } else {
            general_state.is_brewing = false;
        }

        let delta = now - self.prev_update;
        let delta_millis = delta.as_micros().to_f32().expect("For some weird reason, we couldn't convert a u64 to a f32") / 1000f32;

        self.brew_boiler_pid.setpoint = self.system_configuration.boiler_temp_setpoint_c;
        self.brew_pressure_pid.setpoint = if system_sensor_state.brew_switch {
            self.system_configuration.pump_pressure_setpoint_bar
        } else {
            0.0
        };

        let brew_out = self.brew_boiler_pid.step(PidIn::new(system_sensor_state.boiler_temp_c, delta_millis));
        let pressure_out = self.brew_pressure_pid.step(PidIn::new(system_sensor_state.boiler_pressure_bar, delta_millis));

//        log::info!("Pressure PID: {:?}", pressure_out);
        
        system_actuator_state.brew_boiler_heating_element.set_duty_cycle(brew_out.out as u8);
        system_actuator_state.brew_boiler_heating_element.update(now);

        if general_state.is_brewing {
            system_actuator_state.pump_duty_cycle = pressure_out.out;
        } else {
            system_actuator_state.pump_duty_cycle = 0.0;
        }

        self.prev_update = now;

        Ok(())
    }
    
    pub async fn handle_commands(&mut self) {
        while let Ok(command) = self.command_channel.try_receive() {
            log_info!("Received command: {:?}", command);
            
            match command {
                SilviaCommands::SetBoilerSetPointCelsius(setpoint) => {
                    self.system_configuration.boiler_temp_setpoint_c = setpoint;
                },
                SilviaCommands::SetBrewPressureBar(setpoint) => {
                    self.brew_pressure_pid.setpoint = setpoint;
                }
            }
        }
    }
}
