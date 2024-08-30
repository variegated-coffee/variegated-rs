use defmt::Format;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant};
use movavg::MovAvg;
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

    boiler_temperature_ma: MovAvg<f32, f32, 5>,
    boiler_pressure_ma: MovAvg<f32, f32, 5>,

    scale_value_ma: MovAvg<f32, f32, 5>,

    scale_tare_g: f32,
    
    prev_update: Instant,
    brew_boiler_pid: PidCtrl<f32>,
    brew_pressure_pid: PidCtrl<f32>,
    system_configuration: SilviaSystemConfiguration,

    prev_water_switch: Option<bool>,
    prev_water_switch_at: Option<Instant>,

    solenoid_open: bool,
    close_solenoid_at: Option<Instant>,

    brew_started_at: Option<Instant>,
}

impl<'a> SilviaController<'a> {
    pub fn new(command_channel: &'a Channel<NoopRawMutex, SilviaCommands, 10>) -> Self {
        let mut brew_temp_pid = PidCtrl::new_with_pid(5.0, 0.01, 100.0);
        brew_temp_pid.limits.try_set_lower(0.0);
        brew_temp_pid.limits.try_set_upper(100.0);
        brew_temp_pid.ki.limits.set_limit(10.0);
        let mut brew_pressure_pid = PidCtrl::new_with_pid(1.0, 0.01, 400.0);
        brew_pressure_pid.limits.try_set_lower(0.0);
        brew_pressure_pid.limits.try_set_upper(100.0);
        brew_pressure_pid.kp.limits.set_limit(100.0);
        brew_pressure_pid.ki.limits.try_set_lower(0.0);
        brew_pressure_pid.ki.limits.try_set_upper(90.0);

        let mut system_configuration = SilviaSystemConfiguration::default();
        system_configuration.boiler_temp_setpoint_c = 100.0;
        system_configuration.pump_pressure_setpoint_preinfusion_bar = 3.0;
        system_configuration.pump_pressure_setpoint_ramp_up_bar = 6.0;
        system_configuration.pump_pressure_setpoint_extraction_bar = 9.0;

        Self {
            command_channel,
            boiler_temperature_ma: MovAvg::new(),
            boiler_pressure_ma: MovAvg::new(),
            scale_value_ma: MovAvg::new(),
            scale_tare_g: 0.0,
            prev_update: Instant::now(),
            brew_boiler_pid: brew_temp_pid,
            brew_pressure_pid,
            system_configuration,
            prev_water_switch: None,
            prev_water_switch_at: None,
            close_solenoid_at: None,
            solenoid_open: false,
            brew_started_at: None,
        }
    }
    pub async fn update_states_from_sensor_state(&mut self, system_sensor_state: &SilviaSystemSensorState, system_actuator_state: &mut SilviaSystemActuatorState, general_state: &mut SilviaSystemGeneralState) -> Result<(), ControllerError> {
        let now = Instant::now();
        
        self.boiler_temperature_ma.feed(system_sensor_state.boiler_temp_c);
        self.boiler_pressure_ma.feed(system_sensor_state.boiler_pressure_bar);
        self.scale_value_ma.feed(system_sensor_state.scale_untared_g);

        if self.prev_water_switch.is_none() {
            self.prev_water_switch = Some(system_sensor_state.water_switch);
            self.prev_water_switch_at = Some(now);
        }

        if system_sensor_state.brew_switch {
            general_state.is_brewing = true;
        } else {
            general_state.is_brewing = false;
        }

        if general_state.is_brewing {
            if !self.solenoid_open {
                self.solenoid_open = true;
            }

            if self.brew_started_at.is_none() {
                self.brew_started_at = Some(now);
                self.scale_tare_g = self.scale_value_ma.get();
            }

            general_state.scale_tared_g = self.scale_value_ma.get() - self.scale_tare_g;
        } else {
            if self.solenoid_open && self.close_solenoid_at.is_none() {
                self.close_solenoid_at = Some(now + Duration::from_millis(400));
            }

            if let Some(close_solenoid_at) = self.close_solenoid_at {
                if now > close_solenoid_at {
                    self.solenoid_open = false;
                    self.close_solenoid_at = None;
                }
            }

            self.brew_started_at = None;
            //general_state.scale_tared_g = self.scale_value_ma.get();
            general_state.scale_tared_g = 0.0;
        }

        system_actuator_state.brew_solenoid = self.solenoid_open;
        if let Some(brew_started_at) = self.brew_started_at {
            general_state.brew_time_s = Some((now - brew_started_at).as_millis() as f32 / 1000.0);
        } else {
            general_state.brew_time_s = None;
        }

        if system_sensor_state.water_switch != self.prev_water_switch.unwrap() {
            // Debounce the water switch
            if let Some(prev_water_switch_at) = self.prev_water_switch_at {
                if now - prev_water_switch_at > Duration::from_millis(500) {
                    if system_sensor_state.water_switch {
                        general_state.extraction_phase = general_state.extraction_phase.next();
                        self.prev_water_switch_at = Some(now);
                    }
                    self.prev_water_switch = Some(system_sensor_state.water_switch);
                }
            }
        }

        let delta = now - self.prev_update;
        let delta_millis = delta.as_micros().to_f32().expect("For some weird reason, we couldn't convert a u64 to a f32") / 1000f32;

        self.brew_boiler_pid.setpoint = self.system_configuration.boiler_temp_setpoint_c;
        general_state.current_pressure_set_point = if general_state.is_brewing {
            self.system_configuration.pressure_set_point_for_phase(general_state.extraction_phase, general_state.brew_time_s.unwrap_or(0.0))
        } else {
            0.0
        };
        self.brew_pressure_pid.setpoint = general_state.current_pressure_set_point;

        let brew_out = self.brew_boiler_pid.step(PidIn::new(self.boiler_temperature_ma.get(), delta_millis));
        general_state.brew_boiler_pid = brew_out;
        //log::info!("Brew temp MA: {:?}, PID: {:?}", self.brew_temperature_ma.get(), brew_out);

        // Avoid integral wind-up when not brewing
        if !general_state.is_brewing {
            self.brew_pressure_pid.ki.limits.try_set_upper(0.0);
        } else {
            self.brew_pressure_pid.ki.limits.try_set_upper(90.0);
        }

        let pressure_out = self.brew_pressure_pid.step(PidIn::new(self.boiler_pressure_ma.get(), delta_millis));
        general_state.brew_pressure_pid = pressure_out;

//        log::info!("Pressure PID: {:?}", pressure_out);
        
        system_actuator_state.brew_boiler_heating_element.set_duty_cycle(brew_out.out as u8);
        system_actuator_state.brew_boiler_heating_element.update(now);

        self.prev_update = now;

        if general_state.is_brewing {
            system_actuator_state.pump_duty_cycle = pressure_out.out;
        } else {
            system_actuator_state.pump_duty_cycle = 0.0;
        }

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
