use embassy_time::Instant;
use pid_ctrl::{PidCtrl, PidIn};
use variegated_controller_lib::{Controller, ControllerError};
use crate::state::{SilviaSystemActuatorState, SilviaSystemConfiguration, SilviaSystemSensorState};
use num_traits::ToPrimitive;


pub struct SilviaController {
    prev_update: Instant,
    brew_boiler_pid: PidCtrl<f32>,
    system_configuration: SilviaSystemConfiguration,
}

impl Default for SilviaController {
    fn default() -> Self {
        SilviaController::new(
            PidCtrl::new_with_pid(5.0, 5.0, 5.0),
            SilviaSystemConfiguration::default(),
        )
    }
}

impl SilviaController {
    pub fn new(brew_boiler_pid: PidCtrl<f32>, mut system_configuration: SilviaSystemConfiguration) -> Self {
        system_configuration.boiler_temp_setpoint_c = 95.0;

        Self {
            prev_update: Instant::now(),
            brew_boiler_pid,
            system_configuration
        }
    }
}

impl Controller<SilviaSystemSensorState, SilviaSystemActuatorState> for SilviaController {
    async fn update_actuator_state_from_sensor_state(&mut self, system_sensor_state: &SilviaSystemSensorState, system_actuator_state: &mut SilviaSystemActuatorState) -> Result<(), ControllerError> {
        let now = Instant::now();

        let delta = now - self.prev_update;
        let delta_millis = delta.as_micros().to_f32().expect("For some weird reason, we couldn't convert a u64 to a f32") / 1000f32;

        self.brew_boiler_pid.setpoint = self.system_configuration.boiler_temp_setpoint_c;
        let brew_out = self.brew_boiler_pid.step(PidIn::new(system_sensor_state.boiler_temp_c, delta_millis));
        log::info!("PID: {:?}", self.brew_boiler_pid);
        log::info!("Brew PID output: {:?}", brew_out);

        system_actuator_state.brew_boiler_heating_element.set_duty_cycle((brew_out.out * 100f32) as u8);

        system_actuator_state.brew_boiler_heating_element.update(now);

        self.prev_update = now;

        Ok(())
    }
}
