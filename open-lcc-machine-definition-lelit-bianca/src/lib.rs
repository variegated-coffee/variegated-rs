#![no_std]

extern crate alloc;

use alloc::boxed::Box;
use open_lcc_controller_lib::*;

#[derive(Default, Copy, Clone)]
pub struct BiancaSystemSensorState {
    brew_switch: bool,
    water_tank_empty: bool,
    service_boiler_water_level_adc: u16,
    brew_boiler_temp_c: f32,
    service_boiler_temp_c: f32,
}

impl SystemSensorState for BiancaSystemSensorState {}

#[derive(Default, Copy, Clone)]
pub struct BiancaSystemActuatorState {
    status_led: bool,
    brew_boiler_heating_element: bool,
    service_boiler_diversion_solenoid: bool,
    line_solenoid: bool,
    service_boiler_heating_element: bool,
    pump: bool,
}

impl SystemActuatorState for BiancaSystemActuatorState {}

pub struct BiancaSystemGeneralState {
    is_brewing: bool,
}

pub struct BiancaSystemConfiguration {
    brew_boiler_temp_setpoint_c: f32,
    service_boiler_temp_setpoint_c: f32,
}

struct DummySensorCluster {
}

impl SensorCluster<BiancaSystemSensorState> for DummySensorCluster {
    fn update_sensor_state(&mut self, previous_state: BiancaSystemSensorState) -> BiancaSystemSensorState {
        let mut state = previous_state.clone();
        state.brew_boiler_temp_c += 5.0;

        state
    }
}

struct DummyController {}

impl Controller<BiancaSystemSensorState, BiancaSystemActuatorState> for DummyController {
    fn update_actuator_state_from_sensor_state(&self, system_sensor_state: &BiancaSystemSensorState, system_actuator_state: BiancaSystemActuatorState) -> BiancaSystemActuatorState {
        BiancaSystemActuatorState {
            status_led: true,
            brew_boiler_heating_element: if system_sensor_state.brew_boiler_temp_c < 90.0 { true } else { false },
            service_boiler_diversion_solenoid: false,
            line_solenoid: false,
            service_boiler_heating_element: false,
            pump: false,
        }
    }
}


pub fn create() -> (Context<BiancaSystemSensorState, BiancaSystemActuatorState, 1, 0, 1>, BiancaSystemSensorState, BiancaSystemActuatorState) {
    (
        Context {
            sensors: [
                Box::new(DummySensorCluster {})
            ],
            actuators: [],
            controllers: [
                Box::new(DummyController {}),
            ]
        }, 
        BiancaSystemSensorState::default(), 
        BiancaSystemActuatorState::default()
    )
}