#![no_std]

extern crate alloc;

use alloc::boxed::Box;
use async_trait::async_trait;

pub trait SystemSensorState: Clone + Sync + Send {

}

pub trait SystemActuatorState: Clone + Sync + Send {

}

pub struct Context<SensorStateT: SystemSensorState, ActuatorStateT: SystemActuatorState, const N_SENSORS: usize, const N_ACTUATORS: usize, const N_CONTROLLERS: usize> {
    pub sensors: [Box<dyn SensorCluster<SensorStateT>>; N_SENSORS],
    pub actuators: [Box<dyn ActuatorCluster<ActuatorStateT>>; N_ACTUATORS],
    pub controllers: [Box<dyn Controller<SensorStateT, ActuatorStateT>>; N_CONTROLLERS]
}

#[async_trait]
pub trait SensorCluster<SensorStateT: SystemSensorState>{
    async fn update_sensor_state(&mut self, previous_state: SensorStateT) -> SensorStateT;
}

#[async_trait]
pub trait ActuatorCluster<ActuatorStateT: SystemActuatorState> {
    async fn update_from_actuator_state(&mut self, system_actuator_state: &ActuatorStateT);
}

pub trait Controller<SensorStateT: SystemSensorState, ActuatorStateT: SystemActuatorState> {
    fn update_actuator_state_from_sensor_state(&mut self, system_sensor_state: &SensorStateT, system_actuator_state: ActuatorStateT) -> ActuatorStateT;
}
