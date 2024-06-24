#![no_std]

extern crate alloc;

use alloc::boxed::Box;
use core::cell::RefCell;
use async_trait::async_trait;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;

pub trait SystemSensorState: Clone + Sync + Send {

}

pub trait SystemActuatorState: Clone + Sync + Send {

}


cfg_if::cfg_if! {
    if #[cfg(feature = "open-lcc-r2a")] {
        pub type ActualBoardFeatures = variegated_board_open_lcc_r2a::R2ABoardFeatures;
        pub type OpenLccBoardFeatures = variegated_board_open_lcc_r2a::R2ABoardFeatures;
    } else if #[cfg(feature = "apec-r0d")] {
        pub type ActualBoardFeatures = variegated_board_apec::ApecR0DBoardFeatures;
        pub type ApecBoardFeatures = variegated_board_apec::ApecR0DBoardFeatures;
    } else {
        compile_error!("No board feature set selected");
    }
}

pub type ActualMutexType<T> = Mutex<ThreadModeRawMutex, T>;
pub type ActualBoardFeaturesMutex = ActualMutexType<ActualBoardFeatures>;

pub struct Context<SensorStateT: SystemSensorState, ActuatorStateT: SystemActuatorState, const N_SENSORS: usize, const N_ACTUATORS: usize, const N_CONTROLLERS: usize> {
    pub sensors: [Box<dyn SensorCluster<SensorStateT>>; N_SENSORS],
    pub actuators: [Box<dyn ActuatorCluster<ActuatorStateT>>; N_ACTUATORS],
    pub controllers: [Box<dyn Controller<SensorStateT, ActuatorStateT>>; N_CONTROLLERS]
}

pub enum SensorClusterError {
    UnknownError
}

#[async_trait]
pub trait SensorCluster<SensorStateT: SystemSensorState> {
    async fn update_sensor_state(&mut self, previous_state: SensorStateT, board_features: &ActualBoardFeaturesMutex) -> Result<SensorStateT, SensorClusterError>;
}

pub enum ActuatorClusterError {
    UnknownError
}

#[async_trait]
pub trait ActuatorCluster<ActuatorStateT: SystemActuatorState> {
    async fn update_from_actuator_state(&mut self, system_actuator_state: &ActuatorStateT) -> Result<(), ActuatorClusterError>;
}

pub trait Controller<SensorStateT: SystemSensorState, ActuatorStateT: SystemActuatorState> {
    fn update_actuator_state_from_sensor_state(&mut self, system_sensor_state: &SensorStateT, system_actuator_state: ActuatorStateT) -> ActuatorStateT;
}
