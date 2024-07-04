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

pub enum SensorClusterError {
    UnknownError
}

pub trait SensorCluster<SensorStateT: SystemSensorState> {
    async fn update_sensor_state(&mut self, previous_state: &mut SensorStateT, board_features: &ActualBoardFeaturesMutex) -> Result<(), SensorClusterError>;
}

pub enum ActuatorClusterError {
    UnknownError
}

pub trait ActuatorCluster<ActuatorStateT: SystemActuatorState> {
    async fn update_from_actuator_state(&mut self, system_actuator_state: &ActuatorStateT) -> Result<(), ActuatorClusterError>;
}

pub enum ControllerError {
    UnknownError
}

pub trait Controller<SensorStateT: SystemSensorState, ActuatorStateT: SystemActuatorState> {
    async fn update_actuator_state_from_sensor_state(&mut self, system_sensor_state: &SensorStateT, system_actuator_state: &mut ActuatorStateT) -> Result<(), ControllerError>;
}

pub type BoilerId = u8;

pub type GroupId = u8;

pub enum RunMode {
    Heatup,
    Normal,
    Standby,
}

pub enum Command {
    SetBoilerSetPoint(BoilerId, f32),
    SetGroupPressureTarget(GroupId, f32),
    DisableBoiler(BoilerId),
    EnableBoiler(BoilerId),
    RequestRunMode(RunMode)
}