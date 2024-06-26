mod silvia_shift_register_actuator_cluster;

use variegated_board_apec::ApecR0DBoardFeatures;
use variegated_controller_lib::{ActualMutexType, ActuatorCluster, ActuatorClusterError};
use crate::actuator_cluster::silvia_shift_register_actuator_cluster::SilviaShiftRegisterOutputCluster;

pub async fn create_actuator_clusters(board_features_mutex: &ActualMutexType<ApecR0DBoardFeatures>) -> (SilviaShiftRegisterOutputCluster) {
    let mut board_features = board_features_mutex.lock().await;

    let shift_register = board_features.dual_shift_register.take().expect("Board features needs to have a dual shift register");

    let shift_register_output_cluster = SilviaShiftRegisterOutputCluster::new(
        shift_register
    );
    
    (shift_register_output_cluster)
}