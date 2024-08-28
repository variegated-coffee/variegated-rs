mod silvia_shift_register_actuator_cluster;
mod silvia_pwm_actuator_cluster;

use variegated_board_apec::ApecR0DBoardFeatures;
use variegated_controller_lib::{ActualMutexType, ActuatorCluster, ActuatorClusterError};
use crate::actuator_cluster::silvia_pwm_actuator_cluster::SilviaPwmOutputCluster;
use crate::actuator_cluster::silvia_shift_register_actuator_cluster::SilviaShiftRegisterOutputCluster;

pub async fn create_actuator_clusters(board_features_mutex: &ActualMutexType<ApecR0DBoardFeatures>) -> (SilviaShiftRegisterOutputCluster, SilviaPwmOutputCluster) {
    let mut board_features = board_features_mutex.lock().await;

    let shift_register = board_features.dual_shift_register.take().expect("Board features needs to have a dual shift register");

    let shift_register_output_cluster = SilviaShiftRegisterOutputCluster::new(
        shift_register
    );
    
    let silvia_pwm_actuator_cluster = SilviaPwmOutputCluster::new(board_features.cn9_4_pwm.take().expect("Must have CN9_4 PWM"));
    
    (shift_register_output_cluster, silvia_pwm_actuator_cluster)
}