extern crate alloc;

use alloc::boxed::Box;
use core::cell::RefCell;
use async_trait::async_trait;
use embassy_rp::uart;
use embassy_rp::uart::{Instance, UartRx};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use variegated_controller_lib::{ActualBoardFeatures, ActualBoardFeaturesMutex, SensorCluster, SensorClusterError};
use variegated_gicar_8_5_04::{high_gain_adc_to_ohm, ntc_ohm_to_celsius};
use crate::BiancaSystemSensorState;

pub struct BiancaGicarSensorCluster<'a, UartT: Instance> {
    gicar_cluster: variegated_gicar_8_5_04::Gicar8504SensorCluster<'a, BiancaSystemSensorState, UartT>
}

impl<'a, UartT: Instance> BiancaGicarSensorCluster<'a, UartT> {
    pub fn new(uart_rx: UartRx<'a, UartT, uart::Async>) -> BiancaGicarSensorCluster<'a, UartT> {
        BiancaGicarSensorCluster {
            gicar_cluster: variegated_gicar_8_5_04::Gicar8504SensorCluster::new(
                |msg, state: &mut BiancaSystemSensorState| {
                    state.brew_switch = msg.cn7;
                    state.water_tank_empty = msg.cn2;

                    state.brew_boiler_temp_c = ntc_ohm_to_celsius(high_gain_adc_to_ohm(msg.cn3_adc_high_gain.to_u16()), 50000, 4016);
                }, uart_rx
            )
        }
    }
}

impl<'a, UartT: Instance + Send + Sync> SensorCluster<BiancaSystemSensorState> for BiancaGicarSensorCluster<'a, UartT> {
    async fn update_sensor_state(&mut self, previous_state: &mut BiancaSystemSensorState, board_features: &ActualBoardFeaturesMutex) -> Result<(), SensorClusterError> {
        self.gicar_cluster.update_sensor_state(previous_state, board_features).await
    }
}
