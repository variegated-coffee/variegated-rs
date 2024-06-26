extern crate alloc;

use alloc::boxed::Box;
use embassy_rp::uart::{Instance, UartTx};
use embassy_rp::uart;
use async_trait::async_trait;
use variegated_controller_lib::{ActuatorCluster, ActuatorClusterError};
use variegated_gicar_8_5_04::ActuatorMessage;
use crate::BiancaSystemActuatorState;

pub struct BiancaGicarActuatorCluster<'a, UartT: Instance> {
    gicar_cluster: variegated_gicar_8_5_04::Gicar8504ActuatorCluster<'a, BiancaSystemActuatorState, UartT>
}

impl<'a, UartT: Instance> BiancaGicarActuatorCluster<'a, UartT> {
    pub fn new(uart_tx: UartTx<'a, UartT, uart::Async>) -> BiancaGicarActuatorCluster<'a, UartT> {
        BiancaGicarActuatorCluster {
            gicar_cluster: variegated_gicar_8_5_04::Gicar8504ActuatorCluster::new(
                |state| {
                    ActuatorMessage {
                        cn5: state.service_boiler_heating_element.get_output(),
                        fa10: state.pump,
                        fa9: false,
                        cn6_1: false,
                        cn6_3: false,
                        cn6_5: true,
                        cn9: state.brew_boiler_heating_element.get_output(),
                        cn7: false,
                        fa7: state.service_boiler_diversion_solenoid,
                        fa8: state.line_solenoid,
                        cn10_n_12v: false,
                        cn10_n_3v3: false,
                        minus_button: false,
                        plus_button: false,
                    }
                }, uart_tx
            )
        }
    }
}

impl<'a, UartT: Instance + Send + Sync> ActuatorCluster<BiancaSystemActuatorState> for BiancaGicarActuatorCluster<'a, UartT> {
    async fn update_from_actuator_state(&mut self, system_actuator_state: &BiancaSystemActuatorState) -> Result<(), ActuatorClusterError>{
        self.gicar_cluster.update_from_actuator_state(system_actuator_state).await
    }
}
