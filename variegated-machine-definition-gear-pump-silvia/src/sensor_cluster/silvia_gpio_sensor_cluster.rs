extern crate alloc;

use alloc::boxed::Box;
use async_trait::async_trait;
use embassy_rp::gpio::{AnyPin, Input};
use variegated_controller_lib::{ActualBoardFeaturesMutex, SensorCluster, SensorClusterError};
use crate::SilviaSystemSensorState;

pub struct SilviaGpioSensorCluster<'a> {
    brew_button_input: Input<'a, AnyPin>,
    water_button_input: Input<'a, AnyPin>,
    steam_button_input: Input<'a, AnyPin>,
}

impl<'a> SilviaGpioSensorCluster<'a> {
    pub fn new(brew_button_input: Input<'a, AnyPin>, water_button_input: Input<'a, AnyPin>, steam_button_input: Input<'a, AnyPin>) -> SilviaGpioSensorCluster<'a> {
        SilviaGpioSensorCluster {
            brew_button_input,
            water_button_input,
            steam_button_input,
        }
    }
}

#[async_trait]
impl<'a> SensorCluster<SilviaSystemSensorState> for SilviaGpioSensorCluster<'a> {
    async fn update_sensor_state(&mut self, state: &mut SilviaSystemSensorState, _board_features: &ActualBoardFeaturesMutex) -> Result<(), SensorClusterError> {
        state.brew_switch = self.brew_button_input.is_low();
        state.water_switch = self.water_button_input.is_low();
        state.steam_switch = self.steam_button_input.is_low();

        Ok(())
    }
}

