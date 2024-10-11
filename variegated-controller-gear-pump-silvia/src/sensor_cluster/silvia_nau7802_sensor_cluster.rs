use embassy_rp::gpio::{AnyPin, Input};
use embassy_time::Delay;
use embedded_hal_async::i2c::I2c;
use num_traits::ToPrimitive;
use variegated_controller_lib::{ActualBoardFeaturesMutex, SensorCluster, SensorClusterError};
use crate::state::SilviaSystemSensorState;
use variegated_embassy_nau7802::{Nau7802, Nau7802DataAvailableStrategy};

pub struct SilviaNau7802SensorCluster<'a> {
    nau7802: Nau7802<Input<'a>, Delay>,
}

#[derive(Debug)]
pub enum SilviaNau7802SensorClusterError {
    InitializationError,
    ReadError,
}

impl<'a> SilviaNau7802SensorCluster<'a> {
    pub async fn new() -> Result<SilviaNau7802SensorCluster<'a>, SilviaNau7802SensorClusterError> {
        let nau7802 = Nau7802::new(Nau7802DataAvailableStrategy::Polling, Delay {});

        Ok(SilviaNau7802SensorCluster {
            nau7802,
        })
    }
    
    pub async fn init<I2cT: I2c>(&self, i2c: &mut I2cT) -> Result<(), SilviaNau7802SensorClusterError> {
        self.nau7802.init(
            i2c,
            variegated_embassy_nau7802::Ldo::L3v3,
            variegated_embassy_nau7802::Gain::G128,
            variegated_embassy_nau7802::SamplesPerSecond::SPS10,
        ).await.map_err(|e| SilviaNau7802SensorClusterError::InitializationError)
    }
}

impl SensorCluster<SilviaSystemSensorState> for SilviaNau7802SensorCluster<'_>{
    async fn update_sensor_state(&mut self, state: &mut SilviaSystemSensorState, board_features: &ActualBoardFeaturesMutex) -> Result<(), SensorClusterError> {
        let mut features = board_features.lock().await;
        let i2c = features.i2c_bus.as_mut().expect("I2c not initialized");
       
        let scale_adc_val = self.nau7802.read(i2c).await.map_err(|_| SensorClusterError::UnknownError)?;

        let zero = -279000.0;
        let hundered = -327900.0;

        let slope = 94.7 / (hundered - zero) as f32;
        let weight = slope * (scale_adc_val as f32 - zero as f32);

        state.scale_untared_g = weight;
//        state.scale_untared_g = scale_adc_val as f32;

        Ok(())
    }
}
