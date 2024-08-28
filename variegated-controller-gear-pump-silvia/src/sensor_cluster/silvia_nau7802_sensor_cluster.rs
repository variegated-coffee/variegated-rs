use embedded_hal_async::i2c::I2c;
use variegated_controller_lib::{ActualBoardFeaturesMutex, SensorCluster, SensorClusterError};
use crate::state::SilviaSystemSensorState;
use variegated_embassy_nau7802::Nau7802;

pub struct SilviaNau7802SensorCluster {
    nau7802: Nau7802,
}

#[derive(Debug)]
pub enum SilviaNau7802SensorClusterError {
    InitializationError,
    ReadError,
}

impl SilviaNau7802SensorCluster {
    pub async fn new<I2cT: I2c>(i2c: &mut I2cT) -> Result<SilviaNau7802SensorCluster, SilviaNau7802SensorClusterError> {
        let nau7802 = Nau7802::new();
        nau7802.init(
            i2c,
            variegated_embassy_nau7802::Ldo::L3v3,
            variegated_embassy_nau7802::Gain::G128,
            variegated_embassy_nau7802::SamplesPerSecond::SPS10,
        ).await.map_err(|e| SilviaNau7802SensorClusterError::InitializationError)?;

        Ok(SilviaNau7802SensorCluster {
            nau7802,
        })
    }
}

impl SensorCluster<SilviaSystemSensorState> for SilviaNau7802SensorCluster{
    async fn update_sensor_state(&mut self, state: &mut SilviaSystemSensorState, board_features: &ActualBoardFeaturesMutex) -> Result<(), SensorClusterError> {
        let mut features = board_features.lock().await;
        let i2c = features.i2c_bus.as_mut().expect("I2c not initialized");
       
        let scale_adc_val = self.nau7802.read(i2c).await.map_err(|_| SensorClusterError::UnknownError)?;

        let zero = -41942;
        let hundered = 7557;

        let slope = 100.0 / (hundered - zero) as f32;
        let weight = slope * (scale_adc_val as f32 - zero as f32);

        state.scale_untared_g = weight;

        Ok(())
    }
}
