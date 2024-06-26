use embassy_rp::gpio::{Input, Pull};
use variegated_board_apec::ApecR0DBoardFeatures;
use variegated_controller_lib::{ActualBoardFeaturesMutex, ActualMutexType};
use variegated_embassy_ads124s08::registers::{IDACMagnitude, IDACMux, Mux, PGAGain, ReferenceInput};
use crate::sensor_cluster::silvia_adc_sensor_cluster::{SilviaAdcSensorCluster, SilviaLinearVoltageToCelsiusConversionParameters};
use crate::sensor_cluster::silvia_gpio_sensor_cluster::SilviaGpioSensorCluster;

pub(crate) mod silvia_gpio_sensor_cluster;
pub(crate) mod silvia_adc_sensor_cluster;

pub async fn create_sensor_clusters(board_features_mutex: &ActualMutexType<ApecR0DBoardFeatures>) -> (SilviaGpioSensorCluster, SilviaAdcSensorCluster) {
    let mut board_features = board_features_mutex.lock().await;
    
    let brew_button_input = Input::new(board_features.cn14_8_pin.take().expect("Board features needs to have CN14_8"), Pull::Up);
    let steam_button_input = Input::new(board_features.cn14_6_pin.take().expect("Board features needs to have CN14_6"), Pull::Up);
    let water_button_input = Input::new(board_features.cn14_4_pin.take().expect("Board features needs to have CN14_4"), Pull::Up);

    let gpio_sensor_cluster = silvia_gpio_sensor_cluster::SilviaGpioSensorCluster::new(
        brew_button_input,
        steam_button_input,
        water_button_input,
    );

    let adc = board_features.ads124s08.take().expect("Board features needs to have an ADS124S08");

    let adc_sensor_cluster = SilviaAdcSensorCluster::new(
        adc,
        Mux::AIN1,
        Mux::AIN2,
        IDACMux::AIN0,
        IDACMux::Disconnected,
        ReferenceInput::Refp0Refn0,
        IDACMagnitude::Mag500uA,
        PGAGain::Gain1,
        SilviaLinearVoltageToCelsiusConversionParameters { k: 2.597065316192702, m: -103.51 },
        Some(3300.0),
        None
    );

    (gpio_sensor_cluster, adc_sensor_cluster)
}