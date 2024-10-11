use embassy_rp::gpio::{Input, Pin, Pull};
use embassy_rp::pio::PioPin;
use variegated_controller_lib::{ActualBoardFeatures, ActualBoardFeaturesMutex, ActualMutexType};
use variegated_embassy_ads124s08::registers::{IDACMagnitude, IDACMux, Mux, PGAGain, ReferenceInput};
use variegated_log::log_info;
use crate::sensor_cluster::silvia_adc_sensor_cluster::{SilviaAdcSensorCluster, SilviaLinearVoltageToCelsiusConversionParameters};
use crate::sensor_cluster::silvia_gpio_sensor_cluster::SilviaGpioSensorCluster;
use crate::sensor_cluster::silvia_nau7802_sensor_cluster::SilviaNau7802SensorCluster;
use crate::sensor_cluster::silvia_pio_frequency_counter::SilviaPioFrequencyCounter;

pub(crate) mod silvia_gpio_sensor_cluster;
pub(crate) mod silvia_adc_sensor_cluster;
pub(crate) mod silvia_pio_frequency_counter;
pub(crate) mod silvia_nau7802_sensor_cluster;

pub async fn create_sensor_clusters(board_features_mutex: &ActualMutexType<ActualBoardFeatures>) -> (SilviaGpioSensorCluster, SilviaAdcSensorCluster, SilviaPioFrequencyCounter, SilviaNau7802SensorCluster) {
    let mut board_features = board_features_mutex.lock().await;
    
    let brew_button_input = Input::new(board_features.cn12_pin.take().expect("Board features needs to have CN14_8").degrade(), Pull::Up);
    let steam_button_input = Input::new(board_features.cn14_6_pin.take().expect("Board features needs to have CN14_6"), Pull::Up);
    let water_button_input = Input::new(board_features.cn13_pin.take().expect("Board features needs to have CN14_4").degrade(), Pull::Up);

    let gpio_sensor_cluster = silvia_gpio_sensor_cluster::SilviaGpioSensorCluster::new(
        brew_button_input,
        water_button_input,
        steam_button_input,
    );

    log_info!("Quux");
    
    let adc = board_features.ads124s08.take().expect("Board features needs to have an ADS124S08");

/*    
        // NTC Thermistor
        
        let adc_sensor_cluster = SilviaAdcSensorCluster::new(
        adc,
        Mux::AIN2,
        Mux::AIN1,
        IDACMux::Disconnected,
        IDACMux::Disconnected,
        ReferenceInput::Internal,
        IDACMagnitude::Mag1000uA,
        PGAGain::Gain1,
        SilviaLinearVoltageToCelsiusConversionParameters { k: 2.597065316192702, m: -103.51 },
        None,
        None
    );
*/
    
    log_info!("Xyzzy");

    let pump_rpm_pin = board_features.cn9_6_pin.take().expect("Board features needs to have CN9_6");
    let flow_meter_pin = board_features.cn14_3_pin.take().expect("Board features needs to have CN14_3");

    let pump_frequency = SilviaPioFrequencyCounter::new(
        board_features.pio0.take().expect("Board features needs to have PIO0"),
        pump_rpm_pin,
        flow_meter_pin
    );
    log_info!("Xyzzy23");
    
    let i2c = board_features.i2c_bus.as_mut().expect("I2c not initialized");
    let nau7802 = SilviaNau7802SensorCluster::new().await.expect("Failed to initialize NAU7802");
    //nau7802.init(i2c).await.expect("Failed to initialize NAU7802");
    log_info!("Xyzzy334");

    // PT100
    let adc_sensor_cluster = SilviaAdcSensorCluster::new(
        adc,
        Mux::AIN1,
        Mux::AIN2,
        IDACMux::AIN0,
        IDACMux::Disconnected,
        ReferenceInput::Refp0Refn0,
        IDACMagnitude::Mag1000uA,
        PGAGain::Gain1,
        SilviaLinearVoltageToCelsiusConversionParameters { k: 2.597065316192702, m: -103.51 },
        Some(499.0),
        None
    );


    (gpio_sensor_cluster, adc_sensor_cluster, pump_frequency, nau7802)
}