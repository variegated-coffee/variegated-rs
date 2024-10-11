extern crate alloc;

use alloc::boxed::Box;
use async_trait::async_trait;
use embassy_rp::spi::{Config, Phase, Polarity};
use variegated_controller_lib::{ActualBoardFeaturesMutex, ActualMutexType, ApecBoardFeatures, SensorCluster, SensorClusterError};
use variegated_embassy_ads124s08::ADS124S08;
use variegated_embassy_ads124s08::registers::{ClockSource, DataRate, DataRateRegister, IDACMagnitude, IDACMux, Mux, PGAGain, ReferenceInput};
use variegated_log::log_info;
use crate::SilviaSystemSensorState;
use embassy_embedded_hal::SetConfig;

pub struct DifferentialAdcMeasurementConfiguration {
    pub sensor_p_mux: Mux,
    pub sensor_n_mux: Mux,
    pub sensor_idac1: IDACMux,
    pub sensor_idac2: IDACMux,
    pub reference_input: ReferenceInput,
    pub mag: IDACMagnitude,
    pub gain: PGAGain,
    pub reference_resistor: Option<f32>,
    pub reference_voltage: Option<(f32, f32)>
}

pub struct SingleEndedAdcMeasurementConfiguration {
    pub sensor_mux: Mux,
    pub reference_input: ReferenceInput,
    pub reference_voltage: Option<(f32, f32)>
}

pub struct SilviaBoilerTemperatureConfiguration {
    pub differential_configuration: Option<DifferentialAdcMeasurementConfiguration>,
    pub single_ended_configration: Option<SingleEndedAdcMeasurementConfiguration>,
    pub celsius_per_volt: f32,
}

pub struct SilviaPressureTransducerConfiguration {
    pub differential_configuration: Option<DifferentialAdcMeasurementConfiguration>,
    pub single_ended_configuration: Option<SingleEndedAdcMeasurementConfiguration>,
    pub bar_per_volt: f32,
}

pub struct SilviaAdcSensorCluster<'a> {
    adc: ADS124S08<'a>,
    spi_config: Config,
    boiler_temperature_sensor_p_mux: Mux,
    boiler_temperature_sensor_n_mux: Mux,
    boiler_temperature_sensor_idac1: IDACMux,
    boiler_temperature_sensor_idac2: IDACMux,
    boiler_temperature_reference_input: ReferenceInput,
    boiler_temperature_mag: IDACMagnitude,
    boiler_temperature_gain: PGAGain,
    boiler_temperature_conversion_parameters: SilviaLinearVoltageToCelsiusConversionParameters,
    boiler_temperature_reference_resistor: Option<f32>,
    boiler_temperature_reference_voltage: Option<(f32, f32)>
}

pub struct SilviaLinearVoltageToCelsiusConversionParameters {
    pub k: f32,
    pub m: f32,
}

impl<'a> SilviaAdcSensorCluster<'a> {
    pub fn new(
        adc: ADS124S08<'a>,
        boiler_temperature_sensor_p_mux: Mux,
        boiler_temperature_sensor_n_mux: Mux,
        boiler_temperature_sensor_idac1: IDACMux,
        boiler_temperature_sensor_idac2: IDACMux,
        boiler_temperature_reference_input: ReferenceInput,
        boiler_temperature_mag: IDACMagnitude,
        boiler_temperature_gain: PGAGain,
        boiler_temperature_conversion_parameters: SilviaLinearVoltageToCelsiusConversionParameters,
        boiler_temperature_reference_resistor: Option<f32>,
        boiler_temperature_reference_voltage: Option<(f32, f32)>
    ) -> SilviaAdcSensorCluster<'a> {
        let mut config = Config::default();
        config.frequency = 1_000_000; // According to docs, max frequency is 10 MHz, we're doing 1 though
        config.phase = Phase::CaptureOnSecondTransition;
        config.polarity = Polarity::IdleLow;

        SilviaAdcSensorCluster {
            adc,
            spi_config: config,
            boiler_temperature_sensor_p_mux,
            boiler_temperature_sensor_n_mux,
            boiler_temperature_sensor_idac1,
            boiler_temperature_sensor_idac2,
            boiler_temperature_reference_input,
            boiler_temperature_mag,
            boiler_temperature_gain,
            boiler_temperature_conversion_parameters,
            boiler_temperature_reference_resistor,
            boiler_temperature_reference_voltage,
        }
    }

    pub(crate) async fn init_adc(&mut self, board_features: &ActualMutexType<ApecBoardFeatures>) -> Result<(), SensorClusterError> {
        let mut features = board_features.lock().await;
        let spi = features.spi_bus.as_mut().expect("SPI not initialized");
        // spi.set_config(&self.spi_config).expect("SPI config failed"); @fixme This needs to change

        self.adc.begin_transaction().await;
        self.adc.reset(spi).await.map_err(|_| SensorClusterError::UnknownError)?;
        let mut datarate = self.adc.read_datarate_reg(spi).await.map_err(|_| SensorClusterError::UnknownError)?;
        datarate.rate = DataRate::SPS100;
        self.adc.write_datarate_reg(spi, datarate).await.map_err(|_| SensorClusterError::UnknownError)?;
        self.adc.end_transaction().await;

        Ok(())
    }

    pub(crate) async fn update_sensor_state(&mut self, sensor_state: &mut SilviaSystemSensorState, board_features: &ActualMutexType<ApecBoardFeatures>) -> Result<(), SensorClusterError> {
        let mut features = board_features.lock().await;
        let spi = features.spi_bus.as_mut().expect("SPI not initialized");
        // spi.set_config(&self.spi_config).expect("SPI config failed"); @fixme This needs to change


        let boiler_code = self.adc.measure_ratiometric_low_side(
            spi,
            self.boiler_temperature_sensor_p_mux,
            self.boiler_temperature_sensor_n_mux,
            self.boiler_temperature_sensor_idac1,
            self.boiler_temperature_sensor_idac2,
            self.boiler_temperature_reference_input,
            self.boiler_temperature_mag,
            self.boiler_temperature_gain
        ).await;

        let boiler_code = match boiler_code {
            Ok(code) => code,
            Err(_) => return Err(SensorClusterError::UnknownError),
        };

        //log_info!("Boiler code: {:?}", boiler_code.ratiometric_resistance(3300.0));

        let referenced_code = match self.boiler_temperature_reference_input {
            ReferenceInput::Internal => boiler_code.internally_referenced_voltage(),
            _ => match self.boiler_temperature_reference_resistor {
                Some(resistor) => boiler_code.ratiometric_resistance(resistor),
                None => match self.boiler_temperature_reference_voltage {
                    Some((v_ref_n, v_ref_p)) => boiler_code.externally_referenced_voltage(v_ref_n, v_ref_p),
                    None => return Err(SensorClusterError::UnknownError),
                }
            }
        };

        //log_info!("Boiler temp V: {:?}", referenced_code);
/*        let bto = -1f32 * (75000f32 * referenced_code) / (referenced_code - 5f32);
        log_info!("Boiler temp R: {:?}", bto);*/

        let boiler_temp_c = (referenced_code + self.boiler_temperature_conversion_parameters.m) * self.boiler_temperature_conversion_parameters.k;


/*        let dvdd = self.adc.read_dvdd_by_4(spi).await;

        let dvdd = match dvdd {
            Ok(dvdd) => dvdd,
            Err(_) => return Err(SensorClusterError::UnknownError),
        };
*/
        let pressure_code = self.adc.measure_single_ended(spi, Mux::AIN4, ReferenceInput::Refp1Refn1).await;

        let boiler_pressure_v = match pressure_code {
            Ok(avcc) => avcc.externally_referenced_voltage(0.0, 5.00),
            Err(_) => return Err(SensorClusterError::UnknownError),
        };
        
        let boiler_pressure_bar = (boiler_pressure_v - 0.5) * 4.0;

        sensor_state.boiler_temp_c = boiler_temp_c;
        sensor_state.boiler_pressure_bar = boiler_pressure_bar;
        
        Ok(())
    }
}

