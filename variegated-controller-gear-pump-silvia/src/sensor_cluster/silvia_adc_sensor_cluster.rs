extern crate alloc;

use alloc::boxed::Box;
use async_trait::async_trait;
use variegated_controller_lib::{ActualBoardFeaturesMutex, ActualMutexType, ApecBoardFeatures, SensorCluster, SensorClusterError};
use variegated_embassy_ads124s08::ADS124S08;
use variegated_embassy_ads124s08::registers::{IDACMagnitude, IDACMux, Mux, PGAGain, ReferenceInput};
use variegated_log::log_info;
use crate::SilviaSystemSensorState;

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
        SilviaAdcSensorCluster {
            adc,
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
}

impl<'a> SensorCluster<SilviaSystemSensorState> for SilviaAdcSensorCluster<'a> {
    async fn update_sensor_state(&mut self, sensor_state: &mut SilviaSystemSensorState, board_features: &ActualMutexType<ApecBoardFeatures>) -> Result<(), SensorClusterError> {
        let mut features = board_features.lock().await;
        let spi = features.spi_bus.as_mut().expect("SPI not initialized");

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

        log_info!("Boiler code: {:?}", boiler_code.ratiometric_resistance(3300.0));

        let referenced_voltage = match self.boiler_temperature_reference_input {
            ReferenceInput::Internal => boiler_code.internally_referenced_voltage(),
            _ => match self.boiler_temperature_reference_resistor {
                Some(resistor) => boiler_code.ratiometric_resistance(resistor),
                None => match self.boiler_temperature_reference_voltage {
                    Some((v_ref_n, v_ref_p)) => boiler_code.externally_referenced_voltage(v_ref_n, v_ref_p),
                    None => return Err(SensorClusterError::UnknownError),
                }
            }
        };

        let boiler_temp_c = (referenced_voltage + self.boiler_temperature_conversion_parameters.m) * self.boiler_temperature_conversion_parameters.k;

        /*
        let dvdd = self.adc.read_dvdd_by_4(spi).await;

        let boiler_temp_c = match dvdd {
            Ok(dvdd) => dvdd,
            Err(_) => return Err(SensorClusterError::UnknownError),
        };
*/

        let avcc = self.adc.read_avdd_by_4(spi).await;

        let boiler_pressure_bar = match avcc {
            Ok(avcc) => avcc,
            Err(_) => return Err(SensorClusterError::UnknownError),
        };

        sensor_state.boiler_temp_c = boiler_temp_c;
        sensor_state.boiler_pressure_bar = boiler_pressure_bar.internally_referenced_voltage();
        
        Ok(())
    }
}

