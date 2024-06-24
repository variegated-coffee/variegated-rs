extern crate alloc;

use alloc::boxed::Box;
use async_trait::async_trait;
use variegated_controller_lib::{ActualBoardFeaturesMutex, ActualMutexType, ApecBoardFeatures, SensorCluster, SensorClusterError};
use variegated_embassy_ads124s08::ADS124S08;
use variegated_embassy_ads124s08::registers::{IDACMagnitude, IDACMux, Mux, PGAGain, ReferenceInput};
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
    boiler_temperature_celsius_per_volt: f32,
    boiler_temperature_reference_resistor: Option<f32>,
    boiler_temperature_reference_voltage: Option<(f32, f32)>
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
        boiler_temperature_celsius_per_volt: f32,
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
            boiler_temperature_celsius_per_volt,
            boiler_temperature_reference_resistor,
            boiler_temperature_reference_voltage,
        }
    }
}

#[async_trait]
impl<'a> SensorCluster<SilviaSystemSensorState> for SilviaAdcSensorCluster<'a> {
    async fn update_sensor_state(&mut self, previous_state: SilviaSystemSensorState, board_features: &ActualMutexType<ApecBoardFeatures>) -> Result<SilviaSystemSensorState, SensorClusterError> {
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

        let boiler_temp_c = referenced_voltage * self.boiler_temperature_celsius_per_volt;

        Ok(SilviaSystemSensorState {
            boiler_temp_c,
            ..previous_state
        })
    }
}

