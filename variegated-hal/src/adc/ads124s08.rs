use embassy_sync::blocking_mutex::raw::{NoopRawMutex, RawMutex};
use embassy_sync::mutex::Mutex;
use embassy_sync::watch::Sender;
use embassy_time::Timer;
use embedded_hal::digital::InputPin;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::spi::SpiDevice;
use embedded_hal_async::digital::Wait;
use variegated_adc_tools::ConversionParameters;
use variegated_ads124s08::ADS124S08;
use variegated_ads124s08::registers::{IDACMagnitude, IDACMux, PGAGain, ReferenceInput};
use variegated_ads124s08::registers::Mux;

use crate::WithTask;

pub enum MeasurementType {
    SingleEnded(Mux, ReferenceInput, f32),
    RatiometricLowSide(Mux, Mux, IDACMux, IDACMux, ReferenceInput, IDACMagnitude, PGAGain, f32),
    AvddBy4,
    DvddBy4,
}

pub struct Ads124S08Sensor<'a, M: RawMutex, SpiDevT: SpiDevice, InputPinT: InputPin + Wait, D: DelayNs, const N: usize> {
    ads124s08: &'a Mutex<M, ADS124S08<SpiDevT, InputPinT, D>>,
    signal: Sender<'a, NoopRawMutex, f32, N>,
    measurement_type: MeasurementType,
    conversion_parameters: ConversionParameters,
    offset: f32,
}

impl<'a, M: RawMutex, SpiDevT: SpiDevice, InputPinT: InputPin + Wait, D: DelayNs, const N: usize> Ads124S08Sensor<'a, M, SpiDevT, InputPinT, D, N> {
    pub fn new(
        ads124s08: &'a Mutex<M, ADS124S08<SpiDevT, InputPinT, D>>,
        signal: Sender<'a, NoopRawMutex, f32, N>,
        measurement_type: MeasurementType,
        conversion_parameters: ConversionParameters,
        offset: f32,
    ) -> Self {
        Ads124S08Sensor {
            ads124s08,
            signal,
            measurement_type,
            conversion_parameters,
            offset
        }
    }
}

impl<'a, M: RawMutex, SpiDevT: SpiDevice, InputPinT: InputPin + Wait, D: DelayNs, const N: usize> WithTask for Ads124S08Sensor<'a, M, SpiDevT, InputPinT, D, N> {
    async fn task(&mut self) {
        loop {
            {
                let mut dev = self.ads124s08.lock().await;
                
                let res = match self.measurement_type {
                    MeasurementType::SingleEnded(mux, reference_input, _) => {
                        dev.measure_single_ended(mux, reference_input).await
                    }
                    MeasurementType::RatiometricLowSide(mux_a, mux_b, idac_mux_a, idac_mux_b, reference_input, idac_magnitude, pga_gain, _) => {
                        dev.measure_ratiometric_low_side(mux_a, mux_b, idac_mux_a, idac_mux_b, reference_input, idac_magnitude, pga_gain).await
                    }
                    MeasurementType::AvddBy4 => {
                        dev.read_avdd_by_4().await
                    }
                    MeasurementType::DvddBy4 => {
                        dev.read_dvdd_by_4().await
                    }
                };
                
                if let Ok(value) = res {
                    let val = match self.measurement_type {
                        MeasurementType::SingleEnded(_, _, v) => {
                            value.externally_referenced_voltage(0.0, v)
                        }
                        MeasurementType::RatiometricLowSide(_, _, _, _, _, _, _, ref_r) => {
                            value.ratiometric_resistance(ref_r)
                        }
                        MeasurementType::AvddBy4 | MeasurementType::DvddBy4 => {
                            value.internally_referenced_voltage()
                        }
                    };
                    
                    let val = self.conversion_parameters.convert(val + self.offset);
                
                    self.signal.send(val);
                    //defmt::info!("Read value: {}", val);
                } else {
                    panic!("Failed to read value: {:?}", res);
                }
            }

            Timer::after_millis(50).await;
        }
    }
}
