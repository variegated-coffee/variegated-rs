use variegated_log::{log_warn, log_info, log_error};
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, RawMutex};
use embassy_sync::mutex::Mutex;
use embassy_sync::watch::Sender;
use embassy_time::{Duration, Instant, Timer};
use embedded_hal::digital::InputPin;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::spi::SpiDevice;
use embedded_hal_async::digital::Wait;
use variegated_adc_tools::ConversionParameters;
use variegated_ads124s08::{ADS124S08, ADS124S08Error};
use variegated_ads124s08::registers::{IDACMagnitude, IDACMux, PGAGain, ReferenceInput};
use variegated_ads124s08::registers::Mux;
use variegated_instrumentation::{async_task_loop, CounterHandle, IndicatorHandle};
use crate::{WithTask, SensorReading};

pub enum MeasurementType {
    SingleEnded(Mux, ReferenceInput, f32),
    RatiometricLowSide(Mux, Mux, IDACMux, IDACMux, ReferenceInput, IDACMagnitude, PGAGain, f32),
    AvddBy4,
    DvddBy4,
}

pub struct Ads124S08Sensor<'a, M: RawMutex, SpiDevT: SpiDevice, InputPinT: InputPin + Wait, D: DelayNs, const N: usize, const I: usize, const J: usize> {
    ads124s08: &'a Mutex<M, ADS124S08<SpiDevT, InputPinT, D>>,
    signal: Sender<'a, NoopRawMutex, SensorReading<f32>, N>,
    measurement_type: MeasurementType,
    conversion_parameters: ConversionParameters,
    offset: f32,
    consecutive_failures: u8,
    max_consecutive_failures: u8,
    last_reset_attempt: Option<Instant>,
    reset_count: u32,
    reset_backoff_ms: u32,
    counter: Option<CounterHandle<I>>,
    time_indicator: Option<IndicatorHandle<J>>
}

impl<'a, M: RawMutex, SpiDevT: SpiDevice, InputPinT: InputPin + Wait, D: DelayNs, const N: usize, const I: usize, const J: usize> Ads124S08Sensor<'a, M, SpiDevT, InputPinT, D, N, I, J> {
    pub fn new(
        ads124s08: &'a Mutex<M, ADS124S08<SpiDevT, InputPinT, D>>,
        signal: Sender<'a, NoopRawMutex, SensorReading<f32>, N>,
        measurement_type: MeasurementType,
        conversion_parameters: ConversionParameters,
        offset: f32,
        counter: Option<CounterHandle<I>>,
        time_indicator: Option<IndicatorHandle<J>>,
    ) -> Self {
        Ads124S08Sensor {
            ads124s08,
            signal,
            measurement_type,
            conversion_parameters,
            offset,
            consecutive_failures: 0,
            max_consecutive_failures: 3,
            last_reset_attempt: None,
            reset_count: 0,
            reset_backoff_ms: 100,
            counter,
            time_indicator,
        }
    }

    pub async fn measure(&mut self) {
        let mut dev = self.ads124s08.lock().await;

        let start = Instant::now();

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

        if let Some(counter) = self.counter {
            counter.increment();
        }

        // One `elapsed()`, read once and used: the binding and the indicator used to call
        // it separately, so the number reported was very slightly later than the one the
        // name suggested.
        let took = start.elapsed();

        if let Some(indicator) = self.time_indicator {
            indicator.set(took.as_millis());
        }

        //drop(dev);

        match res {
            Ok(raw_value) => {
                // Reset failure counter on successful read
                self.consecutive_failures = 0;

                let val = match self.measurement_type {
                    MeasurementType::SingleEnded(_, _, v) => {
                        raw_value.externally_referenced_voltage(0.0, v)
                    }
                    MeasurementType::RatiometricLowSide(_, _, _, _, _, _, _, ref_r) => {
                        //info!("ADS124S08 Measurement: {} ohms", raw_value.ratiometric_resistance(ref_r));

                        raw_value.ratiometric_resistance(ref_r)
                    }
                    MeasurementType::AvddBy4 | MeasurementType::DvddBy4 => {
                        raw_value.internally_referenced_voltage()
                    }
                };

                let transformed_val = self.conversion_parameters.convert(val + self.offset);

                let sensor_reading = SensorReading {
                    raw: val,
                    transformed: transformed_val,
                };

                self.signal.send(sensor_reading);
            }
            Err(e) => {
                // Check if this is a read timeout error
                if matches!(e, ADS124S08Error::ReadTimeoutError) {
                    self.consecutive_failures += 1;
                    log_warn!("ADS124S08 read timeout ({}/{})", self.consecutive_failures, self.max_consecutive_failures);

                    // Check if we should attempt a reset
                    if self.consecutive_failures >= self.max_consecutive_failures {
                        log_info!("ADS124S08: Attempting auto-reset after {} consecutive timeouts", self.consecutive_failures);

                        // Check if we need to apply backoff
                        if let Some(last_reset) = self.last_reset_attempt {
                            let elapsed = Instant::now() - last_reset;
                            let backoff_duration = Duration::from_millis(self.reset_backoff_ms as u64);

                            if elapsed < backoff_duration {
                                let remaining = backoff_duration - elapsed;
                                log_info!("ADS124S08: Waiting {}ms before reset (backoff)", remaining.as_millis());
                                Timer::after(remaining).await;
                            }
                        }

                        // Re-take a lock to reset.
                        //let mut dev = self.ads124s08.lock().await;

                        // Attempt reset
                        match dev.reset().await {
                            Ok(()) => {
                                self.reset_count += 1;
                                self.consecutive_failures = 0;
                                self.last_reset_attempt = Some(Instant::now());

                                // Exponential backoff: double the backoff time up to 5 seconds
                                self.reset_backoff_ms = (self.reset_backoff_ms * 2).min(5000);

                                log_info!("ADS124S08: Reset successful (count: {}, next backoff: {}ms)",
                                              self.reset_count, self.reset_backoff_ms);
                            }
                            Err(_reset_error) => {
                                log_error!("ADS124S08: Reset failed");
                                // Reset the failure counter to prevent immediate retry
                                self.consecutive_failures = 0;
                            }
                        }
                    }
                } else {
                    // For non-timeout errors, just log them
                    log_warn!("ADS124S08: Read error (non-timeout)");
                }
            }
        }
    }
}

impl<'a, M: RawMutex, SpiDevT: SpiDevice, InputPinT: InputPin + Wait, D: DelayNs, const N: usize, const I: usize, const J: usize> WithTask for Ads124S08Sensor<'a, M, SpiDevT, InputPinT, D, N, I, J> {
    async fn task(&mut self) {
        async_task_loop!("ADS124S08 Sensor", Some(Duration::from_millis(5)), {
            self.measure().await;
        })
    }
}
