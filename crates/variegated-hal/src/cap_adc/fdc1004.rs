use variegated_log::{log_warn, log_info, log_error};
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, RawMutex};
use embassy_sync::mutex::Mutex;
use embassy_sync::watch::Sender;
use embassy_time::{Duration, Instant, Timer};
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;
use variegated_fdc1004::{FDC1004, FDC1004Error, Channel, SuccessfulMeasurement};
use variegated_instrumentation::async_task_loop;
use crate::{WithTask, SensorReading};

fn successful_measurement_to_f32(measurement: SuccessfulMeasurement) -> f32 {
    match measurement {
        SuccessfulMeasurement::MeasurementInRange(capacitance) => capacitance.to_pf(),
        SuccessfulMeasurement::Underflow => -1.0,  // Sentinel value for underflow
        SuccessfulMeasurement::Overflow => -2.0,   // Sentinel value for overflow
    }
}

pub struct Fdc1004Sensor<'a, M: RawMutex, I2C: I2c, D: DelayNs, T: Clone, F: Fn(SuccessfulMeasurement) -> T, const N: usize> {
    fdc1004: &'a Mutex<M, FDC1004<I2C, D>>,
    signal: Sender<'a, NoopRawMutex, SensorReading<T>, N>,
    transformer: F,
    channel: Channel,
    consecutive_failures: u8,
    max_consecutive_failures: u8,
    last_reset_attempt: Option<Instant>,
    reset_count: u32,
    reset_backoff_ms: u32,
}

impl<'a, M: RawMutex, I2C: I2c, D: DelayNs, T: Clone, F: Fn(SuccessfulMeasurement) -> T, const N: usize>
    Fdc1004Sensor<'a, M, I2C, D, T, F, N>
where
    I2C::Error: core::fmt::Debug,
{
    pub fn new(
        fdc1004: &'a Mutex<M, FDC1004<I2C, D>>,
        signal: Sender<'a, NoopRawMutex, SensorReading<T>, N>,
        transformer: F,
        channel: Channel,
    ) -> Self {
        Fdc1004Sensor {
            fdc1004,
            signal,
            transformer,
            channel,
            consecutive_failures: 0,
            max_consecutive_failures: 3,
            last_reset_attempt: None,
            reset_count: 0,
            reset_backoff_ms: 100,
        }
    }
}

impl<'a, M: RawMutex, I2C: I2c, D: DelayNs, T: Clone, F: Fn(SuccessfulMeasurement) -> T, const N: usize>
    WithTask for Fdc1004Sensor<'a, M, I2C, D, T, F, N>
where
    I2C::Error: core::fmt::Debug,
{
    async fn task(&mut self) {
        async_task_loop!("FDC1004 Sensor", Some(Duration::from_millis(100)), {
            let mut dev = self.fdc1004.lock().await;

            let res = dev.read_capacitance(self.channel).await;

            match res {
                Ok(measurement) => {
                    // Reset failure counter on successful read
                    self.consecutive_failures = 0;


                    // Transform the measurement and send it
                    let transformed_val = (self.transformer)(measurement);

                    let sensor_reading = SensorReading {
                        raw: successful_measurement_to_f32(measurement),
                        transformed: transformed_val,
                    };

                    self.signal.send(sensor_reading);
                }
                Err(e) => {
                    match e {
                        FDC1004Error::MeasurementNotComplete => {
                            // This is expected occasionally, don't count as failure
                            log_warn!("FDC1004: Measurement not complete");
                        }
                        FDC1004Error::UnableToFindCapdacSetting => {
                            // This indicates a measurement issue, count as failure
                            self.consecutive_failures += 1;
                            log_warn!("FDC1004: Unable to find CAPDAC setting ({}/{})",
                                  self.consecutive_failures, self.max_consecutive_failures);
                        }
                        FDC1004Error::InvalidMeasurementChannel => {
                            // This is a programming error, log it
                            log_error!("FDC1004: Invalid measurement channel configured");
                        }
                        FDC1004Error::I2CError(_) => {
                            // I2C communication error
                            self.consecutive_failures += 1;
                            log_warn!("FDC1004: I2C error ({}/{})",
                                  self.consecutive_failures, self.max_consecutive_failures);
                        }
                    }

                    // Check if we should attempt a reset
                    if self.consecutive_failures >= self.max_consecutive_failures {
                        log_info!("FDC1004: Max consecutive failures reached, may need reset");

                        // Check if we need to apply backoff
                        if let Some(last_reset) = self.last_reset_attempt {
                            let elapsed = Instant::now() - last_reset;
                            let backoff_duration = Duration::from_millis(self.reset_backoff_ms as u64);

                            if elapsed < backoff_duration {
                                let remaining = backoff_duration - elapsed;
                                log_info!("FDC1004: Waiting {}ms before retry (backoff)", remaining.as_millis());
                                Timer::after(remaining).await;
                            }
                        }

                        // For FDC1004, we can't reset the device directly, but we can try to
                        // reinitialize the measurement. Mark the attempt and reset counter.
                        self.reset_count += 1;
                        self.consecutive_failures = 0;
                        self.last_reset_attempt = Some(Instant::now());

                        // Exponential backoff: double the backoff time up to 5 seconds
                        self.reset_backoff_ms = (self.reset_backoff_ms * 2).min(5000);

                        log_info!("FDC1004: Recovery attempt {} (next backoff: {}ms)",
                              self.reset_count, self.reset_backoff_ms);
                    }
                }
            }
        })
    }
}