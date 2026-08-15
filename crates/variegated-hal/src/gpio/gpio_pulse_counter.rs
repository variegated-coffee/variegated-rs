use embassy_rp::gpio::Input;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::watch::Sender;
use embassy_time::{Instant, Timer};
use variegated_log::log_info;
use crate::{WithTask, SensorReading};

pub struct GpioTransformingPulseCounter<'a, M: RawMutex, T: Clone, U: Clone, F: Fn(f32) -> T, G: Fn(u64) -> U, const N: usize> {
    input: Input<'a>,
    frequency_signal: Sender<'a, M, SensorReading<T>, N>,
    total_pulses_signal: Option<Sender<'a, M, SensorReading<U>, N>>,
    frequency_transformer: F,
    total_transformer: G,
    measurements: [(u64, Instant); 10],
    measurement_index: usize,
    total_pulses: u64,
    last_measurement_instant: Instant,
    startup_complete: bool,
}

impl<'a, M: RawMutex, T: Clone, U: Clone, F: Fn(f32) -> T, G: Fn(u64) -> U, const N: usize> GpioTransformingPulseCounter<'a, M, T, U, F, G, N> {
    pub fn new(
        input: Input<'a>,
        frequency_signal: Sender<'a, M, SensorReading<T>, N>,
        total_pulses_signal: Option<Sender<'a, M, SensorReading<U>, N>>,
        frequency_transformer: F,
        total_transformer: G
    ) -> Self {
        let now = Instant::now();
        GpioTransformingPulseCounter {
            input,
            frequency_signal,
            total_pulses_signal,
            frequency_transformer,
            total_transformer,
            measurements: [(0u64, now); 10],
            measurement_index: 0,
            total_pulses: 0,
            last_measurement_instant: now,
            startup_complete: false,
        }
    }

    fn find_measurement_near(&self, target: Instant) -> (u64, Instant) {
        // Find the measurement closest to the target time, but prefer measurements
        // that are at least 500ms in the past for more stable frequency calculation
        let mut best_measurement = self.measurements[0];
        let mut best_diff = if target > best_measurement.1 {
            target.duration_since(best_measurement.1)
        } else {
            best_measurement.1.duration_since(target)
        };

        for i in 1..10 {
            let measurement = self.measurements[i];
            let diff = if target > measurement.1 {
                target.duration_since(measurement.1)
            } else {
                measurement.1.duration_since(target)
            };

            if diff < best_diff {
                best_diff = diff;
                best_measurement = measurement;
            }
        }

        best_measurement
    }
}

impl<'a, M: RawMutex, T: Clone, U: Clone, F: Fn(f32) -> T, G: Fn(u64) -> U, const N: usize> WithTask for GpioTransformingPulseCounter<'a, M, T, U, F, G, N> {
    async fn task(&mut self) {
        let startup_time = Instant::now();

        loop {
            // Create a future that will trigger on the next GPIO edge or after 100ms timeout
            let edge_future = self.input.wait_for_falling_edge();
            let timeout_future = Timer::after(embassy_time::Duration::from_millis(100));

            // Wait for either a pulse or timeout
            match embassy_futures::select::select(edge_future, timeout_future).await {
                embassy_futures::select::Either::First(_) => {
                    // GPIO edge detected - increment pulse count
                    self.total_pulses += 1;
                }
                embassy_futures::select::Either::Second(_) => {
                    // Timeout - time to do measurement and reporting
                    let measurement_instant = Instant::now();

                    // Check if we're past the startup period (2 seconds)
                    if !self.startup_complete && measurement_instant.duration_since(startup_time).as_secs() >= 2 {
                        self.startup_complete = true;
                        log_info!("GPIO pulse counter startup complete");
                    }

                    // Store measurement
                    self.measurements[self.measurement_index] = (self.total_pulses, measurement_instant);
                    self.measurement_index = (self.measurement_index + 1) % 10;

                    // Calculate frequency only after startup period
                    let frequency = if self.startup_complete {
                        // Look for measurement approximately 1 second ago
                        let one_second_duration = embassy_time::Duration::from_secs(1);
                        let one_second_ago = if measurement_instant.as_ticks() > one_second_duration.as_ticks() {
                            Instant::from_ticks(measurement_instant.as_ticks() - one_second_duration.as_ticks())
                        } else {
                            // Use the oldest measurement we have if we haven't been running for 1 second
                            self.measurements.iter()
                                .map(|(_, instant)| *instant)
                                .min()
                                .unwrap_or(measurement_instant)
                        };

                        let (start_total_pulses, start_time) = self.find_measurement_near(one_second_ago);

                        // Calculate precise frequency using total_pulses
                        let elapsed = measurement_instant.duration_since(start_time);
                        let elapsed_seconds = elapsed.as_micros() as f32 / 1_000_000.0;

                        // Only calculate frequency if we have a reasonable time span (>= 500ms)
                        if elapsed_seconds >= 0.5 {
                            let pulse_count = self.total_pulses.saturating_sub(start_total_pulses);
                            let calculated_frequency = pulse_count as f32 / elapsed_seconds;

                            // Sanity check: reject frequencies that seem impossible (>1000 Hz for flow meter)
                            if calculated_frequency <= 1000.0 {
                                calculated_frequency
                            } else {
                                log_info!("Rejecting impossible frequency: {} Hz (pulse_count={}, elapsed={}s)",
                                      calculated_frequency, pulse_count, elapsed_seconds);
                                0.0
                            }
                        } else {
                            // Not enough time elapsed, use previous frequency or 0
                            0.0
                        }
                    } else {
                        // During startup, report 0 frequency
                        0.0
                    };

                    // Update tracking variables
                    self.last_measurement_instant = measurement_instant;

                    // Send transformed frequency signal
                    let transformed_frequency = (self.frequency_transformer)(frequency);
                    let frequency_reading = SensorReading {
                        raw: frequency,
                        transformed: transformed_frequency,
                    };
                    self.frequency_signal.send(frequency_reading);

                    // Send total pulses signal if configured
                    if let Some(ref total_signal) = self.total_pulses_signal {
                        let transformed_total = (self.total_transformer)(self.total_pulses);
                        let total_reading = SensorReading {
                            raw: self.total_pulses as f32,
                            transformed: transformed_total,
                        };
                        total_signal.send(total_reading);
                    }
                }
            }
        }
    }
}