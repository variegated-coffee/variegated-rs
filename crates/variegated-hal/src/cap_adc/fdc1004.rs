use variegated_log::{log_warn, log_info, log_error};
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, RawMutex};
use embassy_sync::mutex::Mutex;
use embassy_sync::watch::Sender;
use embassy_time::{Duration, Instant, Timer};
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;
use variegated_fdc1004::{FDC1004, FDC1004Error, Channel, SuccessfulMeasurement};
use core::sync::atomic::{AtomicU8, Ordering};
use variegated_checkin::{CheckinDetail, CheckinStatus};
use variegated_instrumentation::async_task_loop;
use crate::{WithTask, SensorReading};

/// One check-in row for one FDC1004, shared by every channel read from it.
///
/// # Why the chip and not the channel
///
/// A board reads several channels off one FDC1004 behind one mutex, and almost everything
/// that goes wrong is a property of the chip or the bus rather than of a channel:
/// `I2CError` and `MeasurementNotComplete` hit every channel at once. Giving each channel a
/// row spends a slot per channel to report the same fault twice, and slots are scarce -- the
/// GS3 uses the whole frame.
///
/// What that costs is worth stating plainly: `UnableToFindCapdacSetting` **is** per-channel
/// -- the CAPDAC search depends on that probe's capacitance range, so a disconnected probe
/// on one input produces it while the others are fine -- and this row cannot say which
/// probe. The loss is smaller than it looks, because both that error and `I2CError` already
/// collapse to the same two [`CheckinDetail`] values on the wire; the row would have named
/// the *channel*, not the *fault*, and the `log_warn!` at the failure site names it anyway.
///
/// # One writer
///
/// Several sensor tasks report here, but this object owns the handle, so there is exactly
/// one writer to the slot -- which is the contract `variegated_checkin` enforces. Each
/// channel's state is kept separately and the row is recomputed from all of them, so a
/// channel that recovers clears its own contribution and does not clear anyone else's.
pub struct Fdc1004Health {
    /// Bit per channel: set means that channel's last read failed but is within its retry
    /// budget.
    degraded: AtomicU8,
    /// Bit per channel: set means that channel has exhausted its retry budget.
    unresponsive: AtomicU8,
    checkin: variegated_checkin::CheckinHandle,
}

impl Fdc1004Health {
    pub const fn new(checkin: variegated_checkin::CheckinHandle) -> Self {
        Self {
            degraded: AtomicU8::new(0),
            unresponsive: AtomicU8::new(0),
            checkin,
        }
    }

    /// One bit per channel.
    ///
    /// `CAPDAC` and `DISABLED` are not measurement inputs -- a sensor configured with either
    /// gets `InvalidMeasurementChannel` from the driver on every read -- but they share a
    /// bit rather than aliasing onto `CIN1`, so a misconfigured sensor reports as its own
    /// standing fault instead of blaming a channel that is fine.
    fn bit(channel: Channel) -> u8 {
        1 << match channel {
            Channel::CIN1 => 0,
            Channel::CIN2 => 1,
            Channel::CIN3 => 2,
            Channel::CIN4 => 3,
            Channel::CAPDAC | Channel::DISABLED => 4,
        }
    }

    /// Record one channel's outcome and republish the row.
    ///
    /// `Relaxed` throughout: the two masks are only ever read together to recompute a status
    /// that is itself sampled at 1 Hz, and a reader that saw one update without the other
    /// would be a tick early on a transition it is about to see anyway.
    fn report(&self, channel: Channel, degraded: bool, unresponsive: bool) {
        let bit = Self::bit(channel);
        let set = |mask: &AtomicU8, on: bool| {
            if on {
                mask.fetch_or(bit, Ordering::Relaxed);
            } else {
                mask.fetch_and(!bit, Ordering::Relaxed);
            }
        };
        set(&self.degraded, degraded);
        set(&self.unresponsive, unresponsive);

        // Worst wins across channels: with one probe out and one fine, the water level is
        // still partly unknown, and a row that reported the healthy one would be reporting
        // the wrong thing.
        self.checkin.record(if self.unresponsive.load(Ordering::Relaxed) != 0 {
            CheckinStatus::Error(CheckinDetail::PeripheralUnresponsive)
        } else if self.degraded.load(Ordering::Relaxed) != 0 {
            CheckinStatus::Warning(CheckinDetail::Degraded)
        } else {
            CheckinStatus::Good
        });
    }
}

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
    /// The chip's shared row, if this sensor is monitored. See [`Self::with_health`].
    ///
    /// A borrow rather than an owned handle, because the row belongs to the FDC1004 and
    /// several channels report into it -- see [`Fdc1004Health`].
    health: Option<&'a Fdc1004Health>,
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
            health: None,
        }
    }

    /// Report this channel's outcome into the chip's shared check-in row.
    ///
    /// Pass the **same** [`Fdc1004Health`] to every sensor reading the same FDC1004; that is
    /// the point of it. A caller that sets this must not also wrap [`WithTask::task`] in
    /// `variegated_checkin::watch` -- the health object is the slot's one writer.
    pub fn with_health(mut self, health: &'a Fdc1004Health) -> Self {
        self.health = Some(health);
        self
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
                    if let Some(health) = self.health {
                        health.report(self.channel, false, false);
                    }
                }
                Err(e) => {
                    // Same gradient the retry budget already tracks, and the same reading as
                    // the ADS's: below the threshold the level sensor is missing samples but
                    // recovering, at or above it the machine's water level is unknown.
                    //
                    // `MeasurementNotComplete` below is deliberately *not* excluded here even
                    // though it does not count against `consecutive_failures`: it still means
                    // this pass produced no reading, and a row that stayed green through a
                    // sensor producing nothing would be reporting the loop rather than the
                    // sensor. It lands on `Degraded`, which is what "expected occasionally"
                    // deserves.
                    if let Some(health) = self.health {
                        let exhausted =
                            self.consecutive_failures + 1 >= self.max_consecutive_failures;
                        health.report(self.channel, !exhausted, exhausted);
                    }

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