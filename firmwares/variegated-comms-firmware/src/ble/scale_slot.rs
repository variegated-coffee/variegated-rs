//! The scale measurement loop, and the two protocols that run in it.
//!
//! # Why this is one function and not one per driver
//!
//! The connect/retry cycle, the connection-status edges, the command drain, the
//! per-slot addressing check and the drop-rather-than-block publishing discipline are
//! properties of *this firmware's slot model*, not of any scale protocol. Written out per
//! driver they were 255 lines apiece, and each line carried a rule that a second copy
//! would have to rediscover -- several of which were learned the hard way and are recorded
//! in comments below. A third copy is the drift `CLAUDE.md` warns about in the two
//! espresso firmwares, where duplicated logic has already produced real bugs.
//!
//! So [`run_scale_slot`] owns all of that, and a protocol supplies only what is genuinely
//! its own: how to connect, how to open a stream, how to decode a frame, how to carry out
//! an operation, and what if anything it must do periodically.
//!
//! # Why a trait and not an enum of state machines
//!
//! The enum looks like the memory-safe choice and is the opposite.
//!
//! The driver future is boxed once per assignment, and the note at its allocation site in
//! `devices.rs` records the panic that taught us to care: `memory allocation of 12000
//! bytes failed`, a *contiguity* failure inside one `esp_alloc` region rather than a
//! shortage. Each `match` arm there is boxed separately, so this function's future is
//! allocated at its own size -- 4512 bytes for the ACAIA -- and never at the size of the
//! heaviest driver compiled in.
//!
//! An enum session would still be worse, and for a reason boxing does not address. A
//! notification stream borrows its GATT client, and a self-referential pair cannot live in
//! one struct -- so both clients would have to be function-scope `Option`s, live across
//! every await, and therefore both present in the coroutine *simultaneously*. That is a
//! sum under any layout, and a `GattClient` is 1392 bytes.
//!
//! Which is worth stating precisely, because this frame already holds two of them: one
//! inside the `connect` future and one in the local it lands in. Measured -- halving the
//! GATT notification queue took 2096 bytes off a `GattClient` and 4192 off this future.
//! An enum would make that three or four, not two.
//!
//! The generic costs one extra copy of this function's machine code, which is the same
//! code a hand-written second loop would have cost anyway.

use embassy_futures::select::{select, select3, Either3};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Sender;
use embassy_sync::pubsub::Subscriber;
use embassy_time::{Duration, Instant, Timer};
use trouble_host::prelude::{BdAddr, Connection};
use variegated_adc_tools::{ConversionParameters, KalmanFilterParameters};
use variegated_scale_trouble_driver::acaia_new::{
    AcaiaNewDriver, AcaiaNewGattClient, AcaiaNewNotificationStream, ScaleEvent as AcaiaNewEvent,
};
use variegated_scale_trouble_driver::acaia_old::{
    AcaiaOldDriver, AcaiaOldGattClient, ScaleEvent as AcaiaScaleEvent,
    ScaleNotificationStream as AcaiaScaleNotificationStream, TimerOp as AcaiaTimerOp,
};
use variegated_scale_trouble_driver::bookoo::{
    BookooDriver, BookooGattClient, BookooNotificationStream, Command as BookooCommand,
    ScaleEvent as BookooScaleEvent,
};
use variegated_controller_types::bluetooth::MAX_BLUETOOTH_PERIPHERALS;
use variegated_controller_types::{ExternalPeripheralSensorReading, PeripheralId, ScaleOp};
use variegated_log::{log_error, log_info, log_warn};

use crate::ble::status;
use crate::channels::SENSOR_READING_CAPACITY;
use crate::config::{
    BLUETOOTH_SCALE_ENDPOINT_BATTERY, BLUETOOTH_SCALE_ENDPOINT_FLOW,
    BLUETOOTH_SCALE_ENDPOINT_WEIGHT,
};

use super::{SlotController, SlotHandle, SlotPool, SlotStack};

/// Number of weight samples the flow estimator differences across.
///
/// The baseline is `FLOW_WINDOW_SAMPLES - 1` intervals, so six samples on the scale's
/// 80 ms grid is a 400 ms baseline. That number is chosen against quantisation, which is
/// the dominant error here and not load-cell noise: weight arrives quantised to 0.1 g
/// (`acaia_old::types`), so at a realistic 2 g/s the true step is 0.16 g per sample and a
/// difference between *adjacent* samples can only ever come out as 1.25 or 2.5 g/s, with
/// nothing in between. The quantum is fixed in the numerator, so stretching the baseline
/// five-fold divides its contribution five-fold. The cost is one window of lag, which a
/// 25-30 s shot absorbs easily.
const FLOW_WINDOW_SAMPLES: usize = 6;

/// Median window, for outlier rejection ahead of the Kalman.
///
/// Note what this does and does not do. A median *selects* an existing sample, so it
/// cannot average the quantisation staircase away -- that is the Kalman's job, and the
/// baseline above is what makes the staircase fine enough to be worth averaging. What the
/// median is for is the genuine outlier: a sample delivered a whole connection interval
/// late because one BLE notification carried two frames, which the driver surfaces one at
/// a time.
const FLOW_MEDIAN_WINDOW: usize = 5;

/// Below this, flow reports exactly zero.
///
/// An idle scale still produces a small non-zero slope out of quantisation noise, and a
/// display or a PID reading +-0.08 g/s from a scale with nothing on it is reporting
/// something that is not happening.
///
/// The tradeoff is real and worth stating: this equally suppresses *genuine* slow flow at
/// the tail of a shot, which is exactly where brew-by-weight is deciding when to stop. It
/// is set low enough that it should sit under the noise floor rather than inside the
/// signal, but it is the first constant to revisit if the last gram of a shot reads wrong.
const FLOW_DEADBAND_G_PER_S: f32 = 0.1;

/// Shortest baseline that yields a usable rate, in microseconds.
///
/// Guards the division. `embassy_time` ticks at 1 MHz here, so this is not about clock
/// resolution -- it is that two samples reassembled into the same instant would divide a
/// non-zero weight delta by nearly nothing and produce an enormous rate.
const FLOW_MIN_BASELINE_US: u64 = 1_000;

/// Derives gravimetric flow rate from a stream of weight samples.
///
/// Used only by protocols whose [`ScaleProtocol::NATIVE_FLOW`] is `false`. Lives on this
/// processor rather than the application processor for two reasons. The UART hop and the
/// application processor's scheduling both add latency *between* samples, and
/// differentiation is precisely the operation that turns jitter in sample timing into
/// error in the result. And other Bluetooth scales report flow computed in the scale
/// itself, so the application processor should receive flow as a measurement whoever
/// produced it, rather than knowing that one particular scale needs it synthesised.
///
/// The output is mass flow, g/s. The application processor's `FlowRateType` is nominally
/// ml/s; under the 1 g/ml assumption the rest of the codebase already makes for coffee
/// (see `dual_boiler_single_group`'s output-volume derivation) they are interchangeable,
/// and no conversion is applied.
pub struct FlowEstimator {
    samples: heapless::Deque<(Instant, f32), FLOW_WINDOW_SAMPLES>,
    filter: ConversionParameters,
}

impl FlowEstimator {
    pub fn new() -> Self {
        Self {
            samples: heapless::Deque::new(),
            filter: Self::filter(),
        }
    }

    /// `linear_conversion(1.0, 0.0)` is the identity -- the value is already g/s and needs
    /// no conversion. It is present because `convert()` applies the median *before* the
    /// conversion step and the Kalman *after* it, which is the order this wants, and an
    /// explicit identity is clearer than relying on the no-conversion-configured path.
    fn filter() -> ConversionParameters {
        ConversionParameters::linear_conversion(1.0, 0.0)
            .with_median_filter(FLOW_MEDIAN_WINDOW)
            .with_kalman_preset(KalmanFilterParameters::balanced())
    }

    /// Discard all history, including the filters'.
    ///
    /// The filters are rebuilt rather than reset. `ConversionParameters::reset_kalman_filter`
    /// does not restore the error covariance to its initial value -- the initial value is
    /// not stored on the filter at all, despite a comment in that crate saying it will be --
    /// so a reset filter would carry its old confidence into a fresh signal. Rebuilding is
    /// two allocations of nothing and is exactly right.
    fn reset(&mut self) {
        self.samples.clear();
        self.filter = Self::filter();
    }

    /// Feed a weight sample; returns a flow rate once there is enough history for one.
    ///
    /// `None` rather than `0.0` while warming up. Zero is a *measurement* here -- it means
    /// "not flowing" -- so reporting it before the estimator can tell would be a lie of
    /// exactly the kind the debug snapshot's "absent, never zero" rule exists to prevent.
    ///
    /// A weight of exactly zero is the other side of that same rule and goes the other way:
    /// it is not a gap in the estimator's knowledge, it is a measurement of nothing having
    /// flowed, so it reports `Some(0.0)`. Returning `None` there left the application
    /// processor's `Watch` holding whatever the last real reading was -- in practice the
    /// large negative spike from the cup being lifted or the scale taring -- for the whole
    /// of headspace fill, which is exactly the window in which the true answer is zero.
    pub fn push(&mut self, now: Instant, weight: f32) -> Option<f32> {
        // A reading of exactly zero is the observable end state of a tare, and the tare is
        // the one discontinuity that would otherwise wreck this. Keying on the value rather
        // than on the tare *command* matters: the scale runs several of its own measuring
        // cycles before the reading settles, so a reset when the command is written would
        // discard samples that are still pre-tare and leave the transition in the buffer.
        //
        // It also catches what no command can. A tare from the scale's own button is
        // invisible here -- the driver surfaces only `ScaleEvent::Weight` -- as is a
        // power-on, and the cup being lifted off. All three land on zero.
        //
        // Comparing a float with `==` is safe on this value specifically: it is decoded as
        // `raw_u16 / 10^scale_index` with a separate sign bit, so a zero raw reading is
        // exactly `0.0` or `-0.0` (which compare equal), never an accumulated near-zero.
        if weight == 0.0 {
            self.reset();
            return Some(0.0);
        }

        if self.samples.is_full() {
            self.samples.pop_front();
        }
        let _ = self.samples.push_back((now, weight));

        if !self.samples.is_full() {
            return None;
        }

        let (t_old, w_old) = *self.samples.front()?;
        let (t_new, w_new) = *self.samples.back()?;

        let dt_us = t_new.duration_since(t_old).as_micros();
        if dt_us < FLOW_MIN_BASELINE_US {
            return None;
        }

        let raw = (w_new - w_old) / (dt_us as f32 / 1_000_000.0);
        let smoothed = self.filter.convert(raw);

        Some(if smoothed.abs() < FLOW_DEADBAND_G_PER_S {
            0.0
        } else {
            smoothed
        })
    }
}

/// One notification's worth of measurement.
///
/// **Absent, never zero** -- the same rule the debug snapshot follows, and the reason this
/// is not a struct of bare `f32`s. An ACAIA frame carries a weight alone, a BooKoo weight
/// frame carries weight, flow and battery together, and a BooKoo auto-mode frame carries
/// none of them. A zero would be indistinguishable from a real measurement of nothing.
#[derive(Default)]
pub struct ScaleSample {
    /// Grams.
    pub weight: Option<f32>,
    /// Grams per second. Only ever `Some` when [`ScaleProtocol::NATIVE_FLOW`].
    pub flow: Option<f32>,
    /// Percent, 0-100.
    pub battery: Option<f32>,
}

/// Everything a scale protocol must supply up to the point where a stream exists.
///
/// Split from [`ScaleSession`] because the stream borrows the GATT client, so the client
/// has to be a local of [`run_scale_slot`] and cannot be owned by the thing that reads it.
/// `'static` because every implementation is a unit struct naming a protocol, never a
/// value with a borrow in it. Stating it here is what lets [`ScaleProtocol::open`] return
/// `Self::Session<'g>` without carrying a `where Self: 'g` onto every call site.
#[allow(async_fn_in_trait)]
pub trait ScaleProtocol: 'static {
    /// The per-connection GATT client.
    ///
    /// No lifetime parameter: `ManagerHandle::register_device` returns a
    /// `DeviceHandle<'a, ..>` whose `'a` is the *manager's*, and the manager is `'static`
    /// in `ble_slot_task`, so every driver type here instantiates at `'static`.
    type Gatt;

    /// The live session, borrowing the client above.
    type Session<'g>: ScaleSession
    where
        Self: 'g;

    /// What this protocol is called in log lines.
    const NAME: &'static str;

    /// Whether the scale reports flow itself.
    ///
    /// `false` runs [`FlowEstimator`] over the weight stream. `true` forwards what the
    /// scale sent and never differentiates -- which is not merely a shortcut: BooKoo
    /// documents no notification rate, so a derivative taken over an unknown sample
    /// interval could easily be worse than the scale's own.
    ///
    /// The endpoint does not distinguish the two, deliberately; see
    /// `variegated_hal::scale::bluetooth`.
    const NATIVE_FLOW: bool;

    /// Whether the link is up, without touching it.
    async fn is_connected(handle: &SlotHandle, stack: &'static SlotStack, address: BdAddr) -> bool;

    /// Build a GATT client. The `Connection` comes back so the caller can keep it alive.
    async fn connect(
        handle: &SlotHandle,
        stack: &'static SlotStack,
        address: BdAddr,
    ) -> Option<(Connection<'static, SlotPool>, Self::Gatt)>;

    /// Run the GATT client's own task.
    ///
    /// The error is discarded: this loop treats the task completing as "the connection is
    /// gone" regardless of why, which is what both hand-written loops did.
    async fn run_gatt_task(gatt: &Self::Gatt);

    /// Subscribe and perform whatever handshake the protocol requires, in its own order.
    async fn open(gatt: &Self::Gatt) -> Option<Self::Session<'_>>;
}

/// A connected scale. One per link, dropped when the link goes.
#[allow(async_fn_in_trait)]
pub trait ScaleSession {
    /// The next notification, already decoded.
    ///
    /// `Err(())` means the stream is unusable and the link should be torn down. A frame
    /// that merely failed to parse is *not* that -- a driver should log it and keep
    /// reading, since one corrupt notification on a busy radio is normal.
    async fn next(&mut self) -> Result<ScaleSample, ()>;

    /// Carry out an operation the application processor asked for.
    ///
    /// Logs its own failures and returns nothing: a command that the scale rejected is not
    /// a reason to drop a working connection.
    async fn apply(&mut self, op: ScaleOp);

    /// Called at the bottom of *every* iteration of the loop body.
    ///
    /// ACAIA's implementation is its 2 s heartbeat; BooKoo's is empty, because the
    /// protocol needs no keepalive. The 1 s liveness timer in the `select3` above
    /// guarantees this is reached at least once a second even on a completely silent link,
    /// which is what gives the ACAIA heartbeat a second of margin against its ~3 s
    /// deadline.
    async fn tick(&mut self);
}

/// Publishes scale readings, dropping rather than blocking.
///
/// `try_send`, never `send().await`, and the reason is not merely that a stale sample is
/// worth little. `ScaleSession::tick` runs in the same loop body, so a blocked send stops
/// the ACAIA heartbeat and the scale drops the link within seconds -- turning transient
/// UART backpressure into a BLE disconnect and a five-second reconnect cycle. Dropping
/// costs one sample, which the next notification supersedes ~80 ms later.
///
/// Holding the edge flag here rather than as a local is what stops a second driver
/// rediscovering that.
struct ReadingPublisher {
    sender: Sender<
        'static,
        CriticalSectionRawMutex,
        ExternalPeripheralSensorReading,
        SENSOR_READING_CAPACITY,
    >,
    peripheral_id: PeripheralId,
    /// Edge-triggers the "channel full" log. A persistently full channel would otherwise
    /// log at the notification rate, which is the same flood the drop exists to avoid.
    dropping: bool,
}

impl ReadingPublisher {
    fn new(
        sender: Sender<
            'static,
            CriticalSectionRawMutex,
            ExternalPeripheralSensorReading,
            SENSOR_READING_CAPACITY,
        >,
        peripheral_id: PeripheralId,
    ) -> Self {
        Self {
            sender,
            peripheral_id,
            dropping: false,
        }
    }

    /// Send one reading.
    ///
    /// Only the weight endpoint drives the congestion log. Two edge-triggered flags for
    /// one channel would both fire on the same congestion and say the same thing twice,
    /// and weight is the endpoint that always exists.
    fn publish(&mut self, endpoint: u8, value: f32) {
        let reading = ExternalPeripheralSensorReading {
            id: self.peripheral_id,
            endpoint,
            value,
        };

        match self.sender.try_send(reading) {
            Ok(()) => {
                if endpoint == BLUETOOTH_SCALE_ENDPOINT_WEIGHT && self.dropping {
                    self.dropping = false;
                    log_info!("Sensor channel drained, forwarding scale weights again");
                }
            }
            Err(_) => {
                if endpoint == BLUETOOTH_SCALE_ENDPOINT_WEIGHT && !self.dropping {
                    self.dropping = true;
                    log_error!("Sensor channel full, dropping scale weights");
                }
            }
        }
    }
}

/// Whether the inner loop should keep going or tear the link down.
enum Step {
    Continue,
    Break,
}

/// The measurement loop for one scale, in one slot.
///
/// See the module docs for why this is generic rather than duplicated, and
/// [`ScaleSession::tick`] for the structural reason the ACAIA heartbeat cannot be skipped.
pub async fn run_scale_slot<P: ScaleProtocol>(
    handle: SlotHandle,
    stack: &'static SlotStack,
    address: BdAddr,
    peripheral_id: PeripheralId,
    slot: usize,
    sensor_sender: Sender<
        'static,
        CriticalSectionRawMutex,
        ExternalPeripheralSensorReading,
        SENSOR_READING_CAPACITY,
    >,
    scale_commands: &mut Subscriber<
        'static,
        CriticalSectionRawMutex,
        (PeripheralId, ScaleOp),
        4,
        MAX_BLUETOOTH_PERIPHERALS,
        1,
    >,
) {
    loop {
        if !P::is_connected(&handle, stack, address).await {
            status::set_slot_connected(slot, false);
            Timer::after(Duration::from_secs(1)).await;
            continue;
        }

        log_info!("{} scale connected, creating GATT client...", P::NAME);

        match P::connect(&handle, stack, address).await {
            Some((_conn, gatt)) => {
                log_info!("{} GATT client created", P::NAME);

                select(P::run_gatt_task(&gatt), async {
                    let Some(mut session) = P::open(&gatt).await else {
                        log_error!("Failed to initialize {} scale", P::NAME);
                        return;
                    };

                    log_info!("{} scale initialized successfully", P::NAME);

                    // Only after `open` succeeds, never after `connect` does: a scale that
                    // is connected but not subscribed reports nothing, and saying it is up
                    // would make a silent slot look healthy.
                    status::set_slot_connected(slot, true);

                    // Discard any scale op that arrived while the scale was down.
                    //
                    // The subscriber queues rather than latching, but the hazard is the
                    // same one the `Signal` had: a tare asked for during a disconnect
                    // would fire the moment the link came back -- possibly minutes later,
                    // and possibly mid-shot. A stale tare is worse than a dropped one,
                    // because the operator who asked has long since moved on and zeroing a
                    // scale under a running extraction corrupts it.
                    //
                    // Draining in a loop, where the `Signal` needed one `reset()`: the
                    // queue can hold more than one entry.
                    while scale_commands.try_next_message_pure().is_some() {}

                    let mut publisher = ReadingPublisher::new(sensor_sender, peripheral_id);
                    // Declared inside the connected scope, so a reconnect starts with no
                    // history rather than differencing the first new sample against a
                    // weight from before the link dropped.
                    let mut flow = FlowEstimator::new();

                    loop {
                        // Bound before matching, deliberately. The `select3` future holds a
                        // `&mut` on `session`, and a temporary in a `match` scrutinee lives
                        // for the whole `match` -- so matching directly on it would make
                        // `session.apply()` in the third arm a second mutable borrow.
                        //
                        // Declaration order is priority: `select3` polls in order, so
                        // weights keep priority over a command. That is right -- a command
                        // is a single write and can wait a notification, while a dropped
                        // weight is a gap in a control signal.
                        let event = select3(
                            session.next(),
                            Timer::after(Duration::from_secs(1)),
                            scale_commands.next_message_pure(),
                        )
                        .await;

                        let step = match event {
                            Either3::First(Ok(sample)) => {
                                publish_sample::<P>(&mut publisher, &mut flow, sample);
                                Step::Continue
                            }
                            Either3::First(Err(())) => {
                                log_error!("Failed to read {} event", P::NAME);
                                Step::Break
                            }
                            Either3::Second(_) => {
                                if P::is_connected(&handle, stack, address).await {
                                    Step::Continue
                                } else {
                                    log_info!(
                                        "{} connection lost during measurements, exiting",
                                        P::NAME
                                    );
                                    Step::Break
                                }
                            }
                            // `target` rather than `peripheral_id`: this loop has an id of
                            // its own, and shadowing it would make the comparison below
                            // compare a thing to itself.
                            Either3::Third((target, op)) => {
                                // The id is checked, not assumed. Nothing has validated it
                                // upstream -- it arrives off the UART from the other
                                // processor rather than from this firmware's own dispatcher
                                // -- and every scale loop sees every op, because the channel
                                // broadcasts to all subscribers. An unchecked tare would
                                // zero every scale on the machine.
                                if target == peripheral_id {
                                    session.apply(op).await;
                                } else {
                                    log_info!(
                                        "Ignoring scale op for 0x{:04X}, this loop owns 0x{:04X}",
                                        target,
                                        peripheral_id
                                    );
                                }
                                Step::Continue
                            }
                        };

                        // Every path that stays connected reaches this. The rule the ACAIA
                        // loop stated in a comment -- that the command arm must be an `if`
                        // and not a `continue`, or the heartbeat below it is skipped -- is
                        // now enforced by the shape: the match *returns* a value, so no arm
                        // can jump past what follows it.
                        session.tick().await;

                        if matches!(step, Step::Break) {
                            break;
                        }
                    }
                })
                .await;

                log_info!("{} GATT task completed, connection dropped", P::NAME);
                status::set_slot_connected(slot, false);
            }
            None => {
                log_error!("Failed to create {} GATT client", P::NAME);
                status::set_slot_connected(slot, false);
            }
        }

        log_info!("Restarting {} measurement loop...", P::NAME);
        Timer::after(Duration::from_secs(5)).await;
    }
}

/// ACAIA's older protocol.
///
/// Flow is derived: the protocol reports none. The handshake is order-critical and lives
/// in the driver's `initialize`, and the 2 s heartbeat in [`ScaleSession::tick`] is what
/// keeps the link alive against the scale's ~3 s deadline.
pub struct AcaiaOld;

/// A connected ACAIA scale.
pub struct AcaiaOldSession<'g> {
    gatt: &'g AcaiaOldGattClient<'static, SlotController, SlotPool>,
    stream: AcaiaScaleNotificationStream<'g>,
    last_heartbeat: Instant,
}

impl ScaleProtocol for AcaiaOld {
    type Gatt = AcaiaOldGattClient<'static, SlotController, SlotPool>;
    type Session<'g>
        = AcaiaOldSession<'g>
    where
        Self: 'g;

    const NAME: &'static str = "ACAIA";
    const NATIVE_FLOW: bool = false;

    async fn is_connected(handle: &SlotHandle, stack: &'static SlotStack, address: BdAddr) -> bool {
        let device_handle = handle.register_device(address);
        AcaiaOldDriver::new(device_handle, stack).is_connected().await
    }

    async fn connect(
        handle: &SlotHandle,
        stack: &'static SlotStack,
        address: BdAddr,
    ) -> Option<(Connection<'static, SlotPool>, Self::Gatt)> {
        let device_handle = handle.register_device(address);
        let driver = AcaiaOldDriver::new(device_handle, stack);
        match driver.gatt_client().await {
            Ok(pair) => Some(pair),
            Err(e) => {
                log_error!("Failed to create ACAIA GATT client: {:?}", e);
                None
            }
        }
    }

    async fn run_gatt_task(gatt: &Self::Gatt) {
        let _ = gatt.task().await;
    }

    async fn open(gatt: &Self::Gatt) -> Option<Self::Session<'_>> {
        let stream = match gatt.initialize().await {
            Ok(stream) => stream,
            Err(e) => {
                log_error!("Failed to initialize ACAIA scale: {:?}", e);
                return None;
            }
        };

        // The initial heartbeat is what triggers the scale to start streaming; without it
        // the link is up and silent.
        if let Err(e) = gatt.send_heartbeat().await {
            log_error!("Failed to send initial heartbeat: {:?}", e);
        }

        Some(AcaiaOldSession {
            gatt,
            stream,
            last_heartbeat: Instant::now(),
        })
    }
}

impl ScaleSession for AcaiaOldSession<'_> {
    async fn next(&mut self) -> Result<ScaleSample, ()> {
        match self.stream.next().await {
            Ok(AcaiaScaleEvent::Weight(w)) => Ok(ScaleSample {
                weight: Some(w.weight),
                ..Default::default()
            }),
            Err(e) => {
                log_error!("Failed to read ACAIA event: {:?}", e);
                Err(())
            }
        }
    }

    async fn apply(&mut self, op: ScaleOp) {
        let result = match op {
            ScaleOp::Tare => {
                log_info!("Taring ACAIA scale");
                self.gatt.send_tare().await
            }
            ScaleOp::StartTimer => self.gatt.send_timer(AcaiaTimerOp::Start).await,
            ScaleOp::StopTimer => self.gatt.send_timer(AcaiaTimerOp::Stop).await,
            ScaleOp::ResetTimer => self.gatt.send_timer(AcaiaTimerOp::Reset).await,
            // ACAIA has no combined command, so this is two writes. Both go out from this
            // same future, so they cannot interleave with the heartbeat below.
            ScaleOp::TareAndStartTimer => match self.gatt.send_tare().await {
                Ok(()) => self.gatt.send_timer(AcaiaTimerOp::Start).await,
                Err(e) => Err(e),
            },
            // ACAIA has no dose command in either generation. Dropped rather than
            // approximated: there is nothing to approximate it with, and the setting that
            // sends this is offered on every machine precisely because no part of this
            // firmware can tell in advance which scale will be listening.
            ScaleOp::SetDose(_) => {
                log_info!("ACAIA has no dose command; ignoring SetDose");
                Ok(())
            }
        };

        if let Err(e) = result {
            log_error!("Failed to send ACAIA scale op: {:?}", e);
        }
    }

    async fn tick(&mut self) {
        let now = Instant::now();
        if now.duration_since(self.last_heartbeat) >= Duration::from_secs(2) {
            if let Err(e) = self.gatt.send_heartbeat().await {
                log_error!("Failed to send ACAIA heartbeat: {:?}", e);
            } else {
                self.last_heartbeat = now;
            }
        }
    }
}

/// ACAIA's 2021-and-later protocol.
///
/// Shares every outgoing command with [`AcaiaOld`] and differs from it in GATT topology and
/// incoming framing. Flow is derived, as it is for the older generation: no ACAIA scale of
/// either era reports a flow rate.
pub struct AcaiaNew;

/// A connected 2021+ ACAIA scale.
pub struct AcaiaNewSession<'g> {
    gatt: &'g AcaiaNewGattClient<'static, SlotController, SlotPool>,
    stream: AcaiaNewNotificationStream<'g>,
    last_heartbeat: Instant,
    heartbeats_since_identity: u8,
}

/// How often the heartbeat goes out.
///
/// One second, not the two the legacy session uses, and the reason is jitter rather than the
/// deadline itself. [`ScaleSession::tick`] is reached at least once a second even on a
/// silent link, because of the 1 s liveness timer in the loop below -- so the effective
/// interval is this value plus up to a second. At two seconds that is a three-second worst
/// case, which is *exactly* the disconnect deadline reported for the Lunar 2021, with no
/// margin at all. At one second it is two, leaving 750 ms against the 2750 ms figure two
/// other sources give.
///
/// The cost is one seven-byte write per second on a link already carrying roughly fifteen
/// notifications per second.
const ACAIA_HEARTBEAT_INTERVAL: Duration = Duration::from_secs(1);

/// How often the identity frame is re-sent, counted in heartbeats.
///
/// **The least-evidenced constant here.** `ACAIA.md` records identity-before-every-heartbeat
/// as a *Pyxis* quirk, not a requirement of the protocol, and no source says the other 2021+
/// models want it. Once every ten seconds satisfies that note's intent -- the scale
/// periodically re-hears who it is talking to -- at a twentieth of the traffic. If a Pyxis
/// drops the link on a ten-second rhythm, set this to 1, which is the documented regime.
const ACAIA_IDENTITY_EVERY_N_HEARTBEATS: u8 = 10;

impl ScaleProtocol for AcaiaNew {
    type Gatt = AcaiaNewGattClient<'static, SlotController, SlotPool>;
    type Session<'g>
        = AcaiaNewSession<'g>
    where
        Self: 'g;

    const NAME: &'static str = "ACAIA 2021+";
    const NATIVE_FLOW: bool = false;

    async fn is_connected(handle: &SlotHandle, stack: &'static SlotStack, address: BdAddr) -> bool {
        let device_handle = handle.register_device(address);
        AcaiaNewDriver::new(device_handle, stack).is_connected().await
    }

    async fn connect(
        handle: &SlotHandle,
        stack: &'static SlotStack,
        address: BdAddr,
    ) -> Option<(Connection<'static, SlotPool>, Self::Gatt)> {
        let device_handle = handle.register_device(address);
        let driver = AcaiaNewDriver::new(device_handle, stack);
        match driver.gatt_client().await {
            Ok(pair) => Some(pair),
            Err(e) => {
                log_error!("Failed to create ACAIA 2021+ GATT client: {:?}", e);
                None
            }
        }
    }

    async fn run_gatt_task(gatt: &Self::Gatt) {
        let _ = gatt.task().await;
    }

    async fn open(gatt: &Self::Gatt) -> Option<Self::Session<'_>> {
        let stream = match gatt.initialize().await {
            Ok(stream) => stream,
            Err(e) => {
                log_error!("Failed to initialize ACAIA 2021+ scale: {:?}", e);
                return None;
            }
        };

        // The initial heartbeat is what gets the stream moving; without it the link is up
        // and silent, which looks exactly like a failed handshake.
        if let Err(e) = gatt.send_heartbeat().await {
            log_error!("Failed to send initial heartbeat: {:?}", e);
        }

        Some(AcaiaNewSession {
            gatt,
            stream,
            last_heartbeat: Instant::now(),
            heartbeats_since_identity: 0,
        })
    }
}

impl ScaleSession for AcaiaNewSession<'_> {
    async fn next(&mut self) -> Result<ScaleSample, ()> {
        match self.stream.next().await {
            Ok(AcaiaNewEvent::Weight(w)) => Ok(ScaleSample {
                // Already grams: the codec has applied both the decimal factor and the
                // display unit, so a scale set to ounces needs no special handling here --
                // including in `FlowEstimator`, whose zero-weight tare detection would
                // otherwise be comparing against a value in the wrong scale.
                weight: Some(w.grams),
                // NATIVE_FLOW is false, so `publish_sample` ignores this. Setting it would
                // be dead code that reads as though it worked.
                flow: None,
                // Battery arrives in its own frame on this protocol, not with the weight.
                battery: None,
            }),

            // Published on its own, with no weight. `publish_sample` handles that: the
            // weight arm simply does not fire, so the flow estimator is not fed a phantom
            // sample and only the battery endpoint moves.
            Ok(AcaiaNewEvent::Status(s)) => Ok(ScaleSample {
                battery: Some(s.battery_percent as f32),
                ..Default::default()
            }),

            // Decoded so the bytes are understood, logged so their arrival is visible,
            // published nowhere -- the same treatment BooKoo's Ultra-only frames get. The
            // ack is the useful one: it is the only positive evidence the scale is still
            // listening to the heartbeat.
            Ok(other) => {
                log_info!("ACAIA event: {:?}", other);
                Ok(ScaleSample::default())
            }

            // Not `Err(())`. One bad notification on a shared radio is not a dead link, and
            // tearing it down trades a lost sample for a five-second reconnect. This matters
            // more here than anywhere else, because this is the only scale path with a
            // checksum that can actually reject a frame.
            Err(e) => {
                log_error!("Failed to read ACAIA 2021+ event: {:?}", e);
                Ok(ScaleSample::default())
            }
        }
    }

    async fn apply(&mut self, op: ScaleOp) {
        let result = match op {
            ScaleOp::Tare => self.gatt.send_tare().await,
            ScaleOp::StartTimer => self.gatt.send_timer(AcaiaTimerOp::Start).await,
            ScaleOp::StopTimer => self.gatt.send_timer(AcaiaTimerOp::Stop).await,
            ScaleOp::ResetTimer => self.gatt.send_timer(AcaiaTimerOp::Reset).await,
            // Two writes, as on the older generation: ACAIA has no combined command.
            ScaleOp::TareAndStartTimer => match self.gatt.send_tare().await {
                Ok(()) => self.gatt.send_timer(AcaiaTimerOp::Start).await,
                Err(e) => Err(e),
            },
            // No dose command in this generation either -- see the note on the older one.
            ScaleOp::SetDose(_) => {
                log_info!("ACAIA has no dose command; ignoring SetDose");
                Ok(())
            }
        };

        if let Err(e) = result {
            log_error!("Failed to send ACAIA 2021+ scale op: {:?}", e);
        }
    }

    async fn tick(&mut self) {
        let now = Instant::now();
        if now.duration_since(self.last_heartbeat) < ACAIA_HEARTBEAT_INTERVAL {
            return;
        }

        if self.heartbeats_since_identity >= ACAIA_IDENTITY_EVERY_N_HEARTBEATS
            && self.gatt.send_identification().await.is_ok()
        {
            self.heartbeats_since_identity = 0;
        }

        match self.gatt.send_heartbeat().await {
            Ok(()) => {
                // Updated only on success. A failed write must not reset the clock, or a
                // wedged characteristic looks like a healthy cadence right up until the
                // scale drops the link.
                self.last_heartbeat = now;
                self.heartbeats_since_identity =
                    self.heartbeats_since_identity.saturating_add(1);
            }
            Err(e) => log_error!("Failed to send ACAIA 2021+ heartbeat: {:?}", e),
        }
    }
}

/// BooKoo Themis, Themis Mini and Themis Ultra.
///
/// Flow is native, and [`ScaleSession::tick`] is empty: the protocol needs no keepalive.
pub struct Bookoo;

/// A connected BooKoo scale.
pub struct BookooSession<'g> {
    gatt: &'g BookooGattClient<'static, SlotController, SlotPool>,
    stream: BookooNotificationStream<'g>,
}

impl ScaleProtocol for Bookoo {
    type Gatt = BookooGattClient<'static, SlotController, SlotPool>;
    type Session<'g>
        = BookooSession<'g>
    where
        Self: 'g;

    const NAME: &'static str = "BooKoo";
    const NATIVE_FLOW: bool = true;

    async fn is_connected(handle: &SlotHandle, stack: &'static SlotStack, address: BdAddr) -> bool {
        let device_handle = handle.register_device(address);
        BookooDriver::new(device_handle, stack).is_connected().await
    }

    async fn connect(
        handle: &SlotHandle,
        stack: &'static SlotStack,
        address: BdAddr,
    ) -> Option<(Connection<'static, SlotPool>, Self::Gatt)> {
        let device_handle = handle.register_device(address);
        let driver = BookooDriver::new(device_handle, stack);
        match driver.gatt_client().await {
            Ok(pair) => Some(pair),
            Err(e) => {
                log_error!("Failed to create BooKoo GATT client: {:?}", e);
                None
            }
        }
    }

    async fn run_gatt_task(gatt: &Self::Gatt) {
        let _ = gatt.task().await;
    }

    async fn open(gatt: &Self::Gatt) -> Option<Self::Session<'_>> {
        match gatt.initialize().await {
            Ok(stream) => Some(BookooSession { gatt, stream }),
            Err(e) => {
                log_error!("Failed to initialize BooKoo scale: {:?}", e);
                None
            }
        }
    }
}

impl ScaleSession for BookooSession<'_> {
    async fn next(&mut self) -> Result<ScaleSample, ()> {
        match self.stream.next().await {
            Ok(BookooScaleEvent::Weight(f)) => Ok(ScaleSample {
                weight: Some(f.weight_grams),
                flow: Some(f.flow_grams_per_second),
                battery: Some(f.battery_percent as f32),
            }),
            // Ultra-only frames. Decoded so the bytes are understood, logged so their
            // arrival is visible, and not published: powder weight is neither
            // weight-on-the-scale nor flow, and nothing downstream consumes either.
            Ok(BookooScaleEvent::Powder(f)) => {
                log_info!("BooKoo powder weight: {} g", f.powder_grams);
                Ok(ScaleSample::default())
            }
            Ok(BookooScaleEvent::AutoMode(f)) => {
                log_info!("BooKoo auto-mode event: {:?}", f.event);
                Ok(ScaleSample::default())
            }
            Err(e) => {
                // Not `Err(())`. A frame that failed to parse is one bad notification on a
                // shared radio, not a dead link -- the checksum did its job, and the next
                // frame arrives in tens of milliseconds. Tearing the connection down here
                // would trade a lost sample for a five-second reconnect.
                log_error!("Failed to parse BooKoo notification: {:?}", e);
                Ok(ScaleSample::default())
            }
        }
    }

    async fn apply(&mut self, op: ScaleOp) {
        let command = match op {
            ScaleOp::Tare => BookooCommand::Tare,
            ScaleOp::StartTimer => BookooCommand::StartTimer,
            ScaleOp::StopTimer => BookooCommand::StopTimer,
            ScaleOp::ResetTimer => BookooCommand::ResetTimer,
            // One atomic command here, where ACAIA needs two writes.
            ScaleOp::TareAndStartTimer => BookooCommand::TareAndStartTimer,
            // The only protocol in this tree with the command -- and only the Ultra, from
            // firmware V3.2.4b. A Themis or a Mini has no `0x0D` and drops the frame, which
            // is why nothing here waits for anything.
            //
            // A dose outside the protocol's 0.1-999.0 g yields no command at all rather than
            // a clamped one: the scale would show a number nobody chose and never say so.
            ScaleOp::SetDose(grams) => match BookooCommand::set_powder_weight(grams) {
                Some(command) => command,
                None => {
                    log_warn!("BooKoo: a dose of {} g is outside 0.1-999.0 g, not sent", grams);
                    return;
                }
            },
        };

        if let Err(e) = self.gatt.send_command(command).await {
            log_error!("Failed to send BooKoo scale op: {:?}", e);
        }
    }

    /// Empty: BooKoo requires no keepalive, and the scale streams until it is switched off.
    async fn tick(&mut self) {}
}

/// Route one sample's fields to their endpoints.
///
/// Flow comes from exactly one of two places and never both, which is what
/// [`ScaleProtocol::NATIVE_FLOW`] decides. Weight is timestamped here rather than in the
/// driver because this is as close to arrival as the value gets.
///
/// A derived flow of `None` sends nothing at all -- the application processor's watch keeps
/// its last value, which `BluetoothScale` zeroes on disconnect. A tare is deliberately not
/// one of those gaps: [`FlowEstimator::push`] reports zero for a zero weight, so the watch
/// cannot sit on a pre-tare reading through headspace fill.
fn publish_sample<P: ScaleProtocol>(
    publisher: &mut ReadingPublisher,
    flow: &mut FlowEstimator,
    sample: ScaleSample,
) {
    if let Some(weight) = sample.weight {
        publisher.publish(BLUETOOTH_SCALE_ENDPOINT_WEIGHT, weight);

        if !P::NATIVE_FLOW
            && let Some(rate) = flow.push(Instant::now(), weight)
        {
            publisher.publish(BLUETOOTH_SCALE_ENDPOINT_FLOW, rate);
        }
    }

    if P::NATIVE_FLOW
        && let Some(rate) = sample.flow
    {
        publisher.publish(BLUETOOTH_SCALE_ENDPOINT_FLOW, rate);
    }

    if let Some(battery) = sample.battery {
        publisher.publish(BLUETOOTH_SCALE_ENDPOINT_BATTERY, battery);
    }
}
