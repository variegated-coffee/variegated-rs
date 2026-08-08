//! BLE device management and measurement loops

use bt_hci::controller::ExternalController;
use variegated_log::{log_error, log_info};
use embassy_futures::join::join;
use embassy_futures::select::{select, select3, Either, Either3};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Sender;
use embassy_time::{Duration, Instant, Timer};
use esp_radio::ble::controller::BleConnector;
use portable_atomic::Ordering;
use trouble_host::prelude::*;
use variegated_belka_portal_trouble_driver::BelkaPortalDriver;
use variegated_controller_types::ExternalPeripheralSensorReading;
use variegated_controller_types::debug::DebugEvent;
use crate::debug::bus;
use variegated_scale_trouble_driver::acaia_old::{AcaiaOldDriver, ScaleEvent};
use variegated_trouble_connection_manager::BleConnectionManager;

use crate::ble::status;
use crate::channels::{
    BLE_RECONNECT_REQUEST, SCALE_COMMAND_CHANNEL,
    SENSOR_READING_CAPACITY,
};
use variegated_controller_types::ScaleOp;
use crate::config::{
    BELKA_PERIPHERAL_ID, BLUETOOTH_GROUP_1_SCALE_PERIPHERAL_ID, BLUETOOTH_SCALE_ENDPOINT_FLOW,
    BLUETOOTH_SCALE_ENDPOINT_WEIGHT,
};
use variegated_adc_tools::{ConversionParameters, KalmanFilterParameters};

/// Slot serving the Belka portal while the peripheral set is still compiled in.
///
/// Temporary. These two constants disappear when the measurement loops start taking
/// their slot as an argument; until then they keep the reporting path -- which is now
/// driven by the slot table -- describing the same two peripherals it always did.
const BELKA_SLOT: usize = 0;
/// Slot serving the group 1 scale. See [`BELKA_SLOT`].
const SCALE_SLOT: usize = 1;

/// Set the Belka connection state.
///
/// The edge detection that used to live here now lives in `ble::status`, so that every
/// caller is edge triggered by construction rather than by remembering to be -- see
/// [`crate::ble::status::set_slot_connected`] for why emitting on every call would turn
/// the debug ring over on its own.
fn set_belka_connected(connected: bool) {
    status::set_slot_connected(BELKA_SLOT, connected);
}

/// Set the group 1 scale connection state.
///
/// Called from where the *Belka* loop sets its own -- after the GATT client exists --
/// rather than from the earlier link-layer `is_connected()` poll. The two differ during a
/// link that connects but never completes service discovery, and reporting a scale as
/// connected in that window would be a worse lie than reporting it disconnected: the
/// application processor gates brew-by-weight on this.
fn set_scale_connected(connected: bool) {
    status::set_slot_connected(SCALE_SLOT, connected);
}

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
/// Lives on this processor rather than the application processor for two reasons. The UART
/// hop and the application processor's scheduling both add latency *between* samples, and
/// differentiation is precisely the operation that turns jitter in sample timing into
/// error in the result. And other Bluetooth scales report flow computed in the scale
/// itself, so the application processor should receive flow as a measurement whoever
/// produced it, rather than knowing that one particular scale needs it synthesised.
///
/// The output is mass flow, g/s. The application processor's `FlowRateType` is nominally
/// ml/s; under the 1 g/ml assumption the rest of the codebase already makes for coffee
/// (see `dual_boiler_single_group`'s output-volume derivation) they are interchangeable,
/// and no conversion is applied.
struct FlowEstimator {
    samples: heapless::Deque<(Instant, f32), FLOW_WINDOW_SAMPLES>,
    filter: ConversionParameters,
}

impl FlowEstimator {
    fn new() -> Self {
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
    fn push(&mut self, now: Instant, weight: f32) -> Option<f32> {
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
            return None;
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

/// BLE devices management task
///
/// Manages connections to Belka Portal and ACAIA scale, running their measurement loops.
#[embassy_executor::task]
pub async fn ble_devices_task(
    manager: &'static BleConnectionManager<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
    stack: &'static Stack<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
    belka_address: BdAddr,
    acaia_address: BdAddr,
    sensor_sender: Sender<'static, CriticalSectionRawMutex, ExternalPeripheralSensorReading, SENSOR_READING_CAPACITY>,
) {
    let handle = manager.handle();

    // Claim the two slots the compiled-in peripherals occupy, so the status map, the
    // debug snapshot and `ReconnectBle` all describe them. This is what the reconciler
    // will do instead once the association list drives the peripheral set.
    status::set_slot_peripheral(BELKA_SLOT, Some(BELKA_PERIPHERAL_ID));
    status::set_slot_peripheral(SCALE_SLOT, Some(BLUETOOTH_GROUP_1_SCALE_PERIPHERAL_ID));

    // Register both devices and enable auto-connection
    {
        // Register Belka Portal
        let device_handle = handle.register_device(belka_address);
        let driver = BelkaPortalDriver::new(device_handle, stack);
        driver.set_maintain_connection(true).await;

        // Register ACAIA scale
        let device_handle = handle.register_device(acaia_address);
        let driver = AcaiaOldDriver::new(device_handle, stack);
        driver.set_maintain_connection(true).await;
    }

    log_info!("BLE Devices: Configured Belka {:?} and ACAIA {:?}", belka_address, acaia_address);

    // Run connection manager, the device measurement loop, and the injected-command
    // listener concurrently.
    join(
        manager.run(),
        join(
            join(
                belka_measurement_loop(handle.clone(), stack, belka_address, sensor_sender),
                acaia_measurement_loop(handle.clone(), stack, acaia_address, sensor_sender),
            ),
            reconnect_request_loop(handle.clone(), belka_address),
        ),
    )
    .await;
}

/// Serve `CommsDebugOp::ReconnectBle`.
///
/// Lives here rather than in `debug::commands` because the connection manager's
/// handles are not `'static` -- they borrow the manager, which this task owns -- so
/// there is nowhere else on this processor that can reach a `Connection`.
///
/// The reconnect is performed by *disconnecting*: the manager's maintenance loop
/// notices a device whose connection is no longer alive, clears it, waits out its
/// two-second cooldown and connects again. Asking it to connect directly would not
/// work anyway, since it refuses a device it already believes is connected -- and
/// "already connected" is precisely the state an operator reaches for this command
/// from, when the link is nominally up and behaving badly.
///
/// Ids this firmware does not know are rejected by the dispatcher, before the signal,
/// so anything arriving here is one this loop can act on. It is still matched rather
/// than assumed: the two lists are in different modules and are allowed to drift.
async fn reconnect_request_loop(
    handle: variegated_trouble_connection_manager::ManagerHandle<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
    belka_address: BdAddr,
) {
    loop {
        let id = BLE_RECONNECT_REQUEST.wait().await;
        if id != BELKA_PERIPHERAL_ID {
            continue;
        }

        let device_handle = handle.register_device(belka_address);
        // `with_connection` borrows the manager's `RefCell` for the length of the
        // closure only, and the closure is synchronous, so this cannot overlap the
        // manager's own `borrow_mut` across an await.
        match device_handle.with_connection(|connection| connection.disconnect()) {
            Ok(()) => {
                log_info!("Reconnect requested for Belka Portal; dropping the connection");
                // The manager sees the dead connection on its next pass and emits the
                // `BlePeripheralDisconnected`/`BlePeripheralConnected` pair through
                // `set_belka_connected`, so this site deliberately emits nothing: the
                // events that follow describe what actually happened, and one here
                // would announce an outcome that has not been reached yet.
            }
            // Not connected. Nothing to drop, and the manager is already retrying once
            // a second, so the request is satisfied by what is already happening.
            Err(()) => log_info!("Reconnect requested for Belka Portal, which is not connected"),
        }
    }
}

/// Belka Portal measurement loop
async fn belka_measurement_loop(
    handle: variegated_trouble_connection_manager::ManagerHandle<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
    stack: &'static Stack<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
    belka_address: BdAddr,
    sensor_sender: Sender<'static, CriticalSectionRawMutex, ExternalPeripheralSensorReading, SENSOR_READING_CAPACITY>,
) {
    Timer::after(Duration::from_millis(300)).await;
    loop {
        // Check if connected
        let is_connected = {
            let device_handle = handle.register_device(belka_address);
            let driver = BelkaPortalDriver::new(device_handle, stack);
            driver.is_connected().await
        };

        if !is_connected {
            //info!("Belka Portal not connected, waiting...");
            Timer::after(Duration::from_secs(1)).await;
            continue;
        }

        log_info!("Belka Portal connected, creating GATT client...");

        // Create GATT client and subscribe
        let result = {
            let device_handle = handle.register_device(belka_address);
            let driver = BelkaPortalDriver::new(device_handle, stack);
            driver.gatt_client().await
        };

        match result {
            Ok((_conn, gatt)) => {
                log_info!("GATT client created, running task...");

                // Signal that Belka is connected
                set_belka_connected(true);

                // Run GATT client task alongside operations, exit when either completes
                let _ = select(gatt.task(), async {
                    log_info!("Let's first read measurements...");
                    let r = gatt.read_measurements().await;
                    log_info!("Measurements: {:?}", r);
                    log_info!("Then subscribe...");

                    match gatt.subscribe().await {
                        Ok(mut stream) => {
                            log_info!("Successfully subscribed to Belka Portal measurements");
                            loop {
                                // Race between getting next measurement and checking connection status
                                match select(
                                    stream.next(),
                                    async {
                                        Timer::after(Duration::from_secs(1)).await;
                                        let device_handle = handle.register_device(belka_address);
                                        let driver = BelkaPortalDriver::new(device_handle, stack);
                                        driver.is_connected().await
                                    }
                                ).await {
                                    Either::First(result) => {
                                        match result {
                                            Ok(measurement) => {
                                               /* log_info!(
                                                    "Portal Measurement: EC={}, Temp={} °C, Battery={}",
                                                    measurement.ec,
                                                    measurement.temperature,
                                                    measurement.battery
                                                ); */

                                                // Send EC reading (endpoint 0)
                                                let ec_reading = ExternalPeripheralSensorReading {
                                                    id: BELKA_PERIPHERAL_ID,
                                                    endpoint: 0,
                                                    value: measurement.ec,
                                                };
                                                sensor_sender.send(ec_reading).await;

                                                // Send temperature reading (endpoint 1)
                                                let temp_reading = ExternalPeripheralSensorReading {
                                                    id: BELKA_PERIPHERAL_ID,
                                                    endpoint: 1,
                                                    value: measurement.temperature,
                                                };
                                                sensor_sender.send(temp_reading).await;

                                                // Send battery reading (endpoint 2)
                                                let battery_reading = ExternalPeripheralSensorReading {
                                                    id: BELKA_PERIPHERAL_ID,
                                                    endpoint: 2,
                                                    value: measurement.battery as f32,
                                                };
                                                sensor_sender.send(battery_reading).await;
                                            }
                                            Err(e) => {
                                                log_error!("Failed to read measurement: {:?}", e);
                                                break;
                                            }
                                        }
                                    }
                                    Either::Second(is_connected) => {
                                        if !is_connected {
                                            log_info!("Connection lost during measurements, exiting");
                                            set_belka_connected(false);
                                            break;
                                        }
                                    }
                                }
                            }
                        }
                        Err(e) => {
                            log_error!("Failed to subscribe to measurements: {:?}", e);
                        }
                    }
                }).await;
                log_info!("GATT join completed, connection dropped");

                // Signal disconnection
                set_belka_connected(false);
            }
            Err(e) => {
                log_error!("Failed to create GATT client: {:?}", e);
                set_belka_connected(false);
            }
        }

        // Wait before retrying
        log_info!("Restarting Belka measurement loop...");
        Timer::after(Duration::from_secs(5)).await;
    }
}

/// ACAIA scale measurement loop
///
/// Reports under [`BLUETOOTH_GROUP_1_SCALE_PERIPHERAL_ID`] rather than an ACAIA-specific
/// id: the id names the scale's role on the machine, so swapping in a different make of
/// scale here does not move the readings to a different address on the application
/// processor. See the note in `config.rs`.
async fn acaia_measurement_loop(
    handle: variegated_trouble_connection_manager::ManagerHandle<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
    stack: &'static Stack<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
    acaia_address: BdAddr,
    sensor_sender: Sender<'static, CriticalSectionRawMutex, ExternalPeripheralSensorReading, SENSOR_READING_CAPACITY>,
) {
    // Held for the whole loop, not taken per connection. `PubSubChannel` hands out a
    // fixed number of subscriber slots and only returns them on drop, so acquiring one
    // inside the reconnect loop would leak a slot on every retry -- and this loop retries
    // every five seconds for as long as the scale is switched off.
    let mut scale_commands = SCALE_COMMAND_CHANNEL
        .subscriber()
        .expect("scale command subscriber slots are sized for every scale loop");

    loop {
        // Check if connected
        let is_connected = {
            let device_handle = handle.register_device(acaia_address);
            let driver = AcaiaOldDriver::new(device_handle, stack);
            driver.is_connected().await
        };

        if !is_connected {
            //info!("ACAIA scale not connected, waiting...");
            set_scale_connected(false);
            Timer::after(Duration::from_secs(1)).await;
            continue;
        }

        log_info!("ACAIA scale connected, creating GATT client...");

        // Create GATT client
        let result = {
            let device_handle = handle.register_device(acaia_address);
            let driver = AcaiaOldDriver::new(device_handle, stack);
            driver.gatt_client().await
        };

        match result {
            Ok((_conn, gatt)) => {
                log_info!("ACAIA GATT client created");

                // Run GATT client task alongside operations
                let _ = select(gatt.task(), async {
                    // Initialize scale (subscribe + handshake in correct order)
                    log_info!("Initializing ACAIA scale...");
                    match gatt.initialize().await {
                        Ok(mut stream) => {
                            log_info!("ACAIA scale initialized successfully");
                            set_scale_connected(true);

                            // Discard any scale op that arrived while the scale was down.
                            //
                            // The subscriber queues rather than latching, but the hazard
                            // is the same one the `Signal` had: a tare asked for during a
                            // disconnect would fire the moment the link came back --
                            // possibly minutes later, and possibly mid-shot. A stale tare
                            // is worse than a dropped one, because the operator who asked
                            // has long since moved on and zeroing a scale under a running
                            // extraction corrupts it.
                            //
                            // Draining in a loop, where the `Signal` needed one `reset()`:
                            // the queue can hold more than one entry.
                            while scale_commands.try_next_message_pure().is_some() {}

                            // Send initial heartbeat to trigger data flow
                            log_info!("Sending initial heartbeat");
                            if let Err(e) = gatt.send_heartbeat().await {
                                log_error!("Failed to send initial heartbeat: {:?}", e);
                            }

                            let mut last_heartbeat = Instant::now();
                            // Edge-triggers the "channel full" log below. A persistently
                            // full channel would otherwise log at the notification rate,
                            // which is the same ~12.5 Hz flood the drop exists to avoid.
                            let mut dropping_weights = false;
                            // Declared inside the connected scope, so a reconnect starts
                            // with no history rather than differencing the first new
                            // sample against a weight from before the link dropped.
                            let mut flow = FlowEstimator::new();

                            loop {
                                // Race between: next event, periodic timer, and a tare.
                                //
                                // `select3` polls in declaration order, so weights keep
                                // priority over a tare -- which is right: the tare is a
                                // single write and can wait a notification, while a
                                // dropped weight is a gap in a control signal.
                                //
                                // `send_tare` takes `&self` and `stream` borrows `&gatt`
                                // too, so both are shared borrows and this needs no
                                // restructuring. It is also a single characteristic
                                // write on the path `send_heartbeat` already uses, so it
                                // cannot stall the loop long enough to miss the ~3 s
                                // heartbeat deadline that keeps the scale connected.
                                match select3(
                                    stream.next(),
                                    Timer::after(Duration::from_secs(1)),
                                    scale_commands.next_message_pure(),
                                ).await {
                                    Either3::First(result) => {
                                        match result {
                                            Ok(event) => {
                                                match event {
                                                    ScaleEvent::Weight(w) => {
                                                        log_info!("Scale Weight: {} g", w.weight);

                                                        // `try_send`, where the Belka loop awaits.
                                                        //
                                                        // The difference is the sample rate and what a
                                                        // late sample is worth. Belka notifies about once
                                                        // a second and its three readings are a slow
                                                        // trend, so blocking for a slot is free and never
                                                        // happens. A scale notifies ten to twenty times a
                                                        // second, and the 16-slot channel is drained by a
                                                        // single `select4` loop that also serialises status
                                                        // and writes the UART -- so it *can* back up.
                                                        //
                                                        // Awaiting there would be actively harmful, and not
                                                        // only because the sample is stale by the time it
                                                        // lands: `send_heartbeat` below runs in this same
                                                        // loop body, so a blocked send stops the heartbeat,
                                                        // and the scale drops the link within seconds. That
                                                        // would turn transient UART backpressure into a BLE
                                                        // disconnect and a five-second reconnect cycle.
                                                        //
                                                        // Dropping is the right failure: the next weight
                                                        // arrives ~80 ms later and supersedes this one, so a
                                                        // full channel costs one sample rather than the
                                                        // connection.
                                                        let weight_reading = ExternalPeripheralSensorReading {
                                                            id: BLUETOOTH_GROUP_1_SCALE_PERIPHERAL_ID,
                                                            endpoint: BLUETOOTH_SCALE_ENDPOINT_WEIGHT,
                                                            value: w.weight,
                                                        };
                                                        match sensor_sender.try_send(weight_reading) {
                                                            Ok(()) => {
                                                                if dropping_weights {
                                                                    dropping_weights = false;
                                                                    log_info!("Sensor channel drained, forwarding scale weights again");
                                                                }
                                                            }
                                                            Err(_) => {
                                                                if !dropping_weights {
                                                                    dropping_weights = true;
                                                                    log_error!("Sensor channel full, dropping scale weights");
                                                                }
                                                            }
                                                        }

                                                        // Flow is derived from the same sample, timestamped
                                                        // here rather than in the driver because this is as
                                                        // close to arrival as the value gets.
                                                        //
                                                        // `None` while the estimator warms up or straddles a
                                                        // tare, and in that case nothing is sent at all --
                                                        // the application processor's watch keeps its last
                                                        // value, which `BluetoothScale` zeroes on disconnect.
                                                        //
                                                        // Shares the weight's drop-rather-than-block
                                                        // discipline, but deliberately not its logging: two
                                                        // edge-triggered flags for one channel would both
                                                        // fire on the same congestion and say the same thing
                                                        // twice. The weight flag already reports it.
                                                        if let Some(flow_rate) = flow.push(Instant::now(), w.weight) {
                                                            let flow_reading = ExternalPeripheralSensorReading {
                                                                id: BLUETOOTH_GROUP_1_SCALE_PERIPHERAL_ID,
                                                                endpoint: BLUETOOTH_SCALE_ENDPOINT_FLOW,
                                                                value: flow_rate,
                                                            };
                                                            let _ = sensor_sender.try_send(flow_reading);
                                                        }
                                                    }
                                                }
                                            }
                                            Err(e) => {
                                                log_error!("Failed to read ACAIA event: {:?}", e);
                                                break;
                                            }
                                        }
                                    }
                                    Either3::Second(_) => {
                                        // Check connection
                                        let device_handle = handle.register_device(acaia_address);
                                        let driver = AcaiaOldDriver::new(device_handle, stack);
                                        let is_connected = driver.is_connected().await;

                                        if !is_connected {
                                            log_info!("ACAIA connection lost during measurements, exiting");
                                            break;
                                        }
                                    }
                                    Either3::Third((peripheral_id, op)) => {
                                        // The id is checked, not assumed. Nothing has
                                        // validated it upstream -- unlike
                                        // `BLE_RECONNECT_REQUEST`, it arrives off the
                                        // UART from the other processor rather than from
                                        // this firmware's own dispatcher -- and every
                                        // scale loop now sees every op, because the
                                        // channel broadcasts to all subscribers. An
                                        // unchecked tare would zero every scale on the
                                        // machine.
                                        //
                                        // An `if`, not an early `continue`: the
                                        // heartbeat check at the bottom of this loop
                                        // body is what keeps the link alive, and a
                                        // `continue` here would skip it.
                                        if peripheral_id == BLUETOOTH_GROUP_1_SCALE_PERIPHERAL_ID {
                                            match op {
                                                ScaleOp::Tare => {
                                                    log_info!("Taring ACAIA scale");
                                                    if let Err(e) = gatt.send_tare().await {
                                                        log_error!("Failed to send ACAIA tare: {:?}", e);
                                                    }
                                                }
                                            }
                                        } else {
                                            log_info!(
                                                "Ignoring scale op for 0x{:04X}, this loop owns 0x{:04X}",
                                                peripheral_id,
                                                BLUETOOTH_GROUP_1_SCALE_PERIPHERAL_ID
                                            );
                                        }
                                    }
                                }

                                // Send heartbeat if 2 seconds have passed
                                let now = Instant::now();
                                if now.duration_since(last_heartbeat) >= Duration::from_secs(2) {
                                    if let Err(e) = gatt.send_heartbeat().await {
                                        log_error!("Failed to send ACAIA heartbeat: {:?}", e);
                                    } else {
                                        last_heartbeat = now;
                                    }
                                }
                            }
                        }
                        Err(e) => {
                            log_error!("Failed to initialize ACAIA scale: {:?}", e);
                        }
                    }
                }).await;
                log_info!("ACAIA GATT task completed, connection dropped");
                set_scale_connected(false);
            }
            Err(e) => {
                log_error!("Failed to create ACAIA GATT client: {:?}", e);
                set_scale_connected(false);
            }
        }

        // Wait before retrying
        log_info!("Restarting ACAIA measurement loop...");
        Timer::after(Duration::from_secs(5)).await;
    }
}
