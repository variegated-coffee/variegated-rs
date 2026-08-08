//! BLE device management and measurement loops

use bt_hci::controller::ExternalController;
use variegated_log::{log_error, log_info};
use embassy_futures::join::join;
use embassy_futures::select::{select, select3, Either, Either3};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Sender;
use embassy_sync::pubsub::Subscriber;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Instant, Timer};
use esp_radio::ble::controller::BleConnector;
use trouble_host::prelude::*;
use variegated_belka_portal_trouble_driver::BelkaPortalDriver;
use variegated_controller_types::ExternalPeripheralSensorReading;
use variegated_controller_types::debug::DebugEvent;
use crate::debug::bus;
use variegated_scale_trouble_driver::acaia_old::{AcaiaOldDriver, ScaleEvent};
use variegated_trouble_connection_manager::BleConnectionManager;

use crate::ble::scanner::ScanPrinter;
use crate::ble::status;
use crate::channels::{
    BLE_RECONNECT_REQUEST, BLE_SCAN_REQUEST, BT_ASSOCIATIONS, SCALE_COMMAND_CHANNEL,
    SENSOR_READING_CAPACITY,
};
use variegated_trouble_connection_manager::ScanRequest;
use variegated_controller_types::bluetooth::{
    reconcile_bluetooth_slots, BluetoothDriverKind, BluetoothSlotAssignment,
    BluetoothSlotAssignments, MAX_BLUETOOTH_PERIPHERALS,
};
use variegated_controller_types::{PeripheralId, ScaleOp};
use variegated_log::log_warn;
use crate::config::{BLUETOOTH_SCALE_ENDPOINT_FLOW, BLUETOOTH_SCALE_ENDPOINT_WEIGHT};
use variegated_adc_tools::{ConversionParameters, KalmanFilterParameters};

/// One entry per slot: the peripheral it should be serving, or `None`.
///
/// Written only by [`reconcile_associations_loop`], read by the slot tasks. Separate
/// from `BT_ASSOCIATIONS` because the two answer different questions -- that one is what
/// the user configured, this is which of four fixed workers is responsible for what --
/// and because the mapping has to be *stable*: see
/// [`reconcile_bluetooth_slots`] for why deriving it from list position instead would
/// drop every peripheral's link whenever one was deleted.
static SLOT_ASSIGNMENTS: Watch<
    CriticalSectionRawMutex,
    BluetoothSlotAssignments,
    MAX_BLUETOOTH_PERIPHERALS,
> = Watch::new();

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
/// Runs the connection manager alongside the two loops that have no slot of their own:
/// the reconciler that decides which slot serves what, and the reconnect listener.
/// The per-peripheral work happens in [`ble_slot_task`], spawned once per slot.
#[embassy_executor::task]
pub async fn ble_devices_task(
    manager: &'static BleConnectionManager<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
    scanner: &'static ScanPrinter,
) {
    let handle = manager.handle();

    join(
        manager.run(scanner),
        join(
            reconcile_associations_loop(),
            join(reconnect_request_loop(handle.clone()), scan_request_loop(manager)),
        ),
    )
    .await;
}

/// Forward scan requests from the application processor to the connection manager.
///
/// A hop rather than a direct call because the two ends cannot reach each other: the
/// request arrives in the UART reader, which holds no manager and must not await, while
/// the manager is owned by this task. `Signal::signal` bridges them without either
/// blocking.
async fn scan_request_loop(
    manager: &'static BleConnectionManager<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
) {
    loop {
        let duration_ms = BLE_SCAN_REQUEST.wait().await;
        log_info!("Bluetooth scan requested for {} ms", duration_ms);
        manager.request_scan(ScanRequest {
            duration: Duration::from_millis(duration_ms as u64),
            // Active, because a passive scan sees advertisements only, and most scales
            // put their name in the scan response. A pick-list of bare addresses is not
            // one a human can use.
            active: true,
        });
    }
}

/// Translate the association list into slot assignments.
///
/// **The sole writer of [`SLOT_ASSIGNMENTS`].** Everything about which worker serves
/// which peripheral is decided here, so the slot tasks never have to agree with each
/// other about anything.
///
/// The decision itself is [`reconcile_bluetooth_slots`], which lives in
/// `variegated-controller-types` because it is pure and because its two failure modes --
/// claims keyed on list position, and two associations sharing an address -- are silent
/// and worth testing. This loop is the part that cannot be: waiting, logging, and
/// publishing.
async fn reconcile_associations_loop() {
    let mut associations = BT_ASSOCIATIONS
        .receiver()
        .expect("the association watch is sized for this receiver");
    let sender = SLOT_ASSIGNMENTS.sender();
    let mut assignments = BluetoothSlotAssignments::default();

    loop {
        let list = associations.changed().await;

        let changed = reconcile_bluetooth_slots(&mut assignments, &list, |id, why| {
            // Logged rather than swallowed: a peripheral that is configured and simply
            // never connects is the hardest kind of fault to find from the outside.
            log_warn!("Bluetooth association 0x{:04X} refused: {:?}", id, why);
            bus::emit_event(DebugEvent::CommandRejected {
                reason: variegated_controller_types::debug::name("bt_association"),
            });
        });

        if changed {
            for (slot, assignment) in assignments.iter().enumerate() {
                match assignment {
                    Some(a) => log_info!("BLE slot {} serves 0x{:04X} ({:?})", slot, a.id, a.driver),
                    None => log_info!("BLE slot {} is idle", slot),
                }
            }
            sender.send(assignments);
        }
    }
}

/// One BLE peripheral, whichever one this slot is currently assigned.
///
/// Four of these are spawned at boot and live forever, picking up and putting down
/// peripherals as the association list changes. A task per *slot* rather than per
/// peripheral because a task cannot be unspawned: an association the user deletes has to
/// leave something behind, and an idle worker is a much smaller thing to leave than a
/// leaked one.
///
/// Everything BLE here is `!Send` -- `RefCell` in the manager, and `Connection` is not
/// `Send` -- so these must be spawned onto the same executor as the manager. That is not
/// enforced by a bound; `Spawner::spawn` simply will not accept them anywhere else.
///
/// # Where the memory goes
///
/// `pool_size` allocates this task's future `MAX_BLUETOOTH_PERIPHERALS` times in `.bss`,
/// whether or not any peripheral is associated, and the driver future inside dominates it
/// -- the ACAIA loop carries a `FlowEstimator` with a sample deque, a median window and a
/// Kalman filter. Four copies cost about 42 kB, taken out of `.stack`, which is the SRAM
/// remainder.
///
/// That is the price of this being a fixed pool, and it is paid deliberately: see the
/// note at the driver future below for what happened when it was moved to the heap
/// instead. **`MAX_BLUETOOTH_PERIPHERALS` is the lever here** -- each slot is ~10 kB of
/// `.bss` whether or not it ever serves anything.
#[embassy_executor::task(pool_size = MAX_BLUETOOTH_PERIPHERALS)]
pub async fn ble_slot_task(
    slot: usize,
    manager: &'static BleConnectionManager<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
    stack: &'static Stack<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
    sensor_sender: Sender<'static, CriticalSectionRawMutex, ExternalPeripheralSensorReading, SENSOR_READING_CAPACITY>,
) {
    let handle = manager.handle();
    let mut assignments = SLOT_ASSIGNMENTS
        .receiver()
        .expect("the slot assignment watch is sized for one receiver per slot");

    // Acquired once, for the life of the task rather than the life of an assignment.
    // `PubSubChannel` returns subscriber slots only on drop, so taking one per
    // assignment would leak one every time a peripheral was re-associated.
    let mut scale_commands = SCALE_COMMAND_CHANNEL
        .subscriber()
        .expect("the scale command channel is sized for one subscriber per slot");

    let mut current: Option<BluetoothSlotAssignment> = None;

    loop {
        // Read the authoritative value rather than relying on having been notified.
        // Change notifications are what wake this loop, but they are not what it trusts:
        // re-reading here means a slot that missed one -- because it was busy tearing a
        // connection down -- corrects itself on the next pass instead of serving a
        // peripheral nobody asked for.
        let desired = SLOT_ASSIGNMENTS.try_get().and_then(|all| all[slot]);

        if current != desired {
            if let Some(previous) = current {
                log_info!("BLE slot {}: releasing 0x{:04X}", slot, previous.id);
                // Order matters. `release` disconnects and frees the manager's table
                // entry; `set_slot_peripheral(_, None)` is what emits the disconnect for
                // the outgoing peripheral, which nothing else will now that this slot's
                // driver loop is gone.
                handle.register_device(BdAddr::new(previous.address)).release();
                status::set_slot_peripheral(slot, None);
            }

            current = desired;

            if let Some(next) = current {
                log_info!(
                    "BLE slot {}: taking 0x{:04X} ({:?})",
                    slot,
                    next.id,
                    next.driver
                );
                status::set_slot_peripheral(slot, Some(next.id));
                handle
                    .register_device(BdAddr::new(next.address))
                    .set_maintain_connection(true)
                    .await;
            }
        }

        let Some(assignment) = current else {
            // Idle. Nothing to run, so park until this slot is given something.
            assignments.changed_and(|all| all[slot] != current).await;
            continue;
        };

        // Inline, in `.bss`, and **not** on the heap. This was boxed once; it panicked
        // the machine, and the reason is worth stating so nobody boxes it again.
        //
        // This future is about 10 kB -- the ACAIA arm dominates, carrying a
        // `FlowEstimator` with a sample deque, a median window and a Kalman filter. On
        // the heap that would be ~10 kB per *connected* peripheral rather than
        // `MAX_BLUETOOTH_PERIPHERALS` times in `.bss`, which is genuinely cheaper on a
        // machine with one scale, and it bought 41688 bytes of `.stack`.
        //
        // What it cost was **contiguity**, which is the property that actually matters
        // here. `esp_alloc` gives each region its own `linked_list_allocator::Heap` and
        // tries them in registration order, so an allocation must fit contiguously
        // *within a single region*. The ESPHome server needs one 12000-byte block for its
        // entity table (`esphome/server.rs`), which it leaks and holds forever. A 10 kB
        // block dropped into a region, alongside the constant small churn of
        // `to_allocvec_cobs` on the UART path, can leave that region with ample free space
        // and no 12000-byte run left in it -- and then the request fails outright and
        // `handle_alloc_error` takes the machine down. Observed exactly that, at 52 s
        // uptime, with `memory allocation of 12000 bytes failed`.
        //
        // Growing the heap does not fix that: more total space is not more contiguous
        // space, and the two large permanent consumers are almost the same size, so they
        // fragment each other whichever order they arrive in.
        //
        // `.bss` cannot fail. That is the whole argument for paying for it up front.
        let driver = async {
            match assignment.driver {
                BluetoothDriverKind::BelkaPortal => {
                    belka_measurement_loop(
                        handle.clone(),
                        stack,
                        BdAddr::new(assignment.address),
                        assignment.id,
                        slot,
                        sensor_sender,
                    )
                    .await
                }
                BluetoothDriverKind::AcaiaOld => {
                    acaia_measurement_loop(
                        handle.clone(),
                        stack,
                        BdAddr::new(assignment.address),
                        assignment.id,
                        slot,
                        sensor_sender,
                        &mut scale_commands,
                    )
                    .await
                }
            }
        };

        // `changed_and` marks a value as seen only when the predicate holds, so a change
        // that reassigns a *different* slot re-parks this one instead of cancelling a
        // working driver. Without that, editing any association would interrupt every
        // peripheral on the machine.
        select(driver, assignments.changed_and(|all| all[slot] != current)).await;
    }
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
/// Ids this firmware is not serving are rejected by the dispatcher, before the signal,
/// so anything arriving here is one this loop can act on. The address is still looked up
/// rather than assumed: the dispatcher tests assignment, this needs the address, and the
/// association list can change between the two.
async fn reconnect_request_loop(
    handle: variegated_trouble_connection_manager::ManagerHandle<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
) {
    loop {
        let id = BLE_RECONNECT_REQUEST.wait().await;

        // Resolved from the current assignments rather than from a compiled-in address,
        // and the lookup can legitimately fail: the peripheral may have been
        // disassociated in the moment between the dispatcher accepting the command and
        // this loop being woken.
        let Some(assignment) = SLOT_ASSIGNMENTS
            .try_get()
            .and_then(|all| all.iter().flatten().find(|a| a.id == id).copied())
        else {
            log_info!("Reconnect requested for 0x{:04X}, which is no longer assigned", id);
            continue;
        };

        let device_handle = handle.register_device(BdAddr::new(assignment.address));
        // `with_connection` borrows the manager's `RefCell` for the length of the
        // closure only, and the closure is synchronous, so this cannot overlap the
        // manager's own `borrow_mut` across an await.
        match device_handle.with_connection(|connection| connection.disconnect()) {
            Ok(()) => {
                log_info!("Reconnect requested for 0x{:04X}; dropping the connection", id);
                // The manager sees the dead connection on its next pass and the slot's
                // measurement loop emits the `BlePeripheralDisconnected`/
                // `BlePeripheralConnected` pair, so this site deliberately emits nothing:
                // the events that follow describe what actually happened, and one here
                // would announce an outcome that has not been reached yet.
            }
            // Not connected. Nothing to drop, and the manager is already retrying once
            // a second, so the request is satisfied by what is already happening.
            Err(()) => log_info!("Reconnect requested for 0x{:04X}, which is not connected", id),
        }
    }
}

/// Belka Portal measurement loop
///
/// Reports under whatever [`PeripheralId`] the association gave it, rather than a
/// compiled-in one: the id names the *role* the device fills on this machine, and the
/// application processor routes on it.
async fn belka_measurement_loop(
    handle: variegated_trouble_connection_manager::ManagerHandle<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
    stack: &'static Stack<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
    belka_address: BdAddr,
    peripheral_id: PeripheralId,
    slot: usize,
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
                status::set_slot_connected(slot, true);

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
                                                    id: peripheral_id,
                                                    endpoint: 0,
                                                    value: measurement.ec,
                                                };
                                                sensor_sender.send(ec_reading).await;

                                                // Send temperature reading (endpoint 1)
                                                let temp_reading = ExternalPeripheralSensorReading {
                                                    id: peripheral_id,
                                                    endpoint: 1,
                                                    value: measurement.temperature,
                                                };
                                                sensor_sender.send(temp_reading).await;

                                                // Send battery reading (endpoint 2)
                                                let battery_reading = ExternalPeripheralSensorReading {
                                                    id: peripheral_id,
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
                                            status::set_slot_connected(slot, false);
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
                status::set_slot_connected(slot, false);
            }
            Err(e) => {
                log_error!("Failed to create GATT client: {:?}", e);
                status::set_slot_connected(slot, false);
            }
        }

        // Wait before retrying
        log_info!("Restarting Belka measurement loop...");
        Timer::after(Duration::from_secs(5)).await;
    }
}

/// ACAIA scale measurement loop
///
/// Reports under whatever [`PeripheralId`] the association gave it rather than an
/// ACAIA-specific id: the id names the scale's *role* on the machine, so swapping in a
/// different make of scale does not move the readings to a different address on the
/// application processor. See the note in `config.rs`.
///
/// The subscriber is borrowed rather than taken, because it belongs to the slot and
/// outlives any one assignment -- see [`ble_slot_task`].
async fn acaia_measurement_loop(
    handle: variegated_trouble_connection_manager::ManagerHandle<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
    stack: &'static Stack<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
    acaia_address: BdAddr,
    peripheral_id: PeripheralId,
    slot: usize,
    sensor_sender: Sender<'static, CriticalSectionRawMutex, ExternalPeripheralSensorReading, SENSOR_READING_CAPACITY>,
    scale_commands: &mut Subscriber<'static, CriticalSectionRawMutex, (PeripheralId, ScaleOp), 1, MAX_BLUETOOTH_PERIPHERALS, 1>,
) {
    loop {
        // Check if connected
        let is_connected = {
            let device_handle = handle.register_device(acaia_address);
            let driver = AcaiaOldDriver::new(device_handle, stack);
            driver.is_connected().await
        };

        if !is_connected {
            //info!("ACAIA scale not connected, waiting...");
            status::set_slot_connected(slot, false);
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
                            status::set_slot_connected(slot, true);

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
                                                            id: peripheral_id,
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
                                                                id: peripheral_id,
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
                                    // `target` rather than `peripheral_id`: this loop now
                                    // has an id of its own, and shadowing it here would
                                    // make the comparison below compare a thing to
                                    // itself.
                                    Either3::Third((target, op)) => {
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
                                        if target == peripheral_id {
                                            match op {
                                                ScaleOp::Tare => {
                                                    log_info!("Taring scale 0x{:04X}", peripheral_id);
                                                    if let Err(e) = gatt.send_tare().await {
                                                        log_error!("Failed to send ACAIA tare: {:?}", e);
                                                    }
                                                }
                                            }
                                        } else {
                                            log_info!(
                                                "Ignoring scale op for 0x{:04X}, this loop owns 0x{:04X}",
                                                target,
                                                peripheral_id
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
                status::set_slot_connected(slot, false);
            }
            Err(e) => {
                log_error!("Failed to create ACAIA GATT client: {:?}", e);
                status::set_slot_connected(slot, false);
            }
        }

        // Wait before retrying
        log_info!("Restarting ACAIA measurement loop...");
        Timer::after(Duration::from_secs(5)).await;
    }
}
