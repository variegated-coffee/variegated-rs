//! BLE device management and measurement loops

use bt_hci::controller::ExternalController;
use variegated_log::{log_error, log_info};
use embassy_futures::join::join;
use embassy_futures::select::{select, Either};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Sender;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Timer};
use esp_radio::ble::controller::BleConnector;
use trouble_host::prelude::*;
use variegated_belka_portal_trouble_driver::BelkaPortalDriver;
use variegated_controller_types::ExternalPeripheralSensorReading;
use variegated_controller_types::debug::DebugEvent;
use crate::debug::bus;
use variegated_trouble_connection_manager::BleConnectionManager;

use crate::ble::scale_slot::{run_scale_slot, AcaiaOld, Bookoo};

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
use variegated_controller_types::PeripheralId;
use variegated_log::log_warn;

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

    // One row for four joined loops. Wrapping the arms individually would need four slots
    // and this table has one spare, so it stays coarse until something makes a case for
    // spending them.
    variegated_checkin::watch(
        crate::checkin::MONITOR.claim(crate::checkin::CheckinId::BleDevices),
        join(
            manager.run(scanner),
            join(
                reconcile_associations_loop(),
                join(reconnect_request_loop(handle.clone()), scan_request_loop(manager)),
            ),
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
/// -- a scale loop carries a `FlowEstimator` with a sample deque, a median window and a
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

    // Indexed by the peripheral slot this instance was given. Four instances of this task
    // run at once (`pool_size`), so a shared row would break the one-writer-per-slot
    // contract and report whichever scale happened to write last.
    let checkin = crate::checkin::ble_slot(slot);

    loop {
        checkin.good();

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
            //
            // Heartbeating *this* wait would be sound on its own -- parked-for-an-assignment
            // is the healthy state and there is nothing else here to be stuck in -- but it
            // would buy nothing, because the same slot's other state (below) cannot be
            // heartbeated honestly. One declared period has to describe both, and a period
            // that fits the idle case makes every connected scale render red.
            assignments.changed_and(|all| all[slot] != current).await;
            continue;
        };

        // Boxed, and the history matters because the obvious reading of it is wrong.
        //
        // This future is about 10 kB -- the ACAIA arm dominates, carrying a
        // `FlowEstimator` with a sample deque, a median window and a Kalman filter. Left
        // inline it is part of the task future, which `pool_size` allocates
        // `MAX_BLUETOOTH_PERIPHERALS` times in `.bss` whether or not anything is
        // associated; boxed, it costs one allocation per *connected* peripheral and hands
        // 41 kB back to `.stack`, which is the SRAM remainder.
        //
        // Boxing it the first time panicked the machine with
        // `memory allocation of 12000 bytes failed`. That was not a shortage of heap --
        // it was **contiguity**. `esp_alloc` gives each region its own
        // `linked_list_allocator::Heap` and an allocation must fit contiguously inside
        // one region, and the ESPHome entity table was asking for a single 12000-byte
        // run. A 10 kB block landing in the same region, alongside the constant churn of
        // `to_allocvec_cobs` on the UART path, could leave that region with ample free
        // space and nowhere to put it.
        //
        // The entity table is now built in `.bss` (`esphome/entity_builder.rs`), so the
        // heap no longer has a large contiguous consumer to be fragmented away from. What
        // is left here is a handful of ~10 kB blocks against 112 kB across two regions,
        // with a measured peak occupancy of 28 kB before any of this.
        //
        // One allocation per *assignment*, not per reconnect: the retry loops live inside
        // the measurement loops, and the `select` below only completes when this slot's
        // assignment changes. So this runs when a user associates or removes a
        // peripheral -- long-lived, few and large, which is the least fragmenting shape
        // for this allocator.
        let driver = alloc::boxed::Box::pin(async {
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
                    run_scale_slot::<AcaiaOld>(
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
                BluetoothDriverKind::Bookoo => {
                    run_scale_slot::<Bookoo>(
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
        });

        // What the note above asserts, measured rather than assumed.
        //
        // The whole memory argument for a third driver arm rests on rustc overlapping the
        // locals of mutually exclusive `match` arms in a coroutine, so that this box stays
        // `max(belka, acaia, bookoo)` rather than becoming their sum. That is a claim about
        // a layout optimisation, not a guarantee, and the "about 10 kB" figure it is
        // compared against had no measurement behind it either.
        //
        // One line, logged once per assignment rather than per reconnect, is enough to
        // settle both: associate an ACAIA and then a BooKoo and compare. Roughly equal
        // numbers mean the overlap held; roughly the sum means it did not, and this design
        // needs revisiting before it ships -- which is the one question here that can panic
        // the machine, given the `memory allocation of 12000 bytes failed` above.
        log_info!(
            "BLE slot {} driver future: {} bytes",
            slot,
            core::mem::size_of_val(&*driver)
        );

        // `changed_and` marks a value as seen only when the predicate holds, so a change
        // that reassigns a *different* slot re-parks this one instead of cancelling a
        // working driver. Without that, editing any association would interrupt every
        // peripheral on the machine.
        // **Deliberately not heartbeated.** This `select` is where the task spends nearly all
        // its life -- it completes only when the assignment changes -- so a timer beside it
        // would be the only thing keeping this row fresh, and it would keep firing whether or
        // not the driver inside was making progress. A driver deadlocked on a notification
        // that never arrives would render green forever.
        //
        // A row that says "never aged" is honest about knowing nothing. A row that says
        // "checked in 1 s ago" because a timer fired next to a wedged driver is worse than
        // both a red row and a stale one. The fix is a check-in *inside* the driver loops --
        // `belka_measurement_loop` and the scale drivers already have their own cadences to
        // hang one on -- not a timer out here. Until then this slot declares no period.
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
