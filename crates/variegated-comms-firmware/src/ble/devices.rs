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

use crate::channels::{
    BELKA_CONNECTION_STATUS, BLE_RECONNECT_REQUEST, SCALE_CONNECTION_STATUS, SCALE_TARE_REQUEST,
    SENSOR_READING_CAPACITY,
};
use crate::config::{BELKA_PERIPHERAL_ID, BLUETOOTH_GROUP_1_SCALE_PERIPHERAL_ID};

/// Set the Belka connection flag, emitting a typed event only when it actually
/// changes.
///
/// The four `BELKA_CONNECTION_STATUS.store(..)` sites this replaces are not all
/// transitions. Three of them -- the GATT-client failure arm, the join-completed
/// arm and the connection-lost arm -- sit in a five-second retry loop, and two of
/// those can run with the flag already `false`. Storing unconditionally is
/// harmless; *emitting* unconditionally would not be, because `emit_event`
/// bypasses the log suppressor: a peripheral that is powered off but advertising,
/// or one whose GATT connect keeps failing, would put a `BlePeripheralDisconnected`
/// into a 16-slot ring every five seconds indefinitely.
///
/// `swap` makes the edge the condition rather than the call site, so every caller
/// is edge triggered by construction and no future caller can reintroduce the
/// problem. Steady state -- connected or disconnected -- is silent, and the level
/// is carried by `CommsState::ble_connected` in the 1 Hz snapshot.
fn set_belka_connected(connected: bool) {
    if BELKA_CONNECTION_STATUS.swap(connected, Ordering::Relaxed) == connected {
        return;
    }
    bus::emit_event(if connected {
        DebugEvent::BlePeripheralConnected { id: BELKA_PERIPHERAL_ID }
    } else {
        DebugEvent::BlePeripheralDisconnected { id: BELKA_PERIPHERAL_ID }
    });
}

/// Set the group 1 scale connection flag, emitting a typed event only on a change.
///
/// A copy of [`set_belka_connected`], and the `swap`-as-edge-detector above is load
/// bearing for the same reason: the scale's measurement loop clears this on four
/// separate paths, three of which sit inside a five-second retry cycle and can run with
/// the flag already `false`. `emit_event` bypasses the log suppressor, so an
/// unconditional emit would push a `BlePeripheralDisconnected` into a 16-slot ring every
/// five seconds for as long as the scale is off its charger and out of range.
///
/// It is set from where the *Belka* loop sets its own -- after the GATT client exists --
/// rather than from the earlier link-layer `is_connected()` poll. The two differ during
/// a link that connects but never completes service discovery, and reporting a scale as
/// connected in that window would be a worse lie than reporting it disconnected: the
/// application processor gates brew-by-weight on this.
fn set_scale_connected(connected: bool) {
    if SCALE_CONNECTION_STATUS.swap(connected, Ordering::Relaxed) == connected {
        return;
    }
    bus::emit_event(if connected {
        DebugEvent::BlePeripheralConnected { id: BLUETOOTH_GROUP_1_SCALE_PERIPHERAL_ID }
    } else {
        DebugEvent::BlePeripheralDisconnected { id: BLUETOOTH_GROUP_1_SCALE_PERIPHERAL_ID }
    });
}

/// BLE devices management task
///
/// Manages connections to Belka Portal and ACAIA scale, running their measurement loops.
///
/// Belka is currently commented out while the ACAIA scale driver is being brought up.
#[embassy_executor::task]
pub async fn ble_devices_task(
    manager: &'static BleConnectionManager<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
    stack: &'static Stack<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
    belka_address: BdAddr,
    acaia_address: BdAddr,
    sensor_sender: Sender<'static, CriticalSectionRawMutex, ExternalPeripheralSensorReading, SENSOR_READING_CAPACITY>,
) {
    let handle = manager.handle();

    // Register both devices and enable auto-connection
    {
/*        // Register Belka Portal
        let device_handle = handle.register_device(belka_address);
        let driver = BelkaPortalDriver::new(device_handle, stack);
        driver.set_maintain_connection(true).await; */

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
//        join(
            join(
                //belka_measurement_loop(handle.clone(), stack, belka_address, sensor_sender),
                acaia_measurement_loop(handle.clone(), stack, acaia_address, sensor_sender),
                reconnect_request_loop(handle.clone(), belka_address),
            ),
//        ),
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
#[allow(dead_code)] // Temporarily unwired; see `ble_devices_task`.
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

                            // Discard any tare that arrived while the scale was down.
                            //
                            // `Signal` latches, so without this a tare asked for during
                            // a disconnect would fire the moment the link came back --
                            // possibly minutes later, and possibly mid-shot. A stale
                            // tare is worse than a dropped one: the operator who asked
                            // has long since moved on, and zeroing a scale under a
                            // running extraction corrupts it.
                            SCALE_TARE_REQUEST.reset();

                            // Send initial heartbeat to trigger data flow
                            log_info!("Sending initial heartbeat");
                            if let Err(e) = gatt.send_heartbeat().await {
                                log_error!("Failed to send initial heartbeat: {:?}", e);
                            }

                            let mut last_heartbeat = Instant::now();
                            // Edge-triggers the "channel full" log below. A persistently
                            // full channel would otherwise log at the notification rate,
                            // which is the same 10-20 Hz flood the drop exists to avoid.
                            let mut dropping_weights = false;

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
                                    SCALE_TARE_REQUEST.wait(),
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
                                                        // arrives in 50-100 ms and supersedes this one, so a
                                                        // full channel costs one sample rather than the
                                                        // connection.
                                                        let weight_reading = ExternalPeripheralSensorReading {
                                                            id: BLUETOOTH_GROUP_1_SCALE_PERIPHERAL_ID,
                                                            endpoint: 0,
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
                                    Either3::Third(peripheral_id) => {
                                        // The id is checked, not assumed. Nothing has
                                        // validated it upstream -- unlike
                                        // `BLE_RECONNECT_REQUEST`, it arrives off the
                                        // UART from the other processor rather than from
                                        // this firmware's own dispatcher -- and this loop
                                        // owns exactly one of the three scale roles
                                        // `config.rs` names. Today only group 1 exists,
                                        // so this never rejects; it is written now
                                        // because the day a second scale is added is the
                                        // day an unchecked tare would zero the wrong one.
                                        //
                                        // An `if`, not an early `continue`: the
                                        // heartbeat check at the bottom of this loop
                                        // body is what keeps the link alive, and a
                                        // `continue` here would skip it.
                                        if peripheral_id == BLUETOOTH_GROUP_1_SCALE_PERIPHERAL_ID {
                                            log_info!("Taring ACAIA scale");
                                            if let Err(e) = gatt.send_tare().await {
                                                log_error!("Failed to send ACAIA tare: {:?}", e);
                                            }
                                        } else {
                                            log_info!(
                                                "Ignoring tare for scale 0x{:04X}, this loop owns 0x{:04X}",
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
