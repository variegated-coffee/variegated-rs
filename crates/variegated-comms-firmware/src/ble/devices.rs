//! BLE device management and measurement loops

use bt_hci::controller::ExternalController;
use variegated_log::{log_error, log_info};
use embassy_futures::join::join;
use embassy_futures::select::{select, Either};
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

use crate::channels::{BELKA_CONNECTION_STATUS, SENSOR_READING_CAPACITY};
use crate::config::BELKA_PERIPHERAL_ID;

/// Set the Belka connection flag, emitting a typed event only when it actually
/// changes.
///
/// The five `BELKA_CONNECTION_STATUS.store(..)` sites this replaces are not all
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

    // Register both devices and enable auto-connection
    {
        // Register Belka Portal
        let device_handle = handle.register_device(belka_address);
        let driver = BelkaPortalDriver::new(device_handle, stack);
        driver.set_maintain_connection(true).await;

/*        // Register ACAIA scale
        let device_handle = handle.register_device(acaia_address);
        let driver = AcaiaOldDriver::new(device_handle, stack);
        driver.set_maintain_connection(true).await; */
    }

    log_info!("BLE Devices: Configured Belka {:?} and ACAIA {:?}", belka_address, acaia_address);

    // Run connection manager and both device measurement loops concurrently
    join(
        manager.run(),
//        join(
            belka_measurement_loop(handle.clone(), stack, belka_address, sensor_sender),
            //acaia_measurement_loop(handle, stack, acaia_address),
//        ),
    )
    .await;
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
async fn acaia_measurement_loop(
    handle: variegated_trouble_connection_manager::ManagerHandle<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
    stack: &'static Stack<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
    acaia_address: BdAddr,
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

                            // Send initial heartbeat to trigger data flow
                            log_info!("Sending initial heartbeat");
                            if let Err(e) = gatt.send_heartbeat().await {
                                log_error!("Failed to send initial heartbeat: {:?}", e);
                            }

                            let mut last_heartbeat = Instant::now();

                            loop {
                                // Race between: getting next event and periodic timer
                                match select(
                                    stream.next(),
                                    Timer::after(Duration::from_secs(1))
                                ).await {
                                    Either::First(result) => {
                                        match result {
                                            Ok(event) => {
                                                match event {
                                                    ScaleEvent::Weight(w) => {
                                                        log_info!("Scale Weight: {} g", w.weight);
                                                    }
                                                }
                                            }
                                            Err(e) => {
                                                log_error!("Failed to read ACAIA event: {:?}", e);
                                                break;
                                            }
                                        }
                                    }
                                    Either::Second(_) => {
                                        // Check connection
                                        let device_handle = handle.register_device(acaia_address);
                                        let driver = AcaiaOldDriver::new(device_handle, stack);
                                        let is_connected = driver.is_connected().await;

                                        if !is_connected {
                                            log_info!("ACAIA connection lost during measurements, exiting");
                                            break;
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
            }
            Err(e) => {
                log_error!("Failed to create ACAIA GATT client: {:?}", e);
            }
        }

        // Wait before retrying
        log_info!("Restarting ACAIA measurement loop...");
        Timer::after(Duration::from_secs(5)).await;
    }
}
