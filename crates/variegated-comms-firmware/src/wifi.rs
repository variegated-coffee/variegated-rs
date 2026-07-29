//! WiFi connection management tasks

use defmt::{error, info};
use embassy_net::Runner as NetRunner;
use embassy_time::{Duration, Timer};
use embassy_futures::select::{select, Either};
use esp_radio::wifi::{Config as WifiConfig, Interface, WifiController, sta::StationConfig};

use portable_atomic::Ordering;

use crate::channels::{WIFI_CONNECTED, WIFI_RSSI_SIGNAL};
use crate::config::{PASSWORD, SSID};

/// WiFi connection management task
///
/// Maintains WiFi connection, reconnecting when disconnected.
///
/// esp-radio 0.18 reshaped this quite a bit. The controller is now configured
/// and started at construction (see `bin/main.rs`), so there is no
/// `start_async`/`is_started` to drive here -- `set_config` starts it and
/// dropping it stops it. `sta_state()`/`WifiStaState` are gone in favour of
/// `is_connected()`, which is now a plain `bool`, and the generic
/// `wait_for_event(WifiEvent::StaDisconnected)` became
/// `wait_for_disconnect_async()`.
#[embassy_executor::task]
pub async fn connection_task(mut controller: WifiController<'static>) {
    info!("Starting WiFi connection task");

    // 0.18 removed `start_async`/`is_started`: `set_config` configures *and*
    // starts the controller, and dropping it stops it. So this happens once,
    // up front, instead of being re-checked every iteration.
    //
    // `Ssid` now implements `From<&str>`, so SSID no longer needs `.into()`.
    let station_config = WifiConfig::Station(
        StationConfig::default()
            .with_ssid(SSID)
            .with_password(PASSWORD.into()),
    );
    controller.set_config(&station_config).unwrap();

    loop {
        if controller.is_connected() {
            // While connected, periodically update RSSI and wait for disconnect.
            loop {
                match select(
                    controller.wait_for_disconnect_async(),
                    Timer::after(Duration::from_secs(1)),
                ).await {
                    Either::First(_) => {
                        // Disconnected - clear RSSI and break to reconnect
                        WIFI_CONNECTED.store(false, Ordering::Relaxed);
                        WIFI_RSSI_SIGNAL.signal(None);
                        Timer::after(Duration::from_millis(5000)).await;
                        break;
                    }
                    Either::Second(_) => {
                        // Timer fired - update RSSI (convert i32 to i8)
                        let rssi = controller.rssi().ok().map(|r| r as i8);
                        WIFI_RSSI_SIGNAL.signal(rssi);
                    }
                }
            }
        } else {
            // Not connected - clear RSSI
            WIFI_CONNECTED.store(false, Ordering::Relaxed);
            WIFI_RSSI_SIGNAL.signal(None);
        }

        info!("Connecting to WiFi...");
        match controller.connect_async().await {
            Ok(_) => {
                WIFI_CONNECTED.store(true, Ordering::Relaxed);
                info!("WiFi connected!");
            }
            Err(_e) => {
                error!("Failed to connect to WiFi");
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

/// Network stack runner task
///
/// `WifiDevice` was renamed `Interface` in esp-radio 0.18.
#[embassy_executor::task]
pub async fn net_task(mut runner: NetRunner<'static, Interface<'static>>) {
    runner.run().await
}
