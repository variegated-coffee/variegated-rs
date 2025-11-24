//! WiFi connection management tasks

use defmt::{error, info};
use embassy_net::Runner as NetRunner;
use embassy_time::{Duration, Timer};
use embassy_futures::select::{select, Either};
use esp_radio::wifi::{ClientConfig, ModeConfig, WifiController, WifiDevice, WifiEvent, WifiStaState};

use crate::channels::WIFI_RSSI_SIGNAL;
use crate::config::{PASSWORD, SSID};

/// WiFi connection management task
///
/// Maintains WiFi connection, reconnecting when disconnected.
#[embassy_executor::task]
pub async fn connection_task(mut controller: WifiController<'static>) {
    info!("Starting WiFi connection task");
    loop {
        match esp_radio::wifi::sta_state() {
            WifiStaState::Connected => {
                // While connected, periodically update RSSI and wait for disconnect
                loop {
                    match select(
                        controller.wait_for_event(WifiEvent::StaDisconnected),
                        Timer::after(Duration::from_secs(1)),
                    ).await {
                        Either::First(_) => {
                            // Disconnected - clear RSSI and break to reconnect
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
            }
            _ => {
                // Not connected - clear RSSI
                WIFI_RSSI_SIGNAL.signal(None);
            }
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = ModeConfig::Client(
                ClientConfig::default()
                    .with_ssid(SSID.into())
                    .with_password(PASSWORD.into()),
            );
            controller.set_config(&client_config).unwrap();
            info!("Starting WiFi controller...");
            controller.start_async().await.unwrap();
        }

        info!("Connecting to WiFi...");
        match controller.connect_async().await {
            Ok(_) => info!("WiFi connected!"),
            Err(_e) => {
                error!("Failed to connect to WiFi");
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

/// Network stack runner task
#[embassy_executor::task]
pub async fn net_task(mut runner: NetRunner<'static, WifiDevice<'static>>) {
    runner.run().await
}
