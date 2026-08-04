//! WiFi connection management tasks

use variegated_log::{log_error, log_info};
use embassy_net::Runner as NetRunner;
use embassy_time::{Duration, Timer};
use embassy_futures::select::{select3, Either3};
use esp_radio::wifi::{Config as WifiConfig, Interface, WifiController, sta::StationConfig};

use portable_atomic::Ordering;

use variegated_controller_types::debug::DebugEvent;

use crate::channels::{NO_RSSI, WIFI_CONNECTED, WIFI_RECONNECT_REQUEST, WIFI_RSSI_DBM, WIFI_RSSI_SIGNAL};
use crate::config::{PASSWORD, SSID};
use crate::debug::bus;

/// Set the Wi-Fi connection flag, emitting a typed event only when it actually
/// changes.
///
/// The same shape as `ble::devices::set_belka_connected`, and for the same reason:
/// the edge belongs in the helper rather than at the call site, so no caller -- and
/// no future caller -- can emit a level.
///
/// It matters more here than it looks. Of the three sites that clear this flag,
/// only one is unambiguously a transition (`wait_for_disconnect_async` resolving);
/// the other two run at the top of a retry loop and can execute with the flag
/// already `false`. And the two directions can genuinely get out of step:
/// `connect_async` can return `Err` on an association that then completes, after
/// which `is_connected()` is true while this flag is false, and a later disconnect
/// would report a loss that was never announced as a gain. Gating both directions
/// on the `swap` keeps every `WifiAssociated`/`WifiLost` a matched pair, which is
/// what a host counts. The unpaired truth is in `CommsState::wifi_connected`, once
/// a second, where a level belongs.
fn set_wifi_connected(connected: bool) {
    if WIFI_CONNECTED.swap(connected, Ordering::Relaxed) == connected {
        return;
    }
    bus::emit_event(if connected {
        DebugEvent::WifiAssociated
    } else {
        DebugEvent::WifiLost
    });
}

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
    log_info!("Starting WiFi connection task");

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
                // The reconnect request is the last arm, so it can never displace an
                // actual disconnect notification or an RSSI sample that was ready at
                // the same instant.
                match select3(
                    controller.wait_for_disconnect_async(),
                    Timer::after(Duration::from_secs(1)),
                    WIFI_RECONNECT_REQUEST.wait(),
                ).await {
                    Either3::First(_) => {
                        // Disconnected - clear RSSI and break to reconnect.
                        //
                        // Edge triggered: `wait_for_disconnect_async()` resolving is
                        // the link-loss transition itself -- it is a wait, not a
                        // poll -- and this arm is only reachable from the branch
                        // that was associated. The `swap` inside the helper is belt
                        // and braces.
                        set_wifi_connected(false);
                        WIFI_RSSI_SIGNAL.signal(None);
                        WIFI_RSSI_DBM.store(NO_RSSI, Ordering::Relaxed);
                        Timer::after(Duration::from_millis(5000)).await;
                        break;
                    }
                    Either3::Second(_) => {
                        // Timer fired - update RSSI (convert i32 to i8)
                        let rssi = controller.rssi().ok().map(|r| r as i8);
                        WIFI_RSSI_SIGNAL.signal(rssi);
                        WIFI_RSSI_DBM.store(
                            rssi.map(|r| r as i16).unwrap_or(NO_RSSI),
                            Ordering::Relaxed,
                        );
                    }
                    // A debug host asked for a reconnect. This is the *only* site that
                    // emits `WifiReconnectRequested`, and it emits it here -- at the
                    // point the link is actually about to be torn down and rebuilt --
                    // rather than where the request was raised. The event then means
                    // something a host can act on: the association it is watching is
                    // going away on purpose. See the comment further down for why the
                    // retry loop below deliberately does not emit it.
                    //
                    // Edge triggered by construction: `Signal::wait` consumes the
                    // value, and a request raised while the link is already down is
                    // drained in the `else` branch below by the reconnect that is
                    // already in progress.
                    Either3::Third(()) => {
                        bus::emit_event(DebugEvent::WifiReconnectRequested);
                        // Explicit rather than relying on the AP to drop us: this is
                        // what makes the request do something on a link that is
                        // working but wrong (associated to the wrong band, or holding
                        // a stale DHCP lease). The error is logged rather than
                        // propagated -- if the disconnect failed we fall through to
                        // `connect_async` anyway, which is where a genuinely broken
                        // controller will show up.
                        if controller.disconnect_async().await.is_err() {
                            log_error!("Requested WiFi disconnect failed");
                        }
                        set_wifi_connected(false);
                        WIFI_RSSI_SIGNAL.signal(None);
                        WIFI_RSSI_DBM.store(NO_RSSI, Ordering::Relaxed);
                        break;
                    }
                }
            }
        } else {
            // Not connected - clear RSSI. Reached at boot and on every retry, so the
            // helper's `swap` is doing real work here: it emits only if the flag was
            // actually set, which covers the case where the controller drops an
            // association before the inner loop ever gets to wait on it.
            set_wifi_connected(false);
            WIFI_RSSI_SIGNAL.signal(None);
            WIFI_RSSI_DBM.store(NO_RSSI, Ordering::Relaxed);
            // A reconnect request raised while the link was already down is satisfied
            // by the `connect_async` a few lines below, so it is taken here rather
            // than left to fire the instant the next association succeeds and tear it
            // straight back down.
            let _ = WIFI_RECONNECT_REQUEST.try_take();
        }

        // Deliberately *not* promoted to `DebugEvent::WifiReconnectRequested`, which
        // is emitted only by the injected-request arm above -- the one place where a
        // reconnect was actually asked for rather than merely retried.
        //
        // This is the body of an unbounded retry loop: with the AP unreachable,
        // `connect_async` fails and we are back here five seconds later, forever.
        // A typed event bypasses the log suppressor, so promoting it would put one
        // frame every 5 s into a 16-slot ring for as long as the network is down --
        // evicting exactly the events someone would be looking at. As text the
        // suppressor collapses repeats, and the condition is already carried
        // losslessly by `CommsState::wifi_connected` in the 1 Hz snapshot.
        //
        // Guarding it to fire only on the first attempt after a loss would not help
        // either: that instant is already reported by `WifiLost` above.
        log_info!("Connecting to WiFi...");
        match controller.connect_async().await {
            Ok(_) => {
                // Edge triggered: `connect_async` waits for the association rather
                // than polling, so failed attempts fall to the `Err` arm and emit
                // nothing. The helper's `swap` closes the window between this
                // returning `Ok` and the `is_connected()` check at the top of the
                // loop.
                set_wifi_connected(true);
            }
            Err(_e) => {
                log_error!("Failed to connect to WiFi");
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
