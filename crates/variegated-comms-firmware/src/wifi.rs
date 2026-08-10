//! WiFi connection management tasks

use variegated_log::{log_error, log_info};
use embassy_net::Runner as NetRunner;
use embassy_time::{with_timeout, Duration, Timer};
use embassy_futures::select::{select, select4, Either, Either4};
use esp_radio::wifi::{
    AuthenticationMethod, Config as WifiConfig, Interface, WifiController, scan::ScanConfig,
    sta::StationConfig,
};

use portable_atomic::Ordering;

use variegated_controller_types::debug::DebugEvent;

use crate::channels::{
    NO_RSSI, WIFI_CANDIDATE, WIFI_CANDIDATE_RESULT, WIFI_CONNECTED, WIFI_CREDENTIALS,
    WIFI_RECONNECT_REQUEST, WIFI_RSSI_DBM, WIFI_RSSI_SIGNAL, WIFI_SCAN_REQUEST, WIFI_SCAN_RESULT,
};
use crate::debug::bus;
use variegated_controller_types::wifi::WifiCredentials;
use variegated_improv_trouble::handler::{self, Network};

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
    if connected {
        bus::emit_event(DebugEvent::WifiAssociated);
    } else {
        // Counted here rather than at the call sites for the same reason the event is:
        // this is the one place that knows a *transition* happened. Two of the three
        // sites that clear the flag run at the top of a retry loop and can execute with
        // it already false, so counting there would inflate the rate by however long the
        // network stayed down. Incrementing beside the event also keeps the two
        // consistent by construction -- the counter is the same fact as `WifiLost`, in
        // the form that survives the ring turning over.
        crate::instrumentation::note_wifi_disconnect();
        bus::emit_event(DebugEvent::WifiLost);
    }
}

/// Point the station at a network, and re-assert the power-saving setting.
///
/// Called once when credentials first arrive and again whenever they change. Both halves
/// have to happen together and **in this order** -- see the note on the power-saving call.
///
/// esp-radio 0.18 removed `start_async`/`is_started`: `set_config` configures *and* starts
/// the controller, and dropping it stops it. Calling it again on a running station
/// reconfigures without restarting, which is what makes a credential change cheap.
///
/// `Ssid` implements `From<&str>`, so the SSID needs no conversion; the password does.
fn apply_configuration(controller: &mut WifiController<'static>, credentials: &WifiCredentials) {
    let station_config = WifiConfig::Station(
        StationConfig::default()
            .with_ssid(credentials.ssid.as_str())
            .with_password(credentials.password.as_str().into()),
    );
    // Not `.unwrap()`, unlike the compiled-in configuration this replaced. That one could
    // only fail on a programming error; this one carries a string that arrived over a wire
    // from another processor, and panicking the processor that owns Wi-Fi, BLE and the
    // ESPHome server over a malformed credential would turn a bad provisioning attempt into
    // a reboot loop.
    if let Err(e) = controller.set_config(&station_config) {
        log_error!("Failed to apply Wi-Fi configuration: {:?}", e);
        return;
    }

    // Re-apply after the station is started, because applying it before does nothing.
    //
    // esp-radio already sets this in `wifi::new` -- `set_power_saving(None)` with a
    // comment that the blob default is not the best for bandwidth -- but it does so
    // *before* its own `set_config`, and that call is what starts the station: mode goes
    // NULL -> STA, so the `previous_mode != mode` branch runs `esp_wifi_start()`.
    // `esp_wifi_set_ps` is driver state that the start re-applies from its own default
    // of `WIFI_PS_MIN_MODEM`, so the setting is overwritten before it ever takes effect.
    // Here the mode is already STA, so `set_config` above does not restart anything and
    // this sticks.
    //
    // The symptom was not a power measurement, it was that unicast stopped arriving. A
    // station in modem sleep only listens around DTIM beacons, so the AP buffers unicast
    // for it -- and this one was not collecting, so those frames aged out. Broadcast is
    // flooded to every station regardless and kept arriving the whole time, which is why
    // the device looked perfectly healthy from the inside: `net_probe` showed embassy-net
    // polling every ~10 ms and taking ~40 packets a second while a `curl` to `/` spent
    // ten seconds retransmitting SYNs that never landed. Replies were never the problem,
    // because transmitting does not require being awake to listen.
    if let Err(e) = controller.set_power_saving(esp_radio::wifi::PowerSaveMode::None) {
        log_error!("failed to disable Wi-Fi power saving: {:?}", e);
    }
}

/// How long a candidate credential gets to associate before it is called a failure.
///
/// Thirty seconds is a long time to hold a phone, and it is chosen against the alternative
/// rather than against the user's patience: a WPA handshake behind a busy access point can
/// take ten, and reporting `UnableToConnect` for a password that was in fact correct sends
/// someone off to retype something that was never wrong.
const CANDIDATE_TIMEOUT: Duration = Duration::from_secs(30);

/// Something Improv wants the radio for.
enum RadioRequest {
    Candidate(WifiCredentials),
    Scan,
}

/// Resolve when Improv raises either request.
///
/// One future so it can be dropped as a unit by each `select` below. Dropping a pending
/// `Signal::wait` is safe: it leaves a stale waker that the next `signal()` fires harmlessly,
/// and it is this same task's waker either way.
async fn radio_request() -> RadioRequest {
    match select(WIFI_CANDIDATE.wait(), WIFI_SCAN_REQUEST.wait()).await {
        Either::First(candidate) => RadioRequest::Candidate(candidate),
        Either::Second(()) => RadioRequest::Scan,
    }
}

/// Try a candidate credential, and put the radio back where it was if it fails.
///
/// Returns the credential to treat as current from here on -- the candidate if it associated,
/// the previous one otherwise. **The caller has to adopt it.** The application processor
/// pushes these same credentials back down the link once it has persisted them, and a
/// `current` still holding the old value would read that push as a change and tear down the
/// association the user is at that moment being told succeeded.
async fn try_candidate(
    controller: &mut WifiController<'static>,
    candidate: WifiCredentials,
    previous: &WifiCredentials,
) -> WifiCredentials {
    // Nothing on this path logs the credential, not even the SSID. `WifiCredentials`' `Format`
    // elides the password, but the SSID alone is enough to make a log line worth not writing
    // on a path that runs while someone is provisioning and may be sharing a screen.
    log_info!("Trying candidate Wi-Fi credentials");

    if controller.disconnect_async().await.is_err() {
        log_error!("Disconnect before trying a candidate failed");
    }
    set_wifi_connected(false);
    WIFI_RSSI_SIGNAL.signal(None);
    WIFI_RSSI_DBM.store(NO_RSSI, Ordering::Relaxed);

    apply_configuration(controller, &candidate);

    let associated = matches!(
        with_timeout(CANDIDATE_TIMEOUT, controller.connect_async()).await,
        Ok(Ok(_))
    );

    if associated {
        set_wifi_connected(true);
        WIFI_CANDIDATE_RESULT.signal(true);
        log_info!("Candidate Wi-Fi credentials associated");
        candidate
    } else {
        // The working network goes back before the answer goes out. Answering first would
        // race the Improv task into reporting a credential that is about to be discarded --
        // and more to the point, a failed provisioning attempt must not cost a machine the
        // network it already had.
        log_error!("Candidate Wi-Fi credentials failed to associate");
        let _ = controller.disconnect_async().await;
        apply_configuration(controller, previous);
        WIFI_CANDIDATE_RESULT.signal(false);
        previous.clone()
    }
}

/// Run a scan and publish what it found.
///
/// **No disconnect first, deliberately.** A scan briefly leaves the home channel and the
/// association usually survives it; dropping a working network in order to enumerate the
/// networks beside it would be the worse trade. The only caller asks during a provisioning
/// window, with a user standing at the machine.
async fn run_scan(controller: &mut WifiController<'static>) {
    let config = ScanConfig::default().with_max(handler::MAX_SCAN_RESULTS);
    let networks = match controller.scan_async(&config).await {
        Ok(found) => found
            .into_iter()
            .map(|ap| Network {
                // `Ssid::as_str` truncates at the first invalid byte rather than failing, and
                // `try_from` cannot overflow here -- both sides are 32.
                ssid: heapless::String::try_from(ap.ssid.as_str()).unwrap_or_default(),
                rssi: ap.signal_strength,
                // `None` means the beacon did not say. Treated as "needs a password", which
                // is the safe way to be wrong: an unnecessary prompt is a nuisance, a missing
                // one is an attempt that cannot succeed and says nothing about why.
                requires_password: !matches!(ap.auth_method, Some(AuthenticationMethod::None)),
            })
            .collect(),
        Err(_) => {
            log_error!("Wi-Fi scan failed");
            alloc::vec::Vec::new()
        }
    };
    log_info!("Wi-Fi scan found {} networks", networks.len());
    WIFI_SCAN_RESULT.signal(networks);
}

/// WiFi connection management task
///
/// Maintains WiFi connection, reconnecting when disconnected.
///
/// **This task is the only owner of the `WifiController`, and Improv borrows it by signal
/// rather than being handed one.** That is what makes esp-radio's warning about scanning and
/// connecting at the same time structurally unreachable instead of a thing to remember: the
/// two operations are arms of the same `select` and cannot overlap.
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

    // Nothing is configured until the application processor says so.
    //
    // That processor asks for credentials at boot and repeats every ten seconds until
    // answered, so on a healthy link this resolves within a second or two. On a broken one
    // it blocks forever, which is the correct and *visible* failure: the alternative --
    // retrying an empty configuration -- would fill the log with association failures that
    // say nothing about the actual fault, which is that the link never delivered.
    let mut credentials_rx = WIFI_CREDENTIALS
        .receiver()
        .expect("the credential watch is sized for this receiver");

    log_info!("Waiting for Wi-Fi credentials from the application processor");
    let mut current = loop {
        match credentials_rx.changed().await {
            Some(credentials) => break credentials,
            // A machine with no network configured. Reported at info, not warn: this is the
            // normal state of an unprovisioned machine, not a fault.
            None => log_info!("No Wi-Fi network configured; waiting to be provisioned"),
        }
    };

    apply_configuration(&mut controller, &current);

    loop {
        if controller.is_connected() {
            // While connected, periodically update RSSI and wait for disconnect.
            loop {
                // The reconnect request is the last arm, so it can never displace an
                // actual disconnect notification or an RSSI sample that was ready at
                // the same instant.
                match select4(
                    controller.wait_for_disconnect_async(),
                    Timer::after(Duration::from_secs(1)),
                    // The reconnect request and Improv's radio requests share an arm: both
                    // are things somebody asked for, and both are the least urgent thing here
                    // next to an actual link loss. Reconnect polls first because it is the
                    // one that can be raised while the machine is otherwise idle.
                    select(WIFI_RECONNECT_REQUEST.wait(), radio_request()),
                    credentials_rx.changed(),
                ).await {
                    Either4::First(_) => {
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
                    Either4::Second(_) => {
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
                    Either4::Third(Either::First(())) => {
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
                    // Improv wants the radio.
                    Either4::Third(Either::Second(request)) => match request {
                        RadioRequest::Candidate(candidate) => {
                            current = try_candidate(&mut controller, candidate, &current).await;
                            // Out to the outer loop either way. On success the association is
                            // fresh and the `is_connected()` check re-enters this loop
                            // immediately; on failure the restored configuration needs the
                            // reconnect the outer loop is about to do.
                            break;
                        }
                        // Serviced in place: a scan puts the controller back the way it found
                        // it, so there is nothing for the outer loop to re-establish.
                        RadioRequest::Scan => run_scan(&mut controller).await,
                    },
                    // New credentials from the application processor.
                    //
                    // Disconnected and reconfigured immediately rather than left to fail on
                    // its own: the old network may still be perfectly connectable, so
                    // nothing would ever prompt a change, and a user who has just
                    // provisioned a different network is watching.
                    Either4::Fourth(credentials) => {
                        match credentials {
                            // The application processor echoing back what we just proved.
                            //
                            // A successful Improv provision reports the credential up the
                            // link; that processor persists it and pushes the new value down
                            // unprompted, which arrives here. Without this check the push
                            // reads as a credential change and disconnects the association
                            // the user is at that moment being told succeeded.
                            Some(credentials) if credentials == current => {
                                log_info!(
                                    "Wi-Fi credentials confirmed by the application processor"
                                );
                                continue;
                            }
                            Some(credentials) => {
                                log_info!("Wi-Fi credentials changed; reconnecting");
                                current = credentials;
                                if controller.disconnect_async().await.is_err() {
                                    log_error!("Disconnect before reconfiguration failed");
                                }
                                apply_configuration(&mut controller, &current);
                            }
                            // Credentials cleared. Disconnect and let the loop fall through
                            // to `connect_async`, which will fail against a station with no
                            // SSID -- honest, and rare enough not to be worth a second
                            // parking state. Nothing clears credentials today.
                            None => {
                                log_info!("Wi-Fi credentials cleared; disconnecting");
                                let _ = controller.disconnect_async().await;
                            }
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
        // Raced against Improv's requests rather than awaited bare. An association attempt
        // blocks for as long as it takes, and the common case for a candidate is a machine
        // that has *no* network yet -- so a candidate raised here would otherwise not be seen
        // until an attempt against a network the user is trying to replace had timed out.
        match select(controller.connect_async(), radio_request()).await {
            Either::First(Ok(_)) => {
                // Edge triggered: `connect_async` waits for the association rather
                // than polling, so failed attempts fall to the `Err` arm and emit
                // nothing. The helper's `swap` closes the window between this
                // returning `Ok` and the `is_connected()` check at the top of the
                // loop.
                set_wifi_connected(true);
            }
            Either::First(Err(_e)) => {
                log_error!("Failed to connect to WiFi");
                Timer::after(Duration::from_millis(5000)).await
            }
            // Dropping `connect_async` mid-attempt is safe. The association may still
            // complete inside the driver, but `try_candidate` disconnects before it
            // configures anything, so the two cannot overlap -- and `set_wifi_connected`'s
            // `swap` is what keeps the event stream paired if it does complete late.
            Either::Second(RadioRequest::Candidate(candidate)) => {
                current = try_candidate(&mut controller, candidate, &current).await;
            }
            Either::Second(RadioRequest::Scan) => run_scan(&mut controller).await,
        }
    }
}

/// Network stack runner task
///
/// `WifiDevice` was renamed `Interface` in esp-radio 0.18.
///
/// The driver is wrapped in `instrumentation::CountingDriver` -- an
/// `#[embassy_executor::task]` cannot be generic, so the wrapper has to be named in this
/// signature rather than abstracted over.
#[embassy_executor::task]
pub async fn net_task(
    mut runner: NetRunner<'static, crate::instrumentation::CountingDriver<Interface<'static>>>,
) {
    runner.run().await
}
