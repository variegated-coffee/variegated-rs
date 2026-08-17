//! WiFi connection management tasks

use variegated_log::{log_error, log_info};
use embassy_net::Runner as NetRunner;
use embassy_time::{with_timeout, Duration, Timer};
use embassy_futures::select::{select, select3, select4, Either, Either3, Either4};
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
        Either::First(candidate) => {
            log_info!("Wi-Fi task: taking a candidate credential from Improv");
            RadioRequest::Candidate(candidate)
        }
        Either::Second(()) => {
            log_info!("Wi-Fi task: taking a scan request from Improv");
            RadioRequest::Scan
        }
    }
}

/// Try a candidate credential, and put the radio back where it was if it fails.
///
/// Returns the credential to treat as current from here on -- the candidate if it associated,
/// the previous one otherwise -- and whether it associated. **The caller has to adopt both.**
///
/// The credential, because the application processor pushes these same credentials back down
/// the link once it has persisted them, and a `current` still holding the old value would
/// read that push as a change and tear down the association the user is at that moment being
/// told succeeded.
///
/// The flag, because `controller.is_connected()` is **not** a usable substitute the instant
/// this returns. It lags the association, so a caller that fell through to its reconnect path
/// on `!is_connected()` would call `connect_async` against a station that had just associated
/// and tear it straight back down. That was observed: a successful provision left the machine
/// with no network at all, the DHCP lease never arrived, and Improv answered with no URL.
async fn try_candidate(
    controller: &mut WifiController<'static>,
    candidate: WifiCredentials,
    previous: &WifiCredentials,
) -> (WifiCredentials, bool) {
    // Nothing on this path logs the credential, not even the SSID. `WifiCredentials`' `Format`
    // elides the password, but the SSID alone is enough to make a log line worth not writing
    // on a path that runs while someone is provisioning and may be sharing a screen.
    // Bracketed step by step, because a re-association is this firmware's peak-memory event
    // and nobody knows which part of it costs what. Measured across a whole provisioning
    // cycle the heap goes from ~73 kB to ~115 kB and stays there, but that cycle also holds
    // a BLE connection open, and the two have never been told apart. `set_config` is
    // exonerated by inspection -- on a second call with the mode unchanged it skips both
    // `stop_impl` and `esp_wifi_start` and only reapplies the STA config -- so the cost is
    // either the association itself or something outside this function entirely.
    //
    // Cheap to leave in: four `log_info!` on a path that runs when a human is provisioning.
    log_info!(
        "Trying candidate Wi-Fi credentials (heap free {})",
        crate::debug::snapshot::heap_free()
    );

    if controller.disconnect_async().await.is_err() {
        log_error!("Disconnect before trying a candidate failed");
    }
    set_wifi_connected(false);
    WIFI_RSSI_SIGNAL.signal(None);
    WIFI_RSSI_DBM.store(NO_RSSI, Ordering::Relaxed);
    log_info!(
        "Candidate: disconnected (heap free {})",
        crate::debug::snapshot::heap_free()
    );

    apply_configuration(controller, &candidate);
    log_info!(
        "Candidate: reconfigured (heap free {})",
        crate::debug::snapshot::heap_free()
    );

    let associated = matches!(
        with_timeout(CANDIDATE_TIMEOUT, controller.connect_async()).await,
        Ok(Ok(_))
    );
    log_info!(
        "Candidate: association attempt finished (heap free {})",
        crate::debug::snapshot::heap_free()
    );

    if associated {
        set_wifi_connected(true);
        WIFI_CANDIDATE_RESULT.signal(true);
        log_info!("Candidate Wi-Fi credentials associated");
        (candidate, true)
    } else {
        // The working network goes back before the answer goes out. Answering first would
        // race the Improv task into reporting a credential that is about to be discarded --
        // and more to the point, a failed provisioning attempt must not cost a machine the
        // network it already had.
        log_error!("Candidate Wi-Fi credentials failed to associate");
        let _ = controller.disconnect_async().await;
        apply_configuration(controller, previous);
        WIFI_CANDIDATE_RESULT.signal(false);
        (previous.clone(), false)
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

/// The receiver [`connection_task`] takes on [`WIFI_CREDENTIALS`].
///
/// Named because [`park_until_provisioned`] takes one and the type is unreadable inline.
type CredentialsReceiver = embassy_sync::watch::Receiver<
    'static,
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    Option<WifiCredentials>,
    { crate::channels::WIFI_CREDENTIAL_RECEIVERS },
>;

/// Resolve only when the stored credentials become something *other* than what is in use.
///
/// The application processor re-publishes the credential it already sent -- once at every
/// boot, when the controller sets `wifi_publish_pending`, and again after a successful
/// provision. Such an echo changes nothing and **must not resolve**, because both loops below
/// race this against something that is destroyed by losing the race:
///
/// * `connect_async` calls `esp_wifi_connect` *synchronously* and only then awaits the event,
///   so dropping the future does not cancel the attempt -- the driver is still connecting.
///   Re-issuing it returns `ESP_ERR_WIFI_CONN` ("station control block wrong"), which esp-radio
///   does not map and therefore **panics** on rather than returning. That is a firmware crash
///   from an echo, and it is intermittent because it needs the echo to land inside the
///   association attempt.
/// * `wait_for_disconnect_async` is a pure wait, so dropping and recreating it opens a window
///   in which a disconnect event can be missed entirely.
///
/// Filtering here rather than in each arm is what makes both safe: an echo is consumed and
/// logged without ever waking the select, so nothing gets dropped and nothing is re-issued.
async fn credentials_change(
    credentials_rx: &mut CredentialsReceiver,
    current: &WifiCredentials,
) -> Option<WifiCredentials> {
    loop {
        match credentials_rx.changed().await {
            Some(credentials) if credentials == *current => {
                log_info!("Wi-Fi credentials confirmed by the application processor");
            }
            other => break other,
        }
    }
}

/// Wait until this machine has a network to join, **servicing Improv while waiting**.
///
/// Returns the credentials to treat as current, and whether the radio is *already* associated
/// with them -- which it is when they came from an Improv candidate, and is not when they
/// arrived over the link. The radio is left configured for them either way, so the caller has
/// nothing to apply.
///
/// This is the unprovisioned state, and there is exactly one of it: entered at boot, and
/// returned to when the application processor reports the credentials cleared. Keeping the two
/// identical is the point -- the state a machine is in before it has ever been provisioned is
/// the state Improv exists to get it out of, and a bench machine that could only reach it by
/// having its flash erased was a state nobody tested.
///
/// **Servicing Improv from here is the whole reason this function exists.** The wait used to be
/// a bare `credentials_rx.changed()`, which meant a machine with no stored network -- the
/// primary Improv use case -- parked before the arm that answers a candidate. A client could
/// connect, send a password, and get nothing back but the 30 s timeout in
/// `improv::MachineHandler::provision`, which reported `UnableToConnect` for a credential the
/// radio had never been asked to try.
///
/// Scanning works here even though nothing has configured the station yet: `esp_radio::wifi::new`
/// applies `ControllerConfig::initial_config`, which defaults to `Config::Station(..)`, and that
/// call is what starts it. That was the open question that kept this fix waiting.
async fn park_until_provisioned(
    controller: &mut WifiController<'static>,
    credentials_rx: &mut CredentialsReceiver,
) -> (WifiCredentials, bool) {
    log_info!("Waiting for Wi-Fi credentials from the application processor");

    loop {
        match select(credentials_rx.changed(), radio_request()).await {
            Either::First(Some(credentials)) => {
                apply_configuration(controller, &credentials);
                break (credentials, false);
            }
            // A machine with no network configured. Reported at info, not warn: this is the
            // normal state of an unprovisioned machine, not a fault.
            Either::First(None) => {
                log_info!("No Wi-Fi network configured; waiting to be provisioned")
            }
            Either::Second(RadioRequest::Candidate(candidate)) => {
                // `previous` is the empty credential, because there is no previous -- which is
                // also exactly what the station holds, so the restore `try_candidate` performs
                // on failure puts it back where it started rather than somewhere invented.
                let (adopted, associated) =
                    try_candidate(controller, candidate, &WifiCredentials::default()).await;
                if associated {
                    // Returned *without* re-applying the configuration. `try_candidate` has
                    // already configured and associated, and `set_config` on a live
                    // association tears it down -- the same fault that once left a
                    // provisioned machine with no network. See the note on `try_candidate`.
                    break (adopted, true);
                }
                // A failed candidate leaves the machine where it was: unprovisioned, with the
                // client already told. Keep waiting rather than falling through to a connect
                // loop with nothing to connect to.
            }
            Either::Second(RadioRequest::Scan) => run_scan(controller).await,
        }
    }
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
    // it parks forever, which is the correct and *visible* failure: the alternative --
    // retrying an empty configuration -- would fill the log with association failures that
    // say nothing about the actual fault, which is that the link never delivered.
    let mut credentials_rx = WIFI_CREDENTIALS
        .receiver()
        .expect("the credential watch is sized for this receiver");

    let (mut current, mut just_associated) =
        park_until_provisioned(&mut controller, &mut credentials_rx).await;

    let checkin = crate::checkin::MONITOR.claim(crate::checkin::CheckinId::WifiConnection);

    loop {
        checkin.good();

        // `just_associated` is consulted alongside the controller's own view because
        // `is_connected()` **lags** a fresh association. A candidate that has just succeeded
        // would otherwise be found disconnected here, take the reconnect path, and call
        // `connect_async` over the top of the link it had only just made -- which is the
        // exact fault that left a provisioned machine with no network at all. See the note on
        // `try_candidate`. True for one pass, then cleared.
        if just_associated || controller.is_connected() {
            just_associated = false;
            // While connected, periodically update RSSI and wait for disconnect.
            loop {
                // Inside the *inner* loop, not the outer one. A connected machine never
                // leaves this loop, so a check-in at the top of the outer loop would run
                // once at association and then not again until the link dropped -- the row
                // would age forever on a machine whose Wi-Fi is working perfectly. The 1 s
                // RSSI arm below is what gives this its cadence.
                checkin.good();

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
                    // Filtered, like the retry loop's. Here the cost of an echo waking this
                    // select is subtler than a panic but no more welcome: a `continue` drops
                    // and recreates `wait_for_disconnect_async`, and a disconnect landing in
                    // that window is not reported at all.
                    credentials_change(&mut credentials_rx, &current),
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
                            let (adopted, associated) =
                                try_candidate(&mut controller, candidate, &current).await;
                            current = adopted;
                            // **Stay in this loop when it worked.** The association is fresh
                            // and there is nothing to re-establish; dropping to the outer
                            // loop would re-read `is_connected()`, which lags, find it false,
                            // and reconnect over the top of the link that was just made. That
                            // is exactly what left a provisioned machine with no network.
                            //
                            // On failure the restored configuration does need the outer
                            // loop's reconnect, so that path is unchanged.
                            if !associated {
                                break;
                            }
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
                    // An echo of the credential in use never reaches here: `credentials_change`
                    // absorbs it. That guard is what stops a successful Improv provision --
                    // which the application processor persists and pushes straight back down
                    // -- from reading as a change and tearing down the association the user is
                    // at that moment being told succeeded.
                    Either4::Fourth(credentials) => {
                        match credentials {
                            Some(credentials) => {
                                log_info!("Wi-Fi credentials changed; reconnecting");
                                current = credentials;
                                if controller.disconnect_async().await.is_err() {
                                    log_error!("Disconnect before reconfiguration failed");
                                }
                                apply_configuration(&mut controller, &current);
                            }
                            // Credentials cleared -- `AppDebugOp::ClearWifiCredentials`, which
                            // exists to reach exactly this state.
                            //
                            // Back to the parked state rather than falling through to
                            // `connect_async`. Falling through means an unbounded retry loop
                            // against an empty configuration, which is precisely the
                            // log-spam-that-says-nothing this task avoids at boot.
                            //
                            // The empty configuration is applied first so the radio genuinely
                            // forgets: without it the station keeps the old SSID and the
                            // driver reconnects to the network the operator just asked it to
                            // forget. That leaves this identical to a cold boot, which is the
                            // property that makes the debug op a real reproduction.
                            None => {
                                log_info!("Wi-Fi credentials cleared; returning to the unprovisioned state");
                                let _ = controller.disconnect_async().await;
                                apply_configuration(&mut controller, &WifiCredentials::default());
                                set_wifi_connected(false);
                                WIFI_RSSI_SIGNAL.signal(None);
                                WIFI_RSSI_DBM.store(NO_RSSI, Ordering::Relaxed);
                                let (adopted, associated) =
                                    park_until_provisioned(&mut controller, &mut credentials_rx)
                                        .await;
                                current = adopted;
                                just_associated = associated;
                                break;
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
        // Three-way, not two. The credential receiver was missing from this select entirely,
        // which meant a credential *change* was only ever noticed from the connected loop: a
        // machine sitting here retrying a network it could not reach would ignore a new
        // password until the old one worked, which is the one thing that was never going to
        // happen. Clearing them had the same problem.
        //
        // `credentials_change`, not `credentials_rx.changed()`. Winning this race destroys the
        // connect attempt in a way that cannot be undone or safely repeated -- see the note on
        // that function, which is the whole reason it exists. Every arm below that returns to
        // the top of this loop must leave the driver with no connect in flight.
        match select3(
            controller.connect_async(),
            radio_request(),
            credentials_change(&mut credentials_rx, &current),
        )
        .await
        {
            Either3::First(Ok(_)) => {
                // Edge triggered: `connect_async` waits for the association rather
                // than polling, so failed attempts fall to the `Err` arm and emit
                // nothing. The helper's `swap` closes the window between this
                // returning `Ok` and the `is_connected()` check at the top of the
                // loop.
                set_wifi_connected(true);
            }
            Either3::First(Err(_e)) => {
                log_error!("Failed to connect to WiFi");
                Timer::after(Duration::from_millis(5000)).await
            }
            // Dropping `connect_async` mid-attempt is safe. The association may still
            // complete inside the driver, but `try_candidate` disconnects before it
            // configures anything, so the two cannot overlap -- and `set_wifi_connected`'s
            // `swap` is what keeps the event stream paired if it does complete late.
            //
            // The verdict is adopted here as well, not discarded. Dropping it looks safe --
            // the link was already down, so there is no association to protect -- but if the
            // candidate *succeeded* there now is one, and the next pass of the outer loop
            // reads a lagging `is_connected()`, finds it false, and reconnects over the top
            // of it. Same fault as the connected arm, one loop later.
            Either3::Second(RadioRequest::Candidate(candidate)) => {
                let (adopted, associated) =
                    try_candidate(&mut controller, candidate, &current).await;
                current = adopted;
                just_associated = associated;
            }
            // Disconnected first, unlike the candidate arm above, which gets it from
            // `try_candidate`. `connect_async` has already issued `esp_wifi_connect` and been
            // dropped mid-attempt, so the driver is still connecting; scanning on top of that
            // returns `ESP_ERR_WIFI_STATE`, which esp-radio does not map and panics on.
            Either3::Second(RadioRequest::Scan) => {
                let _ = controller.disconnect_async().await;
                run_scan(&mut controller).await;
            }
            // A credential change while the link is down. Applied rather than left for the
            // next association to notice, because on this path there may never be one: this
            // is the retry loop for a network that is not answering, and the whole reason a
            // new credential arrived is that the old one is not going to start working.
            //
            // An echo of the credential already in use never reaches here -- `credentials_change`
            // absorbs it -- so both arms below are genuine changes, and both disconnect before
            // touching the configuration.
            Either3::Third(credentials) => match credentials {
                Some(credentials) => {
                    log_info!("Wi-Fi credentials changed; retrying with them");
                    let _ = controller.disconnect_async().await;
                    current = credentials;
                    apply_configuration(&mut controller, &current);
                }
                // Cleared. Back to the parked state, exactly as the connected arm does -- see
                // the note there for why an empty configuration is applied rather than the
                // old one left in place.
                None => {
                    log_info!("Wi-Fi credentials cleared; returning to the unprovisioned state");
                    let _ = controller.disconnect_async().await;
                    apply_configuration(&mut controller, &WifiCredentials::default());
                    let (adopted, associated) =
                        park_until_provisioned(&mut controller, &mut credentials_rx).await;
                    current = adopted;
                    just_associated = associated;
                }
            },
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
    // Poll-liveness is the only thing available here -- the loop is inside smoltcp -- and it
    // is worth having: every socket on this board goes quiet when this stops being polled,
    // and nothing else on the table would say why.
    variegated_checkin::watch(
        crate::checkin::MONITOR.claim(crate::checkin::CheckinId::Net),
        runner.run(),
    )
    .await
}
