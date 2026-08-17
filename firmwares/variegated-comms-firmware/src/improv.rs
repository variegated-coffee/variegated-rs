//! Improv Wi-Fi provisioning over BLE.
//!
//! The application processor decides *whether* to provision. It is the only side that knows a
//! shot is being pulled, and on the dual-boiler it is the side with the button. This module
//! is what happens once it has decided: it advertises for the duration it was given, serves
//! the Improv service, and reports what it learns back up the link.
//!
//! The protocol itself is in `variegated-improv-trouble`, which owns the GATT service and the
//! run loop. What lives here is the machine-specific half: where a candidate credential goes
//! to be tried, what a device name is, and where a result is reported.
//!
//! # The password never reaches a log from here
//!
//! [`MachineHandler::provision`] is handed one. It goes into a `WifiCredentials`, which elides
//! it in both `Debug` and `Format`, and then into [`ImprovReport::Provisioned`], whose
//! forwarding arm logs no field of it at all. Nothing between those two points formats the
//! argument.

use embassy_futures::select::{select3, Either3};
use embassy_time::{with_timeout, Duration, Instant, Timer};
use esp_radio::ble::controller::BleConnector;
use portable_atomic::Ordering;
use trouble_host::prelude::*;
use variegated_controller_types::wifi::wifi_credentials;
use variegated_improv_trouble::codec::State;
use variegated_improv_trouble::handler::{
    DeviceInfo, ImprovHandler, NetworkList, ProvisionError, Url,
};
use variegated_improv_trouble::service::{run, ImprovServer, MAX_NAME_LEN};
use variegated_log::{log_error, log_info, log_warn};

use crate::channels::{
    ImprovReport, IMPROV_REPORT_CHANNEL, IMPROV_STATE, MACHINE_DEFINITION, NO_IPV4,
    WIFI_CANDIDATE, WIFI_CANDIDATE_RESULT, WIFI_IPV4, WIFI_PROVISIONING_WINDOW,
    WIFI_SCAN_REQUEST, WIFI_SCAN_RESULT,
};

/// How long to wait for `wifi::connection_task` to report on a candidate.
///
/// Its own attempt is capped at 30 s; this is that plus slack for a task that may have been
/// in the middle of a disconnect when the candidate arrived. If it expires, that task is
/// wedged, and `UnableToConnect` is both true and the only thing there is to say.
const CANDIDATE_REPLY_TIMEOUT: Duration = Duration::from_secs(45);

/// How long a scan gets before the answer is called empty.
///
/// A backstop against a wedged connection task rather than a real bound -- the driver has its
/// own -- and an empty list is a legal answer.
const SCAN_TIMEOUT: Duration = Duration::from_secs(20);

/// How long to wait for DHCP before answering with no URL.
///
/// A lease normally lands within two or three seconds. Fifteen is generous enough that a slow
/// access point does not cost the user the redirect and short enough to stay inside a client's
/// own patience -- and the fallback is a plain success, not a failure.
const ADDRESS_TIMEOUT: Duration = Duration::from_secs(15);

struct MachineHandler {
    device_name: heapless::String<MAX_NAME_LEN>,
}

impl ImprovHandler for MachineHandler {
    async fn provision(&mut self, ssid: &str, password: &str) -> Result<Option<Url>, ProvisionError> {
        let candidate = wifi_credentials(ssid, password);

        // Cleared before the request goes out, not after the answer comes back. A previous
        // attempt that timed out may have left its verdict here, and taking that as the
        // answer to *this* attempt is the one failure a request/reply pair cannot rule out on
        // its own -- the stale answer arrives after the previous requester has given up.
        WIFI_CANDIDATE_RESULT.reset();
        WIFI_CANDIDATE.signal(candidate.clone());

        log_info!("Improv: handed a candidate credential to the Wi-Fi task");

        let associated = match with_timeout(CANDIDATE_REPLY_TIMEOUT, WIFI_CANDIDATE_RESULT.wait())
            .await
        {
            Ok(associated) => {
                log_info!("Improv: candidate verdict: associated={}", associated);
                associated
            }
            // A timeout here does not mean the password was wrong -- it means nothing was
            // listening. `connection_task` only polls for candidates once it has credentials
            // of its own, so an unprovisioned machine parks before the arm that would answer
            // this. Said explicitly because the two are reported to the client identically.
            Err(_) => {
                log_error!(
                    "Improv: the Wi-Fi task did not answer a candidate within {} s -- \
                     is it parked waiting for credentials?",
                    CANDIDATE_REPLY_TIMEOUT.as_secs()
                );
                false
            }
        };

        if !associated {
            return Err(ProvisionError::UnableToConnect);
        }

        // Persisted by the application processor, which is the only side with flash. Reported
        // rather than commanded: `WifiCredentialsProvisioned` is what tells that processor
        // these came off a radio that associated with them, which is what makes storing them
        // unvalidated correct. See the note on `ImprovReport`.
        //
        // `try_send`, because this runs on a BLE connection's event loop and must not block.
        // A drop here means the network is joined but never remembered, so it is logged at
        // error -- the phone will already have said it worked.
        if IMPROV_REPORT_CHANNEL
            .try_send(ImprovReport::Provisioned(candidate))
            .is_err()
        {
            log_error!("Provisioned Wi-Fi credentials dropped: report channel full");
        } else {
            log_info!("Improv: reported the provisioned credential to the application processor");
        }

        Ok(local_url().await)
    }

    async fn scan(&mut self) -> NetworkList {
        WIFI_SCAN_RESULT.reset();
        WIFI_SCAN_REQUEST.signal(());

        let found = match with_timeout(SCAN_TIMEOUT, WIFI_SCAN_RESULT.wait()).await {
            Ok(found) => found,
            // Same caveat as the candidate timeout above: nothing was listening, which is
            // not the same as "no networks", but the protocol cannot express the difference.
            Err(_) => {
                log_error!("Improv: the Wi-Fi task did not answer a scan request");
                alloc::vec::Vec::new()
            }
        };

        // Truncated rather than refused if the connection task somehow returned more than the
        // scan was configured for: a short list is a usable answer and an error is not.
        let mut networks = NetworkList::new();
        for network in found {
            if networks.push(network).is_err() {
                break;
            }
        }
        networks
    }

    fn identify(&mut self) {
        log_info!("Improv identify requested");
        if IMPROV_REPORT_CHANNEL.try_send(ImprovReport::Identify).is_err() {
            log_warn!("Dropped an identify request: report channel full");
        }
    }

    fn device_info(&self) -> DeviceInfo<'_> {
        DeviceInfo {
            firmware_name: "Variegated",
            firmware_version: env!("CARGO_PKG_VERSION"),
            chip_variant: "ESP32-C6",
            device_name: self.device_name.as_str(),
        }
    }

    fn state_changed(&mut self, state: State) {
        IMPROV_STATE.store(state as u8, Ordering::Relaxed);
    }
}

/// The address to send the client to, once DHCP has one.
///
/// `WIFI_IPV4` is refreshed once a second by `comms_status_signaller_task`, so this polls
/// rather than waits. `None` after [`ADDRESS_TIMEOUT`] is a complete answer: provisioning
/// succeeded, there is simply nowhere to point yet. It must not become an error -- the
/// network *is* joined, and a failure here would send the user to retype a correct password.
async fn local_url() -> Option<Url> {
    let deadline = Instant::now() + ADDRESS_TIMEOUT;
    loop {
        let raw = WIFI_IPV4.load(Ordering::Relaxed);
        if raw != NO_IPV4 {
            let octets = raw.to_be_bytes();
            let mut url = Url::new();
            let _ = core::fmt::Write::write_fmt(
                &mut url,
                format_args!("http://{}.{}.{}.{}/", octets[0], octets[1], octets[2], octets[3]),
            );
            return Some(url);
        }
        if Instant::now() >= deadline {
            log_warn!(
                "Improv: provisioned, but no DHCP lease after {} s -- answering with no URL",
                ADDRESS_TIMEOUT.as_secs()
            );
            return None;
        }
        Timer::after(Duration::from_millis(250)).await;
    }
}

/// The machine's name, for the advertisement and for `GET_DEVICE_INFO`.
async fn device_name() -> heapless::String<MAX_NAME_LEN> {
    let name = MACHINE_DEFINITION
        .lock()
        .await
        .as_ref()
        .map(|definition| definition.name.clone());

    match name {
        // Truncated on a byte boundary rather than a character one, unlike
        // `wifi_credentials`. The only consumers are an advertising payload and a scanner's
        // display, and a machine definition with a 30-byte name is not a case worth a second
        // helper -- but the slice has to be taken with `get`, because a multi-byte character
        // straddling the boundary would otherwise panic.
        Some(name) => name
            .get(..name.len().min(MAX_NAME_LEN))
            .and_then(|truncated| heapless::String::try_from(truncated).ok())
            .unwrap_or_else(|| fallback_name()),
        // Reachable at boot: the definition arrives over the link, and a window opened before
        // it lands would otherwise advertise an empty name, which some scanners render as a
        // blank row.
        None => fallback_name(),
    }
}

fn fallback_name() -> heapless::String<MAX_NAME_LEN> {
    heapless::String::try_from("Variegated").unwrap_or_default()
}

/// Advertise the Improv service for as long as the application processor says to.
///
/// Idle otherwise: this firmware does not advertise unless a window is open, and the window
/// *is* the authorization. Nothing here decides whether one may open -- see the
/// `OpenWifiProvisioningWindow` arm on the application processor, which refuses while the
/// machine is busy, on the same grounds a discovery scan is refused.
#[embassy_executor::task]
pub async fn improv_task(
    mut peripheral: Peripheral<
        'static,
        ExternalController<BleConnector<'static>, 20>,
        DefaultPacketPool,
    >,
) {
    log_info!("Improv provisioning task started, window closed");
    let checkin = crate::checkin::MONITOR.claim(crate::checkin::CheckinId::Improv);

    loop {
        checkin.good();

        // A zero is a close for a window that is not open. The application processor sends one
        // on `CloseWifiProvisioningWindow` regardless of what it believes is open, and
        // treating it as "advertise for 0 ms" would spin.
        // Timed out rather than parked. This signal is silent for the entire life of a
        // machine nobody is provisioning, so without a timeout the row could never report
        // this task wedged -- and it holds a BLE peripheral, so a wedge here is a radio that
        // will not advertise when someone finally does hold the button down.
        let duration_ms = loop {
            checkin.good();
            let Ok(requested) =
                with_timeout(variegated_checkin::HEARTBEAT, WIFI_PROVISIONING_WINDOW.wait()).await
            else {
                continue;
            };
            if requested > 0 {
                break requested;
            }
        };

        let name = device_name().await;
        // The other half of the bracket in `wifi::try_candidate`. A provisioning cycle holds
        // a BLE peripheral connection open *and* re-associates Wi-Fi, and the ~42 kB the
        // heap never gets back has never been attributed to one or the other. These two
        // lines bound the BLE session; the four in `try_candidate` bound the Wi-Fi work.
        log_info!(
            "Improv provisioning window open for {} ms (heap free {})",
            duration_ms,
            crate::debug::snapshot::heap_free()
        );

        // Built per window rather than once, because the attribute table borrows `name` and
        // the machine can be renamed between windows. The characteristic value storage behind
        // it is a `static_cell::StaticCell` per characteristic and is *not* rebuilt -- so this
        // must not be reached twice concurrently, and it cannot be: one task, one loop.
        let server = match ImprovServer::new_with_config(GapConfig::Peripheral(PeripheralConfig {
            name: name.as_str(),
            appearance: &appearance::power_device::GENERIC_POWER_DEVICE,
        })) {
            Ok(server) => server,
            Err(error) => {
                log_error!("Could not build the Improv GATT server: {}", error);
                continue;
            }
        };

        let mut handler = MachineHandler { device_name: name.clone() };

        match select3(
            Timer::after(Duration::from_millis(duration_ms as u64)),
            // A second signal closes the window early. Any value ends it, including another
            // non-zero duration: re-opening is the next iteration's job, and extending an open
            // window in place would need a deadline this loop does not keep.
            WIFI_PROVISIONING_WINDOW.wait(),
            run(&mut peripheral, &server, &mut handler, name.as_str()),
        )
        .await
        {
            Either3::First(()) => log_info!("Improv provisioning window expired"),
            Either3::Second(_) => log_info!("Improv provisioning window closed"),
            // `run` loops over connections and only returns on error.
            Either3::Third(result) => match result {
                Ok(()) => log_info!("Improv service stopped"),
                // The error itself, not just that there was one. This arm used to discard
                // it, which made an advertisement the controller rejected -- the single most
                // likely failure here, and the one that produces no other symptom -- look
                // identical to every other way the service can stop.
                Err(error) => log_error!("Improv service failed: {:?}", error),
            },
        }

        // Set here rather than inside `run`, which is dropped without unwinding in two of the
        // three arms above and so cannot be relied on to report its own end.
        IMPROV_STATE.store(State::Stopped as u8, Ordering::Relaxed);

        // Dropping `run` drops the `GattConnection` and the `Advertiser` with it, so whatever
        // the BLE session was holding has been released by the time this runs. A figure here
        // close to the one logged when the window opened means the session gave its memory
        // back and the Wi-Fi path owns the loss; a figure well below it means the opposite.
        log_info!(
            "Improv provisioning window closed (heap free {})",
            crate::debug::snapshot::heap_free()
        );
    }
}
