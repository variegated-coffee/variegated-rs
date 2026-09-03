//! BLE functionality - scanning, device management, and measurement loops

pub mod devices;
pub mod scale_slot;
pub mod scanner;
pub mod status;
pub mod ulanzi_slot;

pub use devices::{ble_devices_task, ble_slot_task};
pub use scanner::ScanPrinter;

use bt_hci::controller::ExternalController;
use core::future::Future;
use variegated_log::{log_error, log_warn};
use embassy_time::{Instant, Timer};
use esp_radio::ble::controller::BleConnector;
use trouble_host::prelude::*;

/// The HCI controller this firmware runs on.
///
/// Spelled out once here because it appears in every BLE signature in this module tree and
/// is neither short nor variable: there is exactly one radio, so a type parameter would be
/// a generic over a set with one member.
pub type SlotController = ExternalController<BleConnector<'static>, 20>;

/// The packet pool, likewise fixed.
pub type SlotPool = DefaultPacketPool;

/// A handle onto the connection manager.
///
/// `'static` because `ManagerHandle::register_device` returns a `DeviceHandle` carrying the
/// *manager's* lifetime, and the manager is `'static` in `ble_slot_task`. That is what lets
/// `ScaleProtocol::Gatt` be a plain associated type rather than a second GAT.
pub type SlotHandle =
    variegated_trouble_connection_manager::ManagerHandle<'static, SlotController, SlotPool>;

/// The BLE host stack.
pub type SlotStack = Stack<'static, SlotController, SlotPool>;

/// BLE host runner task
///
/// Runs the BLE host and delivers scan reports to the event handler.
#[embassy_executor::task]
pub async fn ble_runner_task(
    mut runner: Runner<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
    printer: &'static ScanPrinter,
) {
    // Since boot, not since the last success: `run_with_handler` only returns on error,
    // so there is no success to reset against. What the pair is for is telling a blip from
    // a stack that will not come back -- one failure an hour into a session reads very
    // differently from the fourth in forty seconds, and the log line alone cannot say
    // which without them.
    let mut failures: u32 = 0;
    let mut last_failure: Option<Instant> = None;
    let checkin = crate::checkin::MONITOR.claim(crate::checkin::CheckinId::BleRunner);

    loop {
        // Each pass is one `run_with_handler`, which only returns on error -- so a check-in
        // here means "the runner restarted", not "the runner is healthy". The counter below
        // is what distinguishes the two, and the row's age is what says how long the current
        // attempt has been running.
        checkin.good();

        // Run the BLE host runner with event handler
        // This processes HCI events and delivers scan reports to the printer
        //
        // Wrapped in a `poll_fn` purely to time it. This future is what drains HCI, so
        // the interval between its polls is the interval in which BLE events are not
        // being serviced -- and with seventeen tasks on one cooperative executor, a
        // long-running one elsewhere shows up here and nowhere else. See
        // `IndicatorId::BleRunnerGapMaxMs`.
        //
        // The stamp is taken before delegating rather than after, so the recorded gap is
        // poll-start to poll-start and therefore includes the runner's own work. That is
        // the number wanted: "how long since HCI was last serviced", not "how long the
        // executor was elsewhere".
        //
        // Zero cost to the future's behaviour -- `poll_fn` forwards the same `Context`
        // and the same `Poll`, so wakers and cancellation are unchanged.
        let inner = runner.run_with_handler(printer);
        let mut inner = core::pin::pin!(inner);
        let r = core::future::poll_fn(|cx| {
            crate::instrumentation::note_ble_runner_poll();
            inner.as_mut().poll(cx)
        })
        .await;
        match r {
            // Not expected: the runner loops until something fails. Worth a line of its
            // own rather than falling into the retry path, because "it stopped without an
            // error" and "it failed" have nothing in common as diagnoses.
            Ok(()) => {
                log_warn!("BLE runner returned without an error; restarting it immediately");
                continue;
            }
            Err(e) => {
                failures += 1;
                crate::instrumentation::note_ble_runner_failure();

                let now = Instant::now();
                let since_last_ms = last_failure
                    .map(|previous| (now - previous).as_millis())
                    .unwrap_or(0);
                last_failure = Some(now);

                // The gap is the reason this is logged here rather than left to the
                // sampler: it says whether the runner was being serviced normally when it
                // died. A large gap means the executor was elsewhere long enough for the
                // controller's event buffer to overflow, which is a starvation problem in
                // some other task; a small one means the failure came from the radio or
                // the controller and the executor is not where to look.
                let gap_ms = crate::instrumentation::ble_runner_gap_max_ms();

                match e {
                    // `BleConnectorError` has exactly one variant, `Unknown`, so printing
                    // this error value can never say more than "the controller layer".
                    //
                    // esp-radio does know more -- `parse_hci` logs the underlying
                    // `FromHciBytesError` before returning -- but it logs it through its
                    // own `warn!`, which resolves to `defmt` whenever `esp-radio/defmt` is
                    // on, and this firmware's defmt has no output target at all
                    // (`esp-println` is `no-op`; see the note in Cargo.toml). So that
                    // detail is currently emitted and thrown away. Recovering it means
                    // swapping esp-radio's `defmt` feature for `log-04`, which routes it
                    // to the `log` facade that `bus_sink` actually captures.
                    //
                    // Until then this arm's value is what it rules *out*: the host stack
                    // is fine, and the fault is below it.
                    BleHostError::Controller(_) => {
                        log_error!(
                            "BLE controller failed; esp-radio's error type carries no detail and its own diagnostics go to defmt, which this build discards. Failure {} of this boot, {} ms since the last, worst runner gap {} ms. Retrying in 10 s",
                            failures,
                            since_last_ms,
                            gap_ms
                        );
                    }
                    BleHostError::BleHost(host_error) => {
                        log_error!(
                            "BLE host failed: {:?}; failure {} of this boot, {} ms since the last, worst runner gap {} ms. Retrying in 10 s",
                            host_error,
                            failures,
                            since_last_ms,
                            gap_ms
                        );
                    }
                }

                Timer::after_secs(10).await;
            }
        }
    }
}
