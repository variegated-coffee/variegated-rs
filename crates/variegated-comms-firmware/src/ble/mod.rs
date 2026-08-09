//! BLE functionality - scanning, device management, and measurement loops

pub mod devices;
pub mod scanner;
pub mod status;

pub use devices::{ble_devices_task, ble_slot_task};
pub use scanner::ScanPrinter;

use bt_hci::controller::ExternalController;
use core::future::Future;
use variegated_log::log_error;
use embassy_time::Timer;
use esp_radio::ble::controller::BleConnector;
use trouble_host::prelude::*;

/// BLE host runner task
///
/// Runs the BLE host and delivers scan reports to the event handler.
#[embassy_executor::task]
pub async fn ble_runner_task(
    mut runner: Runner<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
    printer: &'static ScanPrinter,
) {
    loop {
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
        if let Err(_e) = r {
            log_error!("Failed to run BLE, retrying in 10 seconds");
            Timer::after_secs(10).await;
        }
    }
}
