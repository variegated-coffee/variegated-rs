//! BLE functionality - scanning, device management, and measurement loops

pub mod devices;
pub mod scanner;
pub mod status;

pub use devices::ble_devices_task;
pub use scanner::ScanPrinter;

use bt_hci::controller::ExternalController;
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
        let r = runner.run_with_handler(printer).await;
        if let Err(_e) = r {
            log_error!("Failed to run BLE, retrying in 10 seconds");
            Timer::after_secs(10).await;
        }
    }
}
