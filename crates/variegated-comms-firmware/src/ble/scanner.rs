//! BLE scanning and device discovery

use core::cell::RefCell;

use variegated_log::log_info;
use heapless::Deque;
use portable_atomic::Ordering;
use trouble_host::prelude::*;

use crate::channels::BLE_RESCAN_PENDING;

/// Event handler for BLE scanning - tracks and logs discovered devices
pub struct ScanPrinter {
    pub seen: RefCell<Deque<BdAddr, 128>>,
}

impl ScanPrinter {
    pub fn new() -> Self {
        Self {
            seen: RefCell::new(Deque::new()),
        }
    }
}

impl EventHandler for ScanPrinter {
    fn on_adv_reports(&self, mut it: LeAdvReportsIter<'_>) {
        let mut seen = self.seen.borrow_mut();

        // `CommsDebugOp::RescanBle` sets this. Clearing the seen-set is the whole of
        // what a rescan can honestly do on this firmware: there is no free-running
        // scan to restart -- the connection manager scans only inside a filtered
        // `connect` for a device it already maintains -- so the only thing an operator
        // can be asking for is to be told again what is visible, which this
        // suppression is otherwise hiding from them.
        //
        // `swap` rather than load-then-store: this runs from the BLE runner's event
        // callback, which is not the task that set the flag, and a request arriving
        // between the two would otherwise be consumed without ever clearing the list.
        if BLE_RESCAN_PENDING.swap(false, Ordering::Relaxed) {
            log_info!("BLE rescan requested; re-reporting every visible device");
            seen.clear();
        }

        while let Some(Ok(report)) = it.next() {
            log_info!("Adv report: {:?}", report);

            // Decode and print advertising data structures
            log_info!("  Decoded advertising data:");
            for structure in AdStructure::decode(report.data) {
                match structure {
                    Ok(ad) => log_info!("    {:?}", ad),
                    Err(_) => log_info!("    [Decode error]"),
                }
            }

            // Track unique devices
            if seen.iter().find(|b| b.raw() == report.addr.raw()).is_none() {
                log_info!("Discovered BLE device: {:?}, RSSI: {}", report.addr, report.rssi);
                if seen.is_full() {
                    seen.pop_front();
                }
                seen.push_back(report.addr).unwrap();
            }
        }
    }
}
