//! BLE scanning and device discovery

use core::cell::RefCell;

use variegated_log::log_info;
use heapless::Deque;
use trouble_host::prelude::*;

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
