//! BLE scanning and device discovery

use core::cell::RefCell;

use variegated_log::{log_info, log_warn};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Receiver};
use heapless::Deque;
use heapless::index_map::FnvIndexMap;
use portable_atomic::{AtomicBool, AtomicU16, Ordering};
use trouble_host::prelude::*;
use variegated_controller_types::bluetooth::{
    bluetooth_name, BluetoothName, DiscoveredBluetoothPeripheral,
};
use variegated_trouble_connection_manager::ScanSink;

use crate::channels::BLE_RESCAN_PENDING;

/// What the scanner has to tell the application processor.
///
/// One channel rather than a channel plus a signal, so the forwarding task needs one arm
/// instead of two -- the `select` it lives in is already nested two deep.
pub enum ScanReport {
    Discovered(DiscoveredBluetoothPeripheral),
    /// The scan is over. `reports_dropped` counts advertising reports seen but not
    /// forwarded.
    ///
    /// **Best effort.** This is sent from a synchronous callback that cannot await, so a
    /// queue still full of results at the end of a busy scan will drop it. That is why
    /// the application processor times its own scan out rather than waiting for this:
    /// the message is an optimisation, not the mechanism.
    Finished { reports_dropped: u16 },
}

/// How many discovered devices can be queued for the application processor at once.
///
/// Deliberately small. This is drained by the task that also writes the UART, so it is a
/// buffer for a burst rather than a backlog: a busy room produces reports faster than the
/// link carries them, and the right response is to drop and say so, not to grow.
pub const SCAN_RESULT_CAPACITY: usize = 16;

/// How many addresses the reporter remembers within one scan.
///
/// Separate from -- and much smaller than -- the log suppressor's set, because it is
/// cleared at the start of every scan rather than living for the whole uptime.
const SCAN_REPORTED_CAPACITY: usize = 32;

/// Event handler for BLE scanning: logs discovered devices, and during a user-initiated
/// scan forwards them to the application processor.
pub struct ScanPrinter {
    pub seen: RefCell<Deque<BdAddr, 128>>,
    /// Whether a discovery scan is running, so that devices seen incidentally while the
    /// connection loop is connecting are not reported as scan results.
    active: AtomicBool,
    /// Addresses reported during the current scan, and whether the report carried a name.
    ///
    /// The `bool` is the whole trick -- see [`Self::on_adv_reports`].
    reported: RefCell<FnvIndexMap<BdAddr, bool, SCAN_REPORTED_CAPACITY>>,
    results: Channel<CriticalSectionRawMutex, ScanReport, SCAN_RESULT_CAPACITY>,
    dropped: AtomicU16,
}

impl ScanPrinter {
    pub const fn new() -> Self {
        Self {
            seen: RefCell::new(Deque::new()),
            active: AtomicBool::new(false),
            reported: RefCell::new(FnvIndexMap::new()),
            results: Channel::new(),
            dropped: AtomicU16::new(0),
        }
    }

    /// Scan reports, for the task that forwards them to the application processor.
    pub fn results(
        &self,
    ) -> Receiver<'_, CriticalSectionRawMutex, ScanReport, SCAN_RESULT_CAPACITY>
    {
        self.results.receiver()
    }
}

impl ScanSink for ScanPrinter {
    fn begin(&self) {
        self.reported.borrow_mut().clear();
        self.dropped.store(0, Ordering::Relaxed);
        self.active.store(true, Ordering::Relaxed);
    }

    fn end(&self, started: bool) {
        self.active.store(false, Ordering::Relaxed);

        // Logged here rather than in the manager because the manager logs with raw
        // `defmt`, which does not reach this firmware's debug transports -- so a scan
        // that never started looked exactly like one that found nothing.
        if !started {
            log_warn!("Bluetooth scan did not start: the controller refused it");
        }

        let reports_dropped = self.dropped.load(Ordering::Relaxed);
        // If this does not fit, the application processor's own scan deadline ends the
        // scan a few seconds later. Losing it costs the dropped-report count, not the
        // scan.
        if self.results.try_send(ScanReport::Finished { reports_dropped }).is_err() {
            log_info!("Scan-finished report dropped: result queue still full");
        }
    }
}

/// The advertised local name, or empty if the device advertised none.
///
/// `AdStructure`'s name variants carry raw bytes rather than `&str`, and the bytes are
/// chosen by whatever device is in radio range -- so this validates as UTF-8 and
/// truncates on a character boundary. `bluetooth_name` does the latter; a bare slice
/// would panic on a multi-byte character straddling the limit.
fn local_name(data: &[u8]) -> BluetoothName {
    let mut complete: Option<&[u8]> = None;
    let mut shortened: Option<&[u8]> = None;

    for structure in AdStructure::decode(data) {
        match structure {
            Ok(AdStructure::CompleteLocalName(bytes)) => complete = Some(bytes),
            // First one wins: a device sending two shortened names is malformed, and
            // preferring the earlier is as good an answer as any.
            Ok(AdStructure::ShortenedLocalName(bytes)) if shortened.is_none() => {
                shortened = Some(bytes)
            }
            _ => {}
        }
    }

    let bytes = complete.or(shortened).unwrap_or(&[]);
    bluetooth_name(core::str::from_utf8(bytes).unwrap_or(""))
}

impl EventHandler for ScanPrinter {
    fn on_adv_reports(&self, mut it: LeAdvReportsIter<'_>) {
        let mut seen = self.seen.borrow_mut();

        // `CommsDebugOp::RescanBle` sets this. It clears the *log* suppressor, which is a
        // separate thing from the scan reporting below: this makes the firmware re-log
        // every visible device, which is what an operator watching the log asked for.
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

            // Everything below is scan reporting, which only happens while the user asked
            // for a scan. Reports also arrive as a side effect of the connection loop's
            // filtered connects, and forwarding those would put devices in the pick-list
            // at moments nobody asked for one.
            if !self.active.load(Ordering::Relaxed) {
                continue;
            }

            let name = local_name(report.data);
            let has_name = !name.is_empty();

            // **Not first-wins, and this is the subtle part.** Under active scanning a
            // device answers twice: an `AdvInd`, which usually carries flags and service
            // UUIDs but no name, and then a `ScanRsp`, which is where most scales put
            // theirs. Dedup on first sight alone would therefore capture the nameless
            // report and discard the one with the name, and every device would reach the
            // user as a bare address -- which looks like a name-decoding bug and sends
            // you into `AdStructure` when the fault is here.
            //
            // So: at most two reports per address, the second only if it adds a name.
            {
                let reported = self.reported.borrow();
                match reported.get(&report.addr) {
                    Some(true) => continue,
                    Some(false) if !has_name => continue,
                    _ => {}
                }
            }

            let mut address = [0u8; 6];
            // `raw()` is a slice, not an array, so this is a copy rather than a cast.
            address.copy_from_slice(report.addr.raw());

            let device = DiscoveredBluetoothPeripheral {
                address,
                address_random: report.addr_kind == AddrKind::RANDOM,
                name,
                rssi: report.rssi,
            };

            // A synchronous callback cannot await, so a full queue drops rather than
            // blocking -- blocking here would stall the BLE runner, which is what keeps
            // every existing connection alive.
            //
            // Marked as reported only on success, so a device lost to a full queue is
            // retried on its next advertisement instead of being dropped for the whole
            // scan.
            if self.results.try_send(ScanReport::Discovered(device)).is_err() {
                self.dropped.fetch_add(1, Ordering::Relaxed);
            } else {
                let _ = self.reported.borrow_mut().insert(report.addr, has_name);
            }
        }
    }
}
