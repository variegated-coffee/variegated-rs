//! BLE scanning and device discovery

use core::cell::RefCell;

use variegated_log::{log_info, log_warn};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Receiver};
use heapless::Deque;
use heapless::index_map::FnvIndexMap;
use portable_atomic::{AtomicBool, AtomicU16, Ordering};
use trouble_host::prelude::*;
use variegated_belka_portal_trouble_driver::BELKA_SERVICE_UUID;
use variegated_controller_types::bluetooth::{
    bluetooth_name, BluetoothDriverKind, BluetoothName, DiscoveredBluetoothPeripheral,
};
use variegated_scale_trouble_driver::acaia_new::{
    acaia_generation_from_name, Generation, ACAIA_NEW_SERVICE_UUID,
};
use variegated_scale_trouble_driver::acaia_old::ACAIA_OLD_SERVICE_UUID;
use variegated_scale_trouble_driver::bookoo::BOOKOO_SERVICE_UUID;
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
    /// Addresses reported during the current scan, and what those reports carried --
    /// [`SEEN_NAME`] and [`SEEN_DRIVER`].
    ///
    /// Flags rather than a "have we sent this" bool, because a device's name and its
    /// service UUIDs usually arrive in *different* advertising reports, and both are
    /// worth forwarding. See [`Self::on_adv_reports`].
    reported: RefCell<FnvIndexMap<BdAddr, u8, SCAN_REPORTED_CAPACITY>>,
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

    /// Logged through `variegated_log` rather than left in the manager's `defmt` output,
    /// which this firmware's debug transports do not carry.
    fn attempt_failed(&self, error: Option<&trouble_host::Error>) {
        match error {
            Some(error) => log_warn!("Bluetooth scan attempt refused: {:?}", error),
            None => log_warn!("Bluetooth scan attempt refused by the controller"),
        }
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

    // The connect-lifecycle hooks exist only to reach the sampler: `define_counters!`
    // makes its id enum private to `instrumentation`, and the manager is a third crate
    // besides, so this impl is the one place that can see both ends.
    //
    // Counters rather than log lines, unlike the two above. These fire on every attempt
    // in every pass -- a machine with two peripherals switched off produces them
    // continuously -- so as text they would fill the 16-slot event ring and evict the
    // events worth reading. As a sampled series they are free and can be differenced
    // into a rate.

    fn connect_attempt(&self, _address: BdAddr, waited: embassy_time::Duration) {
        crate::instrumentation::note_ble_connect_attempt(waited);
    }

    fn connect_timed_out(&self, _address: BdAddr) {
        crate::instrumentation::note_ble_connect_timeout();
    }

    fn connect_error(&self, _address: BdAddr) {
        crate::instrumentation::note_ble_connect_error();
    }

    fn connect_abandoned(&self, _address: BdAddr) {
        crate::instrumentation::note_ble_connect_abandoned();
    }
}

/// Weakest signal worth reporting, in dBm.
///
/// A filter, not a preference: reports below this never reach the application processor,
/// which keeps both the UART and the sixteen-entry result list
/// ([`variegated_controller_types::bluetooth::MAX_DISCOVERED_BLUETOOTH_PERIPHERALS`])
/// for devices the user could plausibly be
/// holding. Without it a scan in a flat returns every phone, watch and television in range,
/// and the scale the user actually wants can be crowded out of a bounded list by furniture.
///
/// -70 dBm is roughly "in this room". A peripheral sitting on the machine reads around -50,
/// and -70 is about where the same room stops and the next one begins.
///
/// **This is the only thing protecting that bounded list, which is why it is not generous.**
/// `merge_scan_report` does not evict: once sixteen devices are in, a seventeenth is dropped
/// and counted, so the list is first-come-first-served. In a flat full of phones, watches and
/// televisions the slots fill with whatever advertised first, and the scale someone is
/// standing next to can be locked out of its own pick-list.
///
/// The trade this makes is real and worth stating: the previous -80 existed as margin for a
/// scale in an awkward spot -- in a drawer, behind a portafilter, on the far side of a
/// boiler. Such a scale may now fall below the floor and never appear. **If a scale that is
/// definitely powered on does not show up in a scan, this constant is the first thing to
/// revisit**, and moving it back to -80 costs nothing but list pressure.
const MIN_REPORTED_RSSI: i8 = -70;

/// Which driver, if any, claims a service this device advertised.
///
/// Compares against the driver crates' own UUID constants rather than repeating the
/// numbers, so a driver that changes its service cannot silently stop being recognised
/// here.
///
/// Recognition is a *hint*. Many devices advertise no service UUIDs at all -- the data is
/// optional and the payload is small -- and a scale is perfectly usable without them,
/// since the service is discovered on connect regardless. So this pre-fills the driver
/// and ranks the list; it never decides what the user is allowed to see.
fn driver_for_service(uuid: &Uuid) -> Option<BluetoothDriverKind> {
    if *uuid == ACAIA_OLD_SERVICE_UUID {
        Some(BluetoothDriverKind::AcaiaOld)
    } else if *uuid == ACAIA_NEW_SERVICE_UUID {
        Some(BluetoothDriverKind::AcaiaNew)
    } else if *uuid == BOOKOO_SERVICE_UUID {
        Some(BluetoothDriverKind::Bookoo)
    } else if *uuid == BELKA_SERVICE_UUID {
        Some(BluetoothDriverKind::BelkaPortal)
    } else if *uuid == crate::ble::ulanzi_slot::HID_SERVICE_UUID {
        // The only generic suggestion on this list: every UUID above names one vendor's
        // product, while this one names a whole class of device. A keyboard or a mouse
        // advertising HID would be suggested the dial's driver too.
        //
        // That is worth it because a HID device that is *not* suggested a driver cannot be
        // associated at all, and the cost of a wrong suggestion is low: the user picks the
        // driver, this only pre-fills it, and the dial's decoder rejects frames that are
        // not its own rather than acting on them.
        Some(BluetoothDriverKind::UlanziD100H)
    } else {
        None
    }
}

/// What one advertising payload tells us about a device.
///
/// Decoded in a single pass because a report is decoded on every advertisement from every
/// device in range, which during a scan is a great many.
struct Advertisement {
    /// The advertised local name, or empty if the device advertised none.
    ///
    /// `AdStructure`'s name variants carry raw bytes rather than `&str`, and the bytes
    /// are chosen by whatever device is in radio range -- so this is validated as UTF-8
    /// and truncated on a character boundary. A bare slice would panic on a multi-byte
    /// character straddling the limit.
    name: BluetoothName,
    /// The merged answer -- what the pairing UI should pre-fill.
    driver: Option<BluetoothDriverKind>,
    /// What the advertised service UUIDs alone said, before any name was consulted.
    ///
    /// Kept apart from `driver` solely for [`SEEN_DRIVER`]; see the note there, which
    /// describes the report this would otherwise suppress.
    uuid_driver: Option<BluetoothDriverKind>,
}

fn decode_advertisement(data: &[u8]) -> Advertisement {
    let mut complete: Option<&[u8]> = None;
    let mut shortened: Option<&[u8]> = None;
    let mut driver = None;

    for structure in AdStructure::decode(data) {
        match structure {
            Ok(AdStructure::CompleteLocalName(bytes)) => complete = Some(bytes),
            // First one wins: a device sending two shortened names is malformed, and
            // preferring the earlier is as good an answer as any.
            Ok(AdStructure::ShortenedLocalName(bytes)) if shortened.is_none() => {
                shortened = Some(bytes)
            }
            // Advertised as raw little-endian bytes, so they are rebuilt into a `Uuid`
            // and compared against the drivers' own constants.
            // Both list forms, and that is not belt-and-braces. trouble 0.7 split the old
            // `ServiceUuids16`/`ServiceUuids128` into the `Complete` and `Incomplete`
            // variants the spec has always had, and matching only one of each would silently
            // stop recognising devices: a peripheral with more services than fit its
            // advertisement sends an *incomplete* list, which is exactly the case where the
            // one UUID we care about may still be present.
            Ok(AdStructure::CompleteServiceUuids16(uuids))
            | Ok(AdStructure::IncompleteServiceUuids16(uuids)) => {
                for uuid in uuids {
                    if let Some(found) =
                        driver_for_service(&Uuid::new_short(u16::from_le_bytes(*uuid)))
                    {
                        driver = Some(found);
                    }
                }
            }
            Ok(AdStructure::CompleteServiceUuids128(uuids))
            | Ok(AdStructure::IncompleteServiceUuids128(uuids)) => {
                for uuid in uuids {
                    if let Some(found) = driver_for_service(&Uuid::new_long(*uuid)) {
                        driver = Some(found);
                    }
                }
            }
            _ => {}
        }
    }

    let driver_from_uuids = driver;

    let bytes = complete.or(shortened).unwrap_or(&[]);
    let name = bluetooth_name(core::str::from_utf8(bytes).unwrap_or(""));

    // Prefix-matched over the `BluetoothName` this function was going to build anyway, so
    // the added cost is a handful of `starts_with` on a report that has already passed the
    // discovery-scan check and the RSSI floor. This is not the every-advertisement path.
    let named = driver_for_name(&name);

    // **A service UUID wins, because it cannot be a coincidence.** A device that serves a
    // service is that kind of device; a name is a string a user may have edited in a vendor
    // app, and it has already been truncated to fit.
    //
    // Name matching exists at all because ACAIA's 2021+ scales do not reliably advertise
    // their service, so for them there is often nothing else to go on.
    let driver = driver_from_uuids.or(named);

    Advertisement {
        name,
        driver,
        uuid_driver: driver_from_uuids,
    }
}

/// Which driver, if any, an advertised local *name* implies.
///
/// Names carry more weight for ACAIA than for anything else here: the 2021+ models put their
/// name in the scan response and frequently advertise no service UUID at all, so UUID-only
/// recognition leaves a Pyxis looking like an anonymous device.
///
/// Still only a hint. A Lunar 2021 with AL008 hardware speaks the *older* protocol despite
/// its name, so no name test can be authoritative -- which is why the user can always
/// override the choice, and why this never filters anything out.
fn driver_for_name(name: &str) -> Option<BluetoothDriverKind> {
    match acaia_generation_from_name(name) {
        Some(Generation::Modern) => Some(BluetoothDriverKind::AcaiaNew),
        Some(Generation::Legacy) => Some(BluetoothDriverKind::AcaiaOld),
        None => None,
    }
}

/// This address has been reported with a usable name.
const SEEN_NAME: u8 = 1 << 0;
/// This address has been reported with a recognised service UUID.
const SEEN_DRIVER: u8 = 1 << 1;

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
            // Before the `active` check below, deliberately: this counts what the radio
            // heard, not what the UI was told about. Reports arriving outside a discovery
            // scan are exactly the ones that say the receiver is still working while
            // nobody is asking it for a pick-list.
            crate::instrumentation::note_ble_adv_report();

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

            // Too far away to be the device someone is standing next to. Dropped here
            // rather than in the UI so it costs no UART traffic and, more importantly,
            // no slot in the application processor's bounded result list.
            if report.rssi < MIN_REPORTED_RSSI {
                continue;
            }

            let advertisement = decode_advertisement(report.data);

            let mut flags = 0u8;
            if !advertisement.name.is_empty() {
                flags |= SEEN_NAME;
            }
            // **From a UUID only, never from the name.** A name-derived driver arrives
            // *with* the name and is already accounted for by `SEEN_NAME`; counting it here
            // as well would let a scan response that carries only a recognised name set
            // both bits, after which the `AdvInd` carrying the actual service UUIDs would
            // add nothing new and be suppressed by the dedup below. The UUID evidence would
            // then never reach the application processor at all.
            if advertisement.uuid_driver.is_some() {
                flags |= SEEN_DRIVER;
            }

            // **Not first-wins, and this is the subtle part.** Under active scanning a
            // device answers twice: an `AdvInd`, which usually carries flags and service
            // UUIDs but no name, and then a `ScanRsp`, which is where most scales put
            // theirs. Dedup on first sight alone would capture whichever arrived first
            // and discard the other, so a device would reach the user missing either its
            // name or its driver -- which looks like a decoding bug and sends you into
            // `AdStructure` when the fault is here.
            //
            // So a repeat is forwarded only when it carries something the previous
            // reports did not. The application processor merges rather than replaces, so
            // the two halves add up there; see its `Discovered` arm.
            {
                let reported = self.reported.borrow();
                match reported.get(&report.addr) {
                    // Adds nothing over what has already been sent.
                    Some(seen) if flags & !seen == 0 => continue,
                    _ => {}
                }
            }

            let mut address = [0u8; 6];
            // `raw()` is a slice, not an array, so this is a copy rather than a cast.
            address.copy_from_slice(report.addr.raw());

            let device = DiscoveredBluetoothPeripheral {
                address,
                address_random: report.addr_kind == AddrKind::RANDOM,
                name: advertisement.name,
                rssi: report.rssi,
                suggested_driver: advertisement.driver,
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
                let mut reported = self.reported.borrow_mut();
                let seen = reported.get(&report.addr).copied().unwrap_or(0);
                let _ = reported.insert(report.addr, seen | flags);
            }
        }
    }
}
