//! The five Bluetooth commands.
//!
//! A single-boiler machine has a comms processor like any other, so it can carry a Bluetooth
//! scale or a water sensor; none of this was ever dual-boiler-specific, and it was written
//! twice anyway. The scan-report merge below is the part worth having in one place: forty
//! lines of "which half of this device's identity arrived in which advertising report",
//! duplicated, with nothing checking either copy.
//!
//! The one genuine difference between the machines is what counts as *busy*, so that is a
//! parameter rather than something this module tries to work out.

use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::channel::Sender;
use embassy_time::{Duration, Instant};
use variegated_controller_types::bluetooth::{
    BluetoothAssociations, BluetoothPeripheralAssociation, BluetoothScanStatus,
    BluetoothScanUpdate, DiscoveredBluetoothPeripheral,
};
use variegated_controller_types::PeripheralId;
use variegated_log::{log_info, log_warn};

use crate::{BLUETOOTH_SCAN_DURATION_MS, BLUETOOTH_SCAN_SLACK_MS};

/// Whether the caller should write the association list to its store.
///
/// `#[must_use]` for the same reason as the other flags in this module's siblings: the list
/// lives in RAM as well as in flash, so a dropped `Persist` looks entirely correct until the
/// machine is power-cycled and the pairing is gone.
#[must_use = "an association that is not persisted is lost at the next power cycle"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Persist(bool);

impl Persist {
    /// The list changed and should be written.
    pub const YES: Self = Self(true);
    /// Nothing changed.
    pub const NO: Self = Self(false);

    /// Whether a write is wanted.
    pub const fn wanted(self) -> bool {
        self.0
    }
}

/// `AssociateBluetoothPeripheral`.
pub fn associate(
    associations: &mut BluetoothAssociations,
    association: BluetoothPeripheralAssociation,
) -> Persist {
    let id = association.id;
    if associations.upsert(association) {
        log_info!("Associated Bluetooth peripheral 0x{:04X}", id);
        Persist::YES
    } else {
        // Only reachable when the list is full *and* the id is new, since an existing id
        // replaces in place.
        log_warn!("Cannot associate 0x{:04X}: no free Bluetooth peripheral slots", id);
        Persist::NO
    }
}

/// `RemoveBluetoothPeripheral`.
pub fn remove(associations: &mut BluetoothAssociations, id: PeripheralId) -> Persist {
    if associations.remove(id) {
        log_info!("Removed Bluetooth association 0x{:04X}", id);
        Persist::YES
    } else {
        log_warn!("No Bluetooth association for 0x{:04X} to remove", id);
        Persist::NO
    }
}

/// `SetBluetoothPeripheralEnabled`.
pub fn set_enabled(
    associations: &mut BluetoothAssociations,
    id: PeripheralId,
    enabled: bool,
) -> Persist {
    if associations.set_enabled(id, enabled) {
        log_info!("Bluetooth association 0x{:04X} enabled={}", id, enabled);
        Persist::YES
    } else {
        log_warn!("No Bluetooth association for 0x{:04X} to enable/disable", id);
        Persist::NO
    }
}

/// `ScanForBluetoothPeripherals`.
///
/// A discovery scan monopolises a radio shared with Wi-Fi and with the live links to the
/// peripherals themselves, and the ACAIA driver drops its connection if it misses a couple of
/// heartbeats -- so a scan started mid-shot can cost brew-by-weight the shot it is weighing.
/// The comms processor cannot see any of that; this is the only processor that knows coffee
/// is being made, which is why it is the one that refuses.
///
/// **`busy` is the caller's to decide.** It is the one thing the two machines genuinely
/// disagree about: one reads three flags and a steam wand, the other matches on a state enum.
/// Everything after it is the same on both.
pub fn start_scan<M: RawMutex>(
    status: &mut BluetoothScanStatus,
    deadline: &mut Option<Instant>,
    sender: Option<Sender<'_, M, u16, 2>>,
    busy: bool,
) {
    if busy {
        log_warn!("Refusing Bluetooth scan: machine is busy");
        status.blocked = true;
        return;
    }

    let Some(sender) = sender else {
        log_warn!("Refusing Bluetooth scan: no comms processor wired for it");
        status.blocked = true;
        return;
    };

    match sender.try_send(BLUETOOTH_SCAN_DURATION_MS) {
        Ok(()) => {
            log_info!("Starting Bluetooth scan");
            status.blocked = false;
            status.scanning = true;
            status.reports_dropped = 0;
            // Cleared on *start*, not on finish. The user is about to pick from this list,
            // and leaving the previous scan's results visible underneath the new ones would
            // offer them devices that may no longer be there.
            status.discovered.clear();
            *deadline = Some(
                Instant::now()
                    + Duration::from_millis(
                        BLUETOOTH_SCAN_DURATION_MS as u64 + BLUETOOTH_SCAN_SLACK_MS,
                    ),
            );
        }
        Err(_) => log_warn!("Failed to start Bluetooth scan: channel full"),
    }
}

/// Where a device sits in the pick-list: **recognised first, then strongest signal first.**
///
/// Lower sorts earlier. `suggested_driver.is_none()` rather than `is_some()` because `false
/// < true`, so recognised devices come first; [`core::cmp::Reverse`] on the RSSI because a
/// stronger signal is a larger (less negative) number and should sort earlier.
///
/// The address is a final tiebreak, and it is not decoration. Without it two devices at the
/// same RSSI and the same recognition compare equal, and an unstable sort may put them in
/// either order -- so the list could reshuffle under the user's finger on any report, for no
/// reason they could see. With it the order is a total function of the contents.
///
/// This is the same rule the web UI sorts by. The duplication is deliberate: this copy
/// decides which devices are *kept*, which the UI cannot do because it only ever sees the
/// survivors.
fn scan_rank(
    device: &DiscoveredBluetoothPeripheral,
) -> (bool, core::cmp::Reverse<i8>, [u8; 6]) {
    (
        device.suggested_driver.is_none(),
        core::cmp::Reverse(device.rssi),
        device.address,
    )
}

/// Fold one report from the comms processor into the discovered list.
///
/// **Merged, not replaced.** The comms processor reports a device more than once on purpose:
/// a name and a set of service UUIDs usually arrive in different advertising reports -- the
/// name in the scan response, the UUIDs in the advertisement -- and each is forwarded when it
/// adds something. Overwriting would keep whichever came last and throw away the other half.
///
/// **The list holds the best sixteen, not the first sixteen.** It used to simply refuse a
/// device once full, which made discovery first-come-first-served: in a flat full of phones,
/// watches and televisions the slots filled with whatever advertised first, and the scale the
/// user was standing next to could be locked out of its own pick-list with no way to reach
/// it. Now a device that ranks above the current worst evicts it -- so a recognised scale
/// arriving seventeenth still gets in, and what it displaces is the weakest anonymous device
/// in the room.
///
/// The RSSI floor on the comms processor is the other half of this and still matters: it
/// keeps the far-away devices from reaching here at all, which is cheaper than admitting and
/// then evicting them.
pub fn merge_scan_report(
    status: &mut BluetoothScanStatus,
    device: DiscoveredBluetoothPeripheral,
) {
    // The index rather than a `&mut`, so the borrow ends before the arms below need to read
    // the list's length and its other entries.
    let existing = status
        .discovered
        .iter()
        .position(|d| d.address == device.address);

    match existing {
        Some(index) => {
            let existing = &mut status.discovered[index];
            if !device.name.is_empty() {
                existing.name = device.name;
            }
            if device.suggested_driver.is_some() {
                // This can promote a device out of the unrecognised group, which is why the
                // sort below runs on every report and not only on insertion.
                existing.suggested_driver = device.suggested_driver;
            }
            // `rssi` is deliberately left at the first sighting, matching what the field
            // claims. It ranks the list; it is not a measurement, and re-reading it per
            // report would make the order jump around while the user is reading it.
        }
        None if status.discovered.len() < status.discovered.capacity() => {
            // Cannot fail: the guard above is exactly the condition `push` checks.
            let _ = status.discovered.push(device);
        }
        None => {
            let worst = status
                .discovered
                .iter()
                .enumerate()
                .max_by_key(|(_, existing)| scan_rank(existing))
                .map(|(index, _)| index);

            // Counted whichever way it goes. Something *was* lost -- either this device or
            // the one it displaced -- and the count is what tells the user the list is not
            // the whole room. Where the UI already looks, rather than a log nobody reads
            // mid-scan.
            status.reports_dropped = status.reports_dropped.saturating_add(1);

            if let Some(worst) = worst {
                let worst_rank = scan_rank(&status.discovered[worst]);
                // Strictly better, so a device that ties with the worst entry does not
                // displace it. Ties would otherwise thrash: two equal devices could take
                // turns evicting each other on every report.
                if scan_rank(&device) < worst_rank {
                    status.discovered[worst] = device;
                }
            }
        }
    }

    // Sorted here rather than in the consumer, so the bound above and the order the user
    // sees are the same rule -- "the best sixteen" is only meaningful if something defines
    // best. Sixteen entries, and the order is a pure function of the contents, so this
    // neither costs anything nor makes the list move for its own sake.
    status.discovered.sort_unstable_by_key(scan_rank);
}

/// `UpdateBluetoothScan` -- progress relayed from the comms processor.
pub fn apply_scan_update(
    status: &mut BluetoothScanStatus,
    deadline: &mut Option<Instant>,
    update: BluetoothScanUpdate,
) {
    match update {
        BluetoothScanUpdate::Discovered(device) => merge_scan_report(status, device),
        BluetoothScanUpdate::Finished { reports_dropped } => {
            log_info!(
                "Bluetooth scan finished: {} found, {} dropped by the comms processor",
                status.discovered.len(),
                reports_dropped
            );
            *deadline = None;
            status.scanning = false;
            // Added to, not replaced: this side drops reports of its own when the list is
            // full, and those are just as lost as the ones the comms processor never sent.
            status.reports_dropped = status.reports_dropped.saturating_add(reports_dropped);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use variegated_controller_types::bluetooth::{BluetoothDriverKind, BluetoothName};

    fn device(address: u8, name: &str, driver: Option<BluetoothDriverKind>) -> DiscoveredBluetoothPeripheral {
        device_at(address, name, driver, -50)
    }

    fn device_at(
        address: u8,
        name: &str,
        driver: Option<BluetoothDriverKind>,
        rssi: i8,
    ) -> DiscoveredBluetoothPeripheral {
        DiscoveredBluetoothPeripheral {
            address: [address; 6],
            address_random: false,
            name: BluetoothName::try_from(name).expect("the test names are short enough"),
            rssi,
            suggested_driver: driver,
        }
    }

    /// Fill the list with unrecognised devices at a uniform signal strength.
    fn fill_with_unrecognised(status: &mut BluetoothScanStatus, rssi: i8) {
        let capacity = status.discovered.capacity();
        for address in 0..capacity {
            merge_scan_report(status, device_at(address as u8, "junk", None, rssi));
        }
        assert_eq!(status.discovered.len(), capacity);
    }

    fn holds(status: &BluetoothScanStatus, address: u8) -> bool {
        status.discovered.iter().any(|d| d.address == [address; 6])
    }

    /// The whole point of the eviction: a scale that advertises seventeenth still gets in.
    ///
    /// It is *weaker* than everything already in the list, deliberately -- recognition beats
    /// signal, because a recognised peripheral is the thing the user opened this list to
    /// find and an anonymous television is not.
    #[test]
    fn a_recognised_device_evicts_an_unrecognised_one_even_when_weaker() {
        let mut status = BluetoothScanStatus::default();
        fill_with_unrecognised(&mut status, -60);

        merge_scan_report(
            &mut status,
            device_at(200, "Pyxis", Some(BluetoothDriverKind::AcaiaNew), -80),
        );

        assert!(holds(&status, 200), "the recognised scale was refused");
        assert_eq!(status.discovered[0].address, [200; 6], "and it should rank first");
        assert_eq!(status.discovered.len(), status.discovered.capacity());
    }

    /// Within a group, a stronger signal displaces a weaker one.
    #[test]
    fn a_stronger_device_evicts_a_weaker_one_of_the_same_kind() {
        let mut status = BluetoothScanStatus::default();
        fill_with_unrecognised(&mut status, -60);

        merge_scan_report(&mut status, device_at(200, "closer", None, -40));

        assert!(holds(&status, 200));
        assert_eq!(status.discovered[0].address, [200; 6]);
    }

    /// And the converse, which is what stops a distant room from evicting the kitchen.
    #[test]
    fn a_weaker_device_is_refused_when_the_list_is_full() {
        let mut status = BluetoothScanStatus::default();
        fill_with_unrecognised(&mut status, -50);

        merge_scan_report(&mut status, device_at(200, "far away", None, -70));

        assert!(!holds(&status, 200), "a weaker device should not displace anything");
        assert_eq!(status.reports_dropped, 1);
    }

    /// A tie does not displace, because ties would thrash: two equal devices could take
    /// turns evicting each other on every report, and the list would never settle.
    #[test]
    fn a_tie_does_not_displace_the_incumbent() {
        let mut status = BluetoothScanStatus::default();
        fill_with_unrecognised(&mut status, -50);
        let before = status.discovered.clone();

        // Address 200 sorts *after* every incumbent on the tiebreak, so it ranks worse.
        merge_scan_report(&mut status, device_at(200, "identical", None, -50));

        assert_eq!(status.discovered, before, "the list moved for a device it kept out");
        assert_eq!(status.reports_dropped, 1);
    }

    /// Eviction is still a loss, and the counter is what tells the user the list is not the
    /// whole room. It moves whichever device ends up discarded.
    #[test]
    fn eviction_counts_as_a_dropped_report() {
        let mut status = BluetoothScanStatus::default();
        fill_with_unrecognised(&mut status, -60);

        merge_scan_report(&mut status, device_at(200, "closer", None, -40));

        assert_eq!(status.reports_dropped, 1);
    }

    /// Recognition arriving in a later report promotes the device, which is why the sort
    /// runs on every report rather than only on insertion.
    #[test]
    fn recognition_promotes_a_device_up_the_list() {
        let mut status = BluetoothScanStatus::default();
        merge_scan_report(&mut status, device_at(1, "loud telly", None, -40));
        merge_scan_report(&mut status, device_at(2, "quiet scale", None, -75));
        assert_eq!(status.discovered[0].address, [1; 6], "strongest first, so far");

        merge_scan_report(
            &mut status,
            device_at(2, "", Some(BluetoothDriverKind::Bookoo), -75),
        );

        assert_eq!(
            status.discovered[0].address,
            [2; 6],
            "once recognised it should outrank a stronger anonymous device"
        );
    }

    /// The order itself: recognised first, then by descending signal.
    #[test]
    fn the_list_is_ordered_recognised_first_then_by_signal() {
        let mut status = BluetoothScanStatus::default();
        merge_scan_report(&mut status, device_at(1, "", None, -30));
        merge_scan_report(&mut status, device_at(2, "", Some(BluetoothDriverKind::Bookoo), -70));
        merge_scan_report(&mut status, device_at(3, "", None, -50));
        merge_scan_report(&mut status, device_at(4, "", Some(BluetoothDriverKind::AcaiaNew), -60));

        let order: heapless::Vec<u8, 8> =
            status.discovered.iter().map(|d| d.address[0]).collect();
        assert_eq!(order.as_slice(), &[4, 2, 1, 3]);
    }

    /// Two devices that tie on both recognition and signal must land in a defined order, or
    /// the list reshuffles under the user's finger on any report for no visible reason.
    #[test]
    fn a_tie_is_broken_deterministically_by_address() {
        let mut first = BluetoothScanStatus::default();
        merge_scan_report(&mut first, device_at(7, "", None, -55));
        merge_scan_report(&mut first, device_at(3, "", None, -55));

        let mut second = BluetoothScanStatus::default();
        merge_scan_report(&mut second, device_at(3, "", None, -55));
        merge_scan_report(&mut second, device_at(7, "", None, -55));

        assert_eq!(first.discovered, second.discovered, "insertion order leaked into the list");
        assert_eq!(first.discovered[0].address, [3; 6]);
    }

    /// A later report with no name does not blank the name already stored.
    ///
    /// The exact case the merge exists for: the name arrives in the scan response and the
    /// service UUIDs in the advertisement, so one of the two reports always lacks a name.
    #[test]
    fn a_nameless_report_does_not_blank_a_stored_name() {
        let mut status = BluetoothScanStatus::default();
        merge_scan_report(&mut status, device(1, "Lunar", None));

        merge_scan_report(&mut status, device(1, "", Some(BluetoothDriverKind::AcaiaOld)));

        assert_eq!(status.discovered.len(), 1, "the device was duplicated");
        assert_eq!(status.discovered[0].name.as_str(), "Lunar");
        assert_eq!(status.discovered[0].suggested_driver, Some(BluetoothDriverKind::AcaiaOld));
    }

    /// And the other direction: a later report carrying a driver fills it in.
    #[test]
    fn a_later_report_fills_in_a_missing_driver() {
        let mut status = BluetoothScanStatus::default();
        merge_scan_report(&mut status, device(2, "", None));

        merge_scan_report(&mut status, device(2, "Pyxis", Some(BluetoothDriverKind::AcaiaOld)));

        assert_eq!(status.discovered[0].name.as_str(), "Pyxis");
        assert_eq!(status.discovered[0].suggested_driver, Some(BluetoothDriverKind::AcaiaOld));
    }

    /// `rssi` stays at the first sighting, so the pick-list does not reorder while it is read.
    #[test]
    fn rssi_stays_at_the_first_sighting() {
        let mut status = BluetoothScanStatus::default();
        merge_scan_report(&mut status, device(3, "Lunar", None));

        let mut stronger = device(3, "Lunar", None);
        stronger.rssi = -20;
        merge_scan_report(&mut status, stronger);

        assert_eq!(status.discovered[0].rssi, -50);
    }

    /// Two different addresses are two different devices.
    #[test]
    fn distinct_addresses_are_distinct_devices() {
        let mut status = BluetoothScanStatus::default();
        merge_scan_report(&mut status, device(1, "Lunar", None));
        merge_scan_report(&mut status, device(2, "Pyxis", None));

        assert_eq!(status.discovered.len(), 2);
    }

    /// Once the list is full, further *new* devices are counted rather than lost silently.
    #[test]
    fn a_full_list_counts_dropped_reports() {
        let mut status = BluetoothScanStatus::default();
        let capacity = status.discovered.capacity();
        for address in 0..capacity {
            merge_scan_report(&mut status, device(address as u8, "d", None));
        }
        assert_eq!(status.reports_dropped, 0, "nothing should have been dropped yet");

        merge_scan_report(&mut status, device(u8::MAX, "one too many", None));

        assert_eq!(status.discovered.len(), capacity);
        assert_eq!(status.reports_dropped, 1);
    }

    /// A full list still *merges* into a device it already holds.
    ///
    /// Worth its own test: the drop counter is on the insert path, and a merge that fell into
    /// it would report a loss that did not happen while silently discarding the update.
    #[test]
    fn a_full_list_still_merges_a_known_device() {
        let mut status = BluetoothScanStatus::default();
        let capacity = status.discovered.capacity();
        for address in 0..capacity {
            merge_scan_report(&mut status, device(address as u8, "", None));
        }

        merge_scan_report(&mut status, device(0, "Lunar", None));

        assert_eq!(status.reports_dropped, 0);
        assert_eq!(status.discovered[0].name.as_str(), "Lunar");
    }

    /// The comms processor's dropped count is added to this side's, not substituted for it.
    #[test]
    fn finishing_a_scan_adds_the_two_dropped_counts() {
        let mut status = BluetoothScanStatus::default();
        status.reports_dropped = 2;
        status.scanning = true;
        let mut deadline = Some(Instant::from_ticks(1));

        apply_scan_update(
            &mut status,
            &mut deadline,
            BluetoothScanUpdate::Finished { reports_dropped: 3 },
        );

        assert_eq!(status.reports_dropped, 5);
        assert!(!status.scanning);
        assert_eq!(deadline, None, "the failsafe deadline should be disarmed");
    }

    /// A busy machine refuses, says so in the status, and does not arm the deadline.
    #[test]
    fn a_busy_machine_refuses_a_scan() {
        let mut status = BluetoothScanStatus::default();
        let mut deadline = None;

        start_scan::<embassy_sync::blocking_mutex::raw::NoopRawMutex>(
            &mut status, &mut deadline, None, true,
        );

        assert!(status.blocked);
        assert!(!status.scanning);
        assert_eq!(deadline, None);
    }

    /// So does a machine with no comms processor wired for it.
    #[test]
    fn a_machine_with_no_scan_channel_refuses() {
        let mut status = BluetoothScanStatus::default();
        let mut deadline = None;

        start_scan::<embassy_sync::blocking_mutex::raw::NoopRawMutex>(
            &mut status, &mut deadline, None, false,
        );

        assert!(status.blocked);
        assert!(!status.scanning);
        assert_eq!(deadline, None);
    }

    /// An accepted scan clears the previous results before the new ones arrive.
    #[test]
    fn starting_a_scan_clears_the_previous_results() {
        use embassy_sync::blocking_mutex::raw::NoopRawMutex;
        use embassy_sync::channel::Channel;

        let channel: Channel<NoopRawMutex, u16, 2> = Channel::new();
        let mut status = BluetoothScanStatus::default();
        merge_scan_report(&mut status, device(1, "from the last scan", None));
        status.reports_dropped = 4;
        status.blocked = true;
        let mut deadline = None;

        start_scan(&mut status, &mut deadline, Some(channel.sender()), false);

        assert!(status.scanning);
        assert!(!status.blocked);
        assert!(status.discovered.is_empty(), "last scan's devices are still listed");
        assert_eq!(status.reports_dropped, 0);
        assert!(deadline.is_some(), "the failsafe deadline should be armed");
    }

    /// Associating into a full list is refused rather than silently dropped.
    #[test]
    fn associations_report_whether_they_changed_anything() {
        let mut associations = BluetoothAssociations::default();

        assert_eq!(remove(&mut associations, 0x5C1E), Persist::NO);
        assert_eq!(set_enabled(&mut associations, 0x5C1E, false), Persist::NO);
    }
}
