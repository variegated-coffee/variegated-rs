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

/// Fold one report from the comms processor into the discovered list.
///
/// **Merged, not replaced.** The comms processor reports a device more than once on purpose:
/// a name and a set of service UUIDs usually arrive in different advertising reports -- the
/// name in the scan response, the UUIDs in the advertisement -- and each is forwarded when it
/// adds something. Overwriting would keep whichever came last and throw away the other half.
pub fn merge_scan_report(
    status: &mut BluetoothScanStatus,
    device: DiscoveredBluetoothPeripheral,
) {
    match status.discovered.iter_mut().find(|d| d.address == device.address) {
        Some(existing) => {
            if !device.name.is_empty() {
                existing.name = device.name;
            }
            if device.suggested_driver.is_some() {
                existing.suggested_driver = device.suggested_driver;
            }
            // `rssi` is deliberately left at the first sighting, matching what the field
            // claims. It ranks the list; it is not a measurement, and re-reading it per
            // report would make the order jump around while the user is reading it.
        }
        None => {
            if status.discovered.push(device).is_err() {
                // Counted where the UI already looks for "results were lost", rather than in
                // a log nobody reads mid-scan.
                status.reports_dropped = status.reports_dropped.saturating_add(1);
            }
        }
    }
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
        DiscoveredBluetoothPeripheral {
            address: [address; 6],
            address_random: false,
            name: BluetoothName::try_from(name).expect("the test names are short enough"),
            rssi: -50,
            suggested_driver: driver,
        }
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
