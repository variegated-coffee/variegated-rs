//! Associations between Bluetooth peripherals and the roles they fill.
//!
//! # What an association is
//!
//! The comms processor used to carry its peripherals' addresses as constants, which
//! meant changing a scale meant reflashing. These types are what replaced that: the
//! application processor persists a list of associations, pushes it over the
//! inter-processor link, and the comms processor derives its BLE peripheral set from
//! whatever it was last told.
//!
//! An association binds an *address and a driver* to a *[`PeripheralId`]*, and the
//! peripheral id is a **role** -- the group-1 scale, the dose scale, the water sensor --
//! not a make of device. That framing is not new here; it is what the ids have always
//! meant, and it is what keeps this feature from reaching the application processor's
//! device layer at all. `PeripheralRegistry` still registers its providers statically at
//! boot, readings still arrive tagged with an id, and a role that has no association
//! simply never receives any. All of the runtime rebinding happens on the far side of
//! the link.

use crate::PeripheralId;

#[cfg(feature = "sequential-storage")]
use sequential_storage::map::{SerializationError, Value};
#[cfg(feature = "sequential-storage")]
use postcard::{to_slice_crc32, from_bytes_crc32};
#[cfg(feature = "sequential-storage")]
use crc::{Crc, CRC_32_ISCSI};

/// How many peripherals may be associated at once.
///
/// Four covers the roles this firmware names -- two group scales, a dose scale and the
/// water sensor -- and it is not an arbitrary ceiling: the comms processor sizes its BLE
/// `HostResources` at this plus one slot of margin, and each additional slot costs about
/// 576 bytes of SRAM taken directly out of the stack. Raising it means re-measuring
/// there first.
pub const MAX_BLUETOOTH_PERIPHERALS: usize = 4;

/// Longest peripheral name carried on the wire.
///
/// Bounded rather than a `String` because these travel inside `Status` and
/// `Configuration`, which are already the two largest messages on the link -- and those are
/// held inline in several statics on the comms processor, where `.stack` is whatever RWDATA
/// is left after `.bss`, so a byte here is a byte several times over.
///
/// It was also once bounded by the WebSocket's 256-byte inbound frame, which an association
/// command had to fit in one of. That constraint is gone -- inbound frames are heap-backed
/// now -- but the size argument above never depended on it and still holds on its own.
///
/// It is also a *trust* boundary. The default name is whatever a device advertised, so
/// any radio in range chooses these bytes; truncation to this length must be done on a
/// UTF-8 character boundary rather than by slicing.
pub const BLUETOOTH_NAME_LEN: usize = 24;

pub type BluetoothName = heapless::String<BLUETOOTH_NAME_LEN>;

/// Build a [`BluetoothName`], truncating to fit.
///
/// The truncation is the reason this exists rather than being written out at each call
/// site. Names come from advertising data, so any radio in range chooses these bytes,
/// and the obvious `&s[..BLUETOOTH_NAME_LEN]` **panics** the moment one of them lands
/// mid-character -- a two-byte `é` at offset 23 is enough. Walking back to a character
/// boundary costs nothing and takes the whole class of input out of play.
///
/// Truncating rather than rejecting: a name is a label. A scale called something long
/// should appear in the list shortened, not vanish from it.
pub fn bluetooth_name(s: &str) -> BluetoothName {
    let mut end = s.len().min(BLUETOOTH_NAME_LEN);
    while end > 0 && !s.is_char_boundary(end) {
        end -= 1;
    }
    // Infallible by construction: `end <= BLUETOOTH_NAME_LEN` and sits on a boundary.
    BluetoothName::try_from(&s[..end]).unwrap_or_default()
}

/// The driver that speaks to an associated peripheral.
///
/// Distinct from [`crate::PeripheralType`], which says what a peripheral *is* to the
/// machine (a scale, a level sensor) and is part of the machine definition. This says
/// which protocol implementation to instantiate, which is a question only the comms
/// processor asks.
///
/// **Append, never insert.** postcard encodes an enum as its declaration-order
/// discriminant, so reordering silently mis-decodes on any peer built from a different
/// commit.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum BluetoothDriverKind {
    /// Belka Portal water sensor: electrical conductivity, temperature, battery.
    #[default]
    BelkaPortal,
    /// ACAIA scales speaking the older of the two ACAIA protocols.
    AcaiaOld,
    /// BooKoo Themis scales: the Themis, the Themis Mini and the Themis Ultra, which share
    /// one weight frame and one command set.
    Bookoo,
    /// ACAIA scales from 2021 onwards: Pyxis, Lunar AL014+, Pearl 2021, Pearl S, Cinco.
    ///
    /// A separate driver from [`Self::AcaiaOld`] because the GATT topology differs -- a
    /// vendor service with separate notify and write characteristics, where the older
    /// protocol used one characteristic for both -- not because the commands do; those are
    /// byte-identical.
    ///
    /// Note that a Lunar 2021 with AL008 hardware speaks the *older* protocol, so the model
    /// year does not settle which of the two a scale needs.
    AcaiaNew,
}

/// One entry in the association list.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct BluetoothPeripheralAssociation {
    /// The role this peripheral fills. **The identity of the association**: creating one
    /// for an id that already has an association replaces it, and the comms processor's
    /// slot bookkeeping is keyed on this rather than on a position in the list.
    pub id: PeripheralId,
    /// Bluetooth device address, most significant byte first, as it appears in an
    /// advertising report.
    ///
    /// A plain `[u8; 6]` rather than a `BdAddr`, because `BdAddr` belongs to `bt-hci`
    /// and this processor has no Bluetooth stack to take it from. The conversion happens
    /// once, on the comms side, at the point the address reaches the radio.
    pub address: [u8; 6],
    /// Whether `address` is a random address rather than a public one.
    ///
    /// Recorded because the advertising report carries it and nothing else does -- once
    /// a scan result has been discarded there is no way to recover it short of scanning
    /// again. The connection manager can still connect without it (it offers both kinds
    /// in its accept list), so this is an optimisation and a fallback for entries that
    /// predate it, not a requirement.
    pub address_random: bool,
    /// Which driver to run against this peripheral.
    pub driver: BluetoothDriverKind,
    /// Whether to connect at all.
    ///
    /// The reason this is a flag rather than "delete the association" is the rescan:
    /// switching a scale off for a week should not cost the user a discovery pass and a
    /// re-pairing when they want it back.
    pub enabled: bool,
    /// Display name. Defaults to whatever the device advertised, and the user may change
    /// it.
    ///
    /// **Deliberately not part of the peripheral's identity, and not connection-relevant.**
    /// Renaming an association must not disturb a live connection, which means the comms
    /// processor's change detection has to compare the other fields specifically rather
    /// than deriving equality from this struct as a whole.
    pub name: BluetoothName,
}

pub type BluetoothPeripheralList =
    heapless::Vec<BluetoothPeripheralAssociation, MAX_BLUETOOTH_PERIPHERALS>;

/// A device seen during a discovery scan.
///
/// Not an association: this is the raw observation the user picks from.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct DiscoveredBluetoothPeripheral {
    pub address: [u8; 6],
    pub address_random: bool,
    /// The advertised local name, or empty if the device advertised none.
    pub name: BluetoothName,
    /// Signal strength of the report this entry was built from, in dBm.
    ///
    /// First observation wins rather than strongest -- see the reporting path on the
    /// comms processor for why. Good enough to sort a pick-list by, which is all it is
    /// for; not a measurement.
    pub rssi: i8,
    /// The driver whose service this device advertised, if the comms processor
    /// recognised one.
    ///
    /// **`None` does not mean "unsupported".** Plenty of devices advertise a name and no
    /// service UUID at all, and a scale that does so is still perfectly usable -- the
    /// service is discovered on connect either way. So this is a hint for ranking and for
    /// pre-filling the driver, never a reason to hide a device: the failure it would
    /// cause is the user's own scale missing from the list with no way to add it.
    pub suggested_driver: Option<BluetoothDriverKind>,
}

/// How many scan results are carried at once.
///
/// A pick-list, not a census. `Status` is already the largest message on the link, and
/// a user choosing their scale out of a list is not helped by the ninetieth device in
/// range.
pub const MAX_DISCOVERED_BLUETOOTH_PERIPHERALS: usize = 16;

/// Discovery state, as surfaced in [`crate::Status`].
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BluetoothScanStatus {
    /// A scan is running right now.
    pub scanning: bool,
    /// The most recent scan request was refused because the machine was busy.
    ///
    /// A discovery scan is a heavy user of a radio that is shared with Wi-Fi and with
    /// the live links to the peripherals themselves, and losing a scale mid-shot breaks
    /// brew-by-weight. Only this processor knows a shot is in progress, so only this
    /// processor can refuse. Cleared when a scan is next accepted.
    pub blocked: bool,
    /// Reports the comms processor observed but could not forward, because its outbound
    /// queue was full.
    ///
    /// Reported rather than hidden so that a scan which found nothing is
    /// distinguishable from one that found too much.
    pub reports_dropped: u16,
    /// What the last (or current) scan has found so far.
    pub discovered:
        heapless::Vec<DiscoveredBluetoothPeripheral, MAX_DISCOVERED_BLUETOOTH_PERIPHERALS>,
}

#[cfg(feature = "defmt")]
impl defmt::Format for BluetoothScanStatus {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "BluetoothScanStatus {{ scanning: {}, blocked: {}, reports_dropped: {}, discovered: {} }}",
            self.scanning,
            self.blocked,
            self.reports_dropped,
            self.discovered.len(),
        );
    }
}

/// One peripheral, as handed to whichever worker is responsible for connecting to it.
///
/// **Everything here affects the connection, and nothing that does not is here.** That
/// is the whole design: `PartialEq` on this type *is* the "does this need a reconnect?"
/// test, so it cannot be got wrong by comparing the wrong subset of fields.
///
/// The two omissions are deliberate:
///
/// - `name` is absent because renaming a peripheral must not disturb a live connection.
///   Had this held the whole association, deriving equality would tear down and rebuild
///   the link every time a user corrected a typo in a label -- on a scale, mid-shot.
/// - `enabled` is absent because a disabled association is simply not assigned. Carrying
///   the flag down here would mean holding a slot open for a peripheral that has been
///   told not to connect, and there are only [`MAX_BLUETOOTH_PERIPHERALS`] of them.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BluetoothSlotAssignment {
    pub id: PeripheralId,
    pub address: [u8; 6],
    pub address_random: bool,
    pub driver: BluetoothDriverKind,
}

/// Which worker serves which peripheral.
pub type BluetoothSlotAssignments = [Option<BluetoothSlotAssignment>; MAX_BLUETOOTH_PERIPHERALS];

/// Fold an association list into an existing set of slot claims, returning whether
/// anything changed.
///
/// # Why claims are keyed on `PeripheralId` and not on list position
///
/// The obvious mapping -- worker *i* serves association *i* -- is wrong in a way that
/// only shows up in use. The list is ordered by whoever edited it, so deleting the first
/// of three associations renumbers the other two; every worker then sees a different
/// assignment, tears down, and reconnects. **Removing one peripheral would drop the link
/// to every other one**, including a scale in the middle of a shot. Claims keyed on the
/// id survive their neighbours being added and removed.
///
/// Lives here, rather than beside the workers that consume it, because it is pure and it
/// is the part most worth testing: both failure modes above are silent, and neither is
/// reachable from a smoke test.
pub fn reconcile_bluetooth_slots(
    assignments: &mut BluetoothSlotAssignments,
    list: &BluetoothPeripheralList,
    // Called for each entry that is refused, so the caller can log it. Refusals are not
    // returned as an error because they do not stop the reconcile -- the other
    // associations are still valid and still want connecting.
    mut rejected: impl FnMut(PeripheralId, BluetoothSlotRejection),
) -> bool {
    let previous = *assignments;

    // Pass 1: what the list asks for, with duplicates refused.
    //
    // Two associations sharing an id, or sharing an address, are both refused rather
    // than merged. The address case is the dangerous one: a connection manager keys its
    // device table on the address, so two workers claiming one address would have each
    // one's release tear down the other's live connection. Keep-first, because keep-last
    // would let a bad new entry displace a working one.
    //
    // Disabled associations never enter this list. That is what makes "disabled" release
    // a slot for someone else rather than hold one open.
    let mut wanted: heapless::Vec<BluetoothSlotAssignment, MAX_BLUETOOTH_PERIPHERALS> =
        heapless::Vec::new();
    for association in list.iter() {
        if !association.enabled {
            continue;
        }
        let candidate = BluetoothSlotAssignment {
            id: association.id,
            address: association.address,
            address_random: association.address_random,
            driver: association.driver,
        };
        if wanted.iter().any(|w| w.id == candidate.id) {
            rejected(candidate.id, BluetoothSlotRejection::DuplicateId);
            continue;
        }
        if wanted.iter().any(|w| w.address == candidate.address) {
            rejected(candidate.id, BluetoothSlotRejection::DuplicateAddress);
            continue;
        }
        if wanted.push(candidate).is_err() {
            rejected(candidate.id, BluetoothSlotRejection::NoFreeSlot);
        }
    }

    // Pass 2: release claims whose peripheral is gone; update those still wanted on
    // different terms.
    //
    // Updated in place rather than released and re-assigned, so a peripheral that merely
    // moved to a new address keeps its slot, and so a slot never briefly reads as idle
    // for a peripheral that is still configured.
    for assignment in assignments.iter_mut() {
        let Some(current) = *assignment else { continue };
        match wanted.iter().find(|w| w.id == current.id) {
            Some(want) if *want == current => {}
            Some(want) => *assignment = Some(*want),
            None => *assignment = None,
        }
    }

    // Pass 3: everything still unclaimed takes the lowest free slot.
    for want in wanted.iter() {
        if assignments.iter().any(|a| a.map(|a| a.id) == Some(want.id)) {
            continue;
        }
        match assignments.iter_mut().find(|a| a.is_none()) {
            Some(free) => *free = Some(*want),
            None => rejected(want.id, BluetoothSlotRejection::NoFreeSlot),
        }
    }

    previous != *assignments
}

/// Why an association did not get a slot.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BluetoothSlotRejection {
    /// Another association already claims this peripheral id.
    DuplicateId,
    /// Another association already claims this address.
    DuplicateAddress,
    /// More enabled associations than there are slots.
    NoFreeSlot,
}

/// Progress of a discovery scan, as it arrives from the comms processor.
///
/// Carried by [`crate::MachineCommand::UpdateBluetoothScan`], which is not a user
/// command despite living in that enum -- it is the same shape as
/// [`crate::MachineCommand::UpdateCommsStatus`], and for the same reason: the comms
/// processor's only route into the controller is the command channel, and this state has
/// to reach the controller because the controller is what assembles `Status`.
///
/// **Append, never insert.**
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BluetoothScanUpdate {
    /// One device, reported as it is seen rather than batched at the end of the scan.
    Discovered(DiscoveredBluetoothPeripheral),
    /// The scan is over. `reports_dropped` counts what the comms processor saw but could
    /// not forward.
    Finished { reports_dropped: u16 },
}

/// The association list as it is stored in flash.
///
/// A newtype rather than a bare [`BluetoothPeripheralList`] purely so it can carry a
/// `Value` impl: `sequential_storage::map::Value` is a foreign trait and `heapless::Vec`
/// is a foreign type. Having it also lets the list reuse the existing
/// `SequentialStorageSettingsStorage`, which is generic over exactly this shape --
/// `Default + Clone + PartialEq + Value` written as a whole blob -- rather than needing
/// a store of its own.
///
/// It lives in its own flash range rather than inside the machine's persistent
/// configuration, and that is the whole reason it is a separate blob. postcard is
/// positional and these blobs carry no version or migration, so appending a field to the
/// existing configuration would make every previously stored copy fail to deserialize
/// and fall back to `Default` -- silently resetting every boiler and PID setting on the
/// first boot after the upgrade.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BluetoothAssociations(pub BluetoothPeripheralList);

impl BluetoothAssociations {
    /// Insert or replace the association for `association.id`.
    ///
    /// Returns `false` if the list is full and the id was not already present. Replacing
    /// is not a special case the caller asked for -- an id names a role, and a role
    /// cannot be filled twice, so re-associating is the only sensible reading of
    /// "associate a device with a peripheral that already has one".
    pub fn upsert(&mut self, association: BluetoothPeripheralAssociation) -> bool {
        if let Some(existing) = self.0.iter_mut().find(|a| a.id == association.id) {
            *existing = association;
            return true;
        }
        self.0.push(association).is_ok()
    }

    /// Remove the association for `id`, returning whether there was one.
    pub fn remove(&mut self, id: PeripheralId) -> bool {
        match self.0.iter().position(|a| a.id == id) {
            Some(index) => {
                self.0.remove(index);
                true
            }
            None => false,
        }
    }

    /// Set the enabled flag for `id`, returning whether there was an association to set
    /// it on.
    pub fn set_enabled(&mut self, id: PeripheralId, enabled: bool) -> bool {
        match self.0.iter_mut().find(|a| a.id == id) {
            Some(association) => {
                association.enabled = enabled;
                true
            }
            None => false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn association(
        id: PeripheralId,
        address_last_byte: u8,
        enabled: bool,
    ) -> BluetoothPeripheralAssociation {
        BluetoothPeripheralAssociation {
            id,
            address: [0x11, 0x22, 0x33, 0x44, 0x55, address_last_byte],
            address_random: true,
            driver: BluetoothDriverKind::AcaiaOld,
            enabled,
            name: bluetooth_name("scale"),
        }
    }

    fn list(items: &[BluetoothPeripheralAssociation]) -> BluetoothPeripheralList {
        BluetoothPeripheralList::from_slice(items).expect("fits")
    }

    fn reconcile(
        assignments: &mut BluetoothSlotAssignments,
        items: &[BluetoothPeripheralAssociation],
    ) -> bool {
        reconcile_bluetooth_slots(assignments, &list(items), |_, _| {})
    }

    /// The failure this whole design exists to prevent.
    ///
    /// With claims keyed on list position, deleting the first of three associations
    /// renumbers the rest and every worker sees a changed assignment -- so removing one
    /// peripheral drops the link to every other one, including a scale mid-shot.
    #[test]
    fn removing_one_peripheral_leaves_the_others_where_they_were() {
        let mut assignments = BluetoothSlotAssignments::default();
        reconcile(
            &mut assignments,
            &[association(0xB5C0, 1, true), association(0xB5C1, 2, true), association(0xB1CA, 3, true)],
        );
        let before = assignments;

        // Drop the *first* entry, which is the case position-keying gets wrong.
        let changed = reconcile(
            &mut assignments,
            &[association(0xB5C1, 2, true), association(0xB1CA, 3, true)],
        );

        assert!(changed);
        assert_eq!(assignments[0], None, "the removed peripheral's slot should be free");
        assert_eq!(
            (assignments[1], assignments[2]),
            (before[1], before[2]),
            "the surviving peripherals moved slots, which would drop their connections"
        );
    }

    /// A rename must not reach the workers at all: an association differing only in its
    /// name produces no change, so nothing reconnects.
    #[test]
    fn renaming_a_peripheral_changes_nothing() {
        let mut assignments = BluetoothSlotAssignments::default();
        reconcile(&mut assignments, &[association(0xB5C0, 1, true)]);
        let before = assignments;

        let mut renamed = association(0xB5C0, 1, true);
        renamed.name = bluetooth_name("Definitely a different label");

        assert!(!reconcile(&mut assignments, &[renamed]), "a rename asked for a reconnect");
        assert_eq!(assignments, before);
    }

    /// Two associations on one address would have each worker's release tear down the
    /// other's live connection, because the connection manager keys its device table on
    /// the address. Keep-first: a bad new entry must not displace a working one.
    #[test]
    fn a_duplicate_address_is_refused_and_the_first_claim_survives() {
        let mut assignments = BluetoothSlotAssignments::default();
        let mut rejections = alloc::vec::Vec::new();
        reconcile_bluetooth_slots(
            &mut assignments,
            &list(&[association(0xB5C0, 1, true), association(0xB5C1, 1, true)]),
            |id, why| rejections.push((id, why)),
        );

        assert_eq!(rejections, [(0xB5C1, BluetoothSlotRejection::DuplicateAddress)]);
        assert_eq!(assignments[0].map(|a| a.id), Some(0xB5C0));
        assert_eq!(assignments[1], None);
    }

    #[test]
    fn a_duplicate_peripheral_id_is_refused() {
        let mut assignments = BluetoothSlotAssignments::default();
        let mut rejections = alloc::vec::Vec::new();
        reconcile_bluetooth_slots(
            &mut assignments,
            &list(&[association(0xB5C0, 1, true), association(0xB5C0, 2, true)]),
            |id, why| rejections.push((id, why)),
        );

        assert_eq!(rejections, [(0xB5C0, BluetoothSlotRejection::DuplicateId)]);
        assert_eq!(assignments[0].map(|a| a.address[5]), Some(1));
        assert_eq!(assignments[1], None);
    }

    /// Disabling frees the slot rather than holding it, which is what lets a user park a
    /// peripheral they are not using without spending one of four.
    #[test]
    fn disabling_releases_the_slot_and_re_enabling_takes_one_again() {
        let mut assignments = BluetoothSlotAssignments::default();
        reconcile(&mut assignments, &[association(0xB5C0, 1, true)]);
        assert_eq!(assignments[0].map(|a| a.id), Some(0xB5C0));

        assert!(reconcile(&mut assignments, &[association(0xB5C0, 1, false)]));
        assert_eq!(assignments[0], None);

        assert!(reconcile(&mut assignments, &[association(0xB5C0, 1, true)]));
        assert_eq!(assignments[0].map(|a| a.id), Some(0xB5C0));
    }

    /// A peripheral that moved to a new address keeps its slot. Releasing and
    /// re-assigning would make it briefly read as unassigned, and the application
    /// processor drops readings for a peripheral missing from the status map.
    #[test]
    fn a_re_addressed_peripheral_keeps_its_slot() {
        let mut assignments = BluetoothSlotAssignments::default();
        reconcile(&mut assignments, &[association(0xB5C0, 1, true), association(0xB1CA, 2, true)]);

        assert!(reconcile(
            &mut assignments,
            &[association(0xB5C0, 9, true), association(0xB1CA, 2, true)]
        ));

        assert_eq!(assignments[0].map(|a| a.id), Some(0xB5C0));
        assert_eq!(assignments[0].map(|a| a.address[5]), Some(9));
        assert_eq!(assignments[1].map(|a| a.id), Some(0xB1CA));
    }

    /// Reconciling the same list twice must report no change, or every republish of an
    /// unchanged configuration would bounce every connection.
    #[test]
    fn an_unchanged_list_reports_no_change() {
        let mut assignments = BluetoothSlotAssignments::default();
        let items = [association(0xB5C0, 1, true), association(0xB1CA, 2, true)];
        assert!(reconcile(&mut assignments, &items));
        assert!(!reconcile(&mut assignments, &items));
    }
}

// Mirrors the impl on `ScheduleItem`; see the note there about the consumed-length
// return that sequential-storage 6.0 introduced.
#[cfg(feature = "sequential-storage")]
impl<'a> Value<'a> for BluetoothAssociations {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        let crc = Crc::<u32>::new(&CRC_32_ISCSI);

        match to_slice_crc32(self, buffer, crc.digest()) {
            Ok(bytes) => Ok(bytes.len()),
            Err(postcard::Error::SerializeBufferFull) => Err(SerializationError::BufferTooSmall),
            Err(_) => Err(SerializationError::InvalidData),
        }
    }

    fn deserialize_from(buffer: &'a [u8]) -> Result<(Self, usize), SerializationError>
    where
        Self: Sized,
    {
        let crc = Crc::<u32>::new(&CRC_32_ISCSI);

        match from_bytes_crc32(buffer, crc.digest()) {
            // `from_bytes_crc32` reads the whole slice, the trailing four bytes being
            // the CRC, and the slice handed in is exactly what `serialize_into`
            // produced.
            Ok(value) => Ok((value, buffer.len())),
            Err(_) => Err(SerializationError::InvalidFormat),
        }
    }
}
