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
/// `Configuration`, which are already the two largest messages on the link, and because
/// the WebSocket server accepts client frames of at most 256 bytes -- an association
/// command has to fit in one.
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
