use crate::*;
use alloc::vec::Vec;
// Only used by the hand-written `defmt::Format` impl for `CommsStatus` below, so it
// has to carry the same gate -- an unconditional import made the crate fail to build
// with the `defmt` feature off.
#[cfg(feature = "defmt")]
use defmt::Debug2Format;

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone,  Debug)]
pub struct ExternalPeripheralSensorReading {
    pub id: PeripheralId,
    pub endpoint: u8,
    pub value: f32,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
// `PartialEq` is needed because `Status` reaches this type and `Status` now travels
// inside `DebugPayload`, whose `PartialEq` derive is what the codec round-trip tests
// assert on.
#[derive(Clone, Debug, PartialEq)]
pub struct CommsStatus {
    pub timestamp: Option<u64>, // Unix timestamp in seconds
    pub wifi_connected: bool,
    pub wifi_rssi: Option<i8>, // RSSI in dBm, None when disconnected
    pub peripheral_connection_status: FnvIndexMap<PeripheralId, WirelessConnectionStatus, 8>
}

#[cfg(feature = "defmt")]
impl defmt::Format for CommsStatus {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "CommsStatus {{ timestamp: {:?}, wifi_connected: {}, wifi_rssi: {:?}, peripherals: {} }}",
            self.timestamp,
            self.wifi_connected,
            self.wifi_rssi,
            Debug2Format(&self.peripheral_connection_status),
        )
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone,  Debug, Copy, PartialEq)]
pub struct WirelessConnectionStatus {
    pub connected: bool,
    pub rssi: Option<i8>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone)]
pub enum CommsProcessorToApplicationProcessorMessage {
    Command(MachineCommand),
    CommsStatus(CommsStatus),
    RequestStatus,
    RequestMachineDefinition,
    RequestConfiguration,
    RequestRoutines,
    ExternalPeripheralSensorReading(ExternalPeripheralSensorReading),
    RequestShotLogList,
    RequestShotLogEntry(u32),
    /// Debug command injected from a host via the comms processor.
    ///
    /// Appended, not inserted: postcard encodes an enum as its *declaration-order*
    /// discriminant, so putting this anywhere but the end would renumber every
    /// variant after it and silently mis-decode on any peer built from a different
    /// commit. The same applies to every future variant.
    DebugCommand(crate::debug_command::DebugCommand),
    /// Ask for the Bluetooth peripheral associations.
    ///
    /// Appended, not inserted -- see the note on [`Self::DebugCommand`].
    ///
    /// The comms processor sends this at boot and repeats it until answered, because it
    /// has no persistent storage of its own: the association list is the *only* thing
    /// that tells it which Bluetooth devices exist, and until it arrives that processor
    /// has no peripherals at all. An empty list is a complete answer, so the retry has
    /// to stop on receipt rather than on the list being non-empty.
    RequestBluetoothPeripherals,
    /// A device seen during a discovery scan.
    ///
    /// Appended, not inserted -- see the note on [`Self::DebugCommand`].
    ///
    /// One message per device rather than a list at the end of the scan, so results
    /// appear in the UI as they are found rather than eight seconds later. Devices are
    /// de-duplicated on the comms side, but a device may legitimately be reported twice:
    /// once from its advertisement and again from its scan response, which is where most
    /// scales put their name.
    BluetoothPeripheralDiscovered(crate::bluetooth::DiscoveredBluetoothPeripheral),
    /// A discovery scan has ended.
    ///
    /// Appended, not inserted -- see the note on [`Self::DebugCommand`].
    ///
    /// `reports_dropped` counts advertising reports the comms processor observed but
    /// could not forward because its outbound queue was full. Carried rather than
    /// dropped silently so that a scan which found nothing is distinguishable from one
    /// that found too much.
    BluetoothScanFinished { reports_dropped: u16 },
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone)]
pub enum ApplicationProcessorToCommsProcessorMessage {
    Hello(ProtocolConfig),
    Status(Status),
    MachineDefinition(MachineDefinition),
    Configuration(Configuration),
    Routines(RoutineList),
    ShotLogList(ShotLogList),
    ShotLogEntry(ShotLogEntry),
    ShotLogEntryDataPoint(ShotLogEntryDataPoint),
    /// Structured debug frames relayed to the comms processor for TCP fan-out.
    ///
    /// Appended, not inserted -- see the note on
    /// [`CommsProcessorToApplicationProcessorMessage::DebugCommand`].
    ///
    /// `DebugPayload::Status` never appears here: the comms processor already gets
    /// `Status` through the `Status` variant above, and a `Status` frame cannot fit
    /// the relay's per-window byte budget anyway. See
    /// `variegated_debug::relay::relayable`.
    Debug(crate::debug::DebugFrame),
    /// An operation on a scale owned by the comms processor.
    ///
    /// Appended, not inserted -- see the note on
    /// [`CommsProcessorToApplicationProcessorMessage::DebugCommand`].
    ///
    /// The [`PeripheralId`] is mandatory and positional because there is no default
    /// scale: the comms processor names three scale roles (group 1, group 2, and a
    /// dose scale), and an operation that arrived without one would have to guess
    /// which to act on. It is the same id the readings travel under in
    /// [`ExternalPeripheralSensorReading`], so commands and measurements address a
    /// scale the same way in both directions.
    ScaleCommand(PeripheralId, ScaleOp),
    /// The current Bluetooth peripheral associations.
    ///
    /// Appended, not inserted -- see the note on
    /// [`CommsProcessorToApplicationProcessorMessage::DebugCommand`].
    ///
    /// Sent both in answer to
    /// [`CommsProcessorToApplicationProcessorMessage::RequestBluetoothPeripherals`] and
    /// unprompted whenever the list changes, so that associating a scale takes effect
    /// without waiting for the comms processor to ask again.
    ///
    /// The list also rides inside [`Configuration`], which is what the browser reads.
    /// That is not redundancy for its own sake: the two consumers want different
    /// cadences. The browser wants the list alongside everything else it renders, and
    /// the comms processor wants a small message it can act on the moment an address
    /// changes, without a whole-configuration republish in the way.
    BluetoothPeripherals(crate::bluetooth::BluetoothPeripheralList),
    /// Run a Bluetooth discovery scan for `duration_ms`.
    ///
    /// Appended, not inserted -- see the note on
    /// [`CommsProcessorToApplicationProcessorMessage::DebugCommand`].
    ///
    /// Originates as a user action in the UI, and is routed through this processor
    /// rather than handled where it arrives. That costs a round trip, and buys the
    /// only thing that can prevent a scan from breaking a shot: a discovery scan
    /// occupies a radio shared with Wi-Fi and with the live links to the scales
    /// themselves, and *this* is the processor that knows whether coffee is being made.
    StartBluetoothScan { duration_ms: u16 },
}

/// An operation on a scale, as carried by
/// [`ApplicationProcessorToCommsProcessorMessage::ScaleCommand`].
///
/// Only `Tare` today, because it is the only one the ACAIA driver can perform --
/// there is no zero-calibration or reference-weight command in that protocol. The
/// enum exists rather than a bare "tare" message so that a scale which *does* support
/// calibration can be added without a second wire variant. Append, never insert.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ScaleOp {
    Tare,
}
