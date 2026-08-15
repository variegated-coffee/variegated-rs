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
    pub peripheral_connection_status: FnvIndexMap<PeripheralId, WirelessConnectionStatus, 8>,
    /// Improv provisioning state.
    ///
    /// Reported by the processor that owns the radio rather than inferred from the command
    /// that opened the window, so the machine's display shows what is actually being
    /// advertised. A window the comms processor never opened -- because it reset -- reads
    /// as `Stopped` here, which is what the application processor's fallback deadline is
    /// for.
    pub improv: crate::wifi::ImprovState,
    /// How many times the comms processor has successfully synchronised its clock against
    /// the network, since it booted.
    ///
    /// The application processor re-anchors its own clock when this number *changes*, and
    /// ignores `timestamp` otherwise. The distinction is the whole point of the field.
    /// `timestamp` comes from the ESP32's RTC, which runs off an internal RC oscillator
    /// with percent-level, temperature-dependent error; re-anchoring to it once a second --
    /// which is what the application processor used to do -- imported that error wholesale
    /// and threw away the far better clock it has locally. Anchoring only on a real sync
    /// makes this a correction rather than a leash.
    ///
    /// Zero means "never synced", which is also what `timestamp: None` means; they are
    /// consistent because both are written from the same flag.
    ///
    /// **Appended, not inserted.** postcard is positional and this type crosses the UART
    /// between two separately-flashed binaries.
    pub sntp_sync_seq: u32,
}

#[cfg(feature = "defmt")]
impl defmt::Format for CommsStatus {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "CommsStatus {{ timestamp: {:?}, sntp_sync_seq: {}, wifi_connected: {}, wifi_rssi: {:?}, improv: {:?}, peripherals: {} }}",
            self.timestamp,
            self.sntp_sync_seq,
            self.wifi_connected,
            self.wifi_rssi,
            self.improv,
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
    /// Ask for a summary of every stored routine.
    ///
    /// Answered with [`ApplicationProcessorToCommsProcessorMessage::RoutineSummaries`].
    /// This used to bring back every routine's full definition in one frame; it now
    /// brings back names, types and counts, and a definition is fetched one at a time
    /// through [`Self::RequestRoutineChunk`] when something actually needs one.
    ///
    /// Unlike the other periodic requests on this enum, the comms processor repeats this
    /// one forever rather than stopping once answered, because routines change while the
    /// machine runs. That is affordable only because the reply is now small and the
    /// receiver skips its own work when the list is unchanged.
    RequestRoutines,
    ExternalPeripheralSensorReading(ExternalPeripheralSensorReading),
    /// Ask for the most recent stored shots, newest first.
    ///
    /// **Repurposed in place**, not appended. This position previously held a
    /// `RequestShotLogList` with no payload, against a shot-log design the SD card
    /// replaced. Nothing has ever sent it -- both processors are flashed from the same
    /// tree, and the old variant had no producer on either side -- so reusing the
    /// discriminant costs nothing, where appending would leave a permanent hole. The
    /// same applies to the two below and to the three replies.
    RequestShotLogList { limit: u16 },
    /// Ask for `SHOT_LOG_CHUNK_LEN` bytes of a stored shot, starting at `offset`.
    ///
    /// The download is a sequence of these rather than one message: a stored shot runs
    /// to tens of kilobytes and the link's accumulator is 4096 bytes on both ends.
    RequestShotLogChunk {
        id: crate::shot_log::ShotLogId,
        offset: u32,
    },
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
    /// Ask for the stored Wi-Fi credentials.
    ///
    /// Appended, not inserted -- see the note on [`Self::DebugCommand`].
    ///
    /// Sent at boot and repeated until answered, for the same reason
    /// [`Self::RequestBluetoothPeripherals`] is: this processor has no persistent storage,
    /// so until it is told it has no network at all. **`None` is a complete answer**, so
    /// the retry must stop on receipt rather than on credentials being present.
    RequestWifiCredentials,
    /// Credentials that have been *proven* -- the radio associated with them.
    ///
    /// Appended, not inserted -- see the note on [`Self::DebugCommand`].
    ///
    /// Only sent after a successful association, which is why the application processor can
    /// persist it without validating anything: a typo never gets this far. Improv requires
    /// the device to verify before it reports `Provisioned`, so the check has to happen on
    /// this side regardless.
    WifiCredentialsProvisioned(crate::wifi::WifiCredentials),
    /// A client sent the Improv Identify RPC.
    ///
    /// Appended, not inserted -- see the note on [`Self::DebugCommand`].
    WifiProvisioningIdentify,
    /// Ask for one slice of a routine's definition.
    ///
    /// Appended, not inserted -- see the note on [`Self::DebugCommand`].
    ///
    /// Answered with [`ApplicationProcessorToCommsProcessorMessage::RoutineChunk`], or
    /// [`ApplicationProcessorToCommsProcessorMessage::RoutineNotFound`] if the index is
    /// empty.
    RequestRoutineChunk { index: RoutineIndex, offset: u16 },
    /// One slice of a routine on its way to storage.
    ///
    /// Appended, not inserted -- see the note on [`Self::DebugCommand`].
    ///
    /// `index: None` creates, letting the repository assign an index; `Some` replaces the
    /// routine already there. `total` is the full encoded length, carried on every chunk
    /// so an oversized routine can be refused on the first one rather than after
    /// reassembling all of it.
    ///
    /// The chunk is smaller than [`ApplicationProcessorToCommsProcessorMessage::RoutineChunk`]'s
    /// -- see [`crate::ROUTINE_WRITE_CHUNK_LEN`] for why the two directions are not
    /// symmetric.
    ///
    /// This does not travel as a [`MachineCommand`], despite `AddRoutine` and
    /// `UpdateRoutine` existing: those carry a decoded `Routine`, which would put the
    /// decode back on the comms processor that this change exists to keep out of the
    /// business of understanding routines. They remain on the wire for the debug CLI,
    /// which injects them from a host that has memory to spare.
    RoutineWriteChunk {
        index: Option<RoutineIndex>,
        offset: u16,
        total: u16,
        last: bool,
        bytes: heapless::Vec<u8, { crate::ROUTINE_WRITE_CHUNK_LEN }>,
    },
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone)]
pub enum ApplicationProcessorToCommsProcessorMessage {
    Hello(ProtocolConfig),
    Status(Status),
    MachineDefinition(MachineDefinition),
    Configuration(Configuration),
    /// A summary of every stored routine: enough to list them, not enough to run or edit
    /// one.
    ///
    /// **Repurposed in place**, replacing `Routines(RoutineList)` -- the payload kept its
    /// role but not its shape, exactly as the three shot-log variants below did. Both
    /// processors are flashed from the same tree, which is what makes reusing a
    /// discriminant cheaper than appending and leaving a permanent hole.
    ///
    /// The definitions themselves now travel one at a time as opaque bytes, through
    /// [`Self::RoutineChunk`]. The comms processor never read them: it re-keyed the map
    /// by [`RoutineIndex`] and forwarded it, which cost several deep copies of every
    /// routine every fifteen seconds to serve a frontend that wants one at a time.
    ///
    /// Sent in answer to [`CommsProcessorToApplicationProcessorMessage::RequestRoutines`],
    /// and **unsolicited** after a write, so a change announces itself rather than
    /// waiting out the poll interval.
    RoutineSummaries(RoutineSummaryList),
    /// The most recent stored shots, newest first, with their annotations.
    ///
    /// **Repurposed in place** -- see
    /// [`CommsProcessorToApplicationProcessorMessage::RequestShotLogList`]. The payload
    /// type kept its name but not its shape.
    ShotLogList(ShotLogList),
    /// One slice of a stored shot, as it sits on the card.
    ///
    /// **Repurposed in place**, replacing `ShotLogEntry`.
    ///
    /// `total` and `last` both travel because they answer different questions: `total`
    /// lets a receiver show progress from the first chunk, and `last` terminates the
    /// loop without arithmetic on a value it would otherwise have to trust. `id` and
    /// `offset` are echoed because this protocol has no correlation id -- they are the
    /// only way a reply can be matched to the request that asked for it.
    ShotLogChunk {
        id: crate::shot_log::ShotLogId,
        offset: u32,
        total: u32,
        last: bool,
        bytes: heapless::Vec<u8, { crate::shot_log::SHOT_LOG_CHUNK_LEN }>,
    },
    /// The annotations on one stored shot.
    ///
    /// **Repurposed in place**, replacing `ShotLogEntryDataPoint`.
    ///
    /// Separate from the listing so that editing a shot's annotations can be confirmed
    /// without re-reading every other shot on the card. It echoes the block *as stored*
    /// rather than acknowledging the request, so a client sees any entry the machine
    /// refused for want of room -- which a bare ack would hide.
    ShotLogAnnotations {
        id: crate::shot_log::ShotLogId,
        annotations: crate::shot_log::ShotAnnotations,
    },
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
    /// A shot-log request could not be answered.
    ///
    /// Appended, not inserted -- see the note on
    /// [`CommsProcessorToApplicationProcessorMessage::DebugCommand`]. It is appended
    /// rather than slotted beside the three repurposed shot-log replies above because
    /// those occupy fixed historical positions; only the end of the enum is safe.
    ///
    /// It exists so a failure is distinguishable from silence. The shot-log protocol has
    /// no correlation id and the comms processor waits on a timeout, so without an
    /// explicit refusal a request against an empty card slot would block for the full
    /// timeout and then be indistinguishable from a dead application processor -- and the
    /// user would be told "the machine is not responding" about a machine that is fine and
    /// simply has no card in it.
    ///
    /// No `id` field: the errors worth reporting (`CardNotPresent`, `NotExfat`,
    /// `BusUnavailable`) are properties of the card rather than of the shot asked for, and
    /// with one request in flight at a time there is nothing to disambiguate against.
    ShotLogError(crate::shot_log::ShotLogStorageError),
    /// The stored Wi-Fi credentials, or `None` if none are configured.
    ///
    /// Appended, not inserted -- see the note on
    /// [`CommsProcessorToApplicationProcessorMessage::DebugCommand`].
    ///
    /// Sent in answer to [`CommsProcessorToApplicationProcessorMessage::RequestWifiCredentials`]
    /// and unprompted whenever they change, so a machine provisioned over Improv joins its
    /// new network without waiting to be asked again.
    ///
    /// Deliberately *not* carried inside `Configuration`, unlike the Bluetooth association
    /// list: `Configuration` is what the browser receives, and a password has no business
    /// on that path.
    WifiCredentials(Option<crate::wifi::WifiCredentials>),
    /// Advertise the Improv service for `duration_ms`, accepting credentials while it lasts.
    ///
    /// Appended, not inserted -- see the note on
    /// [`CommsProcessorToApplicationProcessorMessage::DebugCommand`].
    ///
    /// **The window is the authorization.** Nothing advertises until a human held a button
    /// on the machine, so a client that can see the service is one a person deliberately
    /// exposed. The comms processor times the window out itself, because it owns the radio;
    /// the application processor keeps its own deadline only as a fallback for a comms
    /// processor that resets mid-window.
    OpenWifiProvisioningWindow { duration_ms: u32 },
    /// Stop advertising now.
    ///
    /// Appended, not inserted -- see the note on
    /// [`CommsProcessorToApplicationProcessorMessage::DebugCommand`].
    CloseWifiProvisioningWindow,
    /// One slice of a routine's postcard encoding, exactly as it would be stored.
    ///
    /// Appended, not inserted -- see the note on
    /// [`CommsProcessorToApplicationProcessorMessage::DebugCommand`].
    ///
    /// **Bytes, not a `Routine`.** The only consumer is an HTTP response body on the
    /// comms processor, so decoding here would be to re-encode identically a moment
    /// later -- and that processor's heap is shared with Wi-Fi, BLE and the ESPHome
    /// server, where a `Routine` costs five to ten allocations per step. This is the same
    /// reasoning `handle_get_shot` already runs on: the download *is* the stored record.
    ///
    /// Chunked, and echoing `index` and `offset`, for the same reasons
    /// [`Self::ShotLogChunk`] does -- this protocol has no correlation id, and those two
    /// fields are the only way a reply can be matched to its request. A maximal routine
    /// is two chunks.
    RoutineChunk {
        index: RoutineIndex,
        offset: u16,
        total: u16,
        last: bool,
        bytes: heapless::Vec<u8, { crate::ROUTINE_CHUNK_LEN }>,
    },
    /// There is no routine at that index.
    ///
    /// Appended, not inserted -- see the note on
    /// [`CommsProcessorToApplicationProcessorMessage::DebugCommand`].
    ///
    /// Separate from an empty [`Self::RoutineChunk`] so that "absent" and "zero bytes"
    /// cannot be confused, and so a request for a deleted routine is a clean 404 rather
    /// than a five-second wait on a timeout.
    RoutineNotFound(RoutineIndex),
    /// The outcome of a [`CommsProcessorToApplicationProcessorMessage::RoutineWriteChunk`]
    /// sequence.
    ///
    /// Appended, not inserted -- see the note on
    /// [`CommsProcessorToApplicationProcessorMessage::DebugCommand`].
    ///
    /// [`RoutineWriteOutcome::Stored`] carries the index actually written, which on a
    /// create is the one the repository *assigned* -- and is the only way the caller
    /// learns it. Before this existed, a save was fire-and-forget and a routine too large
    /// to persist was indistinguishable from one that stored cleanly.
    RoutineWriteResult(RoutineWriteOutcome),
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
