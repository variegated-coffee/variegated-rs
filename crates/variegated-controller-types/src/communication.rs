use crate::*;
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
    /// The network the station is associated with, or empty when it is not.
    ///
    /// This is the SSID the station was *configured* with, mirrored from
    /// `wifi::apply_configuration`, not one read back from the driver -- so it is only
    /// meaningful while [`Self::wifi_connected`]. The comms processor clears it rather than
    /// letting it stand, because a name left behind after a disconnect claims a connection
    /// that does not exist, and this is displayed on a panel with no other way to say
    /// "not this one".
    ///
    /// **Appended, not inserted**, for the reason [`Self::sntp_sync_seq`] gives.
    pub wifi_ssid: heapless::String<{ crate::wifi::WIFI_SSID_LEN }>,
    /// The DHCP address, or `None` before the lease is granted and again once it is lost.
    ///
    /// `[u8; 4]` rather than an address type because nothing in this crate has one, and
    /// [`crate::debug::CommsState::wifi_ip`] already made exactly this choice for exactly
    /// this value. `None` rather than `0.0.0.0`: that address means "this host on this
    /// network" and would render as a plausible-looking lie.
    ///
    /// **Appended, not inserted.**
    pub wifi_ip: Option<[u8; 4]>,
}

#[cfg(feature = "defmt")]
impl defmt::Format for CommsStatus {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "CommsStatus {{ timestamp: {:?}, sntp_sync_seq: {}, wifi_connected: {}, wifi_rssi: {:?}, wifi_ssid: {}, wifi_ip: {:?}, improv: {:?}, peripherals: {} }}",
            self.timestamp,
            self.sntp_sync_seq,
            self.wifi_connected,
            self.wifi_rssi,
            self.wifi_ssid.as_str(),
            self.wifi_ip,
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
    ///
    /// **Repurposed a second time.** It carried a bare `limit` until paging existed; the
    /// payload is now a whole [`crate::shot_log::ShotLogListRequest`], carrying the
    /// cursor and the day filter as well. Same discriminant, same reasoning as before --
    /// both processors are flashed from this tree.
    RequestShotLogList(crate::shot_log::ShotLogListRequest),
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
    /// Ask for the stored shot-log upload configuration.
    ///
    /// Appended, not inserted -- see the note on [`Self::DebugCommand`].
    ///
    /// Sent at boot and repeated until answered, exactly like
    /// [`Self::RequestWifiCredentials`], and carrying the same trap: **a config with both
    /// fields `None` is a complete answer**, so the retry stops on receipt rather than on
    /// the endpoint being present. A machine that has never been configured for uploads
    /// would otherwise ask forever.
    RequestShotUploadConfig,
    /// Remove the routine at an index, and say whether it went.
    ///
    /// Appended, not inserted -- see the note on [`Self::DebugCommand`].
    ///
    /// Answered with [`ApplicationProcessorToCommsProcessorMessage::RoutineDeleteResult`].
    ///
    /// This does not travel as a [`MachineCommand`], despite `RemoveRoutine` existing, and
    /// for the reason [`Self::RoutineWriteChunk`] does not use `AddRoutine`: a command is
    /// fire-and-forget, so a delete that failed on a worn flash sector was indistinguishable
    /// from one that succeeded. `RemoveRoutine` stays on the wire for the debug CLI, which
    /// has no reply path to want.
    DeleteRoutine(RoutineIndex),
    /// A UI command from an input device the comms processor owns.
    ///
    /// Appended, not inserted -- see the note on [`Self::DebugCommand`].
    ///
    /// This does not travel as a [`MachineCommand`], despite that being the enum for
    /// "something happened, do this". A `MachineCommand` reaches the *controller*, and
    /// these have to reach the input task, which is where the menu state a `-` or a
    /// `Select` means anything against actually lives. Routing them through the controller
    /// would mean either duplicating that state or handing the controller a second copy of
    /// the UI, and `MachineCommand` also travels the debug wire, so widening it costs a
    /// [`crate::debug::DEBUG_PROTOCOL_VERSION`] bump for a message that wire has no use for.
    ///
    /// The [`PeripheralId`] travels for the reason a sensor reading carries one: a second
    /// input device then costs an association, not another message.
    InputEvent(PeripheralId, InputCommand),
    /// Ask for the stored Bluetooth bonds.
    ///
    /// Appended, not inserted -- see the note on [`Self::DebugCommand`].
    ///
    /// Sent at boot and repeated until answered, for the same reason
    /// [`Self::RequestBluetoothPeripherals`] is: the comms processor has no persistent
    /// storage, so the application processor's copy is the only one. It carries the same
    /// trap too -- **an empty list is a complete answer**, so the retry has to stop on
    /// receipt rather than on the list being non-empty. A machine whose only paired device
    /// is an unbonded scale would otherwise ask forever.
    RequestBluetoothBonds,
    /// A bond that has just been formed, on its way to flash.
    ///
    /// Appended, not inserted -- see the note on [`Self::DebugCommand`].
    ///
    /// Sent once, by the side that did the pairing. There is no reply and no retry: the
    /// only cost of losing one is that the next reboot re-pairs, which is the behaviour
    /// this message exists to avoid rather than a failure it has to survive.
    BluetoothBondStored(crate::bluetooth::BluetoothBond),
}

/// What an input device asks the machine's UI to do.
///
/// Semantic rather than physical. Each control firmware maps these onto whatever its own
/// input hardware would have produced -- the GS3 onto the panel button that means the same
/// thing, the Silvia onto an encoder detent -- so one dial means on each machine what that
/// machine's own controls mean, and neither firmware grows a second UI.
///
/// This is deliberately the panel's vocabulary and not the menu's. `-`, `+`, Select and
/// Return are what the buttons *mean*; every screen reads that one vocabulary against
/// whatever it is showing, and the idle screen -- which reads them as "run routine 0-3" --
/// is one screen's reading of it rather than a mode. So nothing here has to know whether a
/// menu is open, and no injection site branches on it.
///
/// **Append-only**, for the reason given on
/// [`CommsProcessorToApplicationProcessorMessage::DebugCommand`].
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InputCommand {
    /// `n` steps in the "more / next" direction -- the GS3's `+`, a clockwise detent.
    ///
    /// Batched by the comms processor rather than sent one per step. The first device is a
    /// *stepless* dial, so a quick turn produces a burst of reports at whatever rate the
    /// wheel is moving; forwarding each one would spend a link frame per report and hand
    /// the UI a backlog it works through after the user has stopped turning.
    Increment(u8),
    /// `n` steps in the "less / previous" direction. See [`Self::Increment`].
    Decrement(u8),
    /// Confirm, select, enter -- the GS3's button 3, the Silvia's encoder press.
    Activate,
    /// Back out one level. The GS3's button 4; **ignored on the Silvia**, whose encoder has
    /// one falling edge and no gesture to hang a second meaning on.
    Return,
    /// Open the menu -- what holding the GS3's button 5 does.
    ///
    /// A command of its own because the menu opens on a *hold*, and a device that sends
    /// discrete commands has no hold to send. Without it a dial could only ever reach the
    /// idle screen's reading of the vocabulary. Ignored on the Silvia, where the encoder
    /// press already enters the menu from idle and [`Self::Activate`] therefore covers it.
    Menu,
    /// Reserved. Carried and logged, bound to nothing.
    ///
    /// Here now rather than later so that giving it a meaning is not a wire-format change.
    /// The first device has four keys beyond those the variants above account for, so the
    /// question this answers is already real; what it should *do* is not settled.
    Custom(u8),
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
    /// An operation on a brew sensor owned by the comms processor.
    ///
    /// Appended, not inserted -- see the note on [`Self::ScaleCommand`].
    ///
    /// A sibling of that variant rather than another [`ScaleOp`]: a Belka Portal is not a
    /// scale, and `ScaleOp`'s own doc is explicit about what it covers. Until now
    /// `ScaleCommand` was the only peripheral-directed message in this enum; this is the
    /// second, and it carries a [`PeripheralId`] for the same reason -- there is no default
    /// peripheral, and the id is the one the readings already travel under in
    /// [`ExternalPeripheralSensorReading`].
    BrewSensorCommand(PeripheralId, BrewSensorOp),
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
    /// A shot was stored or deleted.
    ///
    /// Appended, not inserted -- see the note on
    /// [`CommsProcessorToApplicationProcessorMessage::DebugCommand`].
    ///
    /// Unsolicited, like [`Self::BluetoothPeripherals`] and unlike the four shot-log
    /// replies above it. It travels on its own channel rather than on the reply path for
    /// a reason worth keeping: that path has no correlation id, so an unsolicited message
    /// arriving on it can be collected by a client waiting on a listing.
    ShotLogEvent(crate::shot_log::ShotLogEvent),
    /// Where to upload finished shot logs, and the token that authorises it.
    ///
    /// Appended, not inserted -- see the note on
    /// [`CommsProcessorToApplicationProcessorMessage::DebugCommand`].
    ///
    /// Sent in answer to
    /// [`CommsProcessorToApplicationProcessorMessage::RequestShotUploadConfig`] and
    /// unprompted whenever it changes, so a token rotated over the CLI takes effect
    /// without a reboot of either processor.
    ///
    /// No outer `Option`, unlike [`Self::WifiCredentials`]: `ShotUploadConfig`'s fields are
    /// individually optional, so the type already expresses "nothing configured" and a
    /// second layer of it would have two spellings for one state.
    ///
    /// Deliberately *not* carried inside `Configuration`, for the same reason as
    /// [`Self::WifiCredentials`] -- and more sharply, since this token grants write access
    /// to an account on a public service.
    ShotUploadConfig(crate::shot_upload::ShotUploadConfig),
    /// The outcome of a [`CommsProcessorToApplicationProcessorMessage::DeleteRoutine`].
    ///
    /// Appended, not inserted -- see the note on
    /// [`CommsProcessorToApplicationProcessorMessage::DebugCommand`].
    ///
    /// `index` is echoed for the reason [`Self::RoutineChunk`]'s is: this protocol has no
    /// correlation id, and it is the only thing tying a reply to the request that asked for
    /// it. It sits on the message rather than inside every arm of
    /// [`RoutineDeleteOutcome`], which is the same in all of them.
    RoutineDeleteResult {
        index: RoutineIndex,
        outcome: RoutineDeleteOutcome,
    },
    /// The stored Bluetooth bonds.
    ///
    /// Appended, never inserted -- postcard encodes an enum as its declaration-order
    /// discriminant and this link carries no version byte.
    ///
    /// The answer to [`CommsProcessorToApplicationProcessorMessage::RequestBluetoothBonds`],
    /// and also sent unsolicited when the set changes -- which in practice means when an
    /// association is removed and its bond goes with it, since the comms processor is the
    /// side that adds them.
    ///
    /// Sent as a whole list rather than one bond at a time because that makes the message
    /// idempotent: the receiver installs exactly what it is given and forgets what it had,
    /// so a bond removed here cannot survive on the far side.
    BluetoothBonds(crate::bluetooth::BluetoothBonds),
}

/// An operation on a scale, as carried by
/// [`ApplicationProcessorToCommsProcessorMessage::ScaleCommand`].
///
/// Taring and timer control, which every driver in this tree can perform. **Not**
/// calibration: neither protocol has a zero-calibration or reference-weight command, a
/// Bluetooth scale is calibrated by its own vendor app, and
/// [`variegated_hal::scale::ScaleCapabilities`] reports both as unsupported. That
/// distinction is what `variegated-controller-lib`'s `scale_calibration` module is built
/// on, and adding a variant here does not change it.
///
/// The timer runs on the scale and drives the scale's own display. Nothing in this
/// firmware reads it back -- ACAIA's timer notifications are discarded by its driver, and
/// BooKoo's arrive inside the weight frame as a millisecond counter the driver does not
/// publish. These are write-only controls.
///
/// The two protocols differ in one place, and the drivers absorb it: BooKoo has a single
/// atomic tare-and-start command, while ACAIA needs a tare followed by a timer start.
///
/// Append, never insert.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ScaleOp {
    /// Zero the scale.
    Tare,
    /// Start the scale's timer running.
    StartTimer,
    /// Stop it, leaving the elapsed time on the scale's display.
    StopTimer,
    /// Return it to zero.
    ResetTimer,
    /// Zero the scale and start its timer. One command on BooKoo, two writes on ACAIA.
    TareAndStartTimer,
    /// Tell the scale the dry dose, in grams, so it can show it and compute its own ratio.
    ///
    /// **Appended**, per the note above.
    ///
    /// Grams rather than the tenths BooKoo's frame carries, because this enum is the
    /// protocol-neutral vocabulary -- the scaling and the 0.1-999.0 g range are one protocol's
    /// and belong in its codec.
    ///
    /// **Not every scale can do this**, and unlike the timer that is not a difference the
    /// drivers can absorb: only BooKoo's Ultra defines the command, ACAIA has no equivalent,
    /// and a load cell has no display to put it on. A driver that cannot do it drops the op.
    /// Nothing in either protocol acknowledges a command, so a caller cannot learn which
    /// happened -- see the note on `variegated_hal::scale::ScaleCapabilities`, which already
    /// records that this type cannot express a per-driver answer.
    SetDose(f32),
}

/// An operation on a brew sensor, as carried by
/// [`ApplicationProcessorToCommsProcessorMessage::BrewSensorCommand`].
///
/// What the Belka Portal shows on its own display, and nothing else. It is a separate
/// vocabulary from [`ScaleOp`] because the two peripherals share nothing: a Portal has no
/// weight to zero and no timer to run, and a scale has no screen to switch.
///
/// **Write-only, and less answerable than the scale's.** The write itself is acknowledged --
/// unlike the scale commands, which are fire-and-forget -- but the acknowledgement covers
/// receipt, not comprehension. Nothing tells the machine which view the Portal is actually
/// showing, so this is a request rather than a setting, and the firmware keeps no model of the
/// answer.
///
/// Append, never insert.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BrewSensorOp {
    /// Switch the display to its graph view, for the duration of a shot.
    ShowGraph,
    /// Leave it.
    HideGraph,
}

#[cfg(all(test, feature = "serde", feature = "std"))]
mod tests {
    use super::*;

    /// The link's own round trip, for the one message an input device sends.
    ///
    /// The length is pinned rather than merely observed, for the reason
    /// [`crate::shot_upload`]'s round-trip test pins its own: this frame crosses a
    /// `CobsAccumulator<4096>` at both ends, and a change to its size is a wire-format
    /// change worth being told about.
    #[test]
    fn an_input_event_round_trips_the_link() {
        let msg = CommsProcessorToApplicationProcessorMessage::InputEvent(
            0xBA1D,
            InputCommand::Increment(3),
        );

        let mut buf = [0u8; 64];
        let encoded = postcard::to_slice(&msg, &mut buf).expect("serialize");
        assert_eq!(encoded.len(), 6);

        let decoded: CommsProcessorToApplicationProcessorMessage =
            postcard::from_bytes(encoded).expect("deserialize");
        match decoded {
            CommsProcessorToApplicationProcessorMessage::InputEvent(id, command) => {
                assert_eq!(id, 0xBA1D);
                assert_eq!(command, InputCommand::Increment(3));
            }
            _ => panic!("decoded to the wrong variant"),
        }
    }

    /// `InputEvent` is variant 20, and `DeleteRoutine` is still 19.
    ///
    /// postcard encodes an enum as its declaration-order discriminant and this link carries
    /// no version byte, so a variant inserted rather than appended silently mis-decodes
    /// every message after it on a peer built from another commit. Pinning the last two
    /// catches an insertion anywhere ahead of them, which is the whole of that failure
    /// mode; it does not catch two payload variants being swapped with each other.
    #[test]
    fn appending_input_event_did_not_renumber_the_link() {
        let mut buf = [0u8; 64];

        let delete =
            CommsProcessorToApplicationProcessorMessage::DeleteRoutine(RoutineIndex::Function(0));
        assert_eq!(
            postcard::to_slice(&delete, &mut buf).expect("serialize")[0],
            19
        );

        let input = CommsProcessorToApplicationProcessorMessage::InputEvent(
            0xBA1D,
            InputCommand::Activate,
        );
        assert_eq!(
            postcard::to_slice(&input, &mut buf).expect("serialize")[0],
            20
        );
    }

    /// The bond exchange went on the end of both enums.
    ///
    /// Three variants across two directions, so three chances to renumber a link that has
    /// no version byte to catch it.
    #[test]
    fn the_bond_messages_are_appended_to_both_directions() {
        use crate::bluetooth::{BluetoothBond, BluetoothBonds};

        let mut buf = [0u8; 256];

        let request = CommsProcessorToApplicationProcessorMessage::RequestBluetoothBonds;
        assert_eq!(
            postcard::to_slice(&request, &mut buf).expect("serialize")[0],
            21
        );

        let stored = CommsProcessorToApplicationProcessorMessage::BluetoothBondStored(
            BluetoothBond::default(),
        );
        assert_eq!(
            postcard::to_slice(&stored, &mut buf).expect("serialize")[0],
            22
        );

        let bonds =
            ApplicationProcessorToCommsProcessorMessage::BluetoothBonds(BluetoothBonds::default());
        assert_eq!(
            postcard::to_slice(&bonds, &mut buf).expect("serialize")[0],
            23
        );
    }

    /// A stored bond survives the link itself, not just postcard.
    ///
    /// The keys are why this is worth its own test: the comms processor sends this once,
    /// immediately after pairing, and nothing re-sends it. A bond mangled here is not
    /// noticed until the next reboot fails to reconnect.
    #[test]
    fn a_stored_bond_round_trips_the_link() {
        use crate::bluetooth::{BluetoothBond, BluetoothSecurityLevel};

        let bond = BluetoothBond {
            address: [0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff],
            address_random: true,
            long_term_key: u128::MAX,
            identity_resolving_key: Some(1),
            security_level: BluetoothSecurityLevel::Level2,
        };
        let msg = CommsProcessorToApplicationProcessorMessage::BluetoothBondStored(bond);

        let mut buf = [0u8; 256];
        let encoded = postcard::to_slice(&msg, &mut buf).expect("serialize");
        let decoded: CommsProcessorToApplicationProcessorMessage =
            postcard::from_bytes(encoded).expect("deserialize");
        match decoded {
            CommsProcessorToApplicationProcessorMessage::BluetoothBondStored(got) => {
                assert_eq!(got, bond)
            }
            _ => panic!("decoded to the wrong variant"),
        }
    }

    /// The command vocabulary's own discriminants.
    ///
    /// Same reasoning as the enum that carries it: this is append-only, and `Custom` in
    /// particular is reserved, so it must keep its position while it means nothing.
    #[test]
    fn the_input_command_discriminants_are_pinned() {
        let cases: [(InputCommand, u8); 6] = [
            (InputCommand::Increment(1), 0),
            (InputCommand::Decrement(1), 1),
            (InputCommand::Activate, 2),
            (InputCommand::Return, 3),
            (InputCommand::Menu, 4),
            (InputCommand::Custom(0), 5),
        ];

        let mut buf = [0u8; 16];
        for (command, discriminant) in cases {
            let encoded = postcard::to_slice(&command, &mut buf).expect("serialize");
            assert_eq!(encoded[0], discriminant, "{command:?} moved");
        }
    }
}
