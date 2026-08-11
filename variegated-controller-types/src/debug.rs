//! Structured debug payloads emitted by both firmwares.
//!
//! These live here rather than in a dedicated crate because `DebugCommand` wraps
//! `MachineCommand` while `ApplicationProcessorToCommsProcessorMessage` wraps
//! `DebugFrame`; a separate crate would make the dependency circular.
//!
//! Sizing matters: an enum is as large as its largest variant, and a bus of these
//! is static RAM on both MCUs. `CounterSamples` is the largest at 16 * 8 bytes, so
//! `DebugFrame` lands around 150 bytes. Keep it that way -- in particular, metric
//! names are sent one at a time via `MetricName` rather than as a table, which is
//! also what lets a late-attaching client learn them under always-on emission, and
//! `Status` is carried behind a `Box` rather than inline. `debug_frame_stays_small`
//! in `variegated-debug-codec` guards the bound.

use heapless::{String, Vec};

use crate::Status;

/// Wire-format version for the debug protocol.
///
/// Bump on ANY change to the shape of `DebugFrame`, `DebugPayload`, `DebugEvent`,
/// `DebugStateSnapshot`, `DebugCommand`, or anything they contain -- including
/// `Status`, which travels inside `DebugPayload::Status`. postcard is positional and
/// self-describes nothing, so a mismatched pair does not fail, it mis-decodes: the
/// host renders plausible garbage. Adding a field to a struct or a variant anywhere
/// but the end of an enum silently shifts everything after it.
///
/// Appending a variant to the *end* of an enum is the one change that is
/// backward-compatible in the decode direction, and it is still not exempt: an old
/// host handed the new variant's discriminant fails to deserialize, which surfaces
/// as a bare framing error rather than as "your host is out of date". Bump it too.
///
/// The version is carried in the codec's envelope, in front of the postcard bytes
/// and inside the COBS frame -- never as a field of `DebugFrame`. A version inside
/// the payload is useless for the failure it exists to catch: if the payload shape
/// changed, the host cannot decode the frame that would have told it why.
///
/// Numbering starts at `0x81` rather than `1`, and the high bit is the point.
///
/// The check has to reject not only a *differently* versioned peer but an
/// *unversioned* one -- anything built before the envelope existed. Such a frame
/// begins with the postcard encoding of `DebugFrame::source`, i.e. the `DebugSource`
/// discriminant, and a command with `DebugCommand`'s. At `1` the check would have
/// accepted every unversioned `DebugSource::Comms` frame and handed the host a
/// `DebugFrame` deserialized one byte out of phase -- a fabricated event on the
/// wrong processor, which is precisely the mis-decode this constant exists to stop.
///
/// postcard varint-encodes enum discriminants, so a leading byte with the high bit
/// set means a discriminant of at least 128. No enum that can start one of these
/// messages has anything like that many variants, so no unversioned message can
/// begin with `0x81` -- or with `0x82`, `0x83` and the rest of the bump sequence,
/// which inherit the property. Keep bumping inside `0x81..=0xFF`.
///
/// Unrelated to `crate::PROTOCOL_VERSION`, which versions the machine-definition
/// protocol between the two processors.
/// History:
/// * `0x81` -- the envelope's first version.
/// * `0x82` -- `CommsState` gained `wifi_mac`, `bt_address` and `wifi_ip`.
/// * `0x83` -- Bluetooth peripheral associations. `Status` gained `bluetooth`,
///   `Configuration` gained `bluetooth_peripherals`, and `MachineCommand` gained four
///   variants. All three reach this wire: the first two through `DebugPayload::Status`
///   and the configuration relay, the third through `DebugCommand::Machine`.
/// * `0x84` -- `Status` gained `comms_status_age`, immediately after `comms_status`.
/// * `0x85` -- [`MAX_SAMPLES`] 16 -> 24. Not a change of *shape*: a `heapless::Vec`
///   encodes as a length varint plus elements, and the capacity is a type parameter that
///   never reaches the wire, so a 14-counter frame is byte-identical before and after.
///   It is a change of *range*, and that is what needs the bump -- a host built at 16
///   deserializing a 17-element frame fails on capacity, and the version byte turns that
///   into "your host is too old" instead of a decode error pointing at nothing.
/// * `0x86` -- `AppDebugOp` gained `SdCardSelfTest`, appended. Device-inbound, so the
///   hazard runs the other way from the ones above: an *older* device decoding a
///   newer host's frame cannot see the new discriminant, and a newer device decoding
///   an older host's frame is unaffected. The bump exists so the mismatch is reported
///   as a version mismatch rather than as a command the operator did not type -- which
///   is the failure mode `debug_command.rs` warns about, on a machine that heats water.
/// * `0x87` -- shot annotations. `Status` gained `pending_shot_annotations` and
///   `sd_card_present`, and `MachineCommand` gained three variants. Both reach this wire:
///   the first through `DebugPayload::Status`, the second through `DebugCommand::Machine`.
///   One bump covers both, rather than one per field -- they land in the same change, and
///   a host that can decode either can decode both.
/// * `0x88` -- `AppDebugOp` gained `SdListShots`, appended, and
///   `ApplicationProcessorToCommsProcessorMessage` gained `ShotLogError`. Device-inbound
///   and device-outbound respectively, so the two hazards run in opposite directions --
///   which is precisely why one version byte covers both: a host and a device that agree
///   on it agree on the whole envelope, in both directions, rather than on one half of it.
/// * `0x89` -- shot-log format v2. `ShotAnnotationKey` and `ShotAnnotationValue` each
///   lost their trailing `Routine` variant, which is now carried only by
///   `ShotLogMetadata::routine_metadata`; `ShotLogStorageError` gained
///   `UnsupportedVersion`. Both reach this wire through `Status::pending_shot_annotations`.
///   Removing a *trailing* variant renumbers nothing, so an old host decoding a new
///   device still reads every annotation it is sent -- but a host that sends a `Routine`
///   annotation would now decode as garbage, and that is what the bump exists to catch.
/// * `0x8A` -- `AppDebugOp` gained `SdFormatCard`, appended. Device-inbound, and the one
///   command in this enum that destroys data, so a version mismatch here is worth
///   reporting as such rather than letting a host's older frame be read as something
///   adjacent. The command carries its own magic-value guard for the same reason.
/// * `0x8B` -- `CommsStatus` gained `improv`, appended, and `MachineCommand` gained four
///   Wi-Fi provisioning variants. The `MachineCommand` half is the usual appended-variant
///   case an older peer merely fails to recognise. The `CommsStatus` half is not: it is a
///   *struct*, and although the new field is last -- so nothing inside `CommsStatus` moves
///   -- `CommsStatus` is itself a field of `Status`, and postcard has no length prefix to
///   resynchronise on. An older host decoding a new `Status` reads `comms_status` without
///   consuming `improv`, then reads that byte as the start of `comms_status_age`, and
///   every field after it is garbage. That is exactly the failure this version exists to
///   catch, and it is the same shape as the drift that motivated the schema fixtures.
/// * `0x8C` -- `AppDebugOp` gained `ClearWifiCredentials`, appended. Device-inbound, so the
///   same direction as `SdFormatCard` at `0x8A` and bumped for the same reason: it is
///   destructive, and a version mismatch should report itself as one rather than as a command
///   the operator did not type. It carries its own magic-value guard as well, because the
///   version byte protects against a *differently built* peer and the guard against a
///   *corrupted frame* from a correctly built one -- two different failures.
pub const DEBUG_PROTOCOL_VERSION: u8 = 0x8C;

/// Maximum number of counters or indicators carried in one sample frame.
///
/// Raised 16 -> 24 to fit the BLE connect-path metrics next to the network ones that were
/// already using 14 of the 16. Both firmwares share this bound, so the application
/// processor gets the same headroom whether or not it wants it.
///
/// Bounded by [`crate::debug`]'s frame budget rather than by anything here: 24 u64s
/// varint-encode to at most 240 bytes, against the codec's `MAX_FRAME` of 2048, so there
/// is room to raise this again if a subsystem needs it.
pub const MAX_SAMPLES: usize = 24;
/// Capacity of the ad-hoc text escape hatch.
pub const TEXT_LEN: usize = 96;
/// Capacity of a metric or firmware name.
pub const NAME_LEN: usize = 32;

pub type DebugText = String<TEXT_LEN>;
pub type Name = String<NAME_LEN>;

/// Copy `s` into a fixed-capacity string, dropping any tail that does not fit.
/// Pushing char-by-char keeps the result on a UTF-8 boundary.
pub fn fit<const N: usize>(s: &str) -> String<N> {
    let mut out = String::new();
    for c in s.chars() {
        if out.push(c).is_err() {
            break;
        }
    }
    out
}

pub fn name(s: &str) -> Name {
    fit(s)
}

pub fn text(s: &str) -> DebugText {
    fit(s)
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum DebugSource {
    Application,
    Comms,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Severity {
    Trace,
    Debug,
    Info,
    Warn,
    Error,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum MetricKind {
    Counter,
    Indicator,
}

/// One framed unit of debug output. `seq` is per-source and monotonic, so a host
/// can tell dropped frames from quiet periods; `uptime_ms` is that device's own
/// uptime -- the two devices boot independently, so it is not a shared clock.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, PartialEq)]
pub struct DebugFrame {
    pub source: DebugSource,
    pub seq: u32,
    pub uptime_ms: u64,
    pub payload: DebugPayload,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, PartialEq)]
pub enum DebugPayload {
    /// Raw counter values, indexed by counter id.
    CounterSamples(Vec<u64, MAX_SAMPLES>),
    /// Raw indicator values, indexed by indicator id.
    IndicatorSamples(Vec<u64, MAX_SAMPLES>),
    Event(DebugEvent),
    Text(Severity, DebugText),
    StateSnapshot(DebugStateSnapshot),
    /// Names one metric. Re-sent periodically so late clients can label things.
    MetricName {
        kind: MetricKind,
        id: u8,
        label: Name,
    },
    FirmwareInfo {
        firmware: Name,
        counters: u8,
        indicators: u8,
    },
    /// The machine's full published status, boxed.
    ///
    /// Boxed because an enum is as large as its largest variant: inline, this one
    /// variant would grow every frame on the bus to ~1-2 kB and blow the static RAM
    /// budget on both MCUs.
    ///
    /// **This variant does not travel on the shared debug bus.** It has its own
    /// single-slot channel, `variegated_debug::status`, and the reason is a hard one:
    /// `embassy-sync`'s pubsub hands a message to a subscriber by `clone()`ing it --
    /// inside the bus's `CriticalSectionRawMutex` -- in every case except the last
    /// subscriber taking the message at index 0. With one subscriber that never fires;
    /// with two (the local transport plus the inter-processor relay) it fires on every
    /// `Status`, which would be a ~1.7 kB first-fit `LlffHeap::alloc` plus a memcpy
    /// with interrupts disabled on both cores, once a second, in steady state.
    /// Filtering the variant out in the relay does not help -- the clone happens
    /// before the relay's code runs.
    ///
    /// The *allocation* happens in the 1 Hz snapshot task before publication, never on
    /// a control path and never inside the publish itself. Publishing still never
    /// awaits and never back-pressures a producer, so the non-blocking contract holds
    /// -- but it is not allocator-free. Superseding an undelivered `Status` frees a
    /// `Box` through `LlffHeap::dealloc`, whose free-list insert is O(n) and takes a
    /// critical section of its own; `status::publish_with` does that outside the
    /// signal's own critical section, and it only happens when the transport is
    /// stalled or absent. Bear all of this in mind before adding boxed variants, and
    /// in particular before putting one back on the shared bus.
    ///
    /// `postcard` serializes `Box<T>` transparently, so the wire encoding is just
    /// `Status`'s own.
    Status(alloc::boxed::Box<Status>),
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, PartialEq)]
pub enum DebugEvent {
    Boot,
    // Application processor
    TimeSynchronized { unix: u64 },
    TimeSyncIgnoredImplausible { unix: u64 },
    TimeSyncFailed,
    CommandReceived { label: Name },
    ConfigurationSent,
    ConfigurationRequested,
    MachineDefinitionSent,
    RoutinesSent,
    LinkDecodeError,
    CountersReset,
    // Application processor -- machine events.
    //
    // These are the sites where structure earns its keep: a host can count brews,
    // correlate a routine against a shot log, or filter for interlocks, none of
    // which it can do against a free-text line. Every one of them is edge
    // triggered -- entered from a command handler or a state transition, never
    // from a control-loop body -- so promoting them cannot flood the bus.
    //
    // The `log_*!` call that used to sit at each of these sites was removed, not
    // kept alongside: with the `log` -> bus bridge installed, leaving it would put
    // the same occurrence on the bus twice, once typed and once as text.
    BrewStarted { group: u8 },
    BrewStopped { group: u8 },
    SteamStarted,
    SteamStopped,
    /// `index` is `RoutineIndex::to_storage_index()`, which round-trips the
    /// Internal/Function/Custom discriminant as well as the number.
    RoutineStarted { index: u16 },
    RoutineCompleted,
    RoutineCancelled,
    /// A record was committed to flash. `index` is the slot; `0` where the store
    /// holds a single record (settings).
    StorageWrite { store: Name, index: u16 },
    SensorFault { sensor: Name },
    /// A safety interlock refused an operation.
    ///
    /// `Warn`, not `Error`, and deliberately: an interlock trip is the machine
    /// correctly declining to do something unsafe, which is defensive behaviour
    /// working as designed, not a malfunction. Some of the sites this replaced
    /// used `error!`; they are now consistent with the boiler interlocks, which
    /// always used `warn!`. `Error` on this event would mean a host could not
    /// distinguish "refused to brew with an empty tank" from a genuine fault.
    InterlockTripped { interlock: Name },
    // Comms processor
    WifiAssociated,
    WifiLost,
    WifiReconnectRequested,
    SntpSynced { unix: u64 },
    SntpFailed,
    BlePeripheralConnected { id: u16 },
    BlePeripheralDisconnected { id: u16 },
    BleScanStarted,
    EsphomeClientConnected,
    EsphomeClientDisconnected,
    TcpDebugClientConnected,
    TcpDebugClientDisconnected,
    // Either
    SpawnFailed { task: Name },
    HeapReport { used: u32, free: u32 },
    CommandRejected { reason: Name },
}

impl DebugEvent {
    pub fn severity(&self) -> Severity {
        match self {
            DebugEvent::TimeSyncFailed
            | DebugEvent::SntpFailed
            | DebugEvent::LinkDecodeError
            | DebugEvent::SpawnFailed { .. }
            | DebugEvent::CommandRejected { .. }
            | DebugEvent::SensorFault { .. } => Severity::Error,
            DebugEvent::WifiLost
            | DebugEvent::TimeSyncIgnoredImplausible { .. }
            | DebugEvent::BlePeripheralDisconnected { .. }
            | DebugEvent::InterlockTripped { .. } => Severity::Warn,
            _ => Severity::Info,
        }
    }

    /// Stable, allocation-free label used for host-side filtering and display.
    pub fn label(&self) -> &'static str {
        match self {
            DebugEvent::Boot => "boot",
            DebugEvent::TimeSynchronized { .. } => "time_synchronized",
            DebugEvent::TimeSyncIgnoredImplausible { .. } => "time_sync_ignored",
            DebugEvent::TimeSyncFailed => "time_sync_failed",
            DebugEvent::CommandReceived { .. } => "command_received",
            DebugEvent::ConfigurationSent => "configuration_sent",
            DebugEvent::ConfigurationRequested => "configuration_requested",
            DebugEvent::MachineDefinitionSent => "machine_definition_sent",
            DebugEvent::RoutinesSent => "routines_sent",
            DebugEvent::LinkDecodeError => "link_decode_error",
            DebugEvent::CountersReset => "counters_reset",
            DebugEvent::BrewStarted { .. } => "brew_started",
            DebugEvent::BrewStopped { .. } => "brew_stopped",
            DebugEvent::SteamStarted => "steam_started",
            DebugEvent::SteamStopped => "steam_stopped",
            DebugEvent::RoutineStarted { .. } => "routine_started",
            DebugEvent::RoutineCompleted => "routine_completed",
            DebugEvent::RoutineCancelled => "routine_cancelled",
            DebugEvent::StorageWrite { .. } => "storage_write",
            DebugEvent::SensorFault { .. } => "sensor_fault",
            DebugEvent::InterlockTripped { .. } => "interlock_tripped",
            DebugEvent::WifiAssociated => "wifi_associated",
            DebugEvent::WifiLost => "wifi_lost",
            DebugEvent::WifiReconnectRequested => "wifi_reconnect_requested",
            DebugEvent::SntpSynced { .. } => "sntp_synced",
            DebugEvent::SntpFailed => "sntp_failed",
            DebugEvent::BlePeripheralConnected { .. } => "ble_connected",
            DebugEvent::BlePeripheralDisconnected { .. } => "ble_disconnected",
            DebugEvent::BleScanStarted => "ble_scan_started",
            DebugEvent::EsphomeClientConnected => "esphome_connected",
            DebugEvent::EsphomeClientDisconnected => "esphome_disconnected",
            DebugEvent::TcpDebugClientConnected => "tcp_debug_connected",
            DebugEvent::TcpDebugClientDisconnected => "tcp_debug_disconnected",
            DebugEvent::SpawnFailed { .. } => "spawn_failed",
            DebugEvent::HeapReport { .. } => "heap_report",
            DebugEvent::CommandRejected { .. } => "command_rejected",
        }
    }
}

/// The diagnostics that have no other viewer: heap, frame accounting and per-source
/// link state. `Status` itself travels separately as `DebugPayload::Status` -- the
/// two are emitted from the same 1 Hz task so a host can correlate them, but they
/// are kept as distinct payloads so a `Status` that fails to encode cannot take the
/// diagnostics down with it.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, PartialEq)]
pub struct DebugStateSnapshot {
    pub heap_used: u32,
    pub heap_free: u32,
    pub frames_emitted: u32,
    /// Frames lost against the device's intent: no host attached, buffer full, or
    /// evicted from the ring unread. A rising value means something is wrong.
    pub frames_dropped: u32,
    /// Duplicate frames the device collapsed before publishing. Separate from
    /// `frames_dropped` because nothing is lost and nothing is wrong -- an
    /// identical frame went out moments earlier, and a repeating condition is
    /// re-announced on a heartbeat. Folding the two together made a healthy link
    /// read as a failing one at 10 Hz.
    pub frames_suppressed: u32,
    /// Frames refused by the text rate cap. Counted apart from
    /// `frames_suppressed` because these are genuinely **lost**: they were not
    /// duplicates, so nothing else carries what they would have said. Expected to
    /// stay at zero -- the rate limiter has burst capacity for the boot log and
    /// only sustained message diversity drains it.
    pub frames_rate_limited: u32,
    pub source_state: SourceState,
    /// Deepest the main task's stack has been since boot, in bytes, or `None` on a source
    /// that does not measure it.
    ///
    /// The counterpart to `heap_used`, and it belongs beside it for the reason that pair
    /// exists at all: on the comms processor the heap and the stack come out of one pool --
    /// `.stack` is whatever RWDATA is left after `.bss`, and the heap is a `.bss` static --
    /// so a host watching only one of them is watching half the problem. Both edges have
    /// taken that firmware down, within 10 kB of each other.
    ///
    /// A high-water mark rather than an instantaneous depth, unlike `heap_used`: a stack is
    /// almost always shallow at the moment it is sampled, so a 1 Hz reading of the current
    /// depth would say nothing about whether the deep path fits. See
    /// `variegated-comms-firmware/src/stack.rs` for how it is measured, and for the two
    /// readings that mean "unknown" rather than "fine".
    ///
    /// **Appended, not inserted.** postcard is positional and this type crosses the debug
    /// link between two separately-built binaries.
    pub stack_high_water: Option<u32>,
    /// Total bytes the main task's stack can occupy, against which `stack_high_water` is
    /// read.
    ///
    /// Sent rather than assumed, because it is not a constant anyone chose: the linker
    /// hands the task whatever RWDATA is left after `.data` and `.bss`, so it moves
    /// whenever an unrelated static changes size. A host that hard-coded it would go on
    /// reporting a margin that had silently gone.
    pub stack_size: Option<u32>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, PartialEq)]
pub enum SourceState {
    Application(ApplicationState),
    Comms(CommsState),
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, PartialEq)]
pub struct ApplicationState {
    /// `None` while the watchdog's feed time is not plumbed through to this
    /// snapshot. Deliberately an `Option` rather than a `0` sentinel: a zero here
    /// reads as "fed just now", which is a plausible-looking lie, and watchdog feed
    /// age is one of the things the hardware checkpoint exists to observe.
    pub watchdog_fed_ms_ago: Option<u32>,
    pub psram_heap: bool,
    /// `None` means "no routine running, or not determined" -- see the comment at the
    /// construction site.
    pub routine_running: Option<u16>,
    /// Debug frames the inter-processor relay handed to the link's TX queue.
    pub link_frames_relayed: u32,
    /// Debug frames the relay threw away: refused by its byte budget, unencodable,
    /// offered to a full TX queue, or recycled out of the bus ring before it read
    /// them. Filtered `DebugPayload::Status` frames are **not** counted -- those are
    /// a policy decision rather than a loss, and the comms processor receives
    /// `Status` by its own route. Both come from
    /// `variegated_comms::debug_relay::relay_stats`.
    ///
    /// **These are also included in [`DebugStateSnapshot::frames_dropped`].** That
    /// counter is per *transport attempt*, not per frame: one frame refused by both
    /// the USB writer (no host attached) and the relay (budget) adds two. So
    /// `frames_dropped - link_frames_dropped` really is the non-relay share, but
    /// only because every relay drop is counted in both -- do not assume the two are
    /// disjoint sets of frames.
    pub link_frames_dropped: u32,
    /// Deepest core 1's stack has reached, in bytes, or `None` on a board that does not
    /// run core 1.
    ///
    /// Core 0's figures are on [`DebugStateSnapshot`] itself, because every source has a
    /// primary stack; this pair is here because only the application processor has a second
    /// core. `None` means the core is not in use, which is the single-boiler board -- not
    /// that the measurement failed.
    ///
    /// Worth carrying separately rather than reporting the worse of the two: the two stacks
    /// are sized independently and fail independently, and a single "worst" number would
    /// say which value but not which core, which is the first thing anyone needs.
    ///
    /// **Appended, not inserted.** postcard is positional.
    pub core1_stack_high_water: Option<u32>,
    /// Total bytes core 1's stack can occupy, against which
    /// [`Self::core1_stack_high_water`] is read.
    ///
    /// Unlike core 0's, this one *is* a number someone chose -- `CORE1_STACK_LENGTH` in the
    /// board's `main.rs` -- but it is sent rather than assumed for the same reason: a host
    /// that hard-coded it would report a margin against a size the firmware no longer uses.
    pub core1_stack_size: Option<u32>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, PartialEq)]
pub struct CommsState {
    pub wifi_connected: bool,
    pub wifi_rssi: Option<i8>,
    pub sntp_synced_ms_ago: Option<u32>,
    pub ble_connected: Vec<u16, 8>,
    pub tcp_debug_clients: u8,
    /// The station MAC, from eFuse. Fixed for the life of the board, so it is read
    /// once at boot and mirrored rather than re-read per snapshot.
    ///
    /// Not an `Option`: eFuse is readable from the first instruction of `main`, and
    /// the mirror is written before the snapshot task is spawned, so there is no
    /// window in which this is unknown.
    pub wifi_mac: [u8; 6],
    /// The randomly-generated BLE address, which is worth knowing when correlating a
    /// scan capture against a session because it is regenerated every boot.
    ///
    /// `None` before the BLE stack is brought up.
    ///
    /// On the comms processor that window is currently empty, and the reason is worth
    /// stating precisely because it is easy to get backwards: the snapshot task is
    /// spawned before the address is drawn, but `main` is itself a task and does not
    /// yield until its first `.await`, which comes *after* the store. Nothing spawned
    /// in between runs until then, so no snapshot can observe the gap.
    ///
    /// It is an `Option` anyway, and not as a hedge. That guarantee is a property of
    /// statement ordering inside one long function, held up by no test and no type --
    /// inserting any `.await` between the spawn and the store silently opens the
    /// window. A bare `[u8; 6]` would then report `00:00:00:00:00:00`, a syntactically
    /// valid address that no scanner will ever see, and nothing would fail to warn
    /// about it. One byte buys immunity to that edit.
    pub bt_address: Option<[u8; 6]>,
    /// `None` until DHCP completes, and `None` again if the lease is lost.
    ///
    /// Deliberately not a `[0, 0, 0, 0]` sentinel: `0.0.0.0` is a renderable, specific
    /// and false claim about the network, and the host shows `unknown` instead. Same
    /// discipline as `wifi_rssi` and `ApplicationState::watchdog_fed_ms_ago`.
    pub wifi_ip: Option<[u8; 4]>,
}
