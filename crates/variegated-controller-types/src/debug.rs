//! Structured debug payloads emitted by both firmwares.
//!
//! These live here rather than in a dedicated crate because `DebugCommand` wraps
//! `MachineCommand` while `ApplicationProcessorToCommsProcessorMessage` wraps
//! `DebugFrame`; a separate crate would make the dependency circular.
//!
//! Sizing matters: an enum is as large as its largest variant, and a bus of these
//! is static RAM on both MCUs -- the bus is a 16-slot channel, so every byte here
//! is sixteen bytes of RAM per device. `CounterSamples` is the largest at
//! `MAX_SAMPLES * 8` plus a length, i.e. 200 bytes, which puts `DebugFrame` at
//! around 224 against the `< 256` bound. `CheckinReport` is sized to match it
//! exactly rather than to exceed it. Keep it that way -- in particular, metric and
//! check-in names are sent one at a time via `MetricName`/`CheckinSlotInfo` rather
//! than as a table, which is also what lets a late-attaching client learn them under
//! always-on emission, and `Status` is carried behind a `Box` rather than inline.
//! `debug_frame_stays_small` in `variegated-debug-codec` guards the bound.

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
/// * `0x8D` -- `MachineCommand` gained `DeleteShotLog`, appended. Device-inbound and
///   destructive, like `SdFormatCard` and `ClearWifiCredentials` before it, so a version
///   mismatch must report itself as one rather than as a command nobody typed. The same
///   bump covers `RequestShotLogList` changing shape and `ShotLogEvent` being appended to
///   the inter-processor reply enum; neither reaches the debug wire directly, but the
///   shot-log types they carry reach it through `Status::pending_shot_annotations`.
/// * `0x8E` -- `MachineCommand` gained `SetShotUploadConfig`, appended. Device-inbound, so
///   the same direction and the same reasoning as the three above, with one addition: this
///   one carries a *secret*. A host built against an older version encodes some other
///   variant's payload at this discriminant, and the failure mode of getting that wrong is
///   a bearer token written into whichever field the old layout happens to land on. The
///   same bump covers `RequestShotUploadConfig` and `ShotUploadConfig` being appended to
///   the two inter-processor enums.
/// * `0x8F` -- `DebugEvent` gained `ShotUploaded` and `ShotUploadFailed`, appended at the
///   end of the enum rather than filed with the comms events they belong with topically.
///   Device-*outbound*, unlike the four above, so the failure it guards against is the
///   mirror image: an older host reads a discriminant it does not know, or -- had these
///   been inserted rather than appended -- reads `SpawnFailed` as one of these and renders
///   confident nonsense about a task that started fine.
/// * `0x90` -- shot-upload settings became editable from the browser. `MachineCommand`
///   gained `SetShotUploadSettings`, appended; `ShotUploadConfig` gained a trailing
///   `enabled` field; and `Configuration` gained `shot_upload`. The middle one is the reason
///   this bump is not optional in either direction: postcard is positional, so a host built
///   against the older shape reads the new `enabled` byte as the start of whatever it
///   thought came next. `Configuration` reaches the debug wire inside `Status`.
/// * `0x91` -- task check-ins. `DebugPayload` gained `CheckinReport` and `CheckinSlotInfo`,
///   both appended, and `FirmwareInfo` gained a trailing `checkins` field. The appended
///   variants are the usual device-outbound case an older host merely fails to decode. The
///   `FirmwareInfo` field is not: it is a *struct*, and although the new field is last,
///   postcard has no length prefix to resynchronise on, so an older host reads the count byte
///   as the start of the next frame's content. That is the same shape as the `CommsStatus`
///   break at `0x8B`, and it is what makes this bump mandatory rather than courteous.
/// * `0x92` -- SD stall diagnosis. `AppDebugOp` gained `SdBusState` and
///   `ShotLogStorageError` gained `OperationTimedOut`, both appended. Device-inbound and
///   device-outbound respectively, so this bump covers both directions at once.
///
///   The second one is the reason it is not optional. `ShotLogStorageError` travels on the
///   *inter-processor* link inside `ApplicationProcessorToCommsProcessorMessage::ShotLogError`
///   as well as reaching a host, so a comms processor flashed from an older build decodes an
///   unknown discriminant on a reply it is waiting for -- and the answer it fails to decode
///   is precisely the one that says the card stalled. Flash both ends together.
/// * `0x93` -- `http+noise://` shot upload. `ShotUploadConfig` and `ShotUploadSettings` each
///   gained a trailing `server_key` and `device_key`, and `ShotUploadKeyUpdate` is a new type
///   inside the latter.
///
///   Mandatory in both directions, and for the same reason `0x90` was: these are *structs*,
///   postcard has no length prefix to resynchronise on, and `ShotUploadConfig` crosses the
///   inter-processor link inside `ApplicationProcessorToCommsProcessorMessage` while
///   `ShotUploadSettings` arrives from a host inside a `MachineCommand`. An older peer reads
///   the first appended byte as the start of whatever it thought came next.
///
///   Note this bump does **not** protect the stored settings blob, which carries no version
///   at all: appending to `ShotUploadConfig` makes every previously stored one fail to decode,
///   `load_settings` maps that to `Default`, and every machine loses its endpoint and token
///   once. That is the second time -- see the field comment on `enabled` -- and it was again
///   accepted rather than paying for a fifth settings store.
pub const DEBUG_PROTOCOL_VERSION: u8 = 0x93;

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

/// Maximum check-in slots carried in one [`DebugPayload::CheckinReport`].
///
/// 24, and the number is arithmetic rather than taste. [`CheckinEntry`] is 8 bytes, so
/// `Vec<CheckinEntry, 24>` is 200 -- byte-for-byte the size of `Vec<u64, MAX_SAMPLES>`,
/// which is what `DebugFrame` is already sized by. This payload therefore costs the
/// frame nothing, and `debug_frame_stays_small` in `variegated-debug-codec` reports the
/// same number before and after it was added.
///
/// Raising it is not like raising [`MAX_SAMPLES`], which was free for the same reason
/// this is: there, 24 u64s still fit under the existing largest variant. Here 24 *is*
/// the largest variant, so 25 grows `DebugFrame` and every one of the bus's 16 slots
/// with it. 27 is the last value that fits under 256 at all. Widening [`CheckinEntry`]
/// has the same effect three crates away from where it would be edited, which is what
/// the `size_of` assertion beside it is for.
///
/// A firmware with more monitorable tasks than this does not get a bigger frame: it
/// picks the 24 worth watching. A slot that is never overdue is a slot that taught you
/// nothing.
pub const MAX_CHECKINS: usize = 24;

/// Why a check-in slot is not reporting [`CheckinStatus::Good`].
///
/// Deliberately coarse, for the reason `ShotLogStorageError` gives: a host cannot do
/// anything differently for "the ADS returned CRC error 0x3" than for "the ADS timed
/// out", and the underlying error is logged with its full detail at the point it
/// occurs. What the wire carries is the *kind* of trouble; the `log_warn!` at the site
/// carries which.
///
/// What deliberately does not appear here:
///
/// * *Which* device failed. That is the slot's name, already on the wire via
///   [`DebugPayload::CheckinSlotInfo`].
/// * The driver's own error value -- logged at the site.
/// * A faulted sensor as an *event*: [`DebugEvent::SensorFault`].
/// * An interlock refusal: [`DebugEvent::InterlockTripped`]. An interlock is the machine
///   correctly declining to do something unsafe, and this enum is for faults.
/// * A task that never started: [`DebugEvent::SpawnFailed`], already emitted at the
///   spawn site.
/// * How long an overrun took -- that is an indicator, and both firmwares already carry
///   `*TimeMs` indicators of exactly that shape.
///
/// There is no `Other`, and there is no numeric sub-code. An `Other` is where a closed
/// enum goes to die: the point of the type is that a host can act on it, and nobody can
/// act on `Other`. A sub-code is worse -- a private namespace on a shared wire,
/// renderable only by someone holding the firmware source, and once it exists every new
/// distinction goes there instead of into this enum. `DebugEvent::ShotUploadFailed` is
/// the counter-example in this file: its doc has to enumerate the valid reasons in prose
/// because its type does not. If a distinction must be machine-readable, it earns a
/// variant, and the version bump that costs is the feature -- it is what tells an old
/// host it is old. For a per-task namespace, declare more slots; they are 8 bytes each
/// and need no protocol change.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum CheckinDetail {
    /// A device on a local bus did not answer. Look at the board.
    ///
    /// Split from [`Self::PeerUnresponsive`] because the two send you to different
    /// places: "the ADC came loose" and "the comms processor rebooted" must not render
    /// identically on a host.
    PeripheralUnresponsive,
    /// The far end of a link did not answer. Look at the other end.
    PeerUnresponsive,
    /// A mutex, bus lease or shared device could not be taken in time.
    ///
    /// The distinction `ShotLogStorageError::BusUnavailable` also draws: the thing may
    /// be perfectly healthy and merely unreachable from here, and conflating that with
    /// absence sends you looking at the wrong component.
    ResourceUnavailable,
    /// A send was refused because a queue was full. Work is being dropped **now**.
    ///
    /// The one detail that reports active data loss rather than slowness, which is why
    /// it is not folded into [`Self::Degraded`].
    QueueFull,
    /// The cycle ran but could do nothing: no clock, no credentials, no card.
    ///
    /// From outside, a loop in this state is indistinguishable from a healthy one -- it
    /// wakes on schedule forever and produces nothing.
    PreconditionUnmet,
    /// Running, but on a retry or fallback path.
    ///
    /// The state between clean and dead: one failure into a retry budget, backed off to
    /// a long reconnect interval, or running on a substituted default because the stored
    /// settings would not load. Worth seeing *before* a shot goes wrong, which is why it
    /// is not merged into the unresponsive variants.
    Degraded,
    /// The cycle took longer than its declared period.
    Overrun,
    /// The future returned when it was supposed to run forever.
    ///
    /// Produced by `variegated_checkin::watch` and by nothing else.
    TaskExited,
}

/// The state of one check-in slot.
///
/// [`Self::NotStarted`] is a variant rather than a sentinel age because a slot is in that
/// state from boot until its task first runs -- and forever on a task that never runs,
/// which is precisely the condition worth reporting. Encoding it as an out-of-range
/// `age_ms` would be the `watchdog_fed_ms_ago` trap in reverse: a plausible-looking value
/// where an honest absent one belongs. Encoding it by making the age an `Option<u32>`
/// would widen [`CheckinEntry`] to 12 bytes and, at [`MAX_CHECKINS`], grow `DebugFrame`
/// past its bound. As a variant it costs nothing.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum CheckinStatus {
    /// Declared, but never reached. Produced by the monitor's initial state, never by a
    /// task; the accompanying `age_ms` is meaningless alongside it.
    NotStarted,
    Good,
    Warning(CheckinDetail),
    Error(CheckinDetail),
}

/// One slot's line in a [`DebugPayload::CheckinReport`], positional by slot id.
///
/// The id is the index, not a field: the report is the whole table in declaration order,
/// so carrying an id per entry would be a byte a slot spent restating the position it is
/// already in.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct CheckinEntry {
    pub status: CheckinStatus,
    /// Milliseconds since this slot last checked in, as of the carrying frame's
    /// `uptime_ms`.
    ///
    /// An age rather than an absolute uptime, and the difference is wire bytes: a healthy
    /// 100 ms slot varint-encodes an age in two, where an absolute uptime after a day
    /// takes five. Times [`MAX_CHECKINS`], every second, on a link the relay is only
    /// allowed 5% of. The host reconstructs the absolute value from the frame's own
    /// `uptime_ms`, which it has anyway.
    ///
    /// Meaningless when `status` is [`CheckinStatus::NotStarted`].
    pub age_ms: u32,
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
        /// How many check-in slots this firmware declares.
        ///
        /// Lets a host size its table before any `CheckinSlotInfo` arrives, the same
        /// service the two counts above perform. Zero from a firmware that declares
        /// none, which is honest: there is no slot whose name is merely late.
        checkins: u8,
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
    /// Every check-in slot's current state, positional by slot id.
    ///
    /// **Level triggered**, and that is the point: a host attaching mid-session sees the
    /// whole table on the next tick rather than waiting for something to change. The
    /// edge-triggered alternative would also be indistinguishable from a dropped frame on
    /// a bus that drops frames under load, which is the failure this payload exists to
    /// report on.
    CheckinReport(Vec<CheckinEntry, MAX_CHECKINS>),
    /// Names one check-in slot and declares how often it expects to check in.
    ///
    /// Re-sent periodically for the reason `MetricName` is, and separate from it for one
    /// `MetricName` cannot serve: `period_ms` is meaningless for a counter or an
    /// indicator. Folding this in as a third `MetricKind` would put an `Option` on the
    /// wire that conflates "this slot has no period" with "this kind has no periods",
    /// which is the distinction `CommsState::wifi_ip` is documented for.
    CheckinSlotInfo {
        id: u8,
        label: Name,
        /// How often this slot expects to check in, or `None` if it is event-driven.
        ///
        /// The device never reads this. It ships the number and the host decides what
        /// counts as overdue, exactly as `COMMS_STATUS_STALE_AFTER` splits the threshold
        /// from the measurement. A single global threshold could not work here: the
        /// periods in one firmware span three orders of magnitude, so one number would
        /// leave something permanently red.
        ///
        /// `None` means the slot only checks in when work arrives -- a task parked on
        /// `receive().await` is healthy, and the host must not age it. This is what
        /// replaces an `Idle` variant of `CheckinStatus`, and it is what keeps
        /// [`CheckinDetail`] describing only problems.
        period_ms: Option<u32>,
    },
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
    /// A shot reached the upload endpoint. Carries the id rather than a count, so a host
    /// can tell *which* shots made it without correlating against a listing.
    ///
    /// **Appended at the end, not filed with the comms events above where it belongs
    /// topically.** postcard encodes an enum as its declaration-order discriminant, so
    /// inserting here would renumber `SpawnFailed`, `HeapReport` and `CommandRejected` --
    /// and this enum travels to hosts built separately from the firmware.
    ShotUploaded { day: u32, time: u32 },
    /// A shot did not. `reason` is one of a small fixed set -- `endpoint`, `clock`,
    /// `network`, `link`, `http`, `quota`, `rng`, `tls` -- because the distinctions are
    /// what make this actionable: `clock` clears itself, `http` means check the token,
    /// `network` means look at the router.
    ///
    /// Appended, not inserted; see the note above.
    ShotUploadFailed { reason: Name },
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
            | DebugEvent::InterlockTripped { .. }
            // Warn rather than Error: a shot that failed to upload is still on the card,
            // and the browser can fetch it. Nothing has been lost yet.
            | DebugEvent::ShotUploadFailed { .. } => Severity::Warn,
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
            DebugEvent::ShotUploaded { .. } => "shot_uploaded",
            DebugEvent::ShotUploadFailed { .. } => "shot_upload_failed",
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

#[cfg(test)]
mod tests {
    use super::*;

    /// The bound the check-in report is sized against, asserted where it can be edited.
    ///
    /// `Vec<CheckinEntry, MAX_CHECKINS>` is the largest `DebugPayload` variant it can
    /// become, and it only avoids growing `DebugFrame` because 24 * 8 lands exactly on
    /// `MAX_SAMPLES * 8`. Widening `CheckinEntry` -- an `Option<u32>` age, a per-entry id,
    /// a second detail byte that pushes past the alignment -- costs nothing here and
    /// breaks `debug_frame_stays_small` in `variegated-debug-codec`, two crates away from
    /// the edit. This says so at the edit instead.
    #[test]
    fn a_checkin_entry_is_eight_bytes() {
        assert_eq!(
            core::mem::size_of::<CheckinEntry>(),
            8,
            "CheckinEntry grew; see MAX_CHECKINS for what that costs DebugFrame"
        );
        assert_eq!(
            core::mem::size_of::<[CheckinEntry; MAX_CHECKINS]>(),
            core::mem::size_of::<[u64; MAX_SAMPLES]>(),
            "the check-in report must stay no larger than the sample payload it is sized against"
        );
    }
}
