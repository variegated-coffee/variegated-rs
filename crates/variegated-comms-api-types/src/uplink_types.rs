//! The wire types for the Plantlet uplink.
//!
//! A separate envelope from [`ws_types::WsMessage`], carrying the same payload types. The
//! separation is the security boundary: the LAN socket may grow a variant tomorrow, and that
//! variant must not become sendable to a remote server because someone forgot a match arm.
//!
//! # Why not reuse `WsMessage`
//!
//! It would have been less code. It also carries [`MachineCommand`], which is authority over
//! what the machine physically does — boiler setpoints, valve openness, routine execution.
//! That authority does not cross this link, and the way to guarantee it is for the type to
//! have no way to express it, not for a filter to remember to reject it.
//!
//! What *is* reused is everything below the envelope: [`Status`], [`RoutineSummaryStorage`],
//! [`QueryOutcome`] and its two halves. Those already say exactly the right things, and the
//! firmware's existing `ClientQuery` handler arms transfer to [`UplinkQuery`] almost verbatim.
//!
//! # Append only
//!
//! postcard encodes an enum as its declaration-order discriminant, and this transport carries
//! no version byte. Reordering or inserting a variant is undetectable at run time: a machine
//! already in the field decodes by position and gets a different message than was sent.
//! `tests` below pins every discriminant for that reason. Appending is safe; nothing else is.
//!
//! [`ws_types::WsMessage`]: crate::ws_types::WsMessage
//! [`MachineCommand`]: variegated_controller_types::MachineCommand

use serde::{Deserialize, Serialize};
use variegated_controller_types::{MachineMode, RoutineIndex, Status};

use crate::api_types::RoutineSummaryStorage;
use crate::ws_types::{EncodedPayload, QueryOutcome};

/// The revision of this protocol that the generated TypeScript describes.
///
/// Bumped when [`UplinkMessage`] or anything reachable from it changes shape — which, since
/// both enums are append-only, means whenever a variant or field is added. Each version gets
/// its own frozen schema file on the Plantlet side, for the reason the shot log does: postcard
/// is positional, so a schema that followed the Rust types forward would mis-decode every
/// message from a machine that had not been reflashed rather than failing on one.
///
/// Unlike `SHOT_LOG_FORMAT_VERSION` this is *not* on the wire. A stored shot outlives the
/// firmware that wrote it and has to be self-describing; a live session does not, because both
/// ends are reachable and a mismatch shows up immediately as a refused handshake.
pub const UPLINK_SCHEMA_VERSION: u32 = 1;

/// Plaintext bytes per sealed frame within a record.
///
/// Matches `SHOT_LOG_CHUNK_LEN` and the Noise upload's `PLAINTEXT_CHUNK`, because the frames
/// are the chunks the inter-processor link already delivers. A shot is streamed from the
/// application processor one of these at a time and sealed straight onto the wire, so choosing
/// a different number here would mean buffering to re-chunk it.
pub const UPLINK_CHUNK: u32 = 1024;

/// The largest record Plantlet will accept from a machine.
///
/// Cloudflare's ceiling on a single WebSocket message, and therefore not ours to raise. It is
/// what puts shot logs over roughly a megabyte onto the POST transport instead: the firmware
/// picks by size, and this is the threshold it picks against.
pub const MAX_UPLINK_RECORD_LEN: usize = 1024 * 1024;

/// The largest record a *machine* will accept from Plantlet.
///
/// Deliberately far smaller than [`MAX_UPLINK_RECORD_LEN`], and reusing the LAN socket's
/// inbound bound rather than inventing a second one: the largest legitimate thing Plantlet can
/// say is a routine write, which is exactly what that constant was sized for. The two
/// directions are not symmetric and sizing them together would size the constrained one for
/// the unconstrained one.
pub const MAX_UPLINK_CLIENT_RECORD_LEN: usize = crate::ws_types::MAX_CLIENT_FRAME_LEN;

const _: () = assert!(MAX_UPLINK_CLIENT_RECORD_LEN <= MAX_UPLINK_RECORD_LEN);

/// How often an unprompted status goes up while the machine is on.
pub const STATUS_INTERVAL_ACTIVE_SECS: u64 = 60;

/// How often one goes up while it is off or in power save.
pub const STATUS_INTERVAL_IDLE_SECS: u64 = 600;

/// How often one goes up while the machine is brewing or running a routine.
///
/// A shot is thirty seconds long and is the one thing on this machine somebody watches second
/// by second: a pressure curve sampled once a minute is one point.
pub const STATUS_INTERVAL_BUSY_SECS: u64 = 1;

/// How long the busy cadence may run before it is read as a stuck flag rather than a shot.
///
/// **A failsafe, not an operational timeout.** No brew and no routine runs five minutes, so
/// this cannot fire on working hardware -- which is the test it has to pass, because what it
/// bounds is the case where the *machine* is wrong. `Status::routine_execution` that is never
/// cleared, or an `is_brewing` left set by a controller that lost its way, would otherwise put a
/// machine on the one-second cadence indefinitely: 3,600 Durable Object wakes and 3,600 D1
/// writes an hour, forever, for a machine doing nothing.
///
/// Five minutes caps one such episode at 300 statuses and then falls back to the interval the
/// mode asks for. It re-arms when the flag next goes false, so a machine that recovers is not
/// punished for having been wrong once.
pub const BUSY_CADENCE_LIMIT_SECS: u64 = 300;

/// How many statuses follow a command from Plantlet, and how far apart.
///
/// A command is the one moment somebody is looking at a page waiting for it to change, and a
/// single status afterwards is a coin toss: it may be read before the application processor has
/// applied the command, in which case the page says nothing happened. Five of them a second
/// apart cover the whole window it takes to cross the link and come back, and the first one
/// going out immediately means the page acknowledges the click at once even if what it carries
/// is still the old state.
///
/// This replaced a single delayed status -- the old `COMMAND_SETTLE` -- which had to guess how
/// long the link would take and reported the old state whenever it guessed low.
pub const COMMAND_BURST_COUNT: u8 = 5;

/// The spacing of the statuses in a post-command burst. See [`COMMAND_BURST_COUNT`].
pub const COMMAND_BURST_INTERVAL_SECS: u64 = 1;

/// How long until the next unprompted status, given what the machine is doing.
///
/// A status is the one thing on this link that is *not* free: a record wakes the hibernating
/// Durable Object and costs a row written in two places, where a keepalive ping is answered by
/// Cloudflare's runtime without waking anything. Watching a switched-off machine once a minute
/// is most of what Plantlet costs to run, and it buys nothing — the machine is not doing
/// anything, and whether it is *reachable* is answered by the socket rather than by this.
///
/// So the cadence follows the machine: once a second while it is pulling a shot, a minute while
/// someone might be watching a temperature climb, ten minutes while it sits there off.
///
/// **This lives here, and not in the firmware's uplink loop, because it is a pure function and
/// the firmware crate runs no tests** — it sets `harness = false`, so a `#[test]` beside the
/// loop would neither run nor say it had not. Here it is covered.
///
/// Exhaustive on purpose: a fourth [`MachineMode`] should stop the build and be given an
/// interval deliberately, rather than defaulting into ten-minute silence.
///
/// **`None` means the mode is not known yet, and gets the active interval** — not the default
/// one. `MachineMode::default()` is `Off`, so anything that reached for a default here would
/// put a machine that has just booted, and may well be on, into ten minutes of silence. The
/// caller passes the `Option` through rather than resolving it precisely so that this case is
/// decided here, where it can be tested.
///
/// # Busy, and why it outranks the mode
///
/// `busy_for_secs` is `None` when the machine is not doing anything, and otherwise how long it
/// has been continuously busy — see `Status::is_busy`, which is what "busy" means. The caller
/// owns that clock, so this stays a pure function of durations rather than something that has to
/// be tested against a timer.
///
/// A machine reporting brewing while `Off` is contradictory, and this resolves it in favour of
/// **busy**: the useful way to be wrong is to report the thing that is happening. That is only
/// safe to say because [`BUSY_CADENCE_LIMIT_SECS`] bounds what a stuck flag can cost — past the
/// limit this falls back to whatever the mode asks for, which is exactly the ten-minute silence
/// an `Off` machine deserves.
pub fn status_interval_secs(mode: Option<MachineMode>, busy_for_secs: Option<u64>) -> u64 {
    match busy_for_secs {
        Some(elapsed) if elapsed < BUSY_CADENCE_LIMIT_SECS => STATUS_INTERVAL_BUSY_SECS,
        // Either not busy, or busy for longer than any real shot -- see the constant.
        _ => match mode {
            Some(MachineMode::On) => STATUS_INTERVAL_ACTIVE_SECS,
            // Standby is not distinguished from off anywhere in the controllers, and there is
            // nothing to watch in either: the boilers are cold and the machine is waiting to be
            // asked for something.
            Some(MachineMode::Off | MachineMode::PowerSaveStandby) => STATUS_INTERVAL_IDLE_SECS,
            // Only in the seconds between the socket opening and the first status arriving from
            // the application processor. Reporting too often for a moment is the harmless
            // direction to be wrong in.
            None => STATUS_INTERVAL_ACTIVE_SECS,
        },
    }
}

/// Whether a pushed update should wait for the next scheduled status.
///
/// A status is not the only thing that wakes the hibernating Durable Object. The machine also
/// pushes its configuration whenever a setting moves and its routine list whenever somebody
/// saves a routine — and on a machine nobody is looking at, neither is worth a wake of its own.
/// Deferred, they ride along with the next status, which on a sleeping machine is ten minutes
/// away. Piggybacking rather than running a second timer is what keeps this exactly one wake
/// per interval and stops the two cadences drifting apart.
///
/// Deferring means *waiting*, never dropping: the send still happens, and
/// `send_configuration` compares bytes at that point so an update that turned out to be a
/// no-op costs nothing.
///
/// **`None` does not defer**, for the same reason it takes the active interval above: a machine
/// that has not said what it is doing yet may well be on, with its owner watching.
///
/// This does not reach the sends a session makes when it opens, nor an explicit
/// `RequestConfiguration` or `RequestRoutineList` — a server that has just connected knowing
/// nothing is not the case this is about, and an ask always gets an answer.
pub fn defer_updates(mode: Option<MachineMode>) -> bool {
    matches!(mode, Some(MachineMode::Off | MachineMode::PowerSaveStandby))
}

/// Which way a message is allowed to travel.
///
/// See [`UplinkMessage::direction`] for why this exists as a type rather than as a rule
/// people remember.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    /// Machine to Plantlet. Plantlet accepts these; the machine refuses them inbound.
    Uplink,
    /// Plantlet to machine. The machine accepts these; Plantlet refuses them inbound.
    Downlink,
}

/// The uplink envelope. **Append only** — see the module docs.
#[derive(Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
pub enum UplinkMessage {
    /// The machine's state, whole and unprojected.
    ///
    /// Sent on connect, on the interval [`status_interval_secs`] gives for the machine's current
    /// mode, whenever that mode changes, and in answer to [`Self::RequestStatus`]. Plantlet
    /// stores the bytes and derives what it displays from them, so a field added here reaches
    /// the server without a server change.
    Status(Status),

    /// Every routine the machine holds, summarised.
    ///
    /// Summaries, not definitions — the comms processor does not hold definitions and should
    /// not start. A definition is fetched one at a time through
    /// [`UplinkQuery::RoutineDefinition`].
    RoutineList(RoutineSummaryStorage),

    /// A complete shot log, postcard bytes verbatim.
    ///
    /// A `Vec<u8>` in a message that is otherwise small, and that is deliberate: it is how a
    /// ~90 kB shot rides the socket without the firmware ever holding one. The frames of a
    /// record are streamed as WebSocket continuation frames, so the sender writes this
    /// variant's discriminant and length prefix by hand and then streams the body. See
    /// [`EncodedPayload`].
    ///
    /// Shots over [`MAX_UPLINK_RECORD_LEN`] cannot travel this way and take the POST
    /// transport instead.
    ShotLog(EncodedPayload),

    /// The answer to a [`Self::Query`].
    Reply {
        /// The `id` of the query this answers.
        id: u32,
        outcome: QueryOutcome,
    },

    /// Ask the machine to send a [`Self::Status`] now.
    ///
    /// A trigger, not a query: it provokes the ordinary unsolicited push rather than a
    /// correlated reply, so a requested status reaches Plantlet by exactly the same path as a
    /// scheduled one. `WsMessage::RequestConfiguration` already works this way.
    RequestStatus,

    /// Ask the machine to send a [`Self::RoutineList`] now. A trigger, as above.
    RequestRoutineList,

    /// Ask the machine something that has an answer.
    Query {
        /// Correlation id, echoed in the [`Self::Reply`] this provokes.
        id: u32,
        query: UplinkQuery,
    },

    /// What the machine *is*: its boilers, groups, taps, wands, tanks and function buttons.
    ///
    /// Sent on connect and in answer to [`Self::RequestMachineDefinition`], and almost never
    /// otherwise -- a machine definition is a property of the hardware, so unlike [`Status`]
    /// there is nothing to poll for.
    ///
    /// It is here for one concrete thing a server cannot otherwise know: `function_routines`
    /// maps a slot number to the name of the panel button bound to it. Without it Plantlet
    /// can only offer a bare number when asked to put a routine on a button, bounded by
    /// `MAX_FUNCTION_ROUTINES` and meaning nothing to the person choosing.
    MachineDefinition(variegated_controller_types::MachineDefinition),

    /// Ask the machine to send a [`Self::MachineDefinition`] now. A trigger, as above.
    ///
    /// Worth having despite the definition arriving on connect: a server that added a column
    /// for it, or lost one, has no other way to fill it without waiting for the machine to
    /// reconnect -- which for a machine that is behaving itself could be weeks.
    RequestMachineDefinition,

    /// Every setting the machine holds, and its schedules.
    ///
    /// Sent on connect and again whenever it changes, which is what makes it the answer to a
    /// [`Self::Command`] as well as a thing in its own right: a setpoint that was accepted
    /// shows up here within a second, and one that was not does not. There is deliberately no
    /// per-command acknowledgement — see the note on `Command`.
    ///
    /// Schedules ride inside it. `Configuration.schedules` already carries them for the LAN
    /// frontend, so scheduling needed no message of its own; only the three commands that
    /// change them did.
    Configuration(variegated_controller_types::Configuration),

    /// Ask the machine to send a [`Self::Configuration`] now. A trigger, as above.
    RequestConfiguration,

    /// Tell the machine to do one of the seven things Plantlet may ask for.
    ///
    /// # Why no acknowledgement
    ///
    /// A delete gets a correlated reply and this does not, which is a real difference and not
    /// an inconsistency. A routine delete is destructive, its failure modes are silent, and
    /// nothing else reports them — so the query had to. A command here changes a setting whose
    /// new value is republished as [`Self::Configuration`] within a second, on a channel the
    /// server is already reading. The echo *is* the acknowledgement, and it is a better one
    /// than an ack: an ack says the machine received it, and the configuration says what the
    /// machine now believes.
    Command(UplinkCommand),
}

/// What Plantlet may ask a machine for. Reached only through [`UplinkMessage::Query`], so its
/// discriminants are a contract with deployed firmware exactly as the envelope's are.
/// **Append only.**
///
/// Note what is absent. `ClientQuery` on the LAN socket also carries `ShotLogPage`; this does
/// not, because a remote server has no business enumerating the card. The omission is the
/// point — there is no filter to forget.
#[derive(Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
pub enum UplinkQuery {
    /// Fetch one routine's definition, so Plantlet can hash it and match it to the library.
    RoutineDefinition(RoutineIndex),

    /// Store a routine.
    ///
    /// `index: None` creates and the machine assigns the slot, which comes back as
    /// `QueryOk::RoutineStored` — the only way a create learns where it landed. `Some(..)`
    /// writes in place.
    WriteRoutine {
        index: Option<RoutineIndex>,
        /// A postcard-encoded `Routine`, passed through opaquely. See [`EncodedPayload`].
        routine: EncodedPayload,
    },

    /// Remove a routine.
    ///
    /// Note what this is *not*. `MachineCommand::RemoveRoutine` already exists and already
    /// works, and it is not what travels here: `MachineCommand` is the authority this whole
    /// module is built to keep off the link, and admitting one variant of it would be
    /// admitting the type. What crosses is a query with a bounded meaning -- remove this
    /// routine, and say what happened -- which is a strictly smaller grant than "run
    /// arbitrary commands", and it is the type that says so rather than a filter.
    ///
    /// A query rather than a trigger for the reason [`Self::WriteRoutine`] is one: the
    /// command form is fire-and-forget, so a delete that failed on a worn flash sector was
    /// indistinguishable from one that worked, and Plantlet would have dropped the row
    /// either way.
    DeleteRoutine(RoutineIndex),
}

/// The postcard prefix of [`UplinkMessage::ShotLog`], for a payload of `total` bytes.
///
/// # Why this exists rather than just encoding the message
///
/// Encoding `ShotLog(bytes)` means building the `Vec<u8>` it wraps, and a shot is exactly the
/// thing the firmware cannot hold in memory — it arrives a kilobyte at a time and is sealed as
/// it flows. So the prefix is written by hand and the shot follows it straight off the link.
///
/// Here rather than in the firmware because it is wire knowledge, and because the firmware
/// crate sets `harness = false` and so runs no tests: written there, nothing would check it
/// against the encoder it has to agree with. A prefix that disagreed would not be a corrupt
/// shot — it would decode as a *different message*.
/// A buffer and its length, rather than a `Vec`: this runs on the constrained side, and a
/// heap allocation for at most five bytes would be the only one on the whole shot path.
pub fn shot_log_prefix(total: u32) -> ([u8; 8], usize) {
    let mut prefix = [0u8; 8];
    // `ShotLog` is discriminant 2, and a `Vec<u8>` is a varint length then the bytes.
    prefix[0] = 2;
    let mut len = 1;

    let mut remaining = total;
    loop {
        let mut byte = (remaining & 0x7f) as u8;
        remaining >>= 7;
        if remaining > 0 {
            byte |= 0x80;
        }
        prefix[len] = byte;
        len += 1;
        if remaining == 0 {
            break;
        }
    }
    (prefix, len)
}

/// What Plantlet may tell a machine to *do*. **Append only.**
///
/// # This is the boundary this module is about, so read the whole note
///
/// The module docs above say `MachineCommand` does not cross this link, because it is
/// authority over what the machine physically does. That is still true and still the design:
/// what crosses is this, which is a strict subset with a total conversion into it — the same
/// shape [`UplinkQuery`] has against `ClientQuery`, and the same shape
/// [`ScheduleAction`](variegated_controller_types::ScheduleAction) already had against
/// `MachineCommand` before this existed. The guarantee is unchanged: a command that is not
/// here cannot be expressed, so there is no filter to forget.
///
/// What *has* changed is where the line sits, and it moved deliberately. These seven are what
/// the ESPHome integration already exposes on the local network — `command_mapper.rs` maps
/// exactly four `MachineCommand`s and the rest of its hundred-odd entities are read-only —
/// plus the three schedule operations, which have no local equivalent because the LAN
/// frontend edits them through `Configuration`.
///
/// **The exposure is genuinely wider than ESPHome's**, and that is worth being clear about
/// rather than eliding: ESPHome answers on the LAN, where being on the network is most of the
/// authorization, and this answers to a server on the internet. What bounds it is that the set
/// is small, enumerated, and every member of it is something the machine's own front panel can
/// already do.
///
/// Note what is still absent, and not by oversight. There is no `RunRoutine` and no
/// `CancelRoutine` — a remote server starting a shot on an unattended machine is a different
/// kind of authority from adjusting a setpoint, and nothing asks for it. `ScheduleAction` may
/// carry `RunRoutine`, and that is not a contradiction: a schedule runs on the machine's own
/// clock and the person who set it is the person standing next to it.
#[derive(Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
pub enum UplinkCommand {
    /// On, Off, or PowerSaveStandby.
    SetMachineMode(variegated_controller_types::MachineMode),

    /// A boiler's control mode, and optionally its targets in the same message.
    ///
    /// The optional values are why this and [`Self::SetBoilerControlTargetValues`] are both
    /// here rather than one being expressible as the other: switching a boiler from
    /// temperature to pressure control *and* setting the pressure is one atomic change, where
    /// two messages would leave it briefly holding a pressure target it was given for the
    /// previous mode.
    SetBoilerControlTarget(
        variegated_controller_types::BoilerIndex,
        variegated_controller_types::BoilerControlMode,
        Option<variegated_controller_types::BoilerControlTargetValuesUpdate>,
    ),

    /// A boiler's targets, leaving its control mode alone.
    SetBoilerControlTargetValues(
        variegated_controller_types::BoilerIndex,
        variegated_controller_types::BoilerControlTargetValuesUpdate,
    ),

    /// PID gains and limits for one control loop.
    ///
    /// Whole rather than per-gain, which is what ESPHome's mapper has to reconstruct: it reads
    /// the current configuration, edits one field and sends the lot. Sending the lot directly
    /// removes the read-modify-write, and with it the race two clients editing kP and kI at
    /// once would otherwise have.
    SetPidParameters(
        variegated_controller_types::PidParameterTarget,
        variegated_controller_types::PidParameters,
    ),

    /// Add a schedule item. The machine assigns its index.
    AddScheduleItem(variegated_controller_types::ScheduleItem),

    /// Remove the schedule item at an index.
    RemoveScheduleItem(u32),

    /// Replace the schedule item at an index.
    UpdateScheduleItem(u32, variegated_controller_types::ScheduleItem),
}

/// Widen an uplink command into the machine command it stands for.
///
/// Total and lossless, exactly as the query conversion below is, and that totality is what
/// makes it safe: every arm is a `MachineCommand` this enum already named, so there is no arm
/// where something is dropped and none where something is invented. Nothing converts the other
/// way — `MachineCommand` has some sixty variants and all but seven of them have no
/// `UplinkCommand` to become, which is the asymmetry the whole module is built around.
impl From<UplinkCommand> for variegated_controller_types::MachineCommand {
    fn from(command: UplinkCommand) -> Self {
        match command {
            UplinkCommand::SetMachineMode(mode) => Self::SetMachineMode(mode),
            UplinkCommand::SetBoilerControlTarget(index, mode, values) => {
                Self::SetBoilerControlTarget(index, mode, values)
            }
            UplinkCommand::SetBoilerControlTargetValues(index, values) => {
                Self::SetBoilerControlTargetValues(index, values)
            }
            UplinkCommand::SetPidParameters(target, parameters) => {
                Self::SetPidParameters(target, parameters)
            }
            UplinkCommand::AddScheduleItem(item) => Self::AddScheduleItem(item),
            UplinkCommand::RemoveScheduleItem(index) => Self::RemoveScheduleItem(index),
            UplinkCommand::UpdateScheduleItem(index, item) => {
                Self::UpdateScheduleItem(index, item)
            }
        }
    }
}

/// Widen an uplink query into the one the firmware already knows how to serve.
///
/// The two enums are separate types on purpose — the note above is about what `UplinkQuery`
/// deliberately cannot express — but *answering* one is the same work either way: a routine
/// read is chunks off the inter-processor link, a write is a round trip with a five-way
/// refusal, and neither has heard of a transport. So the firmware serves both through one
/// implementation, and this is the only place that knows the two shapes coincide.
///
/// Total and lossless, which is what makes it safe: `UplinkQuery` is a strict subset, so this
/// cannot fail and there is no arm where something is dropped. Note the direction — nothing
/// converts the other way, because a `ShotLogPage` has no `UplinkQuery` to become, and that
/// asymmetry is the omission this module is built around.
impl From<UplinkQuery> for crate::ws_types::ClientQuery {
    fn from(query: UplinkQuery) -> Self {
        match query {
            UplinkQuery::RoutineDefinition(index) => Self::RoutineDefinition(index),
            UplinkQuery::WriteRoutine { index, routine } => Self::WriteRoutine { index, routine },
            UplinkQuery::DeleteRoutine(index) => Self::DeleteRoutine(index),
        }
    }
}

impl UplinkMessage {
    /// Which way this variant is allowed to travel.
    ///
    /// # Why this is a function and not a comment
    ///
    /// This match has no `_` arm, and that is the whole mechanism. Appending a variant to
    /// [`UplinkMessage`] fails to compile until it is classified here, so "which side may send
    /// this" is answered at the moment the variant is written rather than at the moment
    /// someone remembers to update a filter. A runtime allow-list would have been shorter and
    /// would have failed open the first time a `_ =>` arm was added to quiet the compiler.
    pub fn direction(&self) -> Direction {
        match self {
            Self::Status(_)
            | Self::RoutineList(_)
            | Self::ShotLog(_)
            | Self::Reply { .. }
            | Self::MachineDefinition(_)
            | Self::Configuration(_) => Direction::Uplink,
            Self::RequestStatus
            | Self::RequestRoutineList
            | Self::Query { .. }
            | Self::RequestMachineDefinition
            | Self::RequestConfiguration
            | Self::Command(_) => Direction::Downlink,
        }
    }

    /// Whether a machine may act on this having received it.
    ///
    /// The firmware calls this before dispatch. A machine receiving `Status` is either a bug
    /// or someone reflecting its own traffic back at it; either way there is nothing sensible
    /// to do with it.
    pub fn acceptable_by_machine(&self) -> bool {
        matches!(self.direction(), Direction::Downlink)
    }

    /// Whether Plantlet may act on this having received it.
    pub fn acceptable_by_server(&self) -> bool {
        matches!(self.direction(), Direction::Uplink)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;
    use variegated_controller_types::RoutineWriteError;

    use crate::ws_types::{QueryError, QueryOk};

    /// The whole point of the adaptive cadence: an idle machine costs a tenth as much.
    #[test]
    fn an_idle_machine_reports_a_tenth_as_often() {
        assert_eq!(status_interval_secs(Some(MachineMode::On), None), 60);
        assert_eq!(status_interval_secs(Some(MachineMode::Off), None), 600);
    }

    /// The other end of the same idea: a shot is watched second by second.
    #[test]
    fn a_brewing_machine_reports_every_second() {
        assert_eq!(status_interval_secs(Some(MachineMode::On), Some(0)), 1);
        assert_eq!(status_interval_secs(Some(MachineMode::On), Some(29)), 1);
    }

    /// Busy outranks the mode, and it has to: `is_brewing` and `mode` come from the same
    /// status, so a machine that reports both `Off` and brewing is telling us something is
    /// happening. Reporting it is the useful way to be wrong; [`BUSY_CADENCE_LIMIT_SECS`] is
    /// what makes saying so affordable.
    #[test]
    fn busy_beats_every_mode() {
        for mode in [
            Some(MachineMode::On),
            Some(MachineMode::Off),
            Some(MachineMode::PowerSaveStandby),
            None,
        ] {
            assert_eq!(
                status_interval_secs(mode, Some(0)),
                STATUS_INTERVAL_BUSY_SECS,
                "{mode:?} while busy"
            );
        }
    }

    /// The failsafe. A `routine_execution` that is never cleared would otherwise hold a machine
    /// at one status a second forever -- 3,600 Durable Object wakes an hour for a machine doing
    /// nothing. Past the limit the mode decides again.
    #[test]
    fn a_stuck_busy_flag_falls_back_to_the_modes_interval() {
        assert_eq!(
            status_interval_secs(Some(MachineMode::On), Some(BUSY_CADENCE_LIMIT_SECS - 1)),
            STATUS_INTERVAL_BUSY_SECS
        );
        assert_eq!(
            status_interval_secs(Some(MachineMode::On), Some(BUSY_CADENCE_LIMIT_SECS)),
            STATUS_INTERVAL_ACTIVE_SECS
        );
        // And a machine that is *also* off goes all the way back to silence, rather than
        // stopping at the active interval.
        assert_eq!(
            status_interval_secs(Some(MachineMode::Off), Some(BUSY_CADENCE_LIMIT_SECS)),
            STATUS_INTERVAL_IDLE_SECS
        );
    }

    /// The limit has to be longer than any real shot or routine, or it stops being a failsafe
    /// and starts being a thing that fires on working hardware.
    #[test]
    fn the_busy_limit_cannot_fire_on_a_real_shot() {
        // A long blooming routine is a few minutes; a shot is well under one.
        const LONGEST_PLAUSIBLE_ROUTINE_SECS: u64 = 4 * 60;
        assert!(BUSY_CADENCE_LIMIT_SECS > LONGEST_PLAUSIBLE_ROUTINE_SECS);
    }

    /// A burst has to be over well before the cadence it interrupts would have sent anything,
    /// or the two would interleave and a command would leave the machine reporting faster than
    /// it should for a minute.
    #[test]
    fn a_command_burst_finishes_inside_the_active_interval() {
        let burst = COMMAND_BURST_COUNT as u64 * COMMAND_BURST_INTERVAL_SECS;
        assert!(burst < STATUS_INTERVAL_ACTIVE_SECS, "a {burst}s burst");
    }

    /// Which modes are "asleep" is decided twice -- once for the interval, once for whether a
    /// pushed update waits for it -- and the two must not drift. A fourth `MachineMode` given
    /// an interval in one and forgotten in the other fails here.
    #[test]
    fn deferring_updates_agrees_with_the_idle_interval() {
        for mode in [
            Some(MachineMode::On),
            Some(MachineMode::Off),
            Some(MachineMode::PowerSaveStandby),
            None,
        ] {
            assert_eq!(
                defer_updates(mode),
                status_interval_secs(mode, None) == STATUS_INTERVAL_IDLE_SECS,
                "{mode:?}"
            );
        }
    }

    /// A machine that has not reported yet pushes immediately, for the same reason it reports
    /// on the active interval: it may well be on, with somebody watching.
    #[test]
    fn an_unknown_mode_does_not_defer_updates() {
        assert!(!defer_updates(None));
        assert!(!defer_updates(Some(MachineMode::On)));
        assert!(defer_updates(Some(MachineMode::Off)));
        assert!(defer_updates(Some(MachineMode::PowerSaveStandby)));
    }

    /// Standby is idle, not active.
    ///
    /// Worth its own assertion because `PowerSaveStandby` is the mode a machine reaches *on its
    /// own*, from a schedule, with nobody watching — so it is both the one most likely to be
    /// overlooked here and the one that spends the most hours in a real day.
    #[test]
    fn standby_reports_as_rarely_as_off() {
        assert_eq!(
            status_interval_secs(Some(MachineMode::PowerSaveStandby), None),
            status_interval_secs(Some(MachineMode::Off), None)
        );
    }

    /// A machine that has not said what it is doing yet reports *often*, not rarely.
    ///
    /// The trap this pins: [`MachineMode::default()`] is `Off`, so any resolution of the unknown
    /// case that reaches for a default puts a freshly booted machine — which may well be on, and
    /// whose owner is watching the page — into ten minutes of silence. This is the only reason
    /// the function takes an `Option` instead of the caller resolving it.
    #[test]
    fn a_machine_that_has_not_reported_yet_uses_the_active_interval() {
        assert_eq!(status_interval_secs(None, None), STATUS_INTERVAL_ACTIVE_SECS);
        assert_ne!(
            status_interval_secs(None, None),
            status_interval_secs(Some(MachineMode::default()), None)
        );
    }

    /// An idle machine maps to one of the two mode intervals, and nothing invents a fourth.
    ///
    /// The busy interval is deliberately outside this: it is not a property of the mode, and
    /// `busy_beats_every_mode` is what covers it.
    #[test]
    fn every_mode_maps_to_one_of_the_two_idle_intervals() {
        for mode in [
            MachineMode::On,
            MachineMode::Off,
            MachineMode::PowerSaveStandby,
        ] {
            let interval = status_interval_secs(Some(mode), None);
            assert!(
                interval == STATUS_INTERVAL_ACTIVE_SECS || interval == STATUS_INTERVAL_IDLE_SECS,
                "{mode:?} mapped to {interval}"
            );
        }
    }

    /// Two idle intervals have to fit inside the server's lease, with room to spare.
    ///
    /// The server calls a machine connected until `CONNECTED_LEASE_MS` after its last status
    /// (`variegated-plantlet-ts/apps/worker/src/uplink.ts`). If that lease ever fell below this
    /// interval, every idle machine would drop off the listing between statuses while being
    /// perfectly healthy. The two constants live in different languages and cannot share a
    /// `const` assert, so this test is the joint.
    #[test]
    fn the_idle_interval_leaves_room_inside_the_server_lease() {
        const SERVER_LEASE_SECS: u64 = 25 * 60;
        assert!(STATUS_INTERVAL_IDLE_SECS * 2 < SERVER_LEASE_SECS);
    }

    /// Pins the discriminant of every variant.
    ///
    /// This transport carries no version byte, so reordering the enum is undetectable at run
    /// time — a machine in the field decodes by position and silently gets a different
    /// message than was sent. These numbers are a contract with already-flashed firmware, not
    /// with a sibling declaration. Appending is safe; anything else is not.
    ///
    /// Encoded rather than hand-asserted so the check exercises the same serde path the
    /// transport uses.
    #[test]
    fn variant_discriminants_are_pinned() {
        let cases: [(UplinkMessage, u8); 12] = [
            (UplinkMessage::Status(Status::new()), 0),
            (
                UplinkMessage::RoutineList(RoutineSummaryStorage {
                    internal: Default::default(),
                    function: Default::default(),
                    custom: Default::default(),
                }),
                1,
            ),
            (UplinkMessage::ShotLog(vec![]), 2),
            (
                UplinkMessage::Reply {
                    id: 1,
                    outcome: QueryOutcome::Failed(QueryError::NotFound),
                },
                3,
            ),
            (UplinkMessage::RequestStatus, 4),
            (UplinkMessage::RequestRoutineList, 5),
            (
                UplinkMessage::Query {
                    id: 1,
                    query: UplinkQuery::RoutineDefinition(RoutineIndex::Custom(0)),
                },
                6,
            ),
            (
                UplinkMessage::MachineDefinition(
                    variegated_controller_types::MachineDefinition::default(),
                ),
                7,
            ),
            (UplinkMessage::RequestMachineDefinition, 8),
            (
                UplinkMessage::Configuration(
                    variegated_controller_types::Configuration::default(),
                ),
                9,
            ),
            (UplinkMessage::RequestConfiguration, 10),
            (
                UplinkMessage::Command(UplinkCommand::SetMachineMode(
                    variegated_controller_types::MachineMode::Off,
                )),
                11,
            ),
        ];

        for (message, expected) in cases {
            let bytes = postcard::to_allocvec(&message).expect("encodes");
            assert_eq!(
                bytes[0], expected,
                "a discriminant moved -- this is a contract with flashed firmware"
            );
        }
    }

    /// `UplinkCommand`'s discriminants are a contract too, and its widening must not cross.
    ///
    /// Reached through `Command`, so the envelope test does not see it — the same gap the
    /// query enums had. It matters more here than there: these are the messages that change
    /// what the machine physically does, and adjacent variants carry near-identical payloads.
    /// `SetBoilerControlTarget` and `SetBoilerControlTargetValues` both start with a boiler
    /// index, and `RemoveScheduleItem` and `UpdateScheduleItem` both start with a `u32`, so in
    /// each pair only position tells the machine which one it was told to do.
    #[test]
    fn uplink_command_discriminants_are_pinned() {
        use variegated_controller_types::{
            BoilerControlMode, BoilerControlTargetValuesUpdate, MachineCommand, MachineMode,
            ScheduleItem,
        };

        let values = BoilerControlTargetValuesUpdate {
            temperature: Some(93.0),
            pressure: None,
        };

        let cases: [(UplinkCommand, u8); 7] = [
            (UplinkCommand::SetMachineMode(MachineMode::On), 0),
            (
                UplinkCommand::SetBoilerControlTarget(0, BoilerControlMode::Temperature, None),
                1,
            ),
            (UplinkCommand::SetBoilerControlTargetValues(0, values), 2),
            (
                UplinkCommand::SetPidParameters(
                    variegated_controller_types::PidParameterTarget::BoilerTemperature(0),
                    Default::default(),
                ),
                3,
            ),
            (UplinkCommand::AddScheduleItem(ScheduleItem::default()), 4),
            (UplinkCommand::RemoveScheduleItem(7), 5),
            (
                UplinkCommand::UpdateScheduleItem(7, ScheduleItem::default()),
                6,
            ),
        ];

        for (command, expected) in cases {
            let bytes = postcard::to_allocvec(&command).expect("encodes");
            assert_eq!(
                bytes[0], expected,
                "an UplinkCommand discriminant moved -- a machine in the field would act on a \
                 different command than it was sent"
            );
        }

        // Each widens into its own `MachineCommand` and not its neighbour. The compiler makes
        // the conversion exhaustive; nothing but this makes it *correct*.
        let widened: [(UplinkCommand, &str); 7] = [
            (UplinkCommand::SetMachineMode(MachineMode::On), "SetMachineMode"),
            (
                UplinkCommand::SetBoilerControlTarget(0, BoilerControlMode::Temperature, None),
                "SetBoilerControlTarget",
            ),
            (
                UplinkCommand::SetBoilerControlTargetValues(0, values),
                "SetBoilerControlTargetValues",
            ),
            (
                UplinkCommand::SetPidParameters(
                    variegated_controller_types::PidParameterTarget::BoilerTemperature(0),
                    Default::default(),
                ),
                "SetPidParameters",
            ),
            (
                UplinkCommand::AddScheduleItem(ScheduleItem::default()),
                "AddScheduleItem",
            ),
            (UplinkCommand::RemoveScheduleItem(7), "RemoveScheduleItem"),
            (
                UplinkCommand::UpdateScheduleItem(7, ScheduleItem::default()),
                "UpdateScheduleItem",
            ),
        ];

        for (command, expected) in widened {
            // `label()` rather than a match: it is the exhaustive naming `MachineCommand`
            // already maintains, so this reads the machine command's own idea of what it is.
            assert_eq!(
                MachineCommand::from(command).label(),
                expected,
                "an uplink command widened into the wrong machine command"
            );
        }
    }

    /// `Status` is the largest variant, and nothing appended may take that from it.
    ///
    /// **This is a memory guard, not a wire guard.** `UplinkMessage` is sized by its largest
    /// variant whatever a given message actually is, and the firmware materialises whole
    /// envelopes by value -- `encode_status` builds one to serialise, and `handle` decodes one
    /// before narrowing to `Downlink`. So the largest variant is what every trigger and every
    /// status costs in stack, once per send and once per receive.
    ///
    /// `Status` is what the firmware is already sized for: it is cached in RAM as a static, so
    /// its cost is paid whether or not this enum exists. A variant that grew past it would be
    /// new cost, and would be invisible -- the enum would simply be bigger and nothing would
    /// say so.
    ///
    /// The margin is genuinely thin. When `MachineDefinition` was added here it measured 4488
    /// bytes against `Status`'s 4496, so it came within eight bytes of moving this number
    /// without anyone noticing. That is the whole reason this test exists.
    #[test]
    fn status_is_the_largest_variant() {
        let envelope = core::mem::size_of::<UplinkMessage>();
        let status = core::mem::size_of::<Status>();

        assert_eq!(
            envelope, status,
            "an appended variant is now larger than `Status`, so the envelope grew -- every \
             send and every receive on the uplink pays the difference in stack. Either shrink \
             it, box it (serde encodes a `Box<T>` as `T`, so the wire is unchanged), or decide \
             deliberately that the envelope may grow and rewrite this test to say so."
        );
    }

    /// The two triggers encode to a single byte each, which is what the firmware's inbound
    /// path sees most often.
    #[test]
    fn triggers_encode_to_one_byte() {
        assert_eq!(
            postcard::to_allocvec(&UplinkMessage::RequestStatus).unwrap(),
            vec![4]
        );
        assert_eq!(
            postcard::to_allocvec(&UplinkMessage::RequestRoutineList).unwrap(),
            vec![5]
        );
    }

    /// `UplinkQuery`'s own discriminants are equally a contract — it is reached through the
    /// envelope, so a reorder here mis-decodes just as silently.
    #[test]
    fn query_discriminants_are_pinned() {
        let definition = postcard::to_allocvec(&UplinkQuery::RoutineDefinition(
            RoutineIndex::Custom(0),
        ))
        .expect("encodes");
        assert_eq!(definition[0], 0);

        let write = postcard::to_allocvec(&UplinkQuery::WriteRoutine {
            index: None,
            routine: vec![],
        })
        .expect("encodes");
        assert_eq!(write[0], 1);

        let delete = postcard::to_allocvec(&UplinkQuery::DeleteRoutine(RoutineIndex::Custom(0)))
            .expect("encodes");
        assert_eq!(delete[0], 2);
    }

    /// Every variant travels exactly one way, and the two acceptance predicates are exact
    /// complements of each other.
    ///
    /// The complement check is the one that matters: it is what makes "the machine refuses
    /// what Plantlet may say, and vice versa" true by construction rather than by two lists
    /// that have to be kept in agreement.
    #[test]
    fn direction_partitions_the_enum() {
        let uplink: [UplinkMessage; 6] = [
            UplinkMessage::Status(Status::new()),
            UplinkMessage::RoutineList(RoutineSummaryStorage {
                internal: Default::default(),
                function: Default::default(),
                custom: Default::default(),
            }),
            UplinkMessage::ShotLog(vec![]),
            UplinkMessage::Reply {
                id: 0,
                outcome: QueryOutcome::Ok(QueryOk::RoutineStored(RoutineIndex::Custom(1))),
            },
            UplinkMessage::MachineDefinition(
                variegated_controller_types::MachineDefinition::default(),
            ),
            UplinkMessage::Configuration(variegated_controller_types::Configuration::default()),
        ];
        let downlink: [UplinkMessage; 6] = [
            UplinkMessage::RequestStatus,
            UplinkMessage::RequestRoutineList,
            UplinkMessage::Query {
                id: 0,
                query: UplinkQuery::WriteRoutine {
                    index: None,
                    routine: vec![],
                },
            },
            UplinkMessage::RequestMachineDefinition,
            UplinkMessage::RequestConfiguration,
            UplinkMessage::Command(UplinkCommand::RemoveScheduleItem(0)),
        ];

        for message in uplink {
            assert_eq!(message.direction(), Direction::Uplink);
            assert!(message.acceptable_by_server());
            assert!(
                !message.acceptable_by_machine(),
                "a machine must refuse what only it may send"
            );
        }
        for message in downlink {
            assert_eq!(message.direction(), Direction::Downlink);
            assert!(message.acceptable_by_machine());
            assert!(
                !message.acceptable_by_server(),
                "Plantlet must refuse what only it may send"
            );
        }
    }

    /// The hand-written shot prefix is what `to_allocvec` would have produced.
    ///
    /// The firmware writes this by hand and then streams the shot after it, so nothing
    /// downstream ever compares the two. If they diverged the record would still decrypt and
    /// still decode -- as a different message, or as a shot of the wrong length -- which is
    /// the kind of disagreement that is only ever found on hardware.
    ///
    /// The sizes cross both varint boundaries a real shot can sit on: 127/128 is one byte to
    /// two, and 16,383/16,384 is two to three. A twenty-kilobyte shot is a three-byte length,
    /// so the boundary at 16,384 is one that ordinary use crosses.
    #[test]
    fn the_shot_log_prefix_matches_postcard() {
        for total in [1usize, 127, 128, 1024, 16_383, 16_384, 20 * 1024] {
            let whole = postcard::to_allocvec(&UplinkMessage::ShotLog(vec![0u8; total]))
                .expect("encode");
            let (prefix, len) = shot_log_prefix(total as u32);

            assert_eq!(&whole[..len], &prefix[..len], "prefix disagrees at {total} bytes");
            // And the prefix is the *whole* header: what follows it is the payload itself,
            // which is what lets the firmware stream the shot straight after it.
            assert_eq!(whole.len(), len + total, "at {total} bytes");
        }
    }

    /// An uplink query widens into the client query the firmware already serves.
    ///
    /// The firmware answers both transports through one implementation, so this conversion is
    /// the join between them. What it has to preserve is the *whole* question: an index that
    /// changed on the way through would fetch or overwrite the wrong routine slot, which is a
    /// silent wrong answer rather than a failure.
    #[test]
    fn an_uplink_query_widens_without_losing_anything() {
        let read: crate::ws_types::ClientQuery =
            UplinkQuery::RoutineDefinition(RoutineIndex::Function(7)).into();
        assert!(matches!(
            read,
            crate::ws_types::ClientQuery::RoutineDefinition(RoutineIndex::Function(7))
        ));

        // A create, where `None` is what makes the machine choose a slot -- and the one case
        // where confusing `None` with `Some(..)` would write over a routine somebody has.
        let create: crate::ws_types::ClientQuery = UplinkQuery::WriteRoutine {
            index: None,
            routine: vec![1, 2, 3],
        }
        .into();
        match create {
            crate::ws_types::ClientQuery::WriteRoutine { index, routine } => {
                assert_eq!(index, None);
                assert_eq!(routine, vec![1, 2, 3]);
            }
            _ => panic!("a write must stay a write"),
        }

        let update: crate::ws_types::ClientQuery = UplinkQuery::WriteRoutine {
            index: Some(RoutineIndex::Custom(3)),
            routine: vec![4],
        }
        .into();
        match update {
            crate::ws_types::ClientQuery::WriteRoutine { index, routine } => {
                assert_eq!(index, Some(RoutineIndex::Custom(3)));
                assert_eq!(routine, vec![4]);
            }
            _ => panic!("a write must stay a write"),
        }

        // A delete must not widen into a write. They are adjacent variants carrying the same
        // payload shape, which is exactly the pair a hand-written match arm gets wrong -- and
        // getting it wrong here would replace a routine with whatever bytes followed rather
        // than removing it.
        let delete: crate::ws_types::ClientQuery =
            UplinkQuery::DeleteRoutine(RoutineIndex::Custom(3)).into();
        assert!(matches!(
            delete,
            crate::ws_types::ClientQuery::DeleteRoutine(RoutineIndex::Custom(3))
        ));
    }

    /// A maximal routine write fits what a machine will accept inbound.
    ///
    /// This is the assertion the inbound bound rests on. `MAX_UPLINK_CLIENT_RECORD_LEN` is
    /// `ROUTINE_MAX_ENCODED_LEN` plus envelope slack, and a routine write is the largest
    /// thing Plantlet can legitimately say — so if this ever stops holding, saving a routine
    /// from the cloud fails against a real machine with nothing else to say why.
    #[test]
    fn a_maximal_routine_write_fits_the_client_bound() {
        let routine = vec![0u8; variegated_controller_types::ROUTINE_MAX_ENCODED_LEN];
        let encoded = postcard::to_allocvec(&UplinkMessage::Query {
            id: u32::MAX,
            query: UplinkQuery::WriteRoutine {
                index: Some(RoutineIndex::Custom(u32::MAX)),
                routine,
            },
        })
        .expect("encodes");

        assert!(
            encoded.len() <= MAX_UPLINK_CLIENT_RECORD_LEN,
            "a maximal routine write is {} bytes, over the {} byte client bound",
            encoded.len(),
            MAX_UPLINK_CLIENT_RECORD_LEN
        );
    }

    /// `QueryOutcome` is reused from the LAN socket rather than redeclared, so a failure the
    /// machine can already express reaches Plantlet without being flattened into something
    /// coarser. `RoutineWrite` is the one that motivated it: HTTP projected its five cases
    /// onto three status codes and the frontend read only the number.
    #[test]
    fn a_routine_write_failure_round_trips_whole() {
        let message = UplinkMessage::Reply {
            id: 7,
            outcome: QueryOutcome::Failed(QueryError::RoutineWrite(
                RoutineWriteError::Immutable,
            )),
        };
        let bytes = postcard::to_allocvec(&message).expect("encodes");
        let decoded: UplinkMessage = postcard::from_bytes(&bytes).expect("decodes");

        match decoded {
            UplinkMessage::Reply { id, outcome } => {
                assert_eq!(id, 7);
                assert!(matches!(
                    outcome,
                    QueryOutcome::Failed(QueryError::RoutineWrite(RoutineWriteError::Immutable))
                ));
            }
            _ => panic!("decoded as the wrong variant"),
        }
    }
}
