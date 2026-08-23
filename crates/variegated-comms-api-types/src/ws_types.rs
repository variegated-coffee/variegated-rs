//! WebSocket message types for bidirectional communication

use serde::{Serialize, Deserialize};
use variegated_controller_types::{
    Status, Configuration, MachineDefinition, MachineCommand
};
use crate::api_types::RoutineSummaryStorage;

/// The largest postcard payload this server will put on the wire.
///
/// Outbound had no bound at all: `encode_ws_message` was `postcard::to_allocvec` and grew
/// until the allocator refused, on a chip whose heap has been measured 424 bytes from its
/// ceiling during a TLS shot upload -- where MbedTLS answers an allocation failure by
/// returning an error rather than aborting, so the upload fails and nothing says why.
///
/// **The floor under 8192 is what fixes it, not the ceiling.** `MachineDefinition` (~3,660
/// bytes), `Configuration` (~3,464) and `Status` (~2,408) all legitimately cross this socket,
/// so a bound anywhere near them breaks the machine rather than protecting it. 8192 is a
/// tripwire on runaway growth, not a budget: ordinary growth -- another boiler, another
/// peripheral slot -- must never approach it.
pub const MAX_WS_FRAME_LEN: usize = 8192;

/// The largest payload a *client* may send.
///
/// Deliberately much tighter than [`MAX_WS_FRAME_LEN`], because the two directions are not
/// symmetric and sizing them together would be sizing the cheap one for the expensive one.
/// The largest legal client message is a maximal `SetShotUploadSettings` at ~330 bytes;
/// nothing a browser can say comes near this. Granting inbound the outbound ceiling would
/// let any peer on the network make the firmware allocate 8 kB against that same 424-byte
/// margin -- which is the failure this bound exists to prevent, not to enable.
///
/// The number is [`variegated_controller_types::ROUTINE_MAX_ENCODED_LEN`] plus envelope
/// slack, matching `ROUTINE_BODY_LIMIT` in the firmware's `http.rs`. That is not a
/// coincidence worth losing: it is the ceiling the far side will actually store, so the two
/// ways of writing a routine agree on what is too big, and a routine write could return to
/// this socket without needing a second sizing argument.
///
/// Checked against the *declared* length before anything is allocated, so an absurd length
/// costs nothing to refuse.
pub const MAX_CLIENT_FRAME_LEN: usize =
    variegated_controller_types::ROUTINE_MAX_ENCODED_LEN + 256;

const _: () = assert!(MAX_CLIENT_FRAME_LEN <= MAX_WS_FRAME_LEN);

/// WebSocket message envelope for all communication
#[derive(Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
pub enum WsMessage<'a> {
    // Server → Client (push)
    /// Real-time status update
    StatusUpdate(Status),
    /// Configuration update
    ConfigurationUpdate(Configuration),
    /// Machine definition (sent on request)
    MachineDefinition(MachineDefinition),
    /// Routine summaries (sent on request, and pushed whenever the set changes)
    ///
    /// **Repurposed in place**: the payload used to be every routine's full definition.
    /// The variant keeps its position because postcard encodes an enum as its
    /// declaration-order discriminant, and this transport carries no version byte -- so a
    /// renumbering mis-decodes against firmware already in the field rather than failing.
    /// `variegated-cli` used to hold hand-copied mirrors of this type, with nothing to catch
    /// that; it re-exports these definitions now, and `tests` below pins the numbers.
    ///
    /// Definitions are fetched one at a time, through [`ClientQuery::RoutineDefinition`].
    /// They used to come over HTTP, because the client half of this enum had to fit a fixed
    /// 256-byte inbound buffer and a routine definition never could; [`MAX_CLIENT_FRAME_LEN`]
    /// clears one now. The split itself stays for the original reason, which was never about
    /// the frame: the comms processor has a 56 kB heap shared with Wi-Fi, BLE and the ESPHome
    /// server, and holding every routine's full definition to serve an editor that opens one
    /// occasionally is what this variant exists to avoid.
    RoutinesUpdate(RoutineSummaryStorage),
    /// Acknowledgment of a command
    CommandAck {
        /// Command ID for correlation
        id: u32,
        /// Whether the command succeeded
        success: bool,
        /// Error message if failed
        #[serde(borrow)]
        error: Option<&'a str>,
    },

    // Client → Server
    /// Request machine definition
    RequestMachineDefinition,
    /// Request current routines
    RequestRoutines,
    /// Send a machine command
    SendMachineCommand(MachineCommand),
    /// Request the current configuration.
    ///
    /// Appended rather than grouped with the other two requests above, because postcard
    /// encodes an enum as its declaration-order discriminant: inserting it next to its
    /// siblings would renumber `SendMachineCommand` and silently mis-decode every command
    /// a client sends. See the note on `RoutinesUpdate`, and `tests` below, which pins these
    /// numbers as the contract with firmware already in the field.
    ///
    /// Unlike `RequestRoutines`, this is **not** answered from the comms processor's
    /// cache. It is forwarded to the application processor, whose reply arrives on the
    /// ordinary configuration publish path and so reaches every connected client rather
    /// than only the one that asked. A configuration is the one piece of state a client
    /// gets no other way -- there is no HTTP fetch for it in the frontend, and the
    /// pubsub only carries what is published *after* a client subscribes, so a page
    /// loaded between two publishes had nothing to show until the next one.
    RequestConfiguration,

    /// A shot was stored or deleted on the card.
    ///
    /// Appended rather than grouped with the other server-to-client variants at the top,
    /// because postcard encodes an enum as its declaration-order discriminant: inserting
    /// it there would renumber `SendMachineCommand` and silently mis-decode every command
    /// a client sends. See the note on `RoutinesUpdate`, and `tests` below, which pins these
    /// numbers as the contract with firmware already in the field.
    ///
    /// It does not grow this enum: `MachineDefinition` at 3,660 bytes still sets its
    /// size.
    ShotLogEvent(variegated_controller_types::shot_log::ShotLogEvent),

    /// A machine command that wants an answer.
    ///
    /// Appended, like every variant since `CommandAck` -- postcard encodes an enum as its
    /// declaration-order discriminant, so this must stay last. See the note on
    /// [`Self::RoutinesUpdate`].
    ///
    /// # Why this exists alongside [`Self::SendMachineCommand`]
    ///
    /// That one is fire-and-forget, and silently so: a full command channel drops the
    /// command and the client is never told. Every settings write the SPA makes went out
    /// that way, which is half the reason routine CRUD moved to HTTP -- not the frame size
    /// alone, but that HTTP could answer "it worked" and this could not.
    ///
    /// The plain variant is kept rather than replaced. `variegated-cli` and the TUI send
    /// it, and a command nobody is waiting on has no use for a correlation id.
    ///
    /// # What the ack means -- read this before relying on it
    ///
    /// [`Self::CommandAck`] here confirms that the **comms processor accepted the command
    /// and queued it for the application processor**. It does *not* confirm the machine
    /// applied it, and it cannot: the inter-processor link has no correlation ids at all,
    /// so there is nothing for a reply to be matched against. Adding them is a protocol
    /// change across both firmwares and is not what this variant is.
    ///
    /// What it does buy is the distinction that actually fails in practice -- "queued"
    /// versus "dropped, because the channel was full" -- delivered immediately rather than
    /// as silence.
    ///
    /// **For confirmation that a setting took, watch the state broadcast instead.** The
    /// application processor republishes `Configuration` after a settings change, and it
    /// reaches every client as [`Self::ConfigurationUpdate`]. Ack means accepted; the
    /// update means applied. The shot-upload settings UI is built on exactly that pair.
    SendMachineCommandWithId {
        /// Correlation id, echoed in the [`Self::CommandAck`] this provokes.
        ///
        /// Chosen by the client and opaque here. Zero is not reserved, but note that the
        /// two "not available yet" acks answer with `id: 0` because they reply to requests
        /// that carry no id -- a client that also uses 0 cannot tell them apart.
        id: u32,
        command: MachineCommand,
    },

    /// Ask the machine for something and get an answer back.
    ///
    /// Appended, and must stay so -- see the note on [`Self::RoutinesUpdate`].
    ///
    /// # Why a generic query rather than a variant per operation
    ///
    /// One correlation mechanism, one pending-request map on the client, one handler arm on
    /// the firmware. A new question extends [`ClientQuery`] and [`QueryOk`] instead of
    /// growing this enum, which matters because *this* enum's discriminants are a contract
    /// with firmware already in the field while those two are reached only through it.
    ///
    /// # How this differs from [`Self::SendMachineCommandWithId`]
    ///
    /// That one answers "was the command queued". This one answers the question itself, and
    /// it can: [`QueryError`] carries `RoutineWriteOutcome`'s five failure modes verbatim,
    /// where HTTP projected them onto three status codes and the frontend read only the
    /// number. A command is a thing you do; a query is a thing you ask.
    Query {
        /// Correlation id, echoed in the [`Self::QueryReply`] this provokes.
        id: u32,
        query: ClientQuery,
    },

    /// The answer to a [`Self::Query`].
    ///
    /// Appended, and must stay so.
    QueryReply {
        /// The `id` of the [`Self::Query`] this answers.
        id: u32,
        outcome: QueryOutcome,
    },
}

/// Whether a [`ClientQuery`] was answered, and what with.
///
/// A domain enum rather than `Result<QueryOk, QueryError>`, for two reasons. `Result` has no
/// `PostcardSchema` impl in this tree, and giving it one would put a *generic* named type
/// into the emitted TypeScript whose name would have to be disambiguated per instantiation.
/// More importantly it matches the shape the machine already speaks:
/// `RoutineWriteOutcome::{Stored, Failed}` is exactly this split, and a wire protocol with
/// two spellings for one idea is one spelling too many. **Append only.**
#[derive(Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
pub enum QueryOutcome {
    Ok(QueryOk),
    Failed(QueryError),
}

/// A postcard-encoded payload carried inside another postcard message.
///
/// **Deliberately opaque, and the opacity is the point.** The comms processor does not
/// decode a `Routine` today -- definitions move through `routine_request` as bytes -- and it
/// should not start. Passing the client's bytes through verbatim to the application
/// processor's validator is what keeps `RoutineWriteError::Malformed` meaning "the client
/// sent something that is not a routine" rather than "the middle hop re-encoded it
/// differently". It also keeps a ~650-byte `Routine` off a stack that has 68,200 bytes in
/// total.
///
/// A `Vec<u8>` rather than a borrowed slice or a `bytes`-typed field: serde maps `Vec<u8>`
/// to a *sequence* of `u8`, which postcard encodes as a varint length followed by the raw
/// bytes -- byte-identical to `serialize_bytes`. The generated TypeScript therefore sees
/// `seq(u8())` and works in `number[]`, so the frontend converts at the two call sites with
/// `Array.from` and `new Uint8Array`. Nothing in this tree maps to `Node::Bytes` yet and
/// adding a type that does would mean hand-written `Serialize`/`Deserialize` for no change
/// on the wire.
pub type EncodedPayload = alloc::vec::Vec<u8>;

/// What a client can ask for.
///
/// Reached only through [`WsMessage::Query`], so its discriminants are a contract with
/// deployed firmware exactly as the envelope's are. **Append only.**
#[derive(Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
pub enum ClientQuery {
    /// Fetch one routine's definition.
    ///
    /// The machine reassembles it from chunks and answers with the whole thing in one
    /// frame. That is not just simpler than streaming -- it closes a hazard the HTTP route
    /// had, where `ROUTINE_LOCK` was taken per *chunk* and a save landing between two of
    /// them spliced an old head onto a new tail. postcard is positional, so the result
    /// decoded into a routine nobody wrote.
    RoutineDefinition(variegated_controller_types::RoutineIndex),

    /// Store a routine, and say whether it was stored.
    ///
    /// `index: None` creates a custom routine and the machine assigns the index, which comes
    /// back as [`QueryOk::RoutineStored`]. `Some(..)` updates in place, or creates at that
    /// index for a function routine.
    ///
    /// **The whole point of this being a query rather than a command.**
    /// `MachineCommand::AddRoutine` exists and is fire-and-forget: before the machine could
    /// answer, a routine too large to persist looked exactly like one that saved. The
    /// five-way [`QueryError::RoutineWrite`] is what this buys.
    WriteRoutine {
        index: Option<variegated_controller_types::RoutineIndex>,
        /// A postcard-encoded `Routine`. See [`EncodedPayload`].
        routine: EncodedPayload,
    },

    /// One page of the stored-shot listing.
    ///
    /// Paged rather than whole, and the budget is not this transport's: a page is bounded at
    /// `SHOT_LOG_LIST_BUDGET` (3,800 bytes) by the *inter-processor* link's
    /// `CobsAccumulator::<4096>`, which is a much tighter ceiling than
    /// [`MAX_WS_FRAME_LEN`]. A client resumes with the last entry's id as `before`.
    ///
    /// Listing only. A shot's *contents* are tens of kilobytes and stay on HTTP, where they
    /// stream through a 1 kB buffer and arrive with a filename.
    ShotLogPage(variegated_controller_types::shot_log::ShotLogListRequest),

    /// Remove a routine, and say whether it went.
    ///
    /// A query rather than `MachineCommand::RemoveRoutine`, for the reason
    /// [`Self::WriteRoutine`] is a query rather than `AddRoutine`: a command is
    /// fire-and-forget, so a delete that failed on a worn flash sector was indistinguishable
    /// from one that succeeded, and the client removed the row either way.
    ///
    /// Answered with [`QueryOk::RoutineDeleted`], or a [`QueryError::RoutineDelete`] saying
    /// which of the three ways it did not happen.
    DeleteRoutine(variegated_controller_types::RoutineIndex),
}

/// A successful answer to a [`ClientQuery`]. **Append only.**
#[derive(Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
pub enum QueryOk {
    /// The routine, still postcard-encoded. See [`EncodedPayload`].
    RoutineDefinition(EncodedPayload),
    /// Stored, at this index. Echoed even for an update, so a create and an update have one
    /// success shape -- and a create has no other way to learn where it landed.
    RoutineStored(variegated_controller_types::RoutineIndex),
    /// One page of the listing, newest first, with `truncated` saying whether more follow.
    ShotLogPage(variegated_controller_types::shot_log::ShotLogList),
    /// Removed, from this index.
    ///
    /// The index is echoed for the same reason [`Self::RoutineStored`] echoes one: a client
    /// that named a slot gets confirmation of the slot it actually affected, rather than a
    /// bare acknowledgement it has to trust.
    RoutineDeleted(variegated_controller_types::RoutineIndex),
}

/// Why a [`ClientQuery`] could not be answered. **Append only.**
#[derive(Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
pub enum QueryError {
    /// Nothing at that index. A real answer and a fast one -- the machine says so rather
    /// than letting the client sit out the timeout.
    NotFound,
    /// The machine did not answer in time, or answered the wrong question. Both mean "ask
    /// again"; neither says anything about what is stored.
    Unavailable,
    /// Why a [`ClientQuery::WriteRoutine`] did not store anything.
    ///
    /// Carried whole rather than flattened. The HTTP route this replaced projected these
    /// five onto three status codes, and the frontend read only the number -- so "internal
    /// routines are read-only" and "this routine is too large to persist" both surfaced as
    /// `HTTP error! status: 400`. They are distinguishable again.
    RoutineWrite(variegated_controller_types::RoutineWriteError),
    /// Why the card could not be read.
    ShotLogStorage(variegated_controller_types::shot_log::ShotLogStorageError),
    /// Why a [`ClientQuery::DeleteRoutine`] removed nothing.
    ///
    /// Its own error rather than a reuse of [`Self::RoutineWrite`], because the two disagree
    /// about the case that matters: a write to an empty index creates it, so
    /// `RoutineWriteError` deliberately has no way to say "there was nothing there", which is
    /// a delete's most ordinary failure.
    RoutineDelete(variegated_controller_types::RoutineDeleteError),
}

#[cfg(test)]
mod tests {
    use super::*;
    // `no_std` crate: `vec!` is not in the prelude here, and the discriminant assertions
    // below compare against literal encodings.
    use alloc::vec;
    use variegated_controller_types::shot_log::{ShotLogEvent, ShotLogId};
    use variegated_controller_types::{Configuration, Status};

    /// Pins the discriminant of every variant.
    ///
    /// **This test survived the deletion of the mirrors, and it is worth being clear about
    /// what it guards now.** It no longer catches a hand-copy drifting from the firmware --
    /// there is no copy left to drift. It catches the thing that outlived that problem:
    /// this transport carries no version byte, so *reordering the shared enum* is
    /// undetectable at run time. A machine already in the field decodes by position. Move a
    /// variant and every deployed frontend and CLI mis-decodes against it, silently, and
    /// the only thing standing between that and a release is this list.
    ///
    /// So the numbers are a contract with **already-flashed firmware**, not with a sibling
    /// declaration. Appending is safe; anything else is not.
    ///
    /// Encoded rather than hand-asserted so the check exercises the same serde path
    /// the transport uses. The unit-like variants encode to exactly one byte, which
    /// makes them the cheap ones to pin; the payload-carrying ones are pinned by
    /// their first byte.
    #[test]
    fn variant_discriminants_match_the_firmware() {
        let cases: [(WsMessage, u8); 8] = [
            (WsMessage::RequestMachineDefinition, 5),
            (WsMessage::RequestRoutines, 6),
            (WsMessage::SendMachineCommand(MachineCommand::CancelRoutine), 7),
            (WsMessage::RequestConfiguration, 8),
            // 9 and 10 are the two this crate does not send. They are pinned anyway, and 9
            // is exactly why: `ShotLogEvent` was appended on the firmware side and the old
            // hand-copy here never grew it, so this list stopped at 8 and the gap went
            // unnoticed until 10 was added on top of it.
            (
                WsMessage::ShotLogEvent(ShotLogEvent::Deleted(ShotLogId {
                    day: None,
                    time: 0,
                })),
                9,
            ),
            (
                WsMessage::SendMachineCommandWithId {
                    id: 1,
                    command: MachineCommand::CancelRoutine,
                },
                10,
            ),
            (
                WsMessage::Query {
                    id: 1,
                    query: ClientQuery::RoutineDefinition(
                        variegated_controller_types::RoutineIndex::Custom(0),
                    ),
                },
                11,
            ),
            (
                WsMessage::QueryReply {
                    id: 1,
                    outcome: QueryOutcome::Failed(QueryError::NotFound),
                },
                12,
            ),
        ];

        for (message, expected) in cases {
            let bytes = postcard::to_allocvec(&message).expect("encodes");
            assert_eq!(
                bytes[0], expected,
                "variant discriminant moved -- the firmware's ws_types.rs is the authority"
            );
        }
    }

    /// Pins the discriminants of the three query enums.
    ///
    /// The envelope test above pins [`WsMessage`], which is one layer out. These three are a
    /// contract in exactly the same way and had no test at all: they are reached *through*
    /// `Query` and `QueryReply`, so a reorder here is invisible to that test and just as
    /// undetectable at run time -- a deployed frontend or CLI decodes by position and reads a
    /// different question, or a different reason for a refusal.
    ///
    /// `RoutineDelete` is the one worth naming. It is an error about a routine that could not
    /// be removed and it sits next to `RoutineWrite`, which is an error about one that could
    /// not be stored; nothing but position tells them apart on the wire.
    #[test]
    fn query_variant_discriminants_are_pinned() {
        let index = variegated_controller_types::RoutineIndex::Custom(0);

        let queries: [(ClientQuery, u8); 4] = [
            (ClientQuery::RoutineDefinition(index), 0),
            (
                ClientQuery::WriteRoutine {
                    index: None,
                    routine: vec![],
                },
                1,
            ),
            (
                ClientQuery::ShotLogPage(
                    variegated_controller_types::shot_log::ShotLogListRequest::newest(),
                ),
                2,
            ),
            (ClientQuery::DeleteRoutine(index), 3),
        ];
        for (query, expected) in queries {
            let bytes = postcard::to_allocvec(&query).expect("encodes");
            assert_eq!(bytes[0], expected, "a ClientQuery discriminant moved");
        }

        let oks: [(QueryOk, u8); 4] = [
            (QueryOk::RoutineDefinition(vec![]), 0),
            (QueryOk::RoutineStored(index), 1),
            (
                QueryOk::ShotLogPage(variegated_controller_types::shot_log::ShotLogList {
                    entries: Default::default(),
                    truncated: false,
                }),
                2,
            ),
            (QueryOk::RoutineDeleted(index), 3),
        ];
        for (ok, expected) in oks {
            let bytes = postcard::to_allocvec(&ok).expect("encodes");
            assert_eq!(bytes[0], expected, "a QueryOk discriminant moved");
        }

        let errors: [(QueryError, u8); 5] = [
            (QueryError::NotFound, 0),
            (QueryError::Unavailable, 1),
            (
                QueryError::RoutineWrite(variegated_controller_types::RoutineWriteError::Immutable),
                2,
            ),
            (
                QueryError::ShotLogStorage(
                    variegated_controller_types::shot_log::ShotLogStorageError::CardNotPresent,
                ),
                3,
            ),
            (
                QueryError::RoutineDelete(
                    variegated_controller_types::RoutineDeleteError::NotFound,
                ),
                4,
            ),
        ];
        for (error, expected) in errors {
            let bytes = postcard::to_allocvec(&error).expect("encodes");
            assert_eq!(bytes[0], expected, "a QueryError discriminant moved");
        }
    }

    /// The correlated command must decode back to the same id and command.
    ///
    /// `SendMachineCommand` and `SendMachineCommandWithId` differ only by a `u32` in front
    /// of the payload, which is precisely the kind of pair that survives being confused:
    /// decode one as the other and the id is read out of the command's first bytes. Pinning
    /// the round trip is cheaper than reasoning about that at a call site.
    #[test]
    fn a_correlated_command_round_trips_with_its_id() {
        let message = WsMessage::SendMachineCommandWithId {
            id: 0xDEAD_BEEF,
            command: MachineCommand::CancelRoutine,
        };
        let bytes = postcard::to_allocvec(&message).expect("encodes");

        let decoded: WsMessage = postcard::from_bytes(&bytes).expect("decodes");
        match decoded {
            WsMessage::SendMachineCommandWithId { id, command } => {
                assert_eq!(id, 0xDEAD_BEEF);
                assert!(matches!(command, MachineCommand::CancelRoutine));
            }
            _ => panic!("decoded as the wrong variant"),
        }
    }

    /// The two unit-like request variants are what `variegated-tui` sends on connect,
    /// as single bytes. Documented here because a reader of that call site would
    /// otherwise have to derive the encoding to know why it writes one byte.
    #[test]
    fn request_variants_encode_to_a_single_byte() {
        assert_eq!(
            postcard::to_allocvec(&WsMessage::RequestMachineDefinition).unwrap(),
            vec![5]
        );
        assert_eq!(
            postcard::to_allocvec(&WsMessage::RequestRoutines).unwrap(),
            vec![6]
        );
    }

    /// `StatusUpdate` is discriminant 0, so a status frame is a leading zero byte
    /// followed by `Status`'s own encoding. Round-tripped rather than asserted
    /// byte-wise: `Status` is large and its layout is not this module's business.
    #[test]
    fn status_update_round_trips() {
        let message = WsMessage::StatusUpdate(Status::new());
        let bytes = postcard::to_allocvec(&message).expect("encodes");
        assert_eq!(bytes[0], 0, "StatusUpdate must stay the first variant");

        let decoded: WsMessage = postcard::from_bytes(&bytes).expect("decodes");
        assert!(matches!(decoded, WsMessage::StatusUpdate(_)));
    }

    /// `ConfigurationUpdate` is discriminant 1. `pid-log` depends on this one as
    /// much as on `StatusUpdate`, since it is how the gain columns stay current
    /// after the HTTP seed.
    #[test]
    fn configuration_update_round_trips() {
        let message = WsMessage::ConfigurationUpdate(Configuration::new());
        let bytes = postcard::to_allocvec(&message).expect("encodes");
        assert_eq!(bytes[0], 1, "ConfigurationUpdate must stay the second variant");

        let decoded: WsMessage = postcard::from_bytes(&bytes).expect("decodes");
        assert!(matches!(decoded, WsMessage::ConfigurationUpdate(_)));
    }

    /// A maximal `SetShotUploadSettings` fits what a client is allowed to send.
    ///
    /// This is the assertion the shot-upload migration rests on. That setting lived on HTTP
    /// because a maximal payload is ~330 bytes and inbound frames were capped at 256; it is
    /// back on the socket now, and if it ever grows past `MAX_CLIENT_FRAME_LEN` the save
    /// stops working against a real machine with nothing else to say so.
    ///
    /// Maximal means every field at its documented limit: a 255-character endpoint, a
    /// 64-character token, and two 53-character Noise keys.
    #[test]
    fn a_maximal_shot_upload_settings_command_fits_a_client_frame() {
        use variegated_controller_types::shot_upload::{
            ShotUploadKeyUpdate, ShotUploadSettings, ShotUploadTokenUpdate,
            SHOT_UPLOAD_ENDPOINT_LEN, SHOT_UPLOAD_KEY_LEN, SHOT_UPLOAD_TOKEN_LEN,
        };

        /// Fill to exactly capacity, so the test tracks the constants rather than a literal.
        fn filled<const N: usize>(c: char) -> heapless::String<N> {
            let mut s = heapless::String::<N>::new();
            while s.push(c).is_ok() {}
            s
        }

        let settings = ShotUploadSettings {
            endpoint: Some(filled::<SHOT_UPLOAD_ENDPOINT_LEN>('e')),
            enabled: true,
            token: ShotUploadTokenUpdate::Set(filled::<SHOT_UPLOAD_TOKEN_LEN>('t')),
            server_key: Some(filled::<SHOT_UPLOAD_KEY_LEN>('k')),
            device_key: ShotUploadKeyUpdate::Set(filled::<SHOT_UPLOAD_KEY_LEN>('d')),
        };

        let encoded = postcard::to_allocvec(&WsMessage::SendMachineCommandWithId {
            id: u32::MAX,
            command: MachineCommand::SetShotUploadSettings(settings),
        })
        .expect("encodes");

        assert!(
            encoded.len() <= MAX_CLIENT_FRAME_LEN,
            "a maximal shot-upload settings command is {} bytes, over the {} byte client limit",
            encoded.len(),
            MAX_CLIENT_FRAME_LEN
        );
    }

    /// The big server-to-client messages fit the outbound bound.
    ///
    /// **These are floors, not the real worst case.** `Status::new()` and
    /// `Configuration::new()` are empty; the figure that actually sized
    /// `MAX_WS_FRAME_LEN` is a measured ~3,660-byte `MachineDefinition`, which cannot be
    /// built here without a machine to describe. What this catches is the cheap disaster --
    /// someone lowering the bound below what the protocol routinely sends.
    #[test]
    fn the_large_server_messages_fit_the_frame_limit() {
        for encoded in [
            postcard::to_allocvec(&WsMessage::StatusUpdate(Status::new())).expect("encodes"),
            postcard::to_allocvec(&WsMessage::ConfigurationUpdate(Configuration::new()))
                .expect("encodes"),
        ] {
            assert!(encoded.len() <= MAX_WS_FRAME_LEN, "{}", encoded.len());
        }
    }
}
