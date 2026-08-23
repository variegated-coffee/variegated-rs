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
use variegated_controller_types::{RoutineIndex, Status};

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
    /// Sent on connect, every ten minutes, and in answer to [`Self::RequestStatus`]. Plantlet
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
            | Self::Reply { .. } => Direction::Uplink,
            Self::RequestStatus | Self::RequestRoutineList | Self::Query { .. } => {
                Direction::Downlink
            }
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
        let cases: [(UplinkMessage, u8); 7] = [
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
        ];

        for (message, expected) in cases {
            let bytes = postcard::to_allocvec(&message).expect("encodes");
            assert_eq!(
                bytes[0], expected,
                "a discriminant moved -- this is a contract with flashed firmware"
            );
        }
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
        let uplink: [UplinkMessage; 4] = [
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
        ];
        let downlink: [UplinkMessage; 3] = [
            UplinkMessage::RequestStatus,
            UplinkMessage::RequestRoutineList,
            UplinkMessage::Query {
                id: 0,
                query: UplinkQuery::WriteRoutine {
                    index: None,
                    routine: vec![],
                },
            },
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
