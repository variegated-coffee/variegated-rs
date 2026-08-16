//! Asking the storage task for something, and hearing back.
//!
//! The storage layer lives on core 1 behind a single task, because the SD card shares the
//! display's SPI bus and the bus lease is what keeps an SD command indivisible (see
//! [`crate::sd_card`]). Everything that wants to read a shot -- the comms processor's
//! HTTP handlers, a debug command from a host -- therefore *asks* rather than calls, and
//! these are the two halves of that exchange.
//!
//! # Why this module is not behind `sd-card-storage`
//!
//! Everything it serves is. But the controllers are not: `dual_boiler_single_group` and
//! `single_boiler_single_group` compile in every configuration, and both name
//! [`ShotLogQuery`] in a field and a constructor parameter so that
//! `MachineCommand::SetShotAnnotations` has one interpreter rather than two. Gating this
//! module would mean gating those signatures, which would put a `cfg` in the middle of
//! the controllers for a fact the *example* already knows -- the same reasoning that
//! keeps `Status::sd_card_present` an `Option` passed in from outside.
//!
//! Nothing here depends on the storage implementation. Every type it names comes from
//! `variegated-controller-types`, so an ungated module costs nothing on a build with no
//! card reader beyond two enum definitions nobody constructs.

use variegated_controller_types::{
    ShotAnnotations, ShotLogId, ShotLogList, ShotLogListRequest, ShotLogStorageError,
    SHOT_LOG_CHUNK_LEN,
};

/// A request for the storage task.
///
/// Deliberately **not** a mirror of [`crate::shot_log_storage::ShotLogStorage`]. There is
/// no `ReadAnnotations`, because a listing already carries annotations and nothing can
/// send such a request. Variants get added when something can send them, not in
/// anticipation of it.
#[derive(Debug, Clone, PartialEq)]
// Guarded, since this crate gained a `defmt` feature: a host test build turns it off, and
// a `Format` impl monomorphized there has no `_defmt_acquire` to link against.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ShotLogQuery {
    /// One page of the listing, newest first.
    List(ShotLogListRequest),
    /// Up to [`SHOT_LOG_CHUNK_LEN`] bytes of a stored record, starting at `offset`.
    Chunk { id: ShotLogId, offset: u32 },
    /// Replace a stored shot's annotations, rewriting the whole record.
    SetAnnotations {
        id: ShotLogId,
        annotations: ShotAnnotations,
    },
    /// Remove a stored shot.
    ///
    /// **Answered with no [`ShotLogReply`].** This channel has no correlation id, and
    /// every reply put on it is signalled into the comms processor's single reply slot,
    /// where a concurrent HTTP request can collect it as its own answer. A delete's
    /// confirmation therefore travels on the one-way event channel instead, as a
    /// `ShotLogEvent::Deleted` -- see the storage task in the gs3 firmware.
    Delete { id: ShotLogId },
}

/// The storage task's answer to a [`ShotLogQuery`].
///
/// `Chunk` and `Annotations` echo the `id` they were asked about -- and `Chunk` its
/// `offset` too -- because this protocol has **no correlation id**. A request that times
/// out on the far side leaves its answer in the channel for the *next* requester to
/// collect, so a receiver has to be able to recognise an answer to a question it is no
/// longer asking. `List` carries nothing to check against, which is why the storage task
/// drains a stale reply before producing a new one.
///
/// Errors travel as a reply rather than as a `Result`, because the requester is on the
/// other side of a channel and has no way to observe one.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ShotLogReply {
    List(ShotLogList),
    Chunk {
        id: ShotLogId,
        offset: u32,
        /// Size of the whole record, so a receiver can show progress from the first
        /// chunk rather than after the last.
        total: u32,
        /// Whether this chunk reached the end. Carried alongside `total` rather than
        /// derived from it, so the loop terminates without arithmetic on a value it
        /// would otherwise have to trust.
        last: bool,
        bytes: heapless::Vec<u8, SHOT_LOG_CHUNK_LEN>,
    },
    /// The annotations as stored, after a successful `SetAnnotations`.
    ///
    /// The stored block rather than a bare acknowledgement, so a client sees what the
    /// machine actually holds -- including any entry `ShotAnnotations::set` refused for
    /// want of room, which an ack would hide.
    Annotations {
        id: ShotLogId,
        annotations: ShotAnnotations,
    },
    Error(ShotLogStorageError),
}
