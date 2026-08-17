//! Packing a [`CheckinStatus`] into the 32-bit word a slot stores it in.
//!
//! Lives here rather than in `variegated-controller-types` because it is a storage detail
//! of this crate's atomic table, and `-types` deliberately does nothing: it holds the shape
//! that goes on the wire, and conversions belong outside it.
//!
//! The layout is `kind << 8 | detail`. It is not a wire format -- postcard never sees it --
//! so it may change freely; nothing outside this crate observes it.

use variegated_controller_types::debug::{CheckinDetail, CheckinStatus};

const KIND_NOT_STARTED: u32 = 0;
const KIND_GOOD: u32 = 1;
const KIND_WARNING: u32 = 2;
const KIND_ERROR: u32 = 3;

/// Encode a status into one word.
pub(crate) const fn pack(status: CheckinStatus) -> u32 {
    match status {
        CheckinStatus::NotStarted => KIND_NOT_STARTED << 8,
        CheckinStatus::Good => KIND_GOOD << 8,
        CheckinStatus::Warning(detail) => (KIND_WARNING << 8) | detail_to_u8(detail) as u32,
        CheckinStatus::Error(detail) => (KIND_ERROR << 8) | detail_to_u8(detail) as u32,
    }
}

/// Decode a word written by [`pack`].
///
/// Falls back to [`CheckinStatus::NotStarted`] on a word this crate did not write, which
/// only a torn or corrupted store can produce. Reporting "never reached" for a slot whose
/// storage is nonsense is the honest answer; inventing a `Good` would not be.
pub(crate) fn unpack(word: u32) -> CheckinStatus {
    let detail = detail_from_u8(word as u8);
    match (word >> 8, detail) {
        (KIND_GOOD, _) => CheckinStatus::Good,
        (KIND_WARNING, Some(detail)) => CheckinStatus::Warning(detail),
        (KIND_ERROR, Some(detail)) => CheckinStatus::Error(detail),
        _ => CheckinStatus::NotStarted,
    }
}

/// Exhaustive by construction: adding a [`CheckinDetail`] variant fails to compile here
/// until it is given a number, which is the point of writing this out rather than deriving
/// a discriminant. The numbers are local to this crate and never reach the wire, so they
/// may be reassigned -- but there is no reason to.
const fn detail_to_u8(detail: CheckinDetail) -> u8 {
    match detail {
        CheckinDetail::PeripheralUnresponsive => 0,
        CheckinDetail::PeerUnresponsive => 1,
        CheckinDetail::ResourceUnavailable => 2,
        CheckinDetail::QueueFull => 3,
        CheckinDetail::PreconditionUnmet => 4,
        CheckinDetail::Degraded => 5,
        CheckinDetail::Overrun => 6,
        CheckinDetail::TaskExited => 7,
    }
}

fn detail_from_u8(value: u8) -> Option<CheckinDetail> {
    Some(match value {
        0 => CheckinDetail::PeripheralUnresponsive,
        1 => CheckinDetail::PeerUnresponsive,
        2 => CheckinDetail::ResourceUnavailable,
        3 => CheckinDetail::QueueFull,
        4 => CheckinDetail::PreconditionUnmet,
        5 => CheckinDetail::Degraded,
        6 => CheckinDetail::Overrun,
        7 => CheckinDetail::TaskExited,
        _ => return None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every detail, under both kinds that carry one, plus the two that do not.
    ///
    /// One test rather than three because the property is a single round trip and the
    /// interesting failure -- a `detail_to_u8` number colliding with another variant's --
    /// only shows up when the whole set is covered at once.
    #[test]
    fn every_status_round_trips() {
        const DETAILS: [CheckinDetail; 8] = [
            CheckinDetail::PeripheralUnresponsive,
            CheckinDetail::PeerUnresponsive,
            CheckinDetail::ResourceUnavailable,
            CheckinDetail::QueueFull,
            CheckinDetail::PreconditionUnmet,
            CheckinDetail::Degraded,
            CheckinDetail::Overrun,
            CheckinDetail::TaskExited,
        ];

        for status in [CheckinStatus::NotStarted, CheckinStatus::Good] {
            assert_eq!(unpack(pack(status)), status);
        }

        for detail in DETAILS {
            for status in [CheckinStatus::Warning(detail), CheckinStatus::Error(detail)] {
                assert_eq!(unpack(pack(status)), status, "{status:?} did not survive a round trip");
            }
        }
    }

    /// A word this crate never wrote decodes as `NotStarted` rather than panicking or
    /// fabricating a status. Only a corrupted store can produce one, and a slot reporting
    /// "never reached" is the honest reading of storage that makes no sense.
    #[test]
    fn an_unknown_word_decodes_as_not_started() {
        assert_eq!(unpack(0xDEAD_BEEF), CheckinStatus::NotStarted);
        // A valid kind carrying a detail number no variant has.
        assert_eq!(unpack((3 << 8) | 200), CheckinStatus::NotStarted);
    }

    /// `Good` and `NotStarted` ignore the low byte, so a stale detail left in it by a
    /// previous store must not resurface as a `Warning`.
    #[test]
    fn recovering_to_good_drops_the_previous_detail() {
        let warned = pack(CheckinStatus::Warning(CheckinDetail::Degraded));
        assert_eq!(unpack(warned), CheckinStatus::Warning(CheckinDetail::Degraded));
        assert_eq!(unpack(pack(CheckinStatus::Good)), CheckinStatus::Good);
    }
}
