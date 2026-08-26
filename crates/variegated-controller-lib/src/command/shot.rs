//! The shot-log commands: pending annotations, and the two that reach the card.
//!
//! The two card commands were already written twice and already carried a comment saying they
//! were identical for a reason -- the query sender is `None` on every single-boiler build
//! today, so both always refuse there, but a single-boiler machine that gained a card reader
//! should not need annotation and deletion re-implemented from scratch. Sharing them is what
//! that comment was asking for.

use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::channel::Sender;
use variegated_controller_types::shot_log::{ShotAnnotations, ShotLogId};
use variegated_log::{log_debug, log_warn};

use crate::shot_log_query::ShotLogQuery;

/// `SetPendingShotAnnotations` -- what will be stamped onto the next shot.
///
/// RAM-only and replaced in full, not merged: the browser sends the whole block, and a merge
/// would make clearing a field impossible.
pub fn set_pending_annotations(pending: &mut ShotAnnotations, annotations: ShotAnnotations) {
    *pending = annotations;
    log_debug!("Pending shot annotations set ({} entries)", pending.len());
}

/// Hand a query to the task that owns the card.
///
/// `try_send` rather than `send`: the caller is the control loop, and a depth-1 channel that
/// is already full means a shot-log request is in flight. Parking here to wait for it would
/// stall the PID.
fn dispatch<M: RawMutex>(
    sender: Option<&Sender<'_, M, ShotLogQuery, 1>>,
    query: ShotLogQuery,
    command: &'static str,
    id: ShotLogId,
) {
    match sender {
        Some(sender) => {
            if sender.try_send(query).is_err() {
                log_warn!(
                    "{}({:?}) refused: a shot-log request is already in flight",
                    command, id
                );
            }
        }
        None => log_warn!("{}({:?}) ignored: this machine has no shot-log storage", command, id),
    }
}

/// `SetShotAnnotations` -- replace the annotations on a shot already on the card.
pub fn set_shot_annotations<M: RawMutex>(
    sender: Option<&Sender<'_, M, ShotLogQuery, 1>>,
    id: ShotLogId,
    annotations: ShotAnnotations,
) {
    dispatch(
        sender,
        ShotLogQuery::SetAnnotations { id, annotations },
        "SetShotAnnotations",
        id,
    );
}

/// `DeleteShotLog`.
///
/// **Fire and forget, and irreversible.** Nothing acknowledges it: success is reported by a
/// `ShotLogEvent::Deleted` push, and a failure is logged while the shot stays where it was.
pub fn delete_shot_log<M: RawMutex>(
    sender: Option<&Sender<'_, M, ShotLogQuery, 1>>,
    id: ShotLogId,
) {
    dispatch(sender, ShotLogQuery::Delete { id }, "DeleteShotLog", id);
}

#[cfg(test)]
mod tests {
    use super::*;
    use embassy_sync::blocking_mutex::raw::NoopRawMutex;
    use embassy_sync::channel::Channel;

    fn an_id() -> ShotLogId {
        ShotLogId { day: Some(20_260_826), time: 1 }
    }

    /// A machine with no card storage refuses rather than panicking or blocking.
    ///
    /// This is every single-boiler build today, which is why the arm exists at all.
    #[test]
    fn a_machine_with_no_card_refuses() {
        delete_shot_log::<NoopRawMutex>(None, an_id());
        set_shot_annotations::<NoopRawMutex>(None, an_id(), ShotAnnotations::default());
    }

    /// A request reaches the card task, and a second one while it is in flight is refused
    /// rather than parking the control loop.
    #[test]
    fn a_second_request_in_flight_is_refused_not_awaited() {
        let channel: Channel<NoopRawMutex, ShotLogQuery, 1> = Channel::new();
        let sender = channel.sender();

        delete_shot_log(Some(&sender), an_id());
        assert!(channel.try_receive().is_ok(), "the first request never arrived");

        // Fill the depth-1 channel, then try again: the second must be dropped, not awaited.
        delete_shot_log(Some(&sender), an_id());
        delete_shot_log(Some(&sender), an_id());
        assert!(channel.try_receive().is_ok());
        assert!(channel.try_receive().is_err(), "the refused request was queued after all");
    }

    /// Pending annotations are replaced in full.
    #[test]
    fn pending_annotations_are_replaced_not_merged() {
        let mut pending = ShotAnnotations::default();

        set_pending_annotations(&mut pending, ShotAnnotations::default());

        assert_eq!(pending.len(), 0);
    }
}
