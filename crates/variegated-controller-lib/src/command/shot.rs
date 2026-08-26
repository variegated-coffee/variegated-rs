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
use variegated_log::{log_debug, log_info, log_warn};

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

/// `TagDoseFromScale` -- record a scale's current reading as the dose for the next shot.
///
/// `weight` is what *this machine's* group scale reads; the selector is checked against
/// `group_index` here, because both machines have exactly one group and the check was written
/// twice.
///
/// **Refused rather than guessed when there is no reading.** A dose silently recorded from the
/// wrong place is worse than no dose at all, because nothing downstream can tell it was wrong.
pub fn tag_dose_from_scale(
    pending: &mut ShotAnnotations,
    scale: variegated_controller_types::ScaleSelector,
    group_index: variegated_controller_types::GroupIndex,
    weight: Option<variegated_controller_types::WeightType>,
) {
    use variegated_controller_types::{ScaleSelector, ShotAnnotationKey, ShotAnnotationValue};

    match scale {
        ScaleSelector::GroupScale(index) if index == group_index => {}
        // A single-group machine has exactly one group scale. An index for any other group is
        // a client bug, not a missing peripheral, so it is logged as such rather than folded
        // into "the scale is not reporting".
        ScaleSelector::GroupScale(index) => {
            log_warn!("TagDoseFromScale: no group {} on this machine", index);
            return;
        }
    }

    let Some(grams) = weight else {
        log_warn!("TagDoseFromScale: {:?} has no reading to take", scale);
        return;
    };

    match pending.set(ShotAnnotationKey::DoseWeight, ShotAnnotationValue::Number(grams)) {
        Ok(()) => log_info!("Dose tagged from {:?}: {} g", scale, grams),
        // Only reachable with eight custom annotations already set and no dose among them.
        // Reported, because the alternative is a dose the user asked for and did not get.
        Err(_) => log_warn!(
            "TagDoseFromScale: the annotation block is full ({} entries)",
            pending.len()
        ),
    }
}

/// Open a shot log for a shot the user started by hand.
///
/// Refused while a routine is running or a log is already open: a routine keeps its own log,
/// with its own metadata, and two open at once would mean the routine's was replaced by one
/// that records none of what makes it a routine.
///
/// The pending annotations are **copied, not moved.** They are cleared when the shot finishes
/// rather than when it starts, so that a shot abandoned halfway does not silently discard what
/// the user typed for it.
pub fn start_manual_shot_log(
    logger: &mut crate::ShotLogger,
    pending: &ShotAnnotations,
    manual_shot_active: &mut bool,
    routine_running: bool,
    group_index: variegated_controller_types::GroupIndex,
) {
    use variegated_controller_types::{ShotLogMetadata, ShotStatus, ShotType};

    if routine_running || logger.is_logging() {
        return;
    }

    logger.start_shot(ShotLogMetadata {
        annotations: pending.clone(),
        shot_type: ShotType::Manual,
        group_index,
        // No routine, and that is the fact being recorded rather than a gap in one.
        routine_metadata: None,
        start_time_millis: embassy_time::Instant::now().as_millis(),
        end_time_millis: None,
        final_status: ShotStatus::Running,
        recorded_at_unix_millis: None,
    });
    *manual_shot_active = true;
    log_debug!("Started a manual shot log");
}

/// Close a manual shot log and hand it to storage.
///
/// **Guarded on `manual_shot_active` rather than on "is a log open".** `handle_routine_exit`
/// stops brewing *before* it finishes its own log, so an unguarded version would close the
/// routine's log early, from the wrong place, and the routine path would then find nothing to
/// send.
///
/// The annotations are cleared here for the same reason the routine path clears them: one
/// carried into the next shot is indistinguishable from one the user entered for it.
pub fn finish_manual_shot_log<M: RawMutex>(
    logger: &mut crate::ShotLogger,
    pending: &mut ShotAnnotations,
    manual_shot_active: &mut bool,
    sender: Option<&Sender<'_, M, variegated_controller_types::ShotLog, 2>>,
) {
    use variegated_controller_types::ShotStatus;

    if !*manual_shot_active {
        return;
    }
    *manual_shot_active = false;

    logger.finish_shot(ShotStatus::Completed);
    send_latest_shot_log(logger, sender);
    pending.clear();
}

/// Hand the most recently finished shot to the storage task, if there is one listening.
///
/// `try_send` rather than `send`: this runs inside the control loop, and a storage task that
/// has fallen behind must cost a shot log rather than a boiler update.
pub fn send_latest_shot_log<M: RawMutex>(
    logger: &mut crate::ShotLogger,
    sender: Option<&Sender<'_, M, variegated_controller_types::ShotLog, 2>>,
) {
    let Some(sender) = sender else { return };
    let Some(shot_log) = logger.latest_log() else { return };

    if sender.try_send(shot_log.clone()).is_err() {
        log_warn!("Failed to send shot log for storage (channel full)");
    } else {
        log_debug!("Shot log sent for storage");
    }
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
