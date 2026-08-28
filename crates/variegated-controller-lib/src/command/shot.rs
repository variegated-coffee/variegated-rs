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
        // Both are stamped by the settle read, seconds after this log is closed. See
        // `ShotLogger::set_settled_output`.
        final_weight_grams: None,
        final_volume_ml: None,
    });
    *manual_shot_active = true;
    log_debug!("Started a manual shot log");
}

/// Close a manual shot log, leaving it unsent.
///
/// **Guarded on `manual_shot_active` rather than on "is a log open".** `handle_routine_exit`
/// stops brewing *before* it finishes its own log, so an unguarded version would close the
/// routine's log early, from the wrong place, and the routine path would then find nothing to
/// send.
///
/// The annotations are cleared here for the same reason the routine path clears them: one
/// carried into the next shot is indistinguishable from one the user entered for it.
///
/// **It does not send.** Closing and sending used to be one step, and version 10 of the shot
/// log split them: the yield is read [`crate::shot_log::SETTLE_MILLIS`] after the pump stops,
/// and a log already on its way to the card cannot be given it. The caller arms a settle
/// deadline and sends from `complete_pending_settle`. Returns whether a log was actually
/// closed, so a caller does not arm a deadline for a shot that was never open.
pub fn finish_manual_shot_log(
    logger: &mut crate::ShotLogger,
    pending: &mut ShotAnnotations,
    manual_shot_active: &mut bool,
) -> bool {
    use variegated_controller_types::ShotStatus;

    if !*manual_shot_active {
        return false;
    }
    *manual_shot_active = false;

    logger.finish_shot(ShotStatus::Completed);
    pending.clear();
    true
}

/// Close a routine's shot log and clear what it carried, leaving it unsent.
///
/// The tail of `handle_routine_exit` on both machines, byte for byte. Everything before it --
/// what to stop, what to restore, what state to resume -- genuinely differs and stays in the
/// controllers, where the ordering hazards are documented.
///
/// **The annotations are cleared in full, including beans and grind.** Carrying any of them
/// forward would label the next shot with this one's coffee whether or not the user changed it,
/// and an annotation nobody entered is indistinguishable from one they did.
///
/// Like the manual path, this no longer sends -- see [`finish_manual_shot_log`].
pub fn finish_routine_shot_log(logger: &mut crate::ShotLogger, pending: &mut ShotAnnotations) {
    use variegated_controller_types::ShotStatus;

    logger.finish_shot(ShotStatus::Completed);
    pending.clear();
}

/// Take the settle read and hand the finished shot to storage.
///
/// The second half of finishing a shot, run from the controller's own loop once
/// [`crate::shot_log::SETTLE_MILLIS`] has elapsed since the pump stopped. Shared because both
/// controllers do exactly this and the ordering inside it is the part worth having in one
/// place: **stamp, then send.** `send_latest_shot_log` clones the back of the history, so a
/// send that ran first would put an unsettled copy on the card and leave the settled one
/// nowhere.
///
/// `weight` and `volume` are whatever the machine could read at this moment. `None` is a real
/// answer -- a machine with no scale and no volume measurement -- and is recorded as one.
pub fn complete_settle_and_send<M: RawMutex>(
    logger: &mut crate::ShotLogger,
    weight: Option<variegated_controller_types::WeightType>,
    volume: Option<variegated_controller_types::OutputVolumeType>,
    sender: Option<&Sender<'_, M, variegated_controller_types::ShotLog, 2>>,
) {
    logger.set_settled_output(weight, volume);
    send_latest_shot_log(logger, sender);
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

    // ---- The settle read ------------------------------------------------------------

    /// A logger holding one finished, unsent manual shot.
    fn a_finished_shot() -> (crate::ShotLogger, ShotAnnotations, bool) {
        let mut logger = crate::ShotLogger::new();
        let mut pending = ShotAnnotations::default();
        let mut active = false;

        start_manual_shot_log(&mut logger, &pending, &mut active, false, 0);
        let closed = finish_manual_shot_log(&mut logger, &mut pending, &mut active);
        assert!(closed, "the shot was open, so finishing it must report that it closed");

        (logger, pending, active)
    }

    /// Closing a shot does not send it -- the yield is not known yet.
    ///
    /// This is the property the whole settle design rests on. Before version 10 these were one
    /// step, and a log that left here immediately could never be given a settled weight,
    /// because `send_latest_shot_log` clones rather than moving.
    #[test]
    fn closing_a_shot_does_not_send_it() {
        let channel: Channel<NoopRawMutex, variegated_controller_types::ShotLog, 2> =
            Channel::new();
        let (mut logger, _pending, _active) = a_finished_shot();

        assert!(
            channel.try_receive().is_err(),
            "the log was sent before its yield could be read"
        );

        // And it is still there to be sent once it has been settled.
        assert!(logger.latest_log().is_some());
        complete_settle_and_send(&mut logger, Some(36.5), Some(37.0), Some(&channel.sender()));
        assert!(channel.try_receive().is_ok(), "the settled log never arrived");
    }

    /// The yield is stamped onto the log the storage task receives, not onto a copy.
    ///
    /// `complete_settle_and_send` stamps and then sends, and the ordering is the point: a send
    /// that ran first would put an unsettled log on the card and leave the settled one in a
    /// history nobody reads.
    #[test]
    fn the_settled_yield_reaches_the_stored_log() {
        let channel: Channel<NoopRawMutex, variegated_controller_types::ShotLog, 2> =
            Channel::new();
        let (mut logger, _pending, _active) = a_finished_shot();

        complete_settle_and_send(&mut logger, Some(36.5), Some(37.25), Some(&channel.sender()));

        let stored = channel.try_receive().expect("a settled log was sent");
        // Both halves, and distinct values: they are adjacent `Option<f32>`s, so equal ones
        // would not catch a stamp that crossed them.
        assert_eq!(stored.metadata.final_weight_grams, Some(36.5));
        assert_eq!(stored.metadata.final_volume_ml, Some(37.25));
    }

    /// A machine that can measure neither still sends its shot.
    ///
    /// The case a Silvia with no scale is in on every shot. `None` is what it measured, and it
    /// is recorded as that rather than suppressing the log or standing in a zero -- a shot with
    /// no yield figure is still a shot, and still worth rating.
    #[test]
    fn a_machine_that_measures_neither_still_sends_the_shot() {
        let channel: Channel<NoopRawMutex, variegated_controller_types::ShotLog, 2> =
            Channel::new();
        let (mut logger, _pending, _active) = a_finished_shot();

        complete_settle_and_send(&mut logger, None, None, Some(&channel.sender()));

        let stored = channel.try_receive().expect("a shot with no yield is still sent");
        assert_eq!(stored.metadata.final_weight_grams, None);
        assert_eq!(stored.metadata.final_volume_ml, None);
    }

    /// Finishing a shot that was never open reports so, rather than arming a settle for it.
    ///
    /// `stop_brewing` runs whenever brewing stops, including at the end of a *routine* shot,
    /// whose log the routine path closes itself. The `false` here is what stops the controller
    /// arming a second settle deadline for a shot that has already been dealt with.
    #[test]
    fn finishing_a_shot_that_was_not_open_reports_nothing_closed() {
        let mut logger = crate::ShotLogger::new();
        let mut pending = ShotAnnotations::default();
        let mut active = false;

        assert!(!finish_manual_shot_log(&mut logger, &mut pending, &mut active));
        assert!(logger.latest_log().is_none());
    }

    /// The settle stamps the shot that just finished, not one that started after it.
    ///
    /// `set_settled_output` writes to the back of the history, so this pins the hazard the
    /// controllers' `complete_pending_settle(true)` flush exists to avoid: once a second shot
    /// has been opened and closed, the back is *that* shot, and a settle still pending from the
    /// first would land on the wrong log.
    #[test]
    fn the_settle_stamps_the_most_recently_finished_shot() {
        let mut logger = crate::ShotLogger::new();
        let mut pending = ShotAnnotations::default();
        let mut active = false;

        start_manual_shot_log(&mut logger, &pending, &mut active, false, 0);
        finish_manual_shot_log(&mut logger, &mut pending, &mut active);
        logger.set_settled_output(Some(18.0), Some(18.0));

        start_manual_shot_log(&mut logger, &pending, &mut active, false, 0);
        finish_manual_shot_log(&mut logger, &mut pending, &mut active);
        logger.set_settled_output(Some(36.0), Some(36.0));

        // The second shot has the second yield, and the first kept its own.
        assert_eq!(logger.latest_log().unwrap().metadata.final_weight_grams, Some(36.0));
        assert_eq!(logger.get_log(0).unwrap().metadata.final_weight_grams, Some(18.0));
    }
}
