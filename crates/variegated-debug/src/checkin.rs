//! Publishing the check-in table.
//!
//! The reader half of `variegated-checkin`: it takes a firmware's `Monitor` and the tables
//! `define_checkins!` generated beside it, and puts them on the bus. It never writes a slot.
//!
//! Ungated, unlike [`crate::snapshot`]: both processors have tasks worth watching, and the
//! payloads are source-agnostic -- a `CheckinReport` says nothing about which board it came
//! from that `DebugFrame::source` does not already say.

use core::iter;

use heapless::Vec;
use variegated_checkin::Monitor;
use variegated_controller_types::debug::{name, CheckinEntry, DebugPayload, MAX_CHECKINS};

/// How often the whole table is published.
///
/// Matched to [`crate::snapshot`]'s cadence deliberately: a host lining a check-in age up
/// against a heap figure or a stack high-water mark should not have to reconcile two
/// different clocks.
///
/// This is the knob if the relay budget gets tight. A full 24-slot report is ~128 B/s at
/// this rate, the largest steady-state item after the schema burst, and
/// `relay::steady_state_relay_traffic_fits_the_budget` prints the arithmetic. Doubling this
/// halves that and costs a host one second of resolution on a table whose fastest slot is
/// already sampled far more often than it changes.
pub const REPORT_INTERVAL_MS: u64 = 1_000;

/// How often the slot names and periods are re-emitted.
///
/// Same reasoning as the sampler's: emission is always-on, there is no handshake, and a
/// client may attach at any moment, so periodic re-emission is the only way a late one
/// learns what the rows mean. Same value, so the two schema bursts coincide rather than
/// interleaving into a steadier trickle that is harder to recognise on a trace.
pub const SCHEMA_INTERVAL_MS: u64 = crate::sampler::SCHEMA_INTERVAL_MS as u64;

/// Gap between consecutive schema frames.
///
/// **Not optional, and the sampler's tight loop is not a precedent for dropping it.** The
/// bus is a [`crate::bus::BUS_CAPACITY`]-slot ring published to with `publish_immediate`,
/// which overwrites the oldest rather than back-pressuring. The sampler's schema burst is
/// one `FirmwareInfo` plus one `MetricName` per metric -- nine frames on the application
/// processor -- which fits the ring, so it can afford to emit them without yielding. A
/// check-in schema is one frame *per slot*, and a firmware near [`MAX_CHECKINS`] emits
/// half again the ring's capacity. Burst that and it evicts its own first frames before any
/// subscriber runs: the low-numbered slots would never be named, and their rows would
/// render as `slot[0]`, `slot[1]` and so on, forever, while the high-numbered ones came
/// through fine. The failure looks like a host bug and is not one.
///
/// 20 ms, matching the comms firmware's `SCHEMA_FRAME_SPACING`, which was sized against the
/// same ring for the same reason. At [`MAX_CHECKINS`] the whole burst costs under half a
/// second once every [`SCHEMA_INTERVAL_MS`], and it is a burst of names -- nothing is
/// waiting on it.
pub const SCHEMA_FRAME_SPACING_MS: u64 = 20;

/// Turns a firmware's monitor and name tables into payloads.
///
/// Separate from [`run`] so the payload construction is testable without a bus, a time
/// driver or the counter-delta lock -- the split `Sampler` uses, for the same reason.
pub struct CheckinReporter<const N: usize> {
    monitor: &'static Monitor<N>,
    names: &'static [&'static str],
    periods: &'static [Option<u32>],
}

impl<const N: usize> CheckinReporter<N> {
    /// Panics if the tables do not match `N`, or if `N` exceeds what one frame can carry.
    ///
    /// Both are wiring mistakes -- a slot enum and the `Monitor<N>` beside it having drifted
    /// apart -- and both produce silently mislabelled rows rather than an error, which is
    /// exactly what a host cannot detect. `Sampler::new` makes the same call about its name
    /// tables.
    pub fn new(
        monitor: &'static Monitor<N>,
        names: &'static [&'static str],
        periods: &'static [Option<u32>],
    ) -> Self {
        assert!(N <= MAX_CHECKINS, "too many check-in slots for one frame");
        assert!(names.len() == N, "check-in name table does not match N");
        assert!(periods.len() == N, "check-in period table does not match N");
        Self { monitor, names, periods }
    }

    /// The whole table as of `now_ms`.
    ///
    /// Positional: every slot reports, in id order, including ones nothing has written. A
    /// skipped slot would shift every row after it, and the host has no way to notice.
    pub fn report_payload(&self, now_ms: u32) -> DebugPayload {
        let mut entries: Vec<CheckinEntry, MAX_CHECKINS> = Vec::new();
        for entry in self.monitor.entries(now_ms) {
            // Cannot fail: N <= MAX_CHECKINS is asserted in `new`.
            let _ = entries.push(entry);
        }
        DebugPayload::CheckinReport(entries)
    }

    /// One `CheckinSlotInfo` per slot, in id order.
    pub fn schema_payloads(&self) -> impl Iterator<Item = DebugPayload> + '_ {
        iter::zip(self.names, self.periods)
            .enumerate()
            .map(|(id, (label, period_ms))| DebugPayload::CheckinSlotInfo {
                id: id as u8,
                label: name(label),
                period_ms: *period_ms,
            })
    }
}

/// Publish the table forever, and its schema periodically.
///
/// A plain `async fn` rather than an `#[embassy_executor::task]`, because a task cannot be
/// generic and `CheckinReporter` is generic over its slot count. Each firmware keeps a
/// two-line task that builds one and awaits this -- the same split `sampler::run` uses.
pub async fn run<const N: usize>(reporter: CheckinReporter<N>) -> ! {
    // Starts *at* the interval so the first pass emits the schema immediately, rather than
    // leaving a client attached at boot with a table of unlabelled rows for five seconds.
    let mut since_schema_ms = SCHEMA_INTERVAL_MS;

    loop {
        if since_schema_ms >= SCHEMA_INTERVAL_MS {
            // Spaced, not burst. See `SCHEMA_FRAME_SPACING_MS` -- a burst here evicts its
            // own low-numbered slots and leaves them permanently unnamed.
            let mut burst_ms = 0u64;
            for payload in reporter.schema_payloads() {
                crate::bus::publish(payload);
                embassy_time::Timer::after_millis(SCHEMA_FRAME_SPACING_MS).await;
                burst_ms += SCHEMA_FRAME_SPACING_MS;
            }
            // Counted against the interval rather than ignored: the burst is a real part of
            // the period, and at `MAX_CHECKINS` it is nearly half a second of it.
            since_schema_ms = burst_ms;
        }

        // Truncated to 32 bits to match what a slot stores. Both sides truncate the same
        // uptime, so the `wrapping_sub` in `CheckinSlot::read` closes over the same wrap.
        let now_ms = embassy_time::Instant::now().as_millis() as u32;
        crate::bus::publish(reporter.report_payload(now_ms));

        embassy_time::Timer::after_millis(REPORT_INTERVAL_MS).await;
        since_schema_ms = since_schema_ms.saturating_add(REPORT_INTERVAL_MS);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec as StdVec;
    use variegated_checkin::{CheckinDetail, CheckinStatus};

    static MONITOR: Monitor<3> = Monitor::new();
    const NAMES: &[&str] = &["Controller", "Storage", "Schedule"];
    const PERIODS: &[Option<u32>] = &[Some(100), None, Some(60_000)];

    fn reporter() -> CheckinReporter<3> {
        CheckinReporter::new(&MONITOR, NAMES, PERIODS)
    }

    /// Slots nothing ever wrote still occupy their row. The report is positional, so a
    /// firmware that skipped them would shift every later row's identity by one and the
    /// host would have no way to tell.
    #[test]
    fn every_slot_reports_including_the_ones_nothing_wrote() {
        MONITOR
            .claim(0u8)
            .record_at(1_000, CheckinStatus::Warning(CheckinDetail::Degraded));

        match reporter().report_payload(1_250) {
            DebugPayload::CheckinReport(entries) => {
                assert_eq!(entries.len(), 3);
                assert_eq!(entries[0].status, CheckinStatus::Warning(CheckinDetail::Degraded));
                assert_eq!(entries[0].age_ms, 250);
                assert_eq!(entries[1].status, CheckinStatus::NotStarted);
                assert_eq!(entries[2].status, CheckinStatus::NotStarted);
            }
            other => panic!("expected CheckinReport, got {other:?}"),
        }
    }

    /// Names, periods and ids must line up, because they are three tables the host joins on
    /// an index nothing else checks.
    #[test]
    fn the_schema_carries_every_slots_name_and_period_in_id_order() {
        let r = reporter();
        let payloads: StdVec<_> = r.schema_payloads().collect();

        assert_eq!(payloads.len(), 3);
        assert!(matches!(
            &payloads[0],
            DebugPayload::CheckinSlotInfo { id: 0, label, period_ms: Some(100) }
                if label == "Controller"
        ));
        assert!(
            matches!(
                &payloads[1],
                DebugPayload::CheckinSlotInfo { id: 1, label, period_ms: None }
                    if label == "Storage"
            ),
            "an event-driven slot must ship no period, so the host never ages it"
        );
        assert!(matches!(
            &payloads[2],
            DebugPayload::CheckinSlotInfo { id: 2, period_ms: Some(60_000), .. }
        ));
    }

    #[test]
    #[should_panic(expected = "check-in name table does not match N")]
    fn a_name_table_that_does_not_match_fails_at_startup() {
        let _ = CheckinReporter::new(&MONITOR, &["Controller"], PERIODS);
    }
}
