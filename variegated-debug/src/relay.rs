//! What may cross the inter-processor link, and what may not.
//!
//! The relay itself lives in `variegated_comms::debug_relay`, which cannot be
//! host-tested: it links `embassy-rp`, which does not build for a host target. This
//! module holds the part of the relay that is a *decision* rather than I/O, so the
//! decision has tests.

use variegated_controller_types::debug::DebugPayload;

/// Whether a payload crosses the inter-processor link.
///
/// Checked **before** encoding, not after. Encoding a `Status` is the ~1.7 kB
/// allocation this exists to avoid, so a filter that ran on the encoded length would
/// pay the cost it is meant to prevent.
///
/// Only `DebugPayload::Status` is refused, for two independent reasons:
///
/// 1. **The rate limiter cannot pass it.** [`crate::rate::WINDOW_BUDGET`] is 300
///    bytes per 100 ms window and [`crate::rate::TokenBucket::allow`] admits an item
///    only if it fits inside a single window. A populated `Status` is 1722 bytes
///    COBS-encoded, and even the modest dual-boiler case runs to several hundred, so
///    it is refused every window, forever -- the relay would never deliver machine
///    state, only count drops. Raising the budget to fit one would hand debug traffic
///    a fifth of the whole 576 kbaud link.
/// 2. **It is redundant.** The comms processor already receives `Status` through
///    `ApplicationProcessorToCommsProcessorMessage::Status`, which is what feeds the
///    WebSocket and ESPHome paths. Relaying it again under a debug wrapper would
///    double the link cost of the machine's largest message for no new information.
///    Task 11's TCP server injects the comms processor's own copy into the debug
///    stream instead, so a TCP client still sees machine state -- it just does not
///    cross the link twice.
///
/// On the application processor this is now belt-and-braces: [`crate::status`] moved
/// `Status` off the shared bus entirely, so the relay's subscriber never sees one.
/// It stays because `bus::publish` is public and nothing else stops a caller putting
/// a `Status` on the bus, and because "why is machine state not relayed?" deserves an
/// answer at the place that would otherwise relay it.
pub fn relayable(payload: &DebugPayload) -> bool {
    !matches!(payload, DebugPayload::Status(_))
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::boxed::Box;
    use heapless::Vec;
    use variegated_controller_types::debug::{
        name, text, DebugEvent, DebugFrame, DebugPayload, DebugSource, MetricKind, Severity,
    };
    use variegated_controller_types::Status;

    use crate::rate::{TokenBucket, DEBUG_RELAY_BYTES_PER_SEC, WINDOW_BUDGET, WINDOW_MS};

    fn frame(payload: DebugPayload) -> DebugFrame {
        DebugFrame { source: DebugSource::Application, seq: 0, uptime_ms: 0, payload }
    }

    /// Everything the application processor puts on the bus, one of each.
    fn every_other_payload() -> std::vec::Vec<DebugPayload> {
        let mut samples: Vec<u64, 16> = Vec::new();
        for i in 0..4u64 {
            let _ = samples.push(i * 1_000_000);
        }
        std::vec![
            DebugPayload::CounterSamples(samples.clone()),
            DebugPayload::IndicatorSamples(samples),
            DebugPayload::Event(DebugEvent::BrewStarted { group: 0 }),
            DebugPayload::Text(Severity::Warn, text("water tank empty")),
            DebugPayload::MetricName {
                kind: MetricKind::Counter,
                id: 0,
                label: name("ControllerLoops"),
            },
            DebugPayload::FirmwareInfo { firmware: name("dual-boiler"), counters: 4, indicators: 4 },
        ]
    }

    #[test]
    fn status_never_crosses_the_link() {
        assert!(!relayable(&DebugPayload::Status(Box::new(Status::default()))));
    }

    #[test]
    fn every_other_payload_crosses_the_link() {
        for payload in every_other_payload() {
            assert!(relayable(&payload), "should have been relayed: {payload:?}");
        }
    }

    /// Drives the relay's two gates in the order `debug_relay::relay` applies them,
    /// over a frame stream that mixes `Status` in with everything else. Asserts the
    /// property that matters: `Status` is dropped by the *filter*, and its absence
    /// is what leaves budget for the rest.
    #[test]
    fn a_status_in_the_stream_does_not_starve_the_others() {
        let mut bucket = TokenBucket::new();
        let mut relayed = 0usize;
        let mut filtered = 0usize;
        let mut rate_limited = 0usize;

        let mut stream = std::vec::Vec::new();
        for payload in every_other_payload() {
            stream.push(frame(payload));
            stream.push(frame(DebugPayload::Status(Box::new(Status::default()))));
        }

        for f in &stream {
            if !relayable(&f.payload) {
                filtered += 1;
                continue;
            }
            let encoded = postcard::to_allocvec_cobs(&f).expect("frames encode");
            if bucket.allow(0, encoded.len() as u32) {
                relayed += 1;
            } else {
                rate_limited += 1;
            }
        }

        assert_eq!(filtered, 6, "one Status per other payload");
        assert_eq!(relayed, 6, "every non-Status frame fits one window");
        assert_eq!(rate_limited, 0);
    }

    /// The measurement behind [`DEBUG_RELAY_BYTES_PER_SEC`]: the application
    /// processor's steady-state debug output, wrapped exactly as the relay wraps it,
    /// against the budget.
    ///
    /// Steady state for `examples/dual-boiler` is, per second: two sample frames per
    /// `DEFAULT_SAMPLE_INTERVAL_MS` (500 ms, so four), one `StateSnapshot`, and one
    /// fifth of a schema burst (`SCHEMA_INTERVAL_MS` is 5 s, and the burst is
    /// `FirmwareInfo` plus one `MetricName` per metric). Text frames are excluded:
    /// the log bridge's own token bucket already bounds them, and in steady state on
    /// a healthy machine there are none.
    #[test]
    fn steady_state_relay_traffic_fits_the_budget() {
        use variegated_controller_types::ApplicationProcessorToCommsProcessorMessage as Msg;
        use variegated_controller_types::debug::{
            ApplicationState, DebugStateSnapshot, SourceState,
        };

        fn wrapped_len(payload: DebugPayload) -> usize {
            let msg = Msg::Debug(frame(payload));
            postcard::to_allocvec_cobs(&msg).expect("encodes").len()
        }

        let mut samples: Vec<u64, 16> = Vec::new();
        // Four counters and four indicators in dual-boiler, at values large enough
        // that postcard's varints are not artificially short.
        for _ in 0..4 {
            let _ = samples.push(u32::MAX as u64);
        }

        let counters = wrapped_len(DebugPayload::CounterSamples(samples.clone()));
        let indicators = wrapped_len(DebugPayload::IndicatorSamples(samples));
        let snapshot = wrapped_len(DebugPayload::StateSnapshot(DebugStateSnapshot {
            heap_used: u32::MAX,
            heap_free: u32::MAX,
            frames_emitted: u32::MAX,
            frames_dropped: u32::MAX,
            frames_suppressed: u32::MAX,
            frames_rate_limited: u32::MAX,
            source_state: SourceState::Application(ApplicationState {
                watchdog_fed_ms_ago: Some(u32::MAX),
                psram_heap: true,
                routine_running: Some(u16::MAX),
                link_frames_relayed: u32::MAX,
                link_frames_dropped: u32::MAX,
            }),
        }));

        let mut schema = wrapped_len(DebugPayload::FirmwareInfo {
            firmware: name("dual-boiler"),
            counters: 4,
            indicators: 4,
        });
        for kind in [MetricKind::Counter, MetricKind::Indicator] {
            for id in 0..4u8 {
                schema += wrapped_len(DebugPayload::MetricName {
                    kind,
                    id,
                    label: name("ControllerLoopDuration"),
                });
            }
        }

        // Per second: 2 sample frames every 500 ms, 1 snapshot, 1/5 of a schema burst.
        let per_second = 2 * (counters + indicators) + snapshot + schema / 5;
        std::println!(
            "relay steady state: counters={counters} indicators={indicators} \
             snapshot={snapshot} schema_burst={schema} => {per_second} B/s \
             (budget {DEBUG_RELAY_BYTES_PER_SEC} B/s)"
        );
        assert!(
            per_second < DEBUG_RELAY_BYTES_PER_SEC as usize,
            "steady-state relay traffic {per_second} B/s exceeds the {DEBUG_RELAY_BYTES_PER_SEC} B/s budget"
        );

        // The budget is spent per 100 ms window, not per second, so a frame also has
        // to fit one window on its own -- the trap that `Status` falls into.
        let text = wrapped_len(DebugPayload::Text(
            Severity::Info,
            text("0123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123"),
        ));
        assert!(
            text <= WINDOW_BUDGET as usize,
            "a maximum-length text frame is {text} B and cannot fit one window"
        );

        for (label, size) in [
            ("counters", counters),
            ("indicators", indicators),
            ("snapshot", snapshot),
            ("largest schema frame", schema / 9),
        ] {
            assert!(
                size <= WINDOW_BUDGET as usize,
                "{label} is {size} B, which cannot fit a {WINDOW_MS} ms window of {WINDOW_BUDGET} B"
            );
        }
    }

    /// Pins how much of the boot log survives the relay, because the answer is not
    /// "all of it" and the number should not be able to drift silently.
    ///
    /// [`TokenBucket`] is a hard fixed window with no burst capacity, by design. That
    /// is the right shape for steady state -- which measures at a few hundred bytes a
    /// second against a 3000 B/s budget -- but boot is not steady state: the log
    /// bridge's own bucket admits a burst of ~21 lines within a few hundred
    /// milliseconds (`suppress::BOOT_BURST_LINES`), and a full-width `Text` frame is
    /// large enough that only a couple fit each 100 ms window.
    ///
    /// The consequence is that a host watching over TCP sees a *thinned* boot log
    /// where a host watching over USB sees all of it -- the USB writer has no byte
    /// budget. It is a real loss of information rather than collapsed repetition, and
    /// each dropped line is counted in `link_frames_dropped`, so it is at least
    /// visible rather than silent.
    ///
    /// Recorded, not fixed: giving the relay's bucket burst capacity is a change to
    /// the shared `rate` module and its documented "not a leaky bucket" decision,
    /// which is outside this task. This test exists so whoever revisits that has the
    /// measurement in front of them.
    #[test]
    fn the_boot_burst_is_thinned_by_the_fixed_window() {
        use variegated_controller_types::ApplicationProcessorToCommsProcessorMessage as Msg;

        // `suppress::BOOT_BURST_LINES` worth of distinct init lines, spaced the 15 ms
        // apart that Task 16 measured, at a realistic width.
        const LINES: u32 = 21;
        const SPACING_MS: u64 = 15;

        let mut bucket = TokenBucket::new();
        let mut admitted = 0;
        for i in 0..LINES {
            let payload = DebugPayload::Text(
                Severity::Info,
                text(&std::format!("initialising subsystem number {i} of the boot sequence")),
            );
            let encoded =
                postcard::to_allocvec_cobs(&Msg::Debug(frame(payload))).expect("encodes");
            if bucket.allow(i as u64 * SPACING_MS, encoded.len() as u32) {
                admitted += 1;
            }
        }

        std::println!(
            "boot burst over the relay: {admitted}/{LINES} lines admitted \
             ({SPACING_MS} ms spacing, {WINDOW_BUDGET} B per {WINDOW_MS} ms window)"
        );
        assert!(admitted < LINES, "if this now passes everything, the bucket gained burst capacity -- update this test and the note above it");
        assert!(admitted >= 6, "a regression below this would mean almost no boot log crosses the link");
    }
}
