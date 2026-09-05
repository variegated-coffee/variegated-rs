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
/// Only `DebugPayload::Status` is refused, and the reason is redundancy:
///
/// **The comms processor already receives `Status`** through
/// `ApplicationProcessorToCommsProcessorMessage::Status`, which is what feeds the
/// WebSocket and ESPHome paths. Relaying it again under a debug wrapper would double
/// the link cost of the machine's largest message -- 1722 bytes COBS-encoded in the
/// worst case, several hundred in the ordinary one, once a second -- for no new
/// information. On `variegated-silvia-firmware`'s 115 200 baud link that alone would exceed the
/// entire debug budget. The comms processor's TCP server injects that processor's own copy
/// into the debug stream instead, so a TCP client still sees machine state; it just
/// does not cross the link twice.
///
/// **This function is now the only thing stopping it, and that changed.** Under the
/// fixed window this module used to sit behind, a `Status` frame could not fit any
/// 300-byte window and was refused by arithmetic no matter what happened here. The
/// window has since become a token bucket with [`crate::rate::BURST_BYTES`] of
/// capacity, deliberately sized above the codec's `MAX_FRAME` so that no legal frame
/// is permanently unsendable -- which means a `Status` would now pass the budget. The
/// redundancy argument was always the stronger of the two and it is unaffected, but
/// do not weaken this filter on the assumption that the rate limiter is still a
/// backstop behind it. It is not.
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
        name, text, CheckinDetail, CheckinEntry, CheckinStatus, DebugEvent, DebugFrame,
        DebugPayload, DebugSource, MetricKind, Severity, MAX_CHECKINS, MAX_SAMPLES,
    };
    use variegated_controller_types::Status;

    use crate::rate::{
        bytes_per_sec_for_baud, TokenBucket, BURST_BYTES, DEBUG_RELAY_BYTES_PER_SEC,
        REFERENCE_BAUD,
    };

    /// A deliberately pessimistic link: five times slower than the reference.
    ///
    /// **Not any board's current link.** Both firmwares configure 576 000 baud today
    /// (`variegated-silvia-firmware/src/main.rs` sets it explicitly, above a comment about
    /// the 115 200 it used to run). This is kept as a standing margin rather than retired:
    /// it is the rate the Silvia shipped at, it is what `BURST_BYTES` was sized against,
    /// and a budget that only holds at the fastest link the tree has ever used is a budget
    /// that fails the first time someone lowers one.
    const SLOW_BAUD: u32 = 115_200;

    fn frame(payload: DebugPayload) -> DebugFrame {
        DebugFrame { source: DebugSource::Application, seq: 0, uptime_ms: 0, payload }
    }

    /// Everything the application processor puts on the bus, one of each.
    fn every_other_payload() -> std::vec::Vec<DebugPayload> {
        // Capacity named rather than spelled: these are `DebugPayload`'s own sample
        // vectors, so a literal here would have to be kept in step with `MAX_SAMPLES`
        // by hand, and a bump to it would break this suite rather than these vectors.
        let mut samples: Vec<u64, MAX_SAMPLES> = Vec::new();
        for i in 0..4u64 {
            let _ = samples.push(i * 1_000_000);
        }
        // Deliberately *full*, unlike the sample vectors above. This is the widest frame
        // the relay will ever see in steady state -- a firmware reports every slot it has,
        // every second, whether or not anything is wrong -- so the byte-budget tests below
        // must be sized against a table at capacity, not a token one.
        let mut checkins: Vec<CheckinEntry, MAX_CHECKINS> = Vec::new();
        for i in 0..MAX_CHECKINS {
            let _ = checkins.push(CheckinEntry {
                status: CheckinStatus::Good,
                age_ms: 100 + i as u32,
            });
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
            DebugPayload::FirmwareInfo {
                firmware: name("variegated-gs3-firmware"),
                counters: 4,
                indicators: 6,
                checkins: MAX_CHECKINS as u8,
            },
            DebugPayload::CheckinReport(checkins),
            DebugPayload::CheckinSlotInfo {
                id: 0,
                label: name("Controller"),
                period_ms: Some(100),
            },
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
        let mut bucket = TokenBucket::for_baud(SLOW_BAUD);
        let mut relayed = 0usize;
        let mut filtered = 0usize;
        let mut rate_limited = 0usize;

        // Counted rather than spelled: a payload added to `every_other_payload` should
        // widen this test's coverage, not fail its arithmetic.
        let others = every_other_payload().len();
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

        assert_eq!(filtered, others, "one Status per other payload");
        assert_eq!(relayed, others, "every non-Status frame fits the bucket");
        assert_eq!(rate_limited, 0);
    }

    /// The measurement behind [`DEBUG_RELAY_BYTES_PER_SEC`]: the application
    /// processor's steady-state debug output, wrapped exactly as the relay wraps it,
    /// against the budget.
    ///
    /// Steady state for `variegated-gs3-firmware` is, per second: two sample frames per
    /// `DEFAULT_SAMPLE_INTERVAL_MS` (1000 ms, so two), one `StateSnapshot`, and one
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

        // Capacity named rather than spelled: these are `DebugPayload`'s own sample
        // vectors, so a literal here would have to be kept in step with `MAX_SAMPLES`
        // by hand, and a bump to it would break this suite rather than these vectors.
        //
        // Four counters and six indicators in the GS3 firmware -- the two counts differ
        // since the display's render and flush times were added as indicators only -- at
        // values large enough that postcard's varints are not artificially short.
        const GS3_COUNTERS: u8 = 4;
        const GS3_INDICATORS: u8 = 6;

        fn samples_of(n: u8) -> Vec<u64, MAX_SAMPLES> {
            let mut samples: Vec<u64, MAX_SAMPLES> = Vec::new();
            for _ in 0..n {
                let _ = samples.push(u32::MAX as u64);
            }
            samples
        }

        let counters = wrapped_len(DebugPayload::CounterSamples(samples_of(GS3_COUNTERS)));
        let indicators = wrapped_len(DebugPayload::IndicatorSamples(samples_of(GS3_INDICATORS)));
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
                core1_stack_high_water: Some(u32::MAX),
                core1_stack_size: Some(u32::MAX),
            }),
            stack_high_water: Some(u32::MAX),
            stack_size: Some(u32::MAX),
        }));

        let mut schema = wrapped_len(DebugPayload::FirmwareInfo {
            firmware: name("variegated-gs3-firmware"),
            counters: GS3_COUNTERS,
            indicators: GS3_INDICATORS,
            checkins: MAX_CHECKINS as u8,
        });
        let mut metric_frames = 1usize;
        for (kind, count) in [
            (MetricKind::Counter, GS3_COUNTERS),
            (MetricKind::Indicator, GS3_INDICATORS),
        ] {
            for id in 0..count {
                schema += wrapped_len(DebugPayload::MetricName {
                    kind,
                    id,
                    label: name("ControllerLoopDuration"),
                });
                metric_frames += 1;
            }
        }
        // The check-in schema rides the same burst, one frame per slot, and there are far
        // more slots than metrics -- so this is the larger half of it. At full capacity,
        // which is the case worth measuring.
        for id in 0..MAX_CHECKINS as u8 {
            schema += wrapped_len(DebugPayload::CheckinSlotInfo {
                id,
                label: name("CoordinatedHeatingElement"),
                period_ms: Some(u32::MAX),
            });
        }

        // A full table, every entry aged into a two-byte varint, which is what a healthy
        // machine actually emits. Ages are the only part that varies, and they only get
        // shorter as periods get tighter.
        let mut checkins: Vec<CheckinEntry, MAX_CHECKINS> = Vec::new();
        for i in 0..MAX_CHECKINS {
            let _ = checkins.push(CheckinEntry {
                status: CheckinStatus::Warning(CheckinDetail::Degraded),
                age_ms: 60_000 + i as u32,
            });
        }
        let checkin_report = wrapped_len(DebugPayload::CheckinReport(checkins));

        // Per second: one counter frame and one indicator frame per sample interval,
        // 1 snapshot, 1 check-in report, 1/5 of a schema burst.
        //
        // Derived from the constant rather than spelled, so that retuning the sampler's
        // default cadence moves this budget with it. That is not hypothetical: halving
        // the rate is what paid for the two display indicators above.
        let samples_per_second = 1000 / crate::sampler::DEFAULT_SAMPLE_INTERVAL_MS as usize;
        let per_second = samples_per_second * (counters + indicators)
            + snapshot
            + checkin_report
            + schema / 5;
        std::println!(
            "relay steady state: counters={counters} indicators={indicators} \
             snapshot={snapshot} checkins={checkin_report} schema_burst={schema} \
             => {per_second} B/s (budget {DEBUG_RELAY_BYTES_PER_SEC} B/s)"
        );
        assert!(
            per_second < DEBUG_RELAY_BYTES_PER_SEC as usize,
            "steady-state relay traffic {per_second} B/s exceeds the {DEBUG_RELAY_BYTES_PER_SEC} B/s budget"
        );

        // The same traffic against the pessimistic link. It is the binding case and the one
        // nobody would think to check.
        //
        // **~539 of 576 B/s.** It was 573 -- three bytes of headroom -- until the GS3's two
        // display frame-time indicators were added, which took it to 605 and over the line.
        // What paid for them was `sampler::DEFAULT_SAMPLE_INTERVAL_MS`, 500 ms -> 1000 ms:
        // sample frames are the only item here that scales with the metric count, so
        // halving their rate both funded the addition and stopped the next one from being
        // charged for it. That is the general shape of the knob -- a boot default, which a
        // host that actually wants the resolution raises at runtime with
        // `AppDebugOp::SetSampleIntervalMs`.
        //
        // The remaining knob, if this ever tightens again, is `checkin::REPORT_INTERVAL_MS`:
        // the check-in report is a full table once a second and the largest single item here
        // after the schema burst, so halving its cadence buys back 64 B/s and costs a host
        // one second of resolution on a table whose fastest slot is already sampled far more
        // often than it changes.
        //
        // No board runs 115 200 today -- both links are 576 kbaud, where this is under 20%.
        // This assert exists because that is a fact about the fleet, not about the protocol.
        let slow_budget = bytes_per_sec_for_baud(SLOW_BAUD) as usize;
        std::println!("  against the 115200-baud link: {per_second} B/s of {slow_budget} B/s");
        assert!(
            per_second < slow_budget,
            "steady-state relay traffic {per_second} B/s exceeds the slow link's {slow_budget} B/s budget"
        );

        // No single frame may exceed the bucket's capacity, or it is refused forever
        // however quiet the link gets -- the trap the old fixed window put `Status`
        // in, and which capacity above `MAX_FRAME` now avoids for every legal frame.
        let text = wrapped_len(DebugPayload::Text(
            Severity::Info,
            text("0123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123"),
        ));
        for (label, size) in [
            ("counters", counters),
            ("indicators", indicators),
            ("snapshot", snapshot),
            ("checkin report", checkin_report),
            ("largest schema frame", schema / (metric_frames + MAX_CHECKINS)),
            ("maximum-length text", text),
        ] {
            assert!(
                size <= BURST_BYTES as usize,
                "{label} is {size} B, above the bucket's {BURST_BYTES} B capacity"
            );
        }
    }

    /// The whole boot log must cross the relay, on **both** links.
    ///
    /// This is what [`BURST_BYTES`] is sized against, and it is the test that failed
    /// before the fixed window became a bucket: at 300 bytes per 100 ms only 12 of
    /// the 21 lines got through on the fast link, which is information destroyed
    /// rather than repetition collapsed. `crate::suppress` gives its own cap burst
    /// capacity for exactly this reason, and a fixed window here re-throttles the very
    /// lines that capacity exists to pass.
    ///
    /// Every line is measured at the maximum `TEXT_LEN` width, which is the worst
    /// case rather than the typical one, and the slow link is included because it
    /// refills only ~170 bytes across the whole burst -- nearly all of it has to come
    /// out of capacity there.
    ///
    /// Paired with `rate::the_sustained_bound_survives_the_burst_capacity`, because
    /// neither test suffices alone: a burst test cannot tell a generous bucket from an
    /// unbounded one, and a sustained test cannot tell a bucket from a fixed window.
    #[test]
    fn the_boot_burst_crosses_both_links() {
        use variegated_controller_types::ApplicationProcessorToCommsProcessorMessage as Msg;

        // `suppress::BOOT_BURST_LINES`, at the measured 15 ms spacing.
        const LINES: u32 = 21;
        const SPACING_MS: u64 = 15;

        for baud in [REFERENCE_BAUD, SLOW_BAUD] {
            let mut bucket = TokenBucket::for_baud(baud);
            let mut admitted = 0;
            let mut bytes = 0usize;

            for i in 0..LINES {
                // Maximum width: `TEXT_LEN` is 96 and `text()` truncates to it, so
                // this is the largest `Text` frame the bridge can produce.
                let payload = DebugPayload::Text(
                    Severity::Info,
                    text(&std::format!(
                        "{i:02} initialising subsystem, padded out to the full ninety-six character text capacity!!"
                    )),
                );
                let encoded =
                    postcard::to_allocvec_cobs(&Msg::Debug(frame(payload))).expect("encodes");
                bytes += encoded.len();
                if bucket.allow(i as u64 * SPACING_MS, encoded.len() as u32) {
                    admitted += 1;
                }
            }

            std::println!(
                "boot burst at {baud} baud: {admitted}/{LINES} lines admitted \
                 ({bytes} B offered over {} ms, capacity {BURST_BYTES} B, \
                 refill {} B/s)",
                (LINES as u64 - 1) * SPACING_MS,
                bytes_per_sec_for_baud(baud),
            );
            assert_eq!(
                admitted, LINES,
                "the boot log must cross intact at {baud} baud -- {admitted} of {LINES} got through"
            );
        }
    }

    /// The **inbound** direction of the same link, and why its accumulator is 4096.
    ///
    /// A `DebugCommand` reaches the application processor over three hops: the host
    /// codec encodes it (bounded by `MAX_FRAME`), the comms processor's
    /// `CommandDecoder` decodes it (also `MAX_FRAME`), and then the comms processor
    /// re-wraps it as a `CommsProcessorToApplicationProcessorMessage` and COBS-encodes
    /// it onto the UART, where `variegated_comms::esp_transceiver_main` feeds it to a
    /// `CobsAccumulator`. That accumulator was `<1024>` -- half of `MAX_FRAME` -- so
    /// every command the first two hops accepted and the third could not hold was
    /// discarded there with no counter and no event.
    ///
    /// It is now 4096. The obvious doubling to 2048 would have held this by exactly
    /// zero bytes -- and only because the host envelope's version byte and this
    /// message's enum tag happen to be one byte each, which no rule guarantees and
    /// which a shifted COBS block boundary can break. This test measures the figure
    /// rather than restating an arithmetic argument, because an earlier draft of that
    /// argument was wrong by nine bytes in the other direction.
    ///
    /// Lives here, in the module that owns the link's decisions, for the reason at the
    /// top of this file: `variegated_comms` links `embassy-rp` and cannot be
    /// host-tested at all.
    #[test]
    fn a_max_size_command_survives_the_inter_processor_hop() {
        use postcard::accumulator::{CobsAccumulator, FeedResult};
        use variegated_controller_types::commands::MachineCommand;
        use variegated_controller_types::debug_command::DebugCommand;
        use variegated_controller_types::routines::core::{Routine, RoutineType};
        use variegated_controller_types::CommsProcessorToApplicationProcessorMessage as Inbound;
        use variegated_debug_codec::{encode_command, MAX_FRAME};

        // `Routine::name` is an unbounded `alloc::String`, which is the cheapest way to
        // build a command of a chosen size -- and it is not contrived: `AddRoutine` is
        // named in the codec's own docs as the variant with no finite bound.
        let command = |name_len: usize| {
            DebugCommand::Machine(MachineCommand::AddRoutine(Routine {
                version: variegated_controller_types::ROUTINE_FORMAT_VERSION,
                routine_type: RoutineType::UserDefined,
                name: std::iter::repeat('A').take(name_len).collect(),
                parameters: std::vec::Vec::new(),
                derived_parameters: std::vec::Vec::new(),
                steps: std::vec::Vec::new(),
                finally: std::vec::Vec::new(),
                prerequisites: std::vec::Vec::new(),
                shot_annotations: std::vec::Vec::new(),
            }))
        };

        // The largest command the two upstream hops will pass: grow it until
        // `encode_command` refuses, then step back one.
        let mut buf = std::vec![0u8; MAX_FRAME];
        let mut largest = 0usize;
        for name_len in 0..MAX_FRAME {
            if encode_command(&command(name_len), &mut buf).is_err() {
                break;
            }
            largest = name_len;
        }
        let host_encoded = encode_command(&command(largest), &mut buf).expect("fits").len();
        assert!(
            host_encoded > MAX_FRAME - 16,
            "the fixture must actually reach the codec's ceiling, not stop short of it \
             ({host_encoded} B of {MAX_FRAME})"
        );

        // Now the third hop, exactly as the comms processor performs it.
        let on_the_wire =
            postcard::to_allocvec_cobs(&Inbound::DebugCommand(command(largest))).expect("encodes");
        std::println!(
            "largest injectable command: {host_encoded} B at the host codec, \
             {} B on the inter-processor link",
            on_the_wire.len()
        );

        // The buffer this used to be. It truncated this command silently, at the last
        // hop before the machine would have acted on it, having passed two decoders
        // that both said yes.
        assert!(
            on_the_wire.len() > 1024,
            "a 1024-byte accumulator would have held this ({} B); the fixture no longer \
             demonstrates the defect it exists for",
            on_the_wire.len()
        );
        // And the margin a doubling to 2048 would have left, which is none: 2048 is
        // both the largest message this link can carry and the whole of that buffer.
        // If this ever reads as comfortable headroom, the fixture has stopped reaching
        // the ceiling.
        assert_eq!(
            on_the_wire.len(),
            2048,
            "the worst case on this link is expected to sit exactly on MAX_FRAME"
        );

        // The size the link actually uses, fed in the 8-byte reads
        // `esp_transceiver_main` performs -- the chunking matters, because
        // `CobsAccumulator` decides `OverFull` per feed against what it has already
        // buffered, not against the message as a whole.
        let mut accumulator: CobsAccumulator<4096> = CobsAccumulator::new();
        let mut decoded = false;
        for chunk in on_the_wire.chunks(8) {
            let mut window = chunk;
            while !window.is_empty() {
                window = match accumulator.feed::<Inbound>(window) {
                    FeedResult::Consumed => break,
                    FeedResult::OverFull(_) => panic!(
                        "4096 bytes must hold the largest command the codec accepts ({} B)",
                        on_the_wire.len()
                    ),
                    FeedResult::DeserError(_) => {
                        panic!("the message the comms processor sent must decode")
                    }
                    FeedResult::Success { data, remaining } => {
                        // `DebugCommand` has no `Debug` and no `PartialEq` by design,
                        // so this checks the shape rather than comparing values.
                        assert!(matches!(data, Inbound::DebugCommand(DebugCommand::Machine(_))));
                        decoded = true;
                        remaining
                    }
                };
            }
        }
        assert!(decoded, "the command must arrive whole at the application processor");
    }
}
