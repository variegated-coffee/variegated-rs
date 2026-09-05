//! Generic counter/indicator sampling and schema emission.
//!
//! Replaces per-metric hand-written logging: the device ships raw values and the
//! host derives rates. Names are emitted one `MetricName` at a time and re-sent
//! periodically, which is how a client that attaches late learns them without any
//! handshake.

use core::iter;

use heapless::Vec;
use portable_atomic::{AtomicU32, Ordering};
use variegated_controller_types::debug::{DebugPayload, MetricKind, MAX_SAMPLES, name};
use variegated_instrumentation::{PerformanceCounters, PerformanceIndicators};

/// Sampling period for counters and indicators.
///
/// This is a *boot default*, not a limit: a host that wants finer resolution raises the
/// rate at runtime with `AppDebugOp::SetSampleIntervalMs`, which is clamped only to
/// 50..60000. So the cost of a slower default falls on nobody who is actually watching.
///
/// It was 500 ms until the two display frame-time indicators were added. Sample frames
/// are the only part of steady-state debug traffic that scales with the metric count, and
/// on a 115 200-baud link the total had ~3 B/s of headroom left -- see
/// `relay::tests::steady_state_relay_traffic_fits_the_budget`, which derives its own
/// arithmetic from this constant and is what will fail if it moves back.
pub const DEFAULT_SAMPLE_INTERVAL_MS: u32 = 1_000;
/// How often the full schema is re-emitted for late-attaching clients.
pub const SCHEMA_INTERVAL_MS: u32 = 5_000;

static SAMPLE_INTERVAL_MS: AtomicU32 = AtomicU32::new(DEFAULT_SAMPLE_INTERVAL_MS);

pub fn sample_interval_ms() -> u32 {
    SAMPLE_INTERVAL_MS.load(Ordering::Relaxed)
}

/// Runtime rate control, driven by `AppDebugOp::SetSampleIntervalMs`. Clamped so a
/// bad injected value cannot spin the sampler.
pub fn set_sample_interval_ms(interval: u32) {
    SAMPLE_INTERVAL_MS.store(interval.clamp(50, 60_000), Ordering::Relaxed);
}

pub struct Sampler<const NC: usize, const NI: usize> {
    counters: &'static PerformanceCounters<NC>,
    indicators: &'static PerformanceIndicators<NI>,
    counter_names: &'static [&'static str],
    indicator_names: &'static [&'static str],
    firmware: &'static str,
    checkins: u8,
}

impl<const NC: usize, const NI: usize> Sampler<NC, NI> {
    /// Panics if the metric count exceeds what one frame can carry, or if a name
    /// table does not match its metric count -- both are wiring mistakes that
    /// should fail loudly at startup rather than produce mislabelled data.
    pub fn new(
        counters: &'static PerformanceCounters<NC>,
        indicators: &'static PerformanceIndicators<NI>,
        counter_names: &'static [&'static str],
        indicator_names: &'static [&'static str],
        firmware: &'static str,
    ) -> Self {
        assert!(NC <= MAX_SAMPLES, "too many counters for one frame");
        assert!(NI <= MAX_SAMPLES, "too many indicators for one frame");
        assert!(counter_names.len() == NC, "counter name table does not match NC");
        assert!(indicator_names.len() == NI, "indicator name table does not match NI");
        Self { counters, indicators, counter_names, indicator_names, firmware, checkins: 0 }
    }

    /// Declare how many check-in slots this firmware has, for `FirmwareInfo`.
    ///
    /// A builder rather than a sixth `new` parameter because the count comes from a
    /// different subsystem than everything else here -- the sampler neither reads the
    /// check-in table nor publishes it, and pretending otherwise in the constructor would
    /// suggest it does. `FirmwareInfo` carries the number regardless because it is the
    /// one frame that describes the firmware's shape, and a host wants to size its table
    /// before the first `CheckinSlotInfo` arrives.
    ///
    /// Left unset the count is 0, which is honest for a firmware that declares no slots.
    pub fn with_checkins(mut self, checkins: u8) -> Self {
        self.checkins = checkins;
        self
    }

    pub fn counter_payload(&self) -> DebugPayload {
        let mut samples: Vec<u64, MAX_SAMPLES> = Vec::new();
        // Cannot fail: NC <= MAX_SAMPLES is asserted in `new`.
        let _ = samples.extend_from_slice(&self.counters.read_all());
        DebugPayload::CounterSamples(samples)
    }

    pub fn indicator_payload(&self) -> DebugPayload {
        let mut samples: Vec<u64, MAX_SAMPLES> = Vec::new();
        let _ = samples.extend_from_slice(&self.indicators.read_all());
        DebugPayload::IndicatorSamples(samples)
    }

    /// `FirmwareInfo`, then one `MetricName` per counter, then per indicator.
    pub fn schema_payloads(&self) -> impl Iterator<Item = DebugPayload> + '_ {
        let info = iter::once(DebugPayload::FirmwareInfo {
            firmware: name(self.firmware),
            counters: NC as u8,
            indicators: NI as u8,
            checkins: self.checkins,
        });

        let counters = self.counter_names.iter().enumerate().map(|(id, label)| {
            DebugPayload::MetricName { kind: MetricKind::Counter, id: id as u8, label: name(label) }
        });

        let indicators = self.indicator_names.iter().enumerate().map(|(id, label)| {
            DebugPayload::MetricName { kind: MetricKind::Indicator, id: id as u8, label: name(label) }
        });

        info.chain(counters).chain(indicators)
    }
}

/// Emit counters, indicators and the schema, forever.
///
/// A plain `async fn` rather than an `#[embassy_executor::task]`, because a task cannot be
/// generic and `Sampler` is generic over its metric counts. Each firmware keeps a
/// two-line task that builds its `Sampler` and awaits this -- the same split
/// `esp_transceiver_main` uses in `variegated-comms`.
///
/// It emits [`DebugEvent::Boot`] first, so the event stream starts with something that
/// identifies the run.
///
/// The schema is re-sent every [`SCHEMA_INTERVAL_MS`] rather than once. There is no
/// handshake on this link -- emission is always-on and a client may attach at any point --
/// so periodic re-emission is the only way a late client learns what the metric ids mean.
pub async fn run<const NC: usize, const NI: usize>(sampler: Sampler<NC, NI>) -> ! {
    crate::bus::emit_event(variegated_controller_types::debug::DebugEvent::Boot);

    // Starts *at* the interval so the first pass emits the schema immediately, rather than
    // leaving a client attached at boot without names for the first five seconds.
    let mut since_schema_ms = SCHEMA_INTERVAL_MS;

    loop {
        if since_schema_ms >= SCHEMA_INTERVAL_MS {
            for payload in sampler.schema_payloads() {
                crate::bus::publish(payload);
            }
            since_schema_ms = 0;
        }

        crate::bus::publish(sampler.counter_payload());
        crate::bus::publish(sampler.indicator_payload());

        // Re-read every pass rather than caching: `set_sample_interval_ms` is driven by a
        // debug command, and a cached period would ignore it until the next restart.
        let interval = sample_interval_ms();
        embassy_time::Timer::after_millis(interval as u64).await;
        since_schema_ms = since_schema_ms.saturating_add(interval);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec;
    use variegated_controller_types::debug::{DebugPayload, MetricKind};
    use variegated_instrumentation::{PerformanceCounters, PerformanceIndicators};

    static COUNTERS: PerformanceCounters<2> = PerformanceCounters::new();
    static INDICATORS: PerformanceIndicators<1> = PerformanceIndicators::new();

    fn sampler() -> Sampler<2, 1> {
        Sampler::new(&COUNTERS, &INDICATORS, &["Loops", "Reads"], &["Temp"], "test-fw")
    }

    #[test]
    fn counter_payload_carries_every_counter_in_id_order() {
        COUNTERS.handle(0u8).increment();
        COUNTERS.handle(1u8).add(5);

        match sampler().counter_payload() {
            DebugPayload::CounterSamples(v) => assert_eq!(v.as_slice(), &[1, 5]),
            other => panic!("expected CounterSamples, got {other:?}"),
        }
    }

    #[test]
    fn schema_payloads_cover_counters_then_indicators() {
        let s = sampler();
        let payloads: Vec<_> = s.schema_payloads().collect();

        assert_eq!(payloads.len(), 4, "1 FirmwareInfo + 2 counters + 1 indicator");
        assert!(matches!(payloads[0], DebugPayload::FirmwareInfo { counters: 2, indicators: 1, .. }));
        assert!(matches!(
            &payloads[1],
            DebugPayload::MetricName { kind: MetricKind::Counter, id: 0, label } if label == "Loops"
        ));
        assert!(matches!(
            &payloads[3],
            DebugPayload::MetricName { kind: MetricKind::Indicator, id: 0, label } if label == "Temp"
        ));
    }

    #[test]
    fn sample_interval_is_runtime_settable() {
        set_sample_interval_ms(250);
        assert_eq!(sample_interval_ms(), 250);
        set_sample_interval_ms(DEFAULT_SAMPLE_INTERVAL_MS);
    }
}
