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
pub const DEFAULT_SAMPLE_INTERVAL_MS: u32 = 500;
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
        Self { counters, indicators, counter_names, indicator_names, firmware }
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
