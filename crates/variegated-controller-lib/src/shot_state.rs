//! Deciding which phase of a shot the machine is in.
//!
//! [`ShotState`] names three phases -- the headspace filling, the puck saturating, and
//! extraction proper once the first drop lands -- and this is what decides when one has
//! become the next.
//!
//! It sat in `variegated-controller-types` for a while, which was the wrong crate: this is
//! a state machine with a threshold policy, not a type. It was there because this crate
//! could not be built for a host, and this is exactly the kind of code that has to be run
//! against recorded shots rather than reasoned about -- the previous implementation was
//! reasoned about, and never fired. Both are true here now.
//!
//! # What saturation actually looks like
//!
//! Both of the transitions here are read off the same pair of signals, and it is worth
//! being explicit about the physics because the previous implementation was not and never
//! fired as a result.
//!
//! During headspace fill the pump is pushing water into an empty space above a dry puck.
//! There is almost no resistance, so flow climbs to whatever the pump can deliver -- around
//! 5 ml/s on this machine -- while pressure stays low and flat. As the puck wets through,
//! its resistance rises: flow falls away from that peak and pressure climbs off its floor.
//! That crossover *is* saturation, and it is a ramp lasting several seconds, not an event.
//!
//! The consequence for detection is the one that matters: the per-sample *change* in either
//! signal is small -- of the order of 0.1-0.3 units at a 400 ms cadence -- even though the
//! total excursion over the ramp is several units. Anything that differentiates the signal
//! and compares the result against a threshold big enough to be robust against noise will
//! therefore never fire. This tracks the extremes instead. `peak_input_flow` and
//! `trough_pressure` only ever move one way within a shot, so the comparison is against the
//! whole excursion so far rather than against one sample interval, and it does not depend
//! on the sampling rate at all.

use variegated_controller_types::control::group::ShotState;
use variegated_controller_types::{ECType, FlowRateType, PressureType, WeightType};

/// How often the state is re-evaluated, in milliseconds.
///
/// The controller's loop runs at 100 ms, so this is four ticks. The previous value was 333,
/// which against a 100 ms loop actually gated at 400 -- the constant and the behaviour
/// disagreed, and every comment derived from the constant was wrong by 20%. Naming the real
/// number is free and stops that.
const SAMPLE_INTERVAL_MS: u64 = 400;

/// The pump must have reached at least this flow before saturation can be declared, in
/// ml/s.
///
/// Without it, the first sample of a shot is trivially "below its own peak" and a
/// pressure blip would satisfy everything else. Headspace fill on this machine peaks
/// around 5 ml/s; 2.0 is comfortably below any real fill and comfortably above the
/// dribble that precedes one.
const SATURATION_MIN_PEAK_FLOW: FlowRateType = 2.0;

/// Flow must have fallen to this fraction of its peak.
///
/// Fractional rather than absolute because the peak depends on the routine -- a
/// pre-infusion at 3 ml/s and a full-power fill at 5.5 ml/s are both normal, and a fixed
/// "1 ml/s below peak" would mean quite different things to the two.
const SATURATION_FLOW_DROP_FRACTION: f32 = 0.85;

/// Pressure must have risen this far above its lowest reading since brewing started, in
/// bar.
///
/// Measured from the trough rather than from an absolute setpoint: the trough is where the
/// pump was working against nothing, so the rise above it is the puck's contribution and
/// nothing else. On a 6 bar routine the total rise is around 3 bar and on a 9 bar routine
/// around 6, so 1.0 sits early in the ramp on both without being reachable by noise.
const SATURATION_PRESSURE_RISE_BAR: PressureType = 1.0;

/// Consecutive gated samples that must satisfy the saturation predicate.
///
/// One, i.e. no debouncing. Both terms are already comparisons against a running extreme
/// rather than against a neighbour, so a single noisy sample cannot satisfy them unless it
/// is noisy in two signals at once and in the right directions. Named rather than inlined
/// because it is the first thing to raise if that ever turns out to be optimistic.
const SATURATION_CONFIRMING_SAMPLES: u8 = 1;

/// Output weight that counts as the first drop having landed, in grams.
const FIRST_DROP_WEIGHT_G: WeightType = 1.0;

/// Output conductivity that counts as the first drop having landed.
///
/// Coffee conducts and water essentially does not, so this is a direct observation that
/// what is leaving the group is no longer just the water that was already in the shower
/// screen.
const FIRST_DROP_EC: ECType = 1.0;

/// What the tracker needs to see each time it is updated.
///
/// Every field is optional because every one of them is a sensor a given machine may not
/// have. A missing signal removes the transitions that depend on it rather than defaulting
/// to a number: a machine with no scale should fail to notice the first drop by weight, not
/// decide it weighs zero forever.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ShotStateInputs {
    pub input_flow_rate: Option<FlowRateType>,
    pub pressure: Option<PressureType>,
    pub output_weight: Option<WeightType>,
    pub output_electrical_conductivity: Option<ECType>,
}

/// Tracks which phase of a shot the machine is in.
///
/// Drive it with [`start`](Self::start) when brewing begins, [`update`](Self::update) on
/// every control-loop tick, and [`stop`](Self::stop) when brewing ends. `update` does its
/// own rate limiting, so calling it more often than [`SAMPLE_INTERVAL_MS`] is free and
/// calling it less often simply lowers the resolution.
#[derive(Clone, Copy, Debug, Default)]
pub struct ShotStateTracker {
    state: Option<ShotState>,
    peak_input_flow: FlowRateType,
    /// `None` until the first pressure reading of the shot, so that the trough starts at a
    /// real measurement rather than at zero -- which nothing could ever fall below.
    trough_pressure: Option<PressureType>,
    confirming_samples: u8,
    last_sample_millis: Option<u64>,
    saturation_entered_millis: Option<u64>,
}

impl ShotStateTracker {
    pub fn new() -> Self {
        Self::default()
    }

    /// Begin a shot. Discards everything from the previous one.
    pub fn start(&mut self) {
        *self = Self {
            state: Some(ShotState::HeadspaceFill),
            ..Self::default()
        };
    }

    /// End a shot. The state becomes `None`, which is what "not brewing" looks like.
    pub fn stop(&mut self) {
        *self = Self::default();
    }

    pub fn state(&self) -> Option<ShotState> {
        self.state
    }

    /// When saturation was entered, on the same clock passed to [`update`](Self::update).
    pub fn saturation_entered_millis(&self) -> Option<u64> {
        self.saturation_entered_millis
    }

    /// Advance the state machine, returning the new state **only if it changed**.
    ///
    /// `now_millis` is any monotonic millisecond clock; the tracker only ever takes
    /// differences of it. Plain milliseconds rather than an `embassy_time::Instant` so this
    /// can be run against recorded shots on the host, which is the only way the thresholds
    /// above can be justified rather than guessed at.
    ///
    /// At most one transition per call, deliberately. On a machine with a conductivity
    /// sensor the puck saturating and the first drop arriving can land within one sample of
    /// each other, and collapsing them would mean `Saturation` was entered and left without
    /// ever appearing in a single logged sample -- which reads, in the shot log, exactly
    /// like the saturation detector not working.
    pub fn update(&mut self, now_millis: u64, inputs: ShotStateInputs) -> Option<ShotState> {
        let state = self.state?;

        if let Some(last) = self.last_sample_millis {
            if now_millis.saturating_sub(last) < SAMPLE_INTERVAL_MS {
                return None;
            }
        }
        self.last_sample_millis = Some(now_millis);

        if let Some(flow) = inputs.input_flow_rate {
            if flow > self.peak_input_flow {
                self.peak_input_flow = flow;
            }
        }
        if let Some(pressure) = inputs.pressure {
            self.trough_pressure = Some(match self.trough_pressure {
                Some(trough) if trough <= pressure => trough,
                _ => pressure,
            });
        }

        if state == ShotState::HeadspaceFill && self.saturation_reached(&inputs) {
            self.state = Some(ShotState::Saturation);
            self.saturation_entered_millis = Some(now_millis);
            return Some(ShotState::Saturation);
        }

        if state != ShotState::PostFirstDrop && self.first_drop_reached(state, &inputs) {
            self.state = Some(ShotState::PostFirstDrop);
            return Some(ShotState::PostFirstDrop);
        }

        None
    }

    /// Flow has fallen away from its peak *and* pressure has climbed off its floor.
    ///
    /// Both, not either. Flow alone falls whenever a routine steps the pump down, and
    /// pressure alone rises whenever it steps the pump up; only the two together are the
    /// puck taking up the difference.
    fn saturation_reached(&mut self, inputs: &ShotStateInputs) -> bool {
        let (Some(flow), Some(pressure), Some(trough)) =
            (inputs.input_flow_rate, inputs.pressure, self.trough_pressure)
        else {
            self.confirming_samples = 0;
            return false;
        };

        let satisfied = self.peak_input_flow >= SATURATION_MIN_PEAK_FLOW
            && flow < self.peak_input_flow * SATURATION_FLOW_DROP_FRACTION
            && pressure - trough > SATURATION_PRESSURE_RISE_BAR;

        if !satisfied {
            self.confirming_samples = 0;
            return false;
        }

        self.confirming_samples = self.confirming_samples.saturating_add(1);
        self.confirming_samples >= SATURATION_CONFIRMING_SAMPLES
    }

    /// Whether coffee has started leaving the group.
    ///
    /// Conductivity counts from any state; weight only from [`ShotState::Saturation`]. The
    /// asymmetry is not tidiness, it is the scale's tare. `start_brewing` tares the scale,
    /// and a Bluetooth scale runs several of its own measuring cycles before the reading
    /// settles -- so for the first moments of a shot the weight can be anything the cup
    /// happened to read before. Requiring saturation first means those moments have passed.
    /// Conductivity has no such settling behaviour and water does not conduct, so it can be
    /// trusted from the start, which is what lets a machine with no scale still see the
    /// first drop.
    fn first_drop_reached(&self, state: ShotState, inputs: &ShotStateInputs) -> bool {
        if inputs
            .output_electrical_conductivity
            .is_some_and(|ec| ec > FIRST_DROP_EC)
        {
            return true;
        }

        state == ShotState::Saturation
            && inputs
                .output_weight
                .is_some_and(|weight| weight > FIRST_DROP_WEIGHT_G)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec::Vec;

    /// Drive the tracker with a series of `(millis, flow, pressure, weight, ec)` readings
    /// and collect every transition it reports.
    fn transitions(samples: &[(u64, f32, f32, f32, f32)]) -> Vec<(u64, ShotState)> {
        let mut tracker = ShotStateTracker::new();
        tracker.start();
        let mut seen = Vec::new();
        for &(t, flow, pressure, weight, ec) in samples {
            if let Some(state) = tracker.update(
                t,
                ShotStateInputs {
                    input_flow_rate: Some(flow),
                    pressure: Some(pressure),
                    output_weight: Some(weight),
                    output_electrical_conductivity: Some(ec),
                },
            ) {
                seen.push((t, state));
            }
        }
        seen
    }

    /// A shot pulled on the real machine, at the cadence the tracker actually samples.
    ///
    /// Taken from a recorded 6 bar / 85 mL routine: flow ramps to 5.5 ml/s against a flat
    /// 3.3 bar, then rolls off as pressure climbs to 6, and conductivity crosses at 10.5 s.
    /// The numbers are the recording's, rounded to two decimals.
    const RECORDED_SIX_BAR: &[(u64, f32, f32, f32, f32)] = &[
        (400, 0.36, 4.58, 0.0, 0.0),
        (800, 2.53, 2.28, 0.0, 0.0),
        (1200, 4.39, 3.04, 0.0, 0.0),
        (1600, 5.19, 3.25, 0.0, 0.0),
        (2000, 5.49, 3.32, 0.0, 0.0),
        (2800, 5.35, 3.36, 0.0, 0.0),
        (3200, 5.45, 3.36, 0.0, 0.0),
        (3600, 5.16, 3.36, 0.0, 0.0),
        (4000, 5.03, 3.41, 0.0, 0.0),
        (4400, 4.97, 3.46, 0.0, 0.0),
        (4800, 4.91, 3.55, 0.0, 0.0),
        (5200, 4.85, 3.66, 0.0, 0.0),
        (5600, 5.06, 3.76, 0.0, 0.0),
        (6400, 4.72, 3.95, 0.0, 0.0),
        (6800, 4.98, 4.11, 0.0, 0.0),
        (7200, 4.74, 4.29, 0.0, 0.0),
        (7600, 4.72, 4.45, 0.0, 0.0),
        (8000, 4.19, 4.65, 0.0, 0.0),
        (8800, 3.97, 5.05, 0.0, 0.0),
        (9200, 3.52, 5.47, 0.0, 0.0),
        (9600, 3.40, 5.81, 0.93, 0.0),
        (10000, 2.94, 6.09, 1.27, 0.0),
        (10400, 2.26, 6.11, 2.19, 2.66),
        (10800, 1.57, 5.93, 3.25, 4.47),
    ];

    /// The same, from a recorded 9 bar / 61 mL routine, where the roll-off is much sharper
    /// and conductivity crosses at 9.3 s.
    const RECORDED_NINE_BAR: &[(u64, f32, f32, f32, f32)] = &[
        (400, 0.36, 2.46, 0.0, 0.0),
        (800, 2.48, 2.30, 0.0, 0.0),
        (1200, 3.91, 3.04, 0.0, 0.0),
        (1600, 5.06, 3.22, 0.0, 0.0),
        (2400, 5.07, 3.41, 0.0, 0.0),
        (2800, 5.05, 3.56, -0.06, 0.0),
        (3200, 5.10, 3.70, 0.0, 0.0),
        (3600, 4.86, 3.87, 0.0, 0.0),
        (4000, 4.83, 4.05, 0.0, 0.0),
        (4400, 4.45, 4.28, 0.0, 0.0),
        (4800, 4.40, 4.39, 0.0, 0.0),
        (5200, 4.37, 4.68, 0.0, 0.0),
        (5600, 4.08, 5.15, 0.0, 0.0),
        (6000, 3.72, 5.77, 0.0, 0.0),
        (6400, 3.34, 6.63, 0.0, 0.0),
        (6800, 2.58, 7.59, 0.0, 0.0),
        (7200, 2.16, 8.38, 0.10, 0.0),
        (7600, 1.46, 9.00, 0.39, 0.0),
        (8000, 1.09, 9.27, 0.48, 0.0),
        (8400, 1.11, 8.93, 0.56, 0.0),
        (8800, 0.36, 8.28, 0.99, 0.22),
        (9200, 0.36, 8.07, 1.08, 1.45),
    ];

    /// Both recorded shots reach saturation, and reach it before the first drop.
    ///
    /// This is the regression that motivated the rewrite. The previous predicate compared
    /// each sample against a ten-sample moving average and required a 3.0 ml/s fall *and* a
    /// 3.0 bar rise within one 400 ms sample; replayed over these two shots the largest
    /// values it ever saw were 1.83 / 1.22 and 2.07 / 2.99, so it never fired on either,
    /// and `Saturation` appeared in no logged sample of either shot.
    #[test]
    fn both_recorded_shots_saturate_before_the_first_drop() {
        for (name, samples) in [
            ("6 bar, 85 mL", RECORDED_SIX_BAR),
            ("9 bar, 61 mL", RECORDED_NINE_BAR),
        ] {
            let seen = transitions(samples);

            let saturation = seen
                .iter()
                .find(|(_, state)| *state == ShotState::Saturation)
                .unwrap_or_else(|| panic!("{name} never reached Saturation"));
            let first_drop = seen
                .iter()
                .find(|(_, state)| *state == ShotState::PostFirstDrop)
                .unwrap_or_else(|| panic!("{name} never reached PostFirstDrop"));

            assert!(
                saturation.0 < first_drop.0,
                "{name}: saturation at {} ms is not before the first drop at {} ms",
                saturation.0,
                first_drop.0
            );
            // Saturation is a phase, not an instant. If it were entered one sample before
            // the first drop it would be technically present and practically useless, so
            // this asserts it lasts long enough to mean something.
            assert!(
                first_drop.0 - saturation.0 >= 2 * SAMPLE_INTERVAL_MS,
                "{name}: saturation lasted only {} ms",
                first_drop.0 - saturation.0
            );
        }
    }

    /// Saturation is not declared during the fill, while flow is still climbing.
    ///
    /// The failure this guards against is the opposite of the one above: thresholds loose
    /// enough to fire on the two recordings but also loose enough to fire on the ramp, which
    /// would put every shot in `Saturation` a second after the pump started.
    #[test]
    fn the_fill_itself_does_not_count_as_saturation() {
        for samples in [RECORDED_SIX_BAR, RECORDED_NINE_BAR] {
            let seen = transitions(samples);
            let (at, _) = seen
                .iter()
                .find(|(_, state)| *state == ShotState::Saturation)
                .expect("saturation");
            assert!(
                *at >= 4_000,
                "saturation at {at} ms is inside the headspace fill"
            );
        }
    }

    /// A shot with no conductivity sensor still reaches the first drop, by weight.
    #[test]
    fn weight_reaches_the_first_drop_without_a_conductivity_sensor() {
        let mut tracker = ShotStateTracker::new();
        tracker.start();
        let mut last = None;
        for &(t, flow, pressure, weight, _ec) in RECORDED_SIX_BAR {
            if let Some(state) = tracker.update(
                t,
                ShotStateInputs {
                    input_flow_rate: Some(flow),
                    pressure: Some(pressure),
                    output_weight: Some(weight),
                    output_electrical_conductivity: None,
                },
            ) {
                last = Some(state);
            }
        }
        assert_eq!(last, Some(ShotState::PostFirstDrop));
    }

    /// With neither a conductivity probe nor a scale, a shot still saturates and then stops
    /// there.
    ///
    /// This is a real machine configuration, not a hypothetical: the Silvia has no EC probe
    /// at all, and its scale is a Bluetooth one that reports nothing until it connects. Both
    /// halves matter. Saturation must still be detected, because it is what the pressure and
    /// flow signals alone can support and what a routine's `ShotStateReached` step waits on;
    /// and the shot must not advance past it, because nothing left can observe the first
    /// drop. Parking at `Saturation` is the intended degradation.
    #[test]
    fn a_shot_with_no_scale_and_no_conductivity_sensor_stops_at_saturation() {
        let mut tracker = ShotStateTracker::new();
        tracker.start();
        for &(t, flow, pressure, _weight, _ec) in RECORDED_SIX_BAR {
            tracker.update(
                t,
                ShotStateInputs {
                    input_flow_rate: Some(flow),
                    pressure: Some(pressure),
                    output_weight: None,
                    output_electrical_conductivity: None,
                },
            );
        }
        assert_eq!(tracker.state(), Some(ShotState::Saturation));
    }

    /// A scale that has not finished taring cannot skip the shot straight to the first drop.
    ///
    /// The reading here is a cup left on the scale from the last shot: 36 g, well over the
    /// threshold, for the whole of the fill. Without the state guard in `first_drop_reached`
    /// the very first sample would report `PostFirstDrop` and the shot would have no phases
    /// at all.
    #[test]
    fn a_stale_scale_reading_does_not_trigger_the_first_drop() {
        let mut tracker = ShotStateTracker::new();
        tracker.start();
        for t in [400u64, 800, 1200, 1600] {
            tracker.update(
                t,
                ShotStateInputs {
                    input_flow_rate: Some(1.0 + t as f32 / 1000.0),
                    pressure: Some(3.0),
                    output_weight: Some(36.0),
                    output_electrical_conductivity: Some(0.0),
                },
            );
        }
        assert_eq!(tracker.state(), Some(ShotState::HeadspaceFill));
    }

    /// Updates faster than the sample interval are ignored rather than shortening the
    /// window, so the thresholds mean the same thing whatever the caller's loop rate is.
    #[test]
    fn updates_between_samples_are_ignored() {
        let mut tracker = ShotStateTracker::new();
        tracker.start();

        let inputs = ShotStateInputs {
            input_flow_rate: Some(5.0),
            pressure: Some(3.0),
            ..Default::default()
        };
        assert_eq!(tracker.update(0, inputs), None);
        // A pressure spike inside the interval must not reach the trough tracker, or the
        // floor would be set by whichever sample happened to be lowest at 100 Hz rather
        // than by the 2.5 Hz series the thresholds were chosen against.
        tracker.update(
            100,
            ShotStateInputs {
                pressure: Some(0.1),
                ..inputs
            },
        );
        tracker.update(
            500,
            ShotStateInputs {
                input_flow_rate: Some(1.0),
                pressure: Some(3.0),
                ..Default::default()
            },
        );
        assert_eq!(
            tracker.state(),
            Some(ShotState::HeadspaceFill),
            "the spike at 100 ms was inside the interval and must not have set the trough"
        );
    }

    /// A stopped tracker reports no state, and a restarted one starts over.
    #[test]
    fn stopping_clears_everything() {
        let mut tracker = ShotStateTracker::new();
        tracker.start();
        for &(t, flow, pressure, weight, ec) in RECORDED_SIX_BAR {
            tracker.update(
                t,
                ShotStateInputs {
                    input_flow_rate: Some(flow),
                    pressure: Some(pressure),
                    output_weight: Some(weight),
                    output_electrical_conductivity: Some(ec),
                },
            );
        }
        assert_eq!(tracker.state(), Some(ShotState::PostFirstDrop));

        tracker.stop();
        assert_eq!(tracker.state(), None);
        // Updating a stopped tracker is a no-op rather than a resurrection.
        assert_eq!(tracker.update(20_000, ShotStateInputs::default()), None);
        assert_eq!(tracker.state(), None);

        tracker.start();
        assert_eq!(tracker.state(), Some(ShotState::HeadspaceFill));
        assert_eq!(tracker.saturation_entered_millis(), None);
    }
}
