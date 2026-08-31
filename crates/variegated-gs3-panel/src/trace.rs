//! The shot the post-routine panel draws, accumulated as it happens.
//!
//! Section 6.5 wants a pressure and weight curve, the phase bands behind it, the time of
//! first drop, the peak pressure and the brew ratio. None of that exists in the machine's
//! published status: the controller does keep a full 10 Hz sample log, but it is a private
//! field on the other core and there is no route to it from a display task.
//!
//! So the display accumulates its own, from the status stream it already receives. This is
//! not a second shot log -- it is 190 buckets, the width of the curve, and it exists only
//! until the next shot starts.
//!
//! Two things it deliberately keeps that the status does not:
//!
//! * **The dose**, latched at the start. The controller clears the pending annotations when
//!   a shot finishes, so by the time the post-routine panel is drawn the dose the ratio
//!   needs is already gone.
//! * **When first drop happened.** The status carries the shot *phase*, not the moment it
//!   changed. The phase detector runs on a 400 ms cadence, so the latched time is good to
//!   about that -- which is why it is stated to one decimal and no further.

/// Buckets in the curve: the width in pixels the specification gives it beside a figure
/// block. A full-bleed chart is 356 px and interpolates up; nothing interpolates down.
pub const WIDTH: usize = 190;

/// The finest bucket, and the rate the controller publishes status at. A shot up to
/// `WIDTH * 100 ms` = 19 s is recorded sample for sample.
const BASE_BUCKET_MS: u32 = 100;

/// Where a shot is in its extraction. Mirrors the controller's `ShotState`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Phase {
    /// Filling the headspace above the puck.
    HeadspaceFill,
    /// The puck is saturating.
    Saturation,
    /// Coffee is coming out.
    PostFirstDrop,
}

/// A shot's pressure and weight over time, decimated to the width of the curve.
#[derive(Clone, Debug)]
pub struct ShotTrace {
    pressure: [f32; WIDTH],
    weight: [f32; WIDTH],
    /// Buckets written so far.
    len: usize,
    /// How much time one bucket covers. Doubles whenever the shot outgrows the buffer.
    bucket_ms: u32,
    /// The last elapsed time pushed.
    elapsed_ms: u32,
    peak_pressure: f32,
    saturation_at_ms: Option<u32>,
    first_drop_at_ms: Option<u32>,
    dose_g: Option<f32>,
}

impl Default for ShotTrace {
    fn default() -> Self {
        Self::new()
    }
}

impl ShotTrace {
    /// An empty trace.
    pub const fn new() -> Self {
        Self {
            pressure: [0.0; WIDTH],
            weight: [0.0; WIDTH],
            len: 0,
            bucket_ms: BASE_BUCKET_MS,
            elapsed_ms: 0,
            peak_pressure: 0.0,
            saturation_at_ms: None,
            first_drop_at_ms: None,
            dose_g: None,
        }
    }

    /// Begin a shot, latching the dose it is being pulled against.
    ///
    /// Everything else is cleared. A trace is never merged with the one before it: the
    /// post-routine panel describes one shot, and a stale bucket at the right-hand end would
    /// be indistinguishable from a real sample.
    pub fn start(&mut self, dose_g: Option<f32>) {
        self.pressure = [0.0; WIDTH];
        self.weight = [0.0; WIDTH];
        self.len = 0;
        self.bucket_ms = BASE_BUCKET_MS;
        self.elapsed_ms = 0;
        self.peak_pressure = 0.0;
        self.saturation_at_ms = None;
        self.first_drop_at_ms = None;
        self.dose_g = dose_g;
    }

    /// Record one status.
    ///
    /// `elapsed_ms` is time since the pump started. Calling this twice within one bucket is
    /// normal and expected -- the samples fold together rather than advancing the curve.
    ///
    /// A `None` reading is not zero: it leaves the bucket at whatever the last real reading
    /// put there, which draws as a flat segment rather than as a drop to the floor.
    pub fn push(
        &mut self,
        elapsed_ms: u32,
        pressure: Option<f32>,
        weight: Option<f32>,
        phase: Option<Phase>,
    ) {
        self.elapsed_ms = elapsed_ms;

        match phase {
            Some(Phase::Saturation) if self.saturation_at_ms.is_none() => {
                self.saturation_at_ms = Some(elapsed_ms);
            }
            Some(Phase::PostFirstDrop) => {
                // Both, because a shot can be observed to have skipped straight past
                // saturation: the detector samples at 400 ms and a fast headspace fill can
                // cross both thresholds inside one interval. Leaving `saturation_at_ms`
                // empty would then draw the saturation band as the whole shot.
                if self.saturation_at_ms.is_none() {
                    self.saturation_at_ms = Some(elapsed_ms);
                }
                if self.first_drop_at_ms.is_none() {
                    self.first_drop_at_ms = Some(elapsed_ms);
                }
            }
            _ => {}
        }

        if let Some(bar) = pressure
            && bar > self.peak_pressure
        {
            self.peak_pressure = bar;
        }

        // Fold until the sample fits. A loop rather than a single halving because a status
        // can arrive late -- the display task is not the control loop -- and one push may
        // have to cross two doublings.
        while elapsed_ms / self.bucket_ms >= WIDTH as u32 {
            self.fold();
        }

        let index = (elapsed_ms / self.bucket_ms) as usize;

        if index >= self.len {
            // A bucket this sample is the first to reach. Carry the previous value into any
            // bucket the sample *skipped* -- a gap in the status stream should draw as a
            // held value rather than as a fall to zero -- and then **set** this one.
            //
            // Setting rather than folding matters more than it looks. Carrying forward into
            // the new bucket too and then taking the maximum makes the curve monotonically
            // non-decreasing: every falling sample loses to the value carried in from the
            // bucket before it, and a declining pressure profile draws as a flat line at its
            // peak. That is what this did before, and the rendered curve is the only place
            // it showed.
            let carry = self.len.checked_sub(1);
            for i in self.len..index {
                if let Some(prev) = carry {
                    self.pressure[i] = self.pressure[prev];
                    self.weight[i] = self.weight[prev];
                }
            }
            self.pressure[index] = pressure
                .or_else(|| carry.map(|p| self.pressure[p]))
                .unwrap_or(0.0);
            self.weight[index] = weight
                .or_else(|| carry.map(|p| self.weight[p]))
                .unwrap_or(0.0);
            self.len = index + 1;
        } else {
            // A second sample inside a bucket already open. Pressure keeps the maximum, so a
            // decimated curve cannot drop the peak the legend beside it states; weight keeps
            // the latest, because it only climbs.
            if let Some(bar) = pressure
                && bar > self.pressure[index]
            {
                self.pressure[index] = bar;
            }
            if let Some(grams) = weight {
                self.weight[index] = grams;
            }
        }
    }

    /// Halve the resolution: fold each pair of buckets into one and double the bucket.
    fn fold(&mut self) {
        for i in 0..WIDTH / 2 {
            let (a, b) = (i * 2, i * 2 + 1);
            // Pressure keeps the larger of the pair, weight the later. Weight only climbs,
            // so "later" is also "larger"; pressure does not, and taking its maximum is what
            // keeps the peak on the curve through three foldings of a long shot.
            self.pressure[i] = self.pressure[a].max(self.pressure[b]);
            self.weight[i] = self.weight[b];
        }
        for i in WIDTH / 2..WIDTH {
            self.pressure[i] = 0.0;
            self.weight[i] = 0.0;
        }
        self.len = self.len.div_ceil(2);
        self.bucket_ms *= 2;
    }

    /// The pressure curve, one value per bucket.
    pub fn pressure(&self) -> &[f32] {
        &self.pressure[..self.len]
    }

    /// The weight curve, one value per bucket.
    pub fn weight(&self) -> &[f32] {
        &self.weight[..self.len]
    }

    /// How much time one bucket covers, in milliseconds.
    pub fn bucket_ms(&self) -> u32 {
        self.bucket_ms
    }

    /// Whether anything has been recorded.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// The whole shot's duration, in seconds.
    pub fn elapsed_seconds(&self) -> f32 {
        self.elapsed_ms as f32 / 1000.0
    }

    /// The highest pressure seen, in bar.
    pub fn peak_pressure(&self) -> Option<f32> {
        (self.peak_pressure > 0.0).then_some(self.peak_pressure)
    }

    /// When the puck began saturating, in seconds.
    pub fn saturation_seconds(&self) -> Option<f32> {
        self.saturation_at_ms.map(|ms| ms as f32 / 1000.0)
    }

    /// When the first drop landed, in seconds. Good to about 400 ms; see the module note.
    pub fn first_drop_seconds(&self) -> Option<f32> {
        self.first_drop_at_ms.map(|ms| ms as f32 / 1000.0)
    }

    /// The dose this shot was pulled against, latched at the start.
    pub fn dose_g(&self) -> Option<f32> {
        self.dose_g
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A 19 s shot fits without folding: one bucket per status.
    #[test]
    fn a_short_shot_is_recorded_sample_for_sample() {
        let mut trace = ShotTrace::new();
        trace.start(Some(19.9));
        for i in 0..150u32 {
            trace.push(i * 100, Some(6.0), Some(i as f32 * 0.1), None);
        }
        assert_eq!(trace.bucket_ms(), 100);
        assert_eq!(trace.pressure().len(), 150);
    }

    /// The specification's own shot: 51.1 s, which needs two foldings to fit 190 buckets.
    /// The peak has to survive both, because the legend beside the curve states it.
    #[test]
    fn decimation_preserves_the_peak() {
        let mut trace = ShotTrace::new();
        trace.start(Some(19.9));
        for i in 0..511u32 {
            // A single-sample spike at 8.7 bar, everything else well below it.
            let bar = if i == 173 { 8.7 } else { 3.0 };
            trace.push(i * 100, Some(bar), Some(i as f32 * 0.1), None);
        }
        assert_eq!(trace.bucket_ms(), 400);
        assert!(trace.pressure().len() <= WIDTH);
        assert_eq!(trace.peak_pressure(), Some(8.7));
        let on_curve = trace.pressure().iter().cloned().fold(0.0f32, f32::max);
        assert_eq!(on_curve, 8.7, "the peak was decimated off the curve");
    }

    /// Weight only climbs, so the last bucket must be the final weight however many times
    /// the trace folded.
    #[test]
    fn weight_survives_folding_as_its_final_value() {
        let mut trace = ShotTrace::new();
        trace.start(None);
        for i in 0..511u32 {
            trace.push(i * 100, Some(3.0), Some(i as f32 * 0.1), None);
        }
        let last = *trace.weight().last().unwrap();
        assert!((last - 51.0).abs() < 0.15, "final weight was {last}");
    }

    /// A phase that arrives already past saturation still gets a saturation band.
    #[test]
    fn a_skipped_saturation_is_still_bounded() {
        let mut trace = ShotTrace::new();
        trace.start(None);
        trace.push(0, Some(0.0), Some(0.0), Some(Phase::HeadspaceFill));
        trace.push(3_000, Some(4.0), Some(0.0), Some(Phase::PostFirstDrop));
        assert_eq!(trace.saturation_seconds(), Some(3.0));
        assert_eq!(trace.first_drop_seconds(), Some(3.0));
    }

    /// A gap in the status stream holds the last value rather than dropping to the floor,
    /// which would draw as a pressure release that never happened.
    #[test]
    fn a_gap_holds_the_last_value() {
        let mut trace = ShotTrace::new();
        trace.start(None);
        trace.push(0, Some(6.0), Some(1.0), None);
        trace.push(500, Some(6.5), Some(2.0), None);
        let p = trace.pressure();
        assert_eq!(p.len(), 6);
        assert_eq!(&p[1..5], &[6.0, 6.0, 6.0, 6.0]);
        assert_eq!(p[5], 6.5);
    }

    /// A falling pressure has to fall.
    ///
    /// It did not: the value carried into a newly opened bucket competed with the sample
    /// that opened it, and the maximum won, so every declining profile drew as a flat line
    /// at its peak. Nothing but the rendered curve showed it.
    #[test]
    fn a_declining_profile_declines() {
        let mut trace = ShotTrace::new();
        trace.start(None);
        for i in 0..=90u32 {
            // Up to 9 bar over three seconds, then down to 3 over six.
            let bar = if i <= 30 {
                i as f32 * 0.3
            } else {
                9.0 - (i - 30) as f32 * 0.1
            };
            trace.push(i * 100, Some(bar), Some(0.0), None);
        }
        let p = trace.pressure();
        assert_eq!(p[30], 9.0);
        assert!(
            *p.last().unwrap() < 3.5,
            "the curve ended at {:?}, having never come down",
            p.last()
        );
        // And monotonically down over the tail, which is what a declining profile looks
        // like.
        for pair in p[31..].windows(2) {
            assert!(pair[1] <= pair[0], "{pair:?} rose");
        }
    }

    /// `start` must not leave a bucket of the previous shot at the right-hand end.
    #[test]
    fn starting_clears_the_previous_shot() {
        let mut trace = ShotTrace::new();
        trace.start(Some(18.0));
        for i in 0..100u32 {
            trace.push(i * 100, Some(9.0), Some(30.0), Some(Phase::PostFirstDrop));
        }
        trace.start(Some(20.0));
        assert!(trace.is_empty());
        assert_eq!(trace.peak_pressure(), None);
        assert_eq!(trace.first_drop_seconds(), None);
        assert_eq!(trace.dose_g(), Some(20.0));
    }
}
