//! A proportional-integral-derivative (PID) controller.

use num_traits::{float::FloatCore};
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
pub enum PidError {
    LimitOutBound,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct PidTerm<T: FloatCore + Default> {
    pub positive_scale: T,
    pub negative_scale: T,
    pub limits: Limits<T>,
}

impl <T: FloatCore + Default> PidTerm<T> {
    pub fn new(scale: T, limits: Limits<T>) -> Self {
        PidTerm {
            positive_scale: scale,
            negative_scale: scale,
            limits,
        }
    }

    pub fn new_asymmetric(positive_scale: T, negative_scale: T, limits: Limits<T>) -> Self {
        PidTerm {
            positive_scale,
            negative_scale,
            limits,
        }
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct PidParameters<T: FloatCore + core::default::Default> {
    pub kp: PidTerm<T>,
    pub ki: PidTerm<T>,
    pub kd: PidTerm<T>,
}

#[derive(Copy, Clone, PartialEq, PartialOrd, Hash, Debug)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Limits<T: FloatCore + core::default::Default> {
    lower: T,
    upper: T,
}

impl<T: FloatCore + core::default::Default> Limits<T> {
    fn new() -> Self {
        Limits{lower: T::neg_infinity(), upper: T::infinity()}
    }

    pub fn new_with_limits(lower: T, upper: T) -> Result<Self, PidError> {
        let mut limits = Limits::new();

        limits.try_set_lower(lower)?;
        limits.try_set_upper(upper)?;

        Ok(limits)
    }

    fn clamp(&self, val: T) -> T {
        val.min(self.upper).max(self.lower)
    }

    pub fn set_limit(&mut self, val: T) -> &mut Self {
        self.lower = -val.abs();
        self.upper = val.abs();
        self
    }

    pub fn try_set_upper(&mut self, val: T) -> Result<&mut Self, PidError> {
        if self.lower <= val {
            self.upper = val;
            Ok(self)
        }
        else {
            Err(PidError::LimitOutBound)
        }
    }

    pub fn try_set_lower(&mut self, val: T) -> Result<&mut Self, PidError> {
        if self.upper >= val {
            self.lower = val;
            Ok(self)
        }
        else {
            Err(PidError::LimitOutBound)
        }
    }

    pub fn upper(&self) -> T {
        self.upper
    }

    pub fn lower(&self) -> T {
        self.lower
    }
}

impl<T: FloatCore + core::default::Default> Default for Limits<T> {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Copy, Clone, PartialEq, PartialOrd, Hash, Debug, Default)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct KPTerm<T: FloatCore + core::default::Default> {
    pub limits: Limits<T>,
    positive_scale: T,
    negative_scale: T,
}

impl<T:FloatCore + core::default::Default> KPTerm<T> {
    pub fn new() -> Self {
        KPTerm::default()
    }
    pub fn set_scale(&mut self, val: T) -> &mut Self {
        self.positive_scale = val;
        self.negative_scale = val;
        self
    }
    pub fn set_asymmetric_scale(&mut self, positive: T, negative: T) -> &mut Self {
        self.positive_scale = positive;
        self.negative_scale = negative;
        self
    }
    pub fn step(&self, offset: T) -> (T, T) {
        let scale = if offset >= T::zero() {
            self.positive_scale
        } else {
            self.negative_scale
        };

        (self.limits.clamp(scale * offset), scale)
    }
}

#[derive(Copy, Clone, PartialEq, PartialOrd, Hash, Debug, Default)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct KITerm<T: FloatCore + core::default::Default> {
    pub limits: Limits<T>,
    positive_scale: T,
    negative_scale: T,
    pub accumulate: T
}

impl<T:FloatCore + core::default::Default> KITerm<T> {
    pub fn new() -> Self {
        KITerm::default()
    }
    pub fn set_scale(&mut self, val: T) -> &mut Self {
        self.positive_scale = val;
        self.negative_scale = val;
        self
    }

    pub fn set_asymmetric_scale(&mut self, positive: T, negative: T) -> &mut Self {
        self.positive_scale = positive;
        self.negative_scale = negative;
        self
    }

    pub fn step(&mut self, offset: T, tdelta: T) -> (T, T) {
        let scale = if offset >= T::zero() {
            self.positive_scale
        } else {
            self.negative_scale
        };

        let i = self.limits.clamp(scale * offset * tdelta + self.accumulate);
        self.accumulate = i;
        (i, scale)
    }
}

#[derive(Copy, Clone, PartialEq, PartialOrd, Hash, Debug, Default)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct KDTerm<T: FloatCore + core::default::Default> {
    pub limits: Limits<T>,
    positive_scale: T,
    negative_scale: T,
    pub prev_measurement: T
}

impl<T:FloatCore + core::default::Default> KDTerm<T> {
    pub fn new() -> Self {
        KDTerm::default()
    }
    pub fn set_scale(&mut self, val: T) -> &mut Self {
        self.positive_scale = val;
        self.negative_scale = val;
        self
    }
    pub fn set_asymmetric_scale(&mut self, positive: T, negative: T) -> &mut Self {
        self.positive_scale = positive;
        self.negative_scale = negative;
        self
    }
    pub fn step(&mut self, offset: T, measurement: T, tdelta: T) -> (T, T) {
        let scale = if offset >= T::zero() {
            self.positive_scale
        } else {
            self.negative_scale
        };

        let d = self.limits.clamp(scale * (self.prev_measurement - measurement) / tdelta);
        self.prev_measurement = measurement;
        (d, scale)
    }
}

#[derive(Copy, Clone, PartialEq, PartialOrd, Hash, Debug, Default)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PidCtrl <T: FloatCore + core::default::Default> {
    pub kp: KPTerm<T>,
    pub ki: KITerm<T>,
    pub kd: KDTerm<T>,
    pub limits: Limits<T>,

    pub setpoint: T,
}

impl<T: FloatCore + core::default::Default> PidCtrl<T>
{
    pub fn new() -> Self {
        PidCtrl::default()
    }

    pub fn new_with_pid(p: T, i: T, d: T) -> Self {
        Self{
            kp: KPTerm{limits:Limits::new(), positive_scale: p, negative_scale: p},
            ki: KITerm{limits:Limits::new(), positive_scale: i, negative_scale: i, accumulate:T::zero()},
            kd: KDTerm{limits:Limits::new(), positive_scale: d, negative_scale: d, prev_measurement:T::zero()},
            limits: Limits::new(), setpoint: T::zero(),
        }
    }

    pub fn init(&mut self, setpoint: T, prev_measurement: T) -> &mut Self {
        self.setpoint = setpoint;
        self.kd.prev_measurement = prev_measurement;
        self
    }

    pub fn step(&mut self, input: PidIn<T>) -> PidOut<T> {
        let offset = self.setpoint - input.measurement;
        let (p, acting_kp) = self.kp.step(offset);
        let (i, acting_ki) = self.ki.step(offset, input.tdelta);
        let (d, acting_kd) = self.kd.step(offset, input.measurement, input.tdelta);
        PidOut::new(p, i, d, self.limits.clamp(p + i + d), acting_kp, acting_ki, acting_kd)
    }
    
    pub fn reset(&mut self) {
        self.ki.accumulate = T::zero();
        self.kd.prev_measurement = T::zero();
    }

    /// Infer and set the integral term based on a desired output and current measurement.
    /// This is useful for "bumpless transfer" when switching from open-loop to closed-loop control.
    ///
    /// Given a target output (e.g., current duty cycle) and the current measurement,
    /// this calculates what the integral term should be to produce that output,
    /// assuming the derivative term is zero.
    ///
    /// The calculation: I = target_output - P_term
    /// Also sets D term's previous measurement to zero the derivative.
    pub fn infer_and_set_integral(&mut self, target_output: T, current_measurement: T) -> &mut Self {
        let offset = self.setpoint - current_measurement;
        let (p_term, _) = self.kp.step(offset);
        let inferred_integral = target_output - p_term;
        self.ki.accumulate = inferred_integral;
        self.kd.prev_measurement = current_measurement;
        self
    }

    /// Relax the integral toward the value that would have produced `selected`.
    ///
    /// External reset feedback, for a min-select override: two controllers run against the
    /// same actuator, a selector takes one of their outputs, and the loser must be anchored
    /// to the selected value rather than left to integrate against an error it is not
    /// driving. Without it the deselected controller winds up, and takes over with a step
    /// when the selector next picks it -- the same failure `pump_transfer` describes for
    /// open-to-closed-loop transfer, in a place that recurs every iteration instead of once.
    ///
    /// # The loser's proportional term has to survive
    ///
    /// **This used to assign `selected - p - d` outright, and that was a bug.** Snapping the
    /// *whole output* to `selected` is more than anchoring: it erases the loser's proportional
    /// term, so a loop with real headroom proposes exactly what the loop in control proposes
    /// instead of `kp * error` above it. Two consequences, both observed on a real shot
    /// (`v2hfh1e26wta5jvehdwfaxp8g8`, pressure held 0.6 bar under a 7 bar cap for 38 seconds):
    ///
    /// - **The selector loses its input.** Which loop wins stops being a question about
    ///   whether the limit is near binding and becomes one about ordinary signal variation,
    ///   because the two proposals differ only by their `p` terms' *change* since last tick.
    /// - **Neither loop can advance.** Each is pinned to the pair's last agreed output every
    ///   iteration, so `min` of two mutually-slaved proposals ratchets downward. The main
    ///   loop in that shot accumulated +8.6 duty counts where its gains predicted +36.1.
    ///
    /// So the integral *relaxes* toward that target over the loop's own integral time
    /// `Ti = kp/ki`, rather than snapping to it. The loser's own integration continues
    /// underneath -- the caller steps both loops before selecting -- and the two compose to
    /// `i -> selected - d`, leaving the proposal at `selected + p`: anchored, but still
    /// carrying the offset that says how much room this loop thinks it has. It therefore
    /// wins only as its own error approaches zero, which is the "wide, soft region of action"
    /// a min-select override is supposed to have, with its width set by `kp`.
    ///
    /// `alpha == 1` recovers the old assignment exactly, so the snap is the degenerate case
    /// of this rule: it is what a loop with no proportional term (`Ti == 0`) still gets.
    ///
    /// # Why this is not [`Self::infer_and_set_integral`]
    ///
    /// That one is a *one-shot transfer* primitive and is wrong as a per-tick tracker in two
    /// ways. It assumes `D` is zero, which is only true at a handover; and it zeroes the
    /// derivative by assigning `kd.prev_measurement`, which called every tick would suppress
    /// the deselected controller's D term entirely. This keeps the D contribution out of the
    /// target and leaves `prev_measurement` alone, so the loser keeps a live derivative and
    /// is ready to take over.
    ///
    /// Takes the [`PidOut`] just produced by [`Self::step`] rather than recomputing: `p`, `d`
    /// and the gains that were actually in force are already known, and re-stepping `kd`
    /// would advance its state a second time. Reading the gains off `last` rather than off
    /// `self` is what makes an asymmetric `kp` track at the rate of the direction it is
    /// acting in -- the pump's pressure loop is 20.4 rising and 30.6 falling, so its `Ti` is
    /// 2.0 s one way and 3.0 s the other.
    ///
    /// # The clamp still applies
    ///
    /// This writes `ki.accumulate` directly, but [`Self::step`] clamps it to `ki.limits` on
    /// the next iteration. A controller tracking an output beyond that bound cannot reach it
    /// and will take over low. That is a tuning question about `ki.limits`, not a bug here --
    /// see `pump_transfer`'s note on the same ceiling.
    pub fn track_to(&mut self, selected: T, last: &PidOut<T>, tdelta: T) -> &mut Self {
        // dt/Ti, where Ti = kp/ki. Clamped to 1 so a long tick degrades to the snap rather
        // than overshooting the target and oscillating around it.
        let alpha = if last.acting_kp > T::zero() {
            let alpha = last.acting_ki * tdelta / last.acting_kp;
            if alpha > T::one() { T::one() } else { alpha }
        } else {
            // No proportional term is Ti == 0: there is no offset to preserve, so snap.
            T::one()
        };

        let target = selected - last.p - last.d;
        self.ki.accumulate = self.ki.accumulate + (target - self.ki.accumulate) * alpha;
        self
    }

    pub fn set_parameters(&mut self, parameters: PidParameters<T>) -> &mut Self {
        self.kp.set_asymmetric_scale(parameters.kp.positive_scale, parameters.kp.negative_scale);
        self.kp.limits = parameters.kp.limits;
        self.ki.set_asymmetric_scale(parameters.ki.positive_scale, parameters.ki.negative_scale);
        self.ki.limits = parameters.ki.limits;
        self.kd.set_asymmetric_scale(parameters.kd.positive_scale, parameters.kd.negative_scale);
        self.kd.limits = parameters.kd.limits;
        self
    }
}

#[derive(Copy, Clone, PartialEq, PartialOrd, Hash, Debug, Default)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PidIn <T: FloatCore + core::default::Default> {
    measurement: T,
    tdelta: T,
}

impl<T: FloatCore + core::default::Default> PidIn<T> {
    pub fn new(measurement:T, tdelta:T) -> Self {
        let tdelta_clamped = tdelta.min(T::infinity()).max(T::epsilon());
        PidIn{measurement, tdelta: tdelta_clamped}
    }
}

#[derive(Copy, Clone, PartialEq, PartialOrd, Hash, Debug, Default)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PidOut<T: FloatCore + core::default::Default> {
    pub p: T,
    pub i: T,
    pub d: T,
    pub out: T,
    pub acting_kp: T,
    pub acting_ki: T,
    pub acting_kd: T,
}

impl<T: FloatCore + core::default::Default> PidOut<T> {
    pub fn new(p: T, i: T, d: T, out: T, acting_kp: T, acting_ki: T, acting_kd: T) -> Self {
        Self { p, i, d, out, acting_kp, acting_ki, acting_kd }
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn limits_error() {
        let mut pid = super::PidCtrl::new_with_pid(3.0, 2.0, 1.0);
        pid.kp.limits.try_set_lower(10.0).unwrap();
        assert_eq!(super::PidError::LimitOutBound, pid.kp.limits.try_set_upper(5.0).unwrap_err());
    }

    #[test]
    fn kp() {
        let kp = 0.2;
        let measurement = 0.0;
        let setpoint = 1.0;

        let mut pid = super::PidCtrl::default();
        pid.init(setpoint, 0.0);
        pid.kp.set_scale(kp);

        let kpterm = kp * (setpoint - measurement);

        let inp = super::PidIn::new(measurement, 1.0);
        assert_eq!(pid.step(inp), super::PidOut::new(kpterm, 0.0, 0.0, kpterm, kp, 0.0, 0.0));
    }

    #[test]
    fn ki() {
        let ki = 1.0;
        let measurement = 0.0;
        let setpoint = 1.0;
        let td = 1.0;

        let mut pid = super::PidCtrl::default();
        pid.init(setpoint, 0.0);
        pid.ki.set_scale(ki);

        let mut kiterm = 0.0;

        kiterm += ki * (setpoint - measurement) * td;
        let inp = super::PidIn::new(measurement, td);
        assert_eq!(pid.step(inp), super::PidOut::new(0.0, kiterm, 0.0, kiterm, 0.0, ki, 0.0));

        kiterm += ki * (setpoint - measurement) * td;
        let inp = super::PidIn::new(measurement, td);
        assert_eq!(pid.step(inp), super::PidOut::new(0.0, kiterm, 0.0, kiterm, 0.0, ki, 0.0));
    }

    #[test]
    fn kd() {
        let kd = 1.0;
        let measurement = 0.0;
        let setpoint = 1.0;
        let td = 1.0;

        let mut prev = 0.0;

        let mut pid = super::PidCtrl::default();
        pid.init(setpoint, prev);
        pid.kd.set_scale(kd);

        let mut kdterm = kd * (measurement - prev) / td;
        prev = measurement;
        let inp = super::PidIn::new(measurement, td);
        assert_eq!(pid.step(inp), super::PidOut::new(0.0, 0.0, kdterm, kdterm, 0.0, 0.0, kd));

        kdterm = kd * (measurement - prev) / td;
        let inp = super::PidIn::new(measurement, td);
        assert_eq!(pid.step(inp), super::PidOut::new(0.0, 0.0, kdterm, kdterm, 0.0, 0.0, kd));
    }

    /// A tracked loop is anchored to the selected output, not flattened onto it.
    ///
    /// This is the property the min-select override in `variegated-controller-lib`'s
    /// `pump_limit` is built on: a deselected loop still has to say how much room it thinks
    /// it has, and it says so by proposing `kp * error` *above* the output that won. A
    /// version of `track_to` that assigns `selected - p - d` outright converges here to
    /// `selected` itself, which is a loop that has been silenced rather than held -- see that
    /// method's documentation for what it cost on a real shot.
    ///
    /// The fixture is the pump's pressure limit loop with 0.6 bar of headroom under its cap,
    /// which is the case that went wrong: comfortably below the limit, and so obliged to stay
    /// out of the main loop's way.
    #[test]
    fn a_tracked_loop_still_proposes_its_own_proportional_offset() {
        const SELECTED: f32 = 100.0;
        const CAP: f32 = 7.0;
        const PRESSURE: f32 = 6.4;
        const TDELTA: f32 = 107.0;

        let mut pid = super::PidCtrl::<f32>::new_with_pid(20.4, 0.0102, 0.0);
        pid.setpoint = CAP;
        pid.infer_and_set_integral(SELECTED, PRESSURE);

        // Long enough to converge: Ti is kp/ki = 2000 ms, so ~19 iterations per time
        // constant at this tick.
        let mut out = pid.step(super::PidIn::new(PRESSURE, TDELTA));
        for _ in 0..100 {
            pid.track_to(SELECTED, &out, TDELTA);
            out = pid.step(super::PidIn::new(PRESSURE, TDELTA));
        }

        let offset = 20.4 * (CAP - PRESSURE);
        assert!(
            (out.out - (SELECTED + offset)).abs() < 0.1,
            "a loop {} bar under its cap settled at {}, not the {} that leaves it \
             {offset} of headroom above the selected output",
            CAP - PRESSURE,
            out.out,
            SELECTED + offset
        );
    }

    /// The degenerate case, stated so it cannot be broken silently: with no proportional term
    /// there is no offset to preserve, `Ti` is zero, and tracking is the outright assignment
    /// it always used to be.
    #[test]
    fn a_loop_with_no_proportional_term_still_snaps() {
        let mut pid = super::PidCtrl::<f32>::new_with_pid(0.0, 1.0, 0.0);
        pid.setpoint = 9.0;

        let out = pid.step(super::PidIn::new(0.0, 1.0));
        pid.track_to(40.0, &out, 1.0);

        assert!(
            (pid.ki.accumulate - 40.0).abs() < 0.001,
            "expected the integral to land on 40 in one call, got {}",
            pid.ki.accumulate
        );
    }
}