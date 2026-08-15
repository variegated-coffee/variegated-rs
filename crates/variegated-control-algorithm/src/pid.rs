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
        assert_eq!(pid.step(inp), super::PidOut::new(kpterm, 0.0, 0.0, kpterm));
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
        assert_eq!(pid.step(inp), super::PidOut::new(0.0, kiterm, 0.0, kiterm));

        kiterm += ki * (setpoint - measurement) * td;
        let inp = super::PidIn::new(measurement, td);
        assert_eq!(pid.step(inp), super::PidOut::new(0.0, kiterm, 0.0, kiterm));
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
        assert_eq!(pid.step(inp), super::PidOut::new(0.0, 0.0, kdterm, kdterm));

        kdterm = kd * (measurement - prev) / td;
        let inp = super::PidIn::new(measurement, td);
        assert_eq!(pid.step(inp), super::PidOut::new(0.0, 0.0, kdterm, kdterm));
    }
}