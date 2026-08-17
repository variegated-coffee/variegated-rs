//! One editable quantity.

/// A value with a range and a step, which is the whole of what an editor needs to know.
///
/// It replaces three separate implementations in the Silvia firmware that agreed with each
/// other nowhere: one with per-mode literals, one with a fixed 0.5 step and no ceiling at
/// all, and one whose lower bound was the magic number -100 for *every* quantity it edited,
/// including the brew setpoint.
///
/// `min` must not exceed `max`; `f32::INFINITY` is a legitimate `max` for a quantity whose
/// ceiling nobody has defined, and is used exactly that way for the Silvia's PID components.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Adjustable {
    value: f32,
    min: f32,
    max: f32,
    step: f32,
}

impl Adjustable {
    /// Clamps `value` into range, because an out-of-range starting value is how a config
    /// stored by an older firmware arrives.
    ///
    /// A NaN becomes `min`: NaN compares false against both bounds, so it would otherwise
    /// survive every clamp and leave an editor with no way back to a usable number.
    pub fn new(value: f32, min: f32, max: f32, step: f32) -> Self {
        let value = if value.is_nan() { min } else { clamp(value, min, max) };
        Self { value, min, max, step }
    }

    /// The current value.
    pub const fn value(&self) -> f32 {
        self.value
    }

    /// The lower bound.
    pub const fn min(&self) -> f32 {
        self.min
    }

    /// The upper bound.
    pub const fn max(&self) -> f32 {
        self.max
    }

    /// One step.
    pub const fn step(&self) -> f32 {
        self.step
    }

    /// One step up, stopping at `max`.
    pub fn increase(&mut self) {
        self.value = clamp(self.value + self.step, self.min, self.max);
    }

    /// One step down, stopping at `min`.
    pub fn decrease(&mut self) {
        self.value = clamp(self.value - self.step, self.min, self.max);
    }
}

/// `f32::clamp` panics when `min > max`; this saturates instead.
///
/// A menu that panics on a bad bound takes the machine down, and the bounds come from a
/// caller's table rather than from anything this crate can check at compile time.
fn clamp(value: f32, min: f32, max: f32) -> f32 {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stops_at_both_bounds() {
        let mut a = Adjustable::new(93.0, 0.0, 100.0, 0.5);
        for _ in 0..1000 { a.increase(); }
        assert_eq!(a.value(), 100.0);
        for _ in 0..1000 { a.decrease(); }
        assert_eq!(a.value(), 0.0, "the Silvia clamped every value at -100, including a brew setpoint");
    }

    #[test]
    fn steps_are_exact() {
        let mut a = Adjustable::new(10.0, 0.0, 100.0, 5.0);
        a.increase();
        assert_eq!(a.value(), 15.0);
        a.decrease();
        a.decrease();
        assert_eq!(a.value(), 5.0);
    }

    #[test]
    fn clamps_an_out_of_range_starting_value() {
        // How a config stored by an older firmware arrives.
        assert_eq!(Adjustable::new(250.0, 0.0, 100.0, 1.0).value(), 100.0);
        assert_eq!(Adjustable::new(-40.0, 0.0, 100.0, 1.0).value(), 0.0);
    }

    #[test]
    fn a_nan_starting_value_becomes_the_minimum() {
        // Without this a NaN survives every clamp -- NaN compares false against both
        // bounds -- and the editor becomes unusable with no way to get back out.
        let a = Adjustable::new(f32::NAN, 1.0, 9.0, 0.5);
        assert_eq!(a.value(), 1.0);
    }

    #[test]
    fn an_infinite_upper_bound_is_usable() {
        // The Silvia's PID components have no ceiling anyone has defined, and this change
        // must preserve that rather than invent one.
        let mut a = Adjustable::new(0.0, -100.0, f32::INFINITY, 0.1);
        for _ in 0..100 { a.increase(); }
        assert!(a.value() > 9.0 && a.value().is_finite());
        for _ in 0..10_000 { a.decrease(); }
        assert_eq!(a.value(), -100.0);
    }
}
