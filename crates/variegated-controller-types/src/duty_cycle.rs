//! The two duty cycle scales, and the conversions between them.
//!
//! Duty cycle is the one quantity in this tree that exists at two resolutions. Everything a
//! person authors, edits or sends over the machine API is a percentage -- [`DutyCycle`],
//! 0-100 -- because percent is what an operator reasons in. Everything between a setpoint
//! and the pump is [`HexadecimalDutyCycle`], 0-255, because 100 steps is too coarse for the
//! pump PID: its smallest possible correction would be a full percent of pump output.
//!
//! These are newtypes rather than the `u8` aliases they replace precisely because the two
//! scales are otherwise indistinguishable. A percentage reaching a pump is not a slightly
//! wrong number, it is a pump running at 39% of what was asked for, and nothing about it
//! looks wrong in a log. The compiler catching that at the boundary is the whole reason the
//! split exists, so resist adding an `impl From<u8>` for either of them -- an explicit
//! constructor at each site is the point.
//!
//! The heater path stays on [`DutyCycle`]. Its actuation is a 3-second soft-PWM cycle where
//! one percent is 30 ms, so finer resolution would be meaningless there.

/// A duty cycle as a percentage, 0-100.
///
/// [`DutyCycle::new`] clamps, but this type is also deserialized straight off a wire, so a
/// value above 100 is representable and every conversion here defends against one rather
/// than assuming the invariant holds.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct DutyCycle(u8);

/// A duty cycle over the full range of a byte, 0-255.
///
/// This is what a `Pump` is driven with (the trait lives in `variegated-hal`), and what the
/// pump PID and the evaluated duty curves produce. Unlike [`DutyCycle`] every representable
/// value is valid, so there is nothing here to clamp.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct HexadecimalDutyCycle(u8);

impl DutyCycle {
    /// Fully off.
    pub const OFF: Self = Self(0);
    /// Fully on, 100%.
    pub const FULL: Self = Self(100);

    /// A percentage, clamped into range.
    pub const fn new(percent: u8) -> Self {
        Self(if percent > 100 { 100 } else { percent })
    }

    /// The percentage, 0-100.
    pub const fn value(self) -> u8 {
        // Not `self.0` directly: a value deserialized off a wire has not been through
        // `new`, and every caller here is about to do arithmetic that assumes the bound.
        if self.0 > 100 { 100 } else { self.0 }
    }

    /// A percentage from a float, **rounded rather than truncated**, and saturating.
    ///
    /// `50.5` becomes 51, not 50 -- truncating would make an editor that steps in halves
    /// read a percent low for half its range. Values outside 0-100 saturate, and NaN
    /// becomes zero: a duty cycle is the wrong place to propagate a NaN, and off is the
    /// safe reading of a broken one.
    ///
    /// `+ 0.5` rather than `libm::roundf` because this crate does not link libm, and the
    /// two agree over the non-negative range that survives the guard above.
    pub fn from_f32(value: f32) -> Self {
        if value <= 0.0 {
            Self(0)
        } else if value >= 100.0 {
            Self(100)
        } else {
            // NaN fails both comparisons above and lands here, where the `as` cast
            // saturates it to zero.
            Self((value + 0.5) as u8)
        }
    }
}

impl HexadecimalDutyCycle {
    /// Fully off.
    pub const OFF: Self = Self(0);
    /// Fully on, the top of the range.
    pub const FULL: Self = Self(255);

    /// A raw 0-255 value.
    pub const fn new(value: u8) -> Self {
        Self(value)
    }

    /// The raw value, 0-255.
    pub const fn value(self) -> u8 {
        self.0
    }

    /// A raw value from a float, rounded rather than truncated, and saturating.
    ///
    /// This is the narrowing the pump PID's `f32` output goes through. It saturates rather
    /// than casting bare, which is what closes the older hazard where an unclamped
    /// `pid_out.out as u8` could produce a "percentage" of 254.
    pub fn from_f32(value: f32) -> Self {
        if value <= 0.0 {
            Self(0)
        } else if value >= 255.0 {
            Self(255)
        } else {
            Self((value + 0.5) as u8)
        }
    }
}

/// Percent to raw. 100% is 255, not 256: full scale means fully on.
impl From<DutyCycle> for HexadecimalDutyCycle {
    fn from(percent: DutyCycle) -> Self {
        let pct = percent.value() as u16;
        // Round half up. 20% is exactly 51, 50% is 128, 100% is 255.
        Self(((pct * 255 + 50) / 100) as u8)
    }
}

/// Raw to percent. Lossy in this direction -- 255 values do not divide into 101.
impl From<HexadecimalDutyCycle> for DutyCycle {
    fn from(raw: HexadecimalDutyCycle) -> Self {
        let hex = raw.value() as u16;
        // Round half up; 127 rather than 128 because half of 255 is 127.5.
        Self(((hex * 100 + 127) / 255) as u8)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn percentages_are_clamped_on_construction() {
        assert_eq!(DutyCycle::new(200).value(), 100);
        assert_eq!(DutyCycle::new(100).value(), 100);
        assert_eq!(DutyCycle::new(0).value(), 0);
    }

    #[test]
    fn duty_cycles_round_to_nearest_rather_than_down() {
        assert_eq!(DutyCycle::from_f32(50.5).value(), 51);
        assert_eq!(DutyCycle::from_f32(50.4).value(), 50);
        assert_eq!(HexadecimalDutyCycle::from_f32(127.5).value(), 128);
        assert_eq!(HexadecimalDutyCycle::from_f32(127.4).value(), 127);
    }

    #[test]
    fn duty_cycles_outside_the_range_saturate() {
        assert_eq!(DutyCycle::from_f32(-5.0).value(), 0);
        assert_eq!(DutyCycle::from_f32(99.999_99).value(), 100);
        assert_eq!(DutyCycle::from_f32(1000.0).value(), 100);
        assert_eq!(HexadecimalDutyCycle::from_f32(-5.0).value(), 0);
        assert_eq!(HexadecimalDutyCycle::from_f32(1000.0).value(), 255);
    }

    #[test]
    fn a_nan_duty_cycle_is_off() {
        assert_eq!(DutyCycle::from_f32(f32::NAN).value(), 0);
        assert_eq!(HexadecimalDutyCycle::from_f32(f32::NAN).value(), 0);
    }

    #[test]
    fn the_anchors_convert_exactly() {
        assert_eq!(HexadecimalDutyCycle::from(DutyCycle::new(0)).value(), 0);
        assert_eq!(HexadecimalDutyCycle::from(DutyCycle::new(20)).value(), 51);
        assert_eq!(HexadecimalDutyCycle::from(DutyCycle::new(50)).value(), 128);
        assert_eq!(HexadecimalDutyCycle::from(DutyCycle::new(100)).value(), 255);
    }

    #[test]
    fn an_out_of_range_percentage_cannot_overflow_the_conversion() {
        // Not reachable through `new`, but a wire value is not.
        let smuggled: DutyCycle = DutyCycle(200);
        assert_eq!(HexadecimalDutyCycle::from(smuggled).value(), 255);
    }

    #[test]
    fn every_percentage_survives_a_round_trip() {
        for pct in 0..=100u8 {
            let there = HexadecimalDutyCycle::from(DutyCycle::new(pct));
            let back = DutyCycle::from(there);
            assert_eq!(back.value(), pct, "{pct}% did not survive the round trip");
        }
    }

    #[test]
    fn a_raw_value_generally_does_not_survive_a_round_trip() {
        // The lossy direction, stated as a fact rather than discovered later: 255 values
        // do not divide into 101, so only the raw values that are exactly some percent
        // come back unchanged.
        let survivors = (0..=255u8)
            .filter(|&hex| {
                let there = DutyCycle::from(HexadecimalDutyCycle::new(hex));
                HexadecimalDutyCycle::from(there).value() == hex
            })
            .count();
        assert_eq!(survivors, 101);
    }
}
