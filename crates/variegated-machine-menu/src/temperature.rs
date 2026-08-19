//! Editing a boiler setpoint.

use variegated_menu::Adjustable;

/// Half a degree per press.
///
/// Finer than a user can taste and coarse enough to cross a useful range in a few seconds.
/// Both machines already used it; this is where the number now lives.
pub const BOILER_TEMPERATURE_STEP: f32 = 0.5;

/// An editor for a boiler setpoint, seeded at `current` and stopping at `max`.
///
/// The ceiling is an argument rather than a constant because it is legitimately per-machine:
/// the Silvia's brew boiler stops at `MAX_BREW_TEMPERATURE`, the GS3's at whatever its
/// configuration's `max_temperature` says. Pass the machine's own number rather than a
/// literal -- a setpoint above the interlock can only produce an element that runs to the
/// limit and shuts off, so a wrong ceiling here is a control problem, not a display one.
///
/// The floor is zero on both. A negative brew temperature is not a thing, and `Adjustable`
/// saturates rather than panicking if a caller ever passes a `max` below it.
pub fn boiler_temperature_adjustable(current: f32, max: f32) -> Adjustable {
    Adjustable::new(current, 0.0, max, BOILER_TEMPERATURE_STEP)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_stops_at_the_machines_ceiling() {
        let mut a = boiler_temperature_adjustable(93.0, 105.0);
        for _ in 0..1000 {
            a.increase();
        }
        assert_eq!(a.value(), 105.0);
    }

    #[test]
    fn it_stops_at_zero_rather_than_at_the_silvias_old_minus_one_hundred() {
        let mut a = boiler_temperature_adjustable(93.0, 105.0);
        for _ in 0..1000 {
            a.decrease();
        }
        assert_eq!(a.value(), 0.0);
    }

    #[test]
    fn a_setpoint_stored_above_a_lowered_ceiling_is_pulled_back_in() {
        assert_eq!(boiler_temperature_adjustable(120.0, 105.0).value(), 105.0);
    }

    #[test]
    fn steps_are_half_a_degree() {
        let mut a = boiler_temperature_adjustable(93.0, 105.0);
        a.increase();
        assert_eq!(a.value(), 93.5);
        a.decrease();
        a.decrease();
        assert_eq!(a.value(), 92.5);
    }
}
