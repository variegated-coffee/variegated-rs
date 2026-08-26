//! The two water interlocks, which were identical on both machines and checked by neither.
//!
//! Both are pure predicates over configuration and a reading, which is what lets them be
//! tested here rather than argued about in a comment. The third -- "is the tank empty" --
//! needs a `Tank` from `variegated-hal` and so lives in [`super::hardware`].

use variegated_controller_types::{BoilerConfiguration, MachineConfiguration, WaterLevelType};

/// Is a boiler's water level safe to heat?
///
/// **A `None` reading with the feature enabled is unsafe, not unknown.** If a minimum is
/// configured then a level sensor is fitted, so no reading means the sensor is not answering --
/// and a dry element is destroyed in seconds. Treating silence as permission is the failure
/// this exists to prevent.
///
/// No minimum configured means no sensor is fitted and the feature is off, which is a
/// different thing from a sensor that has gone quiet.
pub fn is_boiler_level_safe(
    level: Option<WaterLevelType>,
    boiler_config: &BoilerConfiguration,
) -> bool {
    let Some(minimum) = boiler_config.minimum_safe_level else {
        return true;
    };

    match level {
        Some(level) => level >= minimum,
        None => false,
    }
}

/// Should a new water operation be refused?
///
/// The policy, in order: the feature can be off; a tank that is not empty never blocks; and an
/// empty tank blocks *unless* a routine is already running and the machine is configured to let
/// one continue.
///
/// **The routine case is the interesting one.** Stopping mid-shot on a tank that has just read
/// empty can be worse than finishing it -- the puck is already wet and the pump is already
/// running -- so `allow_continue_on_empty_tank` exists to let a routine finish what it started
/// while still refusing to *start* anything new. A standalone operation with an empty tank is
/// always refused.
pub fn should_block_water_operation(
    machine_config: &MachineConfiguration,
    tank_empty: bool,
    routine_running: bool,
) -> bool {
    if !machine_config.prevent_start_on_empty_tank {
        return false;
    }
    if !tank_empty {
        return false;
    }
    if routine_running {
        return !machine_config.allow_continue_on_empty_tank;
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    fn boiler_with_minimum(minimum: Option<WaterLevelType>) -> BoilerConfiguration {
        BoilerConfiguration { minimum_safe_level: minimum, ..BoilerConfiguration::default() }
    }

    /// With no minimum configured the feature is off and any reading is fine.
    #[test]
    fn no_configured_minimum_allows_heating() {
        let config = boiler_with_minimum(None);
        assert!(is_boiler_level_safe(None, &config));
        assert!(is_boiler_level_safe(Some(0), &config));
    }

    /// A reading at or above the minimum is safe; below is not.
    #[test]
    fn a_reading_is_checked_against_the_minimum() {
        let config = boiler_with_minimum(Some(30));
        assert!(is_boiler_level_safe(Some(30), &config), "at the minimum is safe");
        assert!(is_boiler_level_safe(Some(80), &config));
        assert!(!is_boiler_level_safe(Some(29), &config));
    }

    /// **A silent sensor blocks heating.**
    ///
    /// The property worth having a test for: a configured minimum means a sensor is fitted, so
    /// no reading means it is not answering rather than that there is nothing to worry about.
    /// A dry element is destroyed in seconds, so the safe reading of silence is "empty".
    #[test]
    fn a_missing_reading_blocks_heating_when_the_feature_is_on() {
        assert!(!is_boiler_level_safe(None, &boiler_with_minimum(Some(30))));
    }

    fn machine(prevent: bool, allow_continue: bool) -> MachineConfiguration {
        MachineConfiguration {
            prevent_start_on_empty_tank: prevent,
            allow_continue_on_empty_tank: allow_continue,
            ..MachineConfiguration::default()
        }
    }

    /// With the feature off, nothing is ever blocked.
    #[test]
    fn the_feature_can_be_off() {
        let config = machine(false, false);
        assert!(!should_block_water_operation(&config, true, false));
        assert!(!should_block_water_operation(&config, true, true));
    }

    /// A tank that is not empty never blocks.
    #[test]
    fn a_full_tank_never_blocks() {
        assert!(!should_block_water_operation(&machine(true, false), false, false));
        assert!(!should_block_water_operation(&machine(true, false), false, true));
    }

    /// An empty tank blocks a standalone operation whatever the continue policy says.
    #[test]
    fn an_empty_tank_blocks_a_standalone_operation() {
        assert!(should_block_water_operation(&machine(true, true), true, false));
        assert!(should_block_water_operation(&machine(true, false), true, false));
    }

    /// A running routine is allowed to finish, or not, according to the policy.
    ///
    /// This is the whole reason the predicate takes `routine_running` at all: stopping mid-shot
    /// on a tank that has just read empty can be worse than finishing, because the puck is
    /// already wet and the pump is already running.
    #[test]
    fn a_running_routine_follows_the_continue_policy() {
        assert!(
            !should_block_water_operation(&machine(true, true), true, true),
            "allow_continue should let a routine finish"
        );
        assert!(
            should_block_water_operation(&machine(true, false), true, true),
            "without allow_continue the routine is stopped"
        );
    }
}
