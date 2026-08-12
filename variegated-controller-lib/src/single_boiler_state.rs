//! Which mode a single-boiler machine moves to when a boiler is enabled or disabled.
//!
//! A single-boiler machine has one heating element and presents it as two: boiler 0 is the
//! brew boiler, boiler 1 the *virtual* steam boiler. Enabling and disabling those two is how
//! the machine changes mode, and this is the whole table.
//!
//! Split out of `single_boiler_single_group` — which is behind the `hardware` feature and so
//! cannot be host-tested — because the table is pure and the interesting property is one a
//! test can state: **every mode has a way out**. It did not, until 2026-08-12: nothing
//! transitioned out of [`PowerSave`], so a machine that entered it stayed there until
//! reboot, and the steam switch, which requires [`BrewModeIdle`], could never work again.
//!
//! [`PowerSave`]: SingleBoilerSingleGroupControllerState::PowerSave
//! [`BrewModeIdle`]: SingleBoilerSingleGroupControllerState::BrewModeIdle

use variegated_controller_types::{BoilerIndex, SingleBoilerSingleGroupControllerState as State};

/// The brew boiler. The real one.
pub const BREW_BOILER: BoilerIndex = 0;
/// The steam boiler, which does not physically exist: enabling it puts the single element
/// under `steam_boiler_control_state` instead.
pub const STEAM_BOILER: BoilerIndex = 1;

/// Where an `EnableBoiler`/`DisableBoiler` moves the machine, or `None` to refuse.
///
/// Refusals are not failures. Switching to steam mid-shot is refused because a machine
/// should not do that, and the caller logs it rather than acting.
pub fn boiler_mode_transition(state: State, enable: bool, boiler: BoilerIndex) -> Option<State> {
    match (state, enable, boiler) {
        // Brew <-> steam, by either spelling. Turning the steam boiler off and turning the
        // brew boiler on are the same request, and a physical steam switch sends the first
        // while a UI is more likely to send the second.
        (State::BrewModeIdle, true, STEAM_BOILER) => Some(State::SteamModeIdle),
        (State::SteamModeIdle, false, STEAM_BOILER) => Some(State::BrewModeIdle),
        (State::SteamModeIdle, true, BREW_BOILER) => Some(State::BrewModeIdle),

        // Power save, and the way back out of it. The second of these was missing.
        (State::BrewModeIdle, false, BREW_BOILER) => Some(State::PowerSave),
        (State::PowerSave, true, BREW_BOILER) => Some(State::BrewModeIdle),

        // Everything else, which is mostly `Brewing` and `PumpingToWaterTap`: the machine
        // is doing something, and changing mode underneath it is not a request worth
        // honouring.
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ALL_STATES: [State; 5] = [
        State::BrewModeIdle,
        State::SteamModeIdle,
        State::Brewing,
        State::PumpingToWaterTap,
        State::PowerSave,
    ];

    /// Every command this table accepts, as `(enable, boiler)`.
    const ALL_COMMANDS: [(bool, BoilerIndex); 4] = [
        (true, BREW_BOILER),
        (true, STEAM_BOILER),
        (false, BREW_BOILER),
        (false, STEAM_BOILER),
    ];

    /// The property the missing arm violated: no mode may be a dead end.
    ///
    /// This is the test worth having. `PowerSave` was reachable and had no exit, so a
    /// machine that entered it was stuck until reboot -- and because nothing on either
    /// board's UI sends these commands, it went unnoticed until the steam switch was
    /// wired up and found it could not work after a power save.
    #[test]
    fn every_reachable_mode_has_a_way_out() {
        // The idle modes are the ones this table governs. `Brewing` and `PumpingToWaterTap`
        // are left by finishing the brew, not by a boiler command, so they are excluded --
        // deliberately and by name, rather than by the test quietly passing over them.
        for state in [State::BrewModeIdle, State::SteamModeIdle, State::PowerSave] {
            let exits = ALL_COMMANDS
                .iter()
                .filter_map(|&(enable, boiler)| boiler_mode_transition(state, enable, boiler))
                .filter(|&next| next != state)
                .count();

            assert!(exits > 0, "{state:?} is a dead end: no boiler command leaves it");
        }
    }

    #[test]
    fn brew_and_steam_are_mutually_reachable() {
        assert_eq!(
            boiler_mode_transition(State::BrewModeIdle, true, STEAM_BOILER),
            Some(State::SteamModeIdle)
        );
        // Both spellings of "go back to brew", because a steam switch turning off sends one
        // and a UI is more likely to send the other.
        assert_eq!(
            boiler_mode_transition(State::SteamModeIdle, false, STEAM_BOILER),
            Some(State::BrewModeIdle)
        );
        assert_eq!(
            boiler_mode_transition(State::SteamModeIdle, true, BREW_BOILER),
            Some(State::BrewModeIdle)
        );
    }

    #[test]
    fn power_save_round_trips() {
        let saved = boiler_mode_transition(State::BrewModeIdle, false, BREW_BOILER);
        assert_eq!(saved, Some(State::PowerSave));
        assert_eq!(
            boiler_mode_transition(State::PowerSave, true, BREW_BOILER),
            Some(State::BrewModeIdle),
            "this is the arm that was missing"
        );
    }

    #[test]
    fn the_machine_will_not_change_mode_while_it_is_busy() {
        // Refusing here is the point, not an omission: switching to steam mid-shot would
        // take the element away from the brew that is running.
        for state in [State::Brewing, State::PumpingToWaterTap] {
            for (enable, boiler) in ALL_COMMANDS {
                assert_eq!(
                    boiler_mode_transition(state, enable, boiler),
                    None,
                    "{state:?} should refuse enable={enable} boiler={boiler}"
                );
            }
        }
    }

    #[test]
    fn an_unknown_boiler_index_is_refused_everywhere() {
        // This machine has two boiler indices and one of them is imaginary. A third is a
        // caller error, and answering it by moving the machine would be worse than refusing.
        for state in ALL_STATES {
            for enable in [true, false] {
                assert_eq!(boiler_mode_transition(state, enable, 2), None);
            }
        }
    }
}
