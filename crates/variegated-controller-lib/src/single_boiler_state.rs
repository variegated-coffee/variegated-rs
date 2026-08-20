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

use variegated_controller_types::{
    BoilerControlMode, BoilerControlState, BoilerControlTargetValues, BoilerIndex, Output,
    SingleBoilerSingleGroupControllerState as State, Status,
};

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

/// The target the single element heats to in steam mode when nothing has configured one.
///
/// **A placeholder, not a tuned value.** It matches the dual-boiler's steam boiler
/// (`dual_boiler_single_group.rs:313`) because that is the only steam target in the tree and
/// it is a conservative one — about 1 bar gauge, which is weak steam for a single-boiler
/// machine, where 140–150 °C is more usual. It is deliberately the low end: this example
/// passes `BoilerConfiguration::default()`, whose `max_temperature` is `None`, so nothing
/// downstream bounds this number. Raise it from the Steam Temperature screen once the
/// boiler's real limit is known.
pub const DEFAULT_STEAM_TARGET_TEMPERATURE: f32 = 120.0;

/// Ceiling for the element in brew mode.
///
/// **A brew target is not a cup temperature.** Water loses heat on its way from the boiler
/// to the puck — through the group, the portafilter and the basket, all of which start
/// colder than the water — so a boiler set at the temperature you want at the puck delivers
/// less than that. Compensating for it is normal practice, and it is why this ceiling has to
/// sit above 100 °C: the interesting brew targets are on the other side of that number.
///
/// This figure was 100.0 when the interlock was introduced, taken from what
/// `current_configuration` had been publishing as the brew boiler's `max_temperature` while
/// nothing enforced it. Advertised and unenforced, 100 bounded nothing and cost nothing;
/// enforced, it cut off exactly the range the machine needs, and the rotary editor —
/// which stops where the interlock cuts in — could no longer reach a usable brew target.
///
/// 110 is a limit, not a recommendation, and the machine is declared good for 150
/// ([`MAX_STEAM_TEMPERATURE`]), so this is not near the hardware's edge. Raise it if a real
/// brew target ever needs more.
pub const MAX_BREW_TEMPERATURE: f32 = 110.0;

/// Ceiling for the element in steam mode, likewise already published for the virtual steam
/// boiler. It is what makes [`DEFAULT_STEAM_TARGET_TEMPERATURE`] look conservative — the
/// machine is declared good for 150 °C.
pub const MAX_STEAM_TEMPERATURE: f32 = 150.0;

/// The ceiling that applies in `state`, since one element serves both roles.
///
/// Brew and steam are the same heater with different limits, so the interlock cannot read a
/// single `boiler_config.max_temperature` the way the dual-boiler's two do.
pub fn max_temperature_for(state: State) -> f32 {
    match state {
        State::SteamModeIdle => MAX_STEAM_TEMPERATURE,
        // Brewing, idling in brew mode, dispensing water, or asleep. The brew ceiling is
        // the lower of the two, so anything not explicitly steam gets the safer one.
        _ => MAX_BREW_TEMPERATURE,
    }
}

/// How the one element's output is split across the two published boiler slots.
///
/// The machine has one heating element and `Status` has two boiler entries, so exactly one
/// of them is the element and the other is a placeholder that must not look live: an
/// interface showing a duty cycle and a set of PID terms on both would be showing the same
/// element twice, and inviting someone to read the wrong one.
///
/// `Output::Off` on the inactive slot is the whole of that convention, and
/// [`active_boiler_index`] is its inverse. **They change together.** If this ever starts
/// publishing the real output on both slots, every consumer that asks *which boiler is
/// driving the element* loses its only answer -- `Status` carries no controller state.
pub fn element_outputs_for(state: State, element: Output) -> (Output, Output) {
    match state {
        State::SteamModeIdle => (Output::Off, element),
        _ => (element, Output::Off),
    }
}

/// The boiler slot whose numbers describe what the element is actually doing.
///
/// The inverse of [`element_outputs_for`]. The steam slot carries a live output exactly
/// while the machine is in steam mode, so a steam slot that is not `Off` is the machine
/// saying the element is under the steam control state -- and the setpoint, duty cycle and
/// PID terms an interface should be showing are that slot's, not the brew slot's.
///
/// The test is the *variant*, not the duty cycle. `update_boiler` returns `Output::Off`
/// only when the effective control mode is `Off`; a boiler that has reached temperature, or
/// been cut off by the over-temperature or dry-run interlock, still reports
/// `PidOutput { out: 0.0, .. }` and is still the live slot.
///
/// **Falls back to the brew boiler when neither slot is live** -- power save, or a machine
/// switched off. That is the right answer for a display, since the brew slot carries the
/// setpoint a user recognises, but it means this cannot tell "steam is off because we are
/// brewing" from "everything is off". It is not a mode oracle.
pub fn active_boiler_index(status: &Status) -> BoilerIndex {
    let steam_is_live = status
        .get_boiler_status(STEAM_BOILER)
        // `matches!` rather than `!= Output::Off`: `PidOutput` carries floats, and an
        // equality test on it is one refactor away from depending on NaN comparing equal.
        .is_some_and(|boiler| !matches!(boiler.output, Output::Off));

    if steam_is_live { STEAM_BOILER } else { BREW_BOILER }
}

/// What the machine should actually heat to in steam mode, given the stored steam state.
///
/// `BoilerControlMode::Off` in this slot is treated as *unconfigured* rather than as a
/// preference, and replaced. On a machine with two real boilers, switching one off is a
/// sensible thing to ask for. Here there is only one element and this state is not a boiler
/// at all — it is "what the element does once the machine is already in steam mode", so
/// `Off` says *entering steam mode shall stop the heating*, which is the one thing steam
/// mode exists not to do.
///
/// It was never asked for, either. The single-boiler default set this to `Off` while the
/// brew slot got `Temperature`, and no interface could change it: the rotary menu only ever
/// sends `SetBoilerControlTarget` for boiler 0, so the only route was the web or debug link
/// aimed at boiler 1 by hand. The stored `Off` on any existing machine is that default,
/// copied into flash by the first unrelated save.
///
/// Self-healing: setting a steam temperature stores `Temperature`, after which this stops
/// substituting anything. Once every machine has been through that, it can be deleted.
pub fn steam_boiler_state_or_default(stored: BoilerControlState) -> BoilerControlState {
    if stored.mode != BoilerControlMode::Off {
        return stored;
    }

    BoilerControlState {
        mode: BoilerControlMode::Temperature,
        values: BoilerControlTargetValues {
            // The stored pressure is kept: it is meaningful if the machine is later put
            // into pressure control, and there is no reason to discard it.
            target_temperature: DEFAULT_STEAM_TARGET_TEMPERATURE,
            ..stored.values
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use variegated_control_algorithm::pid::PidOut;
    use variegated_controller_types::{BoilerStatus, DutyCycleType};

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

    /// A `Status` carrying the two boiler slots and nothing else, as `send_status` builds
    /// it: both slots report the same physical sensors, and only the output differs.
    fn status_with(brew: Output, steam: Output) -> Status {
        let mut status = Status::default();

        for (index, output) in [(BREW_BOILER, brew), (STEAM_BOILER, steam)] {
            let _ = status.boiler_statuses.insert(
                index,
                BoilerStatus {
                    temperature: Some(94.0),
                    pressure: None,
                    water_level: None,
                    output,
                    control_state: BoilerControlState::default(),
                },
            );
        }

        status
    }

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

    /// The reported bug: the switch changed the mode, and the element then went to 0%
    /// because the state steam mode selects said `Off`.
    #[test]
    fn a_stored_off_does_not_leave_steam_mode_unheated() {
        let stored = BoilerControlState {
            mode: BoilerControlMode::Off,
            values: BoilerControlTargetValues::default(),
        };

        let effective = steam_boiler_state_or_default(stored);

        assert_eq!(effective.mode, BoilerControlMode::Temperature);
        assert_eq!(
            effective.values.target_temperature,
            DEFAULT_STEAM_TARGET_TEMPERATURE
        );
    }

    /// The default carries a brew temperature (93 °C), which would heat -- and so look like
    /// it worked -- while producing no steam at all. Substituting the mode is not enough.
    #[test]
    fn the_substituted_target_is_a_steam_temperature() {
        let stored = BoilerControlState {
            mode: BoilerControlMode::Off,
            values: BoilerControlTargetValues::default(),
        };
        assert_eq!(stored.values.target_temperature, 93.0);

        let effective = steam_boiler_state_or_default(stored);
        assert!(
            effective.values.target_temperature > 100.0,
            "a steam target has to be above boiling, got {}",
            effective.values.target_temperature
        );
    }

    /// A configured machine is left exactly alone -- including one configured *below* the
    /// substituted default, which is a legitimate choice and must not be raised.
    #[test]
    fn a_configured_steam_state_is_untouched() {
        for mode in [BoilerControlMode::Temperature, BoilerControlMode::Pressure] {
            let stored = BoilerControlState {
                mode,
                values: BoilerControlTargetValues {
                    target_temperature: 111.0,
                    target_pressure: 1.7,
                },
            };
            assert_eq!(steam_boiler_state_or_default(stored), stored);
        }
    }

    /// Idempotent, because `task` reloads settings every iteration and runs this on each
    /// one. A second application must not drift the target.
    #[test]
    fn substituting_twice_changes_nothing_further() {
        let stored = BoilerControlState {
            mode: BoilerControlMode::Off,
            values: BoilerControlTargetValues::default(),
        };
        let once = steam_boiler_state_or_default(stored);
        assert_eq!(steam_boiler_state_or_default(once), once);
    }

    /// Only steam mode gets the high ceiling. Every other state -- including the brewing and
    /// water-dispensing ones, where the element is under the brew control state -- gets the
    /// lower one, so a mis-set steam target cannot bleed into a brew.
    #[test]
    fn only_steam_mode_gets_the_steam_ceiling() {
        for state in ALL_STATES {
            let expected = if state == State::SteamModeIdle {
                MAX_STEAM_TEMPERATURE
            } else {
                MAX_BREW_TEMPERATURE
            };
            assert_eq!(max_temperature_for(state), expected, "for {:?}", state);
        }
    }

    /// The brew ceiling has to leave room above the boiling point, because a brew target is
    /// not a cup temperature: the water cools between the boiler and the puck, and the
    /// setpoint is raised to pay for it. A ceiling at 100 was enforced briefly and put the
    /// useful part of that range out of reach — both of the interlock and of the rotary
    /// editor, which stops wherever this constant does.
    #[test]
    fn the_brew_ceiling_leaves_room_above_boiling() {
        assert!(
            MAX_BREW_TEMPERATURE > 100.0,
            "a brew target has to be able to exceed 100 °C to compensate for the loss \
             between boiler and grouphead; got {MAX_BREW_TEMPERATURE}"
        );
    }

    /// The substituted steam default has to sit under the ceiling that will be enforced
    /// against it, or the machine would ship heating into its own interlock.
    #[test]
    fn the_steam_default_is_below_the_steam_ceiling() {
        assert!(DEFAULT_STEAM_TARGET_TEMPERATURE < MAX_STEAM_TEMPERATURE);
        assert!(MAX_BREW_TEMPERATURE < MAX_STEAM_TEMPERATURE);
    }

    /// The stored pressure survives, so a machine later put into pressure control still has
    /// the number its owner set.
    #[test]
    fn the_stored_pressure_target_is_preserved() {
        let stored = BoilerControlState {
            mode: BoilerControlMode::Off,
            values: BoilerControlTargetValues {
                target_temperature: 93.0,
                target_pressure: 1.9,
            },
        };
        assert_eq!(
            steam_boiler_state_or_default(stored).values.target_pressure,
            1.9
        );
    }

    /// The property the display depends on: the slot it picks is the slot the element was
    /// published under. Built with `element_outputs_for` rather than by hand, so the two
    /// halves of the convention cannot drift apart without this failing.
    #[test]
    fn the_active_slot_is_the_one_the_element_is_under() {
        let element = Output::FixedDutyCycle(DutyCycleType::new(47));

        for state in ALL_STATES {
            let (brew, steam) = element_outputs_for(state, element);

            let expected = if state == State::SteamModeIdle {
                STEAM_BOILER
            } else {
                BREW_BOILER
            };

            assert_eq!(
                active_boiler_index(&status_with(brew, steam)),
                expected,
                "for {state:?}"
            );
        }
    }

    /// A steam PID that has settled to zero duty is still the live slot.
    ///
    /// This is what a rule of "pick the boiler with a non-zero duty cycle" would get wrong,
    /// and it is the steady state of a machine that has reached its steam temperature --
    /// the moment someone is most likely to be looking at the screen.
    #[test]
    fn a_settled_steam_pid_is_still_the_active_boiler() {
        let settled = Output::PidOutput(PidOut::default());
        assert_eq!(
            active_boiler_index(&status_with(Output::Off, settled)),
            STEAM_BOILER
        );
    }

    /// Power save, and a machine switched off, leave neither slot live. Answering "brew" is
    /// a choice rather than an accident: it is the slot carrying the setpoint a user
    /// recognises.
    #[test]
    fn nothing_running_falls_back_to_the_brew_boiler() {
        assert_eq!(
            active_boiler_index(&status_with(Output::Off, Output::Off)),
            BREW_BOILER
        );
    }

    /// The display holds a `Status::default()` until the first one arrives, and renders from
    /// it. Answering with a boiler index that is not in the map would be worse than useless.
    #[test]
    fn an_absent_steam_slot_is_the_brew_boiler() {
        assert_eq!(active_boiler_index(&Status::default()), BREW_BOILER);
    }
}
