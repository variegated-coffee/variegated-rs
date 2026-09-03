//! Whether a machine can run a routine at all.
//!
//! A routine declares what it needs to sense -- a scale, a conductivity probe -- as a list
//! of [`RoutinePrerequisite`]. This module answers whether those needs are met right now.
//!
//! # Why this exists
//!
//! Before it, nothing checked. A brew-by-weight routine on a machine with no scale started
//! happily: `StateCondition::OutputWeightAbove` reads `output_weight.unwrap_or(0.0)`, an
//! absent scale therefore reads as zero grams, and the exit condition simply never fires.
//! The routine ran to its failsafe or forever, and no layer refused it or warned.
//!
//! # Why it is here rather than in the controllers
//!
//! For the reason [`crate::routine_progress`] gives about itself. Five callers need this
//! answer -- both controllers, both device menus and the write path -- and the two
//! controllers are forked badly enough that a copy in each would be two implementations
//! within a release. It is a pure function over a [`MachineDefinition`] and a [`Status`],
//! so it costs nothing to share and can be tested on a host.
//!
//! # What "available" means
//!
//! Two sources, and both are needed. [`MachineDefinition::peripherals`] is the static
//! declaration -- this machine *has* a Belka portal, which senses conductivity -- and it is
//! fixed at boot. [`PeripheralStatus`] is the live half: whether that peripheral is talking
//! to us this second. A capability is satisfied when some peripheral declares it *and* that
//! same peripheral is currently available.
//!
//! Sensors soldered to the board -- declared on a boiler, group, water tap or tank rather
//! than as a peripheral -- have no liveness half and count whenever they are declared. See
//! [`capability_available`].
//!
//! The live half is a [`PeripheralStatus`] rather than a whole `Status` because that is all
//! this needs, and because the controllers have the map to hand at points where they have no
//! assembled `Status`: one is built per control pass inside `send_status`, and constructing
//! a second just to answer this question would be absurd. Callers that do hold a `Status`
//! pass `&status.peripheral_status`.
//!
//! A peripheral declared in the definition but missing from `peripheral_status` counts as
//! unavailable rather than as available-by-default. That is the safer direction: the map is
//! rebuilt from the registry every status tick, so an absent entry means nothing registered
//! a provider for it, which is exactly the machine that cannot do the sensing.

use embassy_time::Duration;
use variegated_controller_types::{
    MachineDefinition, PeripheralStatus, RoutinePrerequisite, SensorCapability,
};

/// How long a running routine's prerequisites may be unmet before it is abandoned.
///
/// **Failsafe, not operational.** `is_available` is edge-driven from a `Signal` latched into
/// a `Cell` (`variegated-hal`'s `BluetoothScaleStatusProvider`), and BLE re-association is
/// owned entirely by the comms processor, so a link that merely blips must not cost the user
/// a shot. Long enough to outlast that; short enough that a shot does not run blind to
/// completion on a scale that is really gone.
///
/// Not a timeout on anything the machine does -- nothing here waits for it. It only bounds
/// how long a routine may keep running after the thing it depends on has stopped answering.
pub const PREREQUISITE_LOSS_GRACE: Duration = Duration::from_secs(3);

/// Whether one prerequisite is met.
pub fn prerequisite_satisfied(
    prerequisite: &RoutinePrerequisite,
    definition: &MachineDefinition,
    peripherals: &PeripheralStatus,
) -> bool {
    capability_available(prerequisite.capability, definition, peripherals)
}

/// Whether this machine can currently do this kind of sensing, from any source.
///
/// **Two kinds of source, with different liveness rules.**
///
/// A *peripheral* is a separate device -- a scale, a conductivity probe -- that connects and
/// disconnects, so it counts only while `PeripheralStatus` says it is answering.
///
/// A *built-in* sensor declared on a boiler, group, water tap or tank is soldered to the
/// board. It is available whenever the machine declares it, because there is no connection
/// to lose; a failed one reports through `DebugEvent::SensorFault`, which is a different
/// problem with a different remedy.
///
/// Both are needed. Checking only peripherals -- which is what this did at first -- makes
/// `OutputFlowRate`, `InputFlowRate` and `WaterLevel` permanently unsatisfiable on both
/// machines in this tree, because those are declared on the group and the tank and no
/// peripheral offers them. A routine that named one would be greyed out forever on a machine
/// that demonstrably has the sensor, which is worse than having no prerequisite at all: it
/// looks like a hardware fault.
pub fn capability_available(
    capability: SensorCapability,
    definition: &MachineDefinition,
    peripherals: &PeripheralStatus,
) -> bool {
    let from_peripheral = definition.peripherals.iter().any(|(id, peripheral)| {
        peripheral.capabilities.contains(&capability)
            && peripherals
                .peripherals
                .get(id)
                .is_some_and(|info| info.is_available)
    });

    from_peripheral || built_in(definition, capability)
}

/// Whether any boiler, group, water tap or tank declares this capability.
fn built_in(definition: &MachineDefinition, capability: SensorCapability) -> bool {
    definition
        .boilers
        .iter()
        .any(|(_, b)| b.sensors.contains(&capability))
        || definition
            .groups
            .iter()
            .any(|(_, g)| g.sensors.contains(&capability))
        || definition
            .water_taps
            .iter()
            .any(|(_, w)| w.sensors.contains(&capability))
        || definition
            .tanks
            .iter()
            .any(|(_, t)| t.sensors.contains(&capability))
}

/// Every prerequisite of a routine that is not currently met.
///
/// An iterator rather than a `bool` so callers can say *which* capability is missing:
/// "needs a scale" is a usable message where "cannot run" is not. Callers that only want a
/// yes/no use `.next().is_none()`.
pub fn unmet_prerequisites<'a>(
    prerequisites: &'a [RoutinePrerequisite],
    definition: &'a MachineDefinition,
    peripherals: &'a PeripheralStatus,
) -> impl Iterator<Item = &'a RoutinePrerequisite> + 'a {
    prerequisites
        .iter()
        .filter(move |p| !prerequisite_satisfied(p, definition, peripherals))
}

/// Whether every prerequisite is met.
pub fn prerequisites_satisfied(
    prerequisites: &[RoutinePrerequisite],
    definition: &MachineDefinition,
    peripherals: &PeripheralStatus,
) -> bool {
    unmet_prerequisites(prerequisites, definition, peripherals)
        .next()
        .is_none()
}

#[cfg(test)]
mod tests {
    use super::*;
    use variegated_controller_types::{
        PeripheralDefinition, PeripheralInfo, PeripheralType,
    };

    const SCALE: u16 = 0xB5C0;
    const PROBE: u16 = 0xB1CA;

    fn definition() -> MachineDefinition {
        let mut definition = MachineDefinition {
            name: heapless::String::try_from("test").expect("fits"),
            boilers: Default::default(),
            groups: Default::default(),
            water_taps: Default::default(),
            tanks: Default::default(),
            steam_wands: Default::default(),
            environmental_sensors: Default::default(),
            peripherals: Default::default(),
            function_routines: Default::default(),
        };

        let mut scale_caps = heapless::Vec::new();
        let _ = scale_caps.push(SensorCapability::Weight);
        let _ = definition.add_peripheral(
            SCALE,
            PeripheralDefinition {
                peripheral_type: PeripheralType::Scale,
                location: heapless::String::try_from("Group").expect("fits"),
                capabilities: scale_caps,
                support_calibration: false,
                via_comms_mcu: true,
                // A Bluetooth scale: no calibration, but it does drive a timer. Irrelevant to
                // every assertion here, which is about capabilities rather than controls.
                support_timer: true,
            },
        );

        let mut probe_caps = heapless::Vec::new();
        let _ = probe_caps.push(SensorCapability::ElectricalConductivity);
        let _ = probe_caps.push(SensorCapability::Temperature);
        let _ = definition.add_peripheral(
            PROBE,
            PeripheralDefinition {
                peripheral_type: PeripheralType::BrewSensor,
                location: heapless::String::try_from("Cup").expect("fits"),
                capabilities: probe_caps,
                support_calibration: false,
                via_comms_mcu: true,
                support_timer: false,
            },
        );

        definition
    }

    fn status_with(available: &[(u16, bool)]) -> PeripheralStatus {
        let mut status = PeripheralStatus::default();
        for (id, is_available) in available {
            let _ = status.peripherals.insert(
                *id,
                PeripheralInfo {
                    peripheral_type: PeripheralType::Scale,
                    is_available: *is_available,
                },
            );
        }
        status
    }

    fn needs(capability: SensorCapability) -> [RoutinePrerequisite; 1] {
        [RoutinePrerequisite { capability }]
    }

    #[test]
    fn a_declared_and_connected_peripheral_satisfies_its_capability() {
        let status = status_with(&[(SCALE, true), (PROBE, true)]);

        assert!(prerequisites_satisfied(&needs(SensorCapability::Weight), &definition(), &status));
        assert!(prerequisites_satisfied(
            &needs(SensorCapability::ElectricalConductivity),
            &definition(),
            &status
        ));
    }

    #[test]
    fn a_disconnected_peripheral_does_not() {
        // The case the whole feature exists for: the machine *has* a scale, the routine
        // needs one, and the scale is not talking to us.
        let status = status_with(&[(SCALE, false), (PROBE, true)]);

        assert!(!prerequisites_satisfied(&needs(SensorCapability::Weight), &definition(), &status));
        assert!(prerequisites_satisfied(
            &needs(SensorCapability::ElectricalConductivity),
            &definition(),
            &status
        ));
    }

    #[test]
    fn a_peripheral_missing_from_the_status_map_counts_as_unavailable() {
        // Declared in the definition, absent from the live map -- nothing registered a
        // provider for it. Defaulting that to "available" would let a routine start on a
        // machine that cannot do the sensing at all, which is worse than a false refusal.
        let status = status_with(&[(PROBE, true)]);

        assert!(!prerequisites_satisfied(&needs(SensorCapability::Weight), &definition(), &status));
    }

    #[test]
    fn a_capability_nothing_declares_is_never_satisfied() {
        // A routine authored on a machine that has something this one does not.
        //
        // `Pressure`, not `WaterLevel`. This test used `WaterLevel` as its example of "a
        // capability this machine lacks" while both real machines declare it on their tank
        // -- so it passed only because the implementation was wrong in the same direction,
        // and it would have gone on passing after that was fixed only by accident.
        let status = status_with(&[(SCALE, true), (PROBE, true)]);

        assert!(!prerequisites_satisfied(
            &needs(SensorCapability::Pressure),
            &definition(),
            &status
        ));
    }

    #[test]
    fn a_sensor_soldered_to_the_board_needs_no_peripheral() {
        // The case that made three of the five capabilities the editor offers permanently
        // unsatisfiable: `OutputFlowRate` is declared on the *group*, and `WaterLevel` on the
        // *tank*. Neither is a peripheral, so neither has an entry in `PeripheralStatus` and
        // neither ever will -- but the machine has them, and a routine may need them.
        let mut definition = definition();
        let mut group_sensors = heapless::Vec::new();
        let _ = group_sensors.push(SensorCapability::OutputFlowRate);
        let _ = definition.add_group(
            0,
            variegated_controller_types::GroupDefinition {
                name: heapless::String::try_from("Group").expect("fits"),
                sensors: group_sensors,
                actuators: Default::default(),
                control_modes: Default::default(),
            },
        );

        // Nothing connected at all -- built-ins do not depend on it.
        let status = status_with(&[]);

        assert!(prerequisites_satisfied(
            &needs(SensorCapability::OutputFlowRate),
            &definition,
            &status
        ));
        // And a peripheral capability is still gated on liveness.
        assert!(!prerequisites_satisfied(&needs(SensorCapability::Weight), &definition, &status));
    }

    #[test]
    fn no_prerequisites_means_it_runs_anywhere() {
        // What every routine written before this field existed meant in practice, and what
        // an empty list has to keep meaning or the upgrade breaks every one of them.
        let status = status_with(&[]);

        assert!(prerequisites_satisfied(&[], &definition(), &status));
    }

    #[test]
    fn the_unmet_one_is_named_rather_than_just_counted() {
        // "needs a scale" is a message a user can act on; "cannot run" is not.
        let status = status_with(&[(SCALE, false), (PROBE, true)]);
        let definition = definition();
        let prerequisites = [
            RoutinePrerequisite { capability: SensorCapability::ElectricalConductivity },
            RoutinePrerequisite { capability: SensorCapability::Weight },
        ];

        let missing: alloc::vec::Vec<_> =
            unmet_prerequisites(&prerequisites, &definition, &status).collect();

        assert_eq!(missing.len(), 1);
        assert_eq!(missing[0].capability, SensorCapability::Weight);
    }

    #[test]
    fn one_peripheral_can_satisfy_several_capabilities() {
        // The Belka portal declares conductivity *and* temperature. A routine needing both
        // is satisfied by that one device, not blocked for want of a second.
        let status = status_with(&[(PROBE, true)]);
        let prerequisites = [
            RoutinePrerequisite { capability: SensorCapability::ElectricalConductivity },
            RoutinePrerequisite { capability: SensorCapability::Temperature },
        ];

        assert!(prerequisites_satisfied(&prerequisites, &definition(), &status));
    }
}
