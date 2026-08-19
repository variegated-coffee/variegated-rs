//! Whether a machine's scale can be calibrated from its panel, and whether it can right now.
//!
//! # Why the question is not just "is there a scale"
//!
//! Taring works on every scale in this tree. Zero- and reference-weight calibration do not:
//!
//! | driver | `zero_calibration` | `reference_weight_calibration` |
//! |---|---|---|
//! | `variegated-hal`'s `GravityController` (local load cell) | yes | yes, at 100 g |
//! | `variegated-hal`'s `BluetoothScaleController` | **no** | **no** |
//!
//! `ScaleOp` says why: *"Only `Tare` today, because it is the only one the ACAIA driver can
//! perform -- there is no zero-calibration or reference-weight command in that protocol."* A
//! Bluetooth scale is calibrated by its own vendor app, and this firmware has no command that
//! would change that.
//!
//! So on the GS3's default build, which takes its weight from a Bluetooth scale, both
//! operations are unreachable -- and `Group::scale_zero_calibration` returns `Ok(())` when
//! there is no controller at all. A panel row wired straight to the command would be a
//! physical control that reports success and does nothing.
//!
//! # Why it reads `support_calibration` rather than asking the driver
//!
//! [`variegated_hal::scale::ScaleController::get_capabilities`] is the accurate answer, but it
//! lives on the HAL object owned by `Group` inside the controller task, and no menu ever sees
//! a `Group`. [`PeripheralDefinition::support_calibration`] is the same fact already published
//! to everyone who needs it -- it is set per scale in both firmwares, with the Bluetooth site
//! carrying a comment tying it to `get_capabilities` -- and until now it was written and
//! serialised but read by nothing.
//!
//! # Why three answers and not a bool
//!
//! The two ways this can be "no" want opposite treatment on a panel, and collapsing them
//! loses the distinction that tells a user what to do:
//!
//! - [`ScaleCalibration::Unsupported`] is a permanent property of the fitted hardware. Nothing
//!   the user does will change it, so the rows are not drawn at all. A row that can never
//!   become available is worse than no row.
//! - [`ScaleCalibration::Offline`] is transient -- the scale is simply not answering. The rows
//!   are drawn and greyed, the same way an unrunnable routine is, because "switch the scale
//!   on" is a fix and the row is where that gets said.
//!
//! Pure, and here rather than in a firmware for the reason [`crate::routine_prerequisites`]
//! gives: three callers on the GS3 alone -- the button task and both renderers -- and they
//! must agree, because this decides how many rows the Scale submenu *has*. Two sides
//! disagreeing about that is a selection index pointing at different rows on each.

use variegated_controller_types::{MachineDefinition, PeripheralStatus, PeripheralType};

/// Whether the scale calibration rows should be drawn, and whether they can be pressed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ScaleCalibration {
    /// No fitted scale can calibrate. Do not draw the rows.
    Unsupported,
    /// A scale that can calibrate is fitted, but is not answering. Draw greyed, refuse.
    Offline,
    /// Ready.
    Available,
}

impl ScaleCalibration {
    /// Whether the rows exist on this machine at all.
    ///
    /// Deliberately true for [`Self::Offline`]: a scale that is merely switched off must not
    /// change the length of a list the user is already navigating.
    pub const fn is_offered(self) -> bool {
        !matches!(self, Self::Unsupported)
    }

    /// Whether pressing one would do anything.
    pub const fn is_available(self) -> bool {
        matches!(self, Self::Available)
    }
}

/// What the panel should do about the scale calibration rows.
///
/// Considers only peripherals of type [`PeripheralType::Scale`]. `support_calibration` is
/// declared on every peripheral, but a conductivity probe that set it would be talking about
/// its own calibration, not a scale's, and the commands these rows send name a *group scale*.
pub fn scale_calibration(
    definition: &MachineDefinition,
    peripherals: &PeripheralStatus,
) -> ScaleCalibration {
    let mut fitted = false;

    for (id, peripheral) in definition.peripherals.iter() {
        if peripheral.peripheral_type != PeripheralType::Scale || !peripheral.support_calibration {
            continue;
        }
        fitted = true;
        // Same liveness rule as `routine_prerequisites::capability_available`, and absent
        // from the map counts as unavailable for the same reason: the map is rebuilt every
        // status tick, so a missing entry means nothing registered a provider.
        if peripherals
            .peripherals
            .get(id)
            .is_some_and(|info| info.is_available)
        {
            return ScaleCalibration::Available;
        }
    }

    if fitted {
        ScaleCalibration::Offline
    } else {
        ScaleCalibration::Unsupported
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use variegated_controller_types::{
        PeripheralDefinition, PeripheralInfo, SensorCapability,
    };

    const GRAVITY: u16 = 0x5C1E;
    const BLUETOOTH_SCALE: u16 = 0xB5C0;
    const PROBE: u16 = 0xB1CA;

    fn empty_definition() -> MachineDefinition {
        MachineDefinition {
            name: heapless::String::try_from("test").expect("fits"),
            boilers: Default::default(),
            groups: Default::default(),
            water_taps: Default::default(),
            tanks: Default::default(),
            steam_wands: Default::default(),
            environmental_sensors: Default::default(),
            peripherals: Default::default(),
            function_routines: Default::default(),
        }
    }

    fn with_peripheral(
        definition: &mut MachineDefinition,
        id: u16,
        peripheral_type: PeripheralType,
        support_calibration: bool,
    ) {
        let mut capabilities = heapless::Vec::new();
        let _ = capabilities.push(SensorCapability::Weight);
        let _ = definition.add_peripheral(
            id,
            PeripheralDefinition {
                peripheral_type,
                location: heapless::String::try_from("Group 1").expect("fits"),
                capabilities,
                support_calibration,
                via_comms_mcu: true,
            },
        );
    }

    fn live(available: &[(u16, bool)]) -> PeripheralStatus {
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

    #[test]
    fn the_shipping_gs3_does_not_offer_calibration_at_all() {
        // The default build's scale is the Bluetooth one, whose driver reports false for
        // both operations. This is the case the whole gate exists for: the rows must not be
        // drawn, not merely greyed, because no action by the user can ever enable them.
        let mut definition = empty_definition();
        with_peripheral(&mut definition, BLUETOOTH_SCALE, PeripheralType::Scale, false);

        let calibration = scale_calibration(&definition, &live(&[(BLUETOOTH_SCALE, true)]));

        assert_eq!(calibration, ScaleCalibration::Unsupported);
        assert!(!calibration.is_offered());
    }

    #[test]
    fn a_connected_gravity_scale_is_available() {
        let mut definition = empty_definition();
        with_peripheral(&mut definition, GRAVITY, PeripheralType::Scale, true);

        let calibration = scale_calibration(&definition, &live(&[(GRAVITY, true)]));

        assert_eq!(calibration, ScaleCalibration::Available);
        assert!(calibration.is_offered() && calibration.is_available());
    }

    #[test]
    fn a_calibrating_scale_that_is_switched_off_is_offline_not_unsupported() {
        // The distinction the enum exists for. Offline still draws the rows -- greyed --
        // because "switch the scale on" is a fix, and a row that vanished would take the
        // instruction with it.
        let mut definition = empty_definition();
        with_peripheral(&mut definition, GRAVITY, PeripheralType::Scale, true);

        let calibration = scale_calibration(&definition, &live(&[(GRAVITY, false)]));

        assert_eq!(calibration, ScaleCalibration::Offline);
        assert!(calibration.is_offered());
        assert!(!calibration.is_available());
    }

    #[test]
    fn a_scale_missing_from_the_live_map_is_offline_rather_than_available() {
        // Absent is not "assume fine": the map is rebuilt every status tick, so no entry
        // means nothing registered a provider for it.
        let mut definition = empty_definition();
        with_peripheral(&mut definition, GRAVITY, PeripheralType::Scale, true);

        assert_eq!(
            scale_calibration(&definition, &live(&[])),
            ScaleCalibration::Offline,
        );
    }

    #[test]
    fn a_machine_with_no_scale_offers_nothing() {
        assert_eq!(
            scale_calibration(&empty_definition(), &live(&[])),
            ScaleCalibration::Unsupported,
        );
    }

    #[test]
    fn a_calibratable_non_scale_peripheral_does_not_count() {
        // `support_calibration` is declared on every peripheral. A probe that sets it is
        // talking about its own calibration, and these rows send `ZeroCalibrateGroupScale`.
        let mut definition = empty_definition();
        with_peripheral(&mut definition, PROBE, PeripheralType::BrewSensor, true);

        assert_eq!(
            scale_calibration(&definition, &live(&[(PROBE, true)])),
            ScaleCalibration::Unsupported,
        );
    }

    #[test]
    fn a_live_calibrating_scale_wins_over_an_offline_one() {
        // Two scales fitted, only one answering. The rows should be pressable, so the scan
        // must not stop at the first calibrating scale it happens to find.
        let mut definition = empty_definition();
        with_peripheral(&mut definition, BLUETOOTH_SCALE, PeripheralType::Scale, true);
        with_peripheral(&mut definition, GRAVITY, PeripheralType::Scale, true);

        assert_eq!(
            scale_calibration(
                &definition,
                &live(&[(BLUETOOTH_SCALE, false), (GRAVITY, true)]),
            ),
            ScaleCalibration::Available,
        );
    }

    #[test]
    fn a_calibrating_scale_beside_a_non_calibrating_one_is_still_offered() {
        // A machine with both a Bluetooth scale and a load cell: the rows belong to the one
        // that can do the job, and the other must not mask it.
        let mut definition = empty_definition();
        with_peripheral(&mut definition, BLUETOOTH_SCALE, PeripheralType::Scale, false);
        with_peripheral(&mut definition, GRAVITY, PeripheralType::Scale, true);

        assert_eq!(
            scale_calibration(
                &definition,
                &live(&[(BLUETOOTH_SCALE, true), (GRAVITY, true)]),
            ),
            ScaleCalibration::Available,
        );
    }
}
