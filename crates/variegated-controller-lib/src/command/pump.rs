//! What a stored [`PumpConfiguration`] does to a commanded duty cycle.
//!
//! Not a command handler, unlike the rest of this module -- it is the *reading* half of
//! `SetGroupPumpConfiguration` and `SetWaterTapPumpConfiguration`, and it lives here because a
//! setting nothing reads is a setting that silently does nothing. The dual-boiler controller
//! carried two identical copies of this, one per pump.

use variegated_control_algorithm::pid::PidCtrl;
use variegated_controller_types::{HexadecimalDutyCycleType, PidParameters, PumpConfiguration};
use variegated_log::log_info;

/// Which quantity a pump loop is controlling.
///
/// Exists so the three `InferGroup*Integral` commands are one function instead of three
/// twenty-line copies per machine. The copies differed only in the measurement they read, the
/// gains they loaded and the noun in their log lines -- and each machine had its own set.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PumpQuantity {
    /// Pressure at the group.
    Pressure,
    /// Flow *into* the group, as the pump measures it.
    GroupFlowRate,
    /// Flow *out of* the group, as the scale measures it.
    OutputFlowRate,
}

impl PumpQuantity {
    /// The noun this quantity is called in logs.
    pub const fn label(self) -> &'static str {
        match self {
            Self::Pressure => "pressure",
            Self::GroupFlowRate => "flow rate",
            Self::OutputFlowRate => "output flow rate",
        }
    }
}

/// Seed the pump PID's integral so a takeover starts from what the pump is already doing.
///
/// This is bumpless transfer: without it the loop takes over from a standing start, and the
/// pump drops to zero and climbs back. See [`crate::pump_transfer`] for the same idea on the
/// engagement path.
///
/// The caller supplies the duty cycle because **the two machines disagree about which one to
/// read**, and that disagreement is pre-existing rather than something this function should
/// paper over: the single-boiler controller reads the duty the pump is actually running at,
/// while the dual-boiler reads the `FixedDutyCycle` *target*, which is only the same thing
/// when the transfer comes from duty-cycle mode. Unifying them is a change to the dual-boiler's
/// pump behaviour, not a refactor, so it is left for its own commit.
pub fn seed_pump_integral(
    pid: &mut PidCtrl<f32>,
    quantity: PumpQuantity,
    target: f32,
    parameters: PidParameters,
    duty_cycle: HexadecimalDutyCycleType,
    measurement: f32,
) {
    log_info!("Inferring group {} integral for target: {}", quantity.label(), target);

    pid.setpoint = target;
    pid.set_parameters(parameters);
    pid.infer_and_set_integral(duty_cycle.value() as f32, measurement);

    log_info!(
        "Set {} integral based on duty cycle {}/255 and measurement {}",
        quantity.label(),
        duty_cycle.value(),
        measurement
    );
}

/// Clamp a commanded duty cycle to the pump's configured range.
///
/// **`is_off` wins over everything, including a configured minimum.** A minimum duty cycle
/// describes the point below which *this* pump stalls rather than turning slowly; it is not a
/// floor on a pump that has been told to stop. Without that precedence, configuring a minimum
/// would leave the pump running whenever the controller asked for nothing.
///
/// The maximum is applied after the minimum, so a configuration with `min > max` resolves to
/// the maximum. That is arbitrary but it is what both existing copies did, and clamping to the
/// *lower* of two contradictory limits is the safer of the two arbitrary answers.
pub fn apply_pump_limits(
    duty_cycle: HexadecimalDutyCycleType,
    is_off: bool,
    config: Option<&PumpConfiguration>,
) -> HexadecimalDutyCycleType {
    if is_off {
        return HexadecimalDutyCycleType::OFF;
    }

    let Some(config) = config else {
        return duty_cycle;
    };

    let mut limited = duty_cycle;
    if let Some(min) = config.min_duty_cycle {
        limited = limited.max(min.into());
    }
    if let Some(max) = config.max_duty_cycle {
        limited = limited.min(max.into());
    }
    limited
}

#[cfg(test)]
mod tests {
    use super::*;
    use variegated_controller_types::DutyCycleType;

    fn config(min: Option<u8>, max: Option<u8>) -> PumpConfiguration {
        PumpConfiguration {
            min_duty_cycle: min.map(DutyCycleType::new),
            max_duty_cycle: max.map(DutyCycleType::new),
            ..PumpConfiguration::default()
        }
    }

    /// With no configuration stored, the commanded duty passes through untouched.
    ///
    /// This is the case on every machine that has never had one set, which makes it the case
    /// that must not change behaviour.
    #[test]
    fn an_unconfigured_pump_is_unclamped() {
        let commanded = HexadecimalDutyCycleType::new(137);
        assert_eq!(apply_pump_limits(commanded, false, None), commanded);
    }

    /// A minimum raises a duty cycle below it.
    #[test]
    fn a_minimum_raises_a_low_duty_cycle() {
        let limits = config(Some(20), None);
        // 20% of 255 is 51.
        assert_eq!(
            apply_pump_limits(HexadecimalDutyCycleType::new(10), false, Some(&limits)),
            HexadecimalDutyCycleType::new(51)
        );
    }

    /// A maximum caps a duty cycle above it.
    #[test]
    fn a_maximum_caps_a_high_duty_cycle() {
        let limits = config(None, Some(50));
        // 128, not 127: the percent-to-raw conversion rounds half up, and says so.
        assert_eq!(
            apply_pump_limits(HexadecimalDutyCycleType::FULL, false, Some(&limits)),
            HexadecimalDutyCycleType::new(128)
        );
    }

    /// A duty cycle already inside the range is left alone.
    #[test]
    fn a_duty_cycle_within_the_range_is_untouched() {
        let limits = config(Some(10), Some(90));
        let commanded = HexadecimalDutyCycleType::new(128);
        assert_eq!(apply_pump_limits(commanded, false, Some(&limits)), commanded);
    }

    /// **Off beats a configured minimum.**
    ///
    /// The property worth having a test for: a minimum duty cycle exists because a pump
    /// stalls below it, not because the pump should never stop. Getting this backwards leaves
    /// the pump running whenever the controller commands nothing -- which on this machine
    /// means water through the group with no shot in progress.
    #[test]
    fn off_beats_a_configured_minimum() {
        let limits = config(Some(30), None);
        assert_eq!(
            apply_pump_limits(HexadecimalDutyCycleType::FULL, true, Some(&limits)),
            HexadecimalDutyCycleType::OFF
        );
        assert_eq!(
            apply_pump_limits(HexadecimalDutyCycleType::OFF, true, Some(&limits)),
            HexadecimalDutyCycleType::OFF
        );
    }

    /// Off with no configuration is still off.
    #[test]
    fn off_is_off_without_a_configuration() {
        assert_eq!(
            apply_pump_limits(HexadecimalDutyCycleType::FULL, true, None),
            HexadecimalDutyCycleType::OFF
        );
    }

    /// A contradictory configuration resolves to the maximum, which is the safer answer.
    #[test]
    fn a_minimum_above_the_maximum_resolves_to_the_maximum() {
        let limits = config(Some(80), Some(20));
        assert_eq!(
            apply_pump_limits(HexadecimalDutyCycleType::new(128), false, Some(&limits)),
            HexadecimalDutyCycleType::new(51) // 20% of 255
        );
    }
}
