//! The dual-boiler machine's configuration, and how it reaches the wire.
//!
//! Split out of [`crate::dual_boiler_single_group`] and **deliberately ungated**, where the
//! controller that owns it is behind `hardware`. Nothing here touches a peripheral: it is
//! three plain structs, their stored representation, their defaults, and the projection
//! onto the published [`Configuration`]. The controller re-exports all of it, so no caller
//! outside this crate changes.
//!
//! See [`crate::single_boiler_config`] for why the split exists; the same reasoning applies
//! to both machines, and having only one of them out here would defeat it -- the command
//! handlers are shared, so both configurations have to be constructible in a host test.

use crc::{Crc, CRC_32_ISCSI};
use postcard::{from_bytes_crc32, to_slice_crc32};
use sequential_storage::map::{SerializationError, Value};
use variegated_log::{log_info, log_warn};

use variegated_controller_types::DualBoilerSingleGroupControllerBoilers::{BrewBoiler, SteamBoiler};
use variegated_controller_types::SingleGroupControllerGroups::SingleGroup;
use variegated_controller_types::{
    BoilerConfiguration, BoilerControlMode, BoilerControlState, BoilerControlTargetValues,
    Configuration, DutyCycleType, FillConfiguration, GroupBrewControlMode, GroupBrewControlState,
    GroupBrewControlTargetValues, GroupBrewLimitMode, GroupConfiguration, MachineConfiguration,
    MachineMode, PidLimits, PidParameters, PidTerm, SteamWandConfiguration, SteamWandControlState,
    TankConfiguration, WaterDispersalPumpStrategy, WaterTapConfiguration,
};

/// Persistent configuration for dual-boiler single-group machine
/// Uses nested leaf types from variegated-controller-types for clean structure
///
/// IMPORTANT: Structure must remain consistent regardless of feature flags for serialization compatibility
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DualBoilerSingleGroupPersistentConfiguration {
    // General configuration components (leaf types from variegated-controller-types)
    pub machine: MachineConfiguration,
    pub brew_boiler: BoilerConfiguration,
    pub steam_boiler: BoilerConfiguration,
    pub group: GroupConfiguration,
    pub water_tap: WaterTapConfiguration,
    pub tank: TankConfiguration,
    pub steam_wand: SteamWandConfiguration,

    // Default control states (what to reset to on restart)
    pub default_group_brew_control_state: GroupBrewControlState,
    pub default_steam_wand_control_state: SteamWandControlState,

    // Dual-boiler-specific fields (not in general Configuration)
    pub heating_element_contention_strategy: variegated_controller_types::HeatingElementContentionStrategy,
    pub allow_simultaneous_operations: bool,
    pub pump_tacho_pulses_per_liter: Option<f32>,
}

/// Ephemeral (runtime-only) configuration for dual-boiler single-group machine
/// Contains current state that resets to defaults on restart
///
/// IMPORTANT: Structure must remain consistent regardless of feature flags for serialization compatibility
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DualBoilerSingleGroupEphemeralConfiguration {
    pub mode: MachineMode,
    // Current runtime control states (reset to defaults on restart)
    pub group_brew_control_state: GroupBrewControlState,
    pub steam_wand_control_state: SteamWandControlState,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize, PartialEq)]
pub struct DualBoilerSingleGroupConfiguration {
    pub persistent: DualBoilerSingleGroupPersistentConfiguration,
    pub ephemeral: DualBoilerSingleGroupEphemeralConfiguration,
}

impl DualBoilerSingleGroupConfiguration {
    pub fn effective_brew_boiler_control_mode(&self) -> BoilerControlMode {
        if self.ephemeral.mode != MachineMode::On {
            BoilerControlMode::Off
        } else {
            self.persistent.brew_boiler.control_state.mode
        }
    }

    pub fn effective_steam_boiler_control_mode(&self) -> BoilerControlMode {
        if self.ephemeral.mode != MachineMode::On {
            BoilerControlMode::Off
        } else {
            self.persistent.steam_boiler.control_state.mode
        }
    }

    pub fn effective_group_brew_control_mode(&self) -> GroupBrewControlMode {
        if self.ephemeral.mode != MachineMode::On {
            GroupBrewControlMode::Off
        } else {
            self.ephemeral.group_brew_control_state.mode
        }
    }
}

impl<'a> Value<'a> for DualBoilerSingleGroupPersistentConfiguration {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        let crc = Crc::<u32>::new(&CRC_32_ISCSI);

        log_info!("Serializing DualBoilerSingleGroupConfiguration");

        let slice = match to_slice_crc32(self, buffer, crc.digest()) {
            Ok(bytes) => Ok(bytes.len()),
            Err(postcard::Error::SerializeBufferFull) => {
                log_warn!("Serialization buffer too small");

                Err(SerializationError::BufferTooSmall)
            },
            Err(_) => {
                log_warn!("Serialization error");

                Err(SerializationError::InvalidData)
            },
        };

        log_info!("Serialized DualBoilerSingleGroupConfiguration, len = {}", slice.clone().unwrap_or(0));

        slice
    }

    fn deserialize_from(buffer: &'a [u8]) -> Result<(Self, usize), SerializationError>
    where
        Self: Sized
    {
        log_info!("Deserializing configuration");

        let crc = Crc::<u32>::new(&CRC_32_ISCSI);

        let v = match from_bytes_crc32(buffer, crc.digest()) {
            Ok(value) => Ok(value),
            Err(postcard::Error::DeserializeUnexpectedEnd) => {
                log_warn!("Deserialization buffer too small");

                Err(SerializationError::InvalidFormat)
            },
            Err(postcard::Error::DeserializeBadEnum) => {
                log_warn!("Deserialization bad enum");

                Err(SerializationError::InvalidFormat)
            },
            Err(_) => {
                log_warn!("Deserialization error");
                Err(SerializationError::InvalidFormat)
            },
        };

        match v {
            Ok(value) => {
                log_info!("Deserialized configuration");
                // See `ScheduleItem`'s impl: the whole slice is consumed.
                Ok((value, buffer.len()))
            }
            Err(e) => {
                log_warn!("Deserialization failed");
                Err(e)
            }
        }
    }
}

/// Convert persistent configuration to general Configuration
/// This eliminates manual field mapping and ensures all fields are converted
impl From<DualBoilerSingleGroupPersistentConfiguration> for Configuration {
    fn from(persistent: DualBoilerSingleGroupPersistentConfiguration) -> Self {
        let mut configuration = Configuration::default();

        // Machine-wide config
        configuration.machine_config = persistent.machine;

        // Boiler configurations
        configuration.insert_boiler_configuration(
            BrewBoiler.as_index(),
            persistent.brew_boiler
        );
        configuration.insert_boiler_configuration(
            SteamBoiler.as_index(),
            persistent.steam_boiler
        );

        // Group configuration
        configuration.insert_group_configuration(
            SingleGroup.as_index(),
            persistent.group
        );

        // Water tap configuration
        configuration.insert_water_tap_configuration(0, persistent.water_tap);

        // Tank configuration
        configuration.insert_tank_configuration(0, persistent.tank);

        // Steam wand configuration - only insert if feature is enabled (behavior, not structure)
        #[cfg(feature = "pwm-steam-valve")]
        {
            configuration.insert_steam_wand_configuration(0, persistent.steam_wand);
        }
        // Note: steam_wand field exists in persistent config regardless of feature,
        // but we only use it if the feature is enabled

        configuration
    }
}

/// Also implement the reference version for efficiency
impl From<&DualBoilerSingleGroupPersistentConfiguration> for Configuration {
    fn from(persistent: &DualBoilerSingleGroupPersistentConfiguration) -> Self {
        persistent.clone().into()
    }
}

impl Default for DualBoilerSingleGroupEphemeralConfiguration {
    fn default() -> Self {
        Self {
            group_brew_control_state: GroupBrewControlState {
                mode: GroupBrewControlMode::FixedDutyCycle,
                limit: GroupBrewLimitMode::Unlimited,
                values: GroupBrewControlTargetValues {
                    duty_cycle: DutyCycleType::FULL,
                    ..GroupBrewControlTargetValues::default()
                },
            },
            mode: MachineMode::default(),
            steam_wand_control_state: SteamWandControlState::default(),
        }
    }
}

impl Default for DualBoilerSingleGroupPersistentConfiguration {
    fn default() -> Self {
        // Create PID parameters with sensible defaults
        let brew_boiler_temperature_params = PidParameters {
            kp: PidTerm::new(12.0, PidLimits::default()),
            ki: PidTerm::new(0.0003, PidLimits::new_with_limits(0.0, 30.0).unwrap()),
            kd: PidTerm::new(0.0, PidLimits::new_with_limits(-10.0, 10.0).unwrap())
        };
        let brew_boiler_pressure_params = PidParameters {
            kp: PidTerm::new(3.0, PidLimits::default()),
            ki: PidTerm::new(0.01, PidLimits::new_with_limits(-10.0, 10.0).unwrap()),
            kd: PidTerm::new(30.0, PidLimits::new_with_limits(-10.0, 10.0).unwrap())
        };
        let steam_boiler_temperature_params = PidParameters {
            kp: PidTerm::new(12.0, PidLimits::default()),
            ki: PidTerm::new(0.0003, PidLimits::new_with_limits(0.0, 30.0).unwrap()),
            kd: PidTerm::new(0.0, PidLimits::new_with_limits(-10.0, 10.0).unwrap())
        };
        let steam_boiler_pressure_params = PidParameters {
            kp: PidTerm::new(3.0, PidLimits::default()),
            ki: PidTerm::new(0.01, PidLimits::new_with_limits(-10.0, 10.0).unwrap()),
            kd: PidTerm::new(30.0, PidLimits::new_with_limits(-10.0, 10.0).unwrap())
        };
        // The three pump loops, on the pump's own 0-255 scale.
        //
        // **These were 0-100-era numbers until now.** When the pump was given its own 0-255
        // duty cycle the single-boiler controller's defaults were rescaled by 2.55 and
        // carry a comment saying so; these were missed, so a GS3 starting from defaults ran
        // its pump at about 1/2.55 of the intended authority. The figures below are stated
        // directly in duty points, which is why no scale factor appears.
        //
        // `kd` is zero throughout. A derivative term needs a measurement that is fresh and
        // quiet, and neither is true here: the pressure sensor is stale for ~2.2 loop
        // iterations and the group flow meter has a resolution floor around 0.36 ml/s.
        //
        // # Where the pressure gains come from
        //
        // Measured, not guessed: a hand-tuned set from a working machine. The kp asymmetry
        // is the interesting part -- 1.5x stronger when *over* target -- and it is there
        // because the pump can add pressure faster than the puck can shed it. On a declining
        // ramp the loop sits above its target for 63-79% of its samples, so the downward
        // direction is the one short of authority and gets the larger gain.
        let pump_pressure_params = PidParameters {
            kp: PidTerm::new_asymmetric(20.4, 30.6, PidLimits::default()),
            ki: PidTerm::new(0.0102, PidLimits::new_with_limits(-26.0, 128.0).unwrap()),
            kd: PidTerm::new(0.0, PidLimits::new_with_limits(-26.0, 26.0).unwrap())
        };
        // # Where the flow gains come from
        //
        // Derived from the pressure loop above rather than tuned, by holding the
        // dimensionless loop gain equal across the two: `kp * dProcess/dDuty`. Shot logs put
        // the pressure loop at 20.4 * 0.0611 = 1.25, and the group flow meter's process gain
        // at 0.0275 (ml/s) per duty point, which would match at kp = 45. That is derated to
        // 25 because flow is measured fresh every loop iteration where pressure is stale for
        // two, so the same static loop gain acts about twice as often -- and because flow is
        // the noisier signal of the two.
        //
        // `Ti = kp/ki` stays at the pressure loop's 2.0 s. It describes the pump and puck,
        // which are the same plant whichever quantity is being measured, so kp and ki scale
        // together.
        //
        // **Symmetric, unlike pressure.** The pump is positive-displacement: cut the duty
        // and flow falls immediately, with nothing stored to bleed off. The asymmetry above
        // exists for a problem flow does not have.
        let pump_flow_rate_params = PidParameters {
            kp: PidTerm::new(25.0, PidLimits::default()),
            ki: PidTerm::new(0.0125, PidLimits::new_with_limits(-26.0, 204.0).unwrap()),
            kd: PidTerm::new(0.0, PidLimits::new_with_limits(-26.0, 26.0).unwrap())
        };
        // Roughly half the group-flow gain, with `Ti` at twice the length.
        //
        // Not because the process gain differs -- measured, the two are the same to within
        // the shot-to-shot spread -- but because of *where the measurement sits*. The output
        // loop closes around strictly more plant: the puck, the drop transit and the scale's
        // own filtering are all inside it and outside the group-flow loop. Fluctuations in
        // group flow produce essentially no correlated response in scale-derived flow at any
        // lag, which is a path with almost no high-frequency transfer -- push it hard and the
        // actuator moves, the measurement does not answer, and the accumulated correction
        // arrives all at once. The long `Ti` is the more important half of this.
        let pump_output_flow_rate_params = PidParameters {
            kp: PidTerm::new(12.0, PidLimits::default()),
            ki: PidTerm::new(0.003, PidLimits::new_with_limits(-26.0, 204.0).unwrap()),
            kd: PidTerm::new(0.0, PidLimits::new_with_limits(-26.0, 26.0).unwrap())
        };

        Self {
            machine: MachineConfiguration {
                heating_element_interlock: false,
                max_shot_logs: 100,
                log_sample_decimation: 1,
                prevent_start_on_empty_tank: false,
                allow_continue_on_empty_tank: true,
            },
            brew_boiler: BoilerConfiguration {
                temperature_pid_parameters: brew_boiler_temperature_params,
                pressure_pid_parameters: brew_boiler_pressure_params,
                control_state: BoilerControlState {
                    mode: BoilerControlMode::Temperature,
                    values: BoilerControlTargetValues {
                        target_temperature: 93.0,
                        target_pressure: 1.0,
                    },
                },
                max_temperature: Some(105.0),
                max_pressure: Some(15.0),
                temperature_sensor_kalman_parameters: None,
                pressure_sensor_kalman_parameters: None,
                fill_config: None, // Brew boiler typically doesn't auto-fill
                supply_tank_index: Some(0),
                minimum_safe_level: None,
            },
            steam_boiler: BoilerConfiguration {
                temperature_pid_parameters: steam_boiler_temperature_params,
                pressure_pid_parameters: steam_boiler_pressure_params,
                control_state: BoilerControlState {
                    mode: BoilerControlMode::Temperature,
                    values: BoilerControlTargetValues {
                        target_temperature: 120.0,
                        target_pressure: 1.5,
                    },
                },
                max_temperature: Some(130.0),
                max_pressure: Some(2.0),
                temperature_sensor_kalman_parameters: None,
                pressure_sensor_kalman_parameters: None,
                fill_config: Some(FillConfiguration {
                    fill_threshold: Some(20),
                    pump_configuration: None,
                }),
                supply_tank_index: Some(0),
                minimum_safe_level: None,
            },
            group: GroupConfiguration {
                flow_rate_pid_parameters: pump_flow_rate_params,
                output_flow_rate_pid_parameters: pump_output_flow_rate_params,
                pressure_pid_parameters: pump_pressure_params,
                brew_control_state: GroupBrewControlState::default(), // Not used - see default_group_brew_control_state
                max_brew_time_seconds: None,
                auto_tare_enabled: false,
                pump_configuration: None,
                pressure_sensor_kalman_parameters: None,
                flow_sensor_pulses_per_liter: None,
                supply_tank_index: Some(0),
            },
            water_tap: WaterTapConfiguration {
                pump_strategy: WaterDispersalPumpStrategy::AlwaysPump(DutyCycleType::FULL),
                temperature_target: None,
                max_dispense_time_seconds: None,
                flow_rate_limit: None,
                pump_configuration: None,
                supply_tank_index: Some(0),
            },
            tank: TankConfiguration {
                low_level_warning_threshold: None,
                water_level_sensor_kalman_parameters: None,
                empty_threshold: None,
            },
            // Steam wand config - always present, defaults change based on feature
            steam_wand: {
                #[cfg(feature = "pwm-steam-valve")]
                {
                    SteamWandConfiguration {
                        temperature_target: None,
                        openness: Some(100),
                        purge_time_seconds: None,
                        max_steam_time_seconds: None,
                        auto_purge_enabled: false,
                        supply_tank_index: Some(0),
                    }
                }
                #[cfg(not(feature = "pwm-steam-valve"))]
                {
                    SteamWandConfiguration::default()
                }
            },
            // Default control states (what to reset to on restart)
            default_group_brew_control_state: GroupBrewControlState {
                mode: GroupBrewControlMode::FixedDutyCycle,
                limit: GroupBrewLimitMode::Unlimited,
                values: GroupBrewControlTargetValues {
                    duty_cycle: DutyCycleType::FULL,
                    ..GroupBrewControlTargetValues::default()
                },
            },
            // Steam wand default control state - always present, defaults change based on feature
            default_steam_wand_control_state: {
                #[cfg(feature = "pwm-steam-valve")]
                {
                    SteamWandControlState::default()
                }
                #[cfg(not(feature = "pwm-steam-valve"))]
                {
                    SteamWandControlState::default()
                }
            },
            // Machine-specific
            heating_element_contention_strategy: variegated_controller_types::HeatingElementContentionStrategy::default(),
            allow_simultaneous_operations: true,
            pump_tacho_pulses_per_liter: None,
        }
    }
}

impl Default for DualBoilerSingleGroupConfiguration {
    fn default() -> Self {
        let persistent = DualBoilerSingleGroupPersistentConfiguration::default();
        DualBoilerSingleGroupConfiguration {
            ephemeral: DualBoilerSingleGroupEphemeralConfiguration {
                mode: MachineMode::Off,
                // Initialize ephemeral control states from persistent defaults
                group_brew_control_state: persistent.default_group_brew_control_state,
                steam_wand_control_state: persistent.default_steam_wand_control_state,
            },
            persistent,
        }
    }
}
