//! The single-boiler machine's configuration, and how it reaches the wire.
//!
//! Split out of [`crate::single_boiler_single_group`] and **deliberately ungated**, where
//! the controller that owns it is behind `hardware`. Nothing here touches a peripheral: it
//! is four plain structs, their stored representation, their defaults, and the projection
//! onto the published [`Configuration`]. The controller re-exports all of it, so no caller
//! outside this crate changes.
//!
//! Ungating it is what makes the command handlers testable. [`crate::command`] operates on
//! these types through [`crate::command::ConfigurationAccess`], and a host test cannot
//! construct a value whose type only exists in a build carrying a Cortex-M PAC. The same
//! reasoning already put `single_boiler_state`, `pump_transfer` and `pump_limit` out here.

use crc::{Crc, CRC_32_ISCSI};
use postcard::{from_bytes_crc32, to_slice_crc32};
use sequential_storage::map::{SerializationError, Value};
use variegated_log::{log_info, log_warn};

use variegated_controller_types::SingleBoilerSingleGroupControllerBoilers::{
    BrewBoiler, VirtualSteamBoiler,
};
use variegated_controller_types::SingleGroupControllerGroups::SingleGroup;
use variegated_controller_types::{
    BoilerConfiguration, BoilerControlMode, BoilerControlState, BoilerControlTargetValues,
    BoilerIndex, Configuration, DutyCycleType, GroupBrewControlMode, GroupBrewControlState,
    GroupBrewControlTargetValues, GroupBrewLimitMode, GroupConfiguration, GroupIndex,
    KalmanParameters, MachineMode, PidLimits, PidParameterTarget, PidParameters, PidTerm,
    TankConfiguration,
};

#[derive(Clone, Copy, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SingleBoilerSingleGroupPidParameters {
    pub boiler_pressure_params: PidParameters,
    pub boiler_temperature_params: PidParameters,
    pub pump_flow_rate_params: PidParameters,
    pub pump_pressure_params: PidParameters,
    pub pump_output_flow_rate_params: PidParameters,
}

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SingleBoilerSingleGroupPersistentConfiguration {
    pub brew_boiler_control_state: BoilerControlState,
    pub steam_boiler_control_state: BoilerControlState,
    pub pid_parameters: SingleBoilerSingleGroupPidParameters,
    pub temperature_sensor_kalman_parameters: Option<KalmanParameters>,
    pub pressure_sensor_kalman_parameters: Option<KalmanParameters>,
    pub pump_tacho_pulses_per_liter: Option<f32>,
    pub flow_sensor_pulses_per_liter: Option<f32>,
    /// What `SetGroupPumpConfiguration` writes, and what `update_pump` clamps against.
    ///
    /// **Appended, and appending here reset this machine's stored configuration once.**
    /// postcard is positional and this blob carries no version, so an older stored value is a
    /// byte short of the new layout; `SettingsStorage::load_settings` maps that decode failure
    /// to `Default`. Every setpoint, PID tuning, Kalman parameter and pulses-per-litre
    /// calibration in this struct went back to its default on the flash that introduced this
    /// field, and had to be re-entered.
    ///
    /// That was a deliberate trade rather than an oversight: exactly one machine runs this
    /// firmware, so re-entering its settings once cost less than carrying a legacy decode path
    /// for the life of the struct. **On a fleet the trade goes the other way** -- either give
    /// the value a `settings::key` of its own, the way the timezone and the Bluetooth
    /// associations are stored, or write the migration.
    ///
    /// If another field is ever wanted here, add it in the same flash as something else that
    /// needs one: the migration surface is the cost, not the field.
    pub pump_configuration: Option<variegated_controller_types::PumpConfiguration>,
    /// What the machine does to the scale when a brew starts.
    ///
    /// **Appended, and appending here resets this machine's stored configuration once more**,
    /// for exactly the reasons the field above sets out — and on the terms its last paragraph
    /// asks for: one machine runs this firmware, and re-entering its settings once costs less
    /// than a legacy decode path.
    ///
    /// It cannot ride in `GroupConfiguration` the way it does on the GS3, which is where the
    /// asymmetry comes from: this firmware does not *store* a `GroupConfiguration` at all. It
    /// builds one for publication in `From<&SingleBoilerSingleGroupConfiguration>` below, so
    /// there is no existing byte here to reuse.
    pub brew_actions: variegated_controller_types::BrewActions,
}

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SingleBoilerSingleGroupEphemeralConfiguration {
    /// Whether the machine is switched on, as the user understands it.
    ///
    /// Ephemeral, like the dual-boiler's: a machine comes up off and is turned on, rather
    /// than remembering. Nothing here is written to flash, so adding this field cannot
    /// disturb stored settings.
    pub mode: MachineMode,
    pub group_brew_control_state: GroupBrewControlState,
}


#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SingleBoilerSingleGroupConfiguration {
    pub persistent: SingleBoilerSingleGroupPersistentConfiguration,
    pub ephemeral: SingleBoilerSingleGroupEphemeralConfiguration,
}

impl<'a> Value<'a> for SingleBoilerSingleGroupPersistentConfiguration {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        let crc = Crc::<u32>::new(&CRC_32_ISCSI);

        log_info!("Serializing SingleBoilerSingleGroupConfiguration");

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

        log_info!("Serialized SingleBoilerSingleGroupConfiguration, len = {}", slice.clone().unwrap_or(0));

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

impl Default for SingleBoilerSingleGroupEphemeralConfiguration {
    fn default() -> Self {
        Self {
            // Off, matching the dual-boiler. A machine that came up hot without anyone
            // asking would be the surprising choice, not this one.
            mode: MachineMode::Off,
            group_brew_control_state: GroupBrewControlState {
                mode: GroupBrewControlMode::FixedDutyCycle,
                limit: GroupBrewLimitMode::Unlimited,
                values: GroupBrewControlTargetValues {
                    duty_cycle: DutyCycleType::FULL,
                    ..GroupBrewControlTargetValues::default()
                },
            },
        }
    }
}

impl Default for SingleBoilerSingleGroupPersistentConfiguration {
    fn default() -> Self {
        let mut pid_parameters = SingleBoilerSingleGroupPidParameters::default();

        pid_parameters.boiler_temperature_params = PidParameters {
            kp: PidTerm::new(3.0, PidLimits::default()),
            ki: PidTerm::new(0.01, PidLimits::new_with_limits(-10.0, 10.0).unwrap()),
            kd: PidTerm::new(30.0, PidLimits::new_with_limits(-10.0, 10.0).unwrap())
        };
        // The pump gains below are the old 0-100 tuning multiplied by 2.55, because the pump
        // PID's output moved from a percentage to the pump's 0-255 scale and a gain is
        // denominated in output-per-error. The per-term clamps are scaled with them: a clamp
        // is a quantity of output, so leaving one behind would silently tighten it by the
        // same factor.
        //
        // **These are defaults, not a migration.** Gains already stored on a machine are
        // left exactly as they were, so a machine taking this update runs its pump at about
        // 1/2.55 of its previous authority until it is retuned. That is deliberate --
        // rescaling someone's tuning arithmetically assumes their tuning was linear in the
        // clamp, which is the thing least likely to be true of a hand-tuned loop.
        const PUMP_SCALE: f32 = 2.55;
        pid_parameters.pump_flow_rate_params = PidParameters {
            kp: PidTerm::new(10.0 * PUMP_SCALE, PidLimits::default() ),
            ki: PidTerm::new(0.01 * PUMP_SCALE, PidLimits::new_with_limits(-50.0 * PUMP_SCALE, 80.0 * PUMP_SCALE).unwrap() ),
            kd: PidTerm::new(30.0 * PUMP_SCALE, PidLimits::new_with_limits(-10.0 * PUMP_SCALE, 10.0 * PUMP_SCALE).unwrap() )
        };
        pid_parameters.pump_output_flow_rate_params = PidParameters {
            kp: PidTerm::new(10.0 * PUMP_SCALE, PidLimits::default() ),
            ki: PidTerm::new(0.01 * PUMP_SCALE, PidLimits::new_with_limits(-50.0 * PUMP_SCALE, 80.0 * PUMP_SCALE).unwrap() ),
            kd: PidTerm::new(30.0 * PUMP_SCALE, PidLimits::new_with_limits(-10.0 * PUMP_SCALE, 10.0 * PUMP_SCALE).unwrap() )
        };
        pid_parameters.pump_pressure_params = PidParameters {
            kp: PidTerm::new( 10.0 * PUMP_SCALE, PidLimits::default() ),
            ki: PidTerm::new( 0.01 * PUMP_SCALE, PidLimits::new_with_limits(-50.0 * PUMP_SCALE, 80.0 * PUMP_SCALE).unwrap() ),
            kd: PidTerm::new( 30.0 * PUMP_SCALE, PidLimits::new_with_limits(-10.0 * PUMP_SCALE, 10.0 * PUMP_SCALE).unwrap() )
        };

        SingleBoilerSingleGroupPersistentConfiguration {
            brew_boiler_control_state: BoilerControlState {
                mode: BoilerControlMode::Temperature,
                values: BoilerControlTargetValues {
                    target_temperature: 110.0,
                    target_pressure: 1.0,
                },
            },
            // Not a boiler: this is what the single element does once the machine is in
            // steam mode, so `Off` here means "entering steam mode stops the heating".
            // See `single_boiler_state::steam_boiler_state_or_default`, which also repairs
            // the machines that already have the old `Off` in flash.
            steam_boiler_control_state: BoilerControlState {
                mode: BoilerControlMode::Temperature,
                values: BoilerControlTargetValues {
                    target_temperature: crate::single_boiler_state::DEFAULT_STEAM_TARGET_TEMPERATURE,
                    ..BoilerControlTargetValues::default()
                },
            },
            pid_parameters,
            temperature_sensor_kalman_parameters: None,
            pressure_sensor_kalman_parameters: None,
            pump_tacho_pulses_per_liter: None,
            flow_sensor_pulses_per_liter: None,
            // Unconfigured, which `command::pump::apply_pump_limits` treats as unclamped.
            pump_configuration: None,
            // What this machine has always done at brew start, now said out loud. Unlike the
            // GS3 there is no legacy byte to inherit here -- the append resets this blob, so
            // every machine starts from this default rather than from anything stored.
            brew_actions: variegated_controller_types::BrewActions::TARE,
        }
    }
}

impl Default for SingleBoilerSingleGroupConfiguration {
    fn default() -> Self {
        SingleBoilerSingleGroupConfiguration {
            persistent: SingleBoilerSingleGroupPersistentConfiguration::default(),
            ephemeral: SingleBoilerSingleGroupEphemeralConfiguration::default(),
        }
    }
}

impl crate::command::ConfigurationAccess for SingleBoilerSingleGroupConfiguration {
    fn boiler_control_state_mut(&mut self, index: BoilerIndex) -> Option<&mut BoilerControlState> {
        match index {
            0 => Some(&mut self.persistent.brew_boiler_control_state),
            // Not a second boiler: this is what the one heating element does once the
            // machine is in steam mode. See `crate::single_boiler_state`.
            1 => Some(&mut self.persistent.steam_boiler_control_state),
            _ => None,
        }
    }

    fn group_brew_control_state_mut(
        &mut self,
        index: GroupIndex,
    ) -> Option<&mut GroupBrewControlState> {
        match index {
            0 => Some(&mut self.ephemeral.group_brew_control_state),
            _ => None,
        }
    }

    /// Directly on the persistent blob, not in a `GroupConfiguration` -- this machine does not
    /// store one. See the field's own note for why that asymmetry with the dual-boiler exists.
    fn brew_actions_mut(
        &mut self,
        index: GroupIndex,
    ) -> Option<&mut variegated_controller_types::BrewActions> {
        match index {
            0 => Some(&mut self.persistent.brew_actions),
            _ => None,
        }
    }

    /// **The boiler index is deliberately ignored, and that is load-bearing.**
    ///
    /// There is one heating element and one tuning, so both boiler indices resolve to the
    /// same slot. Answering index 1 with a tuning of its own is not a harmless improvement:
    /// the ESPHome bridge exposes kP/kI/kD as Home Assistant numbers for every boiler
    /// declaring `TemperaturePid` -- which the virtual steam boiler does -- and writing one
    /// reads back the *published* parameters, edits a single term and returns the whole
    /// struct. With the tunings shared, that round-trips. With them split, nudging "Virtual
    /// Steam kP" would write an all-zero PID over the real one and save it to flash, which
    /// is a machine that stops heating. See the matching comment in the `From` impl below.
    fn pid_parameters_mut(&mut self, target: PidParameterTarget) -> Option<&mut PidParameters> {
        let params = &mut self.persistent.pid_parameters;
        Some(match target {
            PidParameterTarget::BoilerPressure(_) => &mut params.boiler_pressure_params,
            PidParameterTarget::BoilerTemperature(_) => &mut params.boiler_temperature_params,
            PidParameterTarget::GroupFlowRate(_) => &mut params.pump_flow_rate_params,
            PidParameterTarget::GroupPressure(_) => &mut params.pump_pressure_params,
            PidParameterTarget::GroupOutputFlowRate(_) => &mut params.pump_output_flow_rate_params,
        })
    }
}

impl From<SingleBoilerSingleGroupConfiguration> for Configuration {
    fn from(config: SingleBoilerSingleGroupConfiguration) -> Self {
        let mut configuration = Configuration::default();

        // Add brew boiler configuration
        let brew_boiler_config = BoilerConfiguration {
            temperature_pid_parameters: config.persistent.pid_parameters.boiler_temperature_params.clone(),
            pressure_pid_parameters: config.persistent.pid_parameters.boiler_pressure_params.clone(),
            control_state: config.persistent.brew_boiler_control_state,
            // Same constant the interlock in `update_boiler` enforces, so what the interface
            // shows and what the machine does cannot part company.
            max_temperature: Some(crate::single_boiler_state::MAX_BREW_TEMPERATURE),
            max_pressure: Some(15.0),
            // Embedded sensor configuration
            temperature_sensor_kalman_parameters: None,
            pressure_sensor_kalman_parameters: None,
            // No fill pump for single boiler
            fill_config: None,
            supply_tank_index: None,
            minimum_safe_level: None, // Not stored in persistent config
        };
        configuration.insert_boiler_configuration(BrewBoiler.as_index(), brew_boiler_config);

        // Add virtual steam boiler configuration
        let steam_boiler_config = BoilerConfiguration {
            // The same parameters as the brew boiler, because there is one element and one
            // tuning. Publishing `PidParameters::default()` here was not merely
            // uninformative, it was a way to stop the machine heating: the ESPHome bridge
            // exposes kP/kI/kD as Home Assistant numbers for every boiler declaring
            // `TemperaturePid` -- which this one does (`main.rs`) -- and writing one of them
            // reads back the *published* parameters, changes a single term and returns the
            // whole struct (`esphome/command_mapper.rs`). `SetPidParameters` then ignores
            // the boiler index and writes the shared tuning, so nudging "Virtual Steam kP"
            // replaced a tuned PID with an all-zero one and saved it to flash. The scales
            // are what did it; the default limits are infinite.
            temperature_pid_parameters: config.persistent.pid_parameters.boiler_temperature_params.clone(),
            pressure_pid_parameters: config.persistent.pid_parameters.boiler_pressure_params.clone(),
            control_state: config.persistent.steam_boiler_control_state,
            max_temperature: Some(crate::single_boiler_state::MAX_STEAM_TEMPERATURE),
            max_pressure: Some(3.0),
            // Embedded sensor configuration
            temperature_sensor_kalman_parameters: None,
            pressure_sensor_kalman_parameters: None,
            // No fill pump for virtual steam boiler
            fill_config: None,
            supply_tank_index: None,
            minimum_safe_level: None, // Virtual steam boiler shares the same physical boiler
        };
        configuration.insert_boiler_configuration(VirtualSteamBoiler.as_index(), steam_boiler_config);

        // Add group configuration
        let group_config = GroupConfiguration {
            flow_rate_pid_parameters: config.persistent.pid_parameters.pump_flow_rate_params.clone(),
            output_flow_rate_pid_parameters: config.persistent.pid_parameters.pump_output_flow_rate_params.clone(),
            pressure_pid_parameters: config.persistent.pid_parameters.pump_pressure_params.clone(),
            brew_control_state: config.ephemeral.group_brew_control_state,
            max_brew_time_seconds: Some(300), // 5 minutes max brew time
            // Published from storage now rather than hard-coded `true`, for the reason
            // `pump_configuration` below gives about itself: a value the machine acts on but
            // never publishes is one the UI shows as unset however many times it is written.
            brew_actions: config.persistent.brew_actions,
            // Published now rather than hard-coded `None`: the browser's pump settings page
            // reads this, and a value the machine stores but never publishes is one the UI
            // shows as unset however many times it is written.
            pump_configuration: config.persistent.pump_configuration.clone(),
            pressure_sensor_kalman_parameters: None,
            flow_sensor_pulses_per_liter: None,
            supply_tank_index: None,
        };
        configuration.insert_group_configuration(SingleGroup.as_index(), group_config);

        // Add tank configuration
        let tank_config = TankConfiguration {
            low_level_warning_threshold: Some(20),
            water_level_sensor_kalman_parameters: None,
            empty_threshold: None,
        };
        configuration.insert_tank_configuration(0, tank_config);

        configuration
    }
}
