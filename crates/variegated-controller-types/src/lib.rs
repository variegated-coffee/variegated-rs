#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

use heapless::index_map::FnvIndexMap;

// Constants
pub const MAX_BOILERS: usize = 8;
pub const MAX_GROUPS: usize = 4;
pub const MAX_WATER_TAPS: usize = 4;
pub const MAX_STEAM_WANDS: usize = 4;
pub const MAX_ENVIRONMENTAL_TEMPERATURE_SENSORS: usize = 2;
pub const MAX_TANKS: usize = 2;
pub const MAX_PERIPHERALS: usize = 16;
pub const MAX_FUNCTION_ROUTINES: usize = 16;

// Basic type aliases
pub type TemperatureType = f32; // Celsius
pub type PressureType = f32; // Bar
pub type WaterLevelType = u8; // Percent
pub type FlowRateType = f32; // ml/s
pub type InputVolumeType = f32; // ml
pub type WeightType = f32; // g
pub type WeightChangeType  = f32; // g/s
pub type FrequencyType = f32; // Hz
pub type RPMType = f32; // RPM
/// Percent, 0-100. See [`duty_cycle`] for why this is a newtype and
/// [`HexadecimalDutyCycleType`] for the scale the pump is actually driven with.
pub type DutyCycleType = DutyCycle;
/// 0-255, the pump's scale. See [`duty_cycle`].
pub type HexadecimalDutyCycleType = HexadecimalDutyCycle;
pub type ValveOpenType = u8; // Percent
pub type MixingProportionType = u8; // Percent
/// Electrical conductivity of what is leaving the group, **mS/cm**.
///
/// Coffee conducts and water essentially does not, which is what makes this a first-drop
/// detector as well as an extraction measure -- see `shot_state`'s `FIRST_DROP_EC`.
pub type ECType = f32;
/// Conductivity times output flow rate: **mS·ml/(cm·s)**.
///
/// Computed in the controllers, not measured. Note the fallback there: when a group reports
/// no *output* flow it substitutes *input* flow, which is a different quantity -- so this is
/// only as trustworthy as the group's flow sensing.
pub type ExtractionRateType = f32;
/// The time integral of [`ExtractionRateType`]: **mS·ml/cm**.
///
/// Accumulated over a brew. Beware the zero: the accumulator is initialised to `Some(0.0)`
/// on brew start whether or not the machine has a conductivity probe, so a value that stays
/// flat at zero means "no sensor", not "nothing extracted".
pub type ExtractedSolidsType = f32;
pub type OutputVolumeType = f32; // ml

// Index types
pub type BoilerIndex = u8;
pub type GroupIndex = u8;
pub type WaterTapIndex = u8;
pub type TankIndex = u8;
pub type SteamWandIndex = u8;

// PID type aliases
pub type PidParameters = variegated_control_algorithm::pid::PidParameters<f32>;
pub type PidTerm = variegated_control_algorithm::pid::PidTerm<f32>;
pub type PidLimits = variegated_control_algorithm::pid::Limits<f32>;

// Sensor and ID types
pub type ExternalSensorId = u8; // Unique identifier for external sensors
pub type EnvironmentalSensorId = u8; // Unique identifier for environmental sensors
pub type PeripheralId = u16; // Unique identifier for peripherals

// Protocol constants
pub const PROTOCOL_VERSION: ProtocolVersion = ProtocolVersion { major: 1, minor: 0 };

pub const PROTOCOL_CONFIG: ProtocolConfig = ProtocolConfig {
    protocol_version: PROTOCOL_VERSION,
    max_boilers: MAX_BOILERS,
    max_groups: MAX_GROUPS,
    max_water_taps: MAX_WATER_TAPS,
    max_tanks: MAX_TANKS,
    max_environmental_temperature_sensors: MAX_ENVIRONMENTAL_TEMPERATURE_SENSORS,
};

// Module declarations
pub mod bluetooth;
pub mod commands;
pub mod communication;
pub mod configuration;
pub mod control;
pub mod controller_variants;
pub mod debug;
pub mod debug_command;
pub mod duty_cycle;
pub mod machine_definition;
pub mod machine_mode;
pub mod peripherals;
pub mod routines;
pub mod schedule;
pub mod shot_log;
// `shot_state` is gone from here. It was an implementation -- a state machine with a
// threshold policy -- living in the crate that holds types, and it was put here only
// because `variegated-controller-lib` could not build for a host and so could not test
// it. That is fixed; see the module layout note at the top of that crate's `lib.rs`.
pub mod shot_upload;
pub mod status;
pub mod wifi;

// Re-exports for backward compatibility
pub use bluetooth::*;
pub use commands::*;
pub use communication::*;
pub use configuration::*;
pub use control::boiler::*;
pub use control::group::*;
pub use control::steam_wand::*;
pub use controller_variants::*;
pub use duty_cycle::*;
pub use machine_definition::*;
pub use machine_mode::*;
pub use peripherals::*;
pub use routines::core::*;
pub use routines::parameters::*;
pub use routines::steps::*;
pub use schedule::*;
pub use wifi::*;
pub use shot_log::*;
pub use shot_upload::*;
pub use status::*;
