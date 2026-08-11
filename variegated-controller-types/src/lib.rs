#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

use alloc::collections::BTreeMap;
use alloc::string::String;
use alloc::vec;
use alloc::vec::Vec;
use core::fmt;
use core::time::Duration;
use chrono::{DateTime, Datelike, NaiveDate, NaiveDateTime, Utc, Weekday};
use heapless::index_map::FnvIndexMap;
use heapless::index_set::FnvIndexSet;
use variegated_control_algorithm::pid::PidOut;

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
pub type DutyCycleType = u8; // Percent
pub type ValveOpenType = u8; // Percent
pub type MixingProportionType = u8; // Percent
pub type ECType = f32; // Electrical Conductivity
pub type ExtractionRateType = f32;
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
pub mod machine_definition;
pub mod machine_mode;
pub mod peripherals;
pub mod routines;
pub mod schedule;
pub mod shot_log;
pub mod shot_state;
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
pub use machine_definition::*;
pub use machine_mode::*;
pub use peripherals::*;
pub use routines::core::*;
pub use routines::parameters::*;
pub use routines::steps::*;
pub use schedule::*;
pub use wifi::*;
pub use shot_log::*;
pub use shot_state::*;
pub use status::*;
