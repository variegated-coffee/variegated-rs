use crate::*;
use alloc::string::String;
use alloc::vec::Vec;
use core::fmt;
use heapless::FnvIndexMap;

pub type UserActionIndex = u8;
pub type RoutineParameters = FnvIndexMap<u8, f32, 8>;

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum RoutineIndex {
    Internal(usize),
    Function(usize),
    Custom(usize)
}

impl RoutineIndex {
    /// Convert RoutineIndex to a bit-packed u16 storage index
    /// Bits 15-14: Type (00=Internal, 01=Function, 10=Custom)
    /// Bits 13-0: Index (0-16383)
    pub fn to_storage_index(&self) -> u16 {
        match self {
            RoutineIndex::Internal(n) => {
                debug_assert!(*n < 0x4000, "Internal routine index too large");
                (*n as u16) & 0x3FFF
            }
            RoutineIndex::Function(n) => {
                debug_assert!(*n < 0x4000, "Function routine index too large");
                0x4000 | ((*n as u16) & 0x3FFF)
            }
            RoutineIndex::Custom(n) => {
                debug_assert!(*n < 0x4000, "Custom routine index too large");
                0x8000 | ((*n as u16) & 0x3FFF)
            }
        }
    }

    /// Convert a bit-packed u16 storage index to RoutineIndex
    pub fn from_storage_index(storage_index: u16) -> Option<Self> {
        let type_bits = (storage_index >> 14) & 0x03;
        let index = (storage_index & 0x3FFF) as usize;

        match type_bits {
            0b00 => Some(RoutineIndex::Internal(index)),
            0b01 => Some(RoutineIndex::Function(index)),
            0b10 => Some(RoutineIndex::Custom(index)),
            _ => None, // 0b11 is reserved
        }
    }

    /// Get the inner index value
    pub fn inner(&self) -> usize {
        match self {
            RoutineIndex::Internal(n) | RoutineIndex::Function(n) | RoutineIndex::Custom(n) => *n,
        }
    }
}

impl core::fmt::Display for RoutineIndex {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            RoutineIndex::Internal(n) => write!(f, "Internal({})", n),
            RoutineIndex::Function(n) => write!(f, "Function({})", n),
            RoutineIndex::Custom(n) => write!(f, "Custom({})", n),
        }
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Copy, Debug)]
pub enum ParameterValue {
    Static(f32),
    Parameter(u8), // index into parameter map
    DerivedParameter(u8), // index into derived parameter list (separate namespace)
}

impl fmt::Display for ParameterValue {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ParameterValue::Static(value) => write!(f, "{:.1}", value),
            ParameterValue::Parameter(index) => write!(f, "P{}", index),
            ParameterValue::DerivedParameter(index) => write!(f, "D{}", index),
        }
    }
}

impl ParameterValue {
    // For display purposes, treat Parameter as a placeholder value
    pub fn as_secs(&self) -> u64 {
        match self {
            ParameterValue::Static(value) => *value as u64,
            ParameterValue::Parameter(_) => 0, // Placeholder - should be resolved
            ParameterValue::DerivedParameter(_) => 0, // Placeholder - should be resolved
        }
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ParameterUnit {
    Seconds,
    Celsius,
    Bar,
    MillilitersPerSecond,
    Grams,
    Percent,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Debug)]
pub struct RoutineParameter {
    pub index: u8,
    pub name: String,  // User-facing, e.g. "Preinfusion Time", "Target Pressure"
    pub default: f32,
    pub unit: Option<ParameterUnit>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Debug)]
pub struct DerivedParameter {
    pub index: u8,  // Separate index space from regular parameters
    pub name: String,
    pub unit: Option<ParameterUnit>,
    pub formula: DerivedFormula,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Debug)]
pub enum DerivedFormula {
    Linear {
        base_param: u8,      // Index of base parameter
        multiplier: f32,
        offset: f32,
    },
    Sum {
        params: Vec<u8>,     // Indices of parameters to sum
    },
    Difference {
        param_a: u8,
        param_b: u8,         // a - b
    },
    Product {
        params: Vec<u8>,     // Indices of parameters to multiply
    },
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Copy, Debug)]
pub enum StateCondition {
    Brewing(GroupIndex),
    NotBrewing(GroupIndex),
    BoilerTemperatureAbove(BoilerIndex, ParameterValue),
    BoilerTemperatureBelow(BoilerIndex, ParameterValue),
    BoilerPressureAbove(BoilerIndex, ParameterValue),
    BoilerPressureBelow(BoilerIndex, ParameterValue),
    GroupInputFlowRateAbove(GroupIndex, ParameterValue),
    GroupInputFlowRateBelow(GroupIndex, ParameterValue),
    GroupPressureAbove(GroupIndex, ParameterValue),
    GroupPressureBelow(GroupIndex, ParameterValue),
    WaterTapFlowRateAbove(WaterTapIndex, ParameterValue),
    WaterTapFlowRateBelow(WaterTapIndex, ParameterValue),
    OutputWeightAbove(GroupIndex, ParameterValue),
    OutputWeightBelow(GroupIndex, ParameterValue),
    InputVolumeAboveRelativeToStart(GroupIndex, ParameterValue),
}
