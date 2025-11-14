use crate::*;

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug)]
pub enum PidParameterTarget {
    BoilerTemperature(BoilerIndex),
    BoilerPressure(BoilerIndex),
    GroupFlowRate(GroupIndex),
    GroupOutputFlowRate(GroupIndex),
    GroupPressure(GroupIndex),
}

/// The control mode for a boiler - what type of control is active
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum BoilerControlMode {
    Temperature,  // Control based on temperature
    Pressure,     // Control based on pressure
    #[default]
    Off          // No control - heater off
}

/// All stored target values for boiler control
/// These values persist regardless of current mode
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoilerControlTargetValues {
    pub target_temperature: TemperatureType,
    pub target_pressure: PressureType,
}

impl Default for BoilerControlTargetValues {
    fn default() -> Self {
        Self {
            target_temperature: 93.0,  // Default brew temperature
            target_pressure: 1.0,       // Default pressure in bar
        }
    }
}

/// Update structure for changing boiler target values
/// Only specified fields will be updated
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BoilerControlTargetValuesUpdate {
    pub temperature: Option<TemperatureType>,
    pub pressure: Option<PressureType>,
}

/// Complete boiler control state
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoilerControlState {
    pub mode: BoilerControlMode,
    pub values: BoilerControlTargetValues,
}

impl Default for BoilerControlState {
    fn default() -> Self {
        Self {
            mode: BoilerControlMode::Off,
            values: BoilerControlTargetValues::default(),
        }
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum WaterDispersalPumpStrategy {
    AlwaysPump(DutyCycleType),  // Pump at specified duty cycle percentage
    NoPump,
}

impl Default for WaterDispersalPumpStrategy {
    fn default() -> Self {
        WaterDispersalPumpStrategy::AlwaysPump(100)
    }
}

/// Strategy for resolving heating element contention when multiple boilers
/// request more combined duty cycle than available (>100% total)
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum HeatingElementContentionStrategy {
    /// Brew boiler gets full request, steam boiler gets remainder
    BrewPriority,
    /// Steam boiler gets full request, brew boiler gets remainder
    SteamPriority,
    /// Both scaled proportionally to fit in 100%
    Proportional,
}

impl Default for HeatingElementContentionStrategy {
    fn default() -> Self {
        Self::Proportional
    }
}
