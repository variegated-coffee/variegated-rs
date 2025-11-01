use crate::*;
use heapless::FnvIndexMap;

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug)]
pub struct ProtocolVersion {
    /// Major version of the protocol. Incremented for breaking changes.
    pub major: u8,
    /// Minor version of the protocol. Incremented for non-breaking changes.
    pub minor: u8,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug)]
pub struct ProtocolConfig {
    /// The protocol version.
    pub protocol_version: ProtocolVersion,
    /// The maximum number of boilers supported by this compile of the protocol.
    pub max_boilers: usize,
    /// The maximum number of groups supported by this compile of the protocol.
    pub max_groups: usize,
    /// The maximum number of water taps supported by this compile of the protocol.
    pub max_water_taps: usize,
    /// The maximum number of tanks supported by this compile of the protocol.
    pub max_tanks: usize,
    /// The maximum number of environmental temperature sensors supported by this compile of the protocol.
    pub max_environmental_temperature_sensors: usize,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MachineType {
    SingleBoilerSingleGroup,
    DualBoilerSingleGroup,
    DualBoilerDualGroup,
}

impl Default for MachineType {
    fn default() -> Self {
        MachineType::SingleBoilerSingleGroup
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SensorCapability {
    Temperature,
    Pressure,
    WaterLevel,
    InputFlowRate,
    OutputFlowRate,
    Weight,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ActuatorCapability {
    HeatingElement,
    Pump,
    SolenoidValve,
    ThreeWayValve,
    WaterMixer,
    ScaleTare,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ControlModeCapability {
    TemperaturePid,
    PressurePid,
    FlowRatePid,
    OutputFlowRatePid,
    FixedDutyCycle,
    FullOn,
    Off,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum BoilerType {
    BrewBoiler,
    SteamBoiler,
    VirtualSteamBoiler,
}

impl Default for BoilerType {
    fn default() -> Self {
        BoilerType::BrewBoiler
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Debug)]
pub struct BoilerDefinition {
    #[cfg_attr(feature = "schemars", schemars(with = "String"))]
    pub name: heapless::String<32>,
    pub boiler_type: BoilerType,
    #[cfg_attr(feature = "schemars", schemars(with = "std::vec::Vec<SensorCapability>"))]
    pub sensors: heapless::Vec<SensorCapability, 8>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::vec::Vec<ActuatorCapability>"))]
    pub actuators: heapless::Vec<ActuatorCapability, 8>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::vec::Vec<ControlModeCapability>"))]
    pub control_modes: heapless::Vec<ControlModeCapability, 8>,
    pub has_fill_mechanism: bool,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Debug)]
pub struct GroupDefinition {
    #[cfg_attr(feature = "schemars", schemars(with = "String"))]
    pub name: heapless::String<32>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::vec::Vec<SensorCapability>"))]
    pub sensors: heapless::Vec<SensorCapability, 8>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::vec::Vec<ActuatorCapability>"))]
    pub actuators: heapless::Vec<ActuatorCapability, 8>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::vec::Vec<ControlModeCapability>"))]
    pub control_modes: heapless::Vec<ControlModeCapability, 8>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Debug)]
pub struct WaterTapDefinition {
    #[cfg_attr(feature = "schemars", schemars(with = "String"))]
    pub name: heapless::String<32>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::vec::Vec<SensorCapability>"))]
    pub sensors: heapless::Vec<SensorCapability, 8>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::vec::Vec<ActuatorCapability>"))]
    pub actuators: heapless::Vec<ActuatorCapability, 8>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::vec::Vec<ControlModeCapability>"))]
    pub control_modes: heapless::Vec<ControlModeCapability, 8>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Debug)]
pub struct SteamWandDefinition {
    #[cfg_attr(feature = "schemars", schemars(with = "String"))]
    pub name: heapless::String<32>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::vec::Vec<SensorCapability>"))]
    pub sensors: heapless::Vec<SensorCapability, 8>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::vec::Vec<ActuatorCapability>"))]
    pub actuators: heapless::Vec<ActuatorCapability, 8>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::vec::Vec<ControlModeCapability>"))]
    pub control_modes: heapless::Vec<ControlModeCapability, 8>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Debug)]
pub struct TankDefinition {
    #[cfg_attr(feature = "schemars", schemars(with = "String"))]
    pub name: heapless::String<32>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::vec::Vec<SensorCapability>"))]
    pub sensors: heapless::Vec<SensorCapability, 8>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Debug)]
pub struct PeripheralDefinition {
    pub peripheral_type: PeripheralType,
    #[cfg_attr(feature = "schemars", schemars(with = "String"))]
    pub location: heapless::String<32>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::vec::Vec<SensorCapability>"))]
    pub capabilities: heapless::Vec<SensorCapability, 8>,
    pub support_calibration: bool,
    pub via_comms_mcu: bool,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EnvironmentalSensorType {
    AmbientTemperature,
    CaseTemperature,
    ExternalTemperature,
    Humidity,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Debug)]
pub struct EnvironmentalSensorDefinition {
    #[cfg_attr(feature = "schemars", schemars(with = "String"))]
    pub name: heapless::String<32>,
    pub sensor_type: EnvironmentalSensorType,
    pub measurement_range: Option<(f32, f32)>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Debug)]
pub struct MachineDefinition {
    #[cfg_attr(feature = "schemars", schemars(with = "String"))]
    pub name: heapless::String<32>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::collections::HashMap<BoilerIndex, BoilerDefinition>"))]
    pub boilers: FnvIndexMap<BoilerIndex, BoilerDefinition, MAX_BOILERS>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::collections::HashMap<GroupIndex, GroupDefinition>"))]
    pub groups: FnvIndexMap<GroupIndex, GroupDefinition, MAX_GROUPS>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::collections::HashMap<WaterTapIndex, WaterTapDefinition>"))]
    pub water_taps: FnvIndexMap<WaterTapIndex, WaterTapDefinition, MAX_WATER_TAPS>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::collections::HashMap<TankIndex, TankDefinition>"))]
    pub tanks: FnvIndexMap<TankIndex, TankDefinition, MAX_TANKS>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::collections::HashMap<SteamWandIndex, SteamWandDefinition>"))]
    pub steam_wands: FnvIndexMap<SteamWandIndex, SteamWandDefinition, 4>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::collections::HashMap<EnvironmentalSensorId, EnvironmentalSensorDefinition>"))]
    pub environmental_sensors: FnvIndexMap<EnvironmentalSensorId, EnvironmentalSensorDefinition, MAX_ENVIRONMENTAL_TEMPERATURE_SENSORS>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::collections::HashMap<PeripheralId, PeripheralDefinition>"))]
    pub peripherals: FnvIndexMap<PeripheralId, PeripheralDefinition, MAX_PERIPHERALS>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::collections::HashMap<usize, String>"))]
    pub function_routines: FnvIndexMap<usize, heapless::String<32>, MAX_FUNCTION_ROUTINES>,
}

 impl MachineDefinition {
    pub fn add_boiler(&mut self, index: BoilerIndex, definition: BoilerDefinition) -> Result<(), ()> {
        self.boilers.insert(index, definition).map(|_| ()).map_err(|_| ())
    }

    pub fn add_group(&mut self, index: GroupIndex, definition: GroupDefinition) -> Result<(), ()> {
        self.groups.insert(index, definition).map(|_| ()).map_err(|_| ())
    }

    pub fn add_water_tap(&mut self, index: WaterTapIndex, definition: WaterTapDefinition) -> Result<(), ()> {
        self.water_taps.insert(index, definition).map(|_| ()).map_err(|_| ())
    }

    pub fn add_tank(&mut self, index: TankIndex, definition: TankDefinition) -> Result<(), ()> {
        self.tanks.insert(index, definition).map(|_| ()).map_err(|_| ())
    }

    pub fn add_peripheral(&mut self, id: PeripheralId, definition: PeripheralDefinition) -> Result<(), ()> {
        self.peripherals.insert(id, definition).map(|_| ()).map_err(|_| ())
    }

    pub fn add_steam_wand(&mut self, index: u8, definition: SteamWandDefinition) -> Result<(), ()> {
        self.steam_wands.insert(index, definition).map(|_| ()).map_err(|_| ())
    }

    pub fn add_environmental_sensor(&mut self, id: EnvironmentalSensorId, definition: EnvironmentalSensorDefinition) -> Result<(), ()> {
        self.environmental_sensors.insert(id, definition).map(|_| ()).map_err(|_| ())
    }

    pub fn add_function_routine_description(&mut self, index: usize, description: &str) -> Result<(), ()> {
        let s = heapless::String::try_from(description).map_err(|_| ())?;
        self.function_routines.insert(index, s).map(|_| ()).map_err(|_| ())
    }

    pub fn get_function_routine_description(&self, index: usize) -> Option<&str> {
        self.function_routines.get(&index).map(|s| s.as_str())
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for MachineDefinition {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "MachineDefinition {{ boilers: {} boilers, groups: {} groups, water_taps: {} water_taps, tanks: {} tanks, steam_wands: {} wands, env_sensors: {} sensors, peripherals: {} peripherals, function_routines: {} function routines }}",
            self.boilers.len(),
            self.groups.len(),
            self.water_taps.len(),
            self.tanks.len(),
            self.steam_wands.len(),
            self.environmental_sensors.len(),
            self.peripherals.len(),
            self.function_routines.len()
        );
    }
}
