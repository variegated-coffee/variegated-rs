use crate::*;
use heapless::index_map::FnvIndexMap;

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug)]
pub struct ProtocolVersion {
    /// Major version of the protocol. Incremented for breaking changes.
    pub major: u8,
    /// Minor version of the protocol. Incremented for non-breaking changes.
    pub minor: u8,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
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
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SensorCapability {
    Temperature,
    Pressure,
    WaterLevel,
    InputFlowRate,
    OutputFlowRate,
    Weight,
    ElectricalConductivity
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
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
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
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
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug)]
pub struct BoilerDefinition {
    pub name: heapless::String<32>,
    pub boiler_type: BoilerType,
    pub sensors: heapless::Vec<SensorCapability, 8>,
    pub actuators: heapless::Vec<ActuatorCapability, 8>,
    pub control_modes: heapless::Vec<ControlModeCapability, 8>,
    pub has_fill_mechanism: bool,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug)]
pub struct GroupDefinition {
    pub name: heapless::String<32>,
    pub sensors: heapless::Vec<SensorCapability, 8>,
    pub actuators: heapless::Vec<ActuatorCapability, 8>,
    pub control_modes: heapless::Vec<ControlModeCapability, 8>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug)]
pub struct WaterTapDefinition {
    pub name: heapless::String<32>,
    pub sensors: heapless::Vec<SensorCapability, 8>,
    pub actuators: heapless::Vec<ActuatorCapability, 8>,
    pub control_modes: heapless::Vec<ControlModeCapability, 8>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug)]
pub struct SteamWandDefinition {
    pub name: heapless::String<32>,
    pub sensors: heapless::Vec<SensorCapability, 8>,
    pub actuators: heapless::Vec<ActuatorCapability, 8>,
    pub control_modes: heapless::Vec<ControlModeCapability, 8>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug)]
pub struct TankDefinition {
    pub name: heapless::String<32>,
    pub sensors: heapless::Vec<SensorCapability, 8>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug)]
pub struct PeripheralDefinition {
    pub peripheral_type: PeripheralType,
    pub location: heapless::String<32>,
    pub capabilities: heapless::Vec<SensorCapability, 8>,
    pub support_calibration: bool,
    pub via_comms_mcu: bool,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EnvironmentalSensorType {
    AmbientTemperature,
    CaseTemperature,
    ExternalTemperature,
    Humidity,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug)]
pub struct EnvironmentalSensorDefinition {
    pub name: heapless::String<32>,
    pub sensor_type: EnvironmentalSensorType,
    pub measurement_range: Option<(f32, f32)>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone, Debug)]
pub struct MachineDefinition {
    pub name: heapless::String<32>,
    pub boilers: FnvIndexMap<BoilerIndex, BoilerDefinition, MAX_BOILERS>,
    pub groups: FnvIndexMap<GroupIndex, GroupDefinition, MAX_GROUPS>,
    pub water_taps: FnvIndexMap<WaterTapIndex, WaterTapDefinition, MAX_WATER_TAPS>,
    pub tanks: FnvIndexMap<TankIndex, TankDefinition, MAX_TANKS>,
    pub steam_wands: FnvIndexMap<SteamWandIndex, SteamWandDefinition, 4>,
    pub environmental_sensors: FnvIndexMap<EnvironmentalSensorId, EnvironmentalSensorDefinition, MAX_ENVIRONMENTAL_TEMPERATURE_SENSORS>,
    pub peripherals: FnvIndexMap<PeripheralId, PeripheralDefinition, MAX_PERIPHERALS>,
    pub function_routines: FnvIndexMap<u32, heapless::String<32>, MAX_FUNCTION_ROUTINES>,
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

    pub fn add_function_routine_description(&mut self, index: u32, description: &str) -> Result<(), ()> {
        let s = heapless::String::try_from(description).map_err(|_| ())?;
        self.function_routines.insert(index, s).map(|_| ()).map_err(|_| ())
    }

    pub fn get_function_routine_description(&self, index: u32) -> Option<&str> {
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
