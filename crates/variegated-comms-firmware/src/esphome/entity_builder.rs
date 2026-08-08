use alloc::boxed::Box;
use alloc::format;
use alloc::string::ToString;
use variegated_log::log_warn;
use alloc::vec;
use alloc::vec::Vec;
use esphome_device::{BinarySensorConfig, EntityConfig};
use esphome_device::entity_type::switch::SwitchConfig;
use esphome_device::entity_type::select::SelectConfig;
use esphome_device::api::{EntityCategory, SensorStateClass, NumberMode};
use esphome_device::entity_type::sensor::SensorConfig;
use esphome_device::entity_type::number::NumberConfig;
use variegated_controller_types::{Configuration, BoilerIndex, GroupIndex, WaterTapIndex, TankIndex, SteamWandIndex, BoilerConfiguration, GroupConfiguration, WaterTapConfiguration, TankConfiguration, SteamWandConfiguration, MachineDefinition, Status, SensorCapability, ActuatorCapability, ControlModeCapability, BoilerControlMode};

// Entity type constants for key generation
const ENTITY_TYPE_BOILER: u32 = 0x01;
const ENTITY_TYPE_GROUP: u32 = 0x02;
const ENTITY_TYPE_WATER_TAP: u32 = 0x03;
const ENTITY_TYPE_STEAM_WAND: u32 = 0x04;
const ENTITY_TYPE_MACHINE: u32 = 0x05;
const ENTITY_TYPE_TANK: u32 = 0x06;

// Boiler property constants
const BOILER_TEMPERATURE: u16 = 0x0001;
const BOILER_PRESSURE: u16 = 0x0002;
const BOILER_WATER_LEVEL: u16 = 0x0003;
const BOILER_TEMP_SETPOINT: u16 = 0x0004;
const BOILER_PRESSURE_SETPOINT: u16 = 0x0005;
const BOILER_TEMP_KP: u16 = 0x0006;
const BOILER_TEMP_KI: u16 = 0x0007;
const BOILER_TEMP_KD: u16 = 0x0008;
const BOILER_PRESSURE_KP: u16 = 0x0009;
const BOILER_PRESSURE_KI: u16 = 0x000A;
const BOILER_PRESSURE_KD: u16 = 0x000B;
const BOILER_DUTY_CYCLE: u16 = 0x000C;
const BOILER_MAX_TEMPERATURE: u16 = 0x000D;
const BOILER_MAX_PRESSURE: u16 = 0x000E;
#[allow(dead_code)]
const BOILER_TEMP_KALMAN_PROCESS_NOISE: u16 = 0x000F;
#[allow(dead_code)]
const BOILER_TEMP_KALMAN_MEASUREMENT_NOISE: u16 = 0x0010;
#[allow(dead_code)]
const BOILER_TEMP_KALMAN_ESTIMATED_ERROR: u16 = 0x0011;
#[allow(dead_code)]
const BOILER_TEMP_KALMAN_POSTERIOR_ESTIMATE: u16 = 0x0012;
#[allow(dead_code)]
const BOILER_PRESSURE_KALMAN_PROCESS_NOISE: u16 = 0x0013;
#[allow(dead_code)]
const BOILER_PRESSURE_KALMAN_MEASUREMENT_NOISE: u16 = 0x0014;
#[allow(dead_code)]
const BOILER_PRESSURE_KALMAN_ESTIMATED_ERROR: u16 = 0x0015;
#[allow(dead_code)]
const BOILER_PRESSURE_KALMAN_POSTERIOR_ESTIMATE: u16 = 0x0016;
const BOILER_FILL_THRESHOLD: u16 = 0x0017;
const BOILER_CONTROL_MODE: u16 = 0x001D;
#[allow(dead_code)]
const BOILER_FILL_PUMP_MAX_DUTY_CYCLE: u16 = 0x0018;
#[allow(dead_code)]
const BOILER_FILL_PUMP_MIN_DUTY_CYCLE: u16 = 0x0019;
#[allow(dead_code)]
const BOILER_FILL_PUMP_RAMP_UP_TIME: u16 = 0x001A;
#[allow(dead_code)]
const BOILER_FILL_PUMP_RAMP_DOWN_TIME: u16 = 0x001B;
#[allow(dead_code)]
const BOILER_FILL_PUMP_TACHO_PULSES_PER_LITER: u16 = 0x001C;
const BOILER_PID_P_TERM: u16 = 0x001D;
const BOILER_PID_I_TERM: u16 = 0x001E;
const BOILER_PID_D_TERM: u16 = 0x001F;
const BOILER_TEMP_KP_UPPER_LIMIT: u16 = 0x0020;
const BOILER_TEMP_KP_LOWER_LIMIT: u16 = 0x0021;
const BOILER_TEMP_KI_UPPER_LIMIT: u16 = 0x0022;
const BOILER_TEMP_KI_LOWER_LIMIT: u16 = 0x0023;
const BOILER_TEMP_KD_UPPER_LIMIT: u16 = 0x0024;
const BOILER_TEMP_KD_LOWER_LIMIT: u16 = 0x0025;

// Group property constants
const GROUP_INPUT_FLOW_RATE: u16 = 0x0001;
const GROUP_OUTPUT_FLOW_RATE: u16 = 0x0002;
#[allow(dead_code)]
const GROUP_PRESSURE: u16 = 0x0003;
const GROUP_WEIGHT: u16 = 0x0004;
const GROUP_BREW_TIME: u16 = 0x0005;
const GROUP_IS_BREWING: u16 = 0x0006;
const GROUP_PUMP_DUTY_CYCLE: u16 = 0x0007;
const GROUP_FLOW_RATE_SETPOINT: u16 = 0x0008;
const GROUP_OUTPUT_FLOW_RATE_SETPOINT: u16 = 0x0009;
#[allow(dead_code)]
const GROUP_PRESSURE_SETPOINT: u16 = 0x000A;
#[allow(dead_code)]
const GROUP_FLOW_KP: u16 = 0x000B;
#[allow(dead_code)]
const GROUP_FLOW_KI: u16 = 0x000C;
#[allow(dead_code)]
const GROUP_FLOW_KD: u16 = 0x000D;
#[allow(dead_code)]
const GROUP_OUTPUT_FLOW_KP: u16 = 0x000E;
#[allow(dead_code)]
const GROUP_OUTPUT_FLOW_KI: u16 = 0x000F;
#[allow(dead_code)]
const GROUP_OUTPUT_FLOW_KD: u16 = 0x0010;
#[allow(dead_code)]
const GROUP_PRESSURE_KP: u16 = 0x0011;
#[allow(dead_code)]
const GROUP_PRESSURE_KI: u16 = 0x0012;
#[allow(dead_code)]
const GROUP_PRESSURE_KD: u16 = 0x0013;
const GROUP_MAX_BREW_TIME: u16 = 0x0014;
const GROUP_AUTO_TARE_ENABLED: u16 = 0x0015;
#[allow(dead_code)]
const GROUP_PUMP_MAX_DUTY_CYCLE: u16 = 0x0016;
#[allow(dead_code)]
const GROUP_PUMP_MIN_DUTY_CYCLE: u16 = 0x0017;
#[allow(dead_code)]
const GROUP_PUMP_RAMP_UP_TIME: u16 = 0x0018;
#[allow(dead_code)]
const GROUP_PUMP_RAMP_DOWN_TIME: u16 = 0x0019;
#[allow(dead_code)]
const GROUP_PUMP_TACHO_PULSES_PER_LITER: u16 = 0x001A;
#[allow(dead_code)]
const GROUP_PRESSURE_KALMAN_PROCESS_NOISE: u16 = 0x001B;
#[allow(dead_code)]
const GROUP_PRESSURE_KALMAN_MEASUREMENT_NOISE: u16 = 0x001C;
#[allow(dead_code)]
const GROUP_PRESSURE_KALMAN_ESTIMATED_ERROR: u16 = 0x001D;
#[allow(dead_code)]
const GROUP_PRESSURE_KALMAN_POSTERIOR_ESTIMATE: u16 = 0x001E;
#[allow(dead_code)]
const GROUP_FLOW_SENSOR_PULSES_PER_LITER: u16 = 0x001F;
const GROUP_PID_P_TERM: u16 = 0x0020;
const GROUP_PID_I_TERM: u16 = 0x0021;
const GROUP_PID_D_TERM: u16 = 0x0022;

// Water tap property constants
const WATER_TAP_IS_DISPENSING: u16 = 0x0001;
#[allow(dead_code)]
const WATER_TAP_PUMP_STRATEGY: u16 = 0x0002;
const WATER_TAP_TEMPERATURE_TARGET: u16 = 0x0003;
const WATER_TAP_MAX_DISPENSE_TIME: u16 = 0x0004;
const WATER_TAP_FLOW_RATE_LIMIT: u16 = 0x0005;
#[allow(dead_code)]
const WATER_TAP_PUMP_MAX_DUTY_CYCLE: u16 = 0x0006;
#[allow(dead_code)]
const WATER_TAP_PUMP_MIN_DUTY_CYCLE: u16 = 0x0007;
#[allow(dead_code)]
const WATER_TAP_PUMP_RAMP_UP_TIME: u16 = 0x0008;
#[allow(dead_code)]
const WATER_TAP_PUMP_RAMP_DOWN_TIME: u16 = 0x0009;
#[allow(dead_code)]
const WATER_TAP_PUMP_TACHO_PULSES_PER_LITER: u16 = 0x000A;

// Steam wand property constants
const STEAM_WAND_TEMPERATURE_TARGET: u16 = 0x0001;
const STEAM_WAND_OPENNESS: u16 = 0x0002;
const STEAM_WAND_PURGE_TIME: u16 = 0x0003;
const STEAM_WAND_MAX_STEAM_TIME: u16 = 0x0004;
const STEAM_WAND_AUTO_PURGE_ENABLED: u16 = 0x0005;
const STEAM_WAND_IS_STEAMING: u16 = 0x0006;
const STEAM_WAND_VALVE_OPENNESS: u16 = 0x0007;

// Machine property constants
const MACHINE_HEATING_ELEMENT_INTERLOCK: u16 = 0x0001;
const MACHINE_MODE: u16 = 0x0002;

// Tank property constants
const TANK_WATER_LEVEL: u16 = 0x0001;
const TANK_LOW_LEVEL_WARNING_THRESHOLD: u16 = 0x0002;

// Key generation helper
fn generate_key(entity_type: u32, device_index: u8, property: u16) -> u32 {
    (entity_type << 24) | ((device_index as u32) << 16) | (property as u32)
}

// Key parsing helpers
pub fn parse_key(key: u32) -> (u8, u8, u16) {
    let entity_type = ((key >> 24) & 0xFF) as u8;
    let device_index = ((key >> 16) & 0xFF) as u8;
    let property = (key & 0xFFFF) as u16;
    (entity_type, device_index, property)
}

#[allow(dead_code)]
pub fn is_boiler_key(key: u32) -> bool {
    ((key >> 24) & 0xFF) == ENTITY_TYPE_BOILER
}

#[allow(dead_code)]
pub fn is_group_key(key: u32) -> bool {
    ((key >> 24) & 0xFF) == ENTITY_TYPE_GROUP
}

#[allow(dead_code)]
pub fn is_water_tap_key(key: u32) -> bool {
    ((key >> 24) & 0xFF) == ENTITY_TYPE_WATER_TAP
}

#[allow(dead_code)]
pub fn is_steam_wand_key(key: u32) -> bool {
    ((key >> 24) & 0xFF) == ENTITY_TYPE_STEAM_WAND
}

#[allow(dead_code)]
pub fn is_machine_key(key: u32) -> bool {
    ((key >> 24) & 0xFF) == ENTITY_TYPE_MACHINE
}

#[allow(dead_code)]
pub fn is_tank_key(key: u32) -> bool {
    ((key >> 24) & 0xFF) == ENTITY_TYPE_TANK
}

// Helper functions for boiler control mode conversion
fn get_available_control_modes(boiler_def: &variegated_controller_types::BoilerDefinition) -> Vec<&'static str> {
    let mut modes = vec!["Off"];

    if boiler_def.control_modes.contains(&ControlModeCapability::TemperaturePid) {
        modes.push("Temperature");
    }

    if boiler_def.control_modes.contains(&ControlModeCapability::PressurePid) {
        modes.push("Pressure");
    }

    modes
}

pub fn boiler_control_mode_to_string(mode: BoilerControlMode) -> &'static str {
    match mode {
        BoilerControlMode::Temperature => "Temperature",
        BoilerControlMode::Pressure => "Pressure",
        BoilerControlMode::Off => "Off",
    }
}

pub fn string_to_boiler_control_mode(s: &str) -> Option<BoilerControlMode> {
    match s {
        "Temperature" => Some(BoilerControlMode::Temperature),
        "Pressure" => Some(BoilerControlMode::Pressure),
        "Off" => Some(BoilerControlMode::Off),
        _ => None,
    }
}

// Helper functions for machine mode conversion
pub fn machine_mode_to_string(mode: variegated_controller_types::MachineMode) -> &'static str {
    match mode {
        variegated_controller_types::MachineMode::On => "On",
        variegated_controller_types::MachineMode::Off => "Off",
        variegated_controller_types::MachineMode::PowerSaveStandby => "PowerSaveStandby",
    }
}

pub fn string_to_machine_mode(s: &str) -> Option<variegated_controller_types::MachineMode> {
    match s {
        "On" => Some(variegated_controller_types::MachineMode::On),
        "Off" => Some(variegated_controller_types::MachineMode::Off),
        "PowerSaveStandby" => Some(variegated_controller_types::MachineMode::PowerSaveStandby),
        _ => None,
    }
}

/// Ceiling on the entity table.
///
/// A dual-boiler machine builds about 100. 128 leaves headroom for another component
/// without being so generous that the reservation hurts: at
/// `size_of::<EntityConfig>() == 120` this is 15360 bytes of `.bss`, and `.bss` comes out
/// of the stack.
///
/// Overflow truncates and logs rather than panicking. A machine missing three sensors
/// from Home Assistant is a much better outcome than one that will not boot.
pub const MAX_ENTITIES: usize = 128;

/// The entity table, sized once and held for the life of the process.
pub type EntityList = heapless::Vec<EntityConfig<'static>, MAX_ENTITIES>;

/// Build the ESPHome entity table into caller-provided storage.
///
/// # Why this does not return a `Vec`
///
/// It used to, and the caller `Box::leak`ed it -- which meant asking the allocator for
/// one contiguous 12000-byte block. `esp_alloc` gives each heap region its own
/// `linked_list_allocator::Heap` and an allocation must fit contiguously inside a single
/// region, so that request was the largest and most fragile on the machine: it competed
/// with every other long-lived allocation for an unbroken run, and when it lost,
/// `handle_alloc_error` took the whole processor down. That happened.
///
/// Writing into storage the caller owns -- in practice a `StaticCell`, i.e. `.bss` --
/// removes the failure entirely rather than making it less likely. The table is leaked
/// either way, so a static has exactly the lifetime the heap version had.
///
/// The per-component helpers below still return `Vec`, and deliberately: each is a
/// couple of kilobytes, allocated and dropped immediately, which is the shape a
/// first-fit allocator handles without complaint.
pub fn build_entities(
    config: &Configuration,
    machine_def: &MachineDefinition,
    status: Option<&Status>,
    entities: &mut EntityList,
) {
    let mut dropped = 0usize;

    // `heapless::Vec` implements `Extend` by silently discarding what does not fit, which
    // would make a truncated table indistinguishable from a complete one. Counting is the
    // whole reason this is not `entities.extend(..)`.
    let mut append = |entities: &mut EntityList, built: Vec<EntityConfig<'static>>| {
        for entity in built {
            if entities.push(entity).is_err() {
                dropped += 1;
            }
        }
    };

    // Build machine-level configuration entities
    append(entities, build_machine_entities(&config.machine_config, machine_def));

    // Build boiler entities
    for (&boiler_index, boiler_config) in config.iter_boilers() {
        append(entities, build_boiler_entities(boiler_index, boiler_config, machine_def, status));
    }

    // Build group entities
    for (&group_index, group_config) in config.iter_groups() {
        append(entities, build_group_entities(group_index, group_config, machine_def, status));
    }

    // Build water tap entities
    for (&water_tap_index, water_tap_config) in config.iter_water_taps() {
        append(entities, build_water_tap_entities(water_tap_index, water_tap_config, machine_def, status));
    }

    // Build steam wand entities
    for (&steam_wand_index, steam_wand_config) in config.iter_steam_wands() {
        append(entities, build_steam_wand_entities(steam_wand_index, steam_wand_config, machine_def, status));
    }

    // Build tank entities
    for (&tank_index, tank_config) in config.iter_tanks() {
        append(entities, build_tank_entities(tank_index, tank_config, machine_def, status));
    }

    if dropped > 0 {
        log_warn!(
            "Entity table full at {}: {} entities dropped and will not appear in Home Assistant",
            MAX_ENTITIES,
            dropped
        );
    }
}

fn build_machine_entities(
    _machine_config: &variegated_controller_types::MachineConfiguration,
    _machine_def: &MachineDefinition
) -> Vec<EntityConfig<'static>> {
    let mut entities = Vec::new();

    // Heating element interlock switch
    entities.push(EntityConfig::Switch(SwitchConfig {
        object_id: Box::leak("machine_heating_element_interlock".to_string().into_boxed_str()),
        key: generate_key(ENTITY_TYPE_MACHINE, 0, MACHINE_HEATING_ELEMENT_INTERLOCK),
        name: Box::leak("Heating Element Interlock".to_string().into_boxed_str()),
        unique_id: Box::leak("machine_heating_element_interlock".to_string().into_boxed_str()),
        icon: "mdi:power-socket",
        disabled_by_default: false,
        entity_category: EntityCategory::Config,
        device_class: "",
        assumed_state: false,
    }));

    // Machine mode select
    entities.push(EntityConfig::Select(SelectConfig {
        object_id: Box::leak("machine_mode".to_string().into_boxed_str()),
        key: generate_key(ENTITY_TYPE_MACHINE, 0, MACHINE_MODE),
        name: Box::leak("Machine Mode".to_string().into_boxed_str()),
        unique_id: Box::leak("machine_mode".to_string().into_boxed_str()),
        icon: "mdi:power",
        options: Box::leak(vec!["On", "Off", "PowerSaveStandby"].into_boxed_slice()),
        disabled_by_default: false,
        entity_category: EntityCategory::Config,
    }));

    entities
}

fn build_boiler_entities(
    boiler_index: BoilerIndex,
    config: &BoilerConfiguration,
    machine_def: &MachineDefinition,
    _status: Option<&Status>
) -> Vec<EntityConfig<'static>> {
    let mut entities = Vec::new();

    // Get boiler definition to determine capabilities
    let boiler_def = match machine_def.boilers.get(&boiler_index) {
        Some(def) => def,
        None => return entities, // No definition, no entities
    };

    let boiler_name = boiler_def.name.as_str().to_string();

    // Check capabilities
    let has_temperature_sensor = boiler_def.sensors.contains(&SensorCapability::Temperature);
    let has_pressure_sensor = boiler_def.sensors.contains(&SensorCapability::Pressure);
    let has_water_level_sensor = boiler_def.sensors.contains(&SensorCapability::WaterLevel);
    let has_heating_element = boiler_def.actuators.contains(&ActuatorCapability::HeatingElement);
    let has_temperature_control = boiler_def.control_modes.contains(&ControlModeCapability::TemperaturePid);
    let has_pressure_control = boiler_def.control_modes.contains(&ControlModeCapability::PressurePid);

    // Temperature sensor (only if boiler has temperature sensor)
    if has_temperature_sensor {
        entities.push(EntityConfig::Sensor(SensorConfig {
            object_id: Box::leak(format!("boiler_{}_temp", boiler_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_BOILER, boiler_index, BOILER_TEMPERATURE),
            name: Box::leak(format!("{} Temperature", boiler_name).into_boxed_str()),
            unique_id: Box::leak(format!("boiler_{}_temp", boiler_index).into_boxed_str()),
            icon: "",
            unit_of_measurement: "°C",
            accuracy_decimals: 1,
            force_update: false,
            device_class: "temperature",
            state_class: SensorStateClass::StateClassMeasurement,
            disabled_by_default: false,
            entity_category: EntityCategory::None,
        }));
    }

    // Pressure sensor (only if boiler has pressure sensor)
    if has_pressure_sensor {
        entities.push(EntityConfig::Sensor(SensorConfig {
            object_id: Box::leak(format!("boiler_{}_pressure", boiler_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_BOILER, boiler_index, BOILER_PRESSURE),
            name: Box::leak(format!("{} Pressure", boiler_name).into_boxed_str()),
            unique_id: Box::leak(format!("boiler_{}_pressure", boiler_index).into_boxed_str()),
            icon: "",
            unit_of_measurement: "bar",
            accuracy_decimals: 2,
            force_update: false,
            device_class: "pressure",
            state_class: SensorStateClass::StateClassMeasurement,
            disabled_by_default: false,
            entity_category: EntityCategory::None,
        }));
    }

    // Water level sensor (only if boiler has water level sensor)
    if has_water_level_sensor {
        entities.push(EntityConfig::Sensor(SensorConfig {
            object_id: Box::leak(format!("boiler_{}_water_level", boiler_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_BOILER, boiler_index, BOILER_WATER_LEVEL),
            name: Box::leak(format!("{} Water Level", boiler_name).into_boxed_str()),
            unique_id: Box::leak(format!("boiler_{}_water_level", boiler_index).into_boxed_str()),
            icon: "mdi:water-percent",
            unit_of_measurement: "%",
            accuracy_decimals: 0,
            force_update: false,
            device_class: "water",
            state_class: SensorStateClass::StateClassMeasurement,
            disabled_by_default: false,
            entity_category: EntityCategory::None,
        }));
    }

    // Temperature setpoint (only if temperature control is available)
    if has_temperature_control {
        entities.push(EntityConfig::Number(NumberConfig {
            object_id: Box::leak(format!("boiler_{}_temp_setpoint", boiler_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_BOILER, boiler_index, BOILER_TEMP_SETPOINT),
            name: Box::leak(format!("{} Temperature Setpoint", boiler_name).into_boxed_str()),
            unique_id: Box::leak(format!("boiler_{}_temp_setpoint", boiler_index).into_boxed_str()),
            icon: "mdi:thermometer",
            min_value: 10.0,
            max_value: 150.0,
            step: 0.5,
            disabled_by_default: false,
            entity_category: EntityCategory::Config,
            unit_of_measurement: "°C",
            mode: NumberMode::Box,
            device_class: "temperature",
        }));
    }

    // Pressure setpoint (only if pressure control is available)
    if has_pressure_control {
        entities.push(EntityConfig::Number(NumberConfig {
            object_id: Box::leak(format!("boiler_{}_pressure_setpoint", boiler_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_BOILER, boiler_index, BOILER_PRESSURE_SETPOINT),
            name: Box::leak(format!("{} Pressure Setpoint", boiler_name).into_boxed_str()),
            unique_id: Box::leak(format!("boiler_{}_pressure_setpoint", boiler_index).into_boxed_str()),
            icon: "mdi:gauge",
            min_value: 0.0,
            max_value: config.max_pressure.unwrap_or(15.0),
            step: 0.1,
            disabled_by_default: false,
            entity_category: EntityCategory::Config,
            unit_of_measurement: "bar",
            mode: NumberMode::Box,
            device_class: "pressure",
        }));
    }

    // Control mode select (if multiple modes available)
    let available_modes = get_available_control_modes(boiler_def);
    if available_modes.len() > 1 {
        entities.push(EntityConfig::Select(SelectConfig {
            object_id: Box::leak(format!("boiler_{}_control_mode", boiler_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_BOILER, boiler_index, BOILER_CONTROL_MODE),
            name: Box::leak(format!("{} Control Mode", boiler_name).into_boxed_str()),
            unique_id: Box::leak(format!("boiler_{}_control_mode", boiler_index).into_boxed_str()),
            icon: "mdi:tune",
            options: Box::leak(available_modes.into_boxed_slice()),
            disabled_by_default: false,
            entity_category: EntityCategory::Config,
        }));
    }

    // Temperature PID parameters (only if temperature control is available)
    if has_temperature_control {
        entities.push(EntityConfig::Number(NumberConfig {
            object_id: Box::leak(format!("boiler_{}_temp_kp", boiler_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_BOILER, boiler_index, BOILER_TEMP_KP),
            name: Box::leak(format!("{} Temperature kP", boiler_name).into_boxed_str()),
            unique_id: Box::leak(format!("boiler_{}_temp_kp", boiler_index).into_boxed_str()),
            icon: "mdi:tune",
            min_value: 0.0,
            max_value: 100.0,
            step: 0.1,
            disabled_by_default: false,
            entity_category: EntityCategory::Config,
            unit_of_measurement: "",
            mode: NumberMode::Box,
            device_class: "",
        }));

        entities.push(EntityConfig::Number(NumberConfig {
            object_id: Box::leak(format!("boiler_{}_temp_ki", boiler_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_BOILER, boiler_index, BOILER_TEMP_KI),
            name: Box::leak(format!("{} Temperature kI", boiler_name).into_boxed_str()),
            unique_id: Box::leak(format!("boiler_{}_temp_ki", boiler_index).into_boxed_str()),
            icon: "mdi:tune",
            min_value: 0.0,
            max_value: 10.0,
            step: 0.0001,
            disabled_by_default: false,
            entity_category: EntityCategory::Config,
            unit_of_measurement: "",
            mode: NumberMode::Box,
            device_class: "",
        }));

        entities.push(EntityConfig::Number(NumberConfig {
            object_id: Box::leak(format!("boiler_{}_temp_kd", boiler_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_BOILER, boiler_index, BOILER_TEMP_KD),
            name: Box::leak(format!("{} Temperature kD", boiler_name).into_boxed_str()),
            unique_id: Box::leak(format!("boiler_{}_temp_kd", boiler_index).into_boxed_str()),
            icon: "mdi:tune",
            min_value: 0.0,
            max_value: 10.0,
            step: 0.01,
            disabled_by_default: false,
            entity_category: EntityCategory::Config,
            unit_of_measurement: "",
            mode: NumberMode::Box,
            device_class: "",
        }));

        // Temperature PID Limits
        entities.push(EntityConfig::Number(NumberConfig {
            object_id: Box::leak(format!("boiler_{}_temp_kp_upper_limit", boiler_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_BOILER, boiler_index, BOILER_TEMP_KP_UPPER_LIMIT),
            name: Box::leak(format!("{} Temperature kP Upper Limit", boiler_name).into_boxed_str()),
            unique_id: Box::leak(format!("boiler_{}_temp_kp_upper_limit", boiler_index).into_boxed_str()),
            icon: "mdi:tune-vertical",
            min_value: 0.0,
            max_value: 200.0,
            step: 1.0,
            disabled_by_default: false,
            entity_category: EntityCategory::Diagnostic,
            unit_of_measurement: "",
            mode: NumberMode::Box,
            device_class: "",
        }));

        entities.push(EntityConfig::Number(NumberConfig {
            object_id: Box::leak(format!("boiler_{}_temp_kp_lower_limit", boiler_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_BOILER, boiler_index, BOILER_TEMP_KP_LOWER_LIMIT),
            name: Box::leak(format!("{} Temperature kP Lower Limit", boiler_name).into_boxed_str()),
            unique_id: Box::leak(format!("boiler_{}_temp_kp_lower_limit", boiler_index).into_boxed_str()),
            icon: "mdi:tune-vertical",
            min_value: -200.0,
            max_value: 0.0,
            step: 1.0,
            disabled_by_default: false,
            entity_category: EntityCategory::Diagnostic,
            unit_of_measurement: "",
            mode: NumberMode::Box,
            device_class: "",
        }));

        entities.push(EntityConfig::Number(NumberConfig {
            object_id: Box::leak(format!("boiler_{}_temp_ki_upper_limit", boiler_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_BOILER, boiler_index, BOILER_TEMP_KI_UPPER_LIMIT),
            name: Box::leak(format!("{} Temperature kI Upper Limit", boiler_name).into_boxed_str()),
            unique_id: Box::leak(format!("boiler_{}_temp_ki_upper_limit", boiler_index).into_boxed_str()),
            icon: "mdi:tune-vertical",
            min_value: 0.0,
            max_value: 200.0,
            step: 1.0,
            disabled_by_default: false,
            entity_category: EntityCategory::Diagnostic,
            unit_of_measurement: "",
            mode: NumberMode::Box,
            device_class: "",
        }));

        entities.push(EntityConfig::Number(NumberConfig {
            object_id: Box::leak(format!("boiler_{}_temp_ki_lower_limit", boiler_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_BOILER, boiler_index, BOILER_TEMP_KI_LOWER_LIMIT),
            name: Box::leak(format!("{} Temperature kI Lower Limit", boiler_name).into_boxed_str()),
            unique_id: Box::leak(format!("boiler_{}_temp_ki_lower_limit", boiler_index).into_boxed_str()),
            icon: "mdi:tune-vertical",
            min_value: -200.0,
            max_value: 0.0,
            step: 1.0,
            disabled_by_default: false,
            entity_category: EntityCategory::Diagnostic,
            unit_of_measurement: "",
            mode: NumberMode::Box,
            device_class: "",
        }));

        entities.push(EntityConfig::Number(NumberConfig {
            object_id: Box::leak(format!("boiler_{}_temp_kd_upper_limit", boiler_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_BOILER, boiler_index, BOILER_TEMP_KD_UPPER_LIMIT),
            name: Box::leak(format!("{} Temperature kD Upper Limit", boiler_name).into_boxed_str()),
            unique_id: Box::leak(format!("boiler_{}_temp_kd_upper_limit", boiler_index).into_boxed_str()),
            icon: "mdi:tune-vertical",
            min_value: 0.0,
            max_value: 20.0,
            step: 1.0,
            disabled_by_default: false,
            entity_category: EntityCategory::Diagnostic,
            unit_of_measurement: "",
            mode: NumberMode::Box,
            device_class: "",
        }));

        entities.push(EntityConfig::Number(NumberConfig {
            object_id: Box::leak(format!("boiler_{}_temp_kd_lower_limit", boiler_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_BOILER, boiler_index, BOILER_TEMP_KD_LOWER_LIMIT),
            name: Box::leak(format!("{} Temperature kD Lower Limit", boiler_name).into_boxed_str()),
            unique_id: Box::leak(format!("boiler_{}_temp_kd_lower_limit", boiler_index).into_boxed_str()),
            icon: "mdi:tune-vertical",
            min_value: -200.0,
            max_value: 0.0,
            step: 1.0,
            disabled_by_default: false,
            entity_category: EntityCategory::Diagnostic,
            unit_of_measurement: "",
            mode: NumberMode::Box,
            device_class: "",
        }));
    }

    // Pressure PID parameters (only if pressure control is available)
    if has_pressure_control {
        entities.push(EntityConfig::Number(NumberConfig {
            object_id: Box::leak(format!("boiler_{}_pressure_kp", boiler_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_BOILER, boiler_index, BOILER_PRESSURE_KP),
            name: Box::leak(format!("{} Pressure kP", boiler_name).into_boxed_str()),
            unique_id: Box::leak(format!("boiler_{}_pressure_kp", boiler_index).into_boxed_str()),
            icon: "mdi:tune",
            min_value: 0.0,
            max_value: 100.0,
            step: 0.1,
            disabled_by_default: false,
            entity_category: EntityCategory::Config,
            unit_of_measurement: "",
            mode: NumberMode::Box,
            device_class: "",
        }));

        entities.push(EntityConfig::Number(NumberConfig {
            object_id: Box::leak(format!("boiler_{}_pressure_ki", boiler_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_BOILER, boiler_index, BOILER_PRESSURE_KI),
            name: Box::leak(format!("{} Pressure kI", boiler_name).into_boxed_str()),
            unique_id: Box::leak(format!("boiler_{}_pressure_ki", boiler_index).into_boxed_str()),
            icon: "mdi:tune",
            min_value: 0.0,
            max_value: 10.0,
            step: 0.01,
            disabled_by_default: false,
            entity_category: EntityCategory::Config,
            unit_of_measurement: "",
            mode: NumberMode::Box,
            device_class: "",
        }));

        entities.push(EntityConfig::Number(NumberConfig {
            object_id: Box::leak(format!("boiler_{}_pressure_kd", boiler_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_BOILER, boiler_index, BOILER_PRESSURE_KD),
            name: Box::leak(format!("{} Pressure kD", boiler_name).into_boxed_str()),
            unique_id: Box::leak(format!("boiler_{}_pressure_kd", boiler_index).into_boxed_str()),
            icon: "mdi:tune",
            min_value: 0.0,
            max_value: 10.0,
            step: 0.01,
            disabled_by_default: false,
            entity_category: EntityCategory::Config,
            unit_of_measurement: "",
            mode: NumberMode::Box,
            device_class: "",
        }));
    }

    // Duty cycle sensor (only if heating element is available)
    if has_heating_element {
        entities.push(EntityConfig::Sensor(SensorConfig {
            object_id: Box::leak(format!("boiler_{}_duty_cycle", boiler_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_BOILER, boiler_index, BOILER_DUTY_CYCLE),
            name: Box::leak(format!("{} Duty Cycle", boiler_name).into_boxed_str()),
            unique_id: Box::leak(format!("boiler_{}_duty_cycle", boiler_index).into_boxed_str()),
            icon: "",
            unit_of_measurement: "%",
            accuracy_decimals: 0,
            force_update: false,
            device_class: "",
            state_class: SensorStateClass::StateClassMeasurement,
            disabled_by_default: false,
            entity_category: EntityCategory::Diagnostic,
        }));

        // PID term sensors (only if heating element is available)
        entities.push(EntityConfig::Sensor(SensorConfig {
            object_id: Box::leak(format!("boiler_{}_pid_p_term", boiler_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_BOILER, boiler_index, BOILER_PID_P_TERM),
            name: Box::leak(format!("{} PID P Term", boiler_name).into_boxed_str()),
            unique_id: Box::leak(format!("boiler_{}_pid_p_term", boiler_index).into_boxed_str()),
            icon: "mdi:chart-line-variant",
            unit_of_measurement: "",
            accuracy_decimals: 3,
            force_update: false,
            device_class: "",
            state_class: SensorStateClass::StateClassMeasurement,
            disabled_by_default: false,
            entity_category: EntityCategory::Diagnostic,
        }));

        entities.push(EntityConfig::Sensor(SensorConfig {
            object_id: Box::leak(format!("boiler_{}_pid_i_term", boiler_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_BOILER, boiler_index, BOILER_PID_I_TERM),
            name: Box::leak(format!("{} PID I Term", boiler_name).into_boxed_str()),
            unique_id: Box::leak(format!("boiler_{}_pid_i_term", boiler_index).into_boxed_str()),
            icon: "mdi:chart-line-variant",
            unit_of_measurement: "",
            accuracy_decimals: 3,
            force_update: false,
            device_class: "",
            state_class: SensorStateClass::StateClassMeasurement,
            disabled_by_default: false,
            entity_category: EntityCategory::Diagnostic,
        }));

        entities.push(EntityConfig::Sensor(SensorConfig {
            object_id: Box::leak(format!("boiler_{}_pid_d_term", boiler_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_BOILER, boiler_index, BOILER_PID_D_TERM),
            name: Box::leak(format!("{} PID D Term", boiler_name).into_boxed_str()),
            unique_id: Box::leak(format!("boiler_{}_pid_d_term", boiler_index).into_boxed_str()),
            icon: "mdi:chart-line-variant",
            unit_of_measurement: "",
            accuracy_decimals: 3,
            force_update: false,
            device_class: "",
            state_class: SensorStateClass::StateClassMeasurement,
            disabled_by_default: false,
            entity_category: EntityCategory::Diagnostic,
        }));
    }

    // Max temperature configuration (only if temperature sensor is available)
    if has_temperature_sensor {
        if let Some(_max_temp) = config.max_temperature {
            entities.push(EntityConfig::Number(NumberConfig {
                object_id: Box::leak(format!("boiler_{}_max_temp", boiler_index).into_boxed_str()),
                key: generate_key(ENTITY_TYPE_BOILER, boiler_index, BOILER_MAX_TEMPERATURE),
                name: Box::leak(format!("{} Max Temperature", boiler_name).into_boxed_str()),
                unique_id: Box::leak(format!("boiler_{}_max_temp", boiler_index).into_boxed_str()),
                icon: "mdi:thermometer-high",
                min_value: 10.0,
                max_value: 150.0,
                step: 1.0,
                disabled_by_default: false,
                entity_category: EntityCategory::Config,
                unit_of_measurement: "°C",
                mode: NumberMode::Box,
                device_class: "temperature",
            }));
        }
    }

    // Max pressure configuration (only if pressure sensor is available)
    if has_pressure_sensor {
        if let Some(_max_pressure) = config.max_pressure {
            entities.push(EntityConfig::Number(NumberConfig {
                object_id: Box::leak(format!("boiler_{}_max_pressure", boiler_index).into_boxed_str()),
                key: generate_key(ENTITY_TYPE_BOILER, boiler_index, BOILER_MAX_PRESSURE),
                name: Box::leak(format!("{} Max Pressure", boiler_name).into_boxed_str()),
                unique_id: Box::leak(format!("boiler_{}_max_pressure", boiler_index).into_boxed_str()),
                icon: "mdi:gauge-high",
                min_value: 5.0,
                max_value: 20.0,
                step: 0.1,
                disabled_by_default: false,
                entity_category: EntityCategory::Config,
                unit_of_measurement: "bar",
                mode: NumberMode::Box,
                device_class: "pressure",
            }));
        }
    }

    // Fill configuration (only if boiler has fill mechanism)
    if boiler_def.has_fill_mechanism {
        if let Some(fill_config) = &config.fill_config {
            // Fill threshold
            if let Some(_fill_threshold) = fill_config.fill_threshold {
                entities.push(EntityConfig::Number(NumberConfig {
                    object_id: Box::leak(format!("boiler_{}_fill_threshold", boiler_index).into_boxed_str()),
                    key: generate_key(ENTITY_TYPE_BOILER, boiler_index, BOILER_FILL_THRESHOLD),
                    name: Box::leak(format!("{} Fill Threshold", boiler_name).into_boxed_str()),
                    unique_id: Box::leak(format!("boiler_{}_fill_threshold", boiler_index).into_boxed_str()),
                    icon: "mdi:water-percent",
                    min_value: 0.0,
                    max_value: 100.0,
                    step: 1.0,
                    disabled_by_default: false,
                    entity_category: EntityCategory::Config,
                    unit_of_measurement: "%",
                    mode: NumberMode::Box,
                    device_class: "",
                }));
            }
        }
    }

    entities
}

fn build_group_entities(
    group_index: GroupIndex,
    config: &GroupConfiguration,
    machine_def: &MachineDefinition,
    _status: Option<&Status>
) -> Vec<EntityConfig<'static>> {
    let mut entities = Vec::new();

    // Get group definition to determine capabilities
    let group_def = match machine_def.groups.get(&group_index) {
        Some(def) => def,
        None => return entities, // No definition, no entities
    };

    let group_name = group_def.name.as_str().to_string();

    // Check capabilities
    let has_input_flow_sensor = group_def.sensors.contains(&SensorCapability::InputFlowRate);
    let has_output_flow_sensor = group_def.sensors.contains(&SensorCapability::OutputFlowRate);
    let has_weight_sensor = group_def.sensors.contains(&SensorCapability::Weight);
    let has_pump = group_def.actuators.contains(&ActuatorCapability::Pump);
    let has_scale_tare = group_def.actuators.contains(&ActuatorCapability::ScaleTare);
    let has_flow_rate_control = group_def.control_modes.contains(&ControlModeCapability::FlowRatePid);
    let has_output_flow_rate_control = group_def.control_modes.contains(&ControlModeCapability::OutputFlowRatePid);

    // Input flow rate sensor (only if input flow sensor is available)
    if has_input_flow_sensor {
        entities.push(EntityConfig::Sensor(SensorConfig {
            object_id: Box::leak(format!("group_{}_input_flow_rate", group_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_GROUP, group_index, GROUP_INPUT_FLOW_RATE),
            name: Box::leak(format!("{} Input Flow Rate", group_name).into_boxed_str()),
            unique_id: Box::leak(format!("group_{}_input_flow_rate", group_index).into_boxed_str()),
            icon: "",
            unit_of_measurement: "ml/s",
            accuracy_decimals: 1,
            force_update: false,
            device_class: "flow_rate",
            state_class: SensorStateClass::StateClassMeasurement,
            disabled_by_default: false,
            entity_category: EntityCategory::None,
        }));
    }

    // Output flow rate sensor (only if output flow sensor is available)
    if has_output_flow_sensor {
        entities.push(EntityConfig::Sensor(SensorConfig {
            object_id: Box::leak(format!("group_{}_output_flow_rate", group_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_GROUP, group_index, GROUP_OUTPUT_FLOW_RATE),
            name: Box::leak(format!("{} Output Flow Rate", group_name).into_boxed_str()),
            unique_id: Box::leak(format!("group_{}_output_flow_rate", group_index).into_boxed_str()),
            icon: "mdi:pipe",
            unit_of_measurement: "ml/s",
            accuracy_decimals: 1,
            force_update: false,
            device_class: "",
            state_class: SensorStateClass::StateClassMeasurement,
            disabled_by_default: false,
            entity_category: EntityCategory::None,
        }));
    }

    // Weight sensor (only if weight sensor is available)
    if has_weight_sensor {
        entities.push(EntityConfig::Sensor(SensorConfig {
            object_id: Box::leak(format!("group_{}_weight", group_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_GROUP, group_index, GROUP_WEIGHT),
            name: Box::leak(format!("{} Weight", group_name).into_boxed_str()),
            unique_id: Box::leak(format!("group_{}_weight", group_index).into_boxed_str()),
            icon: "mdi:scale",
            unit_of_measurement: "g",
            accuracy_decimals: 1,
            force_update: false,
            device_class: "weight",
            state_class: SensorStateClass::StateClassMeasurement,
            disabled_by_default: false,
            entity_category: EntityCategory::None,
        }));
    }

    // Is brewing binary sensor (always present for groups)
    entities.push(EntityConfig::BinarySensor(BinarySensorConfig {
        object_id: Box::leak(format!("group_{}_is_brewing", group_index).into_boxed_str()),
        key: generate_key(ENTITY_TYPE_GROUP, group_index, GROUP_IS_BREWING),
        name: Box::leak(format!("{} Is Brewing", group_name).into_boxed_str()),
        unique_id: Box::leak(format!("group_{}_is_brewing", group_index).into_boxed_str()),
        icon: "mdi:coffee",
        device_class: "",
        disabled_by_default: false,
        entity_category: EntityCategory::None,
        is_status_binary_sensor: false,
    }));

    // Brew time sensor (always present for groups)
    entities.push(EntityConfig::Sensor(SensorConfig {
        object_id: Box::leak(format!("group_{}_brew_time", group_index).into_boxed_str()),
        key: generate_key(ENTITY_TYPE_GROUP, group_index, GROUP_BREW_TIME),
        name: Box::leak(format!("{} Brew Time", group_name).into_boxed_str()),
        unique_id: Box::leak(format!("group_{}_brew_time", group_index).into_boxed_str()),
        icon: "mdi:timer",
        unit_of_measurement: "s",
        accuracy_decimals: 1,
        force_update: false,
        device_class: "duration",
        state_class: SensorStateClass::StateClassMeasurement,
        disabled_by_default: false,
        entity_category: EntityCategory::None,
    }));

    // Pump duty cycle sensor (only if pump is available)
    if has_pump {
        entities.push(EntityConfig::Sensor(SensorConfig {
            object_id: Box::leak(format!("group_{}_pump_duty_cycle", group_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_GROUP, group_index, GROUP_PUMP_DUTY_CYCLE),
            name: Box::leak(format!("{} Pump Duty Cycle", group_name).into_boxed_str()),
            unique_id: Box::leak(format!("group_{}_pump_duty_cycle", group_index).into_boxed_str()),
            icon: "",
            unit_of_measurement: "%",
            accuracy_decimals: 0,
            force_update: false,
            device_class: "",
            state_class: SensorStateClass::StateClassMeasurement,
            disabled_by_default: false,
            entity_category: EntityCategory::Diagnostic,
        }));

        // PID term sensors (only if pump is available)
        entities.push(EntityConfig::Sensor(SensorConfig {
            object_id: Box::leak(format!("group_{}_pid_p_term", group_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_GROUP, group_index, GROUP_PID_P_TERM),
            name: Box::leak(format!("{} PID P Term", group_name).into_boxed_str()),
            unique_id: Box::leak(format!("group_{}_pid_p_term", group_index).into_boxed_str()),
            icon: "mdi:chart-line-variant",
            unit_of_measurement: "",
            accuracy_decimals: 3,
            force_update: false,
            device_class: "",
            state_class: SensorStateClass::StateClassMeasurement,
            disabled_by_default: false,
            entity_category: EntityCategory::Diagnostic,
        }));

        entities.push(EntityConfig::Sensor(SensorConfig {
            object_id: Box::leak(format!("group_{}_pid_i_term", group_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_GROUP, group_index, GROUP_PID_I_TERM),
            name: Box::leak(format!("{} PID I Term", group_name).into_boxed_str()),
            unique_id: Box::leak(format!("group_{}_pid_i_term", group_index).into_boxed_str()),
            icon: "mdi:chart-line-variant",
            unit_of_measurement: "",
            accuracy_decimals: 3,
            force_update: false,
            device_class: "",
            state_class: SensorStateClass::StateClassMeasurement,
            disabled_by_default: false,
            entity_category: EntityCategory::Diagnostic,
        }));

        entities.push(EntityConfig::Sensor(SensorConfig {
            object_id: Box::leak(format!("group_{}_pid_d_term", group_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_GROUP, group_index, GROUP_PID_D_TERM),
            name: Box::leak(format!("{} PID D Term", group_name).into_boxed_str()),
            unique_id: Box::leak(format!("group_{}_pid_d_term", group_index).into_boxed_str()),
            icon: "mdi:chart-line-variant",
            unit_of_measurement: "",
            accuracy_decimals: 3,
            force_update: false,
            device_class: "",
            state_class: SensorStateClass::StateClassMeasurement,
            disabled_by_default: false,
            entity_category: EntityCategory::Diagnostic,
        }));
    }

    // Flow rate setpoint (only if flow rate control is available)
    if has_flow_rate_control {
        entities.push(EntityConfig::Number(NumberConfig {
            object_id: Box::leak(format!("group_{}_flow_rate_setpoint", group_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_GROUP, group_index, GROUP_FLOW_RATE_SETPOINT),
            name: Box::leak(format!("{} Flow Rate Setpoint", group_name).into_boxed_str()),
            unique_id: Box::leak(format!("group_{}_flow_rate_setpoint", group_index).into_boxed_str()),
            icon: "mdi:speedometer",
            min_value: 0.0,
            max_value: 50.0,
            step: 0.1,
            disabled_by_default: false,
            entity_category: EntityCategory::Config,
            unit_of_measurement: "ml/s",
            mode: NumberMode::Box,
            device_class: "",
        }));
    }

    // Output flow rate setpoint (only if output flow rate control is available)
    if has_output_flow_rate_control {
        entities.push(EntityConfig::Number(NumberConfig {
            object_id: Box::leak(format!("group_{}_output_flow_rate_setpoint", group_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_GROUP, group_index, GROUP_OUTPUT_FLOW_RATE_SETPOINT),
            name: Box::leak(format!("{} Output Flow Rate Setpoint", group_name).into_boxed_str()),
            unique_id: Box::leak(format!("group_{}_output_flow_rate_setpoint", group_index).into_boxed_str()),
            icon: "mdi:speedometer",
            min_value: 0.0,
            max_value: 50.0,
            step: 0.1,
            disabled_by_default: false,
            entity_category: EntityCategory::Config,
            unit_of_measurement: "ml/s",
            mode: NumberMode::Box,
            device_class: "",
        }));
    }

    entities.push(EntityConfig::Number(NumberConfig {
        object_id: Box::leak(format!("group_{}_max_brew_time", group_index).into_boxed_str()),
        key: generate_key(ENTITY_TYPE_GROUP, group_index, GROUP_MAX_BREW_TIME),
        name: Box::leak(format!("{} Max Brew Time", group_name).into_boxed_str()),
        unique_id: Box::leak(format!("group_{}_max_brew_time", group_index).into_boxed_str()),
        icon: "mdi:timer-stop",
        min_value: 0.0,
        max_value: 600.0,
        step: 1.0,
        disabled_by_default: false,
        entity_category: EntityCategory::Config,
        unit_of_measurement: "s",
        mode: NumberMode::Box,
        device_class: "duration",
    }));

    // Auto tare enabled (only if scale tare capability is available)
    if has_scale_tare {
        entities.push(EntityConfig::Switch(SwitchConfig {
            object_id: Box::leak(format!("group_{}_auto_tare_enabled", group_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_GROUP, group_index, GROUP_AUTO_TARE_ENABLED),
            name: Box::leak(format!("{} Auto Tare Enabled", group_name).into_boxed_str()),
            unique_id: Box::leak(format!("group_{}_auto_tare_enabled", group_index).into_boxed_str()),
            icon: "mdi:scale-balance",
            disabled_by_default: false,
            entity_category: EntityCategory::Config,
            device_class: "",
            assumed_state: false,
        }));
    }

    entities
}

fn build_water_tap_entities(
    water_tap_index: WaterTapIndex,
    config: &WaterTapConfiguration,
    machine_def: &MachineDefinition,
    _status: Option<&Status>
) -> Vec<EntityConfig<'static>> {
    let mut entities = Vec::new();

    // Get water tap definition to determine capabilities
    let water_tap_def = match machine_def.water_taps.get(&water_tap_index) {
        Some(def) => def,
        None => return entities, // No definition, no entities
    };

    let water_tap_name = water_tap_def.name.as_str().to_string();

    // Is dispensing binary sensor (always present for water taps)
    entities.push(EntityConfig::BinarySensor(BinarySensorConfig {
        object_id: Box::leak(format!("water_tap_{}_is_dispensing", water_tap_index).into_boxed_str()),
        key: generate_key(ENTITY_TYPE_WATER_TAP, water_tap_index, WATER_TAP_IS_DISPENSING),
        name: Box::leak(format!("{} Is Dispensing", water_tap_name).into_boxed_str()),
        unique_id: Box::leak(format!("water_tap_{}_is_dispensing", water_tap_index).into_boxed_str()),
        icon: "mdi:water",
        device_class: "",
        disabled_by_default: false,
        entity_category: EntityCategory::None,
        is_status_binary_sensor: false,
    }));

    // Temperature target (if configured)
    if let Some(_temp_target) = config.temperature_target {
        entities.push(EntityConfig::Number(NumberConfig {
            object_id: Box::leak(format!("water_tap_{}_temp_target", water_tap_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_WATER_TAP, water_tap_index, WATER_TAP_TEMPERATURE_TARGET),
            name: Box::leak(format!("{} Temperature Target", water_tap_name).into_boxed_str()),
            unique_id: Box::leak(format!("water_tap_{}_temp_target", water_tap_index).into_boxed_str()),
            icon: "mdi:thermometer",
            min_value: 20.0,
            max_value: 100.0,
            step: 1.0,
            disabled_by_default: false,
            entity_category: EntityCategory::Config,
            unit_of_measurement: "°C",
            mode: NumberMode::Box,
            device_class: "temperature",
        }));
    }

    entities.push(EntityConfig::Number(NumberConfig {
        object_id: Box::leak(format!("water_tap_{}_max_dispense_time", water_tap_index).into_boxed_str()),
        key: generate_key(ENTITY_TYPE_WATER_TAP, water_tap_index, WATER_TAP_MAX_DISPENSE_TIME),
        name: Box::leak(format!("{} Max Dispense Time", water_tap_name).into_boxed_str()),
        unique_id: Box::leak(format!("water_tap_{}_max_dispense_time", water_tap_index).into_boxed_str()),
        icon: "mdi:timer-stop",
        min_value: 5.0,
        max_value: 300.0,
        step: 1.0,
        disabled_by_default: false,
        entity_category: EntityCategory::Config,
        unit_of_measurement: "s",
        mode: NumberMode::Box,
        device_class: "duration",
    }));

    // Flow rate limit (if configured)
    if let Some(_flow_rate_limit) = config.flow_rate_limit {
        entities.push(EntityConfig::Number(NumberConfig {
            object_id: Box::leak(format!("water_tap_{}_flow_rate_limit", water_tap_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_WATER_TAP, water_tap_index, WATER_TAP_FLOW_RATE_LIMIT),
            name: Box::leak(format!("{} Flow Rate Limit", water_tap_name).into_boxed_str()),
            unique_id: Box::leak(format!("water_tap_{}_flow_rate_limit", water_tap_index).into_boxed_str()),
            icon: "mdi:speedometer",
            min_value: 0.5,
            max_value: 50.0,
            step: 0.1,
            disabled_by_default: false,
            entity_category: EntityCategory::Config,
            unit_of_measurement: "ml/s",
            mode: NumberMode::Box,
            device_class: "",
        }));
    }

    entities
}

fn build_steam_wand_entities(
    steam_wand_index: SteamWandIndex,
    config: &SteamWandConfiguration,
    machine_def: &MachineDefinition,
    _status: Option<&Status>
) -> Vec<EntityConfig<'static>> {
    let mut entities = Vec::new();

    // Get steam wand definition to determine capabilities
    let steam_wand_def = match machine_def.steam_wands.get(&steam_wand_index) {
        Some(def) => def,
        None => return entities, // No definition, no entities
    };

    let steam_wand_name = steam_wand_def.name.as_str().to_string();

    // Is steaming binary sensor (always present for steam wands)
    entities.push(EntityConfig::BinarySensor(BinarySensorConfig {
        object_id: Box::leak(format!("steam_wand_{}_is_steaming", steam_wand_index).into_boxed_str()),
        key: generate_key(ENTITY_TYPE_STEAM_WAND, steam_wand_index, STEAM_WAND_IS_STEAMING),
        name: Box::leak(format!("{} Is Steaming", steam_wand_name).into_boxed_str()),
        unique_id: Box::leak(format!("steam_wand_{}_is_steaming", steam_wand_index).into_boxed_str()),
        icon: "mdi:steam",
        device_class: "",
        disabled_by_default: false,
        entity_category: EntityCategory::None,
        is_status_binary_sensor: false,
    }));

    // Valve openness sensor (always present for steam wands)
    entities.push(EntityConfig::Sensor(SensorConfig {
        object_id: Box::leak(format!("steam_wand_{}_valve_openness", steam_wand_index).into_boxed_str()),
        key: generate_key(ENTITY_TYPE_STEAM_WAND, steam_wand_index, STEAM_WAND_VALVE_OPENNESS),
        name: Box::leak(format!("{} Valve Openness", steam_wand_name).into_boxed_str()),
        unique_id: Box::leak(format!("steam_wand_{}_valve_openness", steam_wand_index).into_boxed_str()),
        icon: "mdi:valve",
        unit_of_measurement: "%",
        accuracy_decimals: 0,
        force_update: false,
        device_class: "",
        state_class: SensorStateClass::StateClassMeasurement,
        disabled_by_default: false,
        entity_category: EntityCategory::None,
    }));

    // Temperature target (if configured)
    if let Some(_temp_target) = config.temperature_target {
        entities.push(EntityConfig::Number(NumberConfig {
            object_id: Box::leak(format!("steam_wand_{}_temp_target", steam_wand_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_STEAM_WAND, steam_wand_index, STEAM_WAND_TEMPERATURE_TARGET),
            name: Box::leak(format!("{} Temperature Target", steam_wand_name).into_boxed_str()),
            unique_id: Box::leak(format!("steam_wand_{}_temp_target", steam_wand_index).into_boxed_str()),
            icon: "mdi:thermometer",
            min_value: 100.0,
            max_value: 160.0,
            step: 1.0,
            disabled_by_default: false,
            entity_category: EntityCategory::Config,
            unit_of_measurement: "°C",
            mode: NumberMode::Box,
            device_class: "temperature",
        }));
    }

    // Openness (if configured)
    if let Some(_openness) = config.openness {
        entities.push(EntityConfig::Number(NumberConfig {
            object_id: Box::leak(format!("steam_wand_{}_openness", steam_wand_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_STEAM_WAND, steam_wand_index, STEAM_WAND_OPENNESS),
            name: Box::leak(format!("{} Openness", steam_wand_name).into_boxed_str()),
            unique_id: Box::leak(format!("steam_wand_{}_openness", steam_wand_index).into_boxed_str()),
            icon: "mdi:gauge",
            min_value: 0.0,
            max_value: 100.0,
            step: 1.0,
            disabled_by_default: false,
            entity_category: EntityCategory::Config,
            unit_of_measurement: "%",
            mode: NumberMode::Box,
            device_class: "pressure",
        }));
    }

    // Purge time (if configured)
    if let Some(_purge_time) = config.purge_time_seconds {
        entities.push(EntityConfig::Number(NumberConfig {
            object_id: Box::leak(format!("steam_wand_{}_purge_time", steam_wand_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_STEAM_WAND, steam_wand_index, STEAM_WAND_PURGE_TIME),
            name: Box::leak(format!("{} Purge Time", steam_wand_name).into_boxed_str()),
            unique_id: Box::leak(format!("steam_wand_{}_purge_time", steam_wand_index).into_boxed_str()),
            icon: "mdi:timer",
            min_value: 1.0,
            max_value: 10.0,
            step: 0.5,
            disabled_by_default: false,
            entity_category: EntityCategory::Config,
            unit_of_measurement: "s",
            mode: NumberMode::Box,
            device_class: "duration",
        }));
    }

    // Max steam time (if configured)
    if let Some(_max_steam_time) = config.max_steam_time_seconds {
        entities.push(EntityConfig::Number(NumberConfig {
            object_id: Box::leak(format!("steam_wand_{}_max_steam_time", steam_wand_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_STEAM_WAND, steam_wand_index, STEAM_WAND_MAX_STEAM_TIME),
            name: Box::leak(format!("{} Max Steam Time", steam_wand_name).into_boxed_str()),
            unique_id: Box::leak(format!("steam_wand_{}_max_steam_time", steam_wand_index).into_boxed_str()),
            icon: "mdi:timer-stop",
            min_value: 30.0,
            max_value: 300.0,
            step: 5.0,
            disabled_by_default: false,
            entity_category: EntityCategory::Config,
            unit_of_measurement: "s",
            mode: NumberMode::Box,
            device_class: "duration",
        }));
    }

    // Auto purge enabled switch
    entities.push(EntityConfig::Switch(SwitchConfig {
        object_id: Box::leak(format!("steam_wand_{}_auto_purge_enabled", steam_wand_index).into_boxed_str()),
        key: generate_key(ENTITY_TYPE_STEAM_WAND, steam_wand_index, STEAM_WAND_AUTO_PURGE_ENABLED),
        name: Box::leak(format!("{} Auto Purge Enabled", steam_wand_name).into_boxed_str()),
        unique_id: Box::leak(format!("steam_wand_{}_auto_purge_enabled", steam_wand_index).into_boxed_str()),
        icon: "mdi:auto-fix",
        disabled_by_default: false,
        entity_category: EntityCategory::Config,
        device_class: "",
        assumed_state: false,
    }));

    entities
}

fn build_tank_entities(
    tank_index: TankIndex,
    config: &TankConfiguration,
    machine_def: &MachineDefinition,
    _status: Option<&Status>
) -> Vec<EntityConfig<'static>> {
    let mut entities = Vec::new();

    // Get tank definition to determine capabilities
    let tank_def = match machine_def.tanks.get(&tank_index) {
        Some(def) => def,
        None => return entities, // No definition, no entities
    };

    let tank_name = tank_def.name.as_str().to_string();

    // Check capabilities
    let has_water_level_sensor = tank_def.sensors.contains(&SensorCapability::WaterLevel);

    // Water level sensor (only if tank has water level sensor)
    if has_water_level_sensor {
        entities.push(EntityConfig::Sensor(SensorConfig {
            object_id: Box::leak(format!("tank_{}_water_level", tank_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_TANK, tank_index, TANK_WATER_LEVEL),
            name: Box::leak(format!("{} Water Level", tank_name).into_boxed_str()),
            unique_id: Box::leak(format!("tank_{}_water_level", tank_index).into_boxed_str()),
            icon: "mdi:water-percent",
            unit_of_measurement: "%",
            accuracy_decimals: 0,
            force_update: false,
            device_class: "water",
            state_class: SensorStateClass::StateClassMeasurement,
            disabled_by_default: false,
            entity_category: EntityCategory::None,
        }));
    }

    // Low level warning threshold (if configured)
    if let Some(_threshold) = config.low_level_warning_threshold {
        entities.push(EntityConfig::Number(NumberConfig {
            object_id: Box::leak(format!("tank_{}_low_level_threshold", tank_index).into_boxed_str()),
            key: generate_key(ENTITY_TYPE_TANK, tank_index, TANK_LOW_LEVEL_WARNING_THRESHOLD),
            name: Box::leak(format!("{} Low Level Warning Threshold", tank_name).into_boxed_str()),
            unique_id: Box::leak(format!("tank_{}_low_level_threshold", tank_index).into_boxed_str()),
            icon: "mdi:water-alert",
            min_value: 0.0,
            max_value: 100.0,
            step: 1.0,
            disabled_by_default: false,
            entity_category: EntityCategory::Config,
            unit_of_measurement: "%",
            mode: NumberMode::Box,
            device_class: "",
        }));
    }

    entities
}

// Constants for key parsing - re-export for use in other modules
pub const ENTITY_TYPE_BOILER_CONST: u8 = ENTITY_TYPE_BOILER as u8;
pub const ENTITY_TYPE_GROUP_CONST: u8 = ENTITY_TYPE_GROUP as u8;
pub const ENTITY_TYPE_WATER_TAP_CONST: u8 = ENTITY_TYPE_WATER_TAP as u8;
pub const ENTITY_TYPE_STEAM_WAND_CONST: u8 = ENTITY_TYPE_STEAM_WAND as u8;
pub const ENTITY_TYPE_MACHINE_CONST: u8 = ENTITY_TYPE_MACHINE as u8;
pub const ENTITY_TYPE_TANK_CONST: u8 = ENTITY_TYPE_TANK as u8;

pub const BOILER_TEMPERATURE_CONST: u16 = BOILER_TEMPERATURE;
pub const BOILER_PRESSURE_CONST: u16 = BOILER_PRESSURE;
pub const BOILER_WATER_LEVEL_CONST: u16 = BOILER_WATER_LEVEL;
pub const BOILER_TEMP_SETPOINT_CONST: u16 = BOILER_TEMP_SETPOINT;
pub const BOILER_PRESSURE_SETPOINT_CONST: u16 = BOILER_PRESSURE_SETPOINT;
pub const BOILER_TEMP_KP_CONST: u16 = BOILER_TEMP_KP;
pub const BOILER_TEMP_KI_CONST: u16 = BOILER_TEMP_KI;
pub const BOILER_TEMP_KD_CONST: u16 = BOILER_TEMP_KD;
pub const BOILER_PRESSURE_KP_CONST: u16 = BOILER_PRESSURE_KP;
pub const BOILER_PRESSURE_KI_CONST: u16 = BOILER_PRESSURE_KI;
pub const BOILER_PRESSURE_KD_CONST: u16 = BOILER_PRESSURE_KD;
pub const BOILER_DUTY_CYCLE_CONST: u16 = BOILER_DUTY_CYCLE;
pub const BOILER_MAX_TEMPERATURE_CONST: u16 = BOILER_MAX_TEMPERATURE;
pub const BOILER_MAX_PRESSURE_CONST: u16 = BOILER_MAX_PRESSURE;
pub const BOILER_FILL_THRESHOLD_CONST: u16 = BOILER_FILL_THRESHOLD;

pub const GROUP_INPUT_FLOW_RATE_CONST: u16 = GROUP_INPUT_FLOW_RATE;
pub const GROUP_OUTPUT_FLOW_RATE_CONST: u16 = GROUP_OUTPUT_FLOW_RATE;
pub const GROUP_WEIGHT_CONST: u16 = GROUP_WEIGHT;
pub const GROUP_IS_BREWING_CONST: u16 = GROUP_IS_BREWING;
pub const GROUP_BREW_TIME_CONST: u16 = GROUP_BREW_TIME;
pub const GROUP_PUMP_DUTY_CYCLE_CONST: u16 = GROUP_PUMP_DUTY_CYCLE;
pub const GROUP_FLOW_RATE_SETPOINT_CONST: u16 = GROUP_FLOW_RATE_SETPOINT;
pub const GROUP_OUTPUT_FLOW_RATE_SETPOINT_CONST: u16 = GROUP_OUTPUT_FLOW_RATE_SETPOINT;
pub const GROUP_MAX_BREW_TIME_CONST: u16 = GROUP_MAX_BREW_TIME;
pub const GROUP_AUTO_TARE_ENABLED_CONST: u16 = GROUP_AUTO_TARE_ENABLED;

pub const WATER_TAP_IS_DISPENSING_CONST: u16 = WATER_TAP_IS_DISPENSING;
pub const WATER_TAP_TEMPERATURE_TARGET_CONST: u16 = WATER_TAP_TEMPERATURE_TARGET;
pub const WATER_TAP_MAX_DISPENSE_TIME_CONST: u16 = WATER_TAP_MAX_DISPENSE_TIME;
pub const WATER_TAP_FLOW_RATE_LIMIT_CONST: u16 = WATER_TAP_FLOW_RATE_LIMIT;

pub const STEAM_WAND_TEMPERATURE_TARGET_CONST: u16 = STEAM_WAND_TEMPERATURE_TARGET;
pub const STEAM_WAND_OPENNESS_CONST: u16 = STEAM_WAND_OPENNESS;
pub const STEAM_WAND_PURGE_TIME_CONST: u16 = STEAM_WAND_PURGE_TIME;
pub const STEAM_WAND_MAX_STEAM_TIME_CONST: u16 = STEAM_WAND_MAX_STEAM_TIME;
pub const STEAM_WAND_AUTO_PURGE_ENABLED_CONST: u16 = STEAM_WAND_AUTO_PURGE_ENABLED;
pub const STEAM_WAND_IS_STEAMING_CONST: u16 = STEAM_WAND_IS_STEAMING;
pub const STEAM_WAND_VALVE_OPENNESS_CONST: u16 = STEAM_WAND_VALVE_OPENNESS;

pub const MACHINE_HEATING_ELEMENT_INTERLOCK_CONST: u16 = MACHINE_HEATING_ELEMENT_INTERLOCK;
pub const MACHINE_MODE_CONST: u16 = MACHINE_MODE;

pub const TANK_WATER_LEVEL_CONST: u16 = TANK_WATER_LEVEL;
pub const TANK_LOW_LEVEL_WARNING_THRESHOLD_CONST: u16 = TANK_LOW_LEVEL_WARNING_THRESHOLD;

pub const BOILER_PID_P_TERM_CONST: u16 = BOILER_PID_P_TERM;
pub const BOILER_PID_I_TERM_CONST: u16 = BOILER_PID_I_TERM;
pub const BOILER_PID_D_TERM_CONST: u16 = BOILER_PID_D_TERM;

pub const BOILER_TEMP_KP_UPPER_LIMIT_CONST: u16 = BOILER_TEMP_KP_UPPER_LIMIT;
pub const BOILER_TEMP_KP_LOWER_LIMIT_CONST: u16 = BOILER_TEMP_KP_LOWER_LIMIT;
pub const BOILER_TEMP_KI_UPPER_LIMIT_CONST: u16 = BOILER_TEMP_KI_UPPER_LIMIT;
pub const BOILER_TEMP_KI_LOWER_LIMIT_CONST: u16 = BOILER_TEMP_KI_LOWER_LIMIT;
pub const BOILER_TEMP_KD_UPPER_LIMIT_CONST: u16 = BOILER_TEMP_KD_UPPER_LIMIT;
pub const BOILER_TEMP_KD_LOWER_LIMIT_CONST: u16 = BOILER_TEMP_KD_LOWER_LIMIT;
pub const BOILER_CONTROL_MODE_CONST: u16 = BOILER_CONTROL_MODE;

pub const GROUP_PID_P_TERM_CONST: u16 = GROUP_PID_P_TERM;
pub const GROUP_PID_I_TERM_CONST: u16 = GROUP_PID_I_TERM;
pub const GROUP_PID_D_TERM_CONST: u16 = GROUP_PID_D_TERM;
