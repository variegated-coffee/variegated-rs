use alloc::boxed::Box;
use alloc::format;
use alloc::string::ToString;
use esphome_device::DeviceConfig;
use variegated_controller_types::MachineDefinition;

// Legacy hard-coded entity keys (deprecated - use dynamic entities instead)
#[allow(dead_code)]
pub const BREW_TEMP_KEY: u32 = 1;
#[allow(dead_code)]
pub const BREW_PRESSURE_KEY: u32 = 2;
#[allow(dead_code)]
pub const BREW_BOILER_DUTY_CYCLE_KEY: u32 = 3;
#[allow(dead_code)]
pub const PUMP_DUTY_CYCLE_KEY: u32 = 4;
#[allow(dead_code)]
pub const GROUP_FLOW_RATE_KEY: u32 = 5;
#[allow(dead_code)]
pub const BREW_TEMP_SETPOINT_KEY: u32 = 6;
#[allow(dead_code)]
pub const BREW_BOILER_KP_KEY: u32 = 7;
#[allow(dead_code)]
pub const BREW_BOILER_KI_KEY: u32 = 8;
#[allow(dead_code)]
pub const BREW_BOILER_KD_KEY: u32 = 9;
#[allow(dead_code)]
pub const IS_BREWING_KEY: u32 = 10;
#[allow(dead_code)]
pub const BREW_TIME_KEY: u32 = 11;
#[allow(dead_code)]
pub const OUTPUT_FLOW_RATE_KEY: u32 = 12;
#[allow(dead_code)]
pub const OUTPUT_WEIGHT_KEY: u32 = 13;

pub static DEVICE_CONFIG: &DeviceConfig = &DeviceConfig {
    name: "GS3 Test",
    password: None,
    mac_address: "",
    esphome_version: "",
    compilation_time: "",
    model: "",
    has_deep_sleep: false,
    project_name: "",
    project_version: "",
    webserver_port: 0,
    legacy_bluetooth_proxy_version: 0,
    bluetooth_proxy_feature_flags: 0,
    manufacturer: "",
    friendly_name: "",
    legacy_voice_assistant_version: 0,
    voice_assistant_feature_flags: 0,
    suggested_area: "",
    bluetooth_mac_address: "",
};

/// Build a dynamic device configuration based on machine definition and runtime information
pub fn build_device_config(
    machine_def: &MachineDefinition,
    mac_address: &str,
) -> DeviceConfig<'static> {
    // Extract machine name
    let machine_name = machine_def.name.as_str();

    // Create a friendly device name
    let device_name = Box::leak(format!("{} Controller", machine_name).into_boxed_str());
    let friendly_name = Box::leak(machine_name.to_string().into_boxed_str());

    // Determine manufacturer and model based on machine definition
    let manufacturer = "Variegated";
    let model = "Variegated Controller";

    DeviceConfig {
        name: device_name,
        password: None,
        mac_address: Box::leak(mac_address.to_string().into_boxed_str()),
        esphome_version: "2025.08.1",
        compilation_time: "",
        model,
        has_deep_sleep: false,
        project_name: "variegated-coffee.comms-firmware",
        project_version: env!("CARGO_PKG_VERSION"),
        webserver_port: 0,
        legacy_bluetooth_proxy_version: 0,
        bluetooth_proxy_feature_flags: 0,
        manufacturer,
        friendly_name,
        legacy_voice_assistant_version: 0,
        voice_assistant_feature_flags: 0,
        suggested_area: "Kitchen",
        bluetooth_mac_address: "",
    }
}
