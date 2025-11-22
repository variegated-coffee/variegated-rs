//! Configuration constants for the comms firmware

use trouble_host::prelude::BdAddr;

// WiFi configuration
pub const SSID: &str = env!("SSID");
pub const PASSWORD: &str = env!("PASSWORD");

// SNTP configuration
pub const NTP_SERVER: &str = "pool.ntp.org";
pub const USEC_IN_SEC: u64 = 1_000_000;

// BLE device addresses
pub fn belka_address() -> BdAddr {
    BdAddr::new([0x3E, 0x60, 0xEB, 0x3C, 0x1C, 0x78])
}

pub fn acaia_address() -> BdAddr {
    BdAddr::new([0x2f, 0xa0, 0x1a, 0x97, 0x1c, 0x00])
}

// UART configuration for application processor
pub fn uart_config() -> esp_hal::uart::Config {
    esp_hal::uart::Config::default()
        .with_baudrate(576_000)
        .with_data_bits(esp_hal::uart::DataBits::_8)
        .with_parity(esp_hal::uart::Parity::None)
        .with_stop_bits(esp_hal::uart::StopBits::_1)
        .with_hw_flow_ctrl(esp_hal::uart::HwFlowControl {
            cts: esp_hal::uart::CtsConfig::Enabled,
            rts: esp_hal::uart::RtsConfig::Enabled(122),
        })
}
