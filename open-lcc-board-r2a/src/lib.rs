#![no_std]

use open_lcc_board_features::{BootloaderFeatures, OpenLCCBoardFeatures};
use embassy_rp::{bind_interrupts, i2c, peripherals, Peripherals, pwm, spi, uart};
use embassy_rp::gpio::{AnyPin, Pin};
use embassy_rp::i2c::Config;
use embassy_rp::peripherals::{PWM_CH4, SPI1};
use embassy_rp::pwm::Pwm;
use embassy_rp::spi::Spi;
use embassy_rp::uart::{BufferedInterruptHandler, BufferedUart};
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    UART0_IRQ => BufferedInterruptHandler<peripherals::UART0>;
    UART1_IRQ => BufferedInterruptHandler<peripherals::UART1>;
    I2C0_IRQ => i2c::InterruptHandler<peripherals::I2C0>;
    I2C1_IRQ => i2c::InterruptHandler<peripherals::I2C1>;
});

pub const FLASH_SIZE: usize = 2 * 1024 * 1024;

pub fn create_board_features(p: Peripherals) -> OpenLCCBoardFeatures<
    'static,
    peripherals::PWM_CH4,
    peripherals::UART0,
    peripherals::UART1,
    peripherals::I2C0,
    peripherals::I2C1,
    peripherals::SPI1,
    peripherals::SPI0
> {
    static ESP32_TX_BUF: StaticCell<[u8; 16]> = StaticCell::new();
    let esp32_tx_buf = &mut ESP32_TX_BUF.init([0; 16])[..];
    static ESP32_RX_BUF: StaticCell<[u8; 16]> = StaticCell::new();
    let esp32_rx_buf = &mut ESP32_RX_BUF.init([0; 16])[..];
    let esp32_uart = BufferedUart::new(p.UART0, Irqs, p.PIN_0, p.PIN_1, esp32_tx_buf, esp32_rx_buf, uart::Config::default());

    static IOX_TX_BUF: StaticCell<[u8; 16]> = StaticCell::new();
    let iox_tx_buf = &mut IOX_TX_BUF.init([0; 16])[..];
    static IOX_RX_BUF: StaticCell<[u8; 16]> = StaticCell::new();
    let iox_rx_buf = &mut IOX_RX_BUF.init([0; 16])[..];
    let iox_uart = BufferedUart::new(p.UART1, Irqs, p.PIN_4, p.PIN_5, iox_tx_buf, iox_rx_buf, uart::Config::default());
    
    let mut led_pwm: Option<Pwm<PWM_CH4>> = None;
    let mut led_pin: Option<AnyPin> = None;
    
    if cfg!(feature = "led-pwm") {
        led_pwm = Some(Pwm::new_output_b(p.PWM_CH4, p.PIN_25, pwm::Config::default()));
    } else {
        led_pin = Some(p.PIN_25.degrade());
    }
    
    let mut settings_flash_spi: Option<Spi<SPI1, spi::Async>> = None; 
    let mut settings_flash_sclk_pin: Option<AnyPin> = None;
    let mut settings_flash_miso_pin: Option<AnyPin> = None;
    let mut settings_flash_mosi_pin: Option<AnyPin> = None;
    let mut settings_flash_wp_d2_pin: Option<AnyPin> = None;
    let mut settings_flash_res_d3_pin: Option<AnyPin> = None;
    
    if cfg!(feature = "qspi-settings-flash") {
        unimplemented!("QSPI settings flash not yet implemented");
    } else {
        settings_flash_spi = Some(spi::Spi::new(p.SPI1, p.PIN_10, p.PIN_11, p.PIN_12, p.DMA_CH0, p.DMA_CH1, spi::Config::default()));
    }
    
    let mut sd_card_spi: Option<Spi<peripherals::SPI0, spi::Async>> = None;
    let mut sd_cs_dat3_pin: Option<AnyPin> = None;
    let mut sd_sclk_pin: Option<AnyPin> = None;
    let mut sd_miso_pin: Option<AnyPin> = None;
    let mut sd_mosi_pin: Option<AnyPin> = None;
    let mut sd_dat1_pin: Option<AnyPin> = None;
    let mut sd_dat2_pin: Option<AnyPin> = None;
    
    if cfg!(feature = "4-wire-sd-card") {
        unimplemented!("4-wire SD card not yet implemented");
    } else {
        sd_card_spi = Some(spi::Spi::new(p.SPI0, p.PIN_18, p.PIN_19, p.PIN_20, p.DMA_CH2, p.DMA_CH3, spi::Config::default()));
    }
    
    OpenLCCBoardFeatures {
        esp32_uart,
        iox_uart,
        qwiic1_i2c: i2c::I2c::new_async(p.I2C0, p.PIN_9, p.PIN_8, Irqs, Config::default()),
        qwiic2_i2c: i2c::I2c::new_async(p.I2C1, p.PIN_7, p.PIN_6, Irqs, Config::default()),
        settings_flash_spi,
        sd_card_spi,
        serial_boot_pin: p.PIN_24.degrade(),
        settings_flash_cs_pin: p.PIN_17.degrade(),
        settings_flash_sclk_pin,
        settings_flash_miso_pin,
        settings_flash_mosi_pin,
        settings_flash_wp_d2_pin,
        settings_flash_res_d3_pin,
        sd_det_a_pin: p.PIN_15.degrade(),
        sd_det_b_pin: Some(p.PIN_16.degrade()),
        sd_cs_dat3_pin,
        sd_sclk_pin,
        sd_miso_pin,
        sd_mosi_pin,
        sd_dat1_pin,
        sd_dat2_pin,
        led_pin,
        led_pwm,
        usb: p.USB,
    }
}

pub fn create_bootloader_features(p: Peripherals) -> BootloaderFeatures<'static, peripherals::UART0> {
    static BOOTLOADER_TX_BUF: StaticCell<[u8; 16]> = StaticCell::new();
    let bootloader_tx_buf = &mut BOOTLOADER_TX_BUF.init([0; 16])[..];
    static BOOTLOADER_RX_BUF: StaticCell<[u8; 16]> = StaticCell::new();
    let bootloader_rx_buf = &mut BOOTLOADER_RX_BUF.init([0; 16])[..];
    let bootloader_uart = BufferedUart::new(p.UART0, Irqs, p.PIN_0, p.PIN_1, bootloader_tx_buf, bootloader_rx_buf, uart::Config::default());
    
    BootloaderFeatures {
        bootloader_uart,
        bootloader_trigger_pin: p.PIN_24.degrade(),
        watchdog: p.WATCHDOG,
        flash: p.FLASH,
        flash_size: 2 * 1024 * 1024,
    }
}