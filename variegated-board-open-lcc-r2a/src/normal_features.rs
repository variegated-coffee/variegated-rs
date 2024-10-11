use embassy_rp::{bind_interrupts, i2c, Peripherals, peripherals, pwm, spi, uart};
use embassy_rp::i2c::Config;
use variegated_board_features::OpenLCCBoardFeatures;
use crate::R2ABoardFeatures;

use embassy_rp::gpio::{AnyPin, Pin};
use embassy_rp::peripherals::{PIN_17, PIN_24, PWM_CH4, SPI1};
use embassy_rp::pwm::Pwm;
use embassy_rp::spi::Spi;
use embassy_rp::uart::{BufferedInterruptHandler, BufferedUart, Uart};
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    UART0_IRQ => uart::InterruptHandler<peripherals::UART0>;
    UART1_IRQ => uart::InterruptHandler<peripherals::UART1>;
    I2C0_IRQ => i2c::InterruptHandler<peripherals::I2C0>;
    I2C1_IRQ => i2c::InterruptHandler<peripherals::I2C1>;
});

pub fn create_board_features(p: Peripherals) -> R2ABoardFeatures {
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

    let esp32_uart = Uart::new(p.UART0, p.PIN_0, p.PIN_1, Irqs, p.DMA_CH4, p.DMA_CH5, uart::Config::default());
    let iox_uart = Uart::new(p.UART1, p.PIN_4, p.PIN_5, Irqs, p.DMA_CH6, p.DMA_CH7, uart::Config::default());

    let (esp32_uart_tx, esp32_uart_rx) = esp32_uart.split();
    let (iox_uart_tx, iox_uart_rx) = iox_uart.split();

    OpenLCCBoardFeatures {
        esp32_uart_rx,
        esp32_uart_tx,
        iox_uart_rx: Some(iox_uart_rx),
        iox_uart_tx: Some(iox_uart_tx),
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