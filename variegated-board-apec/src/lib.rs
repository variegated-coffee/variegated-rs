#![no_std]

use bitflags::bitflags;
use variegated_board_features::{AllPurposeEspressoControllerBoardFeatures, BootloaderFeatures, OpenLCCBoardFeatures};
use embassy_rp::{bind_interrupts, i2c, peripherals, Peripherals, pwm, spi, uart};
use embassy_rp::gpio::{AnyPin, Input, Level, Output, Pin, Pull};
use embassy_rp::i2c::Config;
use embassy_rp::pwm::Pwm;
use embassy_rp::spi::{Phase, Polarity, Spi};
use embassy_rp::uart::{BufferedInterruptHandler, BufferedUart, Uart};
use static_cell::StaticCell;
use variegated_embassy_ads124s08::{ADS124S08, WaitStrategy};
use variegated_embassy_dual_c595_shift_register::DualC595ShiftRegister;

bind_interrupts!(struct Irqs {
    UART0_IRQ => uart::InterruptHandler<peripherals::UART0>;
    UART1_IRQ => uart::InterruptHandler<peripherals::UART1>;
    I2C0_IRQ => i2c::InterruptHandler<peripherals::I2C0>;
    I2C1_IRQ => i2c::InterruptHandler<peripherals::I2C1>;
});

pub const FLASH_SIZE: usize = 2 * 1024 * 1024;

pub type ApecR0DBoardFeatures = AllPurposeEspressoControllerBoardFeatures<
    'static,
    peripherals::UART1,
    peripherals::UART0,
    peripherals::I2C1,
    peripherals::SPI1,
    peripherals::PWM_CH4,
    peripherals::PWM_CH5,
    peripherals::PWM_CH5,
    peripherals::PWM_CH1,
    peripherals::PIN_10,
    peripherals::PIN_24,
    peripherals::PIN_25,
    peripherals::PIN_26,
>;

bitflags! {
    pub struct ApecR0dShiftRegisterPositions: u16 {
        const CN1_3V3 = 0b0000_0000_0000_00001;
        const CN1_12V = 0b0000_0000_0000_0010;
        const VOUT1 = 0b0000_0000_0000_0100;
        const VOUT2 = 0b0000_0000_0000_1000;
        const CN11 = 0b0000_0000_0001_0000;
        const CN10 = 0b0000_0000_0010_0000;
        const VOUT3 = 0b0000_0000_0100_0000;
        const VOUT4 = 0b0000_0000_1000_0000;
        const CN9_7 = 0b0000_0001_0000_0000;
        const CN9_5 = 0b0000_0010_0000_0000;
        const CN9_3 = 0b0000_0100_0000_0000;
        const CN2_FA7 = 0b0001_0000_0000_0000;
        const CN2_FA8 = 0b0010_0000_0000_0000;
        const CN2_FA9 = 0b0100_0000_0000_0000;
        const CN2_FA10 = 0b1000_0000_0000_0000;
    }
}

pub fn create_board_features(p: Peripherals) -> ApecR0DBoardFeatures {
/*
    let mut led_pwm: Option<Pwm<peripherals::PWM_CH4>> = None;
    let mut led_pin: Option<AnyPin> = None;

    if cfg!(feature = "led-pwm") {
        led_pwm = Some(Pwm::new_output_b(p.PWM_CH4, p.PIN_25, pwm::Config::default()));
    } else {
        led_pin = Some(p.PIN_25.degrade());
    }
*/

    let serial_out = Output::new(p.PIN_6.degrade(), Level::Low);
    let sclk = Output::new(p.PIN_7.degrade(), Level::Low);
    let rclk = Output::new(p.PIN_8.degrade(), Level::Low);
    
    let dual_shift_register = DualC595ShiftRegister::new(serial_out, rclk, sclk);

    cfg_if::cfg_if! {
        if #[cfg(feature = "cn94-pwm")] {
            let cn9_4_pin = None;
            let cn9_4_pwm = Some(Pwm::new_output_b(p.PWM_CH4, p.PIN_9, pwm::Config::default()));
        } else {
            let cn9_4_pin = Some(p.PIN_9.degrade());
            let cn9_4_pwm = None;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(all(feature = "cn96-pwm", not(feature = "cn98-pwm")))] {
            let cn9_6_pin = None;
            let cn9_6_pwm = Some(Pwm::new_output_a(p.PWM_CH5, p.PIN_10, pwm::Config::default()));;
            
            let cn9_8_pin = Some(p.PIN_11.degrade());
            let cn9_8_pwm = None;
        } else if #[cfg(all(not(feature = "cn96-pwm"), feature = "cn98-pwm"))] {
            let cn9_6_pin = Some(p.PIN_10.degrade());
            let cn9_6_pwm = None;

            let cn9_8_pin = None;
            let cn9_8_pwm = Some(Pwm::new_output_b(p.PWM_CH5, p.PIN_11, pwm::Config::default()));;
        } else if #[cfg(all(feature = "cn96-pwm", feature = "cn98-pwm"))] {
            let cn9_6_pin = None;
            let cn9_8_pin = None;
            
            let cn9_6_pwm = None;
            let cn9_8_pwm = None;
        } else {
            let cn9_6_pin = Some(p.PIN_10);
            let cn9_6_pwm = None;
            let cn9_8_pin = Some(p.PIN_11.degrade());
            let cn9_8_pwm = None;
        }
    }

    let mut spi_config = spi::Config::default();
    spi_config.frequency = 100_000;
    spi_config.phase = Phase::CaptureOnSecondTransition;
    spi_config.polarity = Polarity::IdleLow;

    let spi_bus = Some(spi::Spi::new(p.SPI1, p.PIN_14, p.PIN_15, p.PIN_12, p.DMA_CH0, p.DMA_CH1, spi_config));

    let cn1_uart = Uart::new(p.UART0, p.PIN_16, p.PIN_17, Irqs, p.DMA_CH6, p.DMA_CH7, uart::Config::default());
    let esp32_uart = Uart::new(p.UART1, p.PIN_20, p.PIN_21, Irqs, p.DMA_CH4, p.DMA_CH5, uart::Config::default());

    let (cn1_uart_tx, cn1_uart_rx) = cn1_uart.split();
    let (esp32_uart_tx, esp32_uart_rx) = esp32_uart.split();

    let i2c_bus = i2c::I2c::new_async(p.I2C1, p.PIN_19, p.PIN_18, Irqs, Config::default());

    let adc = ADS124S08::new(
        Output::new(p.PIN_0.degrade(), Level::High),
        WaitStrategy::UseDrdyPin(Input::new(p.PIN_23.degrade(), Pull::Up)),
    );
    
    AllPurposeEspressoControllerBoardFeatures {
        adc_cs_pin: None,
        
        ads124s08: Some(adc),
        
        sd_cs_pin: Some(p.PIN_1.degrade()),

        cn14_10_pin: None, //Some(p.PIN_2.degrade()),
        cn14_8_pin: Some(p.PIN_3.degrade()),
        cn14_6_pin: Some(p.PIN_4.degrade()),
        cn14_4_pin: Some(p.PIN_5.degrade()),
        
        cn14_10_pwm: Some(Pwm::new_output_a(p.PWM_CH1, p.PIN_2, pwm::Config::default())),

        ser_pin: None,
        rclk_pin: None,
        srck_pin: None,

        dual_shift_register: Some(dual_shift_register),
        
        cn9_4_pin,
        cn9_6_pin,
        cn9_8_pin,
        
        cn9_4_pwm,
        cn9_6_pwm,
        cn9_8_pwm,
        
        spi_bus,
        
        settings_flash_cs_pin: Some(p.PIN_13.degrade()),
        
        cn1_uart_rx: Some(cn1_uart_rx),
        cn1_uart_tx: Some(cn1_uart_tx),
        
        i2c_bus: Some(i2c_bus),
        
        esp32_uart_rx,
        esp32_uart_tx,
        
        serial_boot_pin: Some(p.PIN_22.degrade()),
        adc_drdy_pin: None,
        
        cn12_pin: Some(p.PIN_24),
        cn13_pin: Some(p.PIN_25),
        
        cn14_3_pin: Some(p.PIN_26),
        cn14_5_pin: Some(p.PIN_27.degrade()),
        cn14_7_pin: Some(p.PIN_28.degrade()),
        cn14_9_pin: Some(p.PIN_29.degrade()),
        
        pio0: Some(p.PIO0),
        pio1: Some(p.PIO1),
        
        usb: Some(p.USB),
    }
}

pub fn create_bootloader_features(p: Peripherals) -> BootloaderFeatures<'static, peripherals::UART0> {
    let bootloader_uart = Uart::new_blocking(p.UART0, p.PIN_0, p.PIN_1, uart::Config::default());

    BootloaderFeatures {
        bootloader_uart,
        bootloader_trigger_pin: p.PIN_22.degrade(),
        watchdog: p.WATCHDOG,
        flash: p.FLASH,
        flash_size: 2 * 1024 * 1024,
    }
}