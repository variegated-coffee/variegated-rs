#![no_std]
#![no_main]

use embassy_rp::gpio::{Input, Level, Output, Pin, Pull};
use embassy_rp::{bind_interrupts, i2c, Peripherals, peripherals, pwm, spi, uart};
use embassy_rp::i2c::Config;
use embassy_rp::pwm::Pwm;
use embassy_rp::spi::{Phase, Polarity};
use embassy_rp::uart::Uart;
use variegated_board_features::GravityBoardFeatures;
use variegated_embassy_ads124s08::{ADS124S08, WaitStrategy};

bind_interrupts!(struct Irqs {
    UART0_IRQ => uart::InterruptHandler<peripherals::UART0>;
    I2C0_IRQ => i2c::InterruptHandler<peripherals::I2C0>;
});

pub type GravityR0ABoardFeatures = GravityBoardFeatures<
    'static,
    peripherals::UART0,
    peripherals::I2C0,
    peripherals::SPI0,
    peripherals::PWM_CH0,
>;

pub fn create_board_features(p: Peripherals) -> GravityR0ABoardFeatures<> {
    cfg_if::cfg_if! {
        if #[cfg(feature = "led-pwm")] {
            let led_pin = None;
            let led_pwm = Some(Pwm::new_output_b(p.PWM_CH0, p.PIN_17, pwm::Config::default()));
        } else {
            let led_pin = Some(p.PIN_17.degrade());
            let led_pwm = None;
        }
    }

    let mut spi_config = spi::Config::default();
    spi_config.frequency = 5_000_000;
    spi_config.phase = Phase::CaptureOnSecondTransition;
    spi_config.polarity = Polarity::IdleLow;

    let uart_rx_pin = p.PIN_0;
    let uart_tx_pin = p.PIN_1;

    let spi_bus = Some(spi::Spi::new(p.SPI0, p.PIN_6, p.PIN_7, p.PIN_4, p.DMA_CH0, p.DMA_CH1, spi_config));
/*    let uart = Uart::new(p.UART0, uart_rx_pin, uart_tx_pin, Irqs, p.DMA_CH6, p.DMA_CH7, uart::Config::default());

    let (uart_tx, uart_rx) = uart.split();*/
    
    let mut i2c_config = Config::default();
    i2c_config.frequency = 400_000;
    
    let blocking_i2c_bus = if true {
        Some(i2c::I2c::new_blocking(p.I2C0, p.PIN_21, p.PIN_20, i2c_config))
    } else {
        None
        //i2c::I2c::new_async(p.I2C0, p.PIN_21, p.PIN_20, Irqs, i2c_config)
    };
    
    let async_i2c_bus = if false {
        None
        //Some(i2c::I2c::new_async(p.I2C0, p.PIN_21, p.PIN_20, Irqs, i2c_config))
    } else {
        None
    };
    

    let adc = ADS124S08::new(
        Output::new(p.PIN_8.degrade(), Level::High),
        WaitStrategy::UseDrdyPin(Input::new(p.PIN_3.degrade(), Pull::Up)),
    );

    GravityBoardFeatures {
        adc_cs_pin: None,

        ads124s08: Some(adc),

        spi_bus,
        blocking_i2c_bus,
        async_i2c_bus,

        uart_rx: None,
        uart_tx: None,

        uart_rx_pin: Some(uart_rx_pin.degrade()),
        uart_tx_pin: Some(uart_tx_pin.degrade()),

        adc_res_pin: Some(p.PIN_2.degrade()),
        adc_drdy_pin: None,
        adc_start_pin: Some(p.PIN_9.degrade()),

        excitation1_en_pin: Some(p.PIN_15.degrade()),
        excitation2_en_pin: Some(p.PIN_14.degrade()),
        excitation3_en_pin: Some(p.PIN_13.degrade()),
        excitation4_en_pin: Some(p.PIN_12.degrade()),

        led_pin,
        led_pwm,

        usb: Some(p.USB),
    }
}
