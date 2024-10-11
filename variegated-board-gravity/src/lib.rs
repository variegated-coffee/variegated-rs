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
/*    I2C1_IRQ => i2c::InterruptHandler<peripherals::I2C1>;*/
});

pub type GravityR2ABoardFeatures = GravityBoardFeatures<
    'static,
    peripherals::UART0,
    peripherals::I2C0,
    peripherals::I2C1,
    peripherals::I2C1,
    peripherals::PWM_CH7,
    peripherals::PWM_CH7,
>;

pub fn create_board_features(p: Peripherals) -> GravityR2ABoardFeatures<> {
    cfg_if::cfg_if! {
        if #[cfg(feature = "led1-pwm")] {
            let led1_pin = None;
            let led1_pwm = Some(Pwm::new_output_b(p.PWM_CH0, p.PIN_14, pwm::Config::default()));
        } else {
            let led1_pin = Some(p.PIN_14.degrade());
            let led1_pwm = None;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "led2-pwm")] {
            let led2_pin = None;
            let led2_pwm = Some(Pwm::new_output_b(p.PWM_CH0, p.PIN_15, pwm::Config::default()));
        } else {
            let led2_pin = Some(p.PIN_15.degrade());
            let led2_pwm = None;
        }
    }

    let uart_rx_pin = p.PIN_0;
    let uart_tx_pin = p.PIN_1;

/*    let uart = Uart::new(p.UART0, uart_rx_pin, uart_tx_pin, Irqs, p.DMA_CH6, p.DMA_CH7, uart::Config::default());

    let (uart_tx, uart_rx) = uart.split();*/
    
    let mut i2c0_config = Config::default();
    i2c0_config.frequency = 400_000;
    
    let blocking_i2c0_bus = if false {
        None
        //Some(i2c::I2c::new_blocking(p.I2C0, p.PIN_21, p.PIN_20, i2c0_config))
    } else {
        None
        //i2c::I2c::new_async(p.I2C0, p.PIN_21, p.PIN_20, Irqs, i2c_config)
    };
    
    let async_i2c0_bus = if true {
        Some(i2c::I2c::new_async(p.I2C0, p.PIN_21, p.PIN_20, Irqs, i2c0_config))
    } else {
        None
    };
    
    let nau7082_i2c_config = Config::default();

    GravityBoardFeatures {
        blocking_i2c0_bus,
        async_i2c0_bus,

        nau7802_a_i2c_bus: None, //Some(i2c::I2c::new_async(p.I2C1, p.PIN_11, p.PIN_10, Irqs, nau7082_i2c_config)),
        nau7802_a_drdy_pin: Some(p.PIN_7.degrade()),
        nau7802_b_i2c_bus: None,
        nau7802_b_drdy_pin: Some(p.PIN_6.degrade()),

        uart_rx: None,
        uart_tx: None,

        uart_rx_pin: Some(uart_rx_pin.degrade()),
        uart_tx_pin: Some(uart_tx_pin.degrade()),

        button1_pin: Some(p.PIN_25.degrade()),
        button2_pin: Some(p.PIN_12.degrade()),

        led1_pin,
        led1_pwm,

        led2_pin,
        led2_pwm,

        usb: Some(p.USB),
        
        pio0: Some(p.PIO0),
        pio1: Some(p.PIO1)
    }
}
