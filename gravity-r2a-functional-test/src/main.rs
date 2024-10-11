#![no_std]
#![no_main]

extern crate alloc;

use alloc::format;
use defmt::*;
use embassy_executor::{Executor, Spawner};
use embassy_rp::{bind_interrupts, gpio, i2c, peripherals, usb};
use embassy_rp::gpio::{AnyPin, Drive, Input, Pull};
use embassy_rp::i2c::{Blocking, Config, I2c};
use embassy_rp::peripherals::{I2C0, I2C1, PIO0, SPI0, SPI1, USB};
use embassy_rp::spi::{Async, Spi};
use embassy_rp::usb::Driver;
use embassy_time::{Instant, Timer, Delay};
use embedded_alloc::Heap;
use embedded_hal_1::digital::OutputPin;
use embedded_hal_async::spi::SpiBus;
use gpio::{Level, Output};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};
//use variegated_board_features::GravityBoardFeatures;
//use variegated_board_gravity::GravityR2ABoardFeatures;
use variegated_embassy_ads124s08::{ADS124S08, ADS124S08Error, Code, WaitStrategy};
use variegated_embassy_ads124s08::registers::{DataRate, Filter, IDACMagnitude, IDACMux, InputMultiplexerRegister, Mode, Mux, PGAGain, ReferenceInput};
use movavg;

use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use embedded_graphics::mono_font::ascii::FONT_9X18_BOLD;
use variegated_embassy_ads124s08::registers::PGAGain::Gain32;
use oled_async::{prelude::*, Builder};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
    I2C1_IRQ => i2c::InterruptHandler<peripherals::I2C1>;
});

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

#[global_allocator]
static HEAP: Heap = Heap::empty();

static LATEST_WEIGHT_SIG: Signal<CriticalSectionRawMutex, f32> = Signal::new();

static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
#[cortex_m_rt::entry]
fn main() -> ! {
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        unwrap!(spawner.spawn(main_task(spawner)))
    });
}

#[embassy_executor::task]
async fn main_task(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    //let mut board_features = variegated_board_gravity::create_board_features(p);

    let driver = Driver::new(p.USB, Irqs);
    unwrap!(spawner.spawn(logger_task(driver)));

    Timer::after_secs(5).await;

    log_info!("Starting functional test");

    //let i2c = board_features.async_i2c0_bus.take().expect("I2C must be unused");

    //let mut tare = Input::new(board_features.uart_rx_pin.take().expect("Tare pin must be unused"), Pull::Up);

//    let button1_input = Input::new(board_features.button1_pin.take().expect("Button1 pin must be unused"), Pull::Up);
//    let button2_input = Input::new(board_features.button2_pin.take().expect("Button2 pin must be unused"), Pull::Up);

    let nau7082_i2c_config = Config::default();
    let nau7802_a_i2c_bus = i2c::I2c::new_async(p.I2C1, p.PIN_11, p.PIN_10, Irqs, nau7082_i2c_config);

//    let nau7802_b_i2c_bus =

//    let nau7802_a_i2c_bus = board_features.nau7802_a_i2c_bus.take().expect("I2C must be unused");

    spawner.spawn(test_nau7802(
        nau7802_a_i2c_bus,
/*        button1_input,
        button2_input,*/
    )).expect("Failed to spawn test_nau7802");
    /*spawner.spawn(run_display(i2c)).expect("Failed to spawn display runner");*/

    //spawner.spawn(test_pio_i2c()).expect("Failed to spawn PIO i2c test");

    let mut led1 = Output::new(p.PIN_14, Level::Low);
    let mut led2 = Output::new(p.PIN_15, Level::High);

    loop {
        led1.set_high();
        led2.set_low();
        Timer::after_secs(1).await;

        led1.set_low();
        led2.set_high();
        Timer::after_secs(1).await;
    }
}

#[embassy_executor::task]
async fn test_nau7802(
    mut i2c: I2c<'static, I2C1, i2c::Async>,
/*    mut tare_input: Input<'static, AnyPin>,
    mut calibrate_input: Input<'static, AnyPin>*/
) -> ! {
    let mut nau7802 = Nau7802::new(Nau7802DataAvailableStrategy::<Input<AnyPin>>::Polling, Delay {});
    nau7802.init(
        &mut i2c,
        variegated_embassy_nau7802::Ldo::L4v5,
        variegated_embassy_nau7802::Gain::G128,
        variegated_embassy_nau7802::SamplesPerSecond::SPS10,
    ).await.unwrap();

    enum CalibrationType {
        Zero,
        Hundered,
    }

    let mut adc_val_ma: MovAvg<i32, i32, 10> = MovAvg::new();
    let mut zero = -73700;
    let mut hundered = -40000;

    let mut current_calibration = CalibrationType::Zero;

    let mut tare = zero;

    loop {
        let res = nau7802.read(&mut i2c).await;
        if let Ok(res) = res {
            adc_val_ma.feed(res);
/*
            if tare_input.is_low() {
                tare = adc_val_ma.get();
                log_info!("Tare: {}", tare);
            }

            if calibrate_input.is_low() {
                match current_calibration {
                    CalibrationType::Zero => {
                        zero = adc_val_ma.get();
                        log_info!("Zero: {}", zero);
                        current_calibration = CalibrationType::Hundered;
                    },
                    CalibrationType::Hundered => {
                        hundered = adc_val_ma.get();
                        log_info!("Calibrate: {}", hundered);
                    }
                }
            }
*/
            let slope = 100.0 / (hundered - zero) as f32;
            let weight = slope * (res as f32 - tare as f32);

            log_info!("Voltage: {}, Weight: {:?}", res, weight);

            LATEST_WEIGHT_SIG.signal(weight);

            Timer::after_secs(3).await;
        } else {
            log_info!("Failed to read NAU7802, err");
        }
    }
}

#[embassy_executor::task]
async fn run_display(mut i2c: I2c<'static, I2C0, i2c::Async>) -> ! {
    type Display = oled_async::displays::sh1107::Sh1107_128_128;

    let di = display_interface_i2c::I2CInterface::new(i2c, 0x3D, 0x40);

    let raw_disp = Builder::new(Display {})
        .with_rotation(crate::DisplayRotation::Rotate180)
        .connect(di);

    let mut disp: GraphicsMode<_, _, { 128 * 128 / 8 }> = raw_disp.into();

    let res = disp.init().await;

    if let Err(res) = res {
        log_info!("Failed to initialize display");
        log::info!("Error: {:?}", res);

        loop { Timer::after_secs(1).await; }
    }

    log_info!("Display initialized");

    let res = disp.flush().await;

    if let Err(res) = res {
        log_info!("Failed to flush display");

        loop { Timer::after_secs(1).await; }
    }

    log_info!("Display flushed");


    if let Ok(res) = res {
        let res= disp.flush().await;

        if let Ok(res) = res {
            log_info!("Circle drawn");
        } else {
            log_info!("Failed to flush display");
        }
    }

    // Create a new character style
    let style = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);

    loop {
        disp.clear();
        let val = LATEST_WEIGHT_SIG.wait().await;

        let str = format!("{:.1} g", val);

        let res = Text::new(&str, Point::new(10, 20), style)
            .draw(&mut disp);

        let res= disp.flush().await;
        if let Err(res) = res {
            log_info!("Failed to flush display");
        }
    }
}

use cortex_m::asm;
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::primitives::{Circle, PrimitiveStyle};
use embedded_graphics::text::TextStyle;
use movavg::MovAvg;
use variegated_embassy_nau7802::{Nau7802, Nau7802DataAvailableStrategy};
use variegated_log::log_info;

fn busy_wait_us(us: u32) {
    // This constant should be calibrated for your specific MCU and clock speed
    const CYCLES_PER_US: u32 = 133;
    let cycles = us * CYCLES_PER_US;
    asm::delay(cycles);
}