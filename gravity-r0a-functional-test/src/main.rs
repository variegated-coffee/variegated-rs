#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::{Executor, Spawner};
use embassy_rp::{bind_interrupts, gpio, usb};
use embassy_rp::gpio::{AnyPin, Drive, Input, Pull};
use embassy_rp::peripherals::{SPI0, SPI1, USB};
use embassy_rp::spi::{Async, Spi};
use embassy_rp::usb::Driver;
use embassy_time::Timer;
use embedded_alloc::Heap;
use embedded_hal_1::digital::OutputPin;
use embedded_hal_async::spi::SpiBus;
use gpio::{Level, Output};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};
use variegated_board_features::GravityBoardFeatures;
use variegated_board_gravity::GravityR0ABoardFeatures;
use variegated_embassy_ads124s08::{ADS124S08, WaitStrategy};
use variegated_embassy_ads124s08::registers::{Mux, PGAGain, ReferenceInput};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
});

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

#[global_allocator]
static HEAP: Heap = Heap::empty();

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

    let mut board_features = variegated_board_gravity::create_board_features(p);

    let driver = Driver::new(board_features.usb.take().expect("USB must be unused"), Irqs);
    unwrap!(spawner.spawn(logger_task(driver)));

    Timer::after_secs(5).await;

    log::info!("Starting functional test");

    let adc_res = Output::new(board_features.adc_res_pin.take().expect("ADC RES must be unused"), Level::High);
    let adc_start = Output::new(board_features.adc_start_pin.take().expect("ADC START must be unused"), Level::Low);
    let mut spi = board_features.spi_bus.take().expect("SPI bus must be unused");
    let mut ads124s08 = board_features.ads124s08.take().expect("ADS124S08 must be unused");

    let mut excitation_current_2 = Output::new(board_features.excitation2_en_pin.take().expect("Excitation current pin 2 must be unused"), Level::High);

    let mut spi = test_ads124s08(spi, ads124s08).await;
    let mut board_features = test_excitation_current(board_features).await;

    let mut led = Output::new(board_features.led_pin.take().expect("LED pin must be unused"), Level::Low);

    loop {
        led.set_high();
        Timer::after_secs(1).await;

        led.set_low();
        Timer::after_secs(1).await;
    }
}

async fn test_ads124s08<'a>(mut spi: Spi<'a, SPI0, Async>, mut ads124s08: ADS124S08<'_>) -> Spi<'a, SPI0, Async> {
    log::info!("Starting ADS124S08 test");

    let res = ads124s08.read_device_id(&mut spi).await;

    if let Ok(res) = res {
        log::info!("PASS: Successfully read device id");
    } else {
        log::info!("FAIL: Failed to read device id");
        log::info!("---------");
        return spi;
    }

    let res = ads124s08.read_avdd_by_4(&mut spi).await;

    if let Ok(res) = res {
        let avdd = res.internally_referenced_voltage() * 4.0;

        if avdd > 4.0 && avdd < 6.0 {
            log::info!("PASS: Successfully read AVDD by 4");
        } else if avdd > 6.0 {
            log::info!("FAIL: Read AVDD by 4, but value is too high ({}V)", avdd);
        } else {
            log::info!("FAIL: Read AVDD by 4, but value is too low ({}V)", avdd);
        }

        loop {
            let res = ads124s08.measure_differential(&mut spi, Mux::AIN4, Mux::AIN5, ReferenceInput::Refp0Refn0, PGAGain::Gain8).await;
            if let Ok(res) = res {
                //log::info!("PASS: Successfully measured differential");

                let voltage = res.code();

                log::info!("Load cell code: {}", voltage);

                let cal0 = 760;
                let cal100 = 6180;

                let slope = 100.0 / (cal100 - cal0) as f32;

                let weight = slope * (voltage as f32 - cal0 as f32);
                
                log::info!("Weight: {}g", weight);
            } else {
                log::info!("FAIL: Failed to measure differential");

                break;
            }

            Timer::after_secs(1).await;
        }
    } else {
        log::info!("FAIL: Failed to read AVDD by 4");
    }

    log::info!("---------");
    spi
}

async fn test_excitation_current(mut board_features: GravityR0ABoardFeatures) -> GravityR0ABoardFeatures {
    log::info!("Starting excitation current test");

    let mut excitation_current_1 = Output::new(board_features.excitation1_en_pin.take().expect("Excitation current pin 1 must be unused"), Level::Low);
//    let mut excitation_current_2 = Output::new(board_features.excitation2_en_pin.take().expect("Excitation current pin 2 must be unused"), Level::Low);
    let mut excitation_current_3 = Output::new(board_features.excitation3_en_pin.take().expect("Excitation current pin 3 must be unused"), Level::Low);
    let mut excitation_current_4 = Output::new(board_features.excitation4_en_pin.take().expect("Excitation current pin 4 must be unused"), Level::Low);

    excitation_current_1.set_high();
    Timer::after_secs(5).await;

    log::info!("Excitation current 1 is high");

    excitation_current_1.set_low();
//    excitation_current_2.set_high();

    log::info!("Excitation current 2 is high");

    Timer::after_secs(5).await;

    //excitation_current_2.set_low();
    excitation_current_3.set_high();

    log::info!("Excitation current 3 is high");

    Timer::after_secs(5).await;

    excitation_current_3.set_low();
    excitation_current_4.set_high();

    log::info!("Excitation current 4 is high");

    Timer::after_secs(5).await;

    excitation_current_4.set_low();

    log::info!("Excitation current test complete");

    board_features
}