#![no_std]
#![no_main]

extern crate alloc;

use alloc::format;
use defmt::*;
use embassy_executor::{Executor, Spawner};
use embassy_rp::{bind_interrupts, gpio, i2c, usb};
use embassy_rp::gpio::{AnyPin, Drive, Input, Pull};
use embassy_rp::i2c::{Blocking, I2c};
use embassy_rp::peripherals::{I2C0, SPI0, SPI1, USB};
use embassy_rp::spi::{Async, Spi};
use embassy_rp::usb::Driver;
use embassy_time::{Instant, Timer};
use embedded_alloc::Heap;
use embedded_hal_1::digital::OutputPin;
use embedded_hal_async::spi::SpiBus;
use gpio::{Level, Output};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};
use variegated_board_features::GravityBoardFeatures;
use variegated_board_gravity::GravityR0ABoardFeatures;
use variegated_embassy_ads124s08::{ADS124S08, ADS124S08Error, Code, WaitStrategy};
use variegated_embassy_ads124s08::registers::{DataRate, Filter, IDACMagnitude, IDACMux, InputMultiplexerRegister, Mode, Mux, PGAGain, ReferenceInput};
use movavg;

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use embedded_graphics::mono_font::ascii::FONT_9X18_BOLD;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
use ssd1306::mode::BufferedGraphicsMode;
use variegated_embassy_ads124s08::registers::PGAGain::Gain32;

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

    let i2c = board_features.blocking_i2c_bus.take().expect("I2C must be unused");

    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate180)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let adc_res = Output::new(board_features.adc_res_pin.take().expect("ADC RES must be unused"), Level::High);
    let adc_start = Output::new(board_features.adc_start_pin.take().expect("ADC START must be unused"), Level::Low);
    let mut spi = board_features.spi_bus.take().expect("SPI bus must be unused");
    let mut ads124s08 = board_features.ads124s08.take().expect("ADS124S08 must be unused");

    let mut excitation_current_1 = Output::new(board_features.excitation1_en_pin.take().expect("Excitation current pin 1 must be unused"), Level::Low);
    let mut excitation_current_2 = Output::new(board_features.excitation2_en_pin.take().expect("Excitation current pin 2 must be unused"), Level::Low);
    let mut excitation_current_3 = Output::new(board_features.excitation3_en_pin.take().expect("Excitation current pin 3 must be unused"), Level::Low);
    let mut excitation_current_4 = Output::new(board_features.excitation4_en_pin.take().expect("Excitation current pin 4 must be unused"), Level::Low);

    let mut tare = Input::new(board_features.uart_rx_pin.take().expect("Tare pin must be unused"), Pull::Up);

    let mut spi = test_ads124s08(spi, ads124s08, excitation_current_1, excitation_current_2, excitation_current_3, excitation_current_4, tare, display).await;
    //let mut board_features = test_excitation_current(board_features).await;

    //test_nau7802(board_features.async_i2c_bus.take().expect("I2C must be unused")).await;

    let mut led = Output::new(board_features.led_pin.take().expect("LED pin must be unused"), Level::Low);

    loop {
        led.set_high();
        Timer::after_secs(1).await;

        led.set_low();
        Timer::after_secs(1).await;
    }
}

async fn test_nau7802<'a>(mut i2c: I2c<'a, I2C0, i2c::Async>) {
    let nau7802 = Nau7802::new();
    nau7802.init(
        &mut i2c,
        variegated_embassy_nau7802::Ldo::L3v3,
        variegated_embassy_nau7802::Gain::G128,
        variegated_embassy_nau7802::SamplesPerSecond::SPS80,
    ).await.unwrap();

    loop {
        let res = nau7802.read(&mut i2c).await;
        if let Ok(res) = res {
            let zero = -41942;
            let hundered = 7557;

            let slope = 100.0 / (hundered - zero) as f32;
            let weight = slope * (res as f32 - zero as f32);

            log::info!("Voltage: {}, Weight: {:.1}", res, weight);
        } else {
            log::info!("Failed to read NAU7802, err. {:?}", res);
        }

        Timer::after_millis(30).await;
    }
}

async fn test_ads124s08<'a>(
    mut spi: Spi<'a, SPI0, Async>,
    mut ads124s08: ADS124S08<'a>,
    mut e1: Output<'a, AnyPin>,
    mut e2: Output<'a, AnyPin>,
    mut e3: Output<'a, AnyPin>,
    mut e4: Output<'a, AnyPin>,
    mut tare: Input<'a, AnyPin>,
    mut display: Ssd1306<I2CInterface<I2c<'a, I2C0, Blocking>>, DisplaySize128x64, BufferedGraphicsMode<DisplaySize128x64>>
) -> Spi<'a, SPI0, Async> {
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

        let delay_us = 100;

        let mut tare_val = 18335.0;

        ads124s08.begin_transaction().await;
        let mut reg = ads124s08.read_datarate_reg(&mut spi).await.expect("Failed to read datarate reg");
        reg.rate = DataRate::SPS60;
        reg.filter = Filter::SINC3;
        reg.g_chop = true;
        reg.mode = Mode::Continuous;
        ads124s08.write_datarate_reg(&mut spi, reg).await.expect("Failed to write datarate reg");

        let mut pga = ads124s08.read_pga_reg(&mut spi).await.expect("Failed to read PGA reg");
        pga.enable = true;
        pga.gain = Gain32;
        ads124s08.write_pga_reg(&mut spi, pga).await.expect("Failed to write PGA reg");

        let mut idacmag = ads124s08.read_idacmag_reg(&mut spi).await.expect("Failed to read IDACMAG reg");
        idacmag.imag = IDACMagnitude::Off;
        ads124s08.write_idacmag_reg(&mut spi, idacmag).await.expect("Failed to write IDACMAG reg");

        let mut idacmux = ads124s08.read_idacmux_reg(&mut spi).await.expect("Failed to read IDACMUX reg");
        idacmux.i2mux = IDACMux::Disconnected;
        idacmux.i1mux = IDACMux::Disconnected;
        ads124s08.write_idacmux_reg(&mut spi, idacmux).await.expect("Failed to write IDACMUX reg");

        let mut refctrl = ads124s08.read_refctrl_reg(&mut spi).await.expect("Failed to read REFCTRL reg");
        refctrl.refsel = ReferenceInput::Refp0Refn0;
        ads124s08.write_refctrl_reg(&mut spi, refctrl).await.expect("Failed to write REFCTRL reg");

        ads124s08.end_transaction().await;

        e1.set_high();
        e2.set_high();
        e3.set_high();
        e4.set_high();

        let mut ma1: MovAvg<i32, i64, 5> = MovAvg::new();
        let mut ma2: MovAvg<i32, i64, 5> = MovAvg::new();
        let mut ma3: MovAvg<i32, i64, 5> = MovAvg::new();
        let mut ma4: MovAvg<i32, i64, 5> = MovAvg::new();

        let mut temp_ma: MovAvg<i32, i64, 5> = MovAvg::new();

        loop {
            ads124s08.begin_transaction().await;
            ads124s08.start_conversion(&mut spi).await.expect("Failed to start conversion");
//            let res1 = read_fast(&mut spi, &mut ads124s08, Mux::AIN1, Mux::AIN2, &mut e1).await;
//            ma1.feed(res1.expect("Err1").code());
            let res2 = read_fast(&mut spi, &mut ads124s08, Mux::AIN4, Mux::AIN5, &mut e2).await;
            ma2.feed(res2.expect("Err2").code());
//            let res3 = read_fast(&mut spi, &mut ads124s08, Mux::AIN6, Mux::AIN7, &mut e3).await;
//            ma3.feed(res3.expect("Err3").code());
            let res4 = read_fast(&mut spi, &mut ads124s08, Mux::AIN10, Mux::AIN11, &mut e4).await;
            ma4.feed(res4.expect("Err4").code());
            ads124s08.stop_conversion(&mut spi).await;
            ads124s08.end_transaction().await;

            let temp_code = ads124s08.read_temperature(&mut spi).await.expect("Failed to read temperature").code();
            temp_ma.feed(temp_code);

            let code_sum = ma2.get() as i64 + ma4.get() as i64; //+ ma2.get() as i64 + ma3.get() as i64 + ma4.get() as i64;

            //let compensated_code_sum = compensate_temperature(code_sum, temp_ma.get() as i64);
            let compensated_code_sum = code_sum as f64;

            if (tare.is_low()) {
                tare_val = compensated_code_sum;
                log::info!("Tare value set to {}", tare_val);
            }

            let cal0 = 18418;
            let cal100 = 30783;

            let slope = 100.0 / (cal100 - cal0) as f32;

            let weight = slope * (compensated_code_sum as f32 - tare_val as f32);

                        let text_style = MonoTextStyleBuilder::new()
                            .font(&FONT_9X18_BOLD)
                            .text_color(BinaryColor::On)
                            .build();

                        let str = format!("{:.1} g", weight);

                        display.clear(BinaryColor::Off).unwrap();

                        Text::with_baseline(&str, Point::zero(), text_style, Baseline::Top)
                            .draw(&mut display)
                            .unwrap();

                        display.flush().unwrap();
            
            log::info!("Weight: {:.1} g, CodeSum: {}, Temp: {}", weight, code_sum, temp_code);
        }

/*        loop {
            let mut avg_code_sum: i64 = 0;
            let mut avg_temp_code: i64 = 0;
            for i in 0..5 {
                let res1 = read(&mut spi, &mut ads124s08, Mux::AIN1, Mux::AIN2, &mut e1).await;
                let res2 = read(&mut spi, &mut ads124s08, Mux::AIN4, Mux::AIN5, &mut e2).await;
                let res3 = read(&mut spi, &mut ads124s08, Mux::AIN6, Mux::AIN7, &mut e3).await;
                let res4 = read(&mut spi, &mut ads124s08, Mux::AIN10, Mux::AIN11, &mut e4).await;
                let code_sum = res1.expect("Err1").code() as i64 + res2.expect("Err2").code() as i64 + res3.expect("Err3").code()  as i64 + res4.expect("Err4").code() as i64;
                avg_code_sum += code_sum;

                //let temp_code = ads124s08.read_temperature(&mut spi).await.expect("Failed to read temperature").code();
                //avg_temp_code += temp_code as i64;
            }

            let code_sum = avg_code_sum / 5;
            let temp_code = avg_temp_code / 5;

            //let compensated_code_sum = compensate_temperature(code_sum, temp_code);
            let compensated_code_sum = code_sum as f64;

            if (tare.is_low()) {
                tare_val = compensated_code_sum;
                log::info!("Tare value set to {}", tare_val);
            }

            let cal0 = -17424;
            let cal100 = -4936;

            let slope = 100.0 / (cal100 - cal0) as f32;

            let weight = slope * (compensated_code_sum as f32 - tare_val as f32);
            //

/*            let text_style = MonoTextStyleBuilder::new()
                .font(&FONT_9X18_BOLD)
                .text_color(BinaryColor::On)
                .build();

            let str = format!("{:.1} g", weight);

            display.clear(BinaryColor::Off).unwrap();

            Text::with_baseline(&str, Point::zero(), text_style, Baseline::Top)
                .draw(&mut display)
                .unwrap();

            display.flush().unwrap();*/


//            Timer::after_millis(50).await;
        }*/
    } else {
        log::info!("FAIL: Failed to read AVDD by 4");
    }

    log::info!("---------");
    spi
}

async fn read<'a>(
    spi: &mut Spi<'a, SPI0, Async>,
    ads124s08: &mut ADS124S08<'a>,
    mux1: Mux,
    mux2: Mux,
    excitation_current: &mut Output<'a, AnyPin>,
) -> Result<Code, ADS124S08Error> {
//    excitation_current.set_high();
//    Timer::after_micros(100).await;
    let res = ads124s08.measure_differential(spi, mux1, mux2, ReferenceInput::Refp0Refn0, PGAGain::Gain32).await;
//    excitation_current.set_low();

    res
}

async fn read_fast<'a>(
    spi: &mut Spi<'a, SPI0, Async>,
    ads124s08: &mut ADS124S08<'a>,
    mux1: Mux,
    mux2: Mux,
    excitation_current: &mut Output<'a, AnyPin>,
) -> Result<Code, ADS124S08Error> {
    excitation_current.set_high();
    busy_wait_us(100);
    let mut inpmux = InputMultiplexerRegister::default();
    inpmux.p = mux1;
    inpmux.n = mux2;
    ads124s08.write_inpmux_reg(spi, inpmux).await.expect("Failed to write INPMUX reg");
    ads124s08.wait_for_drdy(spi).await?;
    let res = ads124s08.read_data(spi).await;
    excitation_current.set_low();

    res
}

fn handle_res(res: Result<Code, ADS124S08Error>, num: u8, _offset: i32) {
    if let Ok(res) = res {
        let voltage = res.externally_referenced_voltage(0.0, 3.3);

        log::info!("Load cell code {}: {}", num, voltage);

        let cal0 = 760;
        let cal100 = 6180;

        let slope = 100.0 / (cal100 - cal0) as f32;

        let weight = slope * (voltage as f32 - cal0 as f32);

        //log::info!("Weight {}: {}g", num, weight);
    } else {
        log::info!("FAIL: Failed to measure differential port {}", num);
    }
}

fn compensate_temperature(code_sum: i64, temperature: i64) -> f64 {
    let slope = -0.6842;
    let intercept = 300647.1936;

    let raw_adc = code_sum as f64;
    let compensated_adc = raw_adc - (slope * temperature as f64 + intercept);

    compensated_adc
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

use cortex_m::asm;
use movavg::MovAvg;
use variegated_embassy_nau7802::Nau7802;

fn busy_wait_us(us: u32) {
    // This constant should be calibrated for your specific MCU and clock speed
    const CYCLES_PER_US: u32 = 133;
    let cycles = us * CYCLES_PER_US;
    asm::delay(cycles);
}