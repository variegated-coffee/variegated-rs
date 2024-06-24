#![no_std]
#![no_main]

use cortex_m::prelude::_embedded_hal_blocking_spi_Write;
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::{bind_interrupts, gpio, usb};
use embassy_rp::gpio::{AnyPin, Drive, Input, Pull};
use embassy_rp::peripherals::{SPI1, USB};
use embassy_rp::spi::{Async, Spi};
use embassy_rp::usb::Driver;
use embassy_time::Timer;
use embedded_hal_1::digital::OutputPin;
use embedded_hal_async::spi::SpiBus;
use gpio::{Level, Output};
use {defmt_rtt as _, panic_probe as _};
use variegated_board_apec::{ApecR0DBoardFeatures, ApecR0dShiftRegisterPositions};
use variegated_embassy_ads124s08::{ADS124S08, WaitStrategy};
use variegated_embassy_dual_c595_shift_register::DualC595ShiftRegister;
use variegated_embassy_fdc1004::{Channel, OutputRate};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
});

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut board_features = variegated_board_apec::create_board_features(p);

    let driver = Driver::new(board_features.usb.take().expect("USB must be unused"), Irqs);
    unwrap!(spawner.spawn(logger_task(driver)));

    Timer::after_secs(5).await;

    log("Starting test");

    let flash_cs = Output::new(board_features.settings_flash_cs_pin.take().expect("Flash CS must be unused"), Level::High);
    let sd_cs = Output::new(board_features.sd_cs_pin.take().expect("SD CS must be unused"), Level::High);
    let adc_cs = Output::new(board_features.adc_cs_pin.take().expect("ADC CS must be unused"), Level::High);
    let mut spi = board_features.spi_bus.take().expect("SPI bus must be unused");

    let adc_drdy = Input::new(board_features.adc_drdy_pin.take().expect("DRDY must be unused"), Pull::Up);

    let mut board_features = test_fdc1004(board_features).await;
    let mut spi = test_settings_flash(flash_cs, spi).await;
    let mut spi = test_sd_card(sd_cs, spi).await;
    let mut spi = test_ads124s08(adc_cs, spi, adc_drdy).await;
    let mut board_features = test_c595(board_features).await;
    let mut board_features = test_gpio(board_features).await;

    let mut led = Output::new(board_features.cn14_10_pin.take().expect("CN14_10 must be unused"), Level::Low);

    log("Blinkenlights!");

    loop {
        led.set_high();
        Timer::after_secs(1).await;

        led.set_low();
        Timer::after_secs(1).await;
    }
}

#[repr(u8)]
enum Command {
    PageProgram = 0x02,
    ReadData = 0x03,
    ReadStatusRegister1 = 0x05,
    WriteEnable = 0x06,
    SectorErase = 0x20,
    UniqueId = 0x4B,
    Block32Erase = 0x52,
    Block64Erase = 0xD8,
    ChipErase = 0xC7,
    EnableReset = 0x66,
    PowerDown = 0xB9,
    ReleasePowerDown = 0xAB,
    Reset = 0x99,
}

async fn test_gpio(mut board_features: ApecR0DBoardFeatures) -> ApecR0DBoardFeatures {
    log("Starting GPIO test");

    let mut cn14_8 = Output::new(board_features.cn14_8_pin.take().expect("CN14_8 must be unused"), Level::Low);
    let mut cn14_6 = Output::new(board_features.cn14_6_pin.take().expect("CN14_6 must be unused"), Level::Low);
    let mut cn14_4 = Output::new(board_features.cn14_4_pin.take().expect("CN14_4 must be unused"), Level::Low);

    let mut cn9_4 = Output::new(board_features.cn9_4_pin.take().expect("CN9_4 must be unused"), Level::Low);
    let mut cn9_6 = Output::new(board_features.cn9_6_pin.take().expect("CN9_6 must be unused"), Level::Low);
    let mut cn9_8 = Output::new(board_features.cn9_8_pin.take().expect("CN9_8 must be unused"), Level::Low);

    let mut cn12 = Output::new(board_features.cn12_pin.take().expect("CN12 must be unused"), Level::Low);
    let mut cn13 = Output::new(board_features.cn13_pin.take().expect("CN13 must be unused"), Level::Low);

    let mut cn14_3 = Output::new(board_features.cn14_3_pin.take().expect("CN14_3 must be unused"), Level::Low);
    let mut cn14_5 = Output::new(board_features.cn14_5_pin.take().expect("CN14_5 must be unused"), Level::Low);
    let mut cn14_7 = Output::new(board_features.cn14_7_pin.take().expect("CN14_7 must be unused"), Level::Low);
    let mut cn14_9 = Output::new(board_features.cn14_9_pin.take().expect("CN14_9 must be unused"), Level::Low);

    log("CN14_8");
    cn14_8.set_high();
    Timer::after_millis(500).await;
    cn14_8.set_low();

    log("CN14_6");
    cn14_6.set_high();
    Timer::after_millis(500).await;
    cn14_6.set_low();

    log("CN14_4");
    cn14_4.set_high();
    Timer::after_millis(500).await;
    cn14_4.set_low();

    log("CN9_4");
    cn9_4.set_high();
    Timer::after_millis(500).await;
    cn9_4.set_low();

    log("CN9_6");
    cn9_6.set_high();
    Timer::after_millis(500).await;
    cn9_6.set_low();

    log("CN9_8");
    cn9_8.set_high();
    Timer::after_millis(500).await;
    cn9_8.set_low();

    log("CN12");
    cn12.set_high();
    Timer::after_millis(500).await;
    cn12.set_low();

    log("CN13");
    cn13.set_high();
    Timer::after_millis(500).await;
    cn13.set_low();

    log("CN14_3");
    cn14_3.set_high();
    Timer::after_millis(500).await;
    cn14_3.set_low();

    log("CN14_5");
    cn14_5.set_high();
    Timer::after_millis(500).await;
    cn14_5.set_low();

    log("CN14_7");
    cn14_7.set_high();
    Timer::after_millis(500).await;
    cn14_7.set_low();

    log("CN14_9");
    cn14_9.set_high();
    Timer::after_millis(500).await;
    cn14_9.set_low();

    log("---------");

    board_features
}

async fn test_c595(mut board_features: ApecR0DBoardFeatures) -> ApecR0DBoardFeatures {
    let mut sr = board_features.dual_shift_register.take().expect("Shift register must be set");

    log("C595: CN2_FA10");
    sr.write(ApecR0dShiftRegisterPositions::CN2_FA10.bits()).await;
    Timer::after_millis(500).await;

    log("C595: CN2_FA9");
    sr.write(ApecR0dShiftRegisterPositions::CN2_FA9.bits()).await;
    Timer::after_millis(500).await;

    log("C595: CN2_FA8");
    sr.write(ApecR0dShiftRegisterPositions::CN2_FA8.bits()).await;
    Timer::after_millis(500).await;

    log("C595: CN2_FA7");
    sr.write(ApecR0dShiftRegisterPositions::CN2_FA7.bits()).await;
    Timer::after_millis(500).await;

    log("C595: CN9_3");
    sr.write(ApecR0dShiftRegisterPositions::CN9_3.bits()).await;
    Timer::after_millis(500).await;

    log("C595: CN9_5");
    sr.write(ApecR0dShiftRegisterPositions::CN9_5.bits()).await;
    Timer::after_millis(500).await;

    log("C595: CN9_7");
    sr.write(ApecR0dShiftRegisterPositions::CN9_7.bits()).await;
    Timer::after_millis(500).await;

    log("C595: VOUT4");
    sr.write(ApecR0dShiftRegisterPositions::VOUT4.bits()).await;
    Timer::after_millis(500).await;

    log("C595: VOUT3");
    sr.write(ApecR0dShiftRegisterPositions::VOUT3.bits()).await;
    Timer::after_millis(500).await;

    log("C595: CN10");
    sr.write(ApecR0dShiftRegisterPositions::CN10.bits()).await;
    Timer::after_millis(500).await;

    log("C595: CN11");
    sr.write(ApecR0dShiftRegisterPositions::CN11.bits()).await;
    Timer::after_millis(500).await;

    log("C595: VOUT2");
    sr.write(ApecR0dShiftRegisterPositions::VOUT2.bits()).await;
    Timer::after_millis(500).await;

    log("C595: VOUT1");
    sr.write(ApecR0dShiftRegisterPositions::VOUT1.bits()).await;
    Timer::after_millis(500).await;

    log("C595: CN1_12V");
    sr.write(ApecR0dShiftRegisterPositions::CN1_12V.bits()).await;
    Timer::after_millis(500).await;

    log("C595: CN1_3V3");
    sr.write(ApecR0dShiftRegisterPositions::CN1_3V3.bits()).await;
    Timer::after_millis(500).await;

    log("C595: clear");
    sr.write(0).await;
//    sr.write(2).await;

    board_features.dual_shift_register = Some(sr);

    log("---------");

    return board_features;
}

async fn test_sd_card<'a>(mut cs: Output<'a, AnyPin>, mut spi: Spi<'a, SPI1, Async>) -> Spi<'a, SPI1, Async> {
    log("Starting SD Card test");

    cs.set_high();
    for _ in 0..10 {
        spi.transfer_in_place(&mut [0xFF]).await.unwrap();
    }

    // Set CS low to start communication
    cs.set_low();

    // Send CMD0 to reset the SD card
    let mut buffer = [
        0x40,       // CMD0
        0x00, 0x00, 0x00, 0x00, // Argument
        0x95,       // CRC
        0xFF,       // Ensure space for the response
    ];

    // Send command and receive response
    let res = spi.transfer_in_place(&mut buffer).await;
    cs.set_high();

    if (res.is_err()) {
        log("FAIL: Failed to reset SD card");
        log("---------");
        return spi;
    }

    if (buffer[6] != 0x01) {
        log("FAIL: SD card did not respond with idle state");

        log::info!("Response: {:?}", buffer);
        info!("Response: {}", buffer);

        log("---------");
        return spi;
    }

    log("PASS: SD card reset");
    log("---------");

    spi
}

async fn test_settings_flash<'a>(mut cs: Output<'a, AnyPin>, mut spi: Spi<'a, SPI1, Async>) -> Spi<'a, SPI1, Async> {
    log("Starting Settings Flash test");

    let mut buf: [u8; 13] = [0; 13];
    buf[0] = Command::UniqueId as u8;

    cs.set_low();

    let res = spi.transfer_in_place(&mut buf).await;

    if let Err(e) = res {
        log("FAIL: Failed to read unique id");
    } else {
        log("PASS: Successfully read unique id");
    }

    cs.set_high();

    log("---------");

    spi
}

async fn test_ads124s08<'a>(mut cs: Output<'a, AnyPin>, mut spi: Spi<'a, SPI1, Async>, mut drdy: Input<'a, AnyPin>) -> Spi<'a, SPI1, Async> {
    let mut ads124s08 = ADS124S08::new(
        cs,
        WaitStrategy::UseDrdyPin(drdy)
    );

    log("Starting ADS124S08 test");

    let res = ads124s08.read_device_id(&mut spi).await;

    if let Ok(res) = res {
        log("PASS: Successfully read device id");
    } else {
        log("FAIL: Failed to read device id");
        log("---------");
        return spi;
    }

    let res = ads124s08.read_avdd_by_4(&mut spi).await;

    if let Ok(res) = res {
        let avdd = res.internally_referenced_voltage() * 4.0;

        if avdd > 4.0 && avdd < 6.0 {
            log("PASS: Successfully read AVDD by 4");
        } else if avdd > 6.0 {
            log("FAIL: Read AVDD by 4, but value is too high");
        } else {
            log("FAIL: Read AVDD by 4, but value is too low");
        }
    } else {
        log("FAIL: Failed to read AVDD by 4");
    }

    log("---------");
    spi
}

async fn test_fdc1004(mut board_features: ApecR0DBoardFeatures) -> ApecR0DBoardFeatures {
    log("Starting FDC1004 test");

    let mut fdc1004 = variegated_embassy_fdc1004::FDC1004::new(
        0x50,
        OutputRate::SPS100
    );

    let mut i2c = board_features.i2c_bus.take().expect("I2C bus must be unused");

    log("Reading capacitance from CIN1");

    let res = fdc1004.read_capacitance(&mut i2c, Channel::CIN1).await;
    if let Ok(res) = res {
        log("PASS: Successfully measured capacitance");
    } else {
        log("FAIL: Failed to measure capacitance");
    }

    log("---------");

    board_features
}

fn log(s: &str) {
    info!("{}", s);
    log::info!("{}", s);
}