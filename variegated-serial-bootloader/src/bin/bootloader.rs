#![no_std]
#![no_main]

use core::cell::RefCell;
use cortex_m::prelude::_embedded_hal_serial_Read;
use cortex_m::register;
use defmt::*;
use embassy_boot::{AlignedBuffer, BlockingFirmwareUpdater, FirmwareUpdaterConfig};
use embassy_executor::Spawner;
use embassy_rp::{bind_interrupts, gpio, Peripheral, spi, usb};
use embassy_rp::flash::Flash;
use embassy_rp::gpio::Pin;
use embassy_rp::peripherals::{PIN_28, UART0, USB};
use embassy_rp::spi::{Config, Phase, Polarity};
use embassy_rp::uart::{Blocking, BufferedInterruptHandler, Uart};
use embassy_rp::usb::Driver;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_time::Timer;
use embedded_io::Read;
use embedded_storage::nor_flash::NorFlash;
use gpio::{Level, Output};
use postcard::accumulator::{CobsAccumulator, FeedResult};
use serde::{Deserialize, Serialize};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
    UART0_IRQ => BufferedInterruptHandler<UART0>;
});

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

#[derive(Serialize, Deserialize, Debug, Format)]
struct Point {
    x: i32,
    y: i32,
}

pub const FLASH_SIZE: usize = 2 * 1024 * 1024;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let driver = Driver::new(p.USB, Irqs);
    spawner.spawn(logger_task(driver)).unwrap();

    Timer::after_secs(3).await;

    info!("USB inited");

    let uart_config = embassy_rp::uart::Config::default();
    let (mut uart_tx, mut uart_rx) = Uart::new_blocking(p.UART0, p.PIN_0, p.PIN_1, uart_config).split();

    let mut rx_buf = [0u8; 32];
    let mut buffered = uart_rx.into_buffered(Irqs, &mut rx_buf);

    let mut led = Output::new(p.PIN_17.degrade(), Level::Low);

    info!("Initing stuff");

    led.toggle();

    Timer::after_secs(1).await;
    led.toggle();
    
/*
    let flash = Flash::<_, _, FLASH_SIZE>::new_blocking(p.FLASH);
    let flash = Mutex::<NoopRawMutex, _>::new(RefCell::new(flash));

    let config = FirmwareUpdaterConfig::from_linkerfile_blocking(&flash);
    let mut aligned = AlignedBuffer([0; 1]);
    let mut updater = BlockingFirmwareUpdater::new(config, &mut aligned.0);

/*    info!("Starting update");

    let dfu = updater.prepare_update().expect("Error preparing for update");
    dfu.write(0x0, &[0; 4096]).expect("Error writing to flash");*/

    let mut raw_buf = [0u8; 32];
    let mut cobs_buf: CobsAccumulator<256> = CobsAccumulator::new();

    loop {
        info!("Outer loop");

        while let Ok(ct) = buffered.blocking_read(&mut raw_buf) {
            info!("Middle loop");
            // Finished reading input
            if ct == 0 {
                break;
            }

            let buf = &raw_buf[..ct];
            let mut window = &buf[..];

            'cobs: while !window.is_empty() {
                info!("Inner loop");
                window = match cobs_buf.feed::<Point>(&window) {
                    FeedResult::Consumed => break 'cobs,
                    FeedResult::OverFull(new_wind) => new_wind,
                    FeedResult::DeserError(new_wind) => new_wind,
                    FeedResult::Success { data, remaining } => {
                        // Do something with `data: MyData` here.

                        info!("Data received, {:?}, remaining: {:?}", data, remaining);

                        remaining
                    }
                };
            }
        }
    }*/
/*
    loop {
        info!("LED on");
        led.set_high();
        Timer::after_millis(1000).await;
        info!("LED off");
        led.set_low();
        Timer::after_millis(1000).await;

        uart_tx.blocking_write("Hello, world!\r\n".as_bytes()).unwrap();
    }*/
}