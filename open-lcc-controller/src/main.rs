#![no_std]
#![no_main]

use core::cell::RefCell;
use defmt::*;
use embassy_boot_rp::{AlignedBuffer, BlockingFirmwareUpdater, FirmwareUpdaterConfig};
use embedded_storage::nor_flash::NorFlash;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::flash::Flash;
use embassy_rp::peripherals::{PWM_CH4, USB};
use embassy_rp::pwm::Pwm;
use embassy_rp::usb::{Driver, self};
use embassy_rp::watchdog::Watchdog;
use embassy_sync::blocking_mutex::Mutex;
use embassy_time::{Duration, Timer};
use open_lcc_board_r2a;
use log::log;
use {defmt_rtt as _, panic_probe as _};

static APP_B: &[u8] = &[0, 1, 2, 3];
const FLASH_SIZE: usize = 2 * 1024 * 1024;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
});

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

#[embassy_executor::main]
async fn main(_s: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Override bootloader watchdog
    let mut watchdog = Watchdog::new(p.WATCHDOG);
    watchdog.start(Duration::from_secs(8));

    let flash = Flash::<_, _, FLASH_SIZE>::new_blocking(p.FLASH);
    let flash = Mutex::new(RefCell::new(flash));

    let config = FirmwareUpdaterConfig::from_linkerfile_blocking(&flash);
    let mut aligned = AlignedBuffer([0; 1]);
    let mut updater = BlockingFirmwareUpdater::new(config, &mut aligned.0);

    Timer::after_secs(5).await;
    watchdog.feed();
    let mut offset = 0;
    let mut buf: AlignedBuffer<4096> = AlignedBuffer([0; 4096]);
    info!("preparing update");
    let writer = updater
        .prepare_update()
        .map_err(|e| warn!("E: {:?}", defmt::Debug2Format(&e)))
        .unwrap();
    info!("writer created, starting write");
    for chunk in APP_B.chunks(4096) {
        buf.0[..chunk.len()].copy_from_slice(chunk);
        info!("writing block at offset {}", offset);
        writer.write(offset, &buf.0[..]).unwrap();
        offset += chunk.len() as u32;
    }
    watchdog.feed();
    info!("firmware written, marking update");
    updater.mark_updated().unwrap();
    Timer::after_secs(2).await;
    info!("update marked, resetting");
    cortex_m::peripheral::SCB::sys_reset();
}

/*

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let board = open_lcc_board_r2a::create_board_features(embassy_rp::init(Default::default()));

    unwrap!(spawner.spawn(logger_task(Driver::new(board.usb, Irqs))));
    if let Some(led_pwm) = board.led_pwm {
        unwrap!(spawner.spawn(glow_led(led_pwm)));
    }

    loop {
        Timer::after_secs(1).await;
    }
}

#[embassy_executor::task]
async fn glow_led(mut led: Pwm<'static, PWM_CH4>) {
    let mut counter = 0;
    loop {
        counter += 1;
        led.counter();
        Timer::after_millis(5).await;
    }
}

*/