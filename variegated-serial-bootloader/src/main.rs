#![no_std]
#![no_main]

mod serial_flasher;

use core::cell::RefCell;

use cortex_m_rt::{entry, exception};
#[cfg(feature = "defmt")]
use defmt_rtt as _;
use embassy_boot_rp::*;
use embassy_rp::{gpio, uart};
use embassy_rp::flash::Flash;
use embassy_rp::Peripherals;
use embassy_rp::peripherals::{FLASH, WATCHDOG};
use embassy_rp::uart::BufferedUart;
use embassy_rp::watchdog::Watchdog;
use embassy_sync::blocking_mutex::Mutex;
use embassy_time::{Duration, Timer};
use embedded_storage::nor_flash::NorFlash;
use variegated_board_features::BootloaderFeatures;

#[entry]
fn main() -> ! {
    let p = embassy_rp::init(Default::default());

    let bootloader_board_features = variegated_board_open_lcc_r2a::create_bootloader_features(p);

    let serial_boot_input = gpio::Input::new(bootloader_board_features.bootloader_trigger_pin, gpio::Pull::Down);

    if serial_boot_input.is_high() {
        boot_flasher(bootloader_board_features.bootloader_uart, bootloader_board_features.flash, bootloader_board_features.watchdog);
    } else {
        boot_normally(bootloader_board_features.flash, bootloader_board_features.watchdog);
    }
}

fn boot_flasher<UartT>(uart: BufferedUart<UartT>, flash: FLASH, watchdog: WATCHDOG) -> ! where UartT: uart::Instance {
    // Override bootloader watchdog
    let mut watchdog = Watchdog::new(watchdog);
    watchdog.start(Duration::from_secs(8));

    let flash = Flash::<_, _, {variegated_board_open_lcc_r2a::FLASH_SIZE}>::new_blocking(flash);
    let flash = Mutex::new(RefCell::new(flash));

    let config = FirmwareUpdaterConfig::from_linkerfile_blocking(&flash);
    let mut aligned = AlignedBuffer([0; 1]);
    let mut updater = BlockingFirmwareUpdater::new(config, &mut aligned.0);
/*
    watchdog.feed();
    let mut offset = 0;
    let mut buf: AlignedBuffer<4096> = AlignedBuffer([0; 4096]);
    defmt::info!("preparing update");
    let writer = updater
        .prepare_update()
        .map_err(|e| defmt::warn!("E: {:?}", defmt::Debug2Format(&e)))
        .unwrap();
    defmt::info!("writer created, starting write");
    for chunk in APP_B.chunks(4096) {
        buf.0[..chunk.len()].copy_from_slice(chunk);
        defmt::info!("writing block at offset {}", offset);
        writer.write(offset, &buf.0[..]).unwrap();
        offset += chunk.len() as u32;
    }
    watchdog.feed();
    defmt::info!("firmware written, marking update");
    updater.mark_updated().unwrap();
    Timer::after_secs(2).await;
    defmt::info!("update marked, resetting");*/
    cortex_m::peripheral::SCB::sys_reset();
}

fn boot_normally(flash: FLASH, watchdog: WATCHDOG) -> ! {
    // Uncomment this if you are debugging the bootloader with debugger/RTT attached,
    // as it prevents a hard fault when accessing flash 'too early' after boot.
    /*
    for i in 0..10000000 {
        cortex_m::asm::nop();
    }
    */

    let flash = WatchdogFlash::<{variegated_board_open_lcc_r2a::FLASH_SIZE}>::start(flash, watchdog, Duration::from_secs(8));
    let flash = Mutex::new(RefCell::new(flash));

    let config = BootLoaderConfig::from_linkerfile_blocking(&flash);
    let active_offset = config.active.offset();
    let bl: BootLoader = BootLoader::prepare(config);

    unsafe { bl.load(embassy_rp::flash::FLASH_BASE as u32 + active_offset) }
}

#[no_mangle]
#[cfg_attr(target_os = "none", link_section = ".HardFault.user")]
unsafe extern "C" fn HardFault() {
    cortex_m::peripheral::SCB::sys_reset();
}

#[exception]
unsafe fn DefaultHandler(_: i16) -> ! {
    const SCB_ICSR: *const u32 = 0xE000_ED04 as *const u32;
    let irqn = core::ptr::read_volatile(SCB_ICSR) as u8 as i16 - 16;

    panic!("DefaultHandler #{:?}", irqn);
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    cortex_m::asm::udf();
}