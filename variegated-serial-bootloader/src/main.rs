#![no_std]
#![no_main]

pub mod serial_flasher;

use core::cell::RefCell;
use assign_resources::assign_resources;
use cfg_if::cfg_if;

use cortex_m_rt::{entry, exception};
use defmt_rtt as _;
use embassy_boot_rp::*;
use embassy_rp::{bind_interrupts, gpio, peripherals, uart};
use embassy_rp::flash::Flash;
use embassy_rp::gpio::{Level, Output, Pin};
use embassy_rp::interrupt::typelevel::Interrupt;
use embassy_rp::peripherals::*;
use embassy_rp::uart::{Async, Blocking, BufferedUart, Uart};
use embassy_rp::watchdog::Watchdog;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Duration, Instant, Timer};
use embedded_storage::nor_flash::NorFlash;
use log::info;
use variegated_board_cfg::board_cfg;
use variegated_board_features::BootloaderFeatures;
use crate::serial_flasher::main_loop;
use {defmt_rtt as _, panic_probe as _};

static mut rx_buf: [u8; 512] = [0u8; 512];
static mut tx_buf: [u8; 512] = [0u8; 512];

#[board_cfg("led-resources")]
struct LedResources {
    led1: impl Pin,
    led2: impl Pin,
}

#[board_cfg("bootloader-resources")]
struct BootloaderResources {
    trigger_pin: impl Pin,
    uart: (),
    uart_rx: impl Pin,
    uart_tx: impl Pin,
    flash: (),
}

bind_interrupts!(struct Irqs {
    UART0_IRQ => uart::BufferedInterruptHandler<BootloaderResourcesUart>;
});

#[entry]
fn main() -> ! {
    for i in 0..10000 {
        cortex_m::asm::nop();
    }

    let p = embassy_rp::init(Default::default());

    let led_resources = led_resources!(p);
    let bootloader_resources = bootloader_resources!(p);

    //let bootloader_board_features = variegated_board_open_lcc_r2a::create_bootloader_features(p);


    //let resources = split_resources!(p);

    let start = Instant::now();
    let delay = Duration::from_millis(1000);
    while Instant::now() - start < delay {}
    info!("Getting started");

    let foo: Option<UART0> = None;

    let status_output = Output::new(led_resources.led1, Level::High);

    let serial_boot_input = gpio::Input::new(bootloader_resources.trigger_pin, gpio::Pull::Up);

    let mut config = uart::Config::default();
    config.baudrate = 9600;
    let bootloader_uart = unsafe { Uart::new_blocking(bootloader_resources.uart, bootloader_resources.uart_tx, bootloader_resources.uart_rx, config).into_buffered(Irqs, &mut tx_buf, &mut rx_buf) };

/*
    if serial_boot_input.is_low() {
        info!("Serial boot is low, booting into serial boot loader");
        boot_flasher::<_, {variegated_board_open_lcc_r2a::FLASH_SIZE}>(bootloader_board_features.bootloader_uart, bootloader_board_features.flash, bootloader_board_features.watchdog);
    } else {
        info!("Booting normally");
        boot_normally(bootloader_board_features.flash, bootloader_board_features.watchdog, status_output);
    }*/

    loop {}
}

fn boot_flasher<UartT, const FLASH_SIZE: usize>(uart: BufferedUart<UartT>, flash: FLASH, watchdog: WATCHDOG) -> ! where UartT: uart::Instance {
    main_loop::<_, FLASH_SIZE>(flash, watchdog, uart);

    // Override bootloader watchdog
/*    let mut watchdog = Watchdog::new(watchdog);
    watchdog.start(Duration::from_secs(8));

    let flash = Flash::<_, _, FLASH_SIZE>::new_blocking(flash);
    let flash = Mutex::new(RefCell::new(flash));

    let config = FirmwareUpdaterConfig::from_linkerfile_blocking(&flash);
    let mut aligned = AlignedBuffer([0; 1]);
    let mut updater = BlockingFirmwareUpdater::new(config, &mut aligned.0);
    
    main_loop(flash)*/
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

fn boot_normally(flash: FLASH, watchdog: WATCHDOG, mut status_output: Output) -> ! {
    // Uncomment this if you are debugging the bootloader with debugger/RTT attached,
    // as it prevents a hard fault when accessing flash 'too early' after boot.

    for i in 0..10000 {
        cortex_m::asm::nop();
    }


    let flash = WatchdogFlash::<{variegated_board_open_lcc_r2a::FLASH_SIZE}>::start(flash, watchdog, Duration::from_secs(8));
    let flash: Mutex<NoopRawMutex, _> = Mutex::new(RefCell::new(flash));

    let config = BootLoaderConfig::from_linkerfile_blocking(&flash, &flash, &flash);
    let active_offset = config.active.offset();

    loop {
        status_output.toggle();
        let start = Instant::now();
        let delay = Duration::from_millis(500);
        while Instant::now() - start < delay {}
    }
    let bl: BootLoader = BootLoader::prepare(config);

    unsafe { bl.load(embassy_rp::flash::FLASH_BASE as u32 + active_offset) }
}
/*
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
*/
/*
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    cortex_m::asm::udf();
}*/