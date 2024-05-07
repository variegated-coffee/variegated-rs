use core::cell::RefCell;
use cortex_m::prelude::_embedded_hal_serial_Read;
use crc::{CRC_32_BZIP2, NoTable};
use defmt::{info, warn};
use embassy_boot_rp::{AlignedBuffer, BlockingFirmwareUpdater, FirmwareUpdaterConfig};
use embassy_embedded_hal::flash::partition::BlockingPartition;
use embassy_rp::flash::{Blocking, Flash};
use embassy_rp::peripherals::{FLASH, WATCHDOG};
use embassy_rp::uart;
use embassy_rp::uart::BufferedUart;
use embassy_rp::watchdog::Watchdog;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Duration, Timer};
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use serde::{Deserialize, Serialize};
use postcard::from_bytes;
use sha2::Sha256;
use postcard::accumulator::{CobsAccumulator, FeedResult};

type RelativeAddress = u32;
type Length = u32;
type Page = [u8; 256];
type Crc8Checksum = u8;
type Sha256Checksum = [u8; 16];

#[derive(Deserialize)]
enum SerialFlasherCommand {
    PrepareForUpdate,
    WritePage(RelativeAddress, Page, Crc8Checksum),
    CompareChecksum(Length, Sha256Checksum),
    MarkUpdated
}

#[derive(Serialize)]
enum SerialFlasherResponse {
    Ack,
    Nack,
}

fn main_loop<UartT: uart::Instance>(flash: FLASH, watchdog: WATCHDOG, mut uart: BufferedUart<UartT>) -> ! {
    // Override bootloader watchdog
    let mut watchdog = Watchdog::new(watchdog);
    watchdog.start(Duration::from_secs(8));

    let flash = Flash::<_, _, { open_lcc_board_r2a::FLASH_SIZE }>::new_blocking(flash);
    let flash = Mutex::new(RefCell::new(flash));

    let config = FirmwareUpdaterConfig::from_linkerfile_blocking(&flash);
    let mut aligned = AlignedBuffer([0; 1]);
    let mut updater = BlockingFirmwareUpdater::new(config, &mut aligned.0);

    watchdog.feed();

    loop {
        loop {
            let command: SerialFlasherCommand = read_command(&mut uart);

            match command {
                SerialFlasherCommand::PrepareForUpdate => {
                    break;
                }
                _ => {}
            }
        }

        let mut dfu: &mut BlockingPartition<NoopRawMutex, Flash<FLASH, Blocking, 2097152>> = updater.prepare_update().unwrap();

        loop {
            let command: SerialFlasherCommand = read_command(&mut uart);

            match command {
                SerialFlasherCommand::WritePage(addr, page, checksum) => {
                    dfu.write(addr, &page);
                },
                SerialFlasherCommand::CompareChecksum(len, checksum) => {
                    let mut buf = [0; 255];
                    let mut out = [0; 16];
                    let hash = updater.hash::<Sha256>(len, &mut buf, &mut out);
                },
                SerialFlasherCommand::MarkUpdated => {
                    break;
                }
                _ => {}
            }

            watchdog.feed();
        }

        watchdog.feed();

        info!("firmware written, marking update");
        updater.mark_updated().unwrap();
        break;
    }

    info!("update marked, resetting");
    cortex_m::peripheral::SCB::sys_reset();
}

fn read_command<UartT: uart::Instance>(uart: &mut BufferedUart<UartT>) -> SerialFlasherCommand {
    let mut accumulator = CobsAccumulator::new();

    loop {
        let byte = uart.read().unwrap();
        let array = [byte];
        match accumulator.feed(&array) {
            FeedResult::Consumed => {},
            FeedResult::OverFull(d) => {},
            FeedResult::DeserError(d) => {},
            FeedResult::Success {
                data: d,
                remaining: r,
            } => {
                let command: SerialFlasherCommand = from_bytes(&d).unwrap();
                return command;
            }
        }
    }
}

fn write_response<UartT: uart::Instance>(uart: &mut BufferedUart<UartT>, response: SerialFlasherResponse) {
    let mut buf = [0; 255];
    let len = postcard::to_slice(&response, &mut buf).unwrap().len();
    uart.write(&buf[..len]).unwrap();
}