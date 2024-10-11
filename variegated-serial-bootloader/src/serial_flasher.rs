use core::cell::RefCell;
use cortex_m::prelude::_embedded_hal_serial_Read;
use defmt::{Format, Formatter, info, warn};
use embassy_boot::FirmwareUpdater;
use embassy_boot_rp::{AlignedBuffer, BlockingFirmwareUpdater, FirmwareUpdaterConfig};
use embassy_futures::block_on;
use embassy_rp::flash::{Blocking, Flash};
use embassy_rp::peripherals::{FLASH, WATCHDOG};
use embassy_rp::uart;
use embassy_rp::uart::{Async, BufferedUart, Uart};
use embassy_rp::watchdog::Watchdog;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Duration, Timer};
use embedded_io::Write;
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use serde::{Deserialize, Serialize};
use postcard::from_bytes;
use sha2::Sha256;
use postcard::accumulator::{CobsAccumulator, FeedResult};
use serde_big_array::Array;
//use crate::panic;

type RelativeAddress = usize;
type Length = u32;
type Page = Array<u8, 256>;
type Crc8Checksum = u8;
type Sha256Checksum = [u8; 16];

const CRC8: crc::Crc<u8> = crc::Crc::<u8>::new(&crc::CRC_8_SMBUS);

#[derive(Deserialize)]
enum SerialFlasherCommand {
    Hello,
    PrepareForUpdate,
    WritePage(RelativeAddress, Page, Crc8Checksum),
    FinishedWriting,
    CompareChecksum(Length, Sha256Checksum),
    MarkUpdated
}

impl Format for SerialFlasherCommand {
    fn format(&self, f: defmt::Formatter) {
        match self {
            SerialFlasherCommand::Hello => defmt::write!(f, "Hello"),
            SerialFlasherCommand::PrepareForUpdate => defmt::write!(f, "PrepareForUpdate"),
            SerialFlasherCommand::WritePage(_, _, _) => defmt::write!(f, "WritePage"),
            SerialFlasherCommand::FinishedWriting => defmt::write!(f, "WritePage"),
            SerialFlasherCommand::CompareChecksum(_, _) => defmt::write!(f, "CompareChecksum"),
            SerialFlasherCommand::MarkUpdated => defmt::write!(f, "MarkUpdated"),
        }
    }
}

#[derive(Serialize)]
enum SerialFlasherResponse {
    Ack,
    Nack,
}

#[derive(PartialEq)]
enum SerialFlasherState {
    Unprepared,
    Updating,
    Done
}

#[derive(Debug, Format)]
pub enum SerialFlasherError {
    SerializationError,
    UartWriteError,
    NotPreparedForUpdate,
    AlreadyPreparedForUpdate,
    PageCrcMismatch,
    FlashWriteError,
    ChecksumMismatch
}

struct SerialFlasher<'a, UartT: uart::Instance, DFU: NorFlash, STATE: NorFlash> {
    uart: UartWrapper<'a, UartT>,
    firmware_updater: Option<BlockingFirmwareUpdater<'a, DFU, STATE>>,
    watchdog: Watchdog,
    state: SerialFlasherState,
}

struct UartWrapper<'a, UartT: uart::Instance> {
    uart: BufferedUart<'a, UartT>
}

impl<'a, UartT: uart::Instance> UartWrapper<'a, UartT> {
    pub fn write_response(&mut self, response: SerialFlasherResponse) -> Result<(), SerialFlasherError> {
        let mut buf = [0; 255];
        let serialized_response = postcard::to_slice_cobs(&response, &mut buf).map_err(|_| SerialFlasherError::SerializationError)?;
        let size = self.uart.blocking_write(&serialized_response).map_err(|_| SerialFlasherError::UartWriteError)?;
        self.uart.blocking_flush().map_err(|_| SerialFlasherError::UartWriteError)?;
        Ok(())
    }

    pub fn read_command(&mut self) -> SerialFlasherCommand {
        let mut accumulator: CobsAccumulator<640> = CobsAccumulator::new();

        let mut buf: [u8; 16] = [0; 16];

        loop {
            let res = self.uart.blocking_read(&mut buf);

            if let Ok(len) = res {
                let slice = &buf[..len];
                //info!("Read slice {:?}", slice);
                match accumulator.feed::<SerialFlasherCommand>(slice) {
                    FeedResult::Consumed => {
                        info!("Consumed");
                    },
                    FeedResult::OverFull(d) => {
                        info!("Overfull");
                    },
                    FeedResult::DeserError(d) => {
                        info!("DeserError");
                    },
                    FeedResult::Success {
                        data: d,
                        remaining: r,
                    } => {
                        return d;
                    }
                }
            } else {
                info!("Read error");
            }
        }
    }
}

impl<'a, UartT: uart::Instance, DFU: NorFlash, STATE: NorFlash> SerialFlasher<'a, UartT, DFU, STATE> {
    pub fn new(updater: BlockingFirmwareUpdater<'a, DFU, STATE>, uart: BufferedUart<'a, UartT>, watchdog: Watchdog) -> SerialFlasher<'a, UartT, DFU, STATE> {
        SerialFlasher {
            uart: UartWrapper { uart },
            firmware_updater: Some(updater),
            watchdog,
            state: SerialFlasherState::Unprepared
        }
    }

    fn unprepared_loop(uart_wrapper: &mut UartWrapper<UartT>) {
        info!("Entered unprepared loop");
        loop {
            let command = uart_wrapper.read_command();
            info!("Handling command {:?}", command);

            match command {
                SerialFlasherCommand::Hello => uart_wrapper.write_response(SerialFlasherResponse::Ack).unwrap(),
                SerialFlasherCommand::PrepareForUpdate => {
                    // Delay the Ack here until after we've prepared the update in writing loop
                    break;
                },
                _ => uart_wrapper.write_response(SerialFlasherResponse::Nack).unwrap(),
            }
        }
    }

    fn writing_loop(uart_wrapper: &mut UartWrapper<UartT>, updater: &mut BlockingFirmwareUpdater<'a, DFU, STATE>) {
        info!("Entered writing loop");
        updater.mark_booted().unwrap();
        let dfu = updater.prepare_update().unwrap();
        uart_wrapper.write_response(SerialFlasherResponse::Ack).unwrap();

        
        loop {
            let command = uart_wrapper.read_command();
            info!("Handling command {:?}", command);

            match command {
                SerialFlasherCommand::Hello => uart_wrapper.write_response(SerialFlasherResponse::Ack).unwrap(),
                SerialFlasherCommand::WritePage(a, p, b) => {
                    let page_ref = p.as_ref();
                    let calculated_crc = CRC8.checksum(page_ref);

                    info!("Writing page at 0x{:?}, checksum {:?}", a, b);

                    if calculated_crc != b {
                        uart_wrapper.write_response(SerialFlasherResponse::Nack).unwrap();
                        continue;
                    }

                    info!("We're really doing it");

                    let res = dfu.write(a as u32, page_ref).map_err(|_| SerialFlasherError::FlashWriteError);

                    if let Err(e) = res {
                        info!("Write error: {:?}", e);
                        uart_wrapper.write_response(SerialFlasherResponse::Nack).unwrap();
                    } else {

                        info!("It's done.");

                        uart_wrapper.write_response(SerialFlasherResponse::Ack).unwrap();
                    }
                },
                SerialFlasherCommand::FinishedWriting => {
                    uart_wrapper.write_response(SerialFlasherResponse::Ack).unwrap();
                    break;
                }
                _ => uart_wrapper.write_response(SerialFlasherResponse::Nack).unwrap(),
            }
        }
    }

    fn verification_loop(uart_wrapper: &mut UartWrapper<UartT>, updater: &mut BlockingFirmwareUpdater<'a, DFU, STATE>) {
        info!("Entered verification loop");
        loop {
            let command = uart_wrapper.read_command();

            match command {
                SerialFlasherCommand::Hello => uart_wrapper.write_response(SerialFlasherResponse::Ack).unwrap(),
                SerialFlasherCommand::CompareChecksum(a, b) => {
                    let mut chunk_buf = [0; 2];
                    let mut message: Sha256Checksum = [0; 16];

                    let res = updater.hash::<Sha256>(a, &mut chunk_buf, &mut message).map_err(|_| SerialFlasherError::FlashWriteError);

                    if res.is_err() {
                        uart_wrapper.write_response(SerialFlasherResponse::Nack);
                    } else if message == b {
                        uart_wrapper.write_response(SerialFlasherResponse::Ack);
                    } else {
                        uart_wrapper.write_response(SerialFlasherResponse::Nack);
                    }
                },
                SerialFlasherCommand::MarkUpdated => {
                    let res = updater.mark_updated();

                    if res.is_err() {
                        uart_wrapper.write_response(SerialFlasherResponse::Nack).unwrap()
                    } else {
                        uart_wrapper.write_response(SerialFlasherResponse::Ack).unwrap();

                        break;
                    }
                },
                _ => uart_wrapper.write_response(SerialFlasherResponse::Nack).unwrap(),
            }
        }
    }

    pub fn main_loop(&mut self) {
        let mut updater = self.firmware_updater.take().unwrap();

        // Wait for prepare
        Self::unprepared_loop(&mut self.uart);

        Self::writing_loop(&mut self.uart, &mut updater);

        Self::verification_loop(&mut self.uart, &mut updater);

        info!("Done with main loop");
    }
}

pub fn main_loop<UartT: uart::Instance, const FLASH_SIZE: usize>(flash: FLASH, watchdog: WATCHDOG, uart: BufferedUart<UartT>) -> ! {
    // Override bootloader watchdog
    let mut watchdog = Watchdog::new(watchdog);
    //watchdog.start(Duration::from_secs(8));

    info!("Entered main loop");

    let flash = Flash::<_, _, FLASH_SIZE>::new_blocking(flash);
    let flash = Mutex::<NoopRawMutex, _>::new(RefCell::new(flash));

    let config = FirmwareUpdaterConfig::from_linkerfile_blocking(&flash, &flash);

    let mut aligned = AlignedBuffer([0; 1]);
    let updater = BlockingFirmwareUpdater::new(config, &mut aligned.0);

    let mut fl = SerialFlasher::new(updater, uart, watchdog);

    fl.main_loop();

    info!("update marked, resetting");
    cortex_m::peripheral::SCB::sys_reset();
}