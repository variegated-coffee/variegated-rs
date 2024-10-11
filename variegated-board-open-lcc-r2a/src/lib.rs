#![no_std]

#[cfg(feature = "bootloader")]
mod bootloader_features;
#[cfg(not(feature = "bootloader"))]
mod normal_features;

use variegated_board_features::{BootloaderFeatures, OpenLCCBoardFeatures};
use embassy_rp::{bind_interrupts, i2c, peripherals, Peripherals, pwm, spi, uart};
use embassy_rp::gpio::{AnyPin, Pin};
use embassy_rp::i2c::Config;
use embassy_rp::peripherals::{PIN_12, PIN_14, PIN_17, PIN_24, SPI1};
use embassy_rp::pwm::Pwm;
use embassy_rp::spi::Spi;
use embassy_rp::uart::{BufferedInterruptHandler, BufferedUart, Uart};
use static_cell::StaticCell;

pub type R2ABoardFeatures = OpenLCCBoardFeatures<
    'static,
    peripherals::UART0,
    peripherals::UART1,
    peripherals::I2C0,
    peripherals::I2C1,
    peripherals::SPI1,
    peripherals::SPI0>;

pub type R2ABootloaderFeatures = BootloaderFeatures<'static, peripherals::UART0, PIN_12, PIN_14>;
pub const FLASH_SIZE: usize =  2 * 1024 * 1024;

#[cfg(feature = "bootloader")]
pub fn create_bootloader_features(p: Peripherals) -> R2ABootloaderFeatures {
    bootloader_features::create_bootloader_features(p)
}

#[cfg(not(feature = "bootloader"))]
pub fn create_board_features(p: Peripherals) -> R2ABoardFeatures {
    normal_features::create_board_features(p)
}