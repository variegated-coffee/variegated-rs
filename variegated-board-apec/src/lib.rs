#![no_std]

#[cfg(feature = "bootloader")]
mod bootloader_features;
#[cfg(not(feature = "bootloader"))]
mod normal_features;

use bitflags::bitflags;
use variegated_board_features::{AllPurposeEspressoControllerBoardFeatures, BootloaderFeatures, OpenLCCBoardFeatures};
use embassy_rp::{bind_interrupts, i2c, peripherals, Peripherals, pwm, spi, uart};
use embassy_rp::gpio::{AnyPin, Input, Level, Output, Pin, Pull};
use embassy_rp::i2c::Config;
use embassy_rp::pwm::Pwm;
use embassy_rp::spi::{Phase, Polarity, Spi};
use embassy_rp::uart::{BufferedInterruptHandler, BufferedUart, Uart};
use static_cell::StaticCell;
use variegated_embassy_ads124s08::{ADS124S08, WaitStrategy};
use variegated_embassy_dual_c595_shift_register::DualC595ShiftRegister;

pub const FLASH_SIZE: usize = 2 * 1024 * 1024;

pub type ApecR0DBoardFeatures = AllPurposeEspressoControllerBoardFeatures<
    'static,
    peripherals::UART1,
    peripherals::UART0,
    peripherals::I2C1,
    peripherals::SPI1,
    peripherals::PIN_10,
    peripherals::PIN_24,
    peripherals::PIN_25,
    peripherals::PIN_26,
>;

bitflags! {
    pub struct ApecR0dShiftRegisterPositions: u16 {
        const CN1_3V3 = 0b0000_0000_0000_00001;
        const CN1_12V = 0b0000_0000_0000_0010;
        const VOUT1 = 0b0000_0000_0000_0100;
        const VOUT2 = 0b0000_0000_0000_1000;
        const CN11 = 0b0000_0000_0001_0000;
        const CN10 = 0b0000_0000_0010_0000;
        const VOUT3 = 0b0000_0000_0100_0000;
        const VOUT4 = 0b0000_0000_1000_0000;
        const CN9_7 = 0b0000_0001_0000_0000;
        const CN9_5 = 0b0000_0010_0000_0000;
        const CN9_3 = 0b0000_0100_0000_0000;
        const CN2_FA7 = 0b0001_0000_0000_0000;
        const CN2_FA8 = 0b0010_0000_0000_0000;
        const CN2_FA9 = 0b0100_0000_0000_0000;
        const CN2_FA10 = 0b1000_0000_0000_0000;
    }
}

#[cfg(feature = "bootloader")]
pub fn create_bootloader_features(p: Peripherals) -> R0DBootloaderFeatures {
    bootloader_features::create_bootloader_features(p)
}

#[cfg(not(feature = "bootloader"))]
pub fn create_board_features(p: Peripherals) -> ApecR0DBoardFeatures {
    normal_features::create_board_features(p)
}