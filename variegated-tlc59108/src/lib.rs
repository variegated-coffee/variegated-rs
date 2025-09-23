//! # TLC59108 LED Driver
//!
//! Async driver for the TI TLC59108 8-bit I2C LED driver.
//!
//! The TLC59108 is an 8-channel constant-current LED sink driver with individual
//! PWM brightness control and group dimming/blinking capabilities. Each output
//! can sink up to 120mA (configured by external resistor).
//!
//! ## Features
//! - 8 constant-current LED outputs
//! - Individual 8-bit PWM brightness control (97kHz)
//! - Group brightness control and blinking (190Hz or configurable)
//! - I2C Fast-mode Plus (up to 1MHz)
//! - Software reset capability
//!
//! ## Example
//! ```no_run
//! use variegated_tlc59108::{Tlc59108, Tlc59108Config, LedState, GroupMode, IrefConfig};
//!
//! # async fn example() -> Result<(), Box<dyn std::error::Error>> {
//! # let i2c = todo!();
//! # let delay = todo!();
//! // Configure the driver
//! let config = Tlc59108Config {
//!     address: 0x40, // Default address with A0-A3 = 0
//!     output_change_on_ack: false, // Update on STOP command
//!     group_mode: GroupMode::Dimming,
//!     group_brightness: 255, // Full brightness
//!     group_frequency: 0, // Not used in dimming mode
//!     iref_config: Some(IrefConfig::high_current()), // Configure for 50-120mA range
//! };
//!
//! let mut driver = Tlc59108::new(i2c, delay, config);
//! driver.init().await?;
//!
//! // Set LED 0 to 50% brightness
//! driver.set_led(0, 128, LedState::Pwm).await?;
//!
//! // Set all LEDs to different brightness levels
//! let brightness = [255, 200, 150, 100, 50, 25, 10, 0];
//! let states = [LedState::Pwm; 8];
//! driver.set_all_leds(&brightness, &states).await?;
//!
//! // Adjust output current for low power operation
//! driver.set_iref(IrefConfig::low_current()).await?;
//!
//! // Calculate actual output current with 750Ω external resistor
//! let current_ma = driver.calculate_output_current(750.0).await?;
//! // current_ma will be approximately 20mA with low_current() configuration
//! # Ok(())
//! # }
//! ```

#![no_std]
#![warn(missing_docs)]

use core::fmt;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;

mod registers;
pub use registers::IrefConfig;
use registers::*;

/// Errors that can occur when communicating with the TLC59108
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<E> {
    /// I2C communication error
    I2c(E),
    /// Invalid LED channel (must be 0-7)
    InvalidChannel,
    /// Device initialization failed
    InitializationFailed,
}

impl<E: fmt::Debug> fmt::Display for Error<E> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::I2c(e) => write!(f, "I2C error: {:?}", e),
            Error::InvalidChannel => write!(f, "Invalid LED channel"),
            Error::InitializationFailed => write!(f, "Device initialization failed"),
        }
    }
}

/// Group control mode
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GroupMode {
    /// Group control adjusts brightness (190Hz PWM)
    Dimming,
    /// Group control blinks LEDs on/off
    Blinking,
}

/// LED output state configuration
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LedState {
    /// LED output is off
    Off,
    /// LED output is fully on (no PWM)
    FullyOn,
    /// LED brightness controlled by individual PWM register
    Pwm,
    /// LED brightness controlled by PWM and group control
    PwmAndGroup,
}

impl LedState {
    fn to_register_bits(&self) -> u8 {
        match self {
            LedState::Off => 0b00,
            LedState::FullyOn => 0b01,
            LedState::Pwm => 0b10,
            LedState::PwmAndGroup => 0b11,
        }
    }
}

/// Configuration for the TLC59108 driver
#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Tlc59108Config {
    /// I2C address (0x40 + A3:A0 pins)
    pub address: u8,
    /// Update outputs on ACK (true) or STOP (false)
    pub output_change_on_ack: bool,
    /// Group control mode
    pub group_mode: GroupMode,
    /// Group brightness/duty cycle (0-255)
    pub group_brightness: u8,
    /// Group blinking frequency (see datasheet for calculation)
    pub group_frequency: u8,
    /// Output current configuration (None uses hardware default 0xFF)
    pub iref_config: Option<IrefConfig>,
}

impl Default for Tlc59108Config {
    fn default() -> Self {
        Self {
            address: 0x40, // Default with A0-A3 = 0
            output_change_on_ack: false,
            group_mode: GroupMode::Dimming,
            group_brightness: 255,
            group_frequency: 0,
            iref_config: None, // Use hardware default (maximum current)
        }
    }
}

/// TLC59108 LED driver
pub struct Tlc59108<I2C, D> {
    i2c: I2C,
    delay: D,
    config: Tlc59108Config,
}

impl<I2C, D> Tlc59108<I2C, D>
where
    I2C: I2c,
    D: DelayNs,
{
    /// Create a new TLC59108 driver with the given configuration
    pub fn new(i2c: I2C, delay: D, config: Tlc59108Config) -> Self {
        Self { i2c, delay, config }
    }

    /// Initialize the TLC59108
    pub async fn init(&mut self) -> Result<(), Error<I2C::Error>> {
        // Perform software reset
        self.software_reset().await?;

        // Wait for reset to complete
        self.delay.delay_ms(10).await;

        // Configure MODE1 register
        // Enable oscillator, set auto-increment for all registers
        let mode1 = AutoIncrement::All.bits();
        self.write_register(Register::Mode1, mode1).await?;

        // Configure MODE2 register
        let mut mode2 = 0;
        if self.config.output_change_on_ack {
            mode2 |= mode2::OCH;
        }
        if matches!(self.config.group_mode, GroupMode::Blinking) {
            mode2 |= mode2::DMBLNK;
        }
        self.write_register(Register::Mode2, mode2).await?;

        // Set group PWM and frequency
        self.write_register(Register::GrpPwm, self.config.group_brightness).await?;
        self.write_register(Register::GrpFreq, self.config.group_frequency).await?;

        // Configure output current if specified
        if let Some(iref) = self.config.iref_config {
            self.write_register(Register::Iref, iref.to_register()).await?;
        }

        // Initialize all LEDs to off
        self.write_register(Register::LedOut0, 0x00).await?;
        self.write_register(Register::LedOut1, 0x00).await?;

        // Initialize all PWM channels to 0
        for i in 0..8 {
            let reg = Register::Pwm0 as u8 + i;
            self.write_register_raw(reg, 0).await?;
        }

        Ok(())
    }

    /// Perform a software reset
    async fn software_reset(&mut self) -> Result<(), Error<I2C::Error>> {
        let data = [SWRST_BYTE1, SWRST_BYTE2];
        self.i2c
            .write(SWRST_ADDR, &data)
            .await
            .map_err(Error::I2c)?;
        Ok(())
    }

    /// Set a single LED's brightness and state
    pub async fn set_led(
        &mut self,
        channel: u8,
        brightness: u8,
        state: LedState,
    ) -> Result<(), Error<I2C::Error>> {
        if channel > 7 {
            return Err(Error::InvalidChannel);
        }

        // Set PWM brightness
        let pwm_reg = Register::Pwm0 as u8 + channel;
        self.write_register_raw(pwm_reg, brightness).await?;

        // Read current LED output state register
        let ledout_reg = if channel < 4 {
            Register::LedOut0
        } else {
            Register::LedOut1
        };

        let mut ledout = self.read_register(ledout_reg).await?;

        // Calculate bit position (2 bits per LED)
        let bit_pos = (channel % 4) * 2;

        // Clear the bits for this LED
        ledout &= !(0b11 << bit_pos);

        // Set new state
        ledout |= state.to_register_bits() << bit_pos;

        // Write back
        self.write_register(ledout_reg, ledout).await?;

        Ok(())
    }

    /// Set all LEDs at once
    pub async fn set_all_leds(
        &mut self,
        brightness: &[u8; 8],
        states: &[LedState; 8],
    ) -> Result<(), Error<I2C::Error>> {
        // Set up auto-increment for brightness registers
        let mode1 = AutoIncrement::Brightness.bits();
        self.write_register(Register::Mode1, mode1).await?;

        // Write all brightness values in one transaction
        let mut data = [0u8; 9];
        data[0] = Register::Pwm0.addr() | 0x80; // Auto-increment flag
        data[1..9].copy_from_slice(brightness);
        self.i2c
            .write(self.config.address, &data)
            .await
            .map_err(Error::I2c)?;

        // Configure LED output states
        let mut ledout0 = 0u8;
        let mut ledout1 = 0u8;

        for i in 0..4 {
            ledout0 |= states[i].to_register_bits() << (i * 2);
        }
        for i in 4..8 {
            ledout1 |= states[i].to_register_bits() << ((i - 4) * 2);
        }

        self.write_register(Register::LedOut0, ledout0).await?;
        self.write_register(Register::LedOut1, ledout1).await?;

        Ok(())
    }

    /// Read error flags
    pub async fn read_error_flags(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_register(Register::EFlag).await
    }

    /// Clear error flags
    pub async fn clear_error_flags(&mut self) -> Result<(), Error<I2C::Error>> {
        // Read MODE2 to get current settings
        let mode2 = self.read_register(Register::Mode2).await?;

        // Set EFCLR bit
        self.write_register(Register::Mode2, mode2 | mode2::EFCLR).await?;

        // Clear EFCLR bit
        self.write_register(Register::Mode2, mode2 & !mode2::EFCLR).await?;

        Ok(())
    }

    /// Set the output current configuration
    ///
    /// This controls the maximum output current capability for all channels.
    /// The actual output current depends on the external resistor (REXT) value.
    pub async fn set_iref(&mut self, config: IrefConfig) -> Result<(), Error<I2C::Error>> {
        self.write_register(Register::Iref, config.to_register()).await?;
        self.config.iref_config = Some(config);
        Ok(())
    }

    /// Read the current IREF configuration
    pub async fn read_iref(&mut self) -> Result<IrefConfig, Error<I2C::Error>> {
        let value = self.read_register(Register::Iref).await?;
        Ok(IrefConfig::from_register(value))
    }

    /// Calculate the maximum output current for a given external resistor
    ///
    /// Returns the output current in milliamps based on current IREF configuration
    /// and the specified external resistor value.
    pub async fn calculate_output_current(&mut self, rext_ohms: f32) -> Result<f32, Error<I2C::Error>> {
        let iref_config = self.read_iref().await?;
        Ok(iref_config.output_current_ma(rext_ohms))
    }

    /// Write to a register
    async fn write_register(&mut self, reg: Register, value: u8) -> Result<(), Error<I2C::Error>> {
        self.write_register_raw(reg.addr(), value).await
    }

    /// Write to a register by address
    async fn write_register_raw(&mut self, reg: u8, value: u8) -> Result<(), Error<I2C::Error>> {
        let data = [reg, value];
        self.i2c
            .write(self.config.address, &data)
            .await
            .map_err(Error::I2c)?;
        Ok(())
    }

    /// Read from a register
    async fn read_register(&mut self, reg: Register) -> Result<u8, Error<I2C::Error>> {
        let mut buf = [0u8; 1];
        self.i2c
            .write_read(self.config.address, &[reg.addr()], &mut buf)
            .await
            .map_err(Error::I2c)?;
        Ok(buf[0])
    }

    /// Release the I2C bus and delay provider
    pub fn release(self) -> (I2C, D) {
        (self.i2c, self.delay)
    }
}