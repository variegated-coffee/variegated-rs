//! HD44780 LCD AsyncDevice implementation using MCP23017 I2C GPIO expander
//!
//! This module provides an AsyncDevice implementation for the hd44780-controller crate
//! using the MCP23017 I2C GPIO expander. The implementation uses 8-bit mode for
//! optimal performance with bulk I2C operations.
//!
//! Pin mapping:
//! - GPB0-7: DB0-7 (8-bit data bus)
//! - GPA0: RS (Register Select)
//! - GPA1: RW (Read/Write)
//! - GPA2: EN (Enable)
//! - GPA3: Backlight control (optional)

use embassy_time::{Duration, Timer};
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c as AsyncI2c;
use variegated_mcp23017::{Mcp23017, PinDirection};
use hd44780_controller::device::{Device, AsyncDevice, RegisterSelectMode, RWMode};

/// Errors that can occur during HD44780 operations
#[derive(Debug)]
pub enum HD44780Error<E> {
    /// MCP23017 communication error
    Mcp23017(variegated_mcp23017::Error<E>),
    /// Invalid operation or parameter
    InvalidOperation,
}

impl<E> From<variegated_mcp23017::Error<E>> for HD44780Error<E> {
    fn from(err: variegated_mcp23017::Error<E>) -> Self {
        HD44780Error::Mcp23017(err)
    }
}

/// HD44780 control pin definitions for MCP23017.
///
/// Defined in [`crate::lcd_pins`], which is compiled whether or not this module is:
/// builds without `character-display` still need the map, to park the same twelve pins.
use crate::lcd_pins::pins;

/// HD44780 LCD device using MCP23017 I2C GPIO expander
///
/// This implements the hd44780-controller Device and AsyncDevice traits
/// for use with the hd44780-controller crate. Uses 8-bit mode with
/// efficient bulk I2C operations.
pub struct Mcp23017HD44780Device<I2C, D> {
    mcp23017: Mcp23017<I2C, D>,
    // Cache current control pin states to minimize I2C transactions
    control_state: u8,
    // Track if control state needs to be written
    control_dirty: bool,
    // Current data bus value (for debugging/state tracking)
    data_state: u8,
}

impl<I2C, D> Mcp23017HD44780Device<I2C, D>
where
    I2C: AsyncI2c,
    D: DelayNs,
{
    /// Create a new HD44780 device taking ownership of the MCP23017 instance
    pub fn new(mcp23017: Mcp23017<I2C, D>) -> Self {
        Self {
            mcp23017,
            // Initialize with backlight on, all other control pins low
            control_state: pins::BACKLIGHT_MASK,
            control_dirty: true,
            data_state: 0,
        }
    }

    /// Initialize the MCP23017 pins for HD44780 use (8-bit mode)
    pub async fn init_pins(&mut self) -> Result<(), HD44780Error<I2C::Error>> {
        // Configure all control pins on Port A as outputs
        for pin in 0..pins::CONTROL_PIN_COUNT {
            self.mcp23017.set_pin_direction(pin, PinDirection::Output).await?;
        }

        // Configure all data pins on Port B (8-bit mode: GPB0-7) as outputs
        for pin in pins::DATA_PINS {
            self.mcp23017.set_pin_direction(pin, PinDirection::Output).await?;
        }

        // Write initial states
        self.flush_control_state().await?;
        self.mcp23017.write_port_b(0x00).await?; // Clear data bus
        self.data_state = 0;

        Ok(())
    }

    /// Update a control pin bit in the cached state
    fn set_control_bit(&mut self, mask: u8, value: bool) {
        let new_state = if value {
            self.control_state | mask
        } else {
            self.control_state & !mask
        };

        if new_state != self.control_state {
            self.control_state = new_state;
            self.control_dirty = true;
        }
    }

    /// Flush cached control state to the MCP23017 if needed
    async fn flush_control_state(&mut self) -> Result<(), HD44780Error<I2C::Error>> {
        if self.control_dirty {
            self.mcp23017.write_port_a(self.control_state).await?;
            self.control_dirty = false;
        }
        Ok(())
    }
}

/// Implementation of hd44780_controller::Device trait
impl<I2C, D> Device for Mcp23017HD44780Device<I2C, D>
where
    I2C: AsyncI2c,
    D: DelayNs,
{
    fn set_register_select(&mut self, mode: RegisterSelectMode) {
        let rs_high = match mode {
            RegisterSelectMode::Command => false,
            RegisterSelectMode::Data => true,
        };
        // Cache the state change, actual I2C write happens in flush_async()
        self.set_control_bit(pins::RS_MASK, rs_high);
    }

    fn set_rw(&mut self, mode: RWMode) {
        let rw_high = match mode {
            RWMode::Write => false,
            RWMode::Read => true,
        };
        // Cache the state change, actual I2C write happens in flush_async()
        self.set_control_bit(pins::RW_MASK, rw_high);
    }

    fn set_enable(&mut self, enabled: bool) {
        // Cache the state change, actual I2C write happens in flush_async()
        self.set_control_bit(pins::EN_MASK, enabled);
    }

    fn set_backlight(&mut self, enabled: bool) {
        // Cache the state change, actual I2C write happens in flush_async()
        self.set_control_bit(pins::BACKLIGHT_MASK, enabled);
    }

    fn set_data_nibble(&mut self, data: u8) {
        // For 8-bit mode, we don't use nibbles, but we need to implement this
        // for compatibility. Just store the nibble - it won't be used in 8-bit mode.
        let data = data & 0x0F; // Ensure only lower 4 bits
        self.data_state = data;
    }
}

/// Implementation of hd44780_controller::AsyncDevice trait
impl<I2C, D> AsyncDevice for Mcp23017HD44780Device<I2C, D>
where
    I2C: AsyncI2c,
    D: DelayNs,
{
    type Err = HD44780Error<I2C::Error>;

    async fn delay_us_async(&mut self, us: u32) {
        Timer::after(Duration::from_micros(us as u64)).await;
    }

    async fn flush_async(&mut self) -> Result<(), Self::Err> {
        // Flush cached control state if needed
        self.flush_control_state().await?;
        Ok(())
    }

    /// Override write_byte_async for true 8-bit operation
    async fn write_byte_async(
        &mut self,
        mode: RegisterSelectMode,
        byte: u8,
    ) -> Result<(), Self::Err> {
        // Set register select and write mode
        self.set_register_select(mode);
        self.set_rw(RWMode::Write);

        // Write the full 8-bit data to Port B in one I2C transaction
        self.mcp23017.write_port_b(byte).await?;
        self.data_state = byte;

        // Enable pulse: set high, flush, delay, set low, flush
        self.set_enable(true);
        self.flush_async().await?;

        // Short enable pulse delay (typical HD44780 requirement: ~1μs minimum)
        self.delay_us_async(2).await;

        self.set_enable(false);
        self.flush_async().await?;

        Ok(())
    }

    /// Override write_nibble_async for proper 4-bit operation
    async fn write_nibble_async(
        &mut self,
        mode: RegisterSelectMode,
        data: u8,
    ) -> Result<(), Self::Err> {
        // Set register select and write mode
        self.set_register_select(mode);
        self.set_rw(RWMode::Write);

        // For 4-bit mode, write nibble to lower 4 bits of Port B
        let nibble = data & 0x0F;
        // Read current port state and update only lower 4 bits
        let current_port_b = self.mcp23017.read_port_b().await.unwrap_or(0);
        let new_port_b = (current_port_b & 0xF0) | nibble;
        self.mcp23017.write_port_b(new_port_b).await?;
        self.data_state = nibble;

        // Enable pulse: set high, flush, delay, set low, flush
        self.set_enable(true);
        self.flush_async().await?;

        // Short enable pulse delay
        self.delay_us_async(2).await;

        self.set_enable(false);
        self.flush_async().await?;

        Ok(())
    }
}