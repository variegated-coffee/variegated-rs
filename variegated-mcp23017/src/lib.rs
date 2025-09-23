//! # MCP23017 GPIO Expander Driver
//!
//! Async and blocking driver for the MCP23017 16-bit I2C GPIO expander.
//!
//! The MCP23017 provides 16 GPIO pins arranged as two 8-bit ports (A and B),
//! with individual pin configuration for direction, pull-ups, and interrupts.
//!
//! ## Features
//! - 16 GPIO pins (2 x 8-bit ports)
//! - Individual pin direction control
//! - Configurable pull-up resistors
//! - Interrupt-on-change capability
//! - Both blocking and async pin implementations
//! - Feature-gated async digital pin traits (requires `async-digital-pins` feature)
//!
//! ## Example
//! ```no_run
//! use variegated_mcp23017::{Mcp23017, Mcp23017Config};
//! use embassy_sync::mutex::Mutex;
//! use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
//!
//! # async fn example() -> Result<(), Box<dyn std::error::Error>> {
//! # let i2c = todo!();
//! # let delay = todo!();
//! // Configure the driver
//! let config = Mcp23017Config {
//!     address: 0x20, // Default address
//!     ..Default::default()
//! };
//!
//! let driver = Mcp23017::new(i2c, delay, config);
//! let shared_driver = Mutex::<CriticalSectionRawMutex, _>::new(driver);
//!
//! // Initialize the device
//! {
//!     let mut driver = shared_driver.lock().await;
//!     driver.init().await?;
//! }
//!
//! // Create pin instances
//! let pin0 = shared_driver.pin(0);
//! let pin1 = shared_driver.pin(1);
//!
//! // Configure pins and use them...
//! # Ok(())
//! # }
//! ```
//!
//! ## Feature Gates
//!
//! - `async-digital-pins`: Enables async `InputPin` and `OutputPin` trait implementations.
//!   These traits are only available in embedded_hal_async master branch. The `Wait` trait
//!   is always available as it's in embedded_hal_async 1.0.0.
//!
//! ```toml
//! [dependencies]
//! variegated-mcp23017 = { version = "0.1", features = ["async-digital-pins"] }
//! ```

#![no_std]
#![warn(missing_docs)]

use core::fmt;
use embedded_hal::digital::{InputPin as BlockingInputPin, OutputPin as BlockingOutputPin};
#[cfg(not(feature = "async-digital-pins"))]
use embedded_hal_async::delay::DelayNs;
#[cfg(not(feature = "async-digital-pins"))]
use embedded_hal_async::i2c::I2c as AsyncI2c;

#[cfg(feature = "async-digital-pins")]
use embedded_hal_async_git::delay::DelayNs;
#[cfg(feature = "async-digital-pins")]
use embedded_hal_async_git::i2c::I2c as AsyncI2c;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::mutex::Mutex;

mod registers;
use registers::*;

/// Errors that can occur when communicating with the MCP23017
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<E> {
    /// I2C communication error
    I2c(E),
    /// Invalid pin number (must be 0-15)
    InvalidPin,
    /// Device initialization failed
    InitializationFailed,
}

impl<E: fmt::Debug> fmt::Display for Error<E> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::I2c(e) => write!(f, "I2C error: {:?}", e),
            Error::InvalidPin => write!(f, "Invalid pin number"),
            Error::InitializationFailed => write!(f, "Device initialization failed"),
        }
    }
}

// Note: embedded_hal_async::digital::Error trait doesn't exist in stable 1.0.0

#[cfg(feature = "async-digital-pins")]
impl<E> embedded_hal_async_git::digital::Error for Error<E>
where
    E: fmt::Debug,
{
    fn kind(&self) -> embedded_hal_async_git::digital::ErrorKind {
        embedded_hal_async_git::digital::ErrorKind::Other
    }
}

impl<E> embedded_hal::digital::Error for Error<E>
where
    E: fmt::Debug,
{
    fn kind(&self) -> embedded_hal::digital::ErrorKind {
        embedded_hal::digital::ErrorKind::Other
    }
}

/// Pin direction configuration
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PinDirection {
    /// Pin configured as output
    Output,
    /// Pin configured as input
    Input,
}

/// Configuration for the MCP23017 driver
#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Mcp23017Config {
    /// I2C address (0x20-0x27 based on A0-A2 pins)
    pub address: u8,
    /// Enable sequential register operation
    pub sequential_operation: bool,
    /// Enable interrupt mirroring (both INT pins reflect OR of interrupts)
    pub mirror_interrupts: bool,
    /// Interrupt polarity (true = active high, false = active low)
    pub interrupt_active_high: bool,
    /// Use open-drain output for interrupt pins
    pub interrupt_open_drain: bool,
}

impl Default for Mcp23017Config {
    fn default() -> Self {
        Self {
            address: DEFAULT_ADDRESS,
            sequential_operation: true,
            mirror_interrupts: false,
            interrupt_active_high: false,
            interrupt_open_drain: false,
        }
    }
}

/// MCP23017 GPIO expander driver
pub struct Mcp23017<I2C, D> {
    i2c: I2C,
    delay: D,
    config: Mcp23017Config,
    /// Cached register values to optimize I2C operations
    iodir_cache: [u8; 2],    // IODIRA, IODIRB
    gpio_cache: [u8; 2],     // GPIOA, GPIOB
    gppu_cache: [u8; 2],     // GPPUA, GPPUB
    cache_valid: bool,
}

impl<I2C, D> Mcp23017<I2C, D>
where
    I2C: AsyncI2c,
    D: DelayNs,
{
    /// Create a new MCP23017 driver with the given configuration
    pub fn new(i2c: I2C, delay: D, config: Mcp23017Config) -> Self {
        Self {
            i2c,
            delay,
            config,
            iodir_cache: [0xFF; 2], // Default: all inputs
            gpio_cache: [0x00; 2],  // Default: all low
            gppu_cache: [0x00; 2],  // Default: no pull-ups
            cache_valid: false,
        }
    }

    /// Initialize the MCP23017
    pub async fn init(&mut self) -> Result<(), Error<I2C::Error>> {
        // Configure IOCON register
        let mut iocon = 0u8;

        if !self.config.sequential_operation {
            iocon |= iocon::SEQOP;
        }
        if self.config.mirror_interrupts {
            iocon |= iocon::MIRROR;
        }
        if self.config.interrupt_active_high {
            iocon |= iocon::INTPOL;
        }
        if self.config.interrupt_open_drain {
            iocon |= iocon::ODR;
        }

        self.write_register(Register::IoCon, iocon).await?;

        // Initialize all pins as inputs with no pull-ups
        self.write_register(Register::IoDirA, 0xFF).await?;
        self.write_register(Register::IoDirB, 0xFF).await?;
        self.write_register(Register::GpPuA, 0x00).await?;
        self.write_register(Register::GpPuB, 0x00).await?;

        // Clear all interrupt enables
        self.write_register(Register::GpIntEnA, 0x00).await?;
        self.write_register(Register::GpIntEnB, 0x00).await?;

        // Read current GPIO states to initialize cache
        self.gpio_cache[0] = self.read_register(Register::GpioA).await?;
        self.gpio_cache[1] = self.read_register(Register::GpioB).await?;
        self.iodir_cache[0] = 0xFF;
        self.iodir_cache[1] = 0xFF;
        self.gppu_cache[0] = 0x00;
        self.gppu_cache[1] = 0x00;
        self.cache_valid = true;

        Ok(())
    }

    /// Configure a pin's direction
    pub async fn set_pin_direction(&mut self, pin: u8, direction: PinDirection) -> Result<(), Error<I2C::Error>> {
        if pin > 15 {
            return Err(Error::InvalidPin);
        }

        let (port_idx, bit) = self.pin_to_port_bit(pin);
        let register = if port_idx == 0 { Register::IoDirA } else { Register::IoDirB };

        let mut iodir = if self.cache_valid {
            self.iodir_cache[port_idx]
        } else {
            self.read_register(register).await?
        };

        match direction {
            PinDirection::Input => iodir |= 1 << bit,
            PinDirection::Output => iodir &= !(1 << bit),
        }

        self.write_register(register, iodir).await?;
        self.iodir_cache[port_idx] = iodir;
        self.cache_valid = true;

        Ok(())
    }

    /// Enable or disable pull-up for a pin
    pub async fn set_pin_pullup(&mut self, pin: u8, enabled: bool) -> Result<(), Error<I2C::Error>> {
        if pin > 15 {
            return Err(Error::InvalidPin);
        }

        let (port_idx, bit) = self.pin_to_port_bit(pin);
        let register = if port_idx == 0 { Register::GpPuA } else { Register::GpPuB };

        let mut gppu = if self.cache_valid {
            self.gppu_cache[port_idx]
        } else {
            self.read_register(register).await?
        };

        if enabled {
            gppu |= 1 << bit;
        } else {
            gppu &= !(1 << bit);
        }

        self.write_register(register, gppu).await?;
        self.gppu_cache[port_idx] = gppu;
        self.cache_valid = true;

        Ok(())
    }

    /// Read the state of a pin
    pub async fn read_pin(&mut self, pin: u8) -> Result<bool, Error<I2C::Error>> {
        if pin > 15 {
            return Err(Error::InvalidPin);
        }

        let (port_idx, bit) = self.pin_to_port_bit(pin);
        let register = if port_idx == 0 { Register::GpioA } else { Register::GpioB };

        let gpio = self.read_register(register).await?;
        self.gpio_cache[port_idx] = gpio;

        Ok((gpio & (1 << bit)) != 0)
    }

    /// Write the state of a pin (must be configured as output)
    pub async fn write_pin(&mut self, pin: u8, high: bool) -> Result<(), Error<I2C::Error>> {
        if pin > 15 {
            return Err(Error::InvalidPin);
        }

        let (port_idx, bit) = self.pin_to_port_bit(pin);
        let register = if port_idx == 0 { Register::GpioA } else { Register::GpioB };

        let mut gpio = if self.cache_valid {
            self.gpio_cache[port_idx]
        } else {
            self.read_register(register).await?
        };

        if high {
            gpio |= 1 << bit;
        } else {
            gpio &= !(1 << bit);
        }

        self.write_register(register, gpio).await?;
        self.gpio_cache[port_idx] = gpio;
        self.cache_valid = true;

        Ok(())
    }

    /// Write an entire 8-bit value to Port A (pins 0-7)
    pub async fn write_port_a(&mut self, value: u8) -> Result<(), Error<I2C::Error>> {
        self.write_register(Register::GpioA, value).await?;
        self.gpio_cache[0] = value;
        self.cache_valid = true;
        Ok(())
    }

    /// Write an entire 8-bit value to Port B (pins 8-15)
    pub async fn write_port_b(&mut self, value: u8) -> Result<(), Error<I2C::Error>> {
        self.write_register(Register::GpioB, value).await?;
        self.gpio_cache[1] = value;
        self.cache_valid = true;
        Ok(())
    }

    /// Read the entire 8-bit value from Port A (pins 0-7)
    pub async fn read_port_a(&mut self) -> Result<u8, Error<I2C::Error>> {
        let value = self.read_register(Register::GpioA).await?;
        self.gpio_cache[0] = value;
        self.cache_valid = true;
        Ok(value)
    }

    /// Read the entire 8-bit value from Port B (pins 8-15)
    pub async fn read_port_b(&mut self) -> Result<u8, Error<I2C::Error>> {
        let value = self.read_register(Register::GpioB).await?;
        self.gpio_cache[1] = value;
        self.cache_valid = true;
        Ok(value)
    }

    /// Configure interrupt for a pin
    pub async fn set_pin_interrupt(&mut self, pin: u8, mode: InterruptMode) -> Result<(), Error<I2C::Error>> {
        if pin > 15 {
            return Err(Error::InvalidPin);
        }

        let (port_idx, bit) = self.pin_to_port_bit(pin);
        let gpinten_reg = if port_idx == 0 { Register::GpIntEnA } else { Register::GpIntEnB };
        let intcon_reg = if port_idx == 0 { Register::IntConA } else { Register::IntConB };
        let defval_reg = if port_idx == 0 { Register::DefValA } else { Register::DefValB };

        let mut gpinten = self.read_register(gpinten_reg).await?;
        let mut intcon = self.read_register(intcon_reg).await?;
        let mut defval = self.read_register(defval_reg).await?;

        match mode {
            InterruptMode::Disabled => {
                gpinten &= !(1 << bit);
            }
            InterruptMode::OnChange => {
                gpinten |= 1 << bit;
                intcon &= !(1 << bit); // Compare to previous value
            }
            InterruptMode::OnDefault(default_high) => {
                gpinten |= 1 << bit;
                intcon |= 1 << bit; // Compare to default value
                if default_high {
                    defval |= 1 << bit;
                } else {
                    defval &= !(1 << bit);
                }
                self.write_register(defval_reg, defval).await?;
            }
        }

        self.write_register(gpinten_reg, gpinten).await?;
        self.write_register(intcon_reg, intcon).await?;

        Ok(())
    }

    /// Read interrupt flags for all pins
    pub async fn read_interrupt_flags(&mut self) -> Result<u16, Error<I2C::Error>> {
        let intf_a = self.read_register(Register::IntfA).await?;
        let intf_b = self.read_register(Register::IntfB).await?;
        Ok(((intf_b as u16) << 8) | (intf_a as u16))
    }

    /// Read interrupt capture register for all pins
    pub async fn read_interrupt_capture(&mut self) -> Result<u16, Error<I2C::Error>> {
        let intcap_a = self.read_register(Register::IntCapA).await?;
        let intcap_b = self.read_register(Register::IntCapB).await?;
        Ok(((intcap_b as u16) << 8) | (intcap_a as u16))
    }

    /// Helper to convert pin number to port index and bit position
    fn pin_to_port_bit(&self, pin: u8) -> (usize, u8) {
        if pin < 8 {
            (0, pin) // Port A
        } else {
            (1, pin - 8) // Port B
        }
    }

    /// Write to a register
    async fn write_register(&mut self, reg: Register, value: u8) -> Result<(), Error<I2C::Error>> {
        let data = [reg.addr(), value];
        AsyncI2c::write(&mut self.i2c, self.config.address, &data)
            .await
            .map_err(Error::I2c)?;
        Ok(())
    }

    /// Read from a register
    async fn read_register(&mut self, reg: Register) -> Result<u8, Error<I2C::Error>> {
        let mut buf = [0u8; 1];
        AsyncI2c::write_read(&mut self.i2c, self.config.address, &[reg.addr()], &mut buf)
            .await
            .map_err(Error::I2c)?;
        Ok(buf[0])
    }

    /// Release the I2C bus and delay provider
    pub fn release(self) -> (I2C, D) {
        (self.i2c, self.delay)
    }
}

/// Extension trait for creating pin instances from a shared driver
pub trait Mcp23017Ext<M: RawMutex, I2C, D> {
    /// Create a pin instance for the given pin number
    fn pin(&self, pin: u8) -> Mcp23017Pin<M, I2C, D>;

    /// Create an interrupt-capable pin instance
    fn interrupt_pin<INT>(&self, pin: u8, interrupt_pin: INT) -> Mcp23017InterruptPin<M, I2C, D, INT>;
}

impl<M: RawMutex, I2C, D> Mcp23017Ext<M, I2C, D> for Mutex<M, Mcp23017<I2C, D>> {
    fn pin(&self, pin: u8) -> Mcp23017Pin<M, I2C, D> {
        let (port, bit) = if pin < 8 {
            (Port::A, pin)
        } else {
            (Port::B, pin - 8)
        };

        Mcp23017Pin {
            driver: self,
            pin_number: pin,
            port,
            bit,
        }
    }

    fn interrupt_pin<INT>(&self, pin: u8, interrupt_pin: INT) -> Mcp23017InterruptPin<M, I2C, D, INT> {
        let (port, bit) = if pin < 8 {
            (Port::A, pin)
        } else {
            (Port::B, pin - 8)
        };

        Mcp23017InterruptPin {
            driver: self,
            interrupt_pin,
            pin_number: pin,
            port,
            bit,
        }
    }
}

/// Pin wrapper for accessing individual MCP23017 pins
pub struct Mcp23017Pin<'a, M: RawMutex, I2C, D> {
    driver: &'a Mutex<M, Mcp23017<I2C, D>>,
    pin_number: u8,
    port: Port,
    bit: u8,
}

/// Interrupt-capable pin wrapper
pub struct Mcp23017InterruptPin<'a, M: RawMutex, I2C, D, INT> {
    driver: &'a Mutex<M, Mcp23017<I2C, D>>,
    interrupt_pin: INT,
    pin_number: u8,
    port: Port,
    bit: u8,
}

// Blocking trait implementations for standard embedded_hal
impl<M, I2C, D> embedded_hal::digital::ErrorType for Mcp23017Pin<'_, M, I2C, D>
where
    M: RawMutex,
    I2C: AsyncI2c,
{
    type Error = Error<I2C::Error>;
}

impl<M, I2C, D> BlockingInputPin for Mcp23017Pin<'_, M, I2C, D>
where
    M: RawMutex,
    I2C: AsyncI2c,
    D: DelayNs,
{
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        // Use embassy's blocking operations within an async context
        embassy_futures::block_on(async {
            let mut driver = self.driver.lock().await;
            driver.read_pin(self.pin_number).await
        })
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        self.is_high().map(|high| !high)
    }
}

impl<M, I2C, D> BlockingOutputPin for Mcp23017Pin<'_, M, I2C, D>
where
    M: RawMutex,
    I2C: AsyncI2c,
    D: DelayNs,
{
    fn set_high(&mut self) -> Result<(), Self::Error> {
        embassy_futures::block_on(async {
            let mut driver = self.driver.lock().await;
            driver.write_pin(self.pin_number, true).await
        })
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        embassy_futures::block_on(async {
            let mut driver = self.driver.lock().await;
            driver.write_pin(self.pin_number, false).await
        })
    }
}

// Async trait implementations for embedded_hal_async (feature gated)
#[cfg(feature = "async-digital-pins")]
impl<M, I2C, D> embedded_hal_async_git::digital::ErrorType for Mcp23017Pin<'_, M, I2C, D>
where
    M: RawMutex,
    I2C: AsyncI2c,
{
    type Error = Error<I2C::Error>;
}

#[cfg(feature = "async-digital-pins")]
impl<M, I2C, D> embedded_hal_async_git::digital::InputPin for Mcp23017Pin<'_, M, I2C, D>
where
    M: RawMutex,
    I2C: AsyncI2c,
    D: DelayNs,
{
    async fn is_high(&mut self) -> Result<bool, Self::Error> {
        let mut driver = self.driver.lock().await;
        driver.read_pin(self.pin_number).await
    }

    async fn is_low(&mut self) -> Result<bool, Self::Error> {
        let high = embedded_hal_async_git::digital::InputPin::is_high(self).await?;
        Ok(!high)
    }
}

#[cfg(feature = "async-digital-pins")]
impl<M, I2C, D> embedded_hal_async_git::digital::OutputPin for Mcp23017Pin<'_, M, I2C, D>
where
    M: RawMutex,
    I2C: AsyncI2c,
    D: DelayNs,
{
    async fn set_high(&mut self) -> Result<(), Self::Error> {
        let mut driver = self.driver.lock().await;
        driver.write_pin(self.pin_number, true).await
    }

    async fn set_low(&mut self) -> Result<(), Self::Error> {
        let mut driver = self.driver.lock().await;
        driver.write_pin(self.pin_number, false).await
    }
}

// Wait trait implementation for interrupt pins (always available)
#[cfg(not(feature = "async-digital-pins"))]
impl<M, I2C, D, INT> embedded_hal_async::digital::Wait for Mcp23017InterruptPin<'_, M, I2C, D, INT>
where
    M: RawMutex,
    I2C: AsyncI2c,
    D: DelayNs,
    INT: embedded_hal_async::digital::Wait,
{
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        // Enable interrupt for this pin
        {
            let mut driver = self.driver.lock().await;
            driver.set_pin_interrupt(self.pin_number, InterruptMode::OnChange).await?;
        }

        // Wait for interrupt
        self.interrupt_pin.wait_for_high().await.map_err(|_| Error::InitializationFailed)?;

        // Check if this specific pin caused the interrupt
        let mut driver = self.driver.lock().await;
        let flags = driver.read_interrupt_flags().await?;
        if (flags & (1 << self.pin_number)) != 0 {
            // Clear interrupt by reading capture register
            let _ = driver.read_interrupt_capture().await?;
            // Verify pin is actually high
            let is_high = driver.read_pin(self.pin_number).await?;
            if is_high {
                Ok(())
            } else {
                // Pin changed but went low, wait again
                self.wait_for_high().await
            }
        } else {
            // Not our interrupt, wait again
            self.wait_for_high().await
        }
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        // Enable interrupt for this pin
        {
            let mut driver = self.driver.lock().await;
            driver.set_pin_interrupt(self.pin_number, InterruptMode::OnChange).await?;
        }

        // Wait for interrupt
        self.interrupt_pin.wait_for_low().await.map_err(|_| Error::InitializationFailed)?;

        // Check if this specific pin caused the interrupt
        let mut driver = self.driver.lock().await;
        let flags = driver.read_interrupt_flags().await?;
        if (flags & (1 << self.pin_number)) != 0 {
            // Clear interrupt by reading capture register
            let _ = driver.read_interrupt_capture().await?;
            // Verify pin is actually low
            let is_high = driver.read_pin(self.pin_number).await?;
            if !is_high {
                Ok(())
            } else {
                // Pin changed but went high, wait again
                self.wait_for_low().await
            }
        } else {
            // Not our interrupt, wait again
            self.wait_for_low().await
        }
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_high().await
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_low().await
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        // Enable interrupt for this pin
        {
            let mut driver = self.driver.lock().await;
            driver.set_pin_interrupt(self.pin_number, InterruptMode::OnChange).await?;
        }

        // Wait for interrupt on either edge
        self.interrupt_pin.wait_for_any_edge().await.map_err(|_| Error::InitializationFailed)?;

        // Check if this specific pin caused the interrupt
        let mut driver = self.driver.lock().await;
        let flags = driver.read_interrupt_flags().await?;
        if (flags & (1 << self.pin_number)) != 0 {
            // Clear interrupt by reading capture register
            let _ = driver.read_interrupt_capture().await?;
            Ok(())
        } else {
            // Not our interrupt, wait again
            self.wait_for_any_edge().await
        }
    }
}

#[cfg(feature = "async-digital-pins")]
impl<M, I2C, D, INT> embedded_hal_async_git::digital::Wait for Mcp23017InterruptPin<'_, M, I2C, D, INT>
where
    M: RawMutex,
    I2C: AsyncI2c,
    D: DelayNs,
    INT: embedded_hal_async_git::digital::Wait,
{
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        // Enable interrupt for this pin
        {
            let mut driver = self.driver.lock().await;
            driver.set_pin_interrupt(self.pin_number, InterruptMode::OnChange).await?;
        }

        // Wait for interrupt
        self.interrupt_pin.wait_for_high().await.map_err(|_| Error::InitializationFailed)?;

        // Check if this specific pin caused the interrupt
        let mut driver = self.driver.lock().await;
        let flags = driver.read_interrupt_flags().await?;
        if (flags & (1 << self.pin_number)) != 0 {
            // Clear interrupt by reading capture register
            let _ = driver.read_interrupt_capture().await?;
            // Verify pin is actually high
            let is_high = driver.read_pin(self.pin_number).await?;
            if is_high {
                Ok(())
            } else {
                // Pin changed but went low, wait again
                self.wait_for_high().await
            }
        } else {
            // Not our interrupt, wait again
            self.wait_for_high().await
        }
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        // Enable interrupt for this pin
        {
            let mut driver = self.driver.lock().await;
            driver.set_pin_interrupt(self.pin_number, InterruptMode::OnChange).await?;
        }

        // Wait for interrupt
        self.interrupt_pin.wait_for_low().await.map_err(|_| Error::InitializationFailed)?;

        // Check if this specific pin caused the interrupt
        let mut driver = self.driver.lock().await;
        let flags = driver.read_interrupt_flags().await?;
        if (flags & (1 << self.pin_number)) != 0 {
            // Clear interrupt by reading capture register
            let _ = driver.read_interrupt_capture().await?;
            // Verify pin is actually low
            let is_high = driver.read_pin(self.pin_number).await?;
            if !is_high {
                Ok(())
            } else {
                // Pin changed but went high, wait again
                self.wait_for_low().await
            }
        } else {
            // Not our interrupt, wait again
            self.wait_for_low().await
        }
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_high().await
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_low().await
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        // Enable interrupt for this pin
        {
            let mut driver = self.driver.lock().await;
            driver.set_pin_interrupt(self.pin_number, InterruptMode::OnChange).await?;
        }

        // Wait for interrupt on either edge
        self.interrupt_pin.wait_for_any_edge().await.map_err(|_| Error::InitializationFailed)?;

        // Check if this specific pin caused the interrupt
        let mut driver = self.driver.lock().await;
        let flags = driver.read_interrupt_flags().await?;
        if (flags & (1 << self.pin_number)) != 0 {
            // Clear interrupt by reading capture register
            let _ = driver.read_interrupt_capture().await?;
            Ok(())
        } else {
            // Not our interrupt, wait again
            self.wait_for_any_edge().await
        }
    }
}

// Wait trait in stable 1.0.0 requires embedded_hal::digital::ErrorType
#[cfg(not(feature = "async-digital-pins"))]
impl<M, I2C, D, INT> embedded_hal::digital::ErrorType for Mcp23017InterruptPin<'_, M, I2C, D, INT>
where
    M: RawMutex,
    I2C: AsyncI2c,
{
    type Error = Error<I2C::Error>;
}

#[cfg(feature = "async-digital-pins")]
impl<M, I2C, D, INT> embedded_hal_async_git::digital::ErrorType for Mcp23017InterruptPin<'_, M, I2C, D, INT>
where
    M: RawMutex,
    I2C: AsyncI2c,
{
    type Error = Error<I2C::Error>;
}