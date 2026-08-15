//! # NAU7802 Embassy Driver
//!
//! Embassy-based driver for the Nuvoton NAU7802 24-bit precision ADC.
//! This driver provides async support for I2C communication with the NAU7802,
//! designed specifically for load cell and weight measurement applications.
//!
//! The NAU7802 is a precision, 24-bit, analog-to-digital converter (ADC) with
//! programmable gain amplifier (PGA), voltage reference, and calibration capabilities.
//! It is particularly well-suited for bridge sensor applications like load cells.
//!
//! ## Usage Example
//!
//! ```no_run
//! use variegated_nau7802::{Nau7802, Nau7802DataAvailableStrategy, Gain, SamplesPerSecond, Ldo};
//! 
//! # async fn example() -> Result<(), Box<dyn std::error::Error>> {
//! # let i2c = todo!();
//! # let delay = todo!();
//! // Create the driver with I2C bus and delay provider
//! let data_strategy = Nau7802DataAvailableStrategy::Polling;
//! let mut nau = Nau7802::new(i2c, data_strategy, delay, Some(0x2A)); // Default I2C address
//!
//! // Initialize the device with high-level init function
//! nau.init(Ldo::L3v3, Gain::G128, SamplesPerSecond::SPS80).await?;
//!
//! // Read weight measurement
//! nau.wait_for_data_available().await?;
//! let reading = nau.read().await?;
//! // Convert to weight using your calibration factor
//! let weight_grams = (reading as f32) * 0.001; // Example calibration
//! }
//! # Ok(())
//! # }
//! ```
//!
//! Based on [amiraeva/nau7802-rs](https://github.com/amiraeva/nau7802-rs).
//! See the datasheet in `docs/datasheet.pdf` for complete technical specifications.

#![no_std]
#![warn(missing_docs)]

use byteorder::ByteOrder as _;
use core::{fmt, slice};
use embedded_hal_async::digital::Wait;
use embedded_hal_async::i2c::I2c;
use embedded_hal_async::delay::DelayNs;

mod constants;
pub use constants::*;

/// Result type alias for NAU7802 operations
pub type Result<T> = core::result::Result<T, Error>;

/// Errors that can occur when communicating with the NAU7802
#[derive(Debug)]
pub enum Error {
    /// I2C communication error
    I2cError,
    /// Device power-up sequence failed
    PowerupFailed,
}

/// Strategy for detecting when new ADC data is available
pub enum Nau7802DataAvailableStrategy<W: Wait>{
    /// Use polling to check for data ready
    Polling,
    /// Use the dedicated DRDY pin to detect when data is ready
    DrdyPin(W),
}

/// NAU7802 ADC driver with Embassy async support
pub struct Nau7802<I2C: I2c, W: Wait, D: DelayNs> {
    i2c: I2C,
    address: u8,
    wait_strategy: Nau7802DataAvailableStrategy<W>,
    delay: D
}

impl<I2C: I2c, W: Wait, D: DelayNs> Nau7802<I2C, W, D> 
where 
    I2C::Error: fmt::Debug,
{
    /// Create a new NAU7802 driver instance
    pub fn new(i2c: I2C, wait_strategy: Nau7802DataAvailableStrategy<W>, delay: D, address: Option<u8>) -> Self {
        let addr = if let Some(a) = address {
            a
        } else {
            0x2A
        };

        Self {
            i2c,
            wait_strategy,
            delay,
            address: addr
        }
    }

    /// Initialize the NAU7802 with the specified configuration
    pub async fn init(
        &mut self,
        ldo: Ldo,
        gain: Gain,
        sps: SamplesPerSecond,
    ) -> Result<()> {
        self.start_reset().await?;
        self.finish_reset().await?;
        self.power_up().await?;
        self.set_ldo(ldo).await?;
        self.set_gain(gain).await?;
        self.set_sample_rate(sps).await?;
        self.misc_init().await?;
        self.begin_afe_calibration().await?;

        while self.poll_afe_calibration_status().await? != AfeCalibrationStatus::Success {}

        Ok(())
    }

    /// Wait for new ADC data to become available
    pub async fn wait_for_data_available(&mut self) -> Result<()> {
        match self.wait_strategy {
            Nau7802DataAvailableStrategy::Polling => self.wait_for_data_available_i2c().await?,
            Nau7802DataAvailableStrategy::DrdyPin(ref mut w) => w.wait_for_high().await.map_err(|_| Error::I2cError)?
        }
        
        Ok(())
    }

    async fn wait_for_data_available_i2c(&mut self) -> Result<()> {
        loop {
            if self.get_bit(Register::PuCtrl, PuCtrlBits::CR).await? {
                return Ok(());
            }
            self.delay.delay_ms(1).await;
        }
    }
    
    /// Read a 24-bit signed conversion result
    pub async fn read(&mut self) -> Result<i32> {
        self.wait_for_data_available().await?;

        self.request_register(Register::AdcoB2).await?;

        let mut buf = [0u8; 3]; // will hold an i24
        self.i2c
            .read(self.address, &mut buf)
            .await
            .map_err(|_| Error::I2cError)?;

        let adc_result = byteorder::BigEndian::read_i24(&buf);
        Ok(adc_result)
    }

    /// Begin analog front-end calibration
    pub async fn begin_afe_calibration(&mut self) -> Result<()> {
        self.set_bit(Register::Ctrl2, Ctrl2RegisterBits::Cals).await
    }

    /// Check the status of analog front-end calibration
    pub async fn poll_afe_calibration_status(&mut self) -> Result<AfeCalibrationStatus> {
        if self.get_bit(Register::Ctrl2, Ctrl2RegisterBits::Cals).await? {
            return Ok(AfeCalibrationStatus::InProgress);
        }

        if self.get_bit(Register::Ctrl2, Ctrl2RegisterBits::CalError).await? {
            return Ok(AfeCalibrationStatus::Failure);
        }

        Ok(AfeCalibrationStatus::Success)
    }

    /// Set the ADC sample rate
    pub async fn set_sample_rate(&mut self, sps: SamplesPerSecond) -> Result<()> {
        const SPS_MASK: u8 = 0b10001111;
        const SPS_START_BIT_IDX: u8 = 4;

        self.set_function_helper(Register::Ctrl2, SPS_MASK, SPS_START_BIT_IDX, sps as _).await
    }

    /// Set the programmable gain amplifier gain
    pub async fn set_gain(&mut self, gain: Gain) -> Result<()> {
        const GAIN_MASK: u8 = 0b11111000;
        const GAIN_START_BIT: u8 = 0;

        self.set_function_helper(Register::Ctrl1, GAIN_MASK, GAIN_START_BIT, gain as _).await
    }

    /// Set the low-dropout regulator voltage
    pub async fn set_ldo(&mut self, ldo: Ldo) -> Result<()> {
        const LDO_MASK: u8 = 0b11000111;
        const LDO_START_BIT: u8 = 3;

        self.set_function_helper(Register::Ctrl1, LDO_MASK, LDO_START_BIT, ldo as _).await?;

        self.set_bit(Register::PuCtrl, PuCtrlBits::AVDDS).await
    }

    /// Power up the analog and digital circuitry
    pub async fn power_up(&mut self) -> Result<()> {
        const NUM_ATTEMPTS: usize = 100;

        self.set_bit(Register::PuCtrl, PuCtrlBits::PUD).await?;
        self.set_bit(Register::PuCtrl, PuCtrlBits::PUA).await?;

        let mut powered_up = false;
        let mut attempts = 0;
        while !powered_up && attempts < NUM_ATTEMPTS {
            let res = self.get_bit(Register::PuCtrl, PuCtrlBits::PUR).await;
            if let Ok(rdy) = res {
                powered_up = rdy;
            }

            attempts += 1;
        }
        
        if powered_up {
            Ok(())
        } else {
            Err(Error::PowerupFailed)
        }
    }

    /// Start the device reset sequence
    pub async fn start_reset(&mut self) -> Result<()> {
        self.set_bit(Register::PuCtrl, PuCtrlBits::RR).await
    }

    /// Finish the device reset sequence
    pub async fn finish_reset(&mut self) -> Result<()> {
        self.clear_bit(Register::PuCtrl, PuCtrlBits::RR).await
    }


    /// Perform miscellaneous initialization steps
    pub async fn misc_init(&mut self) -> Result<()> {
        const TURN_OFF_CLK_CHPL: u8 = 0x30;

        // Turn off CLK_CHP. From 9.1 power on sequencing
        self.set_register(Register::Adc, TURN_OFF_CLK_CHPL).await?;

        // Enable 330pF decoupling cap on chan 2. From 9.14 application circuit note
        self.set_bit(Register::PgaPwr, PgaPwrRegisterBits::CapEn).await
    }

    async fn set_function_helper(
        &mut self,
        reg: Register,
        mask: u8,
        start_idx: u8,
        new_val: u8,
    ) -> Result<()> {
        let mut val = self.get_register(reg).await?;
        val &= mask;
        val |= new_val << start_idx;

        self.set_register(reg, val).await
    }

    async fn set_bit<B: RegisterBits>(&mut self, addr: Register, bit_idx: B) -> Result<()> {
        let mut val = self.get_register(addr).await?;
        val |= 1 << bit_idx.get();
        self.set_register(addr, val).await
    }

    async fn clear_bit<B: RegisterBits>(&mut self, addr: Register, bit_idx: B) -> Result<()> {
        let mut val = self.get_register(addr).await?;
        val &= !(1 << bit_idx.get());
        self.set_register(addr, val).await
    }

    async fn get_bit<B: RegisterBits>(&mut self, addr: Register, bit_idx: B) -> Result<bool> {
        let mut val = self.get_register(addr).await?;
        val &= 1 << bit_idx.get();
        Ok(val != 0)
    }

    async fn set_register(&mut self, reg: Register, val: u8) -> Result<()> {
        let transaction = [reg as _, val];

        self.i2c
            .write(self.address, &transaction)
            .await
            .map_err(|_| Error::I2cError)
    }

    async fn get_register(&mut self, reg: Register) -> Result<u8> {
        self.request_register(reg).await?;

        let mut val = 0;
        self.i2c
            .read(self.address, slice::from_mut(&mut val))
            .await
            .map_err(|_| Error::I2cError)?;

        Ok(val)
    }

    async fn request_register(&mut self, reg: Register) -> Result<()> {
        let reg = reg as u8;

        self.i2c
            .write(self.address, slice::from_ref(&reg))
            .await
            .map_err(|_| Error::I2cError)
    }
}