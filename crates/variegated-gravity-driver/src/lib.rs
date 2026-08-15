#![no_std]
#![warn(missing_docs)]
#![doc = "Async driver for Gravity I2C weight sensor"]

use core::fmt;
use embedded_hal_async::i2c::I2c;

/// Default I2C address for Gravity sensor
const DEFAULT_ADDRESS: u8 = 0x42;

/// Register addresses for weight data (channels 1-4)
const WEIGHT_CH1: u8 = 0x00;
const WEIGHT_CH2: u8 = 0x01;
const WEIGHT_CH3: u8 = 0x02;
const WEIGHT_CH4: u8 = 0x03;

/// Register addresses for rate-of-change data (channels 1-4)
const RATE_CH1: u8 = 0x10;
const RATE_CH2: u8 = 0x11;
const RATE_CH3: u8 = 0x12;
const RATE_CH4: u8 = 0x13;

/// Register addresses for tare values (channels 1-4)
const TARE_CH1: u8 = 0x20;
const TARE_CH2: u8 = 0x21;
const TARE_CH3: u8 = 0x22;
const TARE_CH4: u8 = 0x23;

/// Register addresses for channel configuration (channels 1-4)
const CONFIG_CH1: u8 = 0x30;
const CONFIG_CH2: u8 = 0x31;
const CONFIG_CH3: u8 = 0x32;
const CONFIG_CH4: u8 = 0x33;

/// Register addresses for weighing configuration (channels 1-4)
const WEIGHING_CONFIG_CH1: u8 = 0x40;
const WEIGHING_CONFIG_CH2: u8 = 0x41;
const WEIGHING_CONFIG_CH3: u8 = 0x42;
const WEIGHING_CONFIG_CH4: u8 = 0x43;

/// Register addresses for channel status (channels 1-4)
const STATUS_CH1: u8 = 0x50;
const STATUS_CH2: u8 = 0x51;
const STATUS_CH3: u8 = 0x52;
const STATUS_CH4: u8 = 0x53;

/// Register addresses for device information
const CHANNEL_COUNT: u8 = 0x5E;
const GLOBAL_STATUS: u8 = 0x5F;
const DEVICE_ID: u8 = 0xF0;
const FIRMWARE_VERSION: u8 = 0xF1;
const PROTOCOL_VERSION_MAJOR: u8 = 0xFE;
const PROTOCOL_VERSION_MINOR: u8 = 0xFF;

/// Command register addresses
const TARE_CMD: u8 = 0xC0;
const ZERO_CMD: u8 = 0xC1;
const CAL_100G_CMD: u8 = 0xC2;
const RESET_CMD: u8 = 0xCF;

/// Expected device ID
const EXPECTED_DEVICE_ID: u8 = 0x5C;

/// Channel enumeration
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Channel {
    /// Channel 1
    Ch1 = 1,
    /// Channel 2
    Ch2 = 2,
    /// Channel 3
    Ch3 = 3,
    /// Channel 4
    Ch4 = 4,
}

/// ADC input enumeration
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AdcInput {
    /// ADC A1
    A1 = 1,
    /// ADC A2
    A2 = 2,
    /// ADC B1
    B1 = 3,
    /// ADC B2
    B2 = 4,
}

/// Channel configuration structure
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ChannelConfig {
    /// Channel enabled
    pub enabled: bool,
    /// ADC A1 used
    pub adc_a1: bool,
    /// ADC A2 used
    pub adc_a2: bool,
    /// ADC B1 used
    pub adc_b1: bool,
    /// ADC B2 used
    pub adc_b2: bool,
}

/// Weighing configuration structure
#[derive(Debug, Clone, Copy, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct WeighingConfig {
    /// Zero tracking enabled
    pub zero_tracking: bool,
    /// Smoothing enabled (Moving average filter)
    pub smoothing: bool,
}

/// Channel status structure
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ChannelStatus {
    /// Error condition
    pub error: bool,
    /// Motion detected
    pub motion: bool,
    /// Zero weight
    pub zero: bool,
}

/// Global status structure
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GlobalStatus {
    /// System ready
    pub system_ready: bool,
    /// ADC A error
    pub adc_a_error: bool,
    /// ADC B error
    pub adc_b_error: bool,
}

/// Errors that can occur when communicating with the Gravity sensor
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<E> {
    /// I2C communication error
    I2c(E),
    /// Invalid channel number
    InvalidChannel,
    /// Device not compatible
    IncompatibleDevice,
    /// Protocol version mismatch
    ProtocolVersionMismatch,
}

impl<E> From<E> for Error<E> {
    fn from(err: E) -> Self {
        Error::I2c(err)
    }
}

/// Gravity I2C weight sensor driver
pub struct Gravity<I2C: I2c> {
    i2c: I2C,
    address: u8,
}

impl<I2C> Gravity<I2C>
where
    I2C: I2c,
    I2C::Error: fmt::Debug,
{
    /// Create a new Gravity driver instance
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C bus instance
    /// * `address` - Optional I2C address (defaults to 0x42)
    pub fn new(i2c: I2C, address: Option<u8>) -> Self {
        Self {
            i2c,
            address: address.unwrap_or(DEFAULT_ADDRESS),
        }
    }

    /// Get register address for weight data based on channel
    fn weight_register(channel: Channel) -> u8 {
        match channel {
            Channel::Ch1 => WEIGHT_CH1,
            Channel::Ch2 => WEIGHT_CH2,
            Channel::Ch3 => WEIGHT_CH3,
            Channel::Ch4 => WEIGHT_CH4,
        }
    }

    /// Get register address for rate-of-change data based on channel
    fn rate_register(channel: Channel) -> u8 {
        match channel {
            Channel::Ch1 => RATE_CH1,
            Channel::Ch2 => RATE_CH2,
            Channel::Ch3 => RATE_CH3,
            Channel::Ch4 => RATE_CH4,
        }
    }

    /// Get register address for tare data based on channel
    fn tare_register(channel: Channel) -> u8 {
        match channel {
            Channel::Ch1 => TARE_CH1,
            Channel::Ch2 => TARE_CH2,
            Channel::Ch3 => TARE_CH3,
            Channel::Ch4 => TARE_CH4,
        }
    }

    /// Get register address for channel configuration based on channel
    fn config_register(channel: Channel) -> u8 {
        match channel {
            Channel::Ch1 => CONFIG_CH1,
            Channel::Ch2 => CONFIG_CH2,
            Channel::Ch3 => CONFIG_CH3,
            Channel::Ch4 => CONFIG_CH4,
        }
    }

    /// Get register address for weighing configuration based on channel
    fn weighing_config_register(channel: Channel) -> u8 {
        match channel {
            Channel::Ch1 => WEIGHING_CONFIG_CH1,
            Channel::Ch2 => WEIGHING_CONFIG_CH2,
            Channel::Ch3 => WEIGHING_CONFIG_CH3,
            Channel::Ch4 => WEIGHING_CONFIG_CH4,
        }
    }

    /// Get register address for channel status based on channel
    fn status_register(channel: Channel) -> u8 {
        match channel {
            Channel::Ch1 => STATUS_CH1,
            Channel::Ch2 => STATUS_CH2,
            Channel::Ch3 => STATUS_CH3,
            Channel::Ch4 => STATUS_CH4,
        }
    }

    /// Read a 32-bit signed integer from a register (MSB first)
    async fn read_i32(&mut self, register: u8) -> Result<i32, Error<I2C::Error>> {
        let mut buffer = [0u8; 4];
        
        self.i2c
            .write_read(self.address, &[register], &mut buffer)
            .await
            .map_err(Error::I2c)?;
        
        // Convert 4 bytes to i32 (MSB first as per protocol)
        Ok(i32::from_be_bytes(buffer))
    }

    /// Read a 32-bit unsigned integer from a register (MSB first)
    async fn read_u32(&mut self, register: u8) -> Result<u32, Error<I2C::Error>> {
        let mut buffer = [0u8; 4];

        self.i2c
            .write_read(self.address, &[register], &mut buffer)
            .await
            .map_err(Error::I2c)?;

        // Convert 4 bytes to i32 (MSB first as per protocol)
        Ok(u32::from_be_bytes(buffer))
    }
    
    /// Write a 32-bit signed integer to a register (MSB first)
    async fn write_i32(&mut self, register: u8, value: i32) -> Result<(), Error<I2C::Error>> {
        let bytes = value.to_be_bytes();
        
        self.i2c
            .write(self.address, &[register, bytes[0], bytes[1], bytes[2], bytes[3]])
            .await
            .map_err(Error::I2c)
    }

    async fn write_u32(&mut self, register: u8, value: u32) -> Result<(), Error<I2C::Error>> {
        let bytes = value.to_be_bytes();

        self.i2c
            .write(self.address, &[register, bytes[0], bytes[1], bytes[2], bytes[3]])
            .await
            .map_err(Error::I2c)
    }
    
    /// Read a single byte from a register
    async fn read_u8(&mut self, register: u8) -> Result<u8, Error<I2C::Error>> {
        let mut buffer = [0u8; 1];
        
        self.i2c
            .write_read(self.address, &[register], &mut buffer)
            .await
            .map_err(Error::I2c)?;
        
        Ok(buffer[0])
    }

    /// Write a single byte to a register
    async fn write_u8(&mut self, register: u8, value: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(self.address, &[register, value])
            .await
            .map_err(Error::I2c)
    }

    /// Read weight value from the specified channel
    /// 
    /// Returns weight in milligrams as a signed 32-bit integer
    pub async fn read_weight(&mut self, channel: Channel) -> Result<i32, Error<I2C::Error>> {
        let register = Self::weight_register(channel);
        self.read_i32(register).await
    }

    /// Read rate-of-change value from the specified channel
    /// 
    /// Returns rate of change in milligrams per second as a signed 32-bit integer
    pub async fn read_rate_of_change(&mut self, channel: Channel) -> Result<i32, Error<I2C::Error>> {
        let register = Self::rate_register(channel);
        self.read_i32(register).await
    }

    /// Read tare value from the specified channel
    /// 
    /// Returns tare offset as raw ADC value (signed 32-bit integer)
    pub async fn read_tare(&mut self, channel: Channel) -> Result<i32, Error<I2C::Error>> {
        let register = Self::tare_register(channel);
        self.read_i32(register).await
    }

    /// Write tare value to the specified channel
    /// 
    /// Sets tare offset to the specified raw ADC value
    pub async fn write_tare(&mut self, channel: Channel, tare_value: i32) -> Result<(), Error<I2C::Error>> {
        let register = Self::tare_register(channel);
        self.write_i32(register, tare_value).await
    }

    /// Execute tare command on specified channels
    /// 
    /// # Arguments
    /// * `channels` - Array of channels to tare [Ch1, Ch2, Ch3, Ch4]
    pub async fn execute_tare(&mut self, channels: [bool; 4]) -> Result<(), Error<I2C::Error>> {
        let mut cmd = 0u8;
        if channels[0] { cmd |= 0x01; } // Ch1
        if channels[1] { cmd |= 0x02; } // Ch2
        if channels[2] { cmd |= 0x04; } // Ch3
        if channels[3] { cmd |= 0x08; } // Ch4
        
        self.write_u8(TARE_CMD, cmd).await
    }
    
    pub async fn execute_tare_single(&mut self, channel: Channel) -> Result<(), Error<I2C::Error>> {
        let mut channels = [false; 4];
        channels[channel as usize - 1] = true;
        self.execute_tare(channels).await
    }

    /// Execute zero calibration command on specified channels
    /// 
    /// # Arguments
    /// * `channels` - Array of channels to zero calibrate [Ch1, Ch2, Ch3, Ch4]
    pub async fn execute_zero_calibration(&mut self, channels: [bool; 4]) -> Result<(), Error<I2C::Error>> {
        let mut cmd = 0u8;
        if channels[0] { cmd |= 0x01; } // Ch1
        if channels[1] { cmd |= 0x02; } // Ch2
        if channels[2] { cmd |= 0x04; } // Ch3
        if channels[3] { cmd |= 0x08; } // Ch4
        
        self.write_u8(ZERO_CMD, cmd).await
    }

    /// Execute 100g calibration command on specified channels
    /// 
    /// # Arguments
    /// * `channels` - Array of channels to calibrate [Ch1, Ch2, Ch3, Ch4]
    pub async fn execute_100g_calibration(&mut self, channels: [bool; 4]) -> Result<(), Error<I2C::Error>> {
        let mut cmd = 0u8;
        if channels[0] { cmd |= 0x01; } // Ch1
        if channels[1] { cmd |= 0x02; } // Ch2
        if channels[2] { cmd |= 0x04; } // Ch3
        if channels[3] { cmd |= 0x08; } // Ch4
        
        self.write_u8(CAL_100G_CMD, cmd).await
    }

    /// Reset the device
    pub async fn reset_device(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write_u8(RESET_CMD, 0x01).await
    }

    /// Read channel configuration
    pub async fn read_channel_config(&mut self, channel: Channel) -> Result<ChannelConfig, Error<I2C::Error>> {
        let register = Self::config_register(channel);
        let value = self.read_u8(register).await?;
        
        Ok(ChannelConfig {
            enabled: (value & 0x01) != 0,
            adc_a1: (value & 0x02) != 0,
            adc_a2: (value & 0x04) != 0,
            adc_b1: (value & 0x08) != 0,
            adc_b2: (value & 0x10) != 0,
        })
    }

    /// Write channel configuration
    pub async fn write_channel_config(&mut self, channel: Channel, config: ChannelConfig) -> Result<(), Error<I2C::Error>> {
        let register = Self::config_register(channel);
        let mut value = 0u8;
        
        if config.enabled { value |= 0x01; }
        if config.adc_a1 { value |= 0x02; }
        if config.adc_a2 { value |= 0x04; }
        if config.adc_b1 { value |= 0x08; }
        if config.adc_b2 { value |= 0x10; }
        
        self.write_u8(register, value).await
    }

    /// Read weighing configuration
    pub async fn read_weighing_config(&mut self, channel: Channel) -> Result<WeighingConfig, Error<I2C::Error>> {
        let register = Self::weighing_config_register(channel);
        let value = self.read_u32(register).await?;
        
        Ok(WeighingConfig {
            zero_tracking: (value & 0x01) != 0,
            smoothing: (value & 0x02) != 0,
        })
    }

    /// Write weighing configuration
    pub async fn write_weighing_config(&mut self, channel: Channel, config: WeighingConfig) -> Result<(), Error<I2C::Error>> {
        let register = Self::weighing_config_register(channel);
        let mut value = 0u32;
        
        if config.zero_tracking { value |= 0x01; }
        if config.smoothing { value |= 0x02; }
        
        self.write_u32(register, value).await
    }
    
    /// Read channel status
    pub async fn read_channel_status(&mut self, channel: Channel) -> Result<ChannelStatus, Error<I2C::Error>> {
        let register = Self::status_register(channel);
        let value = self.read_u8(register).await?;
        
        Ok(ChannelStatus {
            error: (value & 0x01) != 0,
            motion: (value & 0x02) != 0,
            zero: (value & 0x04) != 0,
        })
    }

    /// Read global status
    pub async fn read_global_status(&mut self) -> Result<GlobalStatus, Error<I2C::Error>> {
        let value = self.read_u8(GLOBAL_STATUS).await?;
        
        Ok(GlobalStatus {
            system_ready: (value & 0x01) != 0,
            adc_a_error: (value & 0x02) != 0,
            adc_b_error: (value & 0x04) != 0,
        })
    }

    /// Read number of active channels
    pub async fn read_channel_count(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_u8(CHANNEL_COUNT).await
    }

    /// Read status for channel 1 (legacy method)
    pub async fn read_status(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_u8(STATUS_CH1).await
    }

    /// Read firmware version
    pub async fn read_firmware_version(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_u8(FIRMWARE_VERSION).await
    }
    
    /// Read protocol version (major, minor)
    pub async fn read_protocol_version(&mut self) -> Result<(u8, u8), Error<I2C::Error>> {
        let major = self.read_u8(PROTOCOL_VERSION_MAJOR).await?;
        let minor = self.read_u8(PROTOCOL_VERSION_MINOR).await?;
        
        Ok((major, minor))
    }
    
    /// Read device ID
    pub async fn read_device_id(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_u8(DEVICE_ID).await
    }
    
    /// Check device compatibility per protocol initialization procedure
    pub async fn check_compatibility(&mut self) -> Result<(), Error<I2C::Error>> {
        let (major, _minor) = self.read_protocol_version().await?;
        
        // Check protocol version (major must be 1 for this driver)
        if major != 1 {
            return Err(Error::ProtocolVersionMismatch);
        }
        
        // Check device ID
        let device_id = self.read_device_id().await?;
        if device_id != EXPECTED_DEVICE_ID {
            return Err(Error::IncompatibleDevice);
        }
        
        Ok(())
    }

    /// Read all weight values from all channels
    /// 
    /// Returns an array of weights in milligrams [Ch1, Ch2, Ch3, Ch4]
    pub async fn read_all_weights(&mut self) -> Result<[i32; 4], Error<I2C::Error>> {
        let ch1 = self.read_weight(Channel::Ch1).await?;
        let ch2 = self.read_weight(Channel::Ch2).await?;
        let ch3 = self.read_weight(Channel::Ch3).await?;
        let ch4 = self.read_weight(Channel::Ch4).await?;
        
        Ok([ch1, ch2, ch3, ch4])
    }

    /// Read all rate-of-change values from all channels
    /// 
    /// Returns an array of rates in mg/s [Ch1, Ch2, Ch3, Ch4]
    pub async fn read_all_rates(&mut self) -> Result<[i32; 4], Error<I2C::Error>> {
        let ch1 = self.read_rate_of_change(Channel::Ch1).await?;
        let ch2 = self.read_rate_of_change(Channel::Ch2).await?;
        let ch3 = self.read_rate_of_change(Channel::Ch3).await?;
        let ch4 = self.read_rate_of_change(Channel::Ch4).await?;
        
        Ok([ch1, ch2, ch3, ch4])
    }

    /// Tare all channels
    pub async fn tare_all_channels(&mut self) -> Result<(), Error<I2C::Error>> {
        self.execute_tare([true, true, true, true]).await
    }

    /// Tare a single channel
    pub async fn tare_channel(&mut self, channel: Channel) -> Result<(), Error<I2C::Error>> {
        let mut channels = [false, false, false, false];
        channels[channel as usize - 1] = true;
        self.execute_tare(channels).await
    }

    /// Initialize the device (perform compatibility check)
    pub async fn initialize(&mut self) -> Result<(), Error<I2C::Error>> {
        self.check_compatibility().await
    }
}