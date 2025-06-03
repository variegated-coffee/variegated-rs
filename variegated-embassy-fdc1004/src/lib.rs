#![no_std]
#![doc = include_str!("../README.md")]
#![warn(missing_docs)]

use core::fmt;
use defmt::Format;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;
use ux::i24;

/// Errors that can occur when communicating with the FDC1004 chip.
#[derive(Debug, Clone, Copy, Format)]
pub enum FDC1004Error<E>{
    /// Failed to find a suitable CAPDAC setting for the measurement range.
    UnableToFindCapdacSetting,
    /// Measurement has not completed yet.
    MeasurementNotComplete,
    /// Invalid channel for single-ended measurement.
    InvalidMeasurementChannel,
    /// I2C communication error.
    I2CError(E)
}

/// Output data rate configuration for FDC1004 measurements.
/// 
/// The FDC1004 supports configurable sample rates for capacitance measurements.
/// Higher rates provide faster measurements but may have reduced accuracy.
#[derive(Default, Copy, Clone, Debug)]
pub enum OutputRate {
    /// 100 samples per second (default).
    #[default]
    SPS100,
    /// 200 samples per second.
    SPS200,
    /// 400 samples per second.
    SPS400,
}

impl OutputRate {
    fn delay_ns(&self) -> u32 {
        match self {
            OutputRate::SPS100 => 11_000_000, // 11 milliseconds in nanoseconds
            OutputRate::SPS200 => 6_000_000,  // 6 milliseconds in nanoseconds
            OutputRate::SPS400 => 3_000_000,  // 3 milliseconds in nanoseconds
        }
    }
}

static CAPDAC_MAX: u8 = 0x1F;

/// FDC1004 input channel selection.
/// 
/// The FDC1004 has 4 capacitive input channels (CIN1-CIN4) that can be 
/// configured for single-ended or differential measurements.
#[derive(Copy, Clone, Debug)]
pub enum Channel {
    /// Capacitive input channel 1.
    CIN1,
    /// Capacitive input channel 2.
    CIN2,
    /// Capacitive input channel 3.
    CIN3,
    /// Capacitive input channel 4.
    CIN4,
    /// Internal CAPDAC reference for offset compensation.
    CAPDAC,
    /// Disabled channel (no connection).
    DISABLED
}

/// FDC1004 measurement channel identifier.
/// 
/// The FDC1004 supports up to 4 simultaneous measurements using different
/// measurement configurations. Each measurement can be independently configured
/// with different input channels and CAPDAC settings.
#[derive(Copy, Clone)]
pub enum Measurement {
    /// Measurement channel 1.
    Measurement1,
    /// Measurement channel 2.
    Measurement2,
    /// Measurement channel 3.
    Measurement3,
    /// Measurement channel 4.
    Measurement4,
}

impl Measurement {
    fn msb_register(&self) -> RegisterAddress {
        match self {
            Measurement::Measurement1 => RegisterAddress::Measurement1MSB,
            Measurement::Measurement2 => RegisterAddress::Measurement2MSB,
            Measurement::Measurement3 => RegisterAddress::Measurement3MSB,
            Measurement::Measurement4 => RegisterAddress::Measurement4MSB,
        }
    }

    fn lsb_register(&self) -> RegisterAddress {
        match self {
            Measurement::Measurement1 => RegisterAddress::Measurement1LSB,
            Measurement::Measurement2 => RegisterAddress::Measurement2LSB,
            Measurement::Measurement3 => RegisterAddress::Measurement3LSB,
            Measurement::Measurement4 => RegisterAddress::Measurement4LSB,
        }
    }

    fn config_register(&self) -> RegisterAddress {
        match self {
            Measurement::Measurement1 => RegisterAddress::Measurement1Config,
            Measurement::Measurement2 => RegisterAddress::Measurement2Config,
            Measurement::Measurement3 => RegisterAddress::Measurement3Config,
            Measurement::Measurement4 => RegisterAddress::Measurement4Config,
        }
    }

    fn ready_according_to_config(&self, config: &FDCConfiguration) -> bool {
        match self {
            Measurement::Measurement1 => config.measurement1_done,
            Measurement::Measurement2 => config.measurement2_done,
            Measurement::Measurement3 => config.measurement3_done,
            Measurement::Measurement4 => config.measurement4_done,
        }
    }
}

/// FDC1004 register addresses.
/// 
/// These addresses correspond to the internal registers of the FDC1004 chip
/// as specified in the datasheet. Each register serves a specific function
/// for configuration, measurement data, calibration, or device identification.
#[derive(Copy, Clone, Debug)]
pub enum RegisterAddress {
    /// Measurement 1 most significant byte.
    Measurement1MSB,
    /// Measurement 1 least significant byte.
    Measurement1LSB,
    /// Measurement 2 most significant byte.
    Measurement2MSB,
    /// Measurement 2 least significant byte.
    Measurement2LSB,
    /// Measurement 3 most significant byte.
    Measurement3MSB,
    /// Measurement 3 least significant byte.
    Measurement3LSB,
    /// Measurement 4 most significant byte.
    Measurement4MSB,
    /// Measurement 4 least significant byte.
    Measurement4LSB,
    /// Measurement 1 configuration register.
    Measurement1Config,
    /// Measurement 2 configuration register.
    Measurement2Config,
    /// Measurement 3 configuration register.
    Measurement3Config,
    /// Measurement 4 configuration register.
    Measurement4Config,
    /// FDC configuration register.
    FdcConf,
    /// Offset calibration for CIN1.
    OffsetCalCIN1,
    /// Offset calibration for CIN2.
    OffsetCalCIN2,
    /// Offset calibration for CIN3.
    OffsetCalCIN3,
    /// Offset calibration for CIN4.
    OffsetCalCIN4,
    /// Gain calibration for CIN1.
    GainCalCIN1,
    /// Gain calibration for CIN2.
    GainCalCIN2,
    /// Gain calibration for CIN3.
    GainCalCIN3,
    /// Gain calibration for CIN4.
    GainCalCIN4,
    /// Manufacturer ID register.
    ManufacturerId,
    /// Device ID register.
    DeviceId,
}

impl RegisterAddress {
    pub(crate) fn to_u8(&self) -> u8 {
        match self {
            RegisterAddress::Measurement1MSB => 0x00,
            RegisterAddress::Measurement1LSB => 0x01,
            RegisterAddress::Measurement2MSB => 0x02,
            RegisterAddress::Measurement2LSB => 0x03,
            RegisterAddress::Measurement3MSB => 0x04,
            RegisterAddress::Measurement3LSB => 0x05,
            RegisterAddress::Measurement4MSB => 0x06,
            RegisterAddress::Measurement4LSB => 0x07,
            RegisterAddress::Measurement1Config => 0x08,
            RegisterAddress::Measurement2Config => 0x09,
            RegisterAddress::Measurement3Config => 0x0A,
            RegisterAddress::Measurement4Config => 0x0B,
            RegisterAddress::FdcConf => 0x0C,
            RegisterAddress::OffsetCalCIN1 => 0x0D,
            RegisterAddress::OffsetCalCIN2 => 0x0E,
            RegisterAddress::OffsetCalCIN3 => 0x0F,
            RegisterAddress::OffsetCalCIN4 => 0x10,
            RegisterAddress::GainCalCIN1 => 0x11,
            RegisterAddress::GainCalCIN2 => 0x12,
            RegisterAddress::GainCalCIN3 => 0x13,
            RegisterAddress::GainCalCIN4 => 0x14,
            RegisterAddress::ManufacturerId => 0xFE,
            RegisterAddress::DeviceId => 0xFF,
        }
    }
}

static PICOFARADS_PER_CAPDAC: f32 = 3.125;

/// Result of a capacitance measurement operation.
/// 
/// The FDC1004 can measure capacitances within a certain range. If the measured
/// capacitance is outside this range, the result will indicate overflow or underflow.
#[derive(Debug, Clone, Copy, Format)]
pub enum SuccessfulMeasurement {
    /// Measurement completed successfully and is within the measurable range.
    MeasurementInRange(MeasuredCapacitance),
    /// Capacitance is too small to measure accurately (below measurement range).
    Underflow,
    /// Capacitance is too large to measure accurately (above measurement range).
    Overflow,
}

/// A capacitance measurement result with associated CAPDAC offset.
/// 
/// Contains the raw measurement value from the FDC1004 along with the 
/// CAPDAC setting used during measurement. The CAPDAC provides offset
/// compensation to extend the measurement range.
#[derive(Debug, Clone, Copy)]
pub struct MeasuredCapacitance {
    pub(crate) value: i24,
    pub(crate) capdac: u8,
}

impl Format for MeasuredCapacitance {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "MeasuredCapacitance {} pF", self.to_pf());
    }
}

impl MeasuredCapacitance {
    pub(crate) fn new(value: i24, capdac: u8) -> Self {
        MeasuredCapacitance {
            value,
            capdac,
        }
    }

    pub(crate) fn to_pf(&self) -> f32 {
        let vali32 : i32 = self.value.into();
        let val: f32 = vali32 as f32;

        let mut pf = val / 524_288f32;

        pf += PICOFARADS_PER_CAPDAC * (self.capdac as f32);
        pf
    }
}

#[derive(Debug)]
struct MeasurementConfiguration {
    channel_a: Channel,
    channel_b: Channel,
    offset_capacitance: u8,
}

impl MeasurementConfiguration {
    pub(crate) fn new(channel_a: Channel, channel_b: Channel, offset_capacitance: u8) -> Self {
        MeasurementConfiguration {
            channel_a,
            channel_b,
            offset_capacitance,
        }
    }

    pub(crate) fn to_u16(&self) -> u16 {
        let mut val = 0;

        val |= match self.channel_a {
            Channel::CIN1 => 0x0000,
            Channel::CIN2 => 0x2000,
            Channel::CIN3 => 0x4000,
            Channel::CIN4 => 0x6000,
            _ => 0x0000,
        };

        val |= match self.channel_b {
            Channel::CIN1 => 0x0000,
            Channel::CIN2 => 0x0400,
            Channel::CIN3 => 0x0800,
            Channel::CIN4 => 0x0C00,
            Channel::CAPDAC => 0x1000,
            Channel::DISABLED => 0x1C00,
        };

        val |= (self.offset_capacitance as u16) << 5;

        val
    }
}

#[derive(Default, Debug)]
struct FDCConfiguration {
    reset: bool,
    rate: OutputRate,
    repeat: bool,
    initiate_measurement1: bool,
    initiate_measurement2: bool,
    initiate_measurement3: bool,
    initiate_measurement4: bool,
    measurement1_done: bool,
    measurement2_done: bool,
    measurement3_done: bool,
    measurement4_done: bool,
}

impl FDCConfiguration {
    pub(crate) fn from_u16(d: u16) -> Self {
        let mut config = FDCConfiguration::default();

        config.reset = d & (1u16 << 15) != 0;
        config.rate = match d & 0x0C00 {
            0x0400 => OutputRate::SPS100,
            0x0800 => OutputRate::SPS200,
            0x0C00 => OutputRate::SPS400,
            _ => OutputRate::SPS100,
        };
        config.repeat = d & (1u16 << 8) != 0;
        config.initiate_measurement1 = d & (1u16 << 7) != 0;
        config.initiate_measurement2 = d & (1u16 << 6) != 0;
        config.initiate_measurement3 = d & (1u16 << 5) != 0;
        config.initiate_measurement4 = d & (1u16 << 4) != 0;
        config.measurement1_done = d & (1u16 << 3) != 0;
        config.measurement2_done = d & (1u16 << 2) != 0;
        config.measurement3_done = d & (1u16 << 1) != 0;
        config.measurement4_done = d & (1u16 << 0) != 0;

        config
    }

    pub(crate) fn rate(&mut self, rate: OutputRate) -> &mut Self {
        self.rate = rate;
        self
    }

    pub(crate) fn initiate_measurement1(&mut self, initiate: bool) -> &mut Self {
        self.initiate_measurement1 = initiate;
        self
    }

    pub(crate) fn initiate_measurement2(&mut self, initiate: bool) -> &mut Self {
        self.initiate_measurement2 = initiate;
        self
    }

    pub(crate) fn initiate_measurement3(&mut self, initiate: bool) -> &mut Self {
        self.initiate_measurement3 = initiate;
        self
    }

    pub(crate) fn initiate_measurement4(&mut self, initiate: bool) -> &mut Self {
        self.initiate_measurement4 = initiate;
        self
    }

    #[allow(unused)]
    pub(crate) fn reset(&mut self, reset: bool) -> &mut Self {
        self.reset = reset;
        self
    }

    #[allow(unused)]
    pub(crate) fn repeat(&mut self, repeat: bool) -> &mut Self {
        self.repeat = repeat;
        self
    }

    pub(crate) fn to_u16(&self) -> u16 {
        let mut val = 0;

        val |= if self.reset { 1u16 << 15 } else { 0x0000 };
        val |= match self.rate {
            OutputRate::SPS100 => 0x0400,
            OutputRate::SPS200 => 0x0800,
            OutputRate::SPS400 => 0x0C00,
        };
        val |= if self.repeat { 1u16 << 8 } else { 0x0000 };
        val |= if self.initiate_measurement1 { 1u16 << 7 } else { 0x0000 };
        val |= if self.initiate_measurement2 { 1u16 << 6 } else { 0x0000 };
        val |= if self.initiate_measurement3 { 1u16 << 5 } else { 0x0000 };
        val |= if self.initiate_measurement4 { 1u16 << 4 } else { 0x0000 };
        val |= if self.measurement1_done { 1u16 << 3 } else { 0x0000 };
        val |= if self.measurement2_done { 1u16 << 2 } else { 0x0000 };
        val |= if self.measurement3_done { 1u16 << 1 } else { 0x0000 };
        val |= if self.measurement4_done { 1u16 << 0 } else { 0x0000 };

        val
    }
}

/// FDC1004 Capacitance-to-Digital Converter driver.
/// 
/// This driver provides an async interface to the Texas Instruments FDC1004
/// 4-channel capacitance-to-digital converter. The FDC1004 can measure
/// capacitances from femtofarads to picofarads with high resolution.
/// 
/// # Examples
/// 
/// ```no_run
/// use variegated_embassy_fdc1004::{FDC1004, OutputRate, Channel};
/// 
/// // Create a new FDC1004 driver instance
/// let mut fdc = FDC1004::new(i2c, 0x50, OutputRate::SPS100, delay);
/// 
/// // Read capacitance from channel 1
/// let result = fdc.read_capacitance(Channel::CIN1).await?;
/// ```
pub struct FDC1004<I2C: I2c, D: DelayNs> {
    i2c: I2C,
    address: u8,
    output_rate: OutputRate,
    delay: D,
}

impl<I2C: I2c, D: DelayNs> FDC1004<I2C, D> 
where 
    I2C::Error: fmt::Debug,
{
    /// Create a new FDC1004 driver instance.
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C peripheral for communication with the FDC1004
    /// * `address` - I2C address of the FDC1004 device (typically 0x50)
    /// * `output_rate` - Desired measurement sample rate
    /// * `delay` - Delay provider for timing operations
    pub fn new(i2c: I2C, address: u8, output_rate: OutputRate, delay: D) -> Self {
        FDC1004 {
            i2c,
            address,
            output_rate,
            delay,
        }
    }

    /// Read capacitance from the specified channel with automatic CAPDAC adjustment.
    /// 
    /// This method automatically adjusts the CAPDAC setting to find the optimal
    /// measurement range for the target capacitor. It will iterate through different
    /// CAPDAC values until a valid measurement is obtained or the limits are reached.
    /// 
    /// # Arguments
    /// 
    /// * `channel` - The input channel to measure (CIN1-CIN4)
    /// 
    /// # Returns
    /// 
    /// * `Ok(SuccessfulMeasurement::MeasurementInRange)` - Valid measurement with capacitance data
    /// * `Ok(SuccessfulMeasurement::Underflow)` - Capacitance below measurable range
    /// * `Ok(SuccessfulMeasurement::Overflow)` - Capacitance above measurable range
    /// * `Err` - Communication or configuration error
    pub async fn read_capacitance(&mut self, channel: Channel) -> Result<SuccessfulMeasurement, FDC1004Error<I2C::Error>> {
        let mut capdac: u8 = 0x00;

        for _ in 0..33 {
            let m = self.measure_channel(channel, capdac).await?;
            if m < i24::max_value() && m > i24::min_value() {
                return Ok(SuccessfulMeasurement::MeasurementInRange(MeasuredCapacitance::new(m, capdac)));
            }

            if m == i24::max_value() && capdac < CAPDAC_MAX {
                capdac += 1;
            } else if m == i24::min_value() && capdac > 0 {
                capdac -= 1;
            } else {
                return match capdac {
                    0 => Ok(SuccessfulMeasurement::Underflow),
                    _ => Ok(SuccessfulMeasurement::Overflow)
                };
            }
        }

        Err(FDC1004Error::UnableToFindCapdacSetting)
    }

    /// Measure capacitance on a specific channel with a given CAPDAC setting.
    /// 
    /// This is a lower-level method that performs a single measurement with
    /// a specific CAPDAC value. Use `read_capacitance` for automatic CAPDAC
    /// adjustment instead.
    /// 
    /// # Arguments
    /// 
    /// * `channel` - The input channel to measure (CIN1-CIN4)
    /// * `capdac` - CAPDAC offset value (0-31) for measurement range adjustment
    /// 
    /// # Returns
    /// 
    /// Raw 24-bit measurement value from the FDC1004.
    pub async fn measure_channel(&mut self, channel: Channel, capdac: u8) -> Result<i24, FDC1004Error<I2C::Error>> {
        // Map input channels to measurement slots, with proper error handling for invalid channels
        let measurement = match channel {
            Channel::CIN1 => Measurement::Measurement1,
            Channel::CIN2 => Measurement::Measurement2,
            Channel::CIN3 => Measurement::Measurement3,
            Channel::CIN4 => Measurement::Measurement4,
            // CAPDAC and DISABLED channels cannot be measured directly
            Channel::CAPDAC | Channel::DISABLED => return Err(FDC1004Error::InvalidMeasurementChannel),
        };

        self.configure_single_measurement(channel, measurement.clone(), capdac).await?;
        self.trigger_single_measurement(measurement.clone()).await?;
        self.delay.delay_ns(self.output_rate.delay_ns()).await;

        return self.read_measurement(measurement).await;
    }

    /// Configure a measurement channel with specific input and CAPDAC settings.
    /// 
    /// Sets up one of the four measurement channels with the specified input
    /// channel and CAPDAC offset value.
    /// 
    /// # Arguments
    /// 
    /// * `channel` - Input channel to connect to this measurement
    /// * `measurement` - Which measurement slot (1-4) to configure
    /// * `capdac` - CAPDAC offset value for measurement range adjustment
    pub async fn configure_single_measurement(&mut self, channel: Channel, measurement: Measurement, capdac: u8) -> Result<(), FDC1004Error<I2C::Error>> {
        let config = MeasurementConfiguration::new(channel, Channel::CAPDAC, capdac);

        self.write_u16(measurement.config_register(), config.to_u16()).await
    }

    /// Trigger a measurement on the specified measurement channel.
    /// 
    /// Initiates a capacitance measurement on one of the pre-configured
    /// measurement channels. The measurement must be configured first using
    /// `configure_single_measurement`.
    /// 
    /// # Arguments
    /// 
    /// * `measurement` - Which measurement slot (1-4) to trigger
    pub async fn trigger_single_measurement(&mut self, measurement: Measurement) -> Result<(), FDC1004Error<I2C::Error>> {
        let mut config = FDCConfiguration::default();
        let config = config.rate(self.output_rate);
        let config = match measurement {
            Measurement::Measurement1 => config.initiate_measurement1(true),
            Measurement::Measurement2 => config.initiate_measurement2(true),
            Measurement::Measurement3 => config.initiate_measurement3(true),
            Measurement::Measurement4 => config.initiate_measurement4(true),
        };

        self.write_u16(RegisterAddress::FdcConf, config.to_u16()).await
    }

    /// Read the result of a completed measurement.
    /// 
    /// Retrieves the measurement data from the specified measurement channel.
    /// The measurement must be completed before calling this method, otherwise
    /// a `MeasurementNotComplete` error will be returned.
    /// 
    /// # Arguments
    /// 
    /// * `measurement` - Which measurement slot (1-4) to read from
    /// 
    /// # Returns
    /// 
    /// Raw 24-bit measurement value from the FDC1004.
    pub async fn read_measurement(&mut self, measurement: Measurement) -> Result<i24, FDC1004Error<I2C::Error>> {
        // Wait for measurement to complete with timeout
        const MAX_WAIT_ATTEMPTS: u8 = 10;
        let wait_delay_ns = self.output_rate.delay_ns();
        
        for _ in 0..MAX_WAIT_ATTEMPTS {
            let config = FDCConfiguration::from_u16(self.read_u16(RegisterAddress::FdcConf).await?);
            
            if measurement.ready_according_to_config(&config) {
                break;
            }
            
            // Wait for one sample period before checking again
            self.delay.delay_ns(wait_delay_ns).await;
        }
        
        // Final check - if still not ready, return error
        let config = FDCConfiguration::from_u16(self.read_u16(RegisterAddress::FdcConf).await?);
        if !measurement.ready_according_to_config(&config) {
            return Err(FDC1004Error::MeasurementNotComplete);
        }

        let msb = self.read_u16(measurement.msb_register()).await? as i32;
        let lsb = self.read_u16(measurement.lsb_register()).await? as i32;

        let mut val24 = i24::default();
        val24 |= i24::new(msb) << 8;
        val24 |= i24::new(lsb) >> 8;

        Ok(val24)
    }

    pub(crate) async fn write_u16(&mut self, reg: RegisterAddress, data: u16) -> Result<(), FDC1004Error<I2C::Error>> {
        let data = data.to_be_bytes();
        self.i2c.write(self.address, &[reg.to_u8(), data[0], data[1]]).await.map_err(|e| FDC1004Error::I2CError(e))
    }

    pub(crate) async fn read_u16(&mut self, reg: RegisterAddress) -> Result<u16, FDC1004Error<I2C::Error>> {
        let mut data: [u8; 2] = [0,0];
        self.i2c.write_read(self.address, &[reg.to_u8()], &mut data).await.map_err(|e| FDC1004Error::I2CError(e))?;

        let be = u16::from_be_bytes(data);

        return Ok(be);
    }
}

