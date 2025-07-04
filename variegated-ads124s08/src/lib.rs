//! # ADS124S08 Embassy Driver
//!
//! Embassy-based driver for the Texas Instruments ADS124S08 24-bit precision ADC.
//! This driver provides async support for SPI communication with the ADS124S08,
//! suitable for high-precision analog-to-digital conversion in espresso machine
//! temperature and pressure sensing applications.
//!
//! The ADS124S08 is a precision, 24-bit, analog-to-digital converter (ADC) with
//! 12 single-ended or 6 differential inputs, programmable gain amplifier (PGA),
//! voltage reference, and two excitation current sources (IDACs).
//!
//! ## Usage Example
//!
//! ```no_run
//! use variegated_ads124s08::{ADS124S08, WaitStrategy, PGAGain, ReferenceInput, IDACMagnitude, IDACMux};
//! 
//! # async fn example() -> Result<(), Box<dyn std::error::Error>> {
//! # let spi_device = todo!();
//! # let drdy_pin = todo!();
//! # let delay = todo!();
//! // Create the driver with SPI device, DRDY pin, and delay provider
//! let wait_strategy = WaitStrategy::UseDrdyPin(drdy_pin);
//! let mut adc = ADS124S08::new(spi_device, wait_strategy, delay);
//!
//! // Initialize the device
//! adc.initialize().await?;
//!
//! // Perform a high-level measurement 
//! let result = adc.measure_input_with_reference_and_pga(
//!     InputP::AIN0,      // Positive input
//!     InputN::AIN1,      // Negative input  
//!     ReferenceInput::REF0P_REF0N,  // Reference
//!     PGAGain::GAIN16,   // 16x gain
//!     Some(IDACMagnitude::uA500),   // 500µA excitation current
//!     IDACMux::AIN0,     // Current output to AIN0
//! ).await?;
//!
//! // Convert to voltage
//! let voltage = result.to_voltage();
//! # Ok(())
//! # }
//! ```
//!
//! See the datasheet in `docs/datasheet.pdf` for complete technical specifications.

#![no_std]
#![warn(missing_docs)]

/// Register definitions and configuration structures for the ADS124S08
pub mod registers;

#[cfg(feature = "defmt")]
use defmt::Format;
use embedded_hal_async::delay::DelayNs;
use crate::registers::{Filter, IDACMagnitude, IDACMux, Mode, PGAGain, ReferenceInput, StatusRegisterValue, RegisterAddress, ByteRepresentation, ConfigurationRegisters};
use embedded_hal::digital::InputPin;
use embedded_hal::spi::Operation;
use embedded_hal_async::digital::Wait;
use embedded_hal_async::spi::SpiDevice;
use embassy_futures::select::{select, Either};


/// Strategy for waiting for data ready signal from the ADS124S08
pub enum WaitStrategy<INPUT: InputPin + Wait> {
    /// Use the dedicated DRDY pin to detect when data is ready
    UseDrdyPin(INPUT),
    /// Use delay-based polling instead of the DRDY pin
    Delay
}

/// ADS124S08 commands as defined in the datasheet
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub enum Command {
    /// No operation
    NOP,
    /// Wake up from power-down mode
    WAKEUP,
    /// Enter power-down mode
    POWERDOWN,
    /// Reset the device
    RESET,
    /// Start ADC conversions
    START,
    /// Stop ADC conversions
    STOP,
    /// System offset calibration
    SYOCAL,
    /// System gain calibration
    SYGCAL,
    /// Self offset calibration
    SFOCAL,
    /// Read conversion data
    RDATA,
    /// Read register command
    RREG { 
        /// Starting register address
        address: u8, 
        /// Number of registers to read minus 1
        n: u8 
    },
    /// Write register command
    WREG { 
        /// Starting register address
        address: u8, 
        /// Number of registers to write minus 1
        n: u8 
    },
}

impl Command {
    fn bits(&self) -> (u8, Option<u8>) {
        match self {
            Command::NOP => (0b0000_0000, None),
            Command::WAKEUP => (0b0000_0010, None),
            Command::POWERDOWN => (0b0000_0100, None),
            Command::RESET => (0b0000_0110, None),
            Command::START => (0b0000_1000, None),
            Command::STOP => (0b0000_1010, None),
            Command::SYOCAL => (0b0001_0110, None),
            Command::SYGCAL => (0b0001_0111, None),
            Command::SFOCAL => (0b0001_1001, None),
            Command::RDATA => (0b0001_0010, None),
            Command::RREG { address, n } => (0b0010_0000 | (address & 0b1_1111), Some(n & 0b1_1111)),
            Command::WREG { address, n } => (0b0100_0000 | (address & 0b1_1111), Some(n & 0b1_1111)),
        }
    }
}

/// Errors that can occur when communicating with the ADS124S08
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub enum ADS124S08Error {
    /// SPI communication error
    SPIError,
    /// Timeout occurred while reading data
    ReadTimeoutError,
    /// Timeout occurred while writing data
    WriteTimeoutError,
    /// Unable to restore previous configuration while handling another error
    UnableToRestorePreviousConfigurationWhileHandlingError,
    /// Invalid value read from or written to a register
    InvalidRegisterValue(RegisterAddress, u8),
    /// Invalid command sent to device
    InvalidCommand,
    /// Operation attempted when not in transaction mode
    NotInTransaction,
    /// Configuration readback verification failed
    ConfigurationReadBackFailed(RegisterAddress)
}

/// Conversion result from the ADS124S08 ADC
#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub struct Code {
    /// Raw 24-bit conversion data as 3 bytes
    pub data: [u8; 3],
    /// Reference input used for this conversion
    pub reference: ReferenceInput,
    /// Whether the PGA was enabled for this conversion
    pub pga_enabled: bool,
    /// PGA gain used for this conversion
    pub gain: PGAGain,
    /// IDAC1 magnitude if enabled, None if disabled
    pub idac1_magnitude: Option<IDACMagnitude>,
    /// IDAC2 magnitude if enabled, None if disabled
    pub idac2_magnitude: Option<IDACMagnitude>,
}

impl Code {
    /// Create a new Code from raw data and conversion parameters
    pub fn new(data: [u8; 3], reference: ReferenceInput, pga_enabled: bool, gain: PGAGain, idac1_magnitude: Option<IDACMagnitude>, idac2_magnitude: Option<IDACMagnitude>) -> Self {
        Code {
            data,
            reference,
            pga_enabled,
            gain,
            idac1_magnitude,
            idac2_magnitude,
        }
    }

    /// Create a new Code from a signed 32-bit integer and conversion parameters
    pub fn from_code(code: i32, reference: ReferenceInput, pga_enabled: bool, gain: PGAGain, idac1_magnitude: Option<IDACMagnitude>, idac2_magnitude: Option<IDACMagnitude>) -> Self {
        let original: [u8; 4] = code.to_be_bytes();
        let result: [u8; 3] = original[1..].try_into().unwrap();

        Code {
            data: result,
            reference,
            pga_enabled,
            gain,
            idac1_magnitude,
            idac2_magnitude,
        }
    }

    /// Check if the conversion result indicates over full scale
    pub fn over_fs(&self) -> bool {
        self.data[0] == 0x7f && self.data[1] == 0xff && self.data[2] == 0xff
    }

    /// Check if the conversion result indicates below full scale
    pub fn below_fs(&self) -> bool {
        self.data[0] == 0x80 && self.data[1] == 0x00 && self.data[2] == 0x00
    }

    /// Convert the raw data to a signed 32-bit integer
    pub fn code(&self) -> i32 {
        let fill = if self.data[0] & 0x80 == 0x80 { 0xff } else { 0x00 };

        i32::from_be_bytes([fill, self.data[0], self.data[1], self.data[2]])
    }

    /// Convert to voltage using internal 2.5V reference
    pub fn internally_referenced_voltage(&self) -> f32 {
        self.externally_referenced_voltage(0.0, 2.5)
    }

    /// Convert to voltage using external reference voltages
    pub fn externally_referenced_voltage(&self, v_ref_n: f32, v_ref_p: f32) -> f32 {
        let code = self.code();
        let v_ref = v_ref_p - v_ref_n;
        let gain = if self.pga_enabled { self.gain.value() as f32 } else { 1.0 };
        let v_per_lsb = (2.0 * v_ref) / (gain * (1 << 24) as f32);

        code as f32 * v_per_lsb
    }

    /// Convert to resistance using ratiometric measurement with reference resistance
    pub fn ratiometric_resistance(&self, r_ref: f32) -> f32 {
        let code = self.code() as f32;
        let gain = if self.pga_enabled { self.gain.value() as f32 } else { 1.0 };
        (r_ref * code) / ((1 << 23) as f32 * gain)
    }
}

macro_rules! define_read_write_register_ops {
    ($base_name:ident, $reg:ident, $reg_type:ty) => {
        paste::item! {
            // Generate read function name
            #[allow(dead_code)]
            #[doc = concat!("Read the ", stringify!($base_name), " register")]
            pub async fn [<read_ $base_name _reg>]<'a>(&mut self) -> Result<$reg_type, ADS124S08Error>
             {
                let bits = self.read_reg(registers::RegisterAddress::$reg).await?;
                let res = $reg_type::from_bits(bits).ok_or(ADS124S08Error::InvalidRegisterValue(registers::RegisterAddress::$reg, bits));
                if res.is_ok() {
                    let d = res.unwrap();
                    self.read_configuration_registers.$base_name = d.clone();
                    return Ok(d)
                }

                res
            }

            // Generate write function name
            #[allow(dead_code)]
            #[doc = concat!("Write the ", stringify!($base_name), " register")]
            pub async fn [<write_ $base_name _reg>]<'a>(&mut self, value: $reg_type) -> Result<(), ADS124S08Error>
             {
                self.write_reg(registers::RegisterAddress::$reg, value.bits()).await?;
                self.configuration_registers.$base_name = value;
                self.read_configuration_registers.$base_name = value;
                Ok(())
            }

            // Generate swap function name
            #[allow(dead_code)]
            #[doc = concat!("Swap the ", stringify!($base_name), " register and return the previous value")]
            pub async fn [<swap_ $base_name _reg>]<'a>(&mut self, value: $reg_type) -> Result<$reg_type, ADS124S08Error>
             {
                let current = self.[<read_ $base_name _reg>]().await?;
                self.[<write_ $base_name _reg>](value).await?;
                Ok(current)
            }
        }
    };
}

macro_rules! define_read_only_register_ops {
    ($base_name:ident, $reg:ident, $reg_type:ty) => {
        paste::item! {
            // Generate read function name
            #[allow(dead_code)]
            #[doc = concat!("Read the ", stringify!($base_name), " register")]
            async fn [<read_ $base_name _reg>]<'a>(&mut self) -> Result<$reg_type, ADS124S08Error>
             {
                let bits = self.read_reg(registers::RegisterAddress::$reg).await?;
                $reg_type::from_bits(bits).ok_or(ADS124S08Error::InvalidRegisterValue(registers::RegisterAddress::$reg, bits))
            }
        }
    };
}

/// ADS124S08 ADC driver with Embassy async support
pub struct ADS124S08<SpiDevT: SpiDevice, InputPinT: InputPin + Wait, D: DelayNs> {
    spi: SpiDevT,
    wait_strategy: WaitStrategy<InputPinT>,
    delay: D,
    configuration_registers: ConfigurationRegisters,

    read_configuration_registers: ConfigurationRegisters,
}

impl<SpiDevT: SpiDevice, InputPinT: InputPin + Wait, D: DelayNs> ADS124S08<SpiDevT, InputPinT, D> {
    // High level API
    /// Create a new ADS124S08 driver instance
    pub fn new(spi: SpiDevT, wait_strategy: WaitStrategy<InputPinT>, delay: D) -> ADS124S08<SpiDevT, InputPinT, D> {
        ADS124S08 {
            spi,
            wait_strategy,
            delay,
            configuration_registers: ConfigurationRegisters::default(),
            read_configuration_registers: ConfigurationRegisters::default(),
        }
    }

    /// Perform a single-ended measurement on the specified input pin
    pub async fn measure_single_ended<'a>(&mut self, input: registers::Mux, reference_input: ReferenceInput) -> Result<Code, ADS124S08Error>  {
        // This assumes that AVss is connected to GND, not to a negative voltage, and that AINCOM is connected to 0V
        let mut config = self.configuration_registers.clone();

        config.inpmux.p = input;
        config.inpmux.n = registers::Mux::AINCOM;
        config.pga.enable = false;
        config.pga.gain = registers::PGAGain::Gain1;
        config.idacmag.imag = registers::IDACMagnitude::Off;
        config.idacmux.i2mux = registers::IDACMux::Disconnected;
        config.idacmux.i1mux = registers::IDACMux::Disconnected;
        config.refctrl.refsel = reference_input;

        config = self.swap_all_configuration_registers(config).await?;

        let res = self.start_wait_for_drdy_read_and_stop().await;
        self.swap_all_configuration_registers(config).await?;

        res
    }

    /// Perform a ratiometric low-side measurement with excitation current
    pub async fn measure_ratiometric_low_side<'a>(
        &mut self,
        input_p: registers::Mux,
        input_n: registers::Mux,
        idac1: registers::IDACMux,
        idac2: registers::IDACMux,
        reference_input: registers::ReferenceInput,
        idac_magnitude: registers::IDACMagnitude,
        gain: registers::PGAGain,
    ) -> Result<Code, ADS124S08Error>  {
        // See SBAA275A, 2.3 for schematic
        // SBAA275A, 2.6 and 2.1 also works, but set idac2 to disconnected
        let mut config = self.configuration_registers.clone();

        config.inpmux.p = input_p;
        config.inpmux.n = input_n;
        config.idacmag.imag = idac_magnitude;
        config.idacmux.i2mux = idac2;
        config.idacmux.i1mux = idac1;
        config.refctrl.n_refp_buf = true;
        config.refctrl.n_refn_buf = true;
        config.refctrl.refsel = reference_input;
        config.refctrl.refcon = registers::InternalVoltageReferenceConfiguration::OnButPowersDown;
        config.pga.enable = true;
        config.pga.gain = gain;
        config.datarate.mode = Mode::Continuous;
        config.datarate.rate = registers::DataRate::SPS20;
        config.datarate.filter = Filter::SINC3;

        config = self.swap_all_configuration_registers(config).await?;

        self.start_conversion().await?;
//        let res = self.read_n_sample_average(10).await;
        let res = self.start_wait_for_drdy_read_and_stop().await;

        self.swap_all_configuration_registers(config).await?;

        res
    }

    /// Perform a differential measurement between two input pins
    pub async fn measure_differential<'a>(
        &mut self,
        input_p: registers::Mux,
        input_n: registers::Mux,
        reference_input: registers::ReferenceInput,
        gain: registers::PGAGain,
    ) -> Result<Code, ADS124S08Error>  {
        let mut config = self.configuration_registers.clone();

        config.inpmux.p = input_p;
        config.inpmux.n = input_n;
        config.pga.enable = gain != PGAGain::Gain1;
        config.pga.gain = gain;
        config.idacmag.imag = IDACMagnitude::Off;
        config.idacmux.i2mux = IDACMux::Disconnected;
        config.idacmux.i1mux = IDACMux::Disconnected;
        config.refctrl.refsel = reference_input;

        // Takes around 2 ms
        config = self.swap_all_configuration_registers(config).await?;

        // This call has about 2 ms of overhead
        let res = self.start_wait_for_drdy_read_and_stop().await;

        // Takes around 2 ms
        self.swap_all_configuration_registers(config).await?;

        res
    }

    /// Read the digital supply voltage (DVDD) divided by 4
    pub async fn read_dvdd_by_4<'a>(&mut self) -> Result<Code, ADS124S08Error>  {
        let mut new_config = self.configuration_registers.sys.clone();
        new_config.sys_mon = registers::SystemMonitorConfiguration::DvddBy4Measurement;
        new_config.sendstat = true;
        new_config.crc = true;

        let prev_config = self.swap_sys_reg(
            new_config
        ).await?;

        let mut new_refctl = self.configuration_registers.refctrl.clone();
        new_refctl.refsel = registers::ReferenceInput::Internal;
        new_refctl.refcon = registers::InternalVoltageReferenceConfiguration::AlwaysOn;

        let prev_refctl = self.swap_refctrl_reg(new_refctl).await?;

        let _curr_cfg_read = self.read_sys_reg().await?;
        //log::info!("Current SYS: {:?}", curr_cfg_read);

        let res = self.start_wait_for_drdy_and_read().await;

        self.write_sys_reg(prev_config).await?;
        self.write_refctrl_reg(prev_refctl).await?;

        res
    }

    /// Read the analog supply voltage (AVDD) minus AVSS divided by 4
    pub async fn read_avdd_by_4<'a>(&mut self) -> Result<Code, ADS124S08Error>  {
        let mut new_config = self.configuration_registers.sys.clone();
        new_config.sys_mon = registers::SystemMonitorConfiguration::AvddMinusAvssBy4Measurement;
        new_config.sendstat = true;
        new_config.crc = true;

        let prev_config = self.swap_sys_reg(
            new_config
        ).await?;

        let mut new_refctl = self.configuration_registers.refctrl.clone();
        new_refctl.refsel = registers::ReferenceInput::Internal;
        new_refctl.refcon = registers::InternalVoltageReferenceConfiguration::AlwaysOn;

        let prev_refctl = self.swap_refctrl_reg(new_refctl).await?;

        let res = self.start_wait_for_drdy_read_and_stop().await;

        self.write_sys_reg(prev_config).await?;
        self.write_refctrl_reg(prev_refctl).await?;

        res
    }


    /// Read the internal temperature sensor
    pub async fn read_temperature<'a>(&mut self) -> Result<Code, ADS124S08Error>  {
        let mut new_config = self.configuration_registers.sys.clone();
        new_config.sys_mon = registers::SystemMonitorConfiguration::InternalTemperatureSensor;
        new_config.sendstat = true;
        new_config.crc = true;

        let prev_config = self.swap_sys_reg(
            new_config
        ).await?;

        let mut new_refctl = self.configuration_registers.refctrl.clone();
        new_refctl.refsel = registers::ReferenceInput::Internal;
        new_refctl.refcon = registers::InternalVoltageReferenceConfiguration::AlwaysOn;

        let prev_refctl = self.swap_refctrl_reg(new_refctl).await?;

        let res = self.start_wait_for_drdy_read_and_stop().await;

        self.write_sys_reg(prev_config).await?;
        self.write_refctrl_reg(prev_refctl).await?;

        res
    }

    /// Read the device identification register
    pub async fn read_device_id<'a>(&mut self) -> Result<registers::DeviceId, ADS124S08Error>  {
        self.read_id_reg().await
    }

    // Lower level API
    /// Configure the ADC measurement parameters
    pub async fn configure_measurement<'a>(&mut self, configuration_registers: ConfigurationRegisters) -> Result<(), ADS124S08Error>  {
        self.configuration_registers = configuration_registers;
        self.write_modified_configuration_registers_to_device().await
    }

    /// Start ADC conversions
    pub async fn start_conversion<'a>(&mut self) -> Result<(), ADS124S08Error>  {
        self.write_command(Command::START).await
    }

    /// Stop ADC conversions
    pub async fn stop_conversion<'a>(&mut self) -> Result<(), ADS124S08Error>  {
        self.write_command(Command::STOP).await
    }

    /// Wait for data ready signal and read the conversion result
    pub async fn wait_for_drdy_and_read<'a>(&mut self) -> Result<Code, ADS124S08Error>  {
        self.wait_for_drdy().await?;
        self.read_data().await
    }

    /// Start conversion, wait for data ready, and read the result
    pub async fn start_wait_for_drdy_and_read<'a>(&mut self) -> Result<Code, ADS124S08Error>  {
        self.start_conversion().await?;
        self.wait_for_drdy().await?;
        self.read_data().await
    }

    /// Start conversion, wait for data ready, read result, and stop conversion
    pub async fn start_wait_for_drdy_read_and_stop<'a>(&mut self) -> Result<Code, ADS124S08Error>  {
        self.start_conversion().await?;
        self.wait_for_drdy().await?;

        let d = self.read_data().await?;

        self.stop_conversion().await?;

        Ok(d)
    }

    /// Read and average multiple samples
    pub async fn read_n_sample_average<'a>(&mut self, n: i32) -> Result<Code, ADS124S08Error>  {
        let mut sum = 0;

        for _ in 0..n {
            self.wait_for_drdy().await?;
            sum += self.read_data().await?.code();
        }

        Ok(Code::from_code(sum / n, ReferenceInput::Internal, false, PGAGain::Gain1, None, None))
    }

    /// Read the current configuration registers from the device
    pub async fn read_configuration_registers<'a>(&mut self) -> Result<ConfigurationRegisters, ADS124S08Error>  {
        self.read_configuration_registers_from_device().await?;
        Ok(self.read_configuration_registers.clone())
    }

    // Why are you even looking at this?

    /// Wait for the data ready signal
    pub async fn wait_for_drdy<'a>(&mut self) -> Result<(), ADS124S08Error>  {
        match &mut self.wait_strategy {
            WaitStrategy::UseDrdyPin(drdy_input) => {
                // Use timeout for DRDY pin waiting to prevent infinite hang
                let timeout_ms = self.configuration_registers.datarate.rate.sample_period_ms() * 3; // 3x sample period timeout
                let wait_future = drdy_input.wait_for_low();
                let timeout_future = async {
                    self.delay.delay_ms(timeout_ms).await;
                    Err(ADS124S08Error::ReadTimeoutError)
                };
                
                match select(wait_future, timeout_future).await {
                    Either::First(result) => result.map_err(|_e| ADS124S08Error::SPIError),
                    Either::Second(timeout_err) => timeout_err,
                }
            },
            // Use delay based on configured sample rate with some margin
            WaitStrategy::Delay => {
                let base_period = self.configuration_registers.datarate.rate.sample_period_ms();
                let margin = core::cmp::max(base_period * 5 / 100, 1); // 5% or 1ms, whichever is larger
                let delay_ms = base_period + margin;
                self.delay.delay_ms(delay_ms).await;
                Ok(())
            },
        }
    }

    async fn swap_all_configuration_registers<'a>(&mut self, configuration_registers: ConfigurationRegisters) -> Result<ConfigurationRegisters, ADS124S08Error>  {
        let prev = self.configuration_registers.clone();
        self.configuration_registers = configuration_registers;
        let res = self.write_modified_configuration_registers_to_device().await;

        if res.is_err() {
            self.configuration_registers = prev;
            self.write_all_configuration_registers_to_device().await?;
            return Err(ADS124S08Error::UnableToRestorePreviousConfigurationWhileHandlingError);
        }

        Ok(prev)
    }

    async fn read_configuration_registers_from_device<'a>(&mut self) -> Result<(), ADS124S08Error>  {
        self.configuration_registers.inpmux = self.read_inpmux_reg().await?;
        self.configuration_registers.pga = self.read_pga_reg().await?;
        self.configuration_registers.datarate = self.read_datarate_reg().await?;
        self.configuration_registers.refctrl = self.read_refctrl_reg().await?;
        self.configuration_registers.idacmag = self.read_idacmag_reg().await?;
        self.configuration_registers.idacmux = self.read_idacmux_reg().await?;
        self.configuration_registers.vbias = self.read_vbias_reg().await?;
        self.configuration_registers.sys = self.read_sys_reg().await?;

        Ok(())
    }

    async fn write_all_configuration_registers_to_device<'a>(&mut self) -> Result<(), ADS124S08Error>  {
        self.write_inpmux_reg(self.configuration_registers.inpmux).await?;
        self.write_pga_reg(self.configuration_registers.pga).await?;
        self.write_datarate_reg(self.configuration_registers.datarate).await?;
        self.write_refctrl_reg(self.configuration_registers.refctrl).await?;
        self.write_idacmag_reg(self.configuration_registers.idacmag).await?;
        self.write_idacmux_reg(self.configuration_registers.idacmux).await?;
        self.write_vbias_reg(self.configuration_registers.vbias).await?;
        self.write_sys_reg(self.configuration_registers.sys).await?;

        Ok(())
    }

    async fn write_modified_configuration_registers_to_device<'a>(&mut self) -> Result<(), ADS124S08Error>  {
        if self.configuration_registers.inpmux != self.read_configuration_registers.inpmux {
            self.write_inpmux_reg(self.configuration_registers.inpmux).await?;
        }

        if self.configuration_registers.pga != self.read_configuration_registers.pga {
            self.write_pga_reg(self.configuration_registers.pga).await?;
        }

        if self.configuration_registers.datarate != self.read_configuration_registers.datarate {
            self.write_datarate_reg(self.configuration_registers.datarate).await?;
        }

        if self.configuration_registers.refctrl != self.read_configuration_registers.refctrl {
            self.write_refctrl_reg(self.configuration_registers.refctrl).await?;
        }

        if self.configuration_registers.idacmag != self.read_configuration_registers.idacmag {
            self.write_idacmag_reg(self.configuration_registers.idacmag).await?;
        }

        if self.configuration_registers.idacmux != self.read_configuration_registers.idacmux {
            self.write_idacmux_reg(self.configuration_registers.idacmux).await?;
        }

        if self.configuration_registers.vbias != self.read_configuration_registers.vbias {
            self.write_vbias_reg(self.configuration_registers.vbias).await?;
        }

        if self.configuration_registers.sys != self.read_configuration_registers.sys {
            self.write_sys_reg(self.configuration_registers.sys).await?;
        }

        Ok(())
    }

    define_read_only_register_ops!(id, ID, registers::DeviceId);
    define_read_only_register_ops!(status, STATUS, registers::StatusRegisterValue);
    define_read_write_register_ops!(inpmux, INPMUX, registers::InputMultiplexerRegister);
    define_read_write_register_ops!(pga, PGA, registers::PgaRegister);
    define_read_write_register_ops!(datarate, DATARATE, registers::DataRateRegister);
    define_read_write_register_ops!(refctrl, REF, registers::ReferenceControlRegister);
    define_read_write_register_ops!(idacmag, IDACMAG, registers::IDACMagnitudeRegister);
    define_read_write_register_ops!(idacmux, IDACMUX, registers::IDACMultiplexerRegister);
    define_read_write_register_ops!(vbias, VBIAS, registers::SensorBiasingRegister);
    define_read_write_register_ops!(sys, SYS, registers::SystemControlRegister);

    /// Read conversion data from the ADC
    pub async fn read_data<'a>(&mut self) -> Result<Code, ADS124S08Error>  {
        self.write_command(Command::RDATA).await?;

        if self.configuration_registers.sys.sendstat {
            let mut status_buf = [0x00];
            self.spi_read(&mut status_buf).await?;

            let _ = StatusRegisterValue::from_bits(status_buf[0]);
            //log::info!("Status: {:?}", res);
        }

        let mut data_buf = [0x00; 3];
        self.spi_read(&mut data_buf).await?;

        //log::info!("Data: 0x{:x} 0x{:x} 0x{:x}", data_buf[0], data_buf[1], data_buf[2]);

        if self.configuration_registers.sys.crc {
            let mut crc_buf = [0x00];
            self.spi_read(&mut crc_buf).await?;
            //log::info!("CRC: 0x{:x}", crc_buf[0])
        }

        Ok(Code {
            data: data_buf,
            reference: self.configuration_registers.refctrl.refsel,
            pga_enabled: self.configuration_registers.pga.enable,
            gain: self.configuration_registers.pga.gain,
            idac1_magnitude: if self.configuration_registers.idacmux.i1mux != IDACMux::Disconnected { Some(self.configuration_registers.idacmag.imag) } else { None },
            idac2_magnitude: if self.configuration_registers.idacmux.i2mux != IDACMux::Disconnected { Some(self.configuration_registers.idacmag.imag) } else { None },
        })
    }

    async fn write_reg<'a>(&mut self, reg: RegisterAddress, value: u8) -> Result<(), ADS124S08Error>  {
        let command = Command::WREG { address: reg.addr(), n: 0 };

        let (byte0, byte1) = command.bits();

        if byte1.is_some() {
            self.spi_transaction(&mut [Operation::DelayNs(100000), Operation::Write(&[byte0, byte1.unwrap()]), Operation::Write(&[value]), Operation::DelayNs(100000)]).await.map_err(|_e| ADS124S08Error::SPIError)?;
        } else {
            self.spi_transaction(&mut [Operation::DelayNs(100000), Operation::Write(&[byte0]), Operation::Write(&[value]), Operation::DelayNs(100000)]).await.map_err(|_e| ADS124S08Error::SPIError)?;
        }

        // @fixme Performance measurment workaround
/*        let read_back = self.read_reg(spi, reg).await?;

        if read_back != value {
            return Err(ADS124S08Error::ConfigurationReadBackFailed(reg));
        }*/

        Ok(())
    }

    async fn read_reg<'a>(&mut self, register: RegisterAddress) -> Result<u8, ADS124S08Error>  {
        let command = Command::RREG { address: register.addr(), n: 0 };
        let mut reg_buf = [0x00];

        let (byte0, byte1) = command.bits();
        
        if byte1.is_some() {
            self.spi_transaction(&mut [Operation::DelayNs(100), Operation::Write(&[byte0, byte1.unwrap()]), Operation::Read(&mut reg_buf), Operation::DelayNs(100)]).await.map_err(|_e| ADS124S08Error::SPIError)?;
        } else {
            self.spi_transaction(&mut [Operation::DelayNs(100), Operation::Write(&[byte0]), Operation::Read(&mut reg_buf), Operation::DelayNs(100)]).await.map_err(|_e| ADS124S08Error::SPIError)?;
        }

//        log::info!("Read register: {:?} 0x{:x}", register, reg_buf[0]);

        Ok(reg_buf[0])
    }

    /// Reset the ADC to its default configuration
    pub async fn reset<'a>(&mut self) -> Result<(), ADS124S08Error>  {
        self.write_command(Command::RESET).await?;
        self.delay.delay_ms(100).await;
        self.read_configuration_registers_from_device().await?;

        Ok(())
    }

    async fn write_command<'a>(&mut self, command: Command) -> Result<(), ADS124S08Error>  {
        let (byte0, byte1) = command.bits();

        if byte1.is_some() {
            self.spi_write(&[byte0, byte1.unwrap()]).await?;
        } else {
            self.spi_write(&[byte0]).await?;
        }

        //log::info!("Wrote command: {:?} 0x{:x} {:?}", command, byte0, byte1);

        Ok(())
    }
    
    async fn spi_read<'a>(&mut self, buffer: &mut [u8]) -> Result<(), ADS124S08Error>  {
        let ret = self.spi.transaction(&mut[Operation::DelayNs(100), Operation::Read(buffer), Operation::DelayNs(100)]).await.map_err(|_e| ADS124S08Error::SPIError);

        ret

//        return self.spi.read(buffer).await.map_err(|e| ADS124S08Error::SPIError);

        // @fixme Performance measurment workaround. Make this optional.
/*
        let t1 = self.spi.read(buffer);
        let t2 = async {
            Timer::after(Duration::from_millis(1)).await;
        };

        let either = select(t1, t2).await;

        let res = match either {
            Either::First(p) => p.map_err(|e| ADS124S08Error::SPIError(e)),
            Either::Second(_) => Err(ADS124S08Error::ReadTimeoutError),
        };

        res*/
    }

    async fn spi_write<'a>(&mut self, buffer: &[u8]) -> Result<(), ADS124S08Error>  {
        let ret = self.spi.transaction(&mut[Operation::DelayNs(100), Operation::Write(buffer), Operation::DelayNs(100)]).await.map_err(|_e| ADS124S08Error::SPIError);

        ret

        //return self.spi.write(buffer).await.map_err(|e| ADS124S08Error::SPIError);

        // @fixme Performance measurment workaround. Make this optional.
/*        let t1 = self.spi.write(buffer);
        let t2 = async {
            Timer::after(Duration::from_millis(1)).await;
        };

        let either = select(t1, t2).await;

        let res = match either {
            Either::First(p) => p.map_err(|e| ADS124S08Error::SPIError(e)),
            Either::Second(_) => Err(ADS124S08Error::ReadTimeoutError),
        };

        res*/
    }

    async fn spi_transaction<'a>(&mut self, ops: &mut [Operation<'_, u8>]) -> Result<(), ADS124S08Error>  {
        let ret = self.spi.transaction(ops).await.map_err(|_e| ADS124S08Error::SPIError);

        ret
    }
}