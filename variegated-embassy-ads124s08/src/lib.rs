#![no_std]

pub mod registers;

use core::cmp::PartialEq;
use defmt::Format;
use embassy_futures::select::{Either, select};
use embassy_rp::gpio::{AnyPin, Input, Output, Pin, Pull};
use embassy_rp::spi;
use embassy_rp::spi::{Async, Instance, Spi};
use embassy_time::{Duration, Timer};
use registers::{RegisterAddress, ByteRepresentation, ConfigurationRegisters};
use crate::registers::{IDACMagnitude, IDACMux, PGAGain, ReferenceInput, StatusRegisterValue};

pub enum WaitStrategy<'a> {
    UseRiskyMiso(AnyPin),
    UseDrdyPin(Input<'a, AnyPin>),
    Delay
}

#[derive(Debug, Clone, Copy, Format)]
enum Command {
    NOP,
    WAKEUP,
    POWERDOWN,
    RESET,
    START,
    STOP,
    SYOCAL,
    SYGCAL,
    SFOCAL,
    RDATA,
    RREG { address: u8, n: u8 },
    WREG { address: u8, n: u8 },
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

pub struct ADS124S08<'a> {
    cs: Output<'a, AnyPin>,
    wait_strategy: WaitStrategy<'a>,
    configuration_registers: ConfigurationRegisters,

    read_configuration_registers: ConfigurationRegisters,
}

#[derive(Debug, Format)]
pub enum ADS124S08Error {
    SPIError(spi::Error),
    ReadTimeoutError,
    WriteTimeoutError,
    UnableToRestorePreviousConfigurationWhileHandlingError,
    InvalidRegisterValue(RegisterAddress, u8),
    InvalidCommand,
    NotInTransaction,
    ConfigurationReadBackFailed(RegisterAddress)
}

#[derive(Debug, Format, Copy, Clone)]
pub struct Code {
    pub data: [u8; 3],
    pub reference: ReferenceInput,
    pub pga_enabled: bool,
    pub gain: PGAGain,
    pub idac1_magnitude: Option<IDACMagnitude>,
    pub idac2_magnitude: Option<IDACMagnitude>,
}

impl Code {
    pub fn over_fs(&self) -> bool {
        self.data[0] == 0x7f && self.data[1] == 0xff && self.data[2] == 0xff
    }

    pub fn below_fs(&self) -> bool {
        self.data[0] == 0x80 && self.data[1] == 0x00 && self.data[2] == 0x00
    }

    pub fn code(&self) -> i32 {
        let fill = if self.data[0] & 0x80 == 0x80 { 0xff } else { 0x00 };

        i32::from_be_bytes([fill, self.data[0], self.data[1], self.data[2]])
    }

    pub fn internally_referenced_voltage(&self) -> f32 {
        self.externally_referenced_voltage(-2.5, 2.5)
    }

    pub fn externally_referenced_voltage(&self, v_ref_n: f32, v_ref_p: f32) -> f32 {
        // @todo Consider gain
        let code = self.code();
//        let v_ref = v_ref_p - v_ref_n;
        let v_per_lsb = (2.0 * v_ref_p) / (0x0100_0000 as f32);

        code as f32 * v_per_lsb
    }

    pub fn ratiometric_resistance(&self, r_ref: f32) -> f32 {
        let code = self.code() as f32;
        // @todo Consider gain
        r_ref * code / (0x0080_0000 as f32)
    }
}

macro_rules! define_read_write_register_ops {
    ($base_name:ident, $reg:ident, $reg_type:ty) => {
        paste::item! {
            // Generate read function name
            pub async fn [<read_ $base_name _reg>]<'a, T>(&mut self, spi: &mut Spi<'a, T, Async>) -> Result<$reg_type, ADS124S08Error>
            where T: Instance {
                let bits = self.read_reg(spi, registers::RegisterAddress::$reg).await?;
                let res = $reg_type::from_bits(bits).ok_or(ADS124S08Error::InvalidRegisterValue(registers::RegisterAddress::$reg, bits));
                if res.is_ok() {
                    let d = res.unwrap();
                    self.read_configuration_registers.$base_name = d.clone();
                    return Ok(d)
                }

                res
            }

            // Generate write function name
            pub async fn [<write_ $base_name _reg>]<'a, T>(&mut self, spi: &mut Spi<'a, T, Async>, value: $reg_type) -> Result<(), ADS124S08Error>
            where T: Instance {
                self.write_reg(spi, registers::RegisterAddress::$reg, value.bits()).await?;
                self.configuration_registers.$base_name = value;
                self.read_configuration_registers.$base_name = value;
                Ok(())
            }

            // Generate swap function name
            pub async fn [<swap_ $base_name _reg>]<'a, T>(&mut self, spi: &mut Spi<'_, T, Async>, value: $reg_type) -> Result<$reg_type, ADS124S08Error>
            where T: Instance {
                let current = self.[<read_ $base_name _reg>](spi).await?;
                self.[<write_ $base_name _reg>](spi, value).await?;
                Ok(current)
            }
        }
    };
}

macro_rules! define_read_only_register_ops {
    ($base_name:ident, $reg:ident, $reg_type:ty) => {
        paste::item! {
            // Generate read function name
            async fn [<read_ $base_name _reg>]<'a, T>(&mut self, spi: &mut Spi<'a, T, Async>) -> Result<$reg_type, ADS124S08Error>
            where T: Instance {
                let bits = self.read_reg(spi, registers::RegisterAddress::$reg).await?;
                $reg_type::from_bits(bits).ok_or(ADS124S08Error::InvalidRegisterValue(registers::RegisterAddress::$reg, bits))
            }
        }
    };
}

impl ADS124S08<'_> {
    // High level API
    pub fn new<'a>(cs: Output<'a, AnyPin>, wait_strategy: WaitStrategy<'a>) -> ADS124S08<'a> {
        ADS124S08 {
            cs,
            wait_strategy,
            configuration_registers: ConfigurationRegisters::default(),
            read_configuration_registers: ConfigurationRegisters::default(),
        }
    }

    pub async fn measure_single_ended<'a, T>(&mut self, spi: &mut Spi<'a, T, Async>, input: registers::Mux, reference_input: ReferenceInput) -> Result<Code, ADS124S08Error> where T: Instance {
        self.begin_transaction().await;

        let data = {
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

            config = self.swap_all_configuration_registers(spi, config).await?;

            let res = self.start_wait_for_drdy_read_and_stop(spi).await;

            self.swap_all_configuration_registers(spi, config).await?;

            res
        };

        self.end_transaction().await;

        data
    }

    pub async fn measure_ratiometric_low_side<'a, T>(
        &mut self,
        spi: &mut Spi<'a, T, Async>,
        input_p: registers::Mux,
        input_n: registers::Mux,
        idac1: registers::IDACMux,
        idac2: registers::IDACMux,
        reference_input: registers::ReferenceInput,
        idac_magnitude: registers::IDACMagnitude,
        gain: registers::PGAGain,
    ) -> Result<Code, ADS124S08Error> where T: Instance {
        self.begin_transaction().await;

        let data = {
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

            config = self.swap_all_configuration_registers(spi, config).await?;

            let res = self.start_wait_for_drdy_read_and_stop(spi).await;

            self.swap_all_configuration_registers(spi, config).await?;

            res
        };

        self.end_transaction().await;

        data
    }

    pub async fn read_dvdd_by_4<'a, T>(&mut self, spi: &mut Spi<'a, T, Async>) -> Result<Code, ADS124S08Error> where T: Instance {
        self.begin_transaction().await;

        let data = {
            let mut new_config = self.configuration_registers.sys.clone();
            new_config.sys_mon = registers::SystemMonitorConfiguration::DvddBy4Measurement;
            new_config.sendstat = true;
            new_config.crc = true;

            let prev_config = self.swap_sys_reg(
                spi,
                new_config
            ).await?;

            let mut new_refctl = self.configuration_registers.refctrl.clone();
            new_refctl.refsel = registers::ReferenceInput::Internal;
            new_refctl.refcon = registers::InternalVoltageReferenceConfiguration::AlwaysOn;

            let prev_refctl = self.swap_refctrl_reg(spi, new_refctl).await?;

            let curr_cfg_read = self.read_sys_reg(spi).await?;
            //log::info!("Current SYS: {:?}", curr_cfg_read);

            let res = self.start_wait_for_drdy_and_read(spi).await;

            self.write_sys_reg(spi, prev_config).await?;
            self.write_refctrl_reg(spi, prev_refctl).await?;

            res
        };

        self.end_transaction().await;

        data
    }

    pub async fn read_avdd_by_4<'a, T>(&mut self, spi: &mut Spi<'a, T, Async>) -> Result<Code, ADS124S08Error> where T: Instance {
        self.begin_transaction().await;

        let data = {
            let mut new_config = self.configuration_registers.sys.clone();
            new_config.sys_mon = registers::SystemMonitorConfiguration::AvddMinusAvssBy4Measurement;
            new_config.sendstat = true;
            new_config.crc = true;

            let prev_config = self.swap_sys_reg(
                spi,
                new_config
            ).await?;

            let mut new_refctl = self.configuration_registers.refctrl.clone();
            new_refctl.refsel = registers::ReferenceInput::Internal;
            new_refctl.refcon = registers::InternalVoltageReferenceConfiguration::AlwaysOn;

            let prev_refctl = self.swap_refctrl_reg(spi, new_refctl).await?;

            let res = self.start_wait_for_drdy_read_and_stop(spi).await;

            self.write_sys_reg(spi, prev_config).await?;
            self.write_refctrl_reg(spi, prev_refctl).await?;

            res
        };

        self.end_transaction().await;

        data
    }

    pub async fn read_temperature<'a, T>(&mut self, spi: &mut Spi<'a, T, Async>) -> Result<Code, ADS124S08Error> where T: Instance {
        self.begin_transaction().await;

        let data = {
            let prev_sys = self.swap_sys_reg(
                spi,
                self.configuration_registers.sys.copy_with_configuration(registers::SystemMonitorConfiguration::InternalTemperatureSensor)
            ).await?;

            let mut new_pga = self.configuration_registers.pga.clone();
            new_pga.enable = true;
            new_pga.gain = registers::PGAGain::Gain1;

            let prev_pga = self.swap_pga_reg(
                spi,
                new_pga
            ).await?;

            let res = self.start_wait_for_drdy_read_and_stop(spi).await;

            self.write_sys_reg(spi, prev_sys).await?;
            self.write_pga_reg(spi, prev_pga).await?;

            res
        };

        self.end_transaction().await;

        data
    }

    pub async fn read_device_id<'a, T>(&mut self, spi: &mut Spi<'a, T, Async>) -> Result<registers::DeviceId, ADS124S08Error> where T: Instance {
        self.begin_transaction().await;

        let res = self.read_id_reg(spi).await;

        self.end_transaction().await;

        res
    }

    // Lower level API
    pub async fn begin_transaction(&mut self) {
        self.assert_cs().await;
    }

    pub async fn end_transaction(&mut self) {
        self.deassert_cs().await;
    }

    pub async fn configure_measurement<'a, T>(&mut self, spi: &mut Spi<'a, T, Async>, configuration_registers: ConfigurationRegisters) -> Result<(), ADS124S08Error> where T: Instance {
        self.configuration_registers = configuration_registers;
        self.write_modified_configuration_registers_to_device(spi).await
    }

    pub async fn start_conversion<'a, T>(&mut self, spi: &mut Spi<'a, T, Async>) -> Result<(), ADS124S08Error> where T: Instance {
        self.write_command(spi, Command::START).await
    }

    pub async fn stop_conversion<'a, T>(&mut self, spi: &mut Spi<'a, T, Async>) -> Result<(), ADS124S08Error> where T: Instance {
        self.write_command(spi, Command::STOP).await
    }

    pub async fn wait_for_drdy_and_read<'a, T>(&mut self, spi: &mut Spi<'_, T, Async>) -> Result<Code, ADS124S08Error> where T: Instance {
        self.wait_for_drdy(spi).await?;
        self.read_data(spi).await
    }

    pub async fn start_wait_for_drdy_and_read<'a, T>(&mut self, spi: &mut Spi<'_, T, Async>) -> Result<Code, ADS124S08Error> where T: Instance {
        self.start_conversion(spi).await?;
        self.wait_for_drdy(spi).await?;
        self.read_data(spi).await
    }

    pub async fn start_wait_for_drdy_read_and_stop<'a, T>(&mut self, spi: &mut Spi<'_, T, Async>) -> Result<Code, ADS124S08Error> where T: Instance {
        self.start_conversion(spi).await?;
        self.wait_for_drdy(spi).await?;
        let d = self.read_data(spi).await?;
        self.stop_conversion(spi).await?;

        Ok(d)
    }

    pub async fn read_configuration_registers<'a, T>(&mut self, spi: &mut Spi<'a, T, Async>) -> Result<ConfigurationRegisters, ADS124S08Error> where T: Instance {
        self.read_configuration_registers_from_device(spi).await?;
        Ok(self.read_configuration_registers.clone())
    }

    // Why are you even looking at this?

    async fn wait_for_drdy<'a, T>(&mut self, spi: &mut Spi<'a, T, Async>) -> Result<(), ADS124S08Error> where T: Instance {
        // @todo Use an either with a timeout
        
        match &mut self.wait_strategy {
            WaitStrategy::UseRiskyMiso(miso_pin) => wait_for_unsafe_miso_pin(miso_pin).await,
            WaitStrategy::UseDrdyPin(drdy_input) => drdy_input.wait_for_low().await,
            WaitStrategy::Delay => Timer::after(Duration::from_millis(1000)).await, // @todo Change delay to match sample rate
        }
        
        Ok(())
    }
    
    async fn swap_all_configuration_registers<'a, T>(&mut self, spi: &mut Spi<'a, T, Async>, configuration_registers: ConfigurationRegisters) -> Result<ConfigurationRegisters, ADS124S08Error> where T: Instance {
        let prev = self.configuration_registers.clone();
        self.configuration_registers = configuration_registers;
        let res = self.write_modified_configuration_registers_to_device(spi).await;

        if res.is_err() {
            self.configuration_registers = prev;
            self.write_all_configuration_registers_to_device(spi).await?;
            return Err(ADS124S08Error::UnableToRestorePreviousConfigurationWhileHandlingError);
        }

        Ok(prev)
    }

    async fn read_configuration_registers_from_device<'a, T>(&mut self, spi: &mut Spi<'a, T, Async>) -> Result<(), ADS124S08Error> where T: Instance {
        self.configuration_registers.inpmux = self.read_inpmux_reg(spi).await?;
        self.configuration_registers.pga = self.read_pga_reg(spi).await?;
        self.configuration_registers.datarate = self.read_datarate_reg(spi).await?;
        self.configuration_registers.refctrl = self.read_refctrl_reg(spi).await?;
        self.configuration_registers.idacmag = self.read_idacmag_reg(spi).await?;
        self.configuration_registers.idacmux = self.read_idacmux_reg(spi).await?;
        self.configuration_registers.vbias = self.read_vbias_reg(spi).await?;
        self.configuration_registers.sys = self.read_sys_reg(spi).await?;

        Ok(())
    }

    async fn write_all_configuration_registers_to_device<'a, T>(&mut self, spi: &mut Spi<'a, T, Async>) -> Result<(), ADS124S08Error> where T: Instance {
        self.write_inpmux_reg(spi, self.configuration_registers.inpmux).await?;
        self.write_pga_reg(spi, self.configuration_registers.pga).await?;
        self.write_datarate_reg(spi, self.configuration_registers.datarate).await?;
        self.write_refctrl_reg(spi, self.configuration_registers.refctrl).await?;
        self.write_idacmag_reg(spi, self.configuration_registers.idacmag).await?;
        self.write_idacmux_reg(spi, self.configuration_registers.idacmux).await?;
        self.write_vbias_reg(spi, self.configuration_registers.vbias).await?;
        self.write_sys_reg(spi, self.configuration_registers.sys).await?;

        Ok(())
    }

    async fn write_modified_configuration_registers_to_device<'a, T>(&mut self, spi: &mut Spi<'a, T, Async>) -> Result<(), ADS124S08Error> where T: Instance {
        if self.configuration_registers.inpmux != self.read_configuration_registers.inpmux {
            self.write_inpmux_reg(spi, self.configuration_registers.inpmux).await?;
        }

        if self.configuration_registers.pga != self.read_configuration_registers.pga {
            self.write_pga_reg(spi, self.configuration_registers.pga).await?;
        }

        if self.configuration_registers.datarate != self.read_configuration_registers.datarate {
            self.write_datarate_reg(spi, self.configuration_registers.datarate).await?;
        }

        if self.configuration_registers.refctrl != self.read_configuration_registers.refctrl {
            self.write_refctrl_reg(spi, self.configuration_registers.refctrl).await?;
        }

        if self.configuration_registers.idacmag != self.read_configuration_registers.idacmag {
            self.write_idacmag_reg(spi, self.configuration_registers.idacmag).await?;
        }

        if self.configuration_registers.idacmux != self.read_configuration_registers.idacmux {
            self.write_idacmux_reg(spi, self.configuration_registers.idacmux).await?;
        }

        if self.configuration_registers.vbias != self.read_configuration_registers.vbias {
            self.write_vbias_reg(spi, self.configuration_registers.vbias).await?;
        }

        if self.configuration_registers.sys != self.read_configuration_registers.sys {
            self.write_sys_reg(spi, self.configuration_registers.sys).await?;
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

    pub async fn read_data<'a, T>(&mut self, spi: &mut Spi<'a, T, Async>) -> Result<Code, ADS124S08Error> where T: Instance {
        self.write_command(spi, Command::RDATA).await?;

        if self.configuration_registers.sys.sendstat {
            let mut status_buf = [0x00];
            self.spi_read(spi, &mut status_buf).await?;

            let res = StatusRegisterValue::from_bits(status_buf[0]);
            //log::info!("Status: {:?}", res);
        }

        let mut data_buf = [0x00; 3];
        self.spi_read(spi, &mut data_buf).await?;

        //log::info!("Data: 0x{:x} 0x{:x} 0x{:x}", data_buf[0], data_buf[1], data_buf[2]);

        if self.configuration_registers.sys.crc {
            let mut crc_buf = [0x00];
            self.spi_read(spi, &mut crc_buf).await?;
            //log::info!("CRC: 0x{:x}", crc_buf[0])
        }

        Ok(Code {
            data: data_buf,
            reference: self.configuration_registers.refctrl.refsel,
            pga_enabled: self.configuration_registers.pga.enable,
            gain: self.configuration_registers.pga.gain,
            idac1_magnitude: if (self.configuration_registers.idacmux.i1mux != IDACMux::Disconnected) { Some(self.configuration_registers.idacmag.imag) } else { None },
            idac2_magnitude: if (self.configuration_registers.idacmux.i2mux != IDACMux::Disconnected) { Some(self.configuration_registers.idacmag.imag) } else { None },
        })
    }

    async fn write_reg<'a, T>(&mut self, spi: &mut Spi<'a, T, Async>, reg: RegisterAddress, value: u8) -> Result<(), ADS124S08Error> where T: Instance {
        let command = Command::WREG { address: reg.addr(), n: 0 };

        self.write_command(spi, command).await?;
        self.spi_write(spi, &[value]).await?;

        let read_back = self.read_reg(spi, reg).await?;

        if read_back != value {
            return Err(ADS124S08Error::ConfigurationReadBackFailed(reg));
        }

        Ok(())
    }

    async fn read_reg<'a, T>(&mut self, spi: &mut Spi<'a, T, Async>, register: RegisterAddress) -> Result<u8, ADS124S08Error> where T: Instance {
        let command = Command::RREG { address: register.addr(), n: 0 };

        self.write_command(spi, command).await?;
        let mut reg_buf = [0x00];
        self.spi_read(spi, &mut reg_buf).await?;

//        log::info!("Read register: {:?} 0x{:x}", register, reg_buf[0]);

        Ok(reg_buf[0])
    }

    async fn write_command<'a, T>(&mut self, spi: &mut Spi<'a, T, Async>, command: Command) -> Result<(), ADS124S08Error> where T: Instance {
        let (byte0, byte1) = command.bits();

        if byte1.is_some() {
            self.spi_write(spi, &[byte0, byte1.unwrap()]).await?;
        } else {
            self.spi_write(spi, &[byte0]).await?;
        }

        //log::info!("Wrote command: {:?} 0x{:x} {:?}", command, byte0, byte1);

        Ok(())
    }

    async fn spi_read<'a, T>(&mut self, spi: &mut Spi<'a, T, Async>, buffer: &mut [u8]) -> Result<(), ADS124S08Error> where T: Instance {
        if self.cs.is_set_high() {
            return Err(ADS124S08Error::NotInTransaction);
        }

        let t1 = spi.read(buffer);
        let t2 = async {
            Timer::after(Duration::from_millis(1)).await;
        };

        let either = select(t1, t2).await;

        let res = match either {
            Either::First(p) => p.map_err(|e| ADS124S08Error::SPIError(e)),
            Either::Second(_) => Err(ADS124S08Error::ReadTimeoutError),
        };

        res
    }

    async fn spi_write<'a, T>(&mut self, spi: &mut Spi<'a, T, Async>, buffer: &[u8]) -> Result<(), ADS124S08Error> where T: Instance {
        if self.cs.is_set_high() {
            return Err(ADS124S08Error::NotInTransaction);
        }

        let t1 = spi.write(buffer);
        let t2 = async {
            Timer::after(Duration::from_millis(1)).await;
        };

        let either = select(t1, t2).await;

        let res = match either {
            Either::First(p) => p.map_err(|e| ADS124S08Error::SPIError(e)),
            Either::Second(_) => Err(ADS124S08Error::ReadTimeoutError),
        };

        res
    }

    async fn assert_cs(&mut self) {
        Timer::after_micros(1).await;
        self.cs.set_low();
        Timer::after_micros(1).await;
    }

    async fn deassert_cs(&mut self) {
        Timer::after_micros(1).await;
        self.cs.set_high();
        Timer::after_micros(1).await;
    }
}

async fn wait_for_unsafe_miso_pin(pin: &mut AnyPin) {
    let pin_num = pin.pin();

    // @todo Use an either to handle timeout
    async {
        let mut input = Input::new(pin, Pull::Up);
        input.wait_for_low().await;
    }.await;

    // Obviously, this isn't exactly safe
    let bank = embassy_rp::pac::IO_BANK0;
    bank.gpio(pin_num as _).ctrl().write(|w| w.set_funcsel(1));
}