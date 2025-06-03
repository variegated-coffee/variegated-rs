#![no_std]
#![warn(missing_docs)]

use byteorder::ByteOrder as _;
use core::{fmt, slice};
use embedded_hal_async::digital::Wait;
use embedded_hal_async::i2c::I2c;
use embedded_hal_async::delay::DelayNs;

mod constants;
pub use constants::*;

pub type Result<T> = core::result::Result<T, Error>;

#[derive(Debug)]
pub enum Error {
    I2cError,
    PowerupFailed,
}

pub enum Nau7802DataAvailableStrategy<W: Wait>{
    Polling,
    DrdyPin(W),
}

pub struct Nau7802<W: Wait, D: DelayNs> {
    address: u8,
    wait_strategy: Nau7802DataAvailableStrategy<W>,
    delay: D
}

impl<W: Wait, D: DelayNs> Nau7802<W, D> {
    pub fn new(wait_strategy: Nau7802DataAvailableStrategy<W>, delay: D, address: Option<u8>) -> Self {
        let addr = if let Some(a) = address {
            a
        } else {
            0x2A
        };

        Self {
            wait_strategy,
            delay,
            address: addr
        }
    }

    pub async fn init<I2C, I2CError>(
        &mut self,
        i2c: &mut I2C,
        ldo: Ldo,
        gain: Gain,
        sps: SamplesPerSecond,
    ) -> Result<()> where I2C: I2c<Error =I2CError>, I2CError: fmt::Debug  {
        self.start_reset(i2c).await?;
        self.finish_reset(i2c).await?;
        self.power_up(i2c).await?;
        self.set_ldo(i2c, ldo).await?;
        self.set_gain(i2c, gain).await?;
        self.set_sample_rate(i2c, sps).await?;
        self.misc_init(i2c).await?;
        self.begin_afe_calibration(i2c).await?;

        while self.poll_afe_calibration_status(i2c).await? != AfeCalibrationStatus::Success {}

        Ok(())
    }

    pub async fn wait_for_data_available<I2C, I2CError>(&mut self, i2c: &mut I2C) -> Result<()> where I2C: I2c<Error =I2CError>, I2CError: fmt::Debug {
        match self.wait_strategy {
            Nau7802DataAvailableStrategy::Polling => self.wait_for_data_available_i2c(i2c).await?,
            Nau7802DataAvailableStrategy::DrdyPin(ref mut w) => w.wait_for_high().await.map_err(|_| Error::I2cError)?
        }
        
        Ok(())
    }

    async fn wait_for_data_available_i2c<I2C, I2CError>(&mut self, i2c: &mut I2C) -> Result<()> where I2C: I2c<Error =I2CError>, I2CError: fmt::Debug {
        loop {
            if self.get_bit(i2c, Register::PuCtrl, PuCtrlBits::CR).await? {
                return Ok(());
            }
            self.delay.delay_ms(1).await;
        }
    }
    
    pub async fn read<I2C, I2CError>(&mut self, i2c: &mut I2C) -> Result<i32> where I2C: I2c<Error =I2CError>, I2CError: fmt::Debug {
        self.wait_for_data_available(i2c).await?;

        self.request_register(i2c, Register::AdcoB2).await?;

        let mut buf = [0u8; 3]; // will hold an i24
        i2c
            .read(self.address, &mut buf)
            .await
            .map_err(|_| Error::I2cError)?;

        let adc_result = byteorder::BigEndian::read_i24(&buf);
        Ok(adc_result)
    }

    pub async fn begin_afe_calibration<I2C, I2CError>(&self, i2c: &mut I2C) -> Result<()> where I2C: I2c<Error =I2CError>, I2CError: fmt::Debug {
        self.set_bit(i2c, Register::Ctrl2, Ctrl2RegisterBits::Cals).await
    }

    pub async fn poll_afe_calibration_status<I2C, I2CError>(&self, i2c: &mut I2C) -> Result<AfeCalibrationStatus> where I2C: I2c<Error =I2CError>, I2CError: fmt::Debug {
        if self.get_bit(i2c, Register::Ctrl2, Ctrl2RegisterBits::Cals).await? {
            return Ok(AfeCalibrationStatus::InProgress);
        }

        if self.get_bit(i2c, Register::Ctrl2, Ctrl2RegisterBits::CalError).await? {
            return Ok(AfeCalibrationStatus::Failure);
        }

        Ok(AfeCalibrationStatus::Success)
    }

    pub async fn set_sample_rate<I2C, I2CError>(&self, i2c: &mut I2C, sps: SamplesPerSecond) -> Result<()> where I2C: I2c<Error =I2CError>, I2CError: fmt::Debug {
        const SPS_MASK: u8 = 0b10001111;
        const SPS_START_BIT_IDX: u8 = 4;

        self.set_function_helper(i2c, Register::Ctrl2, SPS_MASK, SPS_START_BIT_IDX, sps as _).await
    }

    pub async fn set_gain<I2C, I2CError>(&self, i2c: &mut I2C, gain: Gain) -> Result<()> where I2C: I2c<Error =I2CError>, I2CError: fmt::Debug {
        const GAIN_MASK: u8 = 0b11111000;
        const GAIN_START_BIT: u8 = 0;

        self.set_function_helper(i2c, Register::Ctrl1, GAIN_MASK, GAIN_START_BIT, gain as _).await
    }

    pub async fn set_ldo<I2C, I2CError>(&self, i2c: &mut I2C, ldo: Ldo) -> Result<()> where I2C: I2c<Error =I2CError>, I2CError: fmt::Debug {
        const LDO_MASK: u8 = 0b11000111;
        const LDO_START_BIT: u8 = 3;

        self.set_function_helper(i2c, Register::Ctrl1, LDO_MASK, LDO_START_BIT, ldo as _).await?;

        self.set_bit(i2c, Register::PuCtrl, PuCtrlBits::AVDDS).await
    }

    pub async fn power_up<I2C, I2CError>(&self, i2c: &mut I2C) -> Result<()> where I2C: I2c<Error =I2CError>, I2CError: fmt::Debug {
        const NUM_ATTEMPTS: usize = 100;

        self.set_bit(i2c, Register::PuCtrl, PuCtrlBits::PUD).await?;
        self.set_bit(i2c, Register::PuCtrl, PuCtrlBits::PUA).await?;

        let mut powered_up = false;
        let mut attempts = 0;
        while !powered_up && attempts < 100 {
            let res = self.get_bit(i2c, Register::PuCtrl, PuCtrlBits::PUR).await;
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

    pub async fn start_reset<I2C, I2CError>(&self, i2c: &mut I2C) -> Result<()> where I2C: I2c<Error =I2CError>, I2CError: fmt::Debug {
        self.set_bit(i2c, Register::PuCtrl, PuCtrlBits::RR).await
    }

    pub async fn finish_reset<I2C, I2CError>(&self, i2c: &mut I2C) -> Result<()> where I2C: I2c<Error =I2CError>, I2CError: fmt::Debug {
        self.clear_bit(i2c, Register::PuCtrl, PuCtrlBits::RR).await
    }


    pub async fn misc_init<I2C, I2CError>(&self, i2c: &mut I2C) -> Result<()> where I2C: I2c<Error =I2CError>, I2CError: fmt::Debug {
        const TURN_OFF_CLK_CHPL: u8 = 0x30;

        // Turn off CLK_CHP. From 9.1 power on sequencing
        self.set_register(i2c, Register::Adc, TURN_OFF_CLK_CHPL).await?;

        // Enable 330pF decoupling cap on chan 2. From 9.14 application circuit note
        self.set_bit(i2c, Register::PgaPwr, PgaPwrRegisterBits::CapEn).await
    }

    async fn set_function_helper<I2C, I2CError>(
        &self,
        i2c: &mut I2C,
        reg: Register,
        mask: u8,
        start_idx: u8,
        new_val: u8,
    ) -> Result<()> where I2C: I2c<Error =I2CError>, I2CError: fmt::Debug {
        let mut val = self.get_register(i2c, reg).await?;
        val &= mask;
        val |= new_val << start_idx;

        self.set_register(i2c, reg, val).await
    }

    async fn set_bit<I2C, I2CError, B: RegisterBits>(&self, i2c: &mut I2C, addr: Register, bit_idx: B) -> Result<()> where I2C: I2c<Error =I2CError>, I2CError: fmt::Debug {
        let mut val = self.get_register(i2c, addr).await?;
        val |= 1 << bit_idx.get();
        self.set_register(i2c, addr, val).await
    }

    async fn clear_bit<I2C, I2CError, B: RegisterBits>(&self, i2c: &mut I2C, addr: Register, bit_idx: B) -> Result<()> where I2C: I2c<Error =I2CError>, I2CError: fmt::Debug {
        let mut val = self.get_register(i2c, addr).await?;
        val &= !(1 << bit_idx.get());
        self.set_register(i2c, addr, val).await
    }

    async fn get_bit<I2C, I2CError, B: RegisterBits>(&self, i2c: &mut I2C, addr: Register, bit_idx: B) -> Result<bool> where I2C: I2c<Error =I2CError>, I2CError: fmt::Debug {
        let mut val = self.get_register(i2c, addr).await?;
        val &= 1 << bit_idx.get();
        Ok(val != 0)
    }

    async fn set_register<I2C, I2CError>(&self, i2c: &mut I2C, reg: Register, val: u8) -> Result<()> where I2C: I2c<Error =I2CError>, I2CError: fmt::Debug {
        let transaction = [reg as _, val];

        i2c
            .write(self.address, &transaction)
            .await
            .map_err(|_| Error::I2cError)
    }

    async fn get_register<I2C, I2CError>(&self, i2c: &mut I2C, reg: Register) -> Result<u8> where I2C: I2c<Error =I2CError>, I2CError: fmt::Debug  {
        self.request_register(i2c, reg).await?;

        let mut val = 0;
        i2c
            .read(self.address, slice::from_mut(&mut val))
            .await
            .map_err(|_| Error::I2cError)?;

        Ok(val)
    }

    async fn request_register<I2C, I2CError>(&self, i2c: &mut I2C, reg: Register) -> Result<()> where I2C: I2c<Error =I2CError>, I2CError: fmt::Debug  {
        let reg = reg as u8;

        i2c
            .write(self.address, slice::from_ref(&reg))
            .await
            .map_err(|_| Error::I2cError)
    }
}