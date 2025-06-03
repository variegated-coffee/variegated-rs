//! Constants and register definitions for the NAU7802 ADC

#[allow(dead_code)]
#[derive(Clone, Copy)]
#[repr(u8)]
/// NAU7802 register addresses
pub enum Register {
    /// Power-up control register
    PuCtrl = 0x00,
    /// Control register 1
    Ctrl1,
    /// Control register 2
    Ctrl2,
    /// Offset calibration byte 2
    Ocal1B2,
    /// Offset calibration byte 1
    Ocal1B1,
    /// Offset calibration byte 0
    Ocal1B0,
    /// Gain calibration byte 3
    Gcal1B3,
    /// Gain calibration byte 2
    Gcal1B2,
    /// Gain calibration byte 1
    Gcal1B1,
    /// Gain calibration byte 0
    Gcal1B0,
    /// Offset calibration 2 byte 2
    Ocal2B2,
    /// Offset calibration 2 byte 1
    Ocal2B1,
    /// Offset calibration 2 byte 0
    Ocal2B0,
    /// Gain calibration 2 byte 3
    Gcal2B3,
    /// Gain calibration 2 byte 2
    Gcal2B2,
    /// Gain calibration 2 byte 1
    Gcal2B1,
    /// Gain calibration 2 byte 0
    Gcal2B0,
    /// I2C control register
    I2CControl,
    /// ADC output byte 2
    AdcoB2,
    /// ADC output byte 1
    AdcoB1,
    /// ADC output byte 0
    AdcoB0,
    /// Shared ADC and OTP register
    Adc = 0x15, // Shared ADC and OTP 32:24
    /// OTP byte 1
    OtpB1,      // OTP 23:16 or 7:0?
    /// OTP byte 0
    OtpB0,      // OTP 15:8
    /// PGA register
    Pga = 0x1B,
    /// PGA power register
    PgaPwr = 0x1C,
    /// Device revision register
    DeviceRev = 0x1F,
}

#[allow(dead_code)]
#[derive(Clone, Copy)]
#[repr(u8)]
/// Power-up control register bit positions
pub enum PuCtrlBits {
    /// Register reset
    RR = 0,
    /// Power-up digital
    PUD,
    /// Power-up analog
    PUA,
    /// Power-up ready
    PUR,
    /// Cycle start
    CS,
    /// Cycle ready
    CR,
    /// System clock source
    OSCS,
    /// AVDD source
    AVDDS,
}

#[allow(dead_code)]
#[derive(Clone, Copy)]
#[repr(u8)]
/// PGA register bit positions
pub enum PgaRegisterBits {
    /// Chopper disable
    ChpDis = 0,
    /// Invert signal
    Inv = 3,
    /// Bypass enable
    BypassEn,
    /// Output enable
    OutEn,
    /// LDO mode
    LdoMode,
    /// Read OTP select
    RdOtpSel,
}

#[allow(dead_code)]
#[derive(Clone, Copy)]
#[repr(u8)]
/// PGA power register bit positions
pub enum PgaPwrRegisterBits {
    /// Current setting
    Curr = 0,
    /// ADC current setting
    AdcCurr = 2,
    /// Master bias current setting
    MstrBiasCurr = 4,
    /// Capacitor enable
    CapEn = 7,
}

#[allow(dead_code)]
#[derive(Clone, Copy)]
#[repr(u8)]
/// Control register 2 bit positions
pub enum Ctrl2RegisterBits {
    /// Calibration mode
    CalMod = 0,
    /// Calibration start
    Cals = 2,
    /// Calibration error
    CalError = 3,
    /// Conversion rate setting
    Crs = 4,
    /// Channel select
    Chs = 7,
}

/// Trait for register bit manipulation
pub trait RegisterBits {
    /// Get the bit position value
    fn get(&self) -> u8;
}

macro_rules! impl_register_bits {
    ($($type:ident),*) => {
        $(
            impl RegisterBits for $type {
                fn get(&self) -> u8 {
                    *self as _
                }
            }
        )*
    }
}

impl_register_bits!(
    PuCtrlBits,
    PgaRegisterBits,
    PgaPwrRegisterBits,
    Ctrl2RegisterBits
);

#[derive(Clone, Copy)]
#[repr(u8)]
/// Low-dropout regulator voltage settings
pub enum Ldo {
    /// 2.4V
    L2v4 = 0b111,
    /// 2.7V
    L2v7 = 0b110,
    /// 3.0V
    L3v0 = 0b101,
    /// 3.3V
    L3v3 = 0b100,
    /// 3.6V
    L3v6 = 0b011,
    /// 3.9V
    L3v9 = 0b010,
    /// 3.2V
    L3v2 = 0b001,
    /// 4.5V
    L4v5 = 0b000,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// Programmable gain amplifier gain settings
pub enum Gain {
    /// Gain of 128
    G128 = 0b111,
    /// Gain of 64
    G64 = 0b110,
    /// Gain of 32
    G32 = 0b101,
    /// Gain of 16
    G16 = 0b100,
    /// Gain of 8
    G8 = 0b011,
    /// Gain of 4
    G4 = 0b010,
    /// Gain of 2
    G2 = 0b001,
    /// Gain of 1
    G1 = 0b000,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// ADC sample rate settings
pub enum SamplesPerSecond {
    /// 320 samples per second
    SPS320 = 0b111,
    /// 80 samples per second
    SPS80 = 0b011,
    /// 40 samples per second
    SPS40 = 0b010,
    /// 20 samples per second
    SPS20 = 0b001,
    /// 10 samples per second
    SPS10 = 0b000,
}

#[derive(PartialEq)]
/// Status of analog front-end calibration
pub enum AfeCalibrationStatus {
    /// Calibration is currently in progress
    InProgress,
    /// Calibration failed
    Failure,
    /// Calibration completed successfully
    Success,
}