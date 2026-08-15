//! TLC59108 register definitions and constants

/// TLC59108 register addresses
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Register {
    /// Mode register 1
    Mode1 = 0x00,
    /// Mode register 2
    Mode2 = 0x01,
    /// PWM channel 0
    Pwm0 = 0x02,
    /// PWM channel 1
    Pwm1 = 0x03,
    /// PWM channel 2
    Pwm2 = 0x04,
    /// PWM channel 3
    Pwm3 = 0x05,
    /// PWM channel 4
    Pwm4 = 0x06,
    /// PWM channel 5
    Pwm5 = 0x07,
    /// PWM channel 6
    Pwm6 = 0x08,
    /// PWM channel 7
    Pwm7 = 0x09,
    /// Group PWM duty cycle control
    GrpPwm = 0x0A,
    /// Group frequency
    GrpFreq = 0x0B,
    /// LED output state 0 (controls LED0-3)
    LedOut0 = 0x0C,
    /// LED output state 1 (controls LED4-7)
    LedOut1 = 0x0D,
    /// I2C bus subaddress 1 (not used)
    SubAdr1 = 0x0E,
    /// I2C bus subaddress 2 (not used)
    SubAdr2 = 0x0F,
    /// I2C bus subaddress 3 (not used)
    SubAdr3 = 0x10,
    /// LED All Call I2C bus address (not used)
    AllCallAdr = 0x11,
    /// Current setting register
    Iref = 0x12,
    /// Error flags
    EFlag = 0x13,
}

impl Register {
    pub const fn addr(&self) -> u8 {
        *self as u8
    }
}

/// MODE1 register bits
pub mod mode1 {
    /// Oscillator off bit
    pub const OSC: u8 = 1 << 4;
    /// Auto-increment bit 0
    pub const AI0: u8 = 1 << 5;
    /// Auto-increment bit 1
    pub const AI1: u8 = 1 << 6;
    /// Auto-increment bit 2
    pub const AI2: u8 = 1 << 7;
}

/// MODE2 register bits
pub mod mode2 {
    /// Output change on ACK (0 = change on STOP, 1 = change on ACK)
    pub const OCH: u8 = 1 << 3;
    /// Group control dimming/blinking (0 = dimming, 1 = blinking)
    pub const DMBLNK: u8 = 1 << 5;
    /// Clear error status flag
    pub const EFCLR: u8 = 1 << 7;
}

/// Auto-increment modes
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AutoIncrement {
    /// No auto-increment
    None = 0b000,
    /// Auto-increment for all registers
    All = 0b100,
    /// Auto-increment for individual brightness registers only
    Brightness = 0b101,
    /// Auto-increment for individual and group registers
    IndividualAndGroup = 0b110,
    /// Auto-increment for global registers only
    Global = 0b111,
}

impl AutoIncrement {
    pub const fn bits(&self) -> u8 {
        (*self as u8) << 5
    }
}

/// LED driver output state (2 bits per LED)
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LedState {
    /// LED driver off
    Off = 0b00,
    /// LED driver on (fully on, no PWM)
    FullyOn = 0b01,
    /// LED driver controlled by PWM register
    Pwm = 0b10,
    /// LED driver controlled by PWM and group PWM/group blinking
    PwmAndGroup = 0b11,
}

impl LedState {
    pub const fn bits(&self) -> u8 {
        *self as u8
    }
}

/// IREF register configuration for output current control
///
/// The IREF register controls the output current capability by setting:
/// - Voltage Gain (VG): Controls voltage at REXT terminal
/// - Current Multiplier (CM): Sets IOUT/IREF ratio (15 or 5)
/// - Current Gain (CG): Overall current scaling factor
///
/// Formula: IOUT = (1.26V / REXT) × VG × 15 × 3^(CM-1)
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct IrefConfig {
    /// Current Multiplier (bit 0)
    /// - false (0): IOUT/IREF = 5, suitable for 5-40mA range
    /// - true (1): IOUT/IREF = 15, suitable for 10-120mA range
    pub current_multiplier: bool,
    /// Voltage sub-band selector (bit 7)
    /// - false (0): Low sub-band, VG = 1/4 to 127/256
    /// - true (1): High sub-band, VG = 1/2 to 127/128
    pub voltage_subband: bool,
    /// Current control value (bits 6:1)
    /// Valid range: 0-63
    pub current_control: u8,
}

impl IrefConfig {
    /// Create default configuration (maximum current capability)
    /// VG = 127/128, CM = 1, CG = 0.992
    pub const fn default() -> Self {
        Self {
            current_multiplier: true,
            voltage_subband: true,
            current_control: 0x3F, // 63
        }
    }

    /// Create configuration for low current applications (5-40mA range)
    /// VG = 1/2, CM = 0, suitable for REXT ≈ 1.5kΩ for 20mA target
    pub const fn low_current() -> Self {
        Self {
            current_multiplier: false,
            voltage_subband: true,
            current_control: 0,
        }
    }

    /// Create configuration for high current applications (50-120mA range)
    /// VG = 127/128, CM = 1, suitable for REXT ≈ 360Ω for 50mA target
    pub const fn high_current() -> Self {
        Self::default()
    }

    /// Convert configuration to register byte value
    pub const fn to_register(&self) -> u8 {
        let mut value = 0u8;

        // Bit 0: Current Multiplier
        if self.current_multiplier {
            value |= 0x01;
        }

        // Bits 6:1: Current Control (ensure only 6 bits are used)
        value |= (self.current_control & 0x3F) << 1;

        // Bit 7: Voltage sub-band
        if self.voltage_subband {
            value |= 0x80;
        }

        value
    }

    /// Create configuration from register byte value
    pub const fn from_register(value: u8) -> Self {
        Self {
            current_multiplier: (value & 0x01) != 0,
            current_control: (value >> 1) & 0x3F,
            voltage_subband: (value & 0x80) != 0,
        }
    }

    /// Calculate voltage gain (VG) from configuration
    /// VG = (1 + HC) × (1 + CC/64) / 4
    pub fn voltage_gain(&self) -> f32 {
        let hc = if self.voltage_subband { 1.0 } else { 0.0 };
        let cc = self.current_control as f32;
        (1.0 + hc) * (1.0 + cc / 64.0) / 4.0
    }

    /// Calculate current gain (CG) from configuration
    /// CG = VG × 3^(CM-1)
    pub fn current_gain(&self) -> f32 {
        let vg = self.voltage_gain();
        if self.current_multiplier {
            vg // 3^(1-1) = 3^0 = 1
        } else {
            vg / 3.0 // 3^(0-1) = 3^-1 = 1/3
        }
    }

    /// Calculate target output current for given external resistor
    /// IOUT = (1.26V / REXT) × 15 × CG
    pub fn output_current_ma(&self, rext_ohms: f32) -> f32 {
        let iref = 1.26 / rext_ohms; // Reference current in Amperes
        let multiplier = if self.current_multiplier { 15.0 } else { 5.0 };
        let cg = self.current_gain();
        iref * multiplier * cg * 1000.0 // Convert to mA
    }
}

/// Software reset I2C sequence
pub const SWRST_ADDR: u8 = 0x4B;
pub const SWRST_BYTE1: u8 = 0xA5;
pub const SWRST_BYTE2: u8 = 0x5A;