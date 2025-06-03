//! Register definitions and bit field structures for the ADS124S08 ADC

use defmt::Format;

/// Trait for types that can be converted to/from raw register bits
pub trait ByteRepresentation {
    /// Convert to register bit representation
    fn bits(&self) -> u8;
    /// Create from register bit representation
    fn from_bits(bits: u8) -> Option<Self> where Self: Sized;
}



/// ADS124S08 register addresses
#[derive(Debug, Clone, Copy, Format)]
/// RegisterAddress
pub enum RegisterAddress {
    /// Device identification register
    /// ID

    ID,
    /// Status register
    /// STATUS

    STATUS,
    /// Input multiplexer register
    /// INPMUX

    INPMUX,
    /// Programmable gain amplifier register
    /// PGA

    PGA,
    /// Data rate register
    /// DATARATE

    DATARATE,
    /// Reference control register
    /// REF

    REF,
    /// IDAC magnitude register
    /// IDACMAG

    IDACMAG,
    /// IDAC multiplexer register
    /// IDACMUX

    IDACMUX,
    /// Sensor biasing register
    /// VBIAS

    VBIAS,
    /// System control register
    /// SYS

    SYS,
    /// Offset calibration register 0
    /// OFCAL0

    OFCAL0,
    /// Offset calibration register 1
    /// OFCAL1

    OFCAL1,
    /// Offset calibration register 2
    /// OFCAL2

    OFCAL2,
    /// Full-scale calibration register 0
    /// FSCAL0

    FSCAL0,
    /// Full-scale calibration register 1
    /// FSCAL1

    FSCAL1,
    /// Full-scale calibration register 2
    /// FSCAL2

    FSCAL2,
    /// GPIO data register
    /// GPIODAT

    GPIODAT,
    /// GPIO configuration register
    /// GPIOCON

    GPIOCON,
}

impl RegisterAddress {
    /// Get the register address value
    pub fn addr(&self) -> u8 {
        match self {
            RegisterAddress::ID => 0x00,
            RegisterAddress::STATUS => 0x01,
            RegisterAddress::INPMUX => 0x02,
            RegisterAddress::PGA => 0x03,
            RegisterAddress::DATARATE => 0x04,
            RegisterAddress::REF => 0x05,
            RegisterAddress::IDACMAG => 0x06,
            RegisterAddress::IDACMUX => 0x07,
            RegisterAddress::VBIAS => 0x08,
            RegisterAddress::SYS => 0x09,
            RegisterAddress::OFCAL0 => 0x0A,
            RegisterAddress::OFCAL1 => 0x0B,
            RegisterAddress::OFCAL2 => 0x0C,
            RegisterAddress::FSCAL0 => 0x0D,
            RegisterAddress::FSCAL1 => 0x0E,
            RegisterAddress::FSCAL2 => 0x0F,
            RegisterAddress::GPIODAT => 0x10,
            RegisterAddress::GPIOCON => 0x11,
        }
    }

    /// Create register address from address value
    pub fn from_addr(bits: u8) -> Option<Self> {
        match bits {
            0x00 => Some(RegisterAddress::ID),
            0x01 => Some(RegisterAddress::STATUS),
            0x02 => Some(RegisterAddress::INPMUX),
            0x03 => Some(RegisterAddress::PGA),
            0x04 => Some(RegisterAddress::DATARATE),
            0x05 => Some(RegisterAddress::REF),
            0x06 => Some(RegisterAddress::IDACMAG),
            0x07 => Some(RegisterAddress::IDACMUX),
            0x08 => Some(RegisterAddress::VBIAS),
            0x09 => Some(RegisterAddress::SYS),
            0x0A => Some(RegisterAddress::OFCAL0),
            0x0B => Some(RegisterAddress::OFCAL1),
            0x0C => Some(RegisterAddress::OFCAL2),
            0x0D => Some(RegisterAddress::FSCAL0),
            0x0E => Some(RegisterAddress::FSCAL1),
            0x0F => Some(RegisterAddress::FSCAL2),
            0x10 => Some(RegisterAddress::GPIODAT),
            0x11 => Some(RegisterAddress::GPIOCON),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Copy, Default, Eq, PartialEq, Format)]
/// ADC data rate settings in samples per second
/// DataRate
pub enum DataRate{
    /// 2.5 samples per second
    /// SPS2_5

    SPS2_5,
    /// 5 samples per second
    /// SPS5

    SPS5,
    /// 10 samples per second
    /// SPS10

    SPS10,
    /// 16.6 samples per second
    /// SPS16_6

    SPS16_6,
    /// 20 samples per second (default)
    #[default]
    /// SPS20

    SPS20,
    /// 50 samples per second
    /// SPS50

    SPS50,
    /// 60 samples per second
    /// SPS60

    SPS60,
    /// 100 samples per second
    /// SPS100

    SPS100,
    /// 200 samples per second
    /// SPS200

    SPS200,
    /// 400 samples per second
    /// SPS400

    SPS400,
    /// 800 samples per second
    /// SPS800

    SPS800,
    /// 1000 samples per second
    /// SPS1000

    SPS1000,
    /// 2000 samples per second
    /// SPS2000

    SPS2000,
    /// 4000 samples per second
    /// SPS4000

    SPS4000,
}

impl DataRate {
    /// Get the sample period in milliseconds for this data rate
    pub fn sample_period_ms(&self) -> u32 {
        match self {
            DataRate::SPS2_5  => 400,   // 1/2.5 = 400ms
            DataRate::SPS5    => 200,   // 1/5 = 200ms  
            DataRate::SPS10   => 100,   // 1/10 = 100ms
            DataRate::SPS16_6 => 60,    // 1/16.6 ≈ 60ms
            DataRate::SPS20   => 50,    // 1/20 = 50ms
            DataRate::SPS50   => 20,    // 1/50 = 20ms
            DataRate::SPS60   => 17,    // 1/60 ≈ 17ms
            DataRate::SPS100  => 10,    // 1/100 = 10ms
            DataRate::SPS200  => 5,     // 1/200 = 5ms
            DataRate::SPS400  => 3,     // 1/400 ≈ 3ms (2.5ms rounded up)
            DataRate::SPS800  => 2,     // 1/800 ≈ 2ms (1.25ms rounded up)
            DataRate::SPS1000 => 1,     // 1/1000 = 1ms
            DataRate::SPS2000 => 1,     // 1/2000 = 0.5ms (rounded up to 1ms)
            DataRate::SPS4000 => 1,     // 1/4000 = 0.25ms (rounded up to 1ms)
        }
    }
}

impl ByteRepresentation for DataRate {
    fn bits(&self) -> u8{
        match self {
            DataRate::SPS2_5  => 0b0000,
            DataRate::SPS5    => 0b0001,
            DataRate::SPS10   => 0b0010,
            DataRate::SPS16_6 => 0b0011,
            DataRate::SPS20   => 0b0100,
            DataRate::SPS50   => 0b0101,
            DataRate::SPS60   => 0b0110,
            DataRate::SPS100  => 0b0111,
            DataRate::SPS200  => 0b1000,
            DataRate::SPS400  => 0b1001,
            DataRate::SPS800  => 0b1010,
            DataRate::SPS1000 => 0b1011,
            DataRate::SPS2000 => 0b1100,
            DataRate::SPS4000 => 0b1101,
        }
    }

    fn from_bits(bits: u8) -> Option<Self> {
        match bits & 0b1111 {
            0b0000 => Some(DataRate::SPS2_5),
            0b0001 => Some(DataRate::SPS5),
            0b0010 => Some(DataRate::SPS10),
            0b0011 => Some(DataRate::SPS16_6),
            0b0100 => Some(DataRate::SPS20),
            0b0101 => Some(DataRate::SPS50),
            0b0110 => Some(DataRate::SPS60),
            0b0111 => Some(DataRate::SPS100),
            0b1000 => Some(DataRate::SPS200),
            0b1001 => Some(DataRate::SPS400),
            0b1010 => Some(DataRate::SPS800),
            0b1011 => Some(DataRate::SPS1000),
            0b1100 => Some(DataRate::SPS2000),
            0b1101 => Some(DataRate::SPS4000),
            _ => None
        }
    }
}

/// Input multiplexer pin selection
#[derive(Debug, Clone, Copy, Eq, PartialEq, Format)]
/// Mux
pub enum Mux {
    /// Analog input 0
    /// AIN0

    AIN0,
    /// Analog input 1
    /// AIN1

    AIN1,
    /// Analog input 2
    /// AIN2

    AIN2,
    /// Analog input 3
    /// AIN3

    AIN3,
    /// Analog input 4
    /// AIN4

    AIN4,
    /// Analog input 5
    /// AIN5

    AIN5,
    /// Analog input 6
    /// AIN6

    AIN6,
    /// Analog input 7
    /// AIN7

    AIN7,
    /// Analog input 8
    /// AIN8

    AIN8,
    /// Analog input 9
    /// AIN9

    AIN9,
    /// Analog input 10
    /// AIN10

    AIN10,
    /// Analog input 11
    /// AIN11

    AIN11,
    /// Common analog input
    /// AINCOM

    AINCOM,
}

impl ByteRepresentation for Mux {
    fn bits(&self) -> u8 {
        match self {
            Mux::AIN0 => 0b0000,
            Mux::AIN1 => 0b0001,
            Mux::AIN2 => 0b0010,
            Mux::AIN3 => 0b0011,
            Mux::AIN4 => 0b0100,
            Mux::AIN5 => 0b0101,
            Mux::AIN6 => 0b0110,
            Mux::AIN7 => 0b0111,
            Mux::AIN8 => 0b1000,
            Mux::AIN9 => 0b1001,
            Mux::AIN10 => 0b1010,
            Mux::AIN11 => 0b1011,
            Mux::AINCOM => 0b1100,
        }
    }

    fn from_bits(bits: u8) -> Option<Self> {
        match bits {
            0b0000 => Some(Mux::AIN0),
            0b0001 => Some(Mux::AIN1),
            0b0010 => Some(Mux::AIN2),
            0b0011 => Some(Mux::AIN3),
            0b0100 => Some(Mux::AIN4),
            0b0101 => Some(Mux::AIN5),
            0b0110 => Some(Mux::AIN6),
            0b0111 => Some(Mux::AIN7),
            0b1000 => Some(Mux::AIN8),
            0b1001 => Some(Mux::AIN9),
            0b1010 => Some(Mux::AIN10),
            0b1011 => Some(Mux::AIN11),
            0b1100 => Some(Mux::AINCOM),
            _ => None,
        }
    }
}

/// IDAC output connection selection
#[derive(Debug, Clone, Copy, Default, Eq, PartialEq, Format)]
/// IDACMux
pub enum IDACMux {
    /// Connect to AIN0
    /// AIN0

    AIN0,
    /// Connect to AIN1
    /// AIN1

    AIN1,
    /// Connect to AIN2
    /// AIN2

    AIN2,
    /// Connect to AIN3
    /// AIN3

    AIN3,
    /// Connect to AIN4
    /// AIN4

    AIN4,
    /// Connect to AIN5
    /// AIN5

    AIN5,
    /// Connect to AIN6
    /// AIN6

    AIN6,
    /// Connect to AIN7
    /// AIN7

    AIN7,
    /// Connect to AIN8
    /// AIN8

    AIN8,
    /// Connect to AIN9
    /// AIN9

    AIN9,
    /// Connect to AIN10
    /// AIN10

    AIN10,
    /// Connect to AIN11
    /// AIN11

    AIN11,
    /// Connect to AINCOM
    /// AINCOM

    AINCOM,
    /// Disconnected (default)
    #[default]
    Disconnected
}

impl ByteRepresentation for IDACMux {
    fn bits(&self) -> u8 {
        match self {
            IDACMux::AIN0 => 0b0000,
            IDACMux::AIN1 => 0b0001,
            IDACMux::AIN2 => 0b0010,
            IDACMux::AIN3 => 0b0011,
            IDACMux::AIN4 => 0b0100,
            IDACMux::AIN5 => 0b0101,
            IDACMux::AIN6 => 0b0110,
            IDACMux::AIN7 => 0b0111,
            IDACMux::AIN8 => 0b1000,
            IDACMux::AIN9 => 0b1001,
            IDACMux::AIN10 => 0b1010,
            IDACMux::AIN11 => 0b1011,
            IDACMux::AINCOM => 0b1100,
            IDACMux::Disconnected => 0b1111,
        }
    }

    fn from_bits(bits: u8) -> Option<Self> {
        match bits {
            0b0000 => Some(IDACMux::AIN0),
            0b0001 => Some(IDACMux::AIN1),
            0b0010 => Some(IDACMux::AIN2),
            0b0011 => Some(IDACMux::AIN3),
            0b0100 => Some(IDACMux::AIN4),
            0b0101 => Some(IDACMux::AIN5),
            0b0110 => Some(IDACMux::AIN6),
            0b0111 => Some(IDACMux::AIN7),
            0b1000 => Some(IDACMux::AIN8),
            0b1001 => Some(IDACMux::AIN9),
            0b1010 => Some(IDACMux::AIN10),
            0b1011 => Some(IDACMux::AIN11),
            0b1100 => Some(IDACMux::AINCOM),
            _ => Some(IDACMux::Disconnected),
        }
    }
}

impl IDACMux {
    /// Create IDACMux from input multiplexer setting
    pub fn from_input_mux(mux: Mux) -> IDACMux {
        match mux {
            Mux::AIN0 => IDACMux::AIN0,
            Mux::AIN1 => IDACMux::AIN1,
            Mux::AIN2 => IDACMux::AIN2,
            Mux::AIN3 => IDACMux::AIN3,
            Mux::AIN4 => IDACMux::AIN4,
            Mux::AIN5 => IDACMux::AIN5,
            Mux::AIN6 => IDACMux::AIN6,
            Mux::AIN7 => IDACMux::AIN7,
            Mux::AIN8 => IDACMux::AIN8,
            Mux::AIN9 => IDACMux::AIN9,
            Mux::AIN10 => IDACMux::AIN10,
            Mux::AIN11 => IDACMux::AIN11,
            Mux::AINCOM => IDACMux::AINCOM,
        }
    }
}

#[derive(Debug, Clone, Copy, Default, Eq, PartialEq, Format)]
/// PGA conversion delay configuration.
/// 
/// This enum configures the delay between PGA settling and the start of conversion
/// as a multiple of the modulator period (TMOD). Longer delays allow for better
/// settling at higher PGA gains but reduce conversion speed.
pub enum PGAConversionDelay {
    #[default]
    /// 14 × modulator period delay (fastest settling, suitable for low PGA gains)
    TMODx14,
    /// 25 × modulator period delay (balanced settling time)
    TMODx25,
    /// 64 × modulator period delay (good settling for medium PGA gains)
    TMODx64,
    /// 256 × modulator period delay (longer settling for high PGA gains)
    TMODx256,
    /// 1024 × modulator period delay (maximum settling time for highest PGA gains)
    TMODx1024,
    /// TMODx4096

    TMODx4096,
    /// TMODx1

    TMODx1,
}

impl ByteRepresentation for PGAConversionDelay {
    fn bits(&self) -> u8 {
        match self {
            PGAConversionDelay::TMODx14 => 0b0000,
            PGAConversionDelay::TMODx25 => 0b0001,
            PGAConversionDelay::TMODx64 => 0b0010,
            PGAConversionDelay::TMODx256 => 0b0011,
            PGAConversionDelay::TMODx1024 => 0b0100,
            PGAConversionDelay::TMODx4096 => 0b0101,
            PGAConversionDelay::TMODx1 => 0b0110,
        }
    }

    fn from_bits(bits: u8) -> Option<Self> {
        match bits {
            0b0000 => Some(PGAConversionDelay::TMODx14),
            0b0001 => Some(PGAConversionDelay::TMODx25),
            0b0010 => Some(PGAConversionDelay::TMODx64),
            0b0011 => Some(PGAConversionDelay::TMODx256),
            0b0100 => Some(PGAConversionDelay::TMODx1024),
            0b0101 => Some(PGAConversionDelay::TMODx4096),
            0b0110 => Some(PGAConversionDelay::TMODx1),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Copy, Default, Eq, PartialEq, Format)]
/// Programmable gain amplifier gain settings
/// PGAGain
pub enum PGAGain {
    /// Gain of 1 (default)
    #[default]
    /// Gain1

    Gain1,
    /// Gain of 2
    /// Gain2

    Gain2,
    /// Gain of 4
    /// Gain4

    Gain4,
    /// Gain of 8
    /// Gain8

    Gain8,
    /// Gain of 16
    /// Gain16

    Gain16,
    /// Gain of 32
    /// Gain32

    Gain32,
    /// Gain of 64
    /// Gain64

    Gain64,
    /// Gain of 128
    /// Gain128

    Gain128,
}

impl PGAGain {
    /// Get the numeric gain value
    pub fn value(&self) -> u8 {
        match self {
            PGAGain::Gain1 => 1,
            PGAGain::Gain2 => 2,
            PGAGain::Gain4 => 4,
            PGAGain::Gain8 => 8,
            PGAGain::Gain16 => 16,
            PGAGain::Gain32 => 32,
            PGAGain::Gain64 => 64,
            PGAGain::Gain128 => 128,
        }
    }
}

impl ByteRepresentation for PGAGain {
    fn bits(&self) -> u8 {
        match self {
            PGAGain::Gain1 => 0b0000,
            PGAGain::Gain2 => 0b0001,
            PGAGain::Gain4 => 0b0010,
            PGAGain::Gain8 => 0b0011,
            PGAGain::Gain16 => 0b0100,
            PGAGain::Gain32 => 0b0101,
            PGAGain::Gain64 => 0b0110,
            PGAGain::Gain128 => 0b0111,
        }
    }

    fn from_bits(bits: u8) -> Option<Self> {
        match bits {
            0b0000 => Some(PGAGain::Gain1),
            0b0001 => Some(PGAGain::Gain2),
            0b0010 => Some(PGAGain::Gain4),
            0b0011 => Some(PGAGain::Gain8),
            0b0100 => Some(PGAGain::Gain16),
            0b0101 => Some(PGAGain::Gain32),
            0b0110 => Some(PGAGain::Gain64),
            0b0111 => Some(PGAGain::Gain128),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Copy, Default, Eq, PartialEq, Format)]
/// ClockSource
pub enum ClockSource {
    #[default]
    /// Internal

    Internal,
    /// External

    External,
}

impl ByteRepresentation for ClockSource {
    fn from_bits(bits: u8) -> Option<Self> {
        match bits {
            0b0000 => Some(ClockSource::Internal),
            0b0001 => Some(ClockSource::External),
            _ => None,
        }
    }

    fn bits(&self) -> u8 {
        match self {
            ClockSource::Internal => 0b0000,
            ClockSource::External => 0b0001,
        }
    }
}

#[derive(Debug, Clone, Copy, Default, Eq, PartialEq, Format)]
/// Mode
pub enum Mode {
    #[default]
    /// Continuous

    Continuous,
    /// SingleShot

    SingleShot,
}

impl ByteRepresentation for Mode {
    fn from_bits(bits: u8) -> Option<Self> {
        match bits {
            0b0000 => Some(Mode::Continuous),
            0b0001 => Some(Mode::SingleShot),
            _ => None,
        }
    }

    fn bits(&self) -> u8 {
        match self {
            Mode::Continuous => 0b0000,
            Mode::SingleShot => 0b0001,
        }
    }
}

#[derive(Debug, Clone, Copy, Default, Eq, PartialEq, Format)]
/// Digital filter configuration for the modulator.
/// 
/// Controls the type of digital filter applied to the modulator output to 
/// reduce noise and improve measurement accuracy.
pub enum Filter {
    /// SINC³ filter - Higher order filter providing excellent noise rejection
    /// with lower data rates. Optimal for precision measurements where speed
    /// is less critical.
    SINC3,
    #[default]
    /// Low-latency filter - Faster response time with moderate noise rejection.
    /// Suitable for applications requiring higher data throughput.
    LowLatency,
}

impl ByteRepresentation for Filter {
    fn from_bits(bits: u8) -> Option<Self> {
        match bits {
            0b0000 => Some(Filter::SINC3),
            0b0001 => Some(Filter::LowLatency),
            _ => None,
        }
    }

    fn bits(&self) -> u8 {
        match self {
            Filter::SINC3 => 0b0000,
            Filter::LowLatency => 0b0001,
        }
    }
}

#[derive(Debug, Clone, Copy, Default, Eq, PartialEq, Format)]
/// Reference voltage monitoring configuration.
/// 
/// Configures the monitoring of reference voltages to detect when they fall
/// outside acceptable thresholds, helping to ensure measurement accuracy.
pub enum ReferenceMonitorConfiguration {
    #[default]
    /// Reference monitoring disabled - no voltage monitoring performed
    Disabled,
    /// Monitor L0 reference with 0.3V threshold - detects low reference voltage
    L0MonitorThreshold03V,
    /// Monitor both L0 and L1 references with 0.3V and 1.3×(AVDD-AVSS) thresholds
    L0L1MonitorThreshold03V13AvddAvss,
    /// Monitor L0 reference with 10MΩ pull-together resistance and 0.3V threshold
    L0Monitor10mohmPullTogetherThreshold03V
}

impl ByteRepresentation for ReferenceMonitorConfiguration {
    fn from_bits(bits: u8) -> Option<Self> {
        match bits {
            0b00 => Some(ReferenceMonitorConfiguration::Disabled),
            0b01 => Some(ReferenceMonitorConfiguration::L0MonitorThreshold03V),
            0b10 => Some(ReferenceMonitorConfiguration::L0L1MonitorThreshold03V13AvddAvss),
            0b11 => Some(ReferenceMonitorConfiguration::L0Monitor10mohmPullTogetherThreshold03V),
            _ => None,
        }
    }

    fn bits(&self) -> u8 {
        match self {
            ReferenceMonitorConfiguration::Disabled => 0b00,
            ReferenceMonitorConfiguration::L0MonitorThreshold03V => 0b01,
            ReferenceMonitorConfiguration::L0L1MonitorThreshold03V13AvddAvss => 0b10,
            ReferenceMonitorConfiguration::L0Monitor10mohmPullTogetherThreshold03V => 0b11,
        }
    }
}

#[derive(Debug, Clone, Copy, Default, Eq, PartialEq, Format)]
/// ADC reference input selection
/// ReferenceInput
pub enum ReferenceInput {
    /// External reference using REFP0 and REFN0 pins (default)
    #[default]
    /// Refp0Refn0

    Refp0Refn0,
    /// External reference using REFP1 and REFN1 pins
    /// Refp1Refn1

    Refp1Refn1,
    /// Internal 2.5V reference
    Internal
}

impl ByteRepresentation for ReferenceInput {
    fn from_bits(bits: u8) -> Option<Self> {
        match bits {
            0b00 => Some(ReferenceInput::Refp0Refn0),
            0b01 => Some(ReferenceInput::Refp1Refn1),
            0b10 => Some(ReferenceInput::Internal),
            _ => None,
        }
    }

    fn bits(&self) -> u8 {
        match self {
            ReferenceInput::Refp0Refn0 => 0b00,
            ReferenceInput::Refp1Refn1 => 0b01,
            ReferenceInput::Internal => 0b10,
        }
    }
}

#[derive(Debug, Clone, Copy, Default, Eq, PartialEq, Format)]
/// Internal voltage reference power management configuration.
/// 
/// Controls how the internal 2.5V voltage reference is powered to balance
/// power consumption with reference stability and settling time.
pub enum InternalVoltageReferenceConfiguration {
    #[default]
    /// Internal reference disabled - use external reference only
    Off,
    /// Internal reference enabled but powers down between conversions to save power.
    /// Requires additional settling time when waking up.
    OnButPowersDown,
    /// Internal reference always powered on - fastest settling but higher power consumption.
    /// Recommended for continuous measurements.
    AlwaysOn,
}

impl ByteRepresentation for InternalVoltageReferenceConfiguration {
    fn from_bits(bits: u8) -> Option<Self> {
        match bits {
            0b00 => Some(InternalVoltageReferenceConfiguration::Off),
            0b01 => Some(InternalVoltageReferenceConfiguration::OnButPowersDown),
            0b10 => Some(InternalVoltageReferenceConfiguration::AlwaysOn),
            _ => None,
        }
    }

    fn bits(&self) -> u8 {
        match self {
            InternalVoltageReferenceConfiguration::Off => 0b00,
            InternalVoltageReferenceConfiguration::OnButPowersDown => 0b01,
            InternalVoltageReferenceConfiguration::AlwaysOn => 0b10,
        }
    }
}

#[derive(Debug, Clone, Copy, Default, Eq, PartialEq, Format)]
/// Excitation current source (IDAC) magnitude configuration.
/// 
/// Sets the magnitude of the current sources used to excite sensors like RTDs
/// (Resistance Temperature Detectors) and bridge sensors. Higher currents
/// provide better signal-to-noise ratio but may cause self-heating in sensors.
pub enum IDACMagnitude {
    #[default]
    /// Current sources disabled
    Off,
    /// 10 μA excitation current - minimal self-heating, suitable for very sensitive sensors
    Mag10uA,
    /// 50 μA excitation current - low self-heating for most RTD applications
    Mag50uA,
    /// 100 μA excitation current - common choice for PT100 RTDs
    Mag100uA,
    /// 250 μA excitation current - higher sensitivity for low-resistance sensors
    Mag250uA,
    /// 500 μA excitation current - good signal level for bridge sensors
    Mag500uA,
    /// 750 μA excitation current - high signal for robust measurements
    Mag750uA,
    /// 1000 μA (1 mA) excitation current - maximum sensitivity but consider self-heating
    Mag1000uA,
    /// 1500 μA (1.5 mA) excitation current - very high signal level
    Mag1500uA,
    /// 2000 μA (2 mA) excitation current - maximum available current
    Mag2000uA,
}

impl ByteRepresentation for IDACMagnitude {
    fn from_bits(bits: u8) -> Option<Self> {
        match bits {
            0b0000 => Some(IDACMagnitude::Off),
            0b0001 => Some(IDACMagnitude::Mag10uA),
            0b0010 => Some(IDACMagnitude::Mag50uA),
            0b0011 => Some(IDACMagnitude::Mag100uA),
            0b0100 => Some(IDACMagnitude::Mag250uA),
            0b0101 => Some(IDACMagnitude::Mag500uA),
            0b0110 => Some(IDACMagnitude::Mag750uA),
            0b0111 => Some(IDACMagnitude::Mag1000uA),
            0b1000 => Some(IDACMagnitude::Mag1500uA),
            0b1001 => Some(IDACMagnitude::Mag2000uA),
            _ => Some(IDACMagnitude::Off),
        }
    }

    fn bits(&self) -> u8 {
        match self {
            IDACMagnitude::Off => 0b0000,
            IDACMagnitude::Mag10uA => 0b0001,
            IDACMagnitude::Mag50uA => 0b0010,
            IDACMagnitude::Mag100uA => 0b0011,
            IDACMagnitude::Mag250uA => 0b0100,
            IDACMagnitude::Mag500uA => 0b0101,
            IDACMagnitude::Mag750uA => 0b0110,
            IDACMagnitude::Mag1000uA => 0b0111,
            IDACMagnitude::Mag1500uA => 0b1000,
            IDACMagnitude::Mag2000uA => 0b1001,
        }
    }
}

#[derive(Debug, Clone, Copy, Default, Eq, PartialEq, Format)]
/// Sensor bias voltage level configuration.
/// 
/// Sets the bias voltage applied to sensor inputs to keep them within the
/// optimal operating range of the ADC input stage.
pub enum VBiasLevel {
    #[default]
    /// Bias voltage set to (AVDD - AVSS) / 2 - mid-supply bias for bipolar signals
    AvddAvssBy2,
    /// Bias voltage set to (AVDD - AVSS) / 12 - lower bias voltage for unipolar signals
    AvddAvssBy12,
}

impl ByteRepresentation for VBiasLevel {
    fn from_bits(bits: u8) -> Option<Self> {
        match bits {
            0b0000 => Some(VBiasLevel::AvddAvssBy2),
            0b0001 => Some(VBiasLevel::AvddAvssBy12),
            _ => Some(VBiasLevel::AvddAvssBy2),
        }
    }

    fn bits(&self) -> u8 {
        match self {
            VBiasLevel::AvddAvssBy2 => 0b0000,
            VBiasLevel::AvddAvssBy12 => 0b0001,
        }
    }
}

#[derive(Debug, Clone, Copy, Eq, PartialEq, Default, Format)]
/// System monitoring and diagnostic configuration.
/// 
/// Enables various system monitoring functions for diagnostics and sensor
/// fault detection including temperature monitoring and burnout detection.
pub enum SystemMonitorConfiguration {
    #[default]
    /// System monitoring disabled - normal measurement mode
    Disabled,
    /// PGA inputs shorted to (AVDD-AVSS)/2 and disconnected from external pins
    /// for offset calibration
    PgaShortedToAvddAvssBy2AndDisconnected,
    /// Internal temperature sensor monitoring - measures die temperature
    InternalTemperatureSensor,
    /// Supply voltage monitoring - measures (AVDD-AVSS)/4 for power supply diagnostics
    AvddMinusAvssBy4Measurement,
    /// Digital supply monitoring - measures DVDD/4 for digital power diagnostics
    DvddBy4Measurement,
    /// Burnout current source 0.2 μA - detects open sensor connections
    BurnOutCurrentSourceEnabled0_2UA,
    /// Burnout current source 1 μA - higher sensitivity burnout detection
    BurnOutCurrentSourceEnabled1UA,
    /// Burnout current source 10 μA - maximum sensitivity burnout detection
    BurnOutCurrentSourceEnabled10UA,
}

impl ByteRepresentation for SystemMonitorConfiguration {
    fn bits(&self) -> u8 {
        match self {
            SystemMonitorConfiguration::Disabled => 0b000,
            SystemMonitorConfiguration::PgaShortedToAvddAvssBy2AndDisconnected => 0b001,
            SystemMonitorConfiguration::InternalTemperatureSensor => 0b010,
            SystemMonitorConfiguration::AvddMinusAvssBy4Measurement => 0b011,
            SystemMonitorConfiguration::DvddBy4Measurement => 0b100,
            SystemMonitorConfiguration::BurnOutCurrentSourceEnabled0_2UA => 0b101,
            SystemMonitorConfiguration::BurnOutCurrentSourceEnabled1UA => 0b110,
            SystemMonitorConfiguration::BurnOutCurrentSourceEnabled10UA => 0b111,
        }
    }

    fn from_bits(bits: u8) -> Option<Self> {
        match bits {
            0b000 => Some(SystemMonitorConfiguration::Disabled),
            0b001 => Some(SystemMonitorConfiguration::PgaShortedToAvddAvssBy2AndDisconnected),
            0b010 => Some(SystemMonitorConfiguration::InternalTemperatureSensor),
            0b011 => Some(SystemMonitorConfiguration::AvddMinusAvssBy4Measurement),
            0b100 => Some(SystemMonitorConfiguration::DvddBy4Measurement),
            0b101 => Some(SystemMonitorConfiguration::BurnOutCurrentSourceEnabled0_2UA),
            0b110 => Some(SystemMonitorConfiguration::BurnOutCurrentSourceEnabled1UA),
            0b111 => Some(SystemMonitorConfiguration::BurnOutCurrentSourceEnabled10UA),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Copy, Eq, PartialEq, Default, Format)]
/// CalibrationSampleSize
pub enum CalibrationSampleSize {
    /// Samples1

    Samples1,
    /// Samples4

    Samples4,
    #[default]
    /// Samples8

    Samples8,
    /// Samples16

    Samples16,
}

impl ByteRepresentation for CalibrationSampleSize {
    fn from_bits(bits: u8) -> Option<Self> {
        match bits {
            0b00 => Some(CalibrationSampleSize::Samples1),
            0b01 => Some(CalibrationSampleSize::Samples4),
            0b10 => Some(CalibrationSampleSize::Samples8),
            0b11 => Some(CalibrationSampleSize::Samples16),
            _ => None,
        }
    }

    fn bits(&self) -> u8 {
        match self {
            CalibrationSampleSize::Samples1 => 0b00,
            CalibrationSampleSize::Samples4 => 0b01,
            CalibrationSampleSize::Samples8 => 0b10,
            CalibrationSampleSize::Samples16 => 0b11,
        }
    }
}

#[derive(Debug, Clone, Copy, Eq, PartialEq, Format)]
/// GpioDirection
pub enum GpioDirection {
    /// INPUT

    INPUT,
    /// OUTPUT

    OUTPUT,
}

#[derive(Debug, Clone, Copy, Eq, PartialEq, Format)]
/// GpioConfiguration
pub enum GpioConfiguration {
    /// AnalogInput

    AnalogInput,
    /// GPIO

    GPIO,
}

#[derive(Debug, Clone, Copy, Eq, PartialEq, Format)]
/// DeviceId
pub enum DeviceId {
    /// ADS124S08

    ADS124S08,
    /// ADS124S06

    ADS124S06,
}

impl ByteRepresentation for DeviceId {
    fn bits(&self) -> u8 {
        match self {
            DeviceId::ADS124S08 => 0b000,
            DeviceId::ADS124S06 => 0b001,
        }
    }

    fn from_bits(bits: u8) -> Option<Self> {
        match bits & 0b111 {
            0b000 => Some(DeviceId::ADS124S08),
            0b001 => Some(DeviceId::ADS124S06),
            _ => None,
        }
    }
}


#[derive(Debug, Clone, Copy, Eq, PartialEq, Format)]
/// StatusRegisterValue
pub struct StatusRegisterValue {
    fl_por: bool,
    n_rdy: bool,
    fl_p_railp: bool,
    fl_p_railn: bool,
    fl_n_railp: bool,
    fl_n_railn: bool,
    fl_ref_l1: bool,
    fl_ref_l0: bool,
}

impl ByteRepresentation for StatusRegisterValue {
    fn from_bits(data: u8) -> Option<StatusRegisterValue> {
        Some(StatusRegisterValue {
            fl_por: data & 0b1000_0000 != 0,
            n_rdy: data & 0b0100_0000 != 0,
            fl_p_railp: data & 0b0010_0000 != 0,
            fl_p_railn: data & 0b0001_0000 != 0,
            fl_n_railp: data & 0b0000_1000 != 0,
            fl_n_railn: data & 0b0000_0100 != 0,
            fl_ref_l1: data & 0b0000_0010 != 0,
            fl_ref_l0: data & 0b0000_0001 != 0
        })
    }

    fn bits(&self) -> u8 {
        let mut bits = 0b0000_0000;
        bits |= if self.fl_por { 0b1000_0000 } else { 0b0000_0000 };
        bits |= if self.n_rdy { 0b0100_0000 } else { 0b0000_0000 };
        bits |= if self.fl_p_railp { 0b0010_0000 } else { 0b0000_0000 };
        bits |= if self.fl_p_railn { 0b0001_0000 } else { 0b0000_0000 };
        bits |= if self.fl_n_railp { 0b0000_1000 } else { 0b0000_0000 };
        bits |= if self.fl_n_railn { 0b0000_0100 } else { 0b0000_0000 };
        bits |= if self.fl_ref_l1 { 0b0000_0010 } else { 0b0000_0000 };
        bits |= if self.fl_ref_l0 { 0b0000_0001 } else { 0b0000_0000 };
        bits
    }
}

impl StatusRegisterValue {
    /// Check if the ADC is ready for conversion
    pub fn ready(&self) -> bool {
        !self.n_rdy
    }
}

#[derive(Debug, Clone, Copy, Eq, PartialEq, Default, Format)]
/// System control register configuration.
/// 
/// Controls system-level features including monitoring functions, calibration,
/// timeout behavior, CRC checking, and status reporting.
pub struct SystemControlRegister {
    /// System monitoring configuration - enables diagnostic measurements
    pub sys_mon: SystemMonitorConfiguration,
    /// Calibration sample size - number of samples used for calibration
    pub cal_samp: CalibrationSampleSize,
    /// SPI timeout enable - enables timeout detection for SPI communication
    pub timeout: bool,
    /// CRC checksum enable - adds CRC to conversion data for error detection
    pub crc: bool,
    /// Status byte enable - includes status information in conversion results
    pub sendstat: bool,
}

impl ByteRepresentation for SystemControlRegister {
    fn bits(&self) -> u8 {
        let mut bits = 0b0000_0000;
        bits |= self.sys_mon.bits() << 5;
        bits |= self.cal_samp.bits() << 3;
        bits |= if self.timeout { 0b0000_0100 } else { 0b0000_0000 };
        bits |= if self.crc { 0b0000_0010 } else { 0b0000_0000 };
        bits |= if self.sendstat { 0b0000_0001 } else { 0b0000_0000 };
        bits
    }

    fn from_bits(bits: u8) -> Option<SystemControlRegister> {
        Some(SystemControlRegister {
            sys_mon: SystemMonitorConfiguration::from_bits(bits >> 5).unwrap(),
            cal_samp: CalibrationSampleSize::from_bits(bits >> 3 & 0b11).unwrap(),
            timeout: bits & 0b0000_0100 != 0,
            crc: bits & 0b0000_0010 != 0,
            sendstat: bits & 0b0000_0001 != 0,
        })
    }
}

impl SystemControlRegister {
    /// Create a copy with modified system monitor configuration
    pub fn copy_with_configuration(&self, sys_mon: SystemMonitorConfiguration) -> SystemControlRegister {
        SystemControlRegister {
            sys_mon,
            cal_samp: self.cal_samp,
            timeout: self.timeout,
            crc: self.crc,
            sendstat: self.sendstat,
        }
    }
}

#[derive(Debug, Clone, Copy, Eq, PartialEq, Format)]
/// Input multiplexer register configuration.
/// 
/// Selects which analog input pins are connected to the positive and negative
/// inputs of the programmable gain amplifier for differential measurements.
pub struct InputMultiplexerRegister {
    /// Positive input multiplexer selection - selects the positive input pin
    pub p: Mux,
    /// Negative input multiplexer selection - selects the negative input pin
    pub n: Mux,
}

impl Default for InputMultiplexerRegister {
    fn default() -> Self {
        InputMultiplexerRegister {
            p: Mux::AIN0,
            n: Mux::AIN1,
        }
    }
}

impl ByteRepresentation for InputMultiplexerRegister {
    fn bits(&self) -> u8 {
        let mut bits = 0b0000_0000;
        bits |= self.p.bits() << 4;
        bits |= self.n.bits();
        bits
    }

    fn from_bits(bits: u8) -> Option<InputMultiplexerRegister> {
        Some(InputMultiplexerRegister {
            p: Mux::from_bits(bits >> 4)?,
            n: Mux::from_bits(bits & 0b1111)?,
        })
    }
}

#[derive(Debug, Clone, Copy, Format)]
/// DeviceIdRegister
pub struct DeviceIdRegister {
    /// device_id

    pub device_id: DeviceId,
}

impl ByteRepresentation for DeviceIdRegister {
    fn bits(&self) -> u8 {
        self.device_id.bits()
    }

    fn from_bits(bits: u8) -> Option<DeviceIdRegister> {
        Some(DeviceIdRegister {
            device_id: DeviceId::from_bits(bits)?,
        })
    }
}

#[derive(Debug, Clone, Copy, Eq, PartialEq, Default, Format)]
/// PgaRegister
pub struct PgaRegister {
    /// gain

    pub gain: PGAGain,
    /// enable

    pub enable: bool,
    /// delay

    pub delay: PGAConversionDelay,
}

impl ByteRepresentation for PgaRegister {
    fn bits(&self) -> u8 {
        let mut bits = 0b0000_0000;
        bits |= self.gain.bits();
        bits |= if self.enable { 0b000_1000 } else { 0b0000_0000 };
        bits |= self.delay.bits() << 5;
        bits
    }

    fn from_bits(bits: u8) -> Option<PgaRegister> {
        Some(PgaRegister {
            gain: PGAGain::from_bits(bits >> 4)?,
            enable: bits & 0b0000_1000 != 0,
            delay: PGAConversionDelay::from_bits(bits & 0b111)?,
        })
    }
}

#[derive(Debug, Clone, Copy, Eq, PartialEq, Default, Format)]
/// Data rate and conversion configuration register.
/// 
/// Controls the ADC conversion parameters including data rate, filter type,
/// conversion mode, clock source, and global chopping for improved accuracy.
pub struct DataRateRegister {
    /// Global chopping enable - alternates input polarity to reduce offset and drift
    pub g_chop: bool,
    /// Clock source selection - internal or external clock reference
    pub clock: ClockSource,
    /// Conversion mode - continuous or single-shot measurements
    pub mode: Mode,
    /// Digital filter type - SINC³ or low-latency filter
    pub filter: Filter,
    /// Data rate selection - conversion speed in samples per second
    pub rate: DataRate,
}

impl ByteRepresentation for DataRateRegister {
    fn bits(&self) -> u8 {
        let mut bits = 0b0000_0000;
        bits |= if self.g_chop { 0b1000_0000 } else { 0b0000_0000 };
        bits |= self.clock.bits() << 6;
        bits |= self.mode.bits() << 4;
        bits |= self.filter.bits() << 3;
        bits |= self.rate.bits();
        bits
    }

    fn from_bits(bits: u8) -> Option<DataRateRegister> {
        Some(DataRateRegister {
            g_chop: bits & 0b1000_0000 != 0,
            clock: ClockSource::from_bits(bits >> 6)?,
            mode: Mode::from_bits(bits >> 4 & 0b1)?,
            filter: Filter::from_bits(bits >> 3 & 0b1)?,
            rate: DataRate::from_bits(bits & 0b1111)?,
        })
    }
}

#[derive(Debug, Clone, Copy, Eq, PartialEq, Format)]
/// Reference voltage control register configuration.
/// 
/// Controls the voltage reference system including monitoring, buffer configuration,
/// reference source selection, and internal reference power management.
pub struct ReferenceControlRegister {
    /// Reference monitoring configuration - enables voltage monitoring and fault detection
    pub fl_ref_en: ReferenceMonitorConfiguration,
    /// Positive reference buffer disable - when true, disables REFP input buffer
    pub n_refp_buf: bool,
    /// Negative reference buffer disable - when true, disables REFN input buffer  
    pub n_refn_buf: bool,
    /// Reference input selection - chooses between internal and external references
    pub refsel: ReferenceInput,
    /// Internal voltage reference configuration - controls internal 2.5V reference
    pub refcon: InternalVoltageReferenceConfiguration,
}

impl Default for ReferenceControlRegister {
    fn default() -> Self {
        ReferenceControlRegister {
            fl_ref_en: ReferenceMonitorConfiguration::Disabled,
            n_refp_buf: false,
            n_refn_buf: true,
            refsel: ReferenceInput::default(),
            refcon: InternalVoltageReferenceConfiguration::default(),
        }
    }
}

impl ByteRepresentation for ReferenceControlRegister {
    fn bits(&self) -> u8 {
        let mut bits = 0b0000_0000;
        bits |= self.fl_ref_en.bits() << 6;
        bits |= if self.n_refp_buf { 0b0100_0000 } else { 0b0000_0000 };
        bits |= if self.n_refn_buf { 0b0010_0000 } else { 0b0000_0000 };
        bits |= self.refsel.bits() << 2;
        bits |= self.refcon.bits();
        bits
    }

    fn from_bits(bits: u8) -> Option<ReferenceControlRegister> {
        Some(ReferenceControlRegister {
            fl_ref_en: ReferenceMonitorConfiguration::from_bits((bits >> 6) & 0b11)?,
            n_refp_buf: bits & 0b0100_0000 != 0,
            n_refn_buf: bits & 0b0010_0000 != 0,
            refsel: ReferenceInput::from_bits((bits >> 2) & 0b11)?,
            refcon: InternalVoltageReferenceConfiguration::from_bits(bits & 0b11)?,
        })
    }
}

#[derive(Debug, Clone, Copy, Eq, PartialEq, Default, Format)]
/// IDAC magnitude and control register configuration.
/// 
/// Controls the excitation current sources including magnitude, power switch
/// control, and rail detection for sensor excitation applications.
pub struct IDACMagnitudeRegister {
    /// Supply rail detection enable - monitors if IDAC output hits supply rails
    pub fl_rail_en: bool,
    /// Power switch control - enables/disables external power switch control
    pub psw: bool,
    /// IDAC magnitude setting - sets the excitation current amplitude
    pub imag: IDACMagnitude,
}

impl ByteRepresentation for IDACMagnitudeRegister {
    fn bits(&self) -> u8 {
        let mut bits = 0b0000_0000;
        bits |= if self.fl_rail_en { 0b1000_0000 } else { 0b0000_0000 };
        bits |= if self.psw { 0b0100_0000 } else { 0b0000_0000 };
        bits |= self.imag.bits();
        bits
    }

    fn from_bits(bits: u8) -> Option<IDACMagnitudeRegister> {
        Some(IDACMagnitudeRegister {
            fl_rail_en: bits & 0b1000_0000 != 0,
            psw: bits & 0b0100_0000 != 0,
            imag: IDACMagnitude::from_bits(bits & 0b1111)?,
        })
    }
}

#[derive(Debug, Clone, Copy, Eq, PartialEq, Default, Format)]
/// IDAC multiplexer register configuration.
/// 
/// Controls the routing of the two excitation current sources (IDAC1 and IDAC2)
/// to the analog input pins for sensor excitation applications.
pub struct IDACMultiplexerRegister {
    /// IDAC2 multiplexer selection - routes second current source to selected pin
    pub i2mux: IDACMux,
    /// IDAC1 multiplexer selection - routes first current source to selected pin  
    pub i1mux: IDACMux,
}

impl ByteRepresentation for IDACMultiplexerRegister {
    fn bits(&self) -> u8 {
        let mut bits = 0b0000_0000;
        bits |= self.i2mux.bits() << 4;
        bits |= self.i1mux.bits();
        bits
    }

    fn from_bits(bits: u8) -> Option<IDACMultiplexerRegister> {
        Some(IDACMultiplexerRegister {
            i2mux: IDACMux::from_bits(bits >> 4)?,
            i1mux: IDACMux::from_bits(bits & 0b1111)?,
        })
    }
}

#[derive(Debug, Clone, Copy, Eq, PartialEq, Default, Format)]
/// Sensor biasing register configuration.
/// 
/// Controls bias voltage application to individual analog input pins to keep
/// sensor signals within the optimal input range of the ADC front-end.
pub struct SensorBiasingRegister {
    /// Bias voltage level selection - sets the bias voltage magnitude
    pub vbias: VBiasLevel,
    /// Bias enable for AINCOM pin - applies bias voltage to common input
    pub vb_ainc: bool,
    /// Bias enable for AIN5 pin - applies bias voltage to analog input 5
    pub vb_ain5: bool,
    /// Bias enable for AIN4 pin - applies bias voltage to analog input 4
    pub vb_ain4: bool,
    /// Bias enable for AIN3 pin - applies bias voltage to analog input 3
    pub vb_ain3: bool,
    /// Bias enable for AIN2 pin - applies bias voltage to analog input 2
    pub vb_ain2: bool,
    /// Bias enable for AIN1 pin - applies bias voltage to analog input 1
    pub vb_ain1: bool,
    /// Bias enable for AIN0 pin - applies bias voltage to analog input 0
    pub vb_ain0: bool,
}

impl ByteRepresentation for SensorBiasingRegister {
    fn bits(&self) -> u8 {
        let mut bits = 0b0000_0000;
        bits |= self.vbias.bits() << 7;
        bits |= if self.vb_ainc { 0b0100_0000 } else { 0b0000_0000 };
        bits |= if self.vb_ain5 { 0b0010_0000 } else { 0b0000_0000 };
        bits |= if self.vb_ain4 { 0b0001_0000 } else { 0b0000_0000 };
        bits |= if self.vb_ain3 { 0b0000_1000 } else { 0b0000_0000 };
        bits |= if self.vb_ain2 { 0b0000_0100 } else { 0b0000_0000 };
        bits |= if self.vb_ain1 { 0b0000_0010 } else { 0b0000_0000 };
        bits |= if self.vb_ain0 { 0b0000_0001 } else { 0b0000_0000 };
        bits
    }

    fn from_bits(bits: u8) -> Option<SensorBiasingRegister> {
        Some(SensorBiasingRegister {
            vbias: VBiasLevel::from_bits(bits >> 7)?,
            vb_ainc: bits & 0b0100_0000 != 0,
            vb_ain5: bits & 0b0010_0000 != 0,
            vb_ain4: bits & 0b0001_0000 != 0,
            vb_ain3: bits & 0b0000_1000 != 0,
            vb_ain2: bits & 0b0000_0100 != 0,
            vb_ain1: bits & 0b0000_0010 != 0,
            vb_ain0: bits & 0b0000_0001 != 0,
        })
    }
}

#[derive(Debug, Clone, Eq, PartialEq, Default, Format)]
/// Complete set of ADS124S08 configuration registers
/// ConfigurationRegisters
pub struct ConfigurationRegisters {
    /// Input multiplexer register
    /// inpmux

    pub inpmux: InputMultiplexerRegister,
    /// PGA register
    /// pga

    pub pga: PgaRegister,
    /// Data rate register
    /// datarate

    pub datarate: DataRateRegister,
    /// Reference control register
    /// refctrl

    pub refctrl: ReferenceControlRegister,
    /// IDAC magnitude register
    /// idacmag

    pub idacmag: IDACMagnitudeRegister,
    /// IDAC multiplexer register
    /// idacmux

    pub idacmux: IDACMultiplexerRegister,
    /// Sensor biasing register
    /// vbias

    pub vbias: SensorBiasingRegister,
    /// System control register
    /// sys

    pub sys: SystemControlRegister,
}