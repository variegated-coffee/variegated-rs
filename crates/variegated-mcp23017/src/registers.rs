//! MCP23017 register definitions and constants

/// MCP23017 register addresses
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Register {
    /// Port A data direction register (1 = input, 0 = output)
    IoDirA = 0x00,
    /// Port B data direction register (1 = input, 0 = output)
    IoDirB = 0x01,
    /// Port A input polarity register (1 = inverted, 0 = same)
    IPolA = 0x02,
    /// Port B input polarity register (1 = inverted, 0 = same)
    IPolB = 0x03,
    /// Port A interrupt-on-change enable register
    GpIntEnA = 0x04,
    /// Port B interrupt-on-change enable register
    GpIntEnB = 0x05,
    /// Port A default compare register for interrupt-on-change
    DefValA = 0x06,
    /// Port B default compare register for interrupt-on-change
    DefValB = 0x07,
    /// Port A interrupt control register (1 = compare to DEFVAL, 0 = compare to previous)
    IntConA = 0x08,
    /// Port B interrupt control register (1 = compare to DEFVAL, 0 = compare to previous)
    IntConB = 0x09,
    /// Configuration register
    IoCon = 0x0A,
    /// Configuration register (mirror of 0x0A)
    IoConMirror = 0x0B,
    /// Port A pull-up enable register (1 = enabled, 0 = disabled)
    GpPuA = 0x0C,
    /// Port B pull-up enable register (1 = enabled, 0 = disabled)
    GpPuB = 0x0D,
    /// Port A interrupt flag register (read-only)
    IntfA = 0x0E,
    /// Port B interrupt flag register (read-only)
    IntfB = 0x0F,
    /// Port A interrupt capture register (read-only)
    IntCapA = 0x10,
    /// Port B interrupt capture register (read-only)
    IntCapB = 0x11,
    /// Port A GPIO register
    GpioA = 0x12,
    /// Port B GPIO register
    GpioB = 0x13,
    /// Port A output latch register
    OLatA = 0x14,
    /// Port B output latch register
    OLatB = 0x15,
}

impl Register {
    pub const fn addr(&self) -> u8 {
        *self as u8
    }
}

/// IOCON register bits
pub mod iocon {
    /// Interrupt polarity bit (1 = active-high, 0 = active-low)
    pub const INTPOL: u8 = 1 << 1;
    /// Interrupt output configuration (1 = open-drain, 0 = active driver)
    pub const ODR: u8 = 1 << 2;
    /// Hardware address enable bit (1 = enable, 0 = disable)
    pub const HAEN: u8 = 1 << 3;
    /// Slew rate control bit for SDA output (1 = disabled, 0 = enabled)
    pub const DISSLW: u8 = 1 << 4;
    /// Sequential operation mode bit (1 = sequential disabled, 0 = enabled)
    pub const SEQOP: u8 = 1 << 5;
    /// Interrupt mirror bit (1 = INT pins are connected, 0 = separate)
    pub const MIRROR: u8 = 1 << 6;
    /// Bank bit (1 = separate banks, 0 = sequential registers)
    pub const BANK: u8 = 1 << 7;
}

/// Port enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Port {
    /// Port A (pins 0-7)
    A,
    /// Port B (pins 8-15)
    B,
}

impl Port {
    /// Get the GPIO register for this port
    pub const fn gpio_register(&self) -> Register {
        match self {
            Port::A => Register::GpioA,
            Port::B => Register::GpioB,
        }
    }

    /// Get the IODIR register for this port
    pub const fn iodir_register(&self) -> Register {
        match self {
            Port::A => Register::IoDirA,
            Port::B => Register::IoDirB,
        }
    }

    /// Get the GPPU register for this port
    pub const fn gppu_register(&self) -> Register {
        match self {
            Port::A => Register::GpPuA,
            Port::B => Register::GpPuB,
        }
    }

    /// Get the GPINTEN register for this port
    pub const fn gpinten_register(&self) -> Register {
        match self {
            Port::A => Register::GpIntEnA,
            Port::B => Register::GpIntEnB,
        }
    }

    /// Get the INTF register for this port
    pub const fn intf_register(&self) -> Register {
        match self {
            Port::A => Register::IntfA,
            Port::B => Register::IntfB,
        }
    }

    /// Get the INTCAP register for this port
    pub const fn intcap_register(&self) -> Register {
        match self {
            Port::A => Register::IntCapA,
            Port::B => Register::IntCapB,
        }
    }
}

/// Pin configuration for interrupt behavior
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InterruptMode {
    /// No interrupt
    Disabled,
    /// Interrupt on change from previous value
    OnChange,
    /// Interrupt when pin differs from default value
    OnDefault(bool),
}

/// Default I2C address for MCP23017 (when A0=A1=A2=0)
pub const DEFAULT_ADDRESS: u8 = 0x20;