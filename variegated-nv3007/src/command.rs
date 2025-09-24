//! NV3007 Commands

use display_interface::{AsyncWriteOnlyDataCommand, DataFormat, DisplayError};

/// NV3007 command opcodes
#[allow(dead_code)]
pub mod opcodes {
    /// Sleep in
    pub const SLPIN: u8 = 0x10;
    /// Sleep out
    pub const SLPOUT: u8 = 0x11;
    /// Partial mode on
    pub const PTLON: u8 = 0x12;
    /// Normal display mode on
    pub const NORON: u8 = 0x13;
    /// Display inversion off
    pub const INVOFF: u8 = 0x20;
    /// Display inversion on
    pub const INVON: u8 = 0x21;
    /// Display off
    pub const DISPOFF: u8 = 0x28;
    /// Display on
    pub const DISPON: u8 = 0x29;
    /// Column address set
    pub const CASET: u8 = 0x2A;
    /// Row address set
    pub const RASET: u8 = 0x2B;
    /// Memory write
    pub const RAMWR: u8 = 0x2C;
    /// Memory read
    pub const RAMRD: u8 = 0x2E;
    /// Partial area
    pub const PTLAR: u8 = 0x30;
    /// Vertical scrolling definition
    pub const VSCRDEF: u8 = 0x33;
    /// Tearing effect line off
    pub const TEOFF: u8 = 0x34;
    /// Tearing effect line on
    pub const TEON: u8 = 0x35;
    /// Memory access control
    pub const MADCTL: u8 = 0x36;
    /// Vertical scrolling start address
    pub const VSCSAD: u8 = 0x37;
    /// Idle mode off
    pub const IDMOFF: u8 = 0x38;
    /// Idle mode on
    pub const IDMON: u8 = 0x39;
    /// Pixel format set
    pub const COLMOD: u8 = 0x3A;
    /// Write memory continue
    pub const WRMEMC: u8 = 0x3C;
    /// Set tear scanline
    pub const STE: u8 = 0x44;
    /// Get scanline
    pub const GSCAN: u8 = 0x45;
    /// Private register enable
    pub const PRVEN: u8 = 0xFF;
}

/// MADCTL (Memory Access Control) bits
#[allow(dead_code)]
pub mod madctl {
    /// Row address order
    pub const MY: u8 = 0x80;
    /// Column address order
    pub const MX: u8 = 0x40;
    /// Row/Column exchange
    pub const MV: u8 = 0x20;
    /// Vertical refresh order
    pub const ML: u8 = 0x10;
    /// RGB/BGR order
    pub const RGB: u8 = 0x00;
    /// BGR order
    pub const BGR: u8 = 0x08;
}

/// Commands for NV3007
#[derive(Debug, Clone)]
pub enum Command {
    /// Sleep in
    SleepIn,
    /// Sleep out
    SleepOut,
    /// Display inversion off
    InvertOff,
    /// Display inversion on
    InvertOn,
    /// Display off
    DisplayOff,
    /// Display on
    DisplayOn,
    /// Set column address range
    ColumnAddressSet {
        /// Start column
        start: u16,
        /// End column
        end: u16,
    },
    /// Set row address range
    RowAddressSet {
        /// Start row
        start: u16,
        /// End row
        end: u16,
    },
    /// Start memory write
    MemoryWrite,
    /// Memory access control (rotation)
    MemoryAccessControl(u8),
    /// Set pixel format
    PixelFormat(PixelFormat),
    /// Enable private registers (0xA5 to enable)
    PrivateRegisterEnable(u8),
    /// Write to a private register
    PrivateRegister {
        /// Register address
        reg: u8,
        /// Data to write
        data: &'static [u8],
    },
}

/// Pixel format options
#[derive(Debug, Clone, Copy)]
pub enum PixelFormat {
    /// 16-bit RGB565
    Rgb565 = 0x55,
    /// 18-bit RGB666
    Rgb666 = 0x66,
}

impl Command {
    /// Send command to NV3007
    pub async fn send<DI>(&self, iface: &mut DI) -> Result<(), DisplayError>
    where
        DI: AsyncWriteOnlyDataCommand,
    {
        match self {
            Command::SleepIn => {
                iface.send_commands(DataFormat::U8(&[opcodes::SLPIN])).await
            }
            Command::SleepOut => {
                iface.send_commands(DataFormat::U8(&[opcodes::SLPOUT])).await
            }
            Command::InvertOff => {
                iface.send_commands(DataFormat::U8(&[opcodes::INVOFF])).await
            }
            Command::InvertOn => {
                iface.send_commands(DataFormat::U8(&[opcodes::INVON])).await
            }
            Command::DisplayOff => {
                iface.send_commands(DataFormat::U8(&[opcodes::DISPOFF])).await
            }
            Command::DisplayOn => {
                iface.send_commands(DataFormat::U8(&[opcodes::DISPON])).await
            }
            Command::ColumnAddressSet { start, end } => {
                iface.send_commands(DataFormat::U8(&[opcodes::CASET])).await?;
                let data = [
                    (*start >> 8) as u8,
                    (*start & 0xff) as u8,
                    (*end >> 8) as u8,
                    (*end & 0xff) as u8,
                ];
                iface.send_data(DataFormat::U8(&data)).await
            }
            Command::RowAddressSet { start, end } => {
                iface.send_commands(DataFormat::U8(&[opcodes::RASET])).await?;
                let data = [
                    (*start >> 8) as u8,
                    (*start & 0xff) as u8,
                    (*end >> 8) as u8,
                    (*end & 0xff) as u8,
                ];
                iface.send_data(DataFormat::U8(&data)).await
            }
            Command::MemoryWrite => {
                iface.send_commands(DataFormat::U8(&[opcodes::RAMWR])).await
            }
            Command::MemoryAccessControl(value) => {
                iface.send_commands(DataFormat::U8(&[opcodes::MADCTL])).await?;
                iface.send_data(DataFormat::U8(&[*value])).await
            }
            Command::PixelFormat(format) => {
                iface.send_commands(DataFormat::U8(&[opcodes::COLMOD])).await?;
                iface.send_data(DataFormat::U8(&[*format as u8])).await
            }
            Command::PrivateRegisterEnable(value) => {
                iface.send_commands(DataFormat::U8(&[opcodes::PRVEN])).await?;
                iface.send_data(DataFormat::U8(&[*value])).await
            }
            Command::PrivateRegister { reg, data } => {
                iface.send_commands(DataFormat::U8(&[*reg])).await?;
                iface.send_data(DataFormat::U8(data)).await
            }
        }
    }
}