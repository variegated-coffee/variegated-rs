//! NV3007 display variants and initialization sequences

use crate::command::{Command, PixelFormat};
use crate::display::DisplayVariant;
use display_interface::{AsyncWriteOnlyDataCommand, DisplayError};
use embassy_time::Timer;

/// NV3007 display variant types
#[derive(Debug, Clone, Copy)]
pub enum Nv3007Variant {
    /// Standard variant for 1.65"/1.68" panels
    Standard,
    /// 279 variant for 2.79" panels (often works better with various panels)
    Variant279,
}

impl Default for Nv3007Variant {
    fn default() -> Self {
        Self::Standard
    }
}

/// Standard NV3007 168x428 display
#[derive(Debug, Clone, Copy)]
pub struct Nv3007_168_428 {
    /// Display variant to use
    pub variant: Nv3007Variant,
}

impl Default for Nv3007_168_428 {
    fn default() -> Self {
        Self {
            variant: Nv3007Variant::Standard,
        }
    }
}

impl DisplayVariant for Nv3007_168_428 {
    const WIDTH: u16 = 168;
    const HEIGHT: u16 = 428;

    async fn init<DI>(iface: &mut DI) -> Result<(), DisplayError>
    where
        DI: AsyncWriteOnlyDataCommand,
    {
        // This method signature is kept for compatibility
        // Default to standard variant
        Self::init_with_variant(iface, Nv3007Variant::Standard).await
    }
}

impl Nv3007_168_428 {
    /// Initialize the display with a specific variant
    pub async fn init_with_variant<DI>(iface: &mut DI, variant: Nv3007Variant) -> Result<(), DisplayError>
    where
        DI: AsyncWriteOnlyDataCommand,
    {
        // Enable private registers
        Command::PrivateRegisterEnable(0xA5).send(iface).await?;

        // Sleep out first
        Command::SleepOut.send(iface).await?;
        Timer::after_millis(120).await;

        // Initialize with appropriate sequence based on variant
        match variant {
            Nv3007Variant::Standard => init_standard_sequence(iface).await?,
            Nv3007Variant::Variant279 => init_279_sequence(iface).await?,
        }

        // Set pixel format to RGB565
        Command::PixelFormat(PixelFormat::Rgb565).send(iface).await?;

        // Sleep out again after configuration
        Command::SleepOut.send(iface).await?;
        Timer::after_millis(200).await;

        // Display on
        Command::DisplayOn.send(iface).await?;
        Timer::after_millis(150).await;

        Ok(())
    }

    /// Initialize with the variant specified in the struct
    pub async fn init_self<DI>(&self, iface: &mut DI) -> Result<(), DisplayError>
    where
        DI: AsyncWriteOnlyDataCommand,
    {
        Self::init_with_variant(iface, self.variant).await
    }
}

/// Alternative NV3007 279 variant initialization
#[derive(Debug, Clone, Copy)]
pub struct Nv3007_279 {}

impl DisplayVariant for Nv3007_279 {
    const WIDTH: u16 = 168;
    const HEIGHT: u16 = 428;

    async fn init<DI>(iface: &mut DI) -> Result<(), DisplayError>
    where
        DI: AsyncWriteOnlyDataCommand,
    {
        // Enable private registers
        Command::PrivateRegisterEnable(0xA5).send(iface).await?;

        // Initialize with 279 variant sequence
        init_279_sequence(iface).await?;

        // Set pixel format to RGB565
        Command::PixelFormat(PixelFormat::Rgb565).send(iface).await?;

        // Sleep out
        Command::SleepOut.send(iface).await?;
        Timer::after_millis(120).await;

        // Display on
        Command::DisplayOn.send(iface).await?;

        Ok(())
    }
}

/// Standard initialization sequence
async fn init_standard_sequence<DI>(iface: &mut DI) -> Result<(), DisplayError>
where
    DI: AsyncWriteOnlyDataCommand,
{
    // Based on NV3007_init_operations from Arduino_GFX

    // Power and timing settings
    Command::PrivateRegister { reg: 0x9a, data: &[0x08] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x9b, data: &[0x08] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x9c, data: &[0xb0] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x9d, data: &[0x17] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x9e, data: &[0xc2] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x8f, data: &[0x22, 0x04] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x84, data: &[0x90] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x83, data: &[0x7B] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x85, data: &[0x4F] }.send(iface).await?;

    // Gamma settings (positive)
    Command::PrivateRegister { reg: 0x6e, data: &[0x0f] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x7e, data: &[0x0f] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x60, data: &[0x00] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x70, data: &[0x00] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x6d, data: &[0x39] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x7d, data: &[0x31] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x61, data: &[0x0A] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x71, data: &[0x0A] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x6c, data: &[0x35] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x7c, data: &[0x29] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x62, data: &[0x0F] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x72, data: &[0x0F] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x68, data: &[0x4f] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x78, data: &[0x45] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x66, data: &[0x33] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x76, data: &[0x33] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x6b, data: &[0x14] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x7b, data: &[0x14] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x63, data: &[0x09] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x73, data: &[0x09] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x6a, data: &[0x13] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x7a, data: &[0x16] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x64, data: &[0x08] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x74, data: &[0x08] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x69, data: &[0x07] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x79, data: &[0x0d] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x65, data: &[0x05] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x75, data: &[0x05] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x67, data: &[0x33] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x77, data: &[0x33] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x6f, data: &[0x00] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x7f, data: &[0x00] }.send(iface).await?;

    // Additional configuration
    Command::PrivateRegister { reg: 0x50, data: &[0x00] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x52, data: &[0xd6] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x53, data: &[0x04] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x54, data: &[0x04] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x55, data: &[0x1b] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x56, data: &[0x1b] }.send(iface).await?;

    // More configuration registers
    Command::PrivateRegister { reg: 0xa0, data: &[0x2a, 0x24, 0x00] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xa1, data: &[0x84] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xa2, data: &[0x85] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xa8, data: &[0x34] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xa9, data: &[0x80] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xaa, data: &[0x73] }.send(iface).await?;

    // Tearing effect and other settings
    Command::PrivateRegister { reg: 0x35, data: &[0x00] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x44, data: &[0x00, 0x10] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x46, data: &[0x10] }.send(iface).await?;

    // Disable private registers
    Command::PrivateRegisterEnable(0x00).send(iface).await?;

    Ok(())
}

/// 279 variant initialization sequence
async fn init_279_sequence<DI>(iface: &mut DI) -> Result<(), DisplayError>
where
    DI: AsyncWriteOnlyDataCommand,
{
    // Based on NV3007_279_init_operations from Arduino_GFX

    // Power and timing settings
    Command::PrivateRegister { reg: 0x9a, data: &[0x08] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x9b, data: &[0x08] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x9c, data: &[0xb0] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x9d, data: &[0x16] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x9e, data: &[0xc4] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x8f, data: &[0x55, 0x04] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x84, data: &[0x90] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x83, data: &[0x7b] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x85, data: &[0x33] }.send(iface).await?;

    // Gamma settings (simplified from Arduino_GFX)
    Command::PrivateRegister { reg: 0x60, data: &[0x00] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x70, data: &[0x00] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x61, data: &[0x02] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x71, data: &[0x02] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x62, data: &[0x04] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x72, data: &[0x04] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x6c, data: &[0x29] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x7c, data: &[0x29] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x6d, data: &[0x31] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x7d, data: &[0x31] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x6e, data: &[0x0f] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x7e, data: &[0x0f] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x66, data: &[0x21] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x76, data: &[0x21] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x68, data: &[0x3a] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x78, data: &[0x3a] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x63, data: &[0x07] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x73, data: &[0x07] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x64, data: &[0x05] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x74, data: &[0x05] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x65, data: &[0x02] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x75, data: &[0x02] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x67, data: &[0x23] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x77, data: &[0x23] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x69, data: &[0x08] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x79, data: &[0x08] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x6a, data: &[0x13] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x7a, data: &[0x13] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x6b, data: &[0x13] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x7b, data: &[0x13] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x6f, data: &[0x00] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x7f, data: &[0x00] }.send(iface).await?;

    // Additional configuration
    Command::PrivateRegister { reg: 0x50, data: &[0x00] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x52, data: &[0xd6] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x53, data: &[0x08] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x54, data: &[0x08] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x55, data: &[0x1e] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x56, data: &[0x1c] }.send(iface).await?;

    // GOA control registers
    Command::PrivateRegister { reg: 0xa0, data: &[0x2b, 0x24, 0x00] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xa1, data: &[0x87] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xa2, data: &[0x86] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xa5, data: &[0x00] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xa6, data: &[0x00] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xa7, data: &[0x00] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xa8, data: &[0x36] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xa9, data: &[0x7e] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xaa, data: &[0x7e] }.send(iface).await?;

    // Clock control registers
    Command::PrivateRegister { reg: 0xb9, data: &[0x85] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xba, data: &[0x84] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xbb, data: &[0x83] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xbc, data: &[0x82] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xbd, data: &[0x81] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xbe, data: &[0x80] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xbf, data: &[0x01] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xc0, data: &[0x02] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xc1, data: &[0x00] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xc2, data: &[0x00] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xc3, data: &[0x00] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xc4, data: &[0x33] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xc5, data: &[0x7e] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xc6, data: &[0x7e] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xc8, data: &[0x33, 0x33] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xc9, data: &[0x68] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xca, data: &[0x69] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xcb, data: &[0x6a] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xcc, data: &[0x6b] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xcd, data: &[0x33, 0x33] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xce, data: &[0x6c] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xcf, data: &[0x6d] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xd0, data: &[0x6e] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xd1, data: &[0x6f] }.send(iface).await?;

    // Additional GOA settings
    Command::PrivateRegister { reg: 0xab, data: &[0x03, 0x67] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xac, data: &[0x03, 0x6b] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xad, data: &[0x03, 0x68] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xae, data: &[0x03, 0x6c] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xb3, data: &[0x00] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xb4, data: &[0x00] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xb5, data: &[0x00] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xb6, data: &[0x32] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xb7, data: &[0x7e] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xb8, data: &[0x7e] }.send(iface).await?;

    // Source control registers
    Command::PrivateRegister { reg: 0xe0, data: &[0x00] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xe1, data: &[0x03, 0x0f] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xe2, data: &[0x04] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xe3, data: &[0x01] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xe4, data: &[0x0e] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xe5, data: &[0x01] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xe6, data: &[0x19] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xe7, data: &[0x10] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xe8, data: &[0x10] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xea, data: &[0x12] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xeb, data: &[0xd0] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xec, data: &[0x04] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xed, data: &[0x07] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xee, data: &[0x07] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xef, data: &[0x09] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xf0, data: &[0xd0] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xf1, data: &[0x0e] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xf9, data: &[0x17] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xf2, data: &[0x2c, 0x1b, 0x0b, 0x20] }.send(iface).await?;
    Command::PrivateRegister { reg: 0xe9, data: &[0x29] }.send(iface).await?;

    // Tearing effect and other settings
    Command::PrivateRegister { reg: 0x35, data: &[0x00] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x44, data: &[0x00, 0x10] }.send(iface).await?;
    Command::PrivateRegister { reg: 0x46, data: &[0x10] }.send(iface).await?;

    // Disable private registers
    Command::PrivateRegisterEnable(0x00).send(iface).await?;

    Ok(())
}