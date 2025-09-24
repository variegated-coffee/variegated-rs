//! Display variant trait

use display_interface::{AsyncWriteOnlyDataCommand, DisplayError};

/// Trait to represent a specific display variant
pub trait DisplayVariant {
    /// Width of display
    const WIDTH: u16;
    /// Height of display
    const HEIGHT: u16;
    /// Column offset
    const COLUMN_OFFSET: u16 = 0;
    /// Row offset
    const ROW_OFFSET: u16 = 0;

    /// Get integral dimensions from DisplaySize
    fn dimensions() -> (u16, u16) {
        (Self::WIDTH, Self::HEIGHT)
    }

    /// Calculate buffer size in bytes for RGB565
    fn buffer_size() -> usize {
        (Self::WIDTH as usize) * (Self::HEIGHT as usize) * 2
    }

    /// Initialize the display
    async fn init<DI>(iface: &mut DI) -> Result<(), DisplayError>
    where
        DI: AsyncWriteOnlyDataCommand;
}