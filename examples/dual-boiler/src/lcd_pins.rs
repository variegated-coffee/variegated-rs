//! Which MCP23017 pins the character LCD hangs off, and what to do with them when there
//! is no LCD.
//!
//! Separate from [`crate::mcp23017_hd44780`] because that module is only compiled with
//! the `character-display` feature, and the pin map is needed either way: with the
//! feature the driver drives these lines, without it [`park_low`] parks them. Keeping
//! one copy is the point -- two lists of the same twelve pins would drift, and the
//! failure would be a display that half works or a line left floating.

use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c as AsyncI2c;
use variegated_mcp23017::{Error, Mcp23017, PinDirection};

/// HD44780 control and data pins on the LCD MCP23017 (I2C address 0x21).
///
/// Port A carries the control lines, port B the 8-bit data bus.
///
/// `dead_code` is allowed because this is a *map of the hardware*, not a set of things
/// that happen to be called: which pin is EN is true whether or not this build drives it,
/// and without `character-display` only the two range constants below are read. Deleting
/// or gating the rest to silence the warning would leave the map incomplete, which is the
/// one thing it exists not to be.
#[allow(dead_code)]
pub mod pins {
    // Control pins on Port A (bits in the port register)
    pub const RS_BIT: u8 = 0; // GPA0 - Register Select
    pub const RW_BIT: u8 = 1; // GPA1 - Read/Write
    pub const EN_BIT: u8 = 2; // GPA2 - Enable
    pub const BACKLIGHT_BIT: u8 = 3; // GPA3 - Backlight

    // Port A bit masks for efficient operations
    pub const RS_MASK: u8 = 1 << RS_BIT;
    pub const RW_MASK: u8 = 1 << RW_BIT;
    pub const EN_MASK: u8 = 1 << EN_BIT;
    pub const BACKLIGHT_MASK: u8 = 1 << BACKLIGHT_BIT;

    // All control pins mask
    pub const CONTROL_MASK: u8 = RS_MASK | RW_MASK | EN_MASK | BACKLIGHT_MASK;

    /// Highest port-A pin the LCD uses; port A pins above this are not ours to touch.
    pub const CONTROL_PIN_COUNT: u8 = 4;
    /// Port B pin numbers, in the expander's flat 0..16 numbering.
    pub const DATA_PINS: core::ops::Range<u8> = 8..16;
}

/// Drive every LCD line low and leave it there.
///
/// For builds without `character-display`. The expander comes out of
/// [`Mcp23017::init`] with all sixteen pins as *inputs*, which leaves these twelve
/// floating: nothing drives them, so they sit at whatever the board's leakage and any
/// unpopulated LCD's pull-ups decide. A floating HD44780 enable line is the one that
/// matters -- noise on EN clocks whatever the data bus happens to read as into a display
/// that is meant to be inert.
///
/// Low rather than high, and that includes the backlight on GPA3: this is the state that
/// asserts nothing. RS, RW and EN idle low on a real HD44780, so a display that *is*
/// fitted on a build without the feature stays quiet rather than being driven into an
/// undefined mode.
///
/// The write comes before the direction change on purpose. Writing GPIO sets the output
/// latch while the pins are still inputs, so by the time they become outputs the latch
/// already holds zero and the pin never briefly drives a stale value. Doing it the other
/// way round would flip the direction first and drive whatever the latch happened to
/// contain.
pub async fn park_low<I2C, D>(mcp: &mut Mcp23017<I2C, D>) -> Result<(), Error<I2C::Error>>
where
    I2C: AsyncI2c,
    D: DelayNs,
{
    mcp.write_port_a(0x00).await?;
    mcp.write_port_b(0x00).await?;

    for pin in 0..pins::CONTROL_PIN_COUNT {
        mcp.set_pin_direction(pin, PinDirection::Output).await?;
    }
    for pin in pins::DATA_PINS {
        mcp.set_pin_direction(pin, PinDirection::Output).await?;
    }

    Ok(())
}
