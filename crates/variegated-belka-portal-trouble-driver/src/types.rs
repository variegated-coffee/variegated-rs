use trouble_host::prelude::*;

/// Service UUID for Belka Portal device (short UUID 0x7400)
pub const BELKA_SERVICE_UUID: Uuid = Uuid::new_short(0x7400);

/// Measurement characteristic UUID (short UUID 0x7410)
pub const MEASUREMENT_CHAR_UUID: Uuid = Uuid::new_short(0x7410);

/// Command characteristic UUID (short UUID 0x7420)
///
/// Written, never read or subscribed. The Portal acknowledges the *write* and nothing else --
/// there is no reply saying a command was understood, so a payload it does not recognise is
/// indistinguishable from one it never received.
pub const COMMAND_CHAR_UUID: Uuid = Uuid::new_short(0x7420);

/// Switch the Portal's display to its graph view.
///
/// # These are bytes, not a number
///
/// **Do not turn this into a `u32`, and do not "fix" the byte order.** It is written here as
/// the four bytes that are known to work, in the order they went out on the wire.
///
/// The provenance: typed into nRF Connect's write field, which sends the bytes as entered, and
/// observed in Apple PacketLogger as `Write Request - Handle:0x002F - Value: 4000 012D` on a
/// write the Portal acted on.
///
/// The temptation to swap them is real, because the Portal's *notification* payload is three
/// little-endian `f32`s -- see [`Measurements::parse`] -- so a reader who assumes the device is
/// little-endian throughout would reach for `0x2D, 0x01, 0x00, 0x40` and be wrong. A command
/// characteristic is not obliged to share an encoding with a measurement characteristic, and
/// this one does not.
///
/// Nothing downstream can catch the mistake. There is no acknowledgement of meaning, no
/// echo, and no error: the Portal would simply keep showing the wrong screen.
pub const SHOW_GRAPH: [u8; 4] = [0x40, 0x00, 0x01, 0x2D];

/// Leave the graph view. See [`SHOW_GRAPH`] for why this is a byte array.
///
/// One byte apart from its opposite, which is worth knowing when reading a capture: a
/// transposition between the two is a single-bit difference and looks like nothing at a glance.
pub const HIDE_GRAPH: [u8; 4] = [0x40, 0x00, 0x01, 0x2E];

/// Measurements from the Belka Portal device
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Measurements {
    /// Electrical conductivity (f32, little-endian, bytes 0-3)
    pub ec: f32,
    /// Internal temperature (presumably) in degrees Celsius (f32, little-endian, bytes 4-7)
    pub internal_temperature: f32,
    /// Temperature in degrees Celsius (f32, little-endian, bytes 8-11)
    pub temperature: f32,
    /// Status byte (byte 12)
    pub battery: u8,
}

impl Measurements {
    /// Parse measurements from a 13-byte array
    ///
    /// Format:
    /// - Bytes 0-3: EC as f32 little-endian
    /// - Bytes 4-7: Internal temperature(?) as f32 little-endian
    /// - Bytes 8-11: Temperature as f32 little-endian
    /// - Byte 12: Battery level byte
    pub fn parse(data: &[u8]) -> Result<Self, crate::Error> {
        if data.len() < 13 {
            return Err(crate::Error::InvalidDataLength);
        }

        // Parse f32 values in little-endian
        let ec = f32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        let internal_temperature = f32::from_le_bytes([data[4], data[5], data[6], data[7]]);
        let temperature = f32::from_le_bytes([data[8], data[9], data[10], data[11]]);
        let battery = data[12];

        Ok(Self {
            ec,
            internal_temperature,
            temperature,
            battery,
        })
    }
}
