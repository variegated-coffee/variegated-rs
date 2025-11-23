use trouble_host::prelude::*;

/// Service UUID for Belka Portal device (short UUID 0x7400)
pub const BELKA_SERVICE_UUID: Uuid = Uuid::new_short(0x7400);

/// Measurement characteristic UUID (short UUID 0x7410)
pub const MEASUREMENT_CHAR_UUID: Uuid = Uuid::new_short(0x7410);

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
