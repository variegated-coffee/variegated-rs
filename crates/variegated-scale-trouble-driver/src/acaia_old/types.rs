use trouble_host::prelude::*;

/// Service UUID for ACAIA Old protocol (Weight Scale Service - standard UUID)
pub const ACAIA_OLD_SERVICE_UUID: Uuid = Uuid::new_short(0x1820);

/// Characteristic UUID for ACAIA Old protocol (Weight Measurement)
pub const ACAIA_OLD_CHAR_UUID: Uuid = Uuid::new_short(0x2a80);

/// Identification message (20 bytes) sent during handshake
/// Based on AcaiaArduinoBLE library format
pub const IDENTIFICATION_MSG: [u8; 20] = [
    0xEF, 0xDD, 0x0B,  // header + cmd
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,  // payload "01234567"
    0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34,        // payload "8901234"
    0x9A, 0x6D,  // checksums
];

/// Notification request message (14 bytes) sent during handshake
/// Based on AcaiaArduinoBLE library format
/// Requests: weight(01), battery(02), timer(05), button(04), key(15)
pub const NOTIFICATION_REQUEST_MSG: [u8; 14] = [
    0xEF, 0xDD, 0x0C, 0x09,  // header + cmd + length (9 bytes payload)
    0x00, 0x01, 0x01, 0x02, 0x02, 0x05, 0x03, 0x04, 0x15,  // payload
    0x06,  // checksum
];

/// Tare command (7 bytes)
/// Format: header(2) + cmd(1) + len(1) + payload(1) + checksums(2)
pub const TARE_CMD: [u8; 7] = [
    0xEF, 0xDD, 0x04, 0x01,  // header + cmd + length (1 byte payload)
    0x00,  // payload
    0x00, 0x00,  // checksums
];

/// Heartbeat message (7 bytes) - must be sent every 1000ms (recommended)
/// Based on AcaiaArduinoBLE library format
pub const HEARTBEAT_MSG: [u8; 7] = [
    0xEF, 0xDD, 0x00, 0x02,  // header + cmd + length
    0x00, 0x02,              // payload
    0x00,                    // checksum
];

/// Events that can be received from the ACAIA Old protocol scale
///
/// Note: Old Acaia protocol only sends weight events. Timer and battery
/// notifications are not supported in this protocol variant.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ScaleEvent {
    /// Weight measurement update
    Weight(WeightMeasurement),
}

/// Weight measurement from the scale
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct WeightMeasurement {
    /// Weight in grams
    pub weight: f32,
}

impl WeightMeasurement {
    /// Parse weight measurement from NEW Acaia protocol payload
    ///
    /// NEW format event payload (after message type byte 0x05):
    /// - Bytes 0-1: Raw weight as u16 little-endian
    /// - Bytes 2-3: Unused
    /// - Byte 4: Scale/unit index (0-4: units, tenths, hundredths, thousandths, ten-thousandths)
    /// - Byte 5: Flags byte (bit 1 = negative)
    pub fn parse_new(payload: &[u8]) -> Result<Self, crate::acaia_old::Error> {
        if payload.len() < 6 {
            return Err(crate::acaia_old::Error::InvalidFrameLength);
        }

        // Extract raw weight (bytes 0-1, little-endian)
        let raw_weight = u16::from_le_bytes([payload[0], payload[1]]) as u32;

        // Extract scale index (byte 4)
        let scale_index = payload[4] as u32;

        // Validate scale index (must be 0-4 to avoid overflow)
        if scale_index > 4 {
            defmt::warn!("Invalid scale index {}", scale_index);
            return Err(crate::acaia_old::Error::ParseError);
        }

        // Extract sign from flags byte (byte 5, bit 1)
        // NEW Acaia: bit 1 set = negative
        let is_negative = (payload[5] & 0x02) != 0;

        // Calculate divisor (10^scale_index)
        let divisor = 10u32.pow(scale_index);

        // Calculate final weight
        let weight = (raw_weight as f32) / (divisor as f32);
        let weight = if is_negative { -weight } else { weight };

        Ok(Self { weight })
    }

    /// Parse weight measurement from OLD Acaia protocol payload
    ///
    /// Old Acaia frame format: [0xEF, 0xDD, weight_lo, weight_hi, ?, ?, scale, sign, ...]
    /// Payload (starting at frame byte 2):
    /// - Bytes 0-1: Raw weight as u16 little-endian
    /// - Byte 4: Scale/unit index (0-4: units, tenths, hundredths, thousandths, ten-thousandths)
    /// - Byte 5: Sign byte (0x00 = positive, non-zero = negative)
    pub fn parse_old(payload: &[u8]) -> Result<Self, crate::acaia_old::Error> {
        if payload.len() < 6 {
            return Err(crate::acaia_old::Error::InvalidFrameLength);
        }

        // Extract raw weight (bytes 0-1, little-endian)
        let raw_weight = u16::from_le_bytes([payload[0], payload[1]]) as u32;

        // Extract scale index (byte 4)
        let scale_index = payload[4] as u32;

        // Validate scale index (must be 0-4 to avoid overflow)
        if scale_index > 4 {
            defmt::warn!("Invalid scale index {}", scale_index);
            return Err(crate::acaia_old::Error::ParseError);
        }

        // Extract sign byte (byte 5)
        // Old Acaia: 0x00 = positive, non-zero = negative
        let is_negative = payload[5] != 0x00;

        // Calculate divisor (10^scale_index)
        let divisor = 10u32.pow(scale_index);

        // Calculate final weight
        let weight = (raw_weight as f32) / (divisor as f32);
        let weight = if is_negative { -weight } else { weight };

        Ok(Self { weight })
    }
}

