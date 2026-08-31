//! Where the GS3's panel content sits inside the bezel's aperture.

#[cfg(feature = "sequential-storage")]
use crc::{CRC_32_ISCSI, Crc};
#[cfg(feature = "sequential-storage")]
use postcard::{from_bytes_crc32, to_slice_crc32};
#[cfg(feature = "sequential-storage")]
use sequential_storage::map::{SerializationError, Value};

/// The top-left of the 390x115 window the GS3's bezel leaves visible, on its 428x168 panel.
///
/// **A stored setting rather than a constant, because it is a fact about one machine.** The
/// firmware has shipped `(25, 34)` since the panel was brought up and nothing records whether
/// that was measured off the bezel or guessed; where an aperture actually sits is judged by
/// eye from in front of the machine, and a rebuild-and-flash cycle is the wrong instrument
/// for a judgement that takes seconds.
///
/// **Not a field on `Configuration`, and this is the whole reason it is a type of its own.**
/// That struct is postcard-positional with no version field, and it crosses the
/// inter-processor UART, the browser's WebSocket, ESPHome and the Plantlet uplink; appending
/// to the *stored* configuration blob would make every previously written copy fail to
/// deserialize, which `SettingsStorage::load_settings` maps to `Default` -- a silent reset of
/// every setpoint, PID tuning, Kalman parameter and pump calibration on the machine, to buy
/// two bytes. So it gets a settings key of its own, exactly as the timezone does and for the
/// same reason, and one more: a bezel alignment cannot be judged from a browser, so there is
/// nothing for the wire to carry.
///
/// `u8` per axis. The travel is 38 px across and 53 down -- the panel minus the window -- and
/// the drawing crate clamps whatever it is handed, so a value from a firmware with a
/// different window size cannot push content off the panel.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PanelOrigin {
    /// Pixels in from the left edge of the panel.
    pub x: u8,
    /// Pixels down from the top edge of the panel.
    pub y: u8,
}

impl PanelOrigin {
    /// What a machine shows until somebody trims it.
    ///
    /// The value the firmware has always drawn at. Deliberately *not* the specification's
    /// centred `(19, 26)`: this is what every machine in the field already looks like, and a
    /// setting whose default moves the picture on the first boot after an update is a setting
    /// that reads as a fault.
    pub const DEFAULT: Self = Self { x: 25, y: 34 };
}

impl Default for PanelOrigin {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Mirrors the impl on `TimezoneSetting`; see the note there.
#[cfg(feature = "sequential-storage")]
impl<'a> Value<'a> for PanelOrigin {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        let crc = Crc::<u32>::new(&CRC_32_ISCSI);

        match to_slice_crc32(self, buffer, crc.digest()) {
            Ok(bytes) => Ok(bytes.len()),
            Err(postcard::Error::SerializeBufferFull) => Err(SerializationError::BufferTooSmall),
            Err(_) => Err(SerializationError::InvalidData),
        }
    }

    fn deserialize_from(buffer: &'a [u8]) -> Result<(Self, usize), SerializationError>
    where
        Self: Sized,
    {
        let crc = Crc::<u32>::new(&CRC_32_ISCSI);

        match from_bytes_crc32(buffer, crc.digest()) {
            Ok(value) => Ok((value, buffer.len())),
            Err(_) => Err(SerializationError::InvalidFormat),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The default is the value the panel has always drawn at. A machine that has never been
    /// trimmed must look exactly as it did before this setting existed.
    #[test]
    fn the_default_is_what_the_firmware_already_drew() {
        assert_eq!(PanelOrigin::default(), PanelOrigin { x: 25, y: 34 });
    }

    /// A round trip through the stored form, since a decode failure here is a silent reset to
    /// the default rather than anything that reports itself.
    #[cfg(feature = "sequential-storage")]
    #[test]
    fn it_survives_a_round_trip() {
        let origin = PanelOrigin { x: 19, y: 26 };
        let mut buffer = [0u8; 32];
        let written = origin.serialize_into(&mut buffer).unwrap();
        let (read, _) = PanelOrigin::deserialize_from(&buffer[..written]).unwrap();
        assert_eq!(read, origin);
    }
}
