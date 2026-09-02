//! Where the GS3's panel content sits inside the bezel's aperture, and what it may put there.

#[cfg(feature = "sequential-storage")]
use crc::{CRC_32_ISCSI, Crc};
#[cfg(feature = "sequential-storage")]
use postcard::{from_bytes_crc32, to_slice_crc32};
#[cfg(feature = "sequential-storage")]
use sequential_storage::map::{SerializationError, Value};

/// The top-left of the 396x111 window the GS3's bezel leaves visible, on its 428x168 panel.
///
/// **A stored setting rather than a constant, because it is a fact about one machine.** Where
/// an aperture actually sits is judged by eye from in front of the machine, and a
/// rebuild-and-flash cycle is the wrong instrument for a judgement that takes seconds.
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
/// `u8` per axis. The travel is 32 px across and 57 down -- the panel minus the window -- and
/// the drawing crate clamps whatever it is handed, so a value stored by a firmware with a
/// different window size cannot push content off the panel. That clamp is not theoretical: the
/// window went from 390x115 to 396x111 and every machine's stored trim survived it, out of
/// range on one axis and merely wrong on the other.
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
    /// What a machine shows until somebody trims it: centred on the panel.
    ///
    /// It used to be `(25, 34)` -- the value the firmware had always drawn at, kept because a
    /// default that moves the picture on the first boot after an update reads as a fault. That
    /// argument ran out when the window changed shape: `(25, 34)` was 6 px and 8 px in from
    /// the centre of a 390x115 window and is 9 px and 6 px off the centre of a 396x111 one, so
    /// it is no longer the trim anybody chose. A fresh machine starts centred and gets trimmed
    /// from the menu; a machine that has already been trimmed keeps its stored value and wants
    /// re-trimming.
    ///
    /// Must agree with `variegated_gs3_panel::Window::DEFAULT`, which derives the same two
    /// numbers from the panel and window sizes.
    pub const DEFAULT: Self = Self { x: 16, y: 28 };
}

impl Default for PanelOrigin {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Which optional data points the GS3's routine screen may draw.
///
/// The screen fills four slots from a ranking, and these five are the ranks an operator can
/// switch off -- every one of them a measurement whose sensor may well have a display of its
/// own, in which case mirroring it on the panel spends a slot to say something already on the
/// bench. Total time and total input are not here: they are the machine's own arithmetic and
/// nothing else shows them.
///
/// **Off hides the rank, never the role.** A quantity the pump is currently targeting or
/// capping, or the one the running step exits on, is drawn whatever this says -- those are
/// what the machine is doing right now, not a reading the operator chose to follow.
///
/// A settings key of its own rather than a field on `Configuration`, for the reason
/// [`PanelOrigin`] gives at length: that blob is postcard-positional with no version field, so
/// appending to it silently resets every setpoint on the machine. This one has the same second
/// reason too -- which figures are worth panel space depends on what is sitting next to the
/// machine, so there is nothing for the wire to carry.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PanelDataPoints {
    /// Weight in the cup.
    pub weight: bool,
    /// Pressure at the group.
    pub pressure: bool,
    /// Flow through the group.
    pub flow: bool,
    /// Conductivity leaving the group.
    pub conductivity: bool,
    /// Temperature leaving the group.
    pub output_temperature: bool,
}

impl PanelDataPoints {
    /// Everything on.
    ///
    /// A machine that has never been told otherwise shows what it can measure. The setting
    /// exists to *remove* a figure the operator can already read elsewhere, so the default has
    /// to be the state where nothing has been removed -- and a fresh machine's owner has not
    /// yet said what is on their bench.
    pub const DEFAULT: Self = Self {
        weight: true,
        pressure: true,
        flow: true,
        conductivity: true,
        output_temperature: true,
    };
}

impl Default for PanelDataPoints {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Mirrors the impl on [`PanelOrigin`]; see the note there.
#[cfg(feature = "sequential-storage")]
impl<'a> Value<'a> for PanelDataPoints {
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

    /// The default centres the window on the panel.
    ///
    /// Written out rather than computed, so that it and the arithmetic in
    /// `variegated_gs3_panel::Window::DEFAULT` have to be changed together: this crate cannot
    /// see the window size, and two defaults that disagree would put a never-trimmed machine's
    /// picture somewhere neither crate intended.
    #[test]
    fn the_default_centres_the_window() {
        // (428 - 396) / 2, (168 - 111) / 2.
        assert_eq!(PanelOrigin::default(), PanelOrigin { x: 16, y: 28 });
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

    /// A machine that has never been told otherwise draws everything it can measure.
    #[test]
    fn every_data_point_starts_on() {
        let d = PanelDataPoints::default();
        assert!(d.weight && d.pressure && d.flow && d.conductivity && d.output_temperature);
    }

    /// The same round trip, and for the same reason: a decode failure resets the operator's
    /// choices to "show everything" without saying so.
    #[cfg(feature = "sequential-storage")]
    #[test]
    fn the_data_points_survive_a_round_trip() {
        // Deliberately not the default, and not all-off either -- a mixed value is the one a
        // field-order mistake would scramble without changing the byte count.
        let points = PanelDataPoints {
            weight: false,
            pressure: true,
            flow: false,
            conductivity: true,
            output_temperature: false,
        };
        let mut buffer = [0u8; 32];
        let written = points.serialize_into(&mut buffer).unwrap();
        let (read, _) = PanelDataPoints::deserialize_from(&buffer[..written]).unwrap();
        assert_eq!(read, points);
    }
}
