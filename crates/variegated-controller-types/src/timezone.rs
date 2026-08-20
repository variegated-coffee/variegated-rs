//! The machine's timezone, as a stored and published setting.

#[cfg(feature = "sequential-storage")]
use sequential_storage::map::{SerializationError, Value};
#[cfg(feature = "sequential-storage")]
use postcard::{from_bytes_crc32, to_slice_crc32};
#[cfg(feature = "sequential-storage")]
use crc::{Crc, CRC_32_ISCSI};

/// Room for any IANA zone name, with margin.
///
/// The longest the database contains is `America/Argentina/ComodRivadavia`, at 32 characters.
/// Sixty-four costs nothing that matters: this is one value under one settings key, not a field
/// repeated per row.
pub const TIMEZONE_NAME_LEN: usize = 64;

/// The machine's timezone, as an IANA zone name.
///
/// **A name, not a `chrono_tz::Tz`.** That type is an enum generated at build time from the
/// tz database, and its discriminants move whenever the database is regenerated *or the
/// build-time `CHRONO_TZ_TIMEZONE_FILTER` regex changes*. postcard is positional, so storing
/// one would make a machine's flash silently mean a different city after a dependency bump or
/// a one-word edit to a `.cargo/config.toml`. A name is stable by definition.
///
/// It is also not a type this crate may name. `-types` is what the schema exporter, the CLI and
/// the ESP32-C6 firmware all link against, and it stays cheap to link precisely because it does
/// nothing; resolving a name to a zone needs the tz database and belongs in
/// `variegated-timekeeping`, where `TimeZoneWrapper::from_iana_name` does it.
///
/// **The empty string is UTC.** That is what `Default` gives, what a machine that has never
/// been configured reads back, and what one reads back after a settings blob fails to
/// deserialize -- so the fallback is the same value in all three cases rather than three
/// different kinds of nothing.
///
/// **Scheduling only.** Shot logs, the debug bus and the DS3231 re-anchor stay on UTC by
/// design -- see `shot_log.rs` and `shot_log_storage.rs`. Changing this must never move a
/// timestamp that has already been written.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct TimezoneSetting {
    /// IANA zone name, e.g. `Europe/Stockholm`. Empty means UTC.
    pub name: heapless::String<TIMEZONE_NAME_LEN>,
}

impl TimezoneSetting {
    /// A setting from a zone name, or `None` if the name does not fit.
    ///
    /// **No truncation**, unlike the Wi-Fi SSID beside it. A shortened SSID is still a label a
    /// human can recognise; `Europe/Stockho` is not a shortened zone, it is a *different
    /// answer* -- it resolves to nothing, falls back to UTC, and the machine heats an hour
    /// early with nothing anywhere saying why.
    pub fn new(name: &str) -> Option<Self> {
        heapless::String::try_from(name).ok().map(|name| Self { name })
    }

    /// The stored name. Empty for UTC.
    pub fn as_str(&self) -> &str {
        self.name.as_str()
    }

    /// Whether this is UTC, however it was spelled.
    ///
    /// Both the empty default and an explicit `"UTC"` answer yes, so a caller rendering this
    /// can print `UTC` rather than an empty field.
    pub fn is_utc(&self) -> bool {
        self.name.is_empty() || self.name == "UTC"
    }
}

/// Mirrors the impl on `StoredWifiCredentials`; see the note there.
#[cfg(feature = "sequential-storage")]
impl<'a> Value<'a> for TimezoneSetting {
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
    use alloc::string::String as AllocString;

    #[test]
    fn the_default_is_utc() {
        let setting = TimezoneSetting::default();
        assert!(setting.as_str().is_empty());
        assert!(setting.is_utc());
    }

    #[test]
    fn an_explicit_utc_also_reads_as_utc() {
        assert!(TimezoneSetting::new("UTC").unwrap().is_utc());
        assert!(!TimezoneSetting::new("Europe/Stockholm").unwrap().is_utc());
    }

    /// A name that does not fit is refused, not truncated.
    ///
    /// Truncating would produce a syntactically fine zone name that resolves to nothing, so the
    /// machine would fall back to UTC and run an hour out with no error anywhere. The longest
    /// real name is 32 characters, so the boundary is only reachable by a client sending
    /// nonsense -- which is exactly when a silent answer is worst.
    #[test]
    fn a_name_that_does_not_fit_is_refused() {
        let at_limit: AllocString = "x".repeat(TIMEZONE_NAME_LEN);
        let over: AllocString = "x".repeat(TIMEZONE_NAME_LEN + 1);

        assert!(TimezoneSetting::new(&at_limit).is_some());
        assert!(TimezoneSetting::new(&over).is_none());
    }

    #[test]
    fn every_real_zone_name_fits() {
        // The longest name the IANA database contains, at 32 characters.
        let longest = "America/Argentina/ComodRivadavia";
        assert!(longest.len() <= TIMEZONE_NAME_LEN);
        assert_eq!(TimezoneSetting::new(longest).unwrap().as_str(), longest);
    }

    #[test]
    fn it_round_trips_through_postcard() {
        let setting = TimezoneSetting::new("Europe/Stockholm").unwrap();
        let bytes = postcard::to_allocvec(&setting).unwrap();
        let back: TimezoneSetting = postcard::from_bytes(&bytes).unwrap();
        assert_eq!(back, setting);
    }

    /// The flash path, which is hand-written and whose breakage is silent: `load_settings`
    /// maps a deserialization error to `Default`, so a broken impl reads as "never configured"
    /// rather than as an error.
    #[cfg(feature = "sequential-storage")]
    #[test]
    fn it_round_trips_through_flash() {
        let setting = TimezoneSetting::new("Europe/Stockholm").unwrap();
        let mut buffer = [0u8; 128];

        let written = setting.serialize_into(&mut buffer).unwrap();
        let (back, _) = TimezoneSetting::deserialize_from(&buffer[..written]).unwrap();

        assert_eq!(back, setting);
    }

    #[cfg(feature = "sequential-storage")]
    #[test]
    fn the_default_round_trips_through_flash_too() {
        let setting = TimezoneSetting::default();
        let mut buffer = [0u8; 128];

        let written = setting.serialize_into(&mut buffer).unwrap();
        let (back, _) = TimezoneSetting::deserialize_from(&buffer[..written]).unwrap();

        assert_eq!(back, setting);
        assert!(back.is_utc());
    }
}
