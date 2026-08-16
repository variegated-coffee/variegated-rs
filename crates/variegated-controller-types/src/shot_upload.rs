//! Where finished shot logs are uploaded, and the credential that authorises it.
//!
//! The application processor holds this and pushes it over the inter-processor link, for
//! the same reason it holds [`crate::wifi::WifiCredentials`]: the comms processor has no
//! flash of its own, so changing endpoint or rotating a token would otherwise mean
//! reflashing an ESP32-C6. See `ApplicationProcessorToCommsProcessorMessage::ShotUploadConfig`.

#[cfg(feature = "sequential-storage")]
use crc::{CRC_32_ISCSI, Crc};
#[cfg(feature = "sequential-storage")]
use postcard::{from_bytes_crc32, to_slice_crc32};
#[cfg(feature = "sequential-storage")]
use sequential_storage::map::{SerializationError, Value};

/// Longest endpoint URL accepted.
///
/// 255 rather than a round power of two because it is the longest value that still encodes
/// its postcard length in two varint bytes, and because a URL longer than this is not one
/// anybody is going to paste into a TUI field correctly.
pub const SHOT_UPLOAD_ENDPOINT_LEN: usize = 255;

/// Longest upload token accepted.
///
/// Plantlet issues 40 random bytes as 64 characters of Crockford base32
/// (`apps/worker/src/machines.ts`), so this is that length exactly, not a guess with
/// headroom. A longer value is a paste error rather than a token.
pub const SHOT_UPLOAD_TOKEN_LEN: usize = 64;

/// The shot-log upload destination, or as much of it as has been configured.
///
/// # Why the `Option`s are per-field, and why there is no `Stored…` newtype
///
/// [`crate::wifi::StoredWifiCredentials`] wraps its whole value in an `Option` because
/// `WifiCredentials` has no way to say "unconfigured" -- empty strings are a *legal*
/// credential and a plausible mis-provisioning, so the two states have to be distinguished
/// outside the struct. This type has no such problem: an endpoint is either present or it
/// is not, and `{None, None}` is a complete and unambiguous "nothing configured". So
/// `Default` means what it should, [`Value`] goes directly on this struct, and there is no
/// wrapper.
///
/// Per-field rather than one `Option` over both because they are set independently and a
/// half-configured machine is a real state: an endpoint with no token is a machine waiting
/// for its token to be issued. The uploader requires both and says which is missing.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone, Default, PartialEq, Eq)]
pub struct ShotUploadConfig {
    /// The full HTTPS URL to `POST` each shot to, e.g. `https://example.org/api/shots`.
    pub endpoint: Option<heapless::String<SHOT_UPLOAD_ENDPOINT_LEN>>,
    /// The bearer token. **A secret**: see the `Debug` impl below before logging this.
    pub token: Option<heapless::String<SHOT_UPLOAD_TOKEN_LEN>>,
}

impl ShotUploadConfig {
    /// Both halves present, which is the only state the uploader can act on.
    pub fn is_complete(&self) -> bool {
        self.endpoint.is_some() && self.token.is_some()
    }
}

impl core::fmt::Debug for ShotUploadConfig {
    /// Prints the endpoint and the token's *length*, never the token.
    ///
    /// **Hand-written on purpose; do not replace with a derive.** This type is carried by
    /// `ApplicationProcessorToCommsProcessorMessage`, which derives `defmt::Format` and is
    /// logged on both processors, and by a `MachineCommand`, which reaches the debug wire
    /// and the TCP debug server. A derived `Debug` here puts a live upload token in all of
    /// those -- and unlike a Wi-Fi password, this one grants write access to an account on
    /// a public service.
    ///
    /// The endpoint is *not* elided. It is not a secret, and it is the field you actually
    /// need to see when an upload is going to the wrong place.
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("ShotUploadConfig")
            .field("endpoint", &self.endpoint.as_ref().map(|e| e.as_str()))
            .field("token_len", &self.token.as_ref().map(|t| t.len()))
            .finish()
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for ShotUploadConfig {
    /// Elides the token, for the reason given on the `Debug` impl above.
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "ShotUploadConfig {{ endpoint: {}, token_len: {} }}",
            self.endpoint.as_ref().map(|e| e.as_str()),
            self.token.as_ref().map(|t| t.len())
        )
    }
}

/// Why [`shot_upload_config`] refused its input.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShotUploadConfigError {
    /// The endpoint exceeded [`SHOT_UPLOAD_ENDPOINT_LEN`].
    EndpointTooLong,
    /// The token exceeded [`SHOT_UPLOAD_TOKEN_LEN`].
    TokenTooLong,
}

/// Build a [`ShotUploadConfig`], treating an empty field as "not configured".
///
/// # Rejects rather than truncates, unlike [`crate::wifi::wifi_credentials`]
///
/// That function truncates on a character boundary, and its own doc comment concedes the
/// call is debatable for a password. Here it would be plainly wrong, in both fields and for
/// different reasons:
///
/// * A truncated **endpoint** is still a syntactically valid URL. It would resolve, connect,
///   and `POST` every one of a user's shots somewhere they did not choose -- silently, since
///   a wrong path returns a perfectly ordinary `404`.
/// * A truncated **token** authenticates against nothing. It produces a `401` per shot with
///   no indication that the value was ever altered, and the natural reading of that is
///   "the token is wrong", which sends you to regenerate a token that was fine.
///
/// Both inputs are exact identifiers pasted from elsewhere, not labels chosen by a human,
/// so there is no case in which a shortened one is closer to what was meant. Callers get an
/// error they can show; `variegated-cli` bounds its fields to these lengths anyway, so the
/// error is a backstop rather than the normal path.
///
/// Empty means `None` deliberately: clearing a field is how a machine is de-provisioned,
/// and an empty string has no other useful meaning here.
pub fn shot_upload_config(
    endpoint: &str,
    token: &str,
) -> Result<ShotUploadConfig, ShotUploadConfigError> {
    Ok(ShotUploadConfig {
        endpoint: optional_field(endpoint).map_err(|_| ShotUploadConfigError::EndpointTooLong)?,
        token: optional_field(token).map_err(|_| ShotUploadConfigError::TokenTooLong)?,
    })
}

fn optional_field<const N: usize>(s: &str) -> Result<Option<heapless::String<N>>, ()> {
    if s.is_empty() {
        return Ok(None);
    }
    heapless::String::try_from(s).map(Some).map_err(|_| ())
}

// Mirrors the impl on `StoredWifiCredentials`; see the note there.
#[cfg(feature = "sequential-storage")]
impl<'a> Value<'a> for ShotUploadConfig {
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

    fn maximal() -> ShotUploadConfig {
        ShotUploadConfig {
            endpoint: Some(heapless::String::try_from("e".repeat(SHOT_UPLOAD_ENDPOINT_LEN).as_str()).unwrap()),
            token: Some(heapless::String::try_from("t".repeat(SHOT_UPLOAD_TOKEN_LEN).as_str()).unwrap()),
        }
    }

    #[cfg(feature = "sequential-storage")]
    #[test]
    fn config_round_trips_through_the_storage_value_impl() {
        let config = shot_upload_config("https://example.org/api/shots", "abc123").unwrap();

        let mut buffer = [0u8; 512];
        let len = config.serialize_into(&mut buffer).unwrap();
        let (restored, _) = ShotUploadConfig::deserialize_from(&buffer[..len]).unwrap();

        assert_eq!(config, restored);
    }

    #[cfg(feature = "sequential-storage")]
    #[test]
    fn the_unconfigured_case_round_trips_too() {
        // The state every machine is in before anyone configures one, and the state
        // `load_settings` synthesises when the key is absent. If this did not round-trip,
        // the failure would look like a machine that forgets its endpoint.
        let config = ShotUploadConfig::default();
        assert!(!config.is_complete());

        let mut buffer = [0u8; 512];
        let len = config.serialize_into(&mut buffer).unwrap();
        let (restored, _) = ShotUploadConfig::deserialize_from(&buffer[..len]).unwrap();

        assert_eq!(config, restored);
    }

    #[cfg(feature = "sequential-storage")]
    #[test]
    fn a_half_configured_value_round_trips() {
        // An endpoint with no token yet is a real state -- a machine waiting for its token
        // to be issued -- so it has to survive a power cycle like any other.
        let config = shot_upload_config("https://example.org/api/shots", "").unwrap();
        assert!(config.endpoint.is_some());
        assert!(config.token.is_none());
        assert!(!config.is_complete());

        let mut buffer = [0u8; 512];
        let len = config.serialize_into(&mut buffer).unwrap();
        let (restored, _) = ShotUploadConfig::deserialize_from(&buffer[..len]).unwrap();

        assert_eq!(config, restored);
    }

    #[cfg(feature = "sequential-storage")]
    #[test]
    fn a_maximal_config_serializes_within_the_stores_buffer() {
        // Deliberately tight. `SequentialStorageSettingsStorage` reads through a 2048-byte
        // buffer, and the point of this bound is to notice if a field grows enough to
        // approach it -- not merely to confirm 328 bytes fit in 2048.
        //
        // 1 + 2 + 255 (endpoint) + 1 + 1 + 64 (token) + 4 (CRC) = 328.
        let mut buffer = [0u8; 512];
        let len = maximal().serialize_into(&mut buffer).unwrap();
        assert!(len <= 384, "maximal config serialized to {len} bytes");
    }

    #[test]
    fn debug_shows_the_endpoint_and_hides_the_token() {
        // The whole point of the hand-written impl. A derive here would put a live upload
        // token on the debug bus and the TCP debug server.
        let config = shot_upload_config("https://example.org/api/shots", "sup3rs3cr3t").unwrap();
        let rendered = alloc::format!("{config:?}");

        assert!(rendered.contains("https://example.org/api/shots"));
        assert!(!rendered.contains("sup3rs3cr3t"));
        assert!(rendered.contains("token_len: Some(11)"));
    }

    #[test]
    fn empty_fields_mean_unconfigured() {
        let config = shot_upload_config("", "").unwrap();
        assert_eq!(config, ShotUploadConfig::default());
    }

    #[test]
    fn over_long_fields_are_refused_rather_than_truncated() {
        // The behaviour this type deliberately does *not* share with `wifi_credentials`.
        // A truncated endpoint would POST every shot somewhere the user did not choose.
        let long_endpoint = "e".repeat(SHOT_UPLOAD_ENDPOINT_LEN + 1);
        assert_eq!(
            shot_upload_config(&long_endpoint, "t"),
            Err(ShotUploadConfigError::EndpointTooLong)
        );

        let long_token = "t".repeat(SHOT_UPLOAD_TOKEN_LEN + 1);
        assert_eq!(
            shot_upload_config("https://example.org/", &long_token),
            Err(ShotUploadConfigError::TokenTooLong)
        );
    }

    #[test]
    fn exactly_maximal_fields_are_accepted() {
        // The boundary the previous test sits one byte past. Off-by-one here would reject
        // a Plantlet token, which is exactly SHOT_UPLOAD_TOKEN_LEN characters.
        let config = shot_upload_config(
            &"e".repeat(SHOT_UPLOAD_ENDPOINT_LEN),
            &"t".repeat(SHOT_UPLOAD_TOKEN_LEN),
        )
        .unwrap();
        assert_eq!(config, maximal());
        assert!(config.is_complete());
    }
}
