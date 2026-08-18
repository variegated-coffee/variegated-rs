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

/// Length of a Noise key as provisioned.
///
/// 32 bytes as Crockford base32 is 52 characters -- 256 bits is not a multiple of five, so
/// the last one carries a single bit -- plus a check symbol, which is what turns a mistyped
/// key into "that is not a valid key" instead of an opaque handshake failure three retries
/// later. See `variegated-shot-upload`'s `crockford` module, which does the decoding.
pub const SHOT_UPLOAD_KEY_LEN: usize = 53;

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
    /// Whether to upload at all.
    ///
    /// Separate from "is it configured" so that turning uploads off does not mean throwing
    /// the endpoint and token away and finding them again later. `Default` is `false`: a
    /// machine whose stored blob fails to decode must come back with uploads *off*, not
    /// silently start posting to whatever endpoint survives.
    ///
    /// **Appended, not inserted.** postcard is positional and this blob carries no version,
    /// so adding this field made every previously stored one fail to decode --
    /// `load_settings` maps that to `Default` -- and every machine lost its endpoint and
    /// token once. That was a deliberate, accepted cost, taken because the alternative was a
    /// second settings key for one bool.
    pub enabled: bool,
    /// The upload server's Noise static **public** key, Crockford base32.
    ///
    /// Only meaningful for an `http+noise://` endpoint. Public, so unlike the token it is
    /// safe to log and safe to show in the settings UI.
    ///
    /// **Appended, with the same cost as `enabled` above**: every previously stored blob
    /// fails to decode once more, and every machine loses its endpoint and token one further
    /// time. Accepted deliberately -- the alternative was a fifth settings store, which would
    /// mean touching `machine_stores()` and both espresso firmwares, and those two have
    /// forked badly enough that the standing advice is not to add to them.
    pub server_key: Option<heapless::String<SHOT_UPLOAD_KEY_LEN>>,
    /// This machine's Noise static **secret**, Crockford base32.
    ///
    /// **A secret of the same class as [`Self::token`]**: see the `Debug` impl below before
    /// logging this, and note that [`crate::configuration::ShotUploadView`] carries only
    /// whether it is set.
    ///
    /// Generated in the operator's browser, not by the server, so the upload service never
    /// holds it. That does not make it device-bound -- it still travels through a clipboard
    /// -- but it does keep it out of the service's logs and database.
    pub device_key: Option<heapless::String<SHOT_UPLOAD_KEY_LEN>>,
}

impl ShotUploadConfig {
    /// Enough present for *some* transport to act on.
    ///
    /// Says nothing about [`Self::enabled`] -- "configured" and "switched on" are different
    /// questions, and the uploader asks both.
    ///
    /// Deliberately does not parse the endpoint to decide *which* credentials are needed:
    /// scheme handling lives in `variegated-shot-upload`, and duplicating it here would be a
    /// second place to keep in step. The uploader still checks the pair its scheme actually
    /// requires and names the missing one.
    pub fn is_complete(&self) -> bool {
        self.endpoint.is_some() && (self.token.is_some() || self.has_noise_keys())
    }

    /// Both Noise keys present, which is what an `http+noise://` endpoint needs.
    pub fn has_noise_keys(&self) -> bool {
        self.server_key.is_some() && self.device_key.is_some()
    }

    /// Merge an edit from the settings UI.
    ///
    /// The endpoint and the switch are replaced outright; the token is whatever
    /// [`ShotUploadTokenUpdate`] says, because the browser is never sent the current one and
    /// so cannot send it back. See that type for why a plain `Option` will not do.
    pub fn apply(&mut self, update: ShotUploadSettings) {
        self.endpoint = update.endpoint;
        self.enabled = update.enabled;
        // The server key is public, so the browser is sent the current one and can send it
        // back: a plain replace is unambiguous, exactly as for the endpoint.
        self.server_key = update.server_key;
        match update.token {
            ShotUploadTokenUpdate::Keep => {}
            ShotUploadTokenUpdate::Clear => self.token = None,
            ShotUploadTokenUpdate::Set(token) => self.token = Some(token),
        }
        // The device key is a secret and gets the same three-way treatment as the token, for
        // the same reason: the browser never receives it, so a blank field cannot be read as
        // "remove it".
        match update.device_key {
            ShotUploadKeyUpdate::Keep => {}
            ShotUploadKeyUpdate::Clear => self.device_key = None,
            ShotUploadKeyUpdate::Set(key) => self.device_key = Some(key),
        }
    }
}

/// What an edit does to the stored token.
///
/// # Why this is not an `Option<String>`
///
/// The settings UI never receives the current token -- `Configuration` carries only a
/// `token_set: bool` -- so a blank field is genuinely ambiguous: it means "I did not touch
/// this" far more often than it means "remove it". An `Option` has two states and this
/// needs three, and collapsing them either makes every endpoint edit demand the token be
/// re-entered, or makes clearing a token impossible.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone, Default, PartialEq, Eq)]
pub enum ShotUploadTokenUpdate {
    /// Leave the stored token alone. The default, and what a blank field means.
    #[default]
    Keep,
    /// Forget it. De-provisions the machine while leaving the endpoint in place.
    Clear,
    /// Replace it. **A secret**: see the `Debug` impl below.
    Set(heapless::String<SHOT_UPLOAD_TOKEN_LEN>),
}

/// What an edit does to the stored Noise device key.
///
/// The same three states as [`ShotUploadTokenUpdate`] and for the same reason -- the browser
/// is never sent the current key, so a blank field means "untouched" rather than "remove".
///
/// A separate type rather than reusing that one because the two hold different lengths: a
/// device key is [`SHOT_UPLOAD_KEY_LEN`] and a token is [`SHOT_UPLOAD_TOKEN_LEN`]. Sharing
/// would mean a 64-capacity string carrying a 53-character value and a fallible conversion on
/// every apply, which is a silent truncation waiting to happen.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone, Default, PartialEq, Eq)]
pub enum ShotUploadKeyUpdate {
    /// Leave the stored key alone. The default, and what a blank field means.
    #[default]
    Keep,
    /// Forget it.
    Clear,
    /// Replace it. **A secret**: see the `Debug` impl below.
    Set(heapless::String<SHOT_UPLOAD_KEY_LEN>),
}

impl core::fmt::Debug for ShotUploadKeyUpdate {
    /// Prints `Set`'s *length*, never its value -- see [`ShotUploadTokenUpdate`]'s impl.
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Keep => f.write_str("Keep"),
            Self::Clear => f.write_str("Clear"),
            Self::Set(key) => f.debug_struct("Set").field("len", &key.len()).finish(),
        }
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for ShotUploadKeyUpdate {
    fn format(&self, f: defmt::Formatter) {
        match self {
            Self::Keep => defmt::write!(f, "Keep"),
            Self::Clear => defmt::write!(f, "Clear"),
            Self::Set(key) => defmt::write!(f, "Set {{ len: {} }}", key.len()),
        }
    }
}

impl core::fmt::Debug for ShotUploadTokenUpdate {
    /// Prints `Set`'s *length*, never its value.
    ///
    /// **Hand-written on purpose; do not replace with a derive**, for exactly the reason
    /// [`ShotUploadConfig`]'s impl gives: this rides inside a `MachineCommand`, which
    /// reaches the debug wire and the TCP debug server. A derive here would undo that
    /// type's care by putting the token on the same paths through a different door.
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Keep => f.write_str("Keep"),
            Self::Clear => f.write_str("Clear"),
            Self::Set(token) => write!(f, "Set(len {})", token.len()),
        }
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for ShotUploadTokenUpdate {
    /// Elides the token, for the reason given on the `Debug` impl above.
    fn format(&self, f: defmt::Formatter) {
        match self {
            Self::Keep => defmt::write!(f, "Keep"),
            Self::Clear => defmt::write!(f, "Clear"),
            Self::Set(token) => defmt::write!(f, "Set(len {})", token.len()),
        }
    }
}

/// One edit from the settings UI, before it is merged into the stored configuration.
///
/// `Debug` and `defmt::Format` are *derived* here, unlike on [`ShotUploadConfig`], and that
/// is safe only because [`ShotUploadTokenUpdate`]'s own impls are hand-written and elide.
/// The secret is one level down; if this struct ever gains a bare token field, these derives
/// have to go.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ShotUploadSettings {
    /// Replaces the stored endpoint outright. `None` clears it, which stops uploads.
    pub endpoint: Option<heapless::String<SHOT_UPLOAD_ENDPOINT_LEN>>,
    /// Replaces the stored switch outright.
    pub enabled: bool,
    /// Three-way; see [`ShotUploadTokenUpdate`].
    pub token: ShotUploadTokenUpdate,
    /// Replaces the stored server key outright. Public, so a plain replace is unambiguous.
    pub server_key: Option<heapless::String<SHOT_UPLOAD_KEY_LEN>>,
    /// Three-way; see [`ShotUploadKeyUpdate`].
    pub device_key: ShotUploadKeyUpdate,
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
    /// need to see when an upload is going to the wrong place. Neither is the server key,
    /// for the same reason -- it is public, and "which server does this machine trust" is
    /// precisely the question a failing handshake raises. The **device** key is a secret and
    /// is treated exactly like the token.
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("ShotUploadConfig")
            .field("endpoint", &self.endpoint.as_ref().map(|e| e.as_str()))
            .field("token_len", &self.token.as_ref().map(|t| t.len()))
            .field("enabled", &self.enabled)
            .field("server_key", &self.server_key.as_ref().map(|k| k.as_str()))
            .field("device_key_len", &self.device_key.as_ref().map(|k| k.len()))
            .finish()
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for ShotUploadConfig {
    /// Elides the token, for the reason given on the `Debug` impl above.
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "ShotUploadConfig {{ endpoint: {}, token_len: {}, enabled: {}, \
             server_key: {}, device_key_len: {} }}",
            self.endpoint.as_ref().map(|e| e.as_str()),
            self.token.as_ref().map(|t| t.len()),
            self.enabled,
            self.server_key.as_ref().map(|k| k.as_str()),
            self.device_key.as_ref().map(|k| k.len())
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
    /// The server key exceeded [`SHOT_UPLOAD_KEY_LEN`].
    ServerKeyTooLong,
    /// The device key exceeded [`SHOT_UPLOAD_KEY_LEN`].
    DeviceKeyTooLong,
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
///
/// `enabled` is a parameter rather than being inferred from the other two. "Configured" and
/// "switched on" are separate questions -- that is the entire reason the flag exists -- and a
/// constructor that guessed would make the CLI unable to express half the states the UI can.
pub fn shot_upload_config(
    endpoint: &str,
    token: &str,
    enabled: bool,
) -> Result<ShotUploadConfig, ShotUploadConfigError> {
    shot_upload_config_with_keys(endpoint, token, enabled, "", "")
}

/// As [`shot_upload_config`], but also setting the `http+noise://` key pair.
///
/// A separate entry point rather than two more parameters on the old one, so every existing
/// caller keeps compiling and reads the same. Both keys are refused rather than truncated for
/// the reasons above, and with one more: a key is not merely wrong when shortened, it is not
/// a key at all, and the resulting handshake failure names nothing.
pub fn shot_upload_config_with_keys(
    endpoint: &str,
    token: &str,
    enabled: bool,
    server_key: &str,
    device_key: &str,
) -> Result<ShotUploadConfig, ShotUploadConfigError> {
    Ok(ShotUploadConfig {
        endpoint: optional_field(endpoint).map_err(|_| ShotUploadConfigError::EndpointTooLong)?,
        token: optional_field(token).map_err(|_| ShotUploadConfigError::TokenTooLong)?,
        enabled,
        server_key: optional_field(server_key)
            .map_err(|_| ShotUploadConfigError::ServerKeyTooLong)?,
        device_key: optional_field(device_key)
            .map_err(|_| ShotUploadConfigError::DeviceKeyTooLong)?,
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

/// A fully-provisioned `http+noise://` settings edit survives the inter-processor wire.
///
/// The largest `SetShotUploadSettings` anybody sends in practice: an endpoint with a host, a
/// port and a path, and both 53-character keys. It comes to 164 bytes as a
/// `CommsProcessorToApplicationProcessorMessage`, against `CobsAccumulator<4096>` at both
/// ends of that link.
///
/// Here because the first `http+noise://` provisioning attempt on real hardware failed with
/// `Failed to deserialize message from ESP32` while a shorter one succeeded, which reads as
/// a capacity problem in these types. It is not: this passes. That makes the question "does
/// the message survive postcard" a five-second one rather than an afternoon of reading
/// buffer sizes.
#[test]
fn a_fully_provisioned_settings_edit_round_trips() {
    use crate::communication::CommsProcessorToApplicationProcessorMessage as Msg;
    use crate::commands::MachineCommand;
    use crate::shot_upload::*;

    let settings = ShotUploadSettings {
        endpoint: Some(
            heapless::String::try_from("http+noise://192.168.10.85:8787/api/noise-upload").unwrap(),
        ),
        enabled: true,
        token: ShotUploadTokenUpdate::Keep,
        server_key: Some(
            heapless::String::try_from("vtzxv1q39fddcfj2keexktfbbk4znhwndq2f9he004femzxmt0100")
                .unwrap(),
        ),
        device_key: ShotUploadKeyUpdate::Set(
            heapless::String::try_from("9961tkqed5gjx06grsfqrtf65j82192r535pa53tdzk6xa28et501")
                .unwrap(),
        ),
    };
    let msg = Msg::Command(MachineCommand::SetShotUploadSettings(settings.clone()));

    let mut buf = [0u8; 4096];
    let encoded = postcard::to_slice(&msg, &mut buf).expect("serialize");
    // Pinned, not merely observed: the number is what a transport-side buffer has to clear,
    // and a change to it means a wire-format change worth noticing here.
    assert_eq!(encoded.len(), 164);

    let decoded: Msg = postcard::from_bytes(encoded).expect("deserialize");
    match decoded {
        Msg::Command(MachineCommand::SetShotUploadSettings(got)) => assert_eq!(got, settings),
        _ => panic!("decoded to the wrong variant"),
    }
}

    fn maximal() -> ShotUploadConfig {
        ShotUploadConfig {
            endpoint: Some(heapless::String::try_from("e".repeat(SHOT_UPLOAD_ENDPOINT_LEN).as_str()).unwrap(),),
            token: Some(heapless::String::try_from("t".repeat(SHOT_UPLOAD_TOKEN_LEN).as_str()).unwrap()),
            enabled: true,
            server_key: Some(key(&"s".repeat(SHOT_UPLOAD_KEY_LEN))),
            device_key: Some(key(&"d".repeat(SHOT_UPLOAD_KEY_LEN))),
        }
    }

    fn token(s: &str) -> heapless::String<SHOT_UPLOAD_TOKEN_LEN> {
        heapless::String::try_from(s).unwrap()
    }

    fn endpoint(s: &str) -> heapless::String<SHOT_UPLOAD_ENDPOINT_LEN> {
        heapless::String::try_from(s).unwrap()
    }

    fn key(s: &str) -> heapless::String<SHOT_UPLOAD_KEY_LEN> {
        heapless::String::try_from(s).unwrap()
    }

    /// A machine that has been fully configured, as the settings UI would find it.
    ///
    /// The token path, deliberately: most of the tests below predate the Noise transport and
    /// assert token behaviour, so leaving the keys unset keeps them testing what they say.
    fn configured() -> ShotUploadConfig {
        ShotUploadConfig {
            endpoint: Some(endpoint("https://plantlet.example/api/shots")),
            token: Some(token("original-token")),
            enabled: true,
            server_key: None,
            device_key: None,
        }
    }

    /// A machine provisioned for `http+noise://`, with no token at all.
    fn noise_configured() -> ShotUploadConfig {
        ShotUploadConfig {
            endpoint: Some(endpoint("http+noise://plantlet.example/api/noise-upload")),
            token: None,
            enabled: true,
            server_key: Some(key(&"s".repeat(SHOT_UPLOAD_KEY_LEN))),
            device_key: Some(key(&"d".repeat(SHOT_UPLOAD_KEY_LEN))),
        }
    }

    #[cfg(feature = "sequential-storage")]
    #[test]
    fn config_round_trips_through_the_storage_value_impl() {
        let config = shot_upload_config("https://example.org/api/shots", "abc123", true).unwrap();

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
        let config = shot_upload_config("https://example.org/api/shots", "", true).unwrap();
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
        // 1 + 2 + 255 (endpoint) + 1 + 1 + 64 (token) + 1 (enabled)
        //   + 1 + 1 + 53 (server key) + 1 + 1 + 53 (device key) + 4 (CRC) = 439.
        let mut buffer = [0u8; 1024];
        let len = maximal().serialize_into(&mut buffer).unwrap();
        assert!(len <= 512, "maximal config serialized to {len} bytes");
    }

    #[test]
    fn a_blank_token_field_keeps_the_stored_one() {
        // **The reason `ShotUploadTokenUpdate` exists.** The browser is never sent the
        // token, so it cannot send it back -- and renaming the endpoint must not mean going
        // to find a 64-character token in Plantlet first.
        let mut config = configured();
        config.apply(ShotUploadSettings {
            endpoint: Some(endpoint("https://plantlet.example/v2/shots")),
            enabled: true,
            token: ShotUploadTokenUpdate::Keep,
            server_key: None,
            device_key: ShotUploadKeyUpdate::Keep,
        });

        assert_eq!(config.endpoint.as_deref(), Some("https://plantlet.example/v2/shots"));
        assert_eq!(config.token, Some(token("original-token")));
    }

    #[test]
    fn clearing_the_token_leaves_the_endpoint_alone() {
        // De-provisioning without forgetting where the machine was pointed.
        let mut config = configured();
        config.apply(ShotUploadSettings {
            endpoint: config.endpoint.clone(),
            enabled: true,
            token: ShotUploadTokenUpdate::Clear,
            server_key: None,
            device_key: ShotUploadKeyUpdate::Keep,
        });

        assert_eq!(config.token, None);
        assert!(config.endpoint.is_some());
        assert!(!config.is_complete());
    }

    #[test]
    fn setting_the_token_replaces_it() {
        let mut config = configured();
        config.apply(ShotUploadSettings {
            endpoint: config.endpoint.clone(),
            enabled: true,
            token: ShotUploadTokenUpdate::Set(token("rotated-token")),
            server_key: None,
            device_key: ShotUploadKeyUpdate::Keep,
        });

        assert_eq!(config.token, Some(token("rotated-token")));
    }

    #[test]
    fn keep_against_an_unconfigured_machine_is_still_no_token() {
        // `Keep` means "do not touch", not "there is one" -- the case a merge written as
        // `token.or(self.token)` would get right by accident and a careless one would not.
        let mut config = ShotUploadConfig::default();
        config.apply(ShotUploadSettings {
            endpoint: Some(endpoint("https://plantlet.example/api/shots")),
            enabled: true,
            token: ShotUploadTokenUpdate::Keep,
            server_key: None,
            device_key: ShotUploadKeyUpdate::Keep,
        });

        assert_eq!(config.token, None);
        assert!(!config.is_complete());
    }

    #[test]
    fn the_switch_is_replaced_outright_and_keeps_the_credentials() {
        // The whole point of a separate flag: off must not mean forgotten.
        let mut config = configured();
        config.apply(ShotUploadSettings {
            endpoint: config.endpoint.clone(),
            enabled: false,
            token: ShotUploadTokenUpdate::Keep,
            server_key: None,
            device_key: ShotUploadKeyUpdate::Keep,
        });

        assert!(!config.enabled);
        assert!(config.is_complete());
    }

    #[test]
    fn a_fresh_config_has_uploads_off() {
        // What a machine comes back as when its stored blob fails to decode -- which every
        // machine's did once, when `enabled` was appended. It must not start posting to a
        // surviving endpoint on its own.
        assert!(!ShotUploadConfig::default().enabled);
    }

    #[test]
    fn debug_hides_the_token_inside_a_token_update() {
        // The same care `ShotUploadConfig`'s impl takes, at the other door into the same
        // debug paths: `ShotUploadSettings` travels inside a `MachineCommand` too.
        let settings = ShotUploadSettings {
            endpoint: Some(endpoint("https://example.org/api/shots")),
            enabled: true,
            token: ShotUploadTokenUpdate::Set(token("sup3rs3cr3t")),
            server_key: None,
            device_key: ShotUploadKeyUpdate::Set(key(&"k".repeat(SHOT_UPLOAD_KEY_LEN))),
        };
        let rendered = alloc::format!("{settings:?}");

        assert!(!rendered.contains("sup3rs3cr3t"));
        assert!(rendered.contains("Set(len 11)"));
        // The device key is a secret of the same class, and travels the same debug paths.
        assert!(!rendered.contains(&"k".repeat(SHOT_UPLOAD_KEY_LEN)));
        assert!(rendered.contains("len: 53"));
    }

    #[test]
    fn debug_shows_the_endpoint_and_hides_the_token() {
        // The whole point of the hand-written impl. A derive here would put a live upload
        // token on the debug bus and the TCP debug server.
        let config =
            shot_upload_config("https://example.org/api/shots", "sup3rs3cr3t", true).unwrap();
        let rendered = alloc::format!("{config:?}");

        assert!(rendered.contains("https://example.org/api/shots"));
        assert!(!rendered.contains("sup3rs3cr3t"));
        assert!(rendered.contains("token_len: Some(11)"));
    }

    #[test]
    fn empty_fields_mean_unconfigured() {
        let config = shot_upload_config("", "", false).unwrap();
        assert_eq!(config, ShotUploadConfig::default());
    }

    #[test]
    fn over_long_fields_are_refused_rather_than_truncated() {
        // The behaviour this type deliberately does *not* share with `wifi_credentials`.
        // A truncated endpoint would POST every shot somewhere the user did not choose.
        let long_endpoint = "e".repeat(SHOT_UPLOAD_ENDPOINT_LEN + 1);
        assert_eq!(
            shot_upload_config(&long_endpoint, "t", true),
            Err(ShotUploadConfigError::EndpointTooLong)
        );

        let long_token = "t".repeat(SHOT_UPLOAD_TOKEN_LEN + 1);
        assert_eq!(
            shot_upload_config("https://example.org/", &long_token, true),
            Err(ShotUploadConfigError::TokenTooLong)
        );
    }

    #[test]
    fn exactly_maximal_fields_are_accepted() {
        // The boundary the previous test sits one byte past. Off-by-one here would reject
        // a Plantlet token, which is exactly SHOT_UPLOAD_TOKEN_LEN characters -- and now
        // also a Noise key, which is exactly SHOT_UPLOAD_KEY_LEN.
        let config = shot_upload_config_with_keys(
            &"e".repeat(SHOT_UPLOAD_ENDPOINT_LEN),
            &"t".repeat(SHOT_UPLOAD_TOKEN_LEN),
            true,
            &"s".repeat(SHOT_UPLOAD_KEY_LEN),
            &"d".repeat(SHOT_UPLOAD_KEY_LEN),
        )
        .unwrap();
        assert_eq!(config, maximal());
        assert!(config.is_complete());
    }

    #[test]
    fn a_key_one_character_too_long_is_refused() {
        // Rejected, not truncated: a shortened key is not a slightly-wrong key, it is not a
        // key at all, and the handshake failure it produces names nothing.
        let long = "k".repeat(SHOT_UPLOAD_KEY_LEN + 1);
        assert_eq!(
            shot_upload_config_with_keys("https://e.example/x", "t", true, &long, ""),
            Err(ShotUploadConfigError::ServerKeyTooLong)
        );
        assert_eq!(
            shot_upload_config_with_keys("https://e.example/x", "t", true, "", &long),
            Err(ShotUploadConfigError::DeviceKeyTooLong)
        );
    }

    #[test]
    fn a_noise_machine_is_complete_without_a_token() {
        // The state an `http+noise://` machine is provisioned into: the device key is the
        // credential, so requiring a token as well would make it permanently unconfigured.
        let config = noise_configured();
        assert!(config.token.is_none());
        assert!(config.has_noise_keys());
        assert!(config.is_complete());
    }

    #[test]
    fn one_noise_key_alone_is_not_enough() {
        let mut config = noise_configured();
        config.device_key = None;
        assert!(!config.has_noise_keys());
        assert!(!config.is_complete());
    }

    #[test]
    fn a_blank_device_key_field_keeps_the_stored_one() {
        // The same three-way rule the token follows, for the same reason: the browser is
        // never shown the device key, so blank means "untouched".
        let mut config = noise_configured();
        config.apply(ShotUploadSettings {
            endpoint: config.endpoint.clone(),
            enabled: true,
            token: ShotUploadTokenUpdate::Keep,
            server_key: config.server_key.clone(),
            device_key: ShotUploadKeyUpdate::Keep,
        });
        assert_eq!(config.device_key, Some(key(&"d".repeat(SHOT_UPLOAD_KEY_LEN))));

        config.apply(ShotUploadSettings {
            endpoint: config.endpoint.clone(),
            enabled: true,
            token: ShotUploadTokenUpdate::Keep,
            server_key: config.server_key.clone(),
            device_key: ShotUploadKeyUpdate::Clear,
        });
        assert_eq!(config.device_key, None);
    }

    #[test]
    fn debug_hides_the_device_key_but_shows_the_server_key() {
        // The asymmetry is the point: one is a secret, the other is public and is exactly
        // what you need to see when a handshake is being refused.
        let config = noise_configured();
        let rendered = alloc::format!("{config:?}");
        assert!(!rendered.contains(&"d".repeat(SHOT_UPLOAD_KEY_LEN)));
        assert!(rendered.contains("device_key_len"));
        assert!(rendered.contains(&"s".repeat(SHOT_UPLOAD_KEY_LEN)));
    }
}
