//! Wi-Fi credentials, and the provisioning state the comms processor reports.

#[cfg(feature = "sequential-storage")]
use sequential_storage::map::{SerializationError, Value};
#[cfg(feature = "sequential-storage")]
use postcard::{from_bytes_crc32, to_slice_crc32};
#[cfg(feature = "sequential-storage")]
use crc::{Crc, CRC_32_ISCSI};

/// Longest SSID the 802.11 standard allows.
pub const WIFI_SSID_LEN: usize = 32;
/// Longest WPA passphrase (63) or a 64-character hex PSK.
pub const WIFI_PASSWORD_LEN: usize = 64;

/// One network's credentials.
///
/// `heapless::String` rather than a byte array, because every consumer wants `&str`:
/// esp-radio's `StationConfig::with_ssid` takes one, and Improv delivers UTF-8. An SSID is
/// formally an opaque byte string, so this does refuse a non-UTF-8 network -- deliberately,
/// since there is nowhere for such a value to go on either side of the link.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
/// `Default` is the *empty* credential -- no SSID, no password -- which is the station's
/// state before anything configures it and the state the comms firmware restores it to when a
/// network is forgotten. It is not a placeholder: `wifi::park_until_provisioned` hands it to
/// `try_candidate` as the "previous" network to fall back to on a machine that has none, so
/// that a failed candidate puts the radio back exactly where `esp_radio::wifi::new` left it
/// rather than somewhere invented. Derives do not reach the wire, so this changes no format.
#[derive(Clone, Default, PartialEq, Eq)]
pub struct WifiCredentials {
    pub ssid: heapless::String<WIFI_SSID_LEN>,
    pub password: heapless::String<WIFI_PASSWORD_LEN>,
}

impl core::fmt::Debug for WifiCredentials {
    /// Prints the SSID and the password's *length*, never the password.
    ///
    /// **Hand-written on purpose; do not replace with a derive.** This type is carried by
    /// `ApplicationProcessorToCommsProcessorMessage`, which derives `defmt::Format` and is
    /// logged on both processors, and by a `MachineCommand`, which reaches the debug wire
    /// and the TCP debug server. A derived `Debug` here puts a live Wi-Fi password in all
    /// of those.
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("WifiCredentials")
            .field("ssid", &self.ssid.as_str())
            .field("password_len", &self.password.len())
            .finish()
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for WifiCredentials {
    /// Elides the password, for the reason given on the `Debug` impl above.
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "WifiCredentials {{ ssid: {}, password_len: {} }}",
            self.ssid.as_str(),
            self.password.len()
        )
    }
}

/// Build a [`WifiCredentials`], truncating either field to fit.
///
/// Mirrors [`crate::bluetooth::bluetooth_name`], and exists for the same reason: the
/// obvious `&s[..WIFI_SSID_LEN]` **panics** the moment a multi-byte character lands on the
/// boundary, and both of these strings come from outside -- an SSID chosen by whoever runs
/// the access point, a password typed by a user. Walking back to a character boundary takes
/// the whole class of input out of play.
///
/// Truncating rather than rejecting is the right call for an SSID, which is a label. It is
/// more debatable for a password, where a silently shortened value produces an association
/// failure with no clue as to why -- but a 64-byte limit is the WPA maximum plus room for a
/// hex PSK, so anything longer was not going to authenticate either way. Callers that can
/// report a length error to a human should check before calling.
pub fn wifi_credentials(ssid: &str, password: &str) -> WifiCredentials {
    WifiCredentials {
        ssid: truncate(ssid),
        password: truncate(password),
    }
}

fn truncate<const N: usize>(s: &str) -> heapless::String<N> {
    let mut end = s.len().min(N);
    while end > 0 && !s.is_char_boundary(end) {
        end -= 1;
    }
    // Infallible by construction: `end <= N` and sits on a boundary.
    heapless::String::try_from(&s[..end]).unwrap_or_default()
}

/// The stored value: credentials, or none configured.
///
/// A newtype so it can carry the `Value<'a>` impl, exactly as `BluetoothAssociations` does
/// (`bluetooth.rs:364-380, 573-600`).
///
/// `Option` inside rather than storing nothing: "no network configured" is a complete
/// answer, and the comms processor must be able to receive it. It stops re-asking on
/// *receipt*, never on the value being `Some` -- the trap `BT_PERIPHERALS_RECEIVED`
/// documents at `application_processor/mod.rs:649-656`.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct StoredWifiCredentials(pub Option<WifiCredentials>);

/// Improv provisioning state, as the comms processor reports it in `CommsStatus`.
///
/// Mirrors `variegated_improv_trouble::codec::State` without depending on it: that crate is
/// in the other repository and reaching for it here would put a BLE crate in the
/// application processor's dependency graph for the sake of five discriminants. The two are
/// a wire contract whose halves must agree, like the peripheral ids in the comms firmware's
/// `config.rs`.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ImprovState {
    /// The provisioning window is closed. The normal condition.
    #[default]
    Stopped,
    AwaitingAuthorization,
    Authorized,
    Provisioning,
    Provisioned,
}

// Mirrors the impl on `BluetoothAssociations`; see the note there.
#[cfg(feature = "sequential-storage")]
impl<'a> Value<'a> for StoredWifiCredentials {
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
    use alloc::format;

    fn credentials(ssid: &str, password: &str) -> WifiCredentials {
        WifiCredentials {
            ssid: heapless::String::try_from(ssid).unwrap(),
            password: heapless::String::try_from(password).unwrap(),
        }
    }

    /// A stored value must come back byte-identical, because the failure mode if it does
    /// not is silent: `load_settings` maps a deserialization error to `Default`, so a
    /// broken impl here reads as "no network configured" and the machine simply never
    /// joins a network again.
    #[cfg(feature = "sequential-storage")]
    #[test]
    fn credentials_round_trip_through_the_storage_value_impl() {
        use sequential_storage::map::Value;

        let stored = StoredWifiCredentials(Some(credentials("MyNet", "hunter2")));
        let mut buffer = [0u8; 256];
        let written = stored.serialize_into(&mut buffer).expect("should serialize");
        let (read, consumed) =
            StoredWifiCredentials::deserialize_from(&buffer[..written]).expect("should read back");

        assert_eq!(read, stored);
        assert_eq!(consumed, written);
    }

    /// "No network configured" is a value, not an absence, and it has to survive the round
    /// trip as such -- the comms processor stops re-asking on *receipt*, so a `None` that
    /// failed to store would leave it asking forever.
    #[cfg(feature = "sequential-storage")]
    #[test]
    fn the_unconfigured_case_round_trips_too() {
        use sequential_storage::map::Value;

        let stored = StoredWifiCredentials(None);
        let mut buffer = [0u8; 256];
        let written = stored.serialize_into(&mut buffer).expect("should serialize");
        let (read, _) =
            StoredWifiCredentials::deserialize_from(&buffer[..written]).expect("should read back");

        assert_eq!(read, StoredWifiCredentials(None));
        assert_eq!(read.0, None);
    }

    /// A maximal credential must fit the buffer the store hands us.
    #[cfg(feature = "sequential-storage")]
    #[test]
    fn a_maximal_credential_serializes_within_the_stores_buffer() {
        use sequential_storage::map::Value;

        let stored = StoredWifiCredentials(Some(credentials(
            &"s".repeat(WIFI_SSID_LEN),
            &"p".repeat(WIFI_PASSWORD_LEN),
        )));
        // `SequentialStorageSettingsStorage::deserialization_buffer` is 2048 bytes and its
        // write path allocates far more; 256 is a deliberately tight bound that would catch
        // this type growing something unbounded.
        let mut buffer = [0u8; 256];
        let written = stored.serialize_into(&mut buffer).expect("should serialize");
        assert!(written < 256);
    }

    /// The password must never reach a log. See the `Debug` impl.
    #[test]
    fn debug_shows_the_ssid_and_hides_the_password() {
        let rendered = format!("{:?}", credentials("MyNet", "hunter2"));
        assert!(rendered.contains("MyNet"));
        assert!(!rendered.contains("hunter2"));
    }

    #[test]
    fn improv_state_defaults_to_stopped() {
        assert_eq!(ImprovState::default(), ImprovState::Stopped);
    }

    /// The truncation must land on a character boundary. A naive `&s[..N]` panics here,
    /// and both of these strings come from outside the firmware.
    #[test]
    fn over_long_fields_truncate_on_a_character_boundary() {
        // 'é' is two bytes, so a 32-byte SSID limit falls mid-character when the string is
        // 16 of them followed by more.
        let ssid = "é".repeat(20);
        let password = "ü".repeat(40);
        let built = wifi_credentials(&ssid, &password);

        assert!(built.ssid.len() <= WIFI_SSID_LEN);
        assert!(built.password.len() <= WIFI_PASSWORD_LEN);
        // Truncated, not emptied: an off-by-one in the boundary walk would land on 0.
        assert_eq!(built.ssid.len(), WIFI_SSID_LEN);
        assert_eq!(built.password.len(), WIFI_PASSWORD_LEN);
        // And still valid UTF-8 with whole characters.
        assert!(built.ssid.chars().all(|c| c == 'é'));
    }

    #[test]
    fn fields_that_fit_are_left_alone() {
        let built = wifi_credentials("MyNet", "hunter2");
        assert_eq!(built.ssid.as_str(), "MyNet");
        assert_eq!(built.password.as_str(), "hunter2");
    }
}
