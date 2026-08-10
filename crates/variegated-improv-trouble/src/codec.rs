//! Improv packet codec.

/// Provisioning state, as reported on the Current State characteristic and in the
/// advertisement's service data.
///
/// `Stopped` is not a state this machine idles in by accident: it means the provisioning
/// window is closed, which is the normal condition. The window is opened by the application
/// processor, which is the only side that knows whether a shot is being pulled.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum State {
    Stopped = 0x00,
    AwaitingAuthorization = 0x01,
    Authorized = 0x02,
    Provisioning = 0x03,
    Provisioned = 0x04,
}

/// Last error, as reported on the Error State characteristic.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum ErrorState {
    None = 0x00,
    InvalidRpc = 0x01,
    UnknownRpc = 0x02,
    UnableToConnect = 0x03,
    NotAuthorized = 0x04,
    BadHostname = 0x05,
    Unknown = 0xFF,
}

/// An RPC a client can write to the RPC Command characteristic.
///
/// # `0x02` is Identify, and the protocol is genuinely ambiguous here
///
/// `improv-wifi/sdk-cpp`'s `improv.h` defines `IDENTIFY = 0x02` and
/// `GET_CURRENT_STATE = 0x02` -- the same value, two names. Over BLE the ambiguity does not
/// arise: the current state is a characteristic a client reads and subscribes to, never
/// something it asks for by RPC, so an RPC of `0x02` can only be Identify. That is why there
/// is no `GetCurrentState` variant here, and why adding one would be a bug rather than
/// completeness.
///
/// `0x05` (Hostname) and `0x06` (Device Name) are deliberately absent. They are not
/// advertised in the capabilities byte, so a client has no reason to send one, and
/// [`Command::from_u8`] refuses them -- which surfaces as `UnknownRpc` rather than as
/// silence.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Command {
    WifiSettings = 0x01,
    Identify = 0x02,
    GetDeviceInfo = 0x03,
    GetWifiNetworks = 0x04,
}

impl Command {
    pub fn from_u8(value: u8) -> Option<Self> {
        match value {
            0x01 => Some(Self::WifiSettings),
            0x02 => Some(Self::Identify),
            0x03 => Some(Self::GetDeviceInfo),
            0x04 => Some(Self::GetWifiNetworks),
            _ => None,
        }
    }
}

/// The device supports the Identify RPC.
pub const CAPABILITY_IDENTIFY: u8 = 0x01;
/// The device supports the Device Info RPC.
pub const CAPABILITY_DEVICE_INFO: u8 = 0x02;
/// The device supports the Scan Wi-Fi Networks RPC.
pub const CAPABILITY_SCAN_WIFI: u8 = 0x04;

/// The six service-data bytes advertised under 16-bit UUID `0x4677`.
///
/// The UUID itself is not included: it belongs in the AD structure's own field, which is
/// what `AdStructure::ServiceData16 { uuid, data }` separates. ESPHome writes the two
/// together as `[0x77, 0x46, state, capabilities, 0, 0, 0, 0]` because the ESP-IDF API it
/// uses takes one flat buffer -- `0x77, 0x46` there is UUID `0x4677` little-endian, not part
/// of the payload.
pub fn service_data(state: State, capabilities: u8) -> [u8; 6] {
    [state as u8, capabilities, 0x00, 0x00, 0x00, 0x00]
}

/// Longest SSID the 802.11 standard allows.
pub const MAX_SSID_LEN: usize = 32;
/// Longest WPA passphrase (63) or a 64-character hex PSK.
pub const MAX_PASSWORD_LEN: usize = 64;
/// Longest RPC command packet this crate will accept.
///
/// `cmd + len + (1 + 32) + (1 + 64) + checksum`. Worth stating as a constant because it is
/// also the size the GATT characteristic's backing buffer has to be: the default ATT MTU of
/// 23 cannot carry this, so a client must either negotiate a larger MTU or use a long write,
/// and a buffer sized for the default would truncate a perfectly legal maximal packet.
pub const MAX_COMMAND_LEN: usize = 2 + (1 + MAX_SSID_LEN) + (1 + MAX_PASSWORD_LEN) + 1;

/// Credentials from a `WIFI_SETTINGS` RPC.
///
/// The `Debug` impl is hand-written and elides the password. See the crate docs.
#[derive(Clone, PartialEq, Eq)]
pub struct WifiSettings {
    pub ssid: heapless::String<MAX_SSID_LEN>,
    pub password: heapless::String<MAX_PASSWORD_LEN>,
}

impl core::fmt::Debug for WifiSettings {
    /// Prints the SSID and the password's *length*, never the password.
    ///
    /// A derived impl here would put a live Wi-Fi password wherever this type is formatted,
    /// which on this firmware includes the debug bus and the TCP debug server. The length is
    /// kept because "did the field arrive at all" is the question this is usually being read
    /// to answer.
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("WifiSettings")
            .field("ssid", &self.ssid.as_str())
            .field("password_len", &self.password.len())
            .finish()
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for WifiSettings {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "WifiSettings {{ ssid: {}, password_len: {} }}",
            self.ssid.as_str(),
            self.password.len()
        )
    }
}

/// A decoded RPC.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Request {
    WifiSettings(WifiSettings),
    Identify,
    GetDeviceInfo,
    GetWifiNetworks,
}

/// Why a packet was refused.
///
/// Distinct from [`ErrorState`] because several of these map to one protocol error and the
/// difference is worth having in a log: "the checksum was wrong" and "the length field
/// disagreed" are the same `InvalidRpc` to the client and completely different problems to
/// whoever is holding the sniffer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ParseError {
    /// Shorter than the three-byte minimum (command, length, checksum).
    TooShort,
    BadChecksum,
    /// The length field disagreed with the packet's actual length.
    LengthMismatch,
    UnknownCommand(u8),
    /// An internal length ran past the end of the data.
    Malformed,
    NotUtf8,
    /// SSID or password longer than this crate stores.
    TooLong,
}

impl ParseError {
    /// The protocol error to report on the Error State characteristic.
    pub fn error_state(&self) -> ErrorState {
        match self {
            Self::UnknownCommand(_) => ErrorState::UnknownRpc,
            _ => ErrorState::InvalidRpc,
        }
    }
}

/// Decode one RPC command packet.
///
/// The layout is `command, data_length, data.., checksum`, where the checksum is the least
/// significant byte of the sum of every preceding byte.
///
/// **The length field is checked against the real packet length before anything is indexed**,
/// and every internal length after it, because this input arrives from whoever is in radio
/// range and a trusted length here is an out-of-bounds read.
pub fn parse_command(packet: &[u8]) -> Result<Request, ParseError> {
    if packet.len() < 3 {
        return Err(ParseError::TooShort);
    }

    let checksum = packet[packet.len() - 1];
    let sum = packet[..packet.len() - 1]
        .iter()
        .fold(0u8, |acc, byte| acc.wrapping_add(*byte));
    if sum != checksum {
        return Err(ParseError::BadChecksum);
    }

    let data_length = packet[1] as usize;
    // `- 3` is command, length and checksum. The C++ reference writes this as
    // `length - 2 - check_checksum`, which is the same thing with the checksum optional.
    if data_length != packet.len() - 3 {
        return Err(ParseError::LengthMismatch);
    }

    let data = &packet[2..packet.len() - 1];

    match Command::from_u8(packet[0]) {
        Some(Command::WifiSettings) => {
            let (ssid, rest) = take_length_prefixed(data)?;
            let (password, rest) = take_length_prefixed(rest)?;
            // Trailing bytes after the password mean the packet is not what it claims,
            // even though the length and checksum agreed. Accepting it would mean
            // accepting a packet nobody meant to send.
            if !rest.is_empty() {
                return Err(ParseError::Malformed);
            }
            Ok(Request::WifiSettings(WifiSettings {
                ssid: to_string(ssid)?,
                password: to_string(password)?,
            }))
        }
        Some(Command::Identify) => Ok(Request::Identify),
        Some(Command::GetDeviceInfo) => Ok(Request::GetDeviceInfo),
        Some(Command::GetWifiNetworks) => Ok(Request::GetWifiNetworks),
        None => Err(ParseError::UnknownCommand(packet[0])),
    }
}

/// Split a `len`-prefixed byte string off the front of `data`.
fn take_length_prefixed(data: &[u8]) -> Result<(&[u8], &[u8]), ParseError> {
    let (length, rest) = data.split_first().ok_or(ParseError::Malformed)?;
    let length = *length as usize;
    if rest.len() < length {
        return Err(ParseError::Malformed);
    }
    Ok(rest.split_at(length))
}

fn to_string<const N: usize>(bytes: &[u8]) -> Result<heapless::String<N>, ParseError> {
    let text = core::str::from_utf8(bytes).map_err(|_| ParseError::NotUtf8)?;
    heapless::String::try_from(text).map_err(|_| ParseError::TooLong)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn command_discriminants_match_the_protocol() {
        assert_eq!(Command::from_u8(0x01), Some(Command::WifiSettings));
        assert_eq!(Command::from_u8(0x02), Some(Command::Identify));
        assert_eq!(Command::from_u8(0x03), Some(Command::GetDeviceInfo));
        assert_eq!(Command::from_u8(0x04), Some(Command::GetWifiNetworks));
        // 0x05 is Hostname and 0x06 DeviceName in the C++ SDK. Neither is implemented
        // here and neither is advertised in the capabilities byte, so both must be
        // refused as unknown rather than silently accepted.
        assert_eq!(Command::from_u8(0x05), None);
        assert_eq!(Command::from_u8(0x00), None);
    }

    #[test]
    fn states_and_errors_have_the_protocol_values() {
        assert_eq!(State::Stopped as u8, 0x00);
        assert_eq!(State::Provisioned as u8, 0x04);
        assert_eq!(ErrorState::None as u8, 0x00);
        assert_eq!(ErrorState::NotAuthorized as u8, 0x04);
        assert_eq!(ErrorState::Unknown as u8, 0xFF);
    }

    #[test]
    fn service_data_is_state_then_capabilities_then_four_reserved() {
        let all = CAPABILITY_IDENTIFY | CAPABILITY_DEVICE_INFO | CAPABILITY_SCAN_WIFI;
        assert_eq!(all, 0x07);
        assert_eq!(
            service_data(State::Authorized, all),
            [0x02, 0x07, 0x00, 0x00, 0x00, 0x00]
        );
    }

    /// `WIFI_SETTINGS` for ssid "MyNet", password "pw".
    ///
    /// Hand-derived from `improv-wifi/sdk-cpp`'s `parse_improv_data`: command, data length,
    /// then `ssid_len ssid pass_len pass`, then the LSB of the sum of everything before it.
    /// 1+9+5+77+121+78+101+116+2+112+119 = 741, and 741 & 0xFF = 0xE5.
    const WIFI_SETTINGS_MYNET: &[u8] = &[
        0x01, 0x09, 0x05, b'M', b'y', b'N', b'e', b't', 0x02, b'p', b'w', 0xE5,
    ];

    #[test]
    fn parses_wifi_settings() {
        let request = parse_command(WIFI_SETTINGS_MYNET).expect("should parse");
        match request {
            Request::WifiSettings(settings) => {
                assert_eq!(settings.ssid.as_str(), "MyNet");
                assert_eq!(settings.password.as_str(), "pw");
            }
            other => panic!("expected WifiSettings, got {other:?}"),
        }
    }

    #[test]
    fn rejects_a_bad_checksum() {
        let mut packet = WIFI_SETTINGS_MYNET.to_vec();
        *packet.last_mut().unwrap() = 0xE6;
        assert_eq!(parse_command(&packet), Err(ParseError::BadChecksum));
        assert_eq!(ParseError::BadChecksum.error_state(), ErrorState::InvalidRpc);
    }

    #[test]
    fn rejects_a_length_field_that_disagrees_with_the_packet() {
        let mut packet = WIFI_SETTINGS_MYNET.to_vec();
        packet[1] = 0x08; // says 8 bytes of data; the packet carries 9
        // Recompute the checksum so this tests the length rule and not the checksum.
        let sum: u32 = packet[..packet.len() - 1].iter().map(|b| *b as u32).sum();
        *packet.last_mut().unwrap() = sum as u8;
        assert_eq!(parse_command(&packet), Err(ParseError::LengthMismatch));
    }

    #[test]
    fn parses_an_open_network_with_an_empty_password() {
        // ssid "A", password "". 1+3+1+65+0 = 70 = 0x46.
        let packet: &[u8] = &[0x01, 0x03, 0x01, b'A', 0x00, 0x46];
        match parse_command(packet).expect("should parse") {
            Request::WifiSettings(settings) => {
                assert_eq!(settings.ssid.as_str(), "A");
                assert_eq!(settings.password.as_str(), "");
            }
            other => panic!("expected WifiSettings, got {other:?}"),
        }
    }

    #[test]
    fn rejects_an_ssid_length_that_runs_past_the_packet() {
        // ssid_len says 200 in a packet that holds 9 bytes of data. Left unchecked this
        // is an out-of-bounds slice, and the length arrives from anyone in radio range.
        let mut packet = WIFI_SETTINGS_MYNET.to_vec();
        packet[2] = 200;
        let sum: u32 = packet[..packet.len() - 1].iter().map(|b| *b as u32).sum();
        *packet.last_mut().unwrap() = sum as u8;
        assert_eq!(parse_command(&packet), Err(ParseError::Malformed));
    }

    #[test]
    fn rejects_a_non_utf8_ssid() {
        // 0xFF is not valid UTF-8 in any position. esp-radio's StationConfig takes a &str,
        // so there is nowhere for a byte-oriented SSID to go.
        let mut packet = WIFI_SETTINGS_MYNET.to_vec();
        packet[3] = 0xFF;
        let sum: u32 = packet[..packet.len() - 1].iter().map(|b| *b as u32).sum();
        *packet.last_mut().unwrap() = sum as u8;
        assert_eq!(parse_command(&packet), Err(ParseError::NotUtf8));
    }

    #[test]
    fn parses_the_payloadless_commands() {
        // IDENTIFY: 0x02 0x00, checksum 2.
        assert_eq!(parse_command(&[0x02, 0x00, 0x02]), Ok(Request::Identify));
        // GET_DEVICE_INFO: 0x03 0x00, checksum 3.
        assert_eq!(parse_command(&[0x03, 0x00, 0x03]), Ok(Request::GetDeviceInfo));
        // GET_WIFI_NETWORKS: 0x04 0x00, checksum 4.
        assert_eq!(parse_command(&[0x04, 0x00, 0x04]), Ok(Request::GetWifiNetworks));
    }

    #[test]
    fn rejects_an_unknown_command_as_unknown_not_invalid() {
        // 0x05 is Hostname, which this firmware does not implement. The distinction
        // matters to the client: UnknownRpc says "not supported", InvalidRpc says
        // "you sent me garbage", and only one of those is true.
        assert_eq!(parse_command(&[0x05, 0x00, 0x05]), Err(ParseError::UnknownCommand(0x05)));
        assert_eq!(
            ParseError::UnknownCommand(0x05).error_state(),
            ErrorState::UnknownRpc
        );
    }

    #[test]
    fn rejects_a_packet_too_short_to_hold_a_header() {
        assert_eq!(parse_command(&[]), Err(ParseError::TooShort));
        assert_eq!(parse_command(&[0x01, 0x00]), Err(ParseError::TooShort));
    }
}
