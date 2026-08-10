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
}
