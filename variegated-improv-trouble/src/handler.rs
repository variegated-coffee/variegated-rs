//! What a machine has to provide for Improv to work.
//!
//! Deliberately free of BLE, and it has to stay that way. The comms firmware's internal
//! channels carry these types between the Improv task and the task that owns the Wi-Fi
//! radio, and putting them behind `ble` would drag `trouble-host` into modules that have
//! nothing to do with it -- including `wifi::connection_task`, which is the one place on
//! that firmware where a BLE dependency would be actively confusing.

use heapless::{String, Vec};

use crate::codec::{ErrorState, State, MAX_SSID_LEN};

/// Longest URL a `WIFI_SETTINGS` result will carry.
///
/// `http://255.255.255.255/` is 23 bytes. The rest is room for a hostname if this ever
/// answers with one instead of a lease that can change.
pub const MAX_URL_LEN: usize = 64;

/// How many networks a scan may report.
///
/// Each one is a separate notification -- see [`crate::codec::build_response`] for why they
/// cannot be batched -- so this bounds airtime rather than a buffer.
pub const MAX_SCAN_RESULTS: usize = 16;

pub type Url = String<MAX_URL_LEN>;

/// One network from a scan.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Network {
    pub ssid: String<MAX_SSID_LEN>,
    pub rssi: i8,
    /// Whether a client should ask the user for a password.
    ///
    /// When the beacon does not say, answer `true`. That is the safe way to be wrong: an
    /// unnecessary password prompt is a nuisance, a missing one is a provisioning attempt
    /// that cannot succeed and gives the user no clue why.
    pub requires_password: bool,
}

pub type NetworkList = Vec<Network, MAX_SCAN_RESULTS>;

/// The four strings a `GET_DEVICE_INFO` result carries, in order.
///
/// The order is the protocol's, not a convention: a client reads them positionally.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DeviceInfo<'a> {
    pub firmware_name: &'a str,
    pub firmware_version: &'a str,
    pub chip_variant: &'a str,
    pub device_name: &'a str,
}

/// Why a candidate credential did not take.
///
/// Narrower than [`ErrorState`] on purpose. A handler can only distinguish these two, and
/// letting it return the full error set would let it answer a provisioning attempt with
/// `InvalidRpc` -- a claim about the packet, which the handler never saw.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ProvisionError {
    UnableToConnect,
    NotAuthorized,
}

impl ProvisionError {
    /// The protocol error to report on the Error State characteristic.
    pub fn error_state(self) -> ErrorState {
        match self {
            Self::UnableToConnect => ErrorState::UnableToConnect,
            Self::NotAuthorized => ErrorState::NotAuthorized,
        }
    }
}

/// The machine's side of Improv.
///
/// # `provision` is handed a live password
///
/// It is the only method that is, and it must not log it, put it in anything that derives
/// `Debug` or `Format`, or keep it anywhere that outlives the call. See the crate docs.
#[allow(async_fn_in_trait)]
pub trait ImprovHandler {
    /// Try these credentials, and answer once the outcome is known.
    ///
    /// `Ok(Some(url))` gives the client somewhere to go next. `Ok(None)` means provisioned
    /// with nothing to point at -- a legal answer, and the right one when the association
    /// worked but no address arrived in time. Do not turn that case into an error: the
    /// network *is* joined, and reporting a failure would send the user to retype a password
    /// that was correct.
    async fn provision(&mut self, ssid: &str, password: &str) -> Result<Option<Url>, ProvisionError>;

    /// Enumerate visible networks.
    ///
    /// Infallible because the protocol cannot express the difference: "none found", "the scan
    /// failed" and "the radio was busy" all reach the client as the same bare terminating
    /// result frame. An empty list is the honest answer to all three.
    async fn scan(&mut self) -> NetworkList;

    /// Make the machine identifiable to someone standing in front of it.
    ///
    /// Synchronous and infallible: this runs on the connection's event loop, between an ATT
    /// write response and the next read, and there is nothing useful to do about a failure.
    fn identify(&mut self);

    fn device_info(&self) -> DeviceInfo<'_>;

    /// Called on every state transition, so the machine can report it elsewhere.
    ///
    /// Defaulted to nothing, because a handler with nowhere to report to is a reasonable
    /// handler.
    fn state_changed(&mut self, _state: State) {}
}
