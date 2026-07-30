//! Structured debug payloads emitted by both firmwares.
//!
//! These live here rather than in a dedicated crate because `DebugCommand` wraps
//! `MachineCommand` while `ApplicationProcessorToCommsProcessorMessage` wraps
//! `DebugFrame`; a separate crate would make the dependency circular.
//!
//! Sizing matters: an enum is as large as its largest variant, and a bus of these
//! is static RAM on both MCUs. `CounterSamples` is the largest at 16 * 8 bytes, so
//! `DebugFrame` lands around 150 bytes. Keep it that way -- in particular, metric
//! names are sent one at a time via `MetricName` rather than as a table, which is
//! also what lets a late-attaching client learn them under always-on emission.

use heapless::{String, Vec};

/// Maximum number of counters or indicators carried in one sample frame.
pub const MAX_SAMPLES: usize = 16;
/// Capacity of the ad-hoc text escape hatch.
pub const TEXT_LEN: usize = 96;
/// Capacity of a metric or firmware name.
pub const NAME_LEN: usize = 32;

pub type DebugText = String<TEXT_LEN>;
pub type Name = String<NAME_LEN>;

/// Copy `s` into a fixed-capacity string, dropping any tail that does not fit.
/// Pushing char-by-char keeps the result on a UTF-8 boundary.
pub fn fit<const N: usize>(s: &str) -> String<N> {
    let mut out = String::new();
    for c in s.chars() {
        if out.push(c).is_err() {
            break;
        }
    }
    out
}

pub fn name(s: &str) -> Name {
    fit(s)
}

pub fn text(s: &str) -> DebugText {
    fit(s)
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum DebugSource {
    Application,
    Comms,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Severity {
    Trace,
    Debug,
    Info,
    Warn,
    Error,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum MetricKind {
    Counter,
    Indicator,
}

/// One framed unit of debug output. `seq` is per-source and monotonic, so a host
/// can tell dropped frames from quiet periods; `uptime_ms` is that device's own
/// uptime -- the two devices boot independently, so it is not a shared clock.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, PartialEq)]
pub struct DebugFrame {
    pub source: DebugSource,
    pub seq: u32,
    pub uptime_ms: u64,
    pub payload: DebugPayload,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, PartialEq)]
pub enum DebugPayload {
    /// Raw counter values, indexed by counter id.
    CounterSamples(Vec<u64, MAX_SAMPLES>),
    /// Raw indicator values, indexed by indicator id.
    IndicatorSamples(Vec<u64, MAX_SAMPLES>),
    Event(DebugEvent),
    Text(Severity, DebugText),
    StateSnapshot(DebugStateSnapshot),
    /// Names one metric. Re-sent periodically so late clients can label things.
    MetricName {
        kind: MetricKind,
        id: u8,
        label: Name,
    },
    FirmwareInfo {
        firmware: Name,
        counters: u8,
        indicators: u8,
    },
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, PartialEq)]
pub enum DebugEvent {
    Boot,
    // Application processor
    TimeSynchronized { unix: u64 },
    TimeSyncIgnoredImplausible { unix: u64 },
    TimeSyncFailed,
    CommandReceived { label: Name },
    ConfigurationSent,
    ConfigurationRequested,
    MachineDefinitionSent,
    RoutinesSent,
    LinkDecodeError,
    CountersReset,
    // Comms processor
    WifiAssociated,
    WifiLost,
    WifiReconnectRequested,
    SntpSynced { unix: u64 },
    SntpFailed,
    BlePeripheralConnected { id: u16 },
    BlePeripheralDisconnected { id: u16 },
    BleScanStarted,
    EsphomeClientConnected,
    EsphomeClientDisconnected,
    TcpDebugClientConnected,
    TcpDebugClientDisconnected,
    // Either
    SpawnFailed { task: Name },
    HeapReport { used: u32, free: u32 },
    CommandRejected { reason: Name },
}

impl DebugEvent {
    pub fn severity(&self) -> Severity {
        match self {
            DebugEvent::TimeSyncFailed
            | DebugEvent::SntpFailed
            | DebugEvent::LinkDecodeError
            | DebugEvent::SpawnFailed { .. }
            | DebugEvent::CommandRejected { .. } => Severity::Error,
            DebugEvent::WifiLost
            | DebugEvent::TimeSyncIgnoredImplausible { .. }
            | DebugEvent::BlePeripheralDisconnected { .. } => Severity::Warn,
            _ => Severity::Info,
        }
    }

    /// Stable, allocation-free label used for host-side filtering and display.
    pub fn label(&self) -> &'static str {
        match self {
            DebugEvent::Boot => "boot",
            DebugEvent::TimeSynchronized { .. } => "time_synchronized",
            DebugEvent::TimeSyncIgnoredImplausible { .. } => "time_sync_ignored",
            DebugEvent::TimeSyncFailed => "time_sync_failed",
            DebugEvent::CommandReceived { .. } => "command_received",
            DebugEvent::ConfigurationSent => "configuration_sent",
            DebugEvent::ConfigurationRequested => "configuration_requested",
            DebugEvent::MachineDefinitionSent => "machine_definition_sent",
            DebugEvent::RoutinesSent => "routines_sent",
            DebugEvent::LinkDecodeError => "link_decode_error",
            DebugEvent::CountersReset => "counters_reset",
            DebugEvent::WifiAssociated => "wifi_associated",
            DebugEvent::WifiLost => "wifi_lost",
            DebugEvent::WifiReconnectRequested => "wifi_reconnect_requested",
            DebugEvent::SntpSynced { .. } => "sntp_synced",
            DebugEvent::SntpFailed => "sntp_failed",
            DebugEvent::BlePeripheralConnected { .. } => "ble_connected",
            DebugEvent::BlePeripheralDisconnected { .. } => "ble_disconnected",
            DebugEvent::BleScanStarted => "ble_scan_started",
            DebugEvent::EsphomeClientConnected => "esphome_connected",
            DebugEvent::EsphomeClientDisconnected => "esphome_disconnected",
            DebugEvent::TcpDebugClientConnected => "tcp_debug_connected",
            DebugEvent::TcpDebugClientDisconnected => "tcp_debug_disconnected",
            DebugEvent::SpawnFailed { .. } => "spawn_failed",
            DebugEvent::HeapReport { .. } => "heap_report",
            DebugEvent::CommandRejected { .. } => "command_rejected",
        }
    }
}

/// Deliberately not a copy of `Status` -- that already has a viewer in
/// `variegated-tui`. This carries what is otherwise invisible.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, PartialEq)]
pub struct DebugStateSnapshot {
    pub heap_used: u32,
    pub heap_free: u32,
    pub frames_emitted: u32,
    pub frames_dropped: u32,
    pub source_state: SourceState,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, PartialEq)]
pub enum SourceState {
    Application(ApplicationState),
    Comms(CommsState),
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, PartialEq)]
pub struct ApplicationState {
    /// `None` while the watchdog's feed time is not plumbed through to this
    /// snapshot. Deliberately an `Option` rather than a `0` sentinel: a zero here
    /// reads as "fed just now", which is a plausible-looking lie, and watchdog feed
    /// age is one of the things the hardware checkpoint exists to observe.
    pub watchdog_fed_ms_ago: Option<u32>,
    pub psram_heap: bool,
    /// `None` means "no routine running, or not determined" -- see the comment at the
    /// construction site.
    pub routine_running: Option<u16>,
    /// Filled by the relay in Task 8. Zero is accurate before then: there is no relay.
    pub link_frames_relayed: u32,
    pub link_frames_dropped: u32,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, PartialEq)]
pub struct CommsState {
    pub wifi_connected: bool,
    pub wifi_rssi: Option<i8>,
    pub sntp_synced_ms_ago: Option<u32>,
    pub ble_connected: Vec<u16, 8>,
    pub tcp_debug_clients: u8,
}
