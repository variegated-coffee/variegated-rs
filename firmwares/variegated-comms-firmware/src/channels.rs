use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::{PubSubChannel, Publisher, Subscriber};
use embassy_sync::signal::Signal;
use embassy_sync::channel::{Channel, Sender, Receiver};
use embassy_sync::mutex::Mutex;
use embassy_sync::watch::Watch;
use portable_atomic::{AtomicBool, AtomicI16, AtomicU8, AtomicU32, AtomicU64, Ordering};
use static_cell::StaticCell;
use variegated_controller_types::bluetooth::{BluetoothPeripheralList, MAX_BLUETOOTH_PERIPHERALS};
use variegated_controller_types::{CommsStatus, Configuration, ExternalPeripheralSensorReading, MachineCommand, MachineDefinition, PeripheralId, RoutineIndex, RoutineSummaryList, RoutineWriteOutcome, ScaleOp, Status};
use variegated_controller_types::shot_log::{
    ShotAnnotations, ShotLogEvent, ShotLogId, ShotLogList, ShotLogListRequest,
    ShotLogStorageError,
};
use variegated_controller_types::debug_command::DebugCommand;
use esphome_device::{ClientEvent, StateChange};

// Re-export Sender type for convenience
pub type MachineCommandSender = Sender<'static, CriticalSectionRawMutex, MachineCommand, MACHINE_COMMAND_CAPACITY>;

// Application Status Channel
pub const APPLICATION_STATUS_RECEIVERS: usize = 4;
pub type ApplicationStatusChannel = PubSubChannel<CriticalSectionRawMutex, Status, 1, APPLICATION_STATUS_RECEIVERS, 1>;
pub type ApplicationStatusSubscriber = Subscriber<'static, CriticalSectionRawMutex, Status, 1, APPLICATION_STATUS_RECEIVERS, 1>;
pub type ApplicationStatusPublisher = Publisher<'static, CriticalSectionRawMutex, Status, 1, APPLICATION_STATUS_RECEIVERS, 1>;

pub static STATUS_CHANNEL: StaticCell<ApplicationStatusChannel> = StaticCell::new();

// Application Configuration Channel
pub const APPLICATION_CONFIGURATION_RECEIVERS: usize = 4;
pub type ApplicationConfigurationChannel = PubSubChannel<CriticalSectionRawMutex, Configuration, 1, APPLICATION_CONFIGURATION_RECEIVERS, 1>;
pub type ApplicationConfigurationSubscriber = Subscriber<'static, CriticalSectionRawMutex, Configuration, 1, APPLICATION_CONFIGURATION_RECEIVERS, 1>;
pub type ApplicationConfigurationPublisher = Publisher<'static, CriticalSectionRawMutex, Configuration, 1, APPLICATION_CONFIGURATION_RECEIVERS, 1>;

pub static CONFIGURATION_CHANNEL: StaticCell<ApplicationConfigurationChannel> = StaticCell::new();

// Comms Status Signal - used to send CommsStatus to application processor
pub static COMMS_STATUS_SIGNAL: Signal<CriticalSectionRawMutex, CommsStatus> = Signal::new();

// WiFi RSSI Signal - updated by connection_task, read by comms_status_signaller_task
pub static WIFI_RSSI_SIGNAL: Signal<CriticalSectionRawMutex, Option<i8>> = Signal::new();

// WiFi RSSI mirror, for readers that must not consume WIFI_RSSI_SIGNAL.
//
// `Signal::try_take` is destructive: it hands the value to exactly one caller and
// leaves the signal empty. `comms_status_signaller_task` already takes it once a
// second, so a second 1 Hz reader -- the debug snapshot task -- would not observe a
// stale value, it would *steal* roughly half of them and the application processor
// would see `wifi_rssi: None` on those cycles. Mirroring into an atomic the way
// WIFI_CONNECTED is mirrored gives the snapshot a non-consuming read.
//
// `i16` rather than `i8` so there is a value outside the RSSI range to mean "not
// known": `NO_RSSI` is returned before the first sample and whenever the link is
// down. It renders as `Option::None`, never as a plausible `0 dBm`.
pub const NO_RSSI: i16 = i16::MIN;
pub static WIFI_RSSI_DBM: AtomicI16 = AtomicI16::new(NO_RSSI);

// Machine Definition - set once at startup, then read-only
// Using Mutex<Option<>> since OnceLock is std-only
pub static MACHINE_DEFINITION: Mutex<CriticalSectionRawMutex, Option<MachineDefinition>> = Mutex::new(None);

/// Every stored routine's name, type and counts -- **not** its definition.
///
/// Periodically refreshed from the application processor. Cached, unlike the shot-log
/// replies further down, because two emitters serve it (the HTTP listing and the
/// WebSocket push) and a client asks for it on every connection: this is a small value
/// read often, where a shot is a large value read once.
///
/// The definitions themselves are not held here and are never decoded on this processor
/// at all. They move as opaque bytes through [`routine_request`], one routine at a time,
/// straight into an HTTP response body.
pub static ROUTINE_CACHE: Mutex<CriticalSectionRawMutex, Option<RoutineSummaryList>> = Mutex::new(None);

// Application Routine Channel - for pushing routine summary updates to WebSocket clients
pub const APPLICATION_ROUTINE_RECEIVERS: usize = 4;
pub type ApplicationRoutineChannel = PubSubChannel<CriticalSectionRawMutex, RoutineSummaryList, 1, APPLICATION_ROUTINE_RECEIVERS, 1>;
pub type ApplicationRoutineSubscriber = Subscriber<'static, CriticalSectionRawMutex, RoutineSummaryList, 1, APPLICATION_ROUTINE_RECEIVERS, 1>;
pub type ApplicationRoutinePublisher = Publisher<'static, CriticalSectionRawMutex, RoutineSummaryList, 1, APPLICATION_ROUTINE_RECEIVERS, 1>;

pub static ROUTINE_CHANNEL: StaticCell<ApplicationRoutineChannel> = StaticCell::new();

// Status Cache - cached status for HTTP server
pub static STATUS_CACHE: Mutex<CriticalSectionRawMutex, Option<Status>> = Mutex::new(None);

// Configuration Cache - cached configuration for HTTP server
pub static CONFIG_CACHE: Mutex<CriticalSectionRawMutex, Option<Configuration>> = Mutex::new(None);

// Machine Command Channel - commands to send to application processor
pub const MACHINE_COMMAND_CAPACITY: usize = 8;
pub static MACHINE_COMMAND_CHANNEL: StaticCell<Channel<CriticalSectionRawMutex, MachineCommand, MACHINE_COMMAND_CAPACITY>> = StaticCell::new();

// Debug Command Channel - commands injected over a debug transport (USB-Serial-JTAG
// always, TCP when `config::TCP_COMMANDS_ENABLED`), handed off to whoever executes
// them. Drained by the application-processor sender, which dispatches through
// `debug::commands::dispatch`.
//
// Capacity 4: injection is interactive, so a backlog deeper than this means nobody
// is draining it. Every producer uses `try_send` and drops on full -- the debug
// readers must never block, and a queued command an operator typed a minute ago is
// worse than no command at all on a machine that heats water.
pub const DEBUG_COMMAND_CAPACITY: usize = 4;
pub static DEBUG_COMMAND_CHANNEL: StaticCell<Channel<CriticalSectionRawMutex, DebugCommand, DEBUG_COMMAND_CAPACITY>> = StaticCell::new();

// The three requests `CommsDebugOp` raises against tasks that own hardware this
// firmware cannot touch from a dispatcher.
//
// `Signal`, not `Channel`, and latest-wins is the right semantics for all of them:
// an operator who asks twice for a Wi-Fi reconnect while one is in flight wants one
// reconnect, not two queued. `signal()` never awaits and never fails, which is what
// lets the dispatcher stay synchronous -- see `debug::commands`.
//
// Each has exactly one consumer, which is what `Signal` requires: `wait()` holds a
// single waker and a second concurrent waiter would displace and re-wake the first
// forever. See `variegated_debug::status`'s module docs for the full argument.

/// Consumed by `wifi::connection_task`. Latching, so a request raised while the link
/// is already down is satisfied by the reconnect already in progress.
pub static WIFI_RECONNECT_REQUEST: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Consumed by `time::sntp_task`, which otherwise sleeps five minutes between syncs.
pub static SNTP_RESYNC_REQUEST: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Consumed by `ble::devices::ble_debug_command_loop`. Carries the peripheral id, and
/// the dispatcher has already rejected ids this firmware does not know about, so the
/// consumer never has to answer for one.
pub static BLE_RECONNECT_REQUEST: Signal<CriticalSectionRawMutex, u16> = Signal::new();

/// Scale operations raised by the application processor and consumed by the measurement
/// loops in `ble::devices`.
///
/// Carries the peripheral id rather than a bare op, because a machine can have more than
/// one Bluetooth scale and each loop owns exactly one of them. Unlike
/// `BLE_RECONNECT_REQUEST` above, nothing has validated the id before it arrives -- it
/// comes off the UART from the other processor, not from the local dispatcher -- so each
/// consumer checks it against its own before acting.
///
/// **Not a `Signal`, and that is the whole reason this is a channel.** The block above
/// spells out `Signal`'s constraint: `wait()` holds a single waker, so a second
/// concurrent waiter displaces and re-wakes the first forever. One scale meant one
/// waiter and the constraint was satisfied by accident; with a peripheral set the
/// application processor decides at runtime there can be up to
/// `MAX_BLUETOOTH_PERIPHERALS` scale loops waiting at once. That failure mode does not
/// panic or log -- it silently wedges, so a tare would simply never arrive, and
/// intermittently.
///
/// Depth 1 with latest-wins semantics, matching what `Signal` gave: an operator who
/// asks twice wants one tare. Published with `immediate_publisher`, which needs no
/// publisher slot and never awaits -- the same non-blocking contract the UART reader
/// requires everywhere else.
///
/// The staleness wrinkle is unchanged and still handled by the consumer: a tare raised
/// while the scale is disconnected would otherwise sit queued and fire on the next
/// connect, possibly minutes later and in the middle of a different shot. Each loop
/// drains its subscriber once the scale is initialised, so only a tare asked for while
/// the link was up survives.
pub static SCALE_COMMAND_CHANNEL: PubSubChannel<
    CriticalSectionRawMutex,
    (PeripheralId, ScaleOp),
    1,
    MAX_BLUETOOTH_PERIPHERALS,
    1,
> = PubSubChannel::new();

/// A discovery scan, requested by the application processor, carrying its duration in
/// milliseconds.
///
/// One consumer -- the forwarding loop in `ble::devices`, which is where a `&'static`
/// connection manager is in scope. The UART reader cannot call the manager directly:
/// nothing hands it one, and it must not await.
///
/// Latest-wins, which is the right reading of a user pressing the button twice.
pub static BLE_SCAN_REQUEST: Signal<CriticalSectionRawMutex, u16> = Signal::new();

/// Read and cleared by `ble::scanner::ScanPrinter` on the next advertising report.
///
/// An atomic rather than a `Signal` because the consumer is an `EventHandler`
/// callback invoked from the BLE runner, not an async task: it cannot await, so it
/// needs a flag it can test and clear in place.
pub static BLE_RESCAN_PENDING: AtomicBool = AtomicBool::new(false);

/// The Bluetooth peripheral associations, as last received from the application
/// processor.
///
/// This firmware has no persistent storage, so this is the *only* thing that says which
/// Bluetooth devices exist. Until it arrives there are no peripherals at all.
///
/// A `Watch` rather than the `Signal` its neighbours above use, for two reasons. It has
/// more than one interested party -- the reconciler waits on changes, while the status
/// signaller and the debug dispatcher want to read the current value -- and `Signal`
/// permits exactly one waiter, per the note above. And `Watch::try_get` is a
/// non-consuming read that needs no receiver slot, so those readers cost nothing; a
/// `PubSubChannel` would make each of them burn a subscriber.
///
/// Written from the UART reader with `send`, which never awaits and never fails. That is
/// the same no-back-pressure requirement `SCALE_TARE_REQUEST.signal()` has, and for the
/// same reason: blocking that task blocks `Status` and every debug frame behind it.
pub static BT_ASSOCIATIONS: Watch<
    CriticalSectionRawMutex,
    BluetoothPeripheralList,
    BT_ASSOCIATION_RECEIVERS,
> = Watch::new();

/// Receiver slots on [`BT_ASSOCIATIONS`]. One, for the reconciler in `ble::devices`.
/// Everything else reads with `try_get`, which needs no slot.
pub const BT_ASSOCIATION_RECEIVERS: usize = 1;

/// Whether the application processor has ever answered `RequestBluetoothPeripherals`.
///
/// Set on **receipt**, never on the list being non-empty. A machine with nothing paired
/// answers with an empty list, and that is a complete answer -- testing for emptiness
/// would make such a machine re-ask every ten seconds forever.
pub static BT_PERIPHERALS_RECEIVED: AtomicBool = AtomicBool::new(false);

/// The Wi-Fi credentials the application processor last sent, or `None` if it says none
/// are configured.
///
/// Written from the UART reader with `send`, which never awaits and never fails -- the same
/// no-back-pressure requirement [`BT_ASSOCIATIONS`] has.
///
/// Nothing reads this yet. `wifi::connection_task` still configures the station from the
/// compiled-in `config::SSID`/`PASSWORD`; making it read from here is the next step, and is
/// deliberately separate because it is the change that can leave a machine with no network.
pub static WIFI_CREDENTIALS: Watch<
    CriticalSectionRawMutex,
    Option<variegated_controller_types::wifi::WifiCredentials>,
    WIFI_CREDENTIAL_RECEIVERS,
> = Watch::new();

/// Receiver slots on [`WIFI_CREDENTIALS`]. One, for the connection task that will read it.
pub const WIFI_CREDENTIAL_RECEIVERS: usize = 1;

/// Whether the application processor has answered [`RequestWifiCredentials`] at all.
///
/// Set on **receipt**, never on credentials being present. A machine with no network
/// configured answers `None`, and that is a complete answer -- testing for `Some` would
/// make such a machine re-ask every ten seconds for as long as it runs. This is the same
/// trap [`BT_PERIPHERALS_RECEIVED`] documents, and it is easier to fall into here because
/// the payload is literally an `Option`.
///
/// [`RequestWifiCredentials`]: variegated_controller_types::CommsProcessorToApplicationProcessorMessage::RequestWifiCredentials
pub static WIFI_CREDENTIALS_RECEIVED: AtomicBool = AtomicBool::new(false);

/// Where finished shot logs are uploaded, and the token that authorises it.
///
/// **`Box`ed, unlike every other payload on this page.** A `Watch` stores its value inline
/// in a `static`, and `ShotUploadConfig` is ~330 bytes of `heapless::String`; `.bss` costs
/// `.stack` one for one and the margin is ~2.4 kB (see the block above `heap_allocator!` in
/// `bin/main.rs`). The same call [`SHOT_LOG_REPLY`] makes about its 1 kB chunk.
///
/// Written from the UART reader with `send`, which never awaits and never fails -- the same
/// no-back-pressure requirement [`WIFI_CREDENTIALS`] has.
pub static SHOT_UPLOAD_CONFIG: Watch<
    CriticalSectionRawMutex,
    alloc::boxed::Box<variegated_controller_types::shot_upload::ShotUploadConfig>,
    SHOT_UPLOAD_CONFIG_RECEIVERS,
> = Watch::new();

/// Receiver slots on [`SHOT_UPLOAD_CONFIG`]. One, for the upload task.
pub const SHOT_UPLOAD_CONFIG_RECEIVERS: usize = 1;

/// Whether the application processor has answered [`RequestShotUploadConfig`] at all.
///
/// Set on **receipt**, never on the endpoint being present -- the same trap
/// [`WIFI_CREDENTIALS_RECEIVED`] documents, and easier to fall into here because the
/// payload has its own empty state rather than being an `Option`. A machine that has never
/// been configured for uploads answers with a default `ShotUploadConfig`, and that is a
/// complete answer; testing for `endpoint.is_some()` would make such a machine re-ask
/// every ten seconds forever.
///
/// [`RequestShotUploadConfig`]: variegated_controller_types::CommsProcessorToApplicationProcessorMessage::RequestShotUploadConfig
pub static SHOT_UPLOAD_CONFIG_RECEIVED: AtomicBool = AtomicBool::new(false);

/// An open provisioning window, in milliseconds, or zero to close one.
///
/// The application processor decides whether a window may open -- it is the only side that
/// knows a shot is in progress -- so anything signalled here has been vetted.
pub static WIFI_PROVISIONING_WINDOW: Signal<CriticalSectionRawMutex, u32> = Signal::new();

/// A candidate credential Improv wants tried, and the verdict.
///
/// Two signals rather than a channel, because there is exactly one requester (the Improv
/// task) and exactly one responder (`wifi::connection_task`), and neither should queue: a
/// second candidate arriving while the first is being tried means the user pressed send
/// again, and latest-wins is the right reading of that. The single-waiter property is what
/// `Signal` requires -- see the block above [`WIFI_RECONNECT_REQUEST`].
///
/// **The result is a bare `bool` on purpose.** The connection task knows only whether the
/// association succeeded. It does not know whether DHCP completed and must not wait to find
/// out, because blocking the owner of the radio on a lease would stall reconnection for every
/// other reason. The address the Improv client wants is the Improv task's problem.
pub static WIFI_CANDIDATE: Signal<
    CriticalSectionRawMutex,
    variegated_controller_types::wifi::WifiCredentials,
> = Signal::new();
pub static WIFI_CANDIDATE_RESULT: Signal<CriticalSectionRawMutex, bool> = Signal::new();

/// A Wi-Fi scan Improv wants run.
pub static WIFI_SCAN_REQUEST: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// What the scan found.
///
/// **On the heap, not inline.** A `Signal` stores its payload inline, so a
/// `heapless::Vec<Network, 16>` here would spend ~600 bytes of permanent `.bss` carrying data
/// that exists for a second or two -- and on this chip `.stack` is the SRAM left over after
/// `.data` and `.bss`, so every byte of static costs a byte of stack, one for one. This is
/// the same call [`ShotLogReply`] documents at length, for the same reason.
///
/// An empty vector means "none found", "the scan failed" and "the radio was busy" alike. The
/// distinction does not survive to the Improv client either way: all three reach it as a bare
/// terminating result frame.
pub static WIFI_SCAN_RESULT: Signal<
    CriticalSectionRawMutex,
    alloc::vec::Vec<variegated_improv_trouble::handler::Network>,
> = Signal::new();

// Comms Status Command - internal commands to update CommsStatus
pub enum CommsStatusCommand {
    TimeUpdate(u64), // Unix epoch offset in seconds
    WifiStatusUpdate(bool), // true if connected, false if disconnected
    WifiRssiUpdate(Option<i8>), // RSSI in dBm, None when disconnected
}

pub const COMMS_STATUS_COMMAND_CAPACITY: usize = 8;
pub static COMMS_STATUS_COMMAND_CHANNEL: StaticCell<Channel<CriticalSectionRawMutex, CommsStatusCommand, COMMS_STATUS_COMMAND_CAPACITY>> = StaticCell::new();

// ESPHome State Change Channel (commands/state changes to ESPHome server)
pub const MAX_QUEUED_STATE_CHANGES: usize = 70;
pub type StateChangeChannel = Channel<CriticalSectionRawMutex, StateChange<'static>, MAX_QUEUED_STATE_CHANGES>;
pub type StateChangeSender = Sender<'static, CriticalSectionRawMutex, StateChange<'static>, MAX_QUEUED_STATE_CHANGES>;
pub type StateChangeReceiver = Receiver<'static, CriticalSectionRawMutex, StateChange<'static>, MAX_QUEUED_STATE_CHANGES>;

pub static STATE_CHANGE_CHANNEL: StaticCell<StateChangeChannel> = StaticCell::new();

// ESPHome Client Event Channel (commands from ESPHome clients)
pub const CLIENT_EVENT_CAPACITY: usize = 8;
pub static CLIENT_EVENT_CHANNEL: StaticCell<Channel<CriticalSectionRawMutex, ClientEvent, CLIENT_EVENT_CAPACITY>> = StaticCell::new();

// External Peripheral Sensor Reading Channel - sensor readings from BLE devices to send to application processor
pub const SENSOR_READING_CAPACITY: usize = 16;
pub static SENSOR_READING_CHANNEL: StaticCell<Channel<CriticalSectionRawMutex, ExternalPeripheralSensorReading, SENSOR_READING_CAPACITY>> = StaticCell::new();

// Per-peripheral connection state is not here: it lives in `ble::status`, as a slot table
// rather than one named static per peripheral. Named statics cannot survive a peripheral
// set the application processor decides at runtime, and holding a peripheral's identity
// and its connection state under one lock is what stops a reassignment from being read
// half-and-half.
//
// Why any of it exists: the application processor's
// `BluetoothScale` drops every reading until it is told the link is up, so a peripheral
// missing from `CommsStatus.peripheral_connection_status` streams weights across the
// UART that are discarded on arrival.

// Time Sync Status - set by sntp_task once the RTC holds a real wall-clock
// time, read by comms_status_signaller_task.
//
// The RTC counts from zero at boot, so without this there is no way to tell
// "three seconds after the epoch" from "three seconds after power-on".
pub static TIME_SYNCED: AtomicBool = AtomicBool::new(false);

// Uptime in milliseconds at the last successful SNTP sync, for the debug snapshot's
// `sntp_synced_ms_ago`.
//
// Only meaningful when TIME_SYNCED is true; the snapshot gates on that rather than
// treating `0` as a timestamp, because `0` is a legitimate uptime and would read as
// "synced at boot" on a device whose clock never synced at all.
pub static LAST_SNTP_SYNC_MS: AtomicU64 = AtomicU64::new(0);

// How many times SNTP has successfully returned an answer since boot. Incremented by
// sntp_task, reported on every `CommsStatus`, and read by the application processor.
//
// Distinct from TIME_SYNCED, which latches: this one keeps counting, and it is the change
// that carries the meaning. The application processor re-anchors its clock when this
// advances and ignores `CommsStatus::timestamp` otherwise, because between syncs that
// timestamp is only this processor's RC-based RTC free-running -- see the field's own
// documentation in `variegated-controller-types`.
//
// Wrapping at u32 is fine and needs no handling: the reader compares for inequality, not
// ordering, and an hourly sync would take half a million years to get there.
pub static SNTP_SYNC_SEQ: AtomicU32 = AtomicU32::new(0);

// WiFi Connection Status - updated by connection_task, read by comms_status_signaller_task.
//
// esp-radio 0.18 removed the free function `wifi::sta_state()`, so connection
// state is only reachable through the `WifiController`, which connection_task
// owns. Mirror it here the same way the Belka status is mirrored.
pub static WIFI_CONNECTED: AtomicBool = AtomicBool::new(false);

/// The Improv provisioning state, mirrored for the 1 Hz `CommsStatus`.
///
/// An atomic, like every other mirror here and for the same reason: the status task must be
/// able to read it without consuming anything and without awaiting.
///
/// The stored byte is `codec::State`'s discriminant, and [`improv_state`] is the one place
/// the two enums are mapped onto each other. They are a wire contract with a half in each
/// repository -- `variegated_controller_types::wifi::ImprovState` says as much -- so the
/// mapping lives in exactly one function rather than at each use.
pub static IMPROV_STATE: AtomicU8 = AtomicU8::new(0);

pub fn improv_state() -> variegated_controller_types::wifi::ImprovState {
    use variegated_controller_types::wifi::ImprovState;
    match IMPROV_STATE.load(Ordering::Relaxed) {
        1 => ImprovState::AwaitingAuthorization,
        2 => ImprovState::Authorized,
        3 => ImprovState::Provisioning,
        4 => ImprovState::Provisioned,
        // Including anything unrecognised. `Stopped` is the safe way to be wrong: the machine
        // UI's indicator goes dark rather than claiming a window is open that is not.
        _ => ImprovState::Stopped,
    }
}

/// What the Improv service has to tell the application processor.
///
/// **Not `MachineCommand`, and the distinction is load bearing.** `MachineCommand` is the
/// *inbound* vocabulary -- what a client asks the machine to do -- and it reaches this
/// processor from HTTP, the WebSocket, ESPHome and the TCP debug port. These two are the
/// comms processor *reporting* something its own radio established.
///
/// The application processor stores a provisioned credential without validating it, and the
/// only thing that makes that correct is that it arrived by this route: see the comment on
/// the `SetWifiCredentials` arm in `dual_boiler_single_group.rs`, which says so in as many
/// words. Folding these into `MachineCommand` would make a credential anybody put on the
/// command channel indistinguishable from one a radio proved.
#[derive(Clone, Debug)]
pub enum ImprovReport {
    /// These associated. Persist them.
    Provisioned(variegated_controller_types::wifi::WifiCredentials),
    /// A client asked the machine to identify itself.
    Identify,
}

/// A `Channel`, not a `Signal`, and depth 2.
///
/// `Signal` is latest-wins, and the one message here whose loss is expensive is
/// [`ImprovReport::Provisioned`] -- an `Identify` arriving behind it would silently discard
/// the credential, leaving a machine that joined a network and never remembered it while the
/// phone said it worked. Two slots is one of each.
///
/// The producer uses `try_send` and never awaits, because it runs on a BLE connection's event
/// loop.
pub static IMPROV_REPORT_CHANNEL: Channel<CriticalSectionRawMutex, ImprovReport, 2> =
    Channel::new();

// Network identity mirrors, for `CommsState`'s `wifi_mac`, `bt_address` and `wifi_ip`.
//
// Atomics, like every other mirror above, and for the same reason: the debug snapshot
// runs in its own task and must be able to read these without consuming anything.
//
// A 48-bit address packs into the low bits of a `u64` big-endian, which keeps the
// store and the load single instructions and needs no lock. `NO_ADDRESS` is all-zeros,
// which is unambiguous as a sentinel -- 00:00:00:00:00:00 is not assignable to an
// interface and is not a legal BLE random address either (a random static address must
// have at least one bit of each polarity in its top 46).
pub const NO_ADDRESS: u64 = 0;

// The station MAC from eFuse. Written once in `main`, before the snapshot task is
// spawned, so `wifi_mac` is never reported as the sentinel in practice.
pub static WIFI_MAC: AtomicU64 = AtomicU64::new(NO_ADDRESS);

// The random BLE address `main` hands to `Address::random`. Written at BLE bring-up,
// which is textually after the snapshot task is spawned but still before `main`'s first
// `.await` -- so nothing spawned in between has run yet and the sentinel is not in fact
// observable today. `CommsState::bt_address` is an `Option` regardless: that guarantee
// rests on statement ordering in one long function and would be undone by inserting any
// `.await` ahead of the store, with no test and no type to catch it. `None` renders as
// `unknown`; a bare array would render zeros as though they were an address.
pub static BT_ADDRESS: AtomicU64 = AtomicU64::new(NO_ADDRESS);

// The DHCP-assigned IPv4 address, in host byte order (`Ipv4Addr::to_bits`).
//
// Refreshed once a second by `comms_status_signaller_task` rather than latched at the
// end of the DHCP wait: a lease can change on reconnect, and a latched value would go
// on asserting an address the device no longer holds.
//
// `NO_IPV4` is `0`, i.e. `0.0.0.0`, which is "this host on this network" in RFC 1122
// and never a real interface address. It renders as `unknown`, never as `0.0.0.0`.
pub const NO_IPV4: u32 = 0;
pub static WIFI_IPV4: AtomicU32 = AtomicU32::new(NO_IPV4);

/// Pack a 48-bit address into the low bytes of a `u64`, big-endian, and publish it.
pub fn store_address48(slot: &AtomicU64, bytes: [u8; 6]) {
    let mut packed = 0u64;
    for b in bytes {
        packed = (packed << 8) | b as u64;
    }
    slot.store(packed, Ordering::Relaxed);
}

/// Read a 48-bit address back, or `None` if nothing has published one yet.
pub fn load_address48(slot: &AtomicU64) -> Option<[u8; 6]> {
    let packed = slot.load(Ordering::Relaxed);
    if packed == NO_ADDRESS {
        return None;
    }
    let mut bytes = [0u8; 6];
    for (i, b) in bytes.iter_mut().enumerate() {
        *b = (packed >> (8 * (5 - i))) as u8;
    }
    Some(bytes)
}

// ============================================================================
// Shot log
// ============================================================================

/// What an HTTP handler wants from the application processor's SD card.
///
/// Mirrors two variants of `CommsProcessorToApplicationProcessorMessage` rather than
/// carrying that type directly: the wire enum also holds a `MachineCommand` and a
/// `CommsStatus`, and a `Signal` stores its payload inline, so signalling the wire type
/// would cost a permanently-resident static the size of its largest variant.
///
/// Writes are absent on purpose. `SetShotAnnotations` travels as a `MachineCommand`
/// through `MACHINE_COMMAND_CHANNEL` like every other write, so it needs nothing here;
/// only the two reads have to wait for an answer.
#[derive(Clone, Debug)]
pub enum ShotLogRequest {
    List(ShotLogListRequest),
    Chunk { id: ShotLogId, offset: u32 },
}

/// What came back.
///
/// **`Chunk` carries its bytes on the heap, not inline.** A `heapless::Vec<u8,
/// SHOT_LOG_CHUNK_LEN>` here would make this enum a kilobyte wide, and a `Signal` stores
/// its payload inline -- so `SHOT_LOG_REPLY` would spend 1,052 bytes of SRAM permanently
/// to carry data that exists for milliseconds. On this chip that is not merely wasteful:
/// `.stack` is laid out as the SRAM left over after `.data` and `.bss`, so **every byte
/// of static costs a byte of stack, one for one**, and the stack here has overflowed
/// before. It also inflated the HTTP task pool, since the download handler holds a chunk
/// across two awaits.
///
/// The heap is the right home for transient bulk: 128 kB of it, with esp-alloc's
/// high-water mark logged every second, against a stack that cannot be measured at all.
///
/// The wire type keeps its `heapless::Vec` -- see
/// `ApplicationProcessorToCommsProcessorMessage::ShotLogChunk`. That one is decoded into
/// a transient, and the application processor has no allocator to spare.
#[derive(Clone, Debug)]
pub enum ShotLogReply {
    List(ShotLogList),
    Chunk {
        id: ShotLogId,
        offset: u32,
        total: u32,
        last: bool,
        bytes: alloc::vec::Vec<u8>,
    },
    Annotations {
        id: ShotLogId,
        annotations: ShotAnnotations,
    },
    /// The application processor refused, and said why.
    Error(ShotLogStorageError),
}

/// Why a shot-log request produced nothing usable.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShotLogRequestError {
    /// No answer within the timeout. The application processor is wedged, the link is
    /// down, or a reply was lost -- from here those are indistinguishable, which is why
    /// [`ShotLogReply::Error`] exists to carry the cases the far side *can* name.
    Timeout,
    /// The far side answered, but not the question that was asked.
    ///
    /// Only reachable if two requests are somehow in flight at once, which
    /// [`SHOT_LOG_LOCK`] exists to prevent. Reported rather than ignored: silently
    /// accepting a mismatched chunk would splice one shot's bytes into another's
    /// download.
    Mismatched,
}

/// Requests bound for the application processor, picked up by its sender task.
pub static SHOT_LOG_REQUEST: Signal<CriticalSectionRawMutex, ShotLogRequest> = Signal::new();

/// Answers, published by the receiver task.
pub static SHOT_LOG_REPLY: Signal<CriticalSectionRawMutex, ShotLogReply> = Signal::new();

/// Serialises shot-log requests to exactly one in flight.
///
/// This is what makes a protocol with **no correlation id** safe. Two concurrent
/// requesters -- and there can be two, since the HTTP server runs more than one
/// connection handler -- would each take whichever reply arrived first. The lock plus the
/// `reset()` in [`shot_log_request`] means a reply can only ever belong to the request
/// currently holding it.
///
/// It also bounds the damage from a download: a second client asking for a list waits its
/// turn rather than interleaving chunk requests into the first client's file.
pub static SHOT_LOG_LOCK: Mutex<CriticalSectionRawMutex, ()> = Mutex::new(());

/// Ask the application processor for something and wait for the answer.
///
/// Deliberately **not cached**, unlike [`ROUTINE_CACHE`], and paging makes the case
/// stronger rather than weaker. A page is asked for once and then superseded by the next
/// one; a chunk is asked for once per download. Caching either would cost permanent
/// `.bss` on a device with 512 kB of it -- and a cache keyed by cursor would be a cache
/// with a different key on every request.
///
/// The freshness problem a cache would otherwise create does not arise either: a client
/// learns about a new or deleted shot from [`SHOT_LOG_EVENT_CHANNEL`], not by re-asking.
pub async fn shot_log_request(
    request: ShotLogRequest,
    timeout: embassy_time::Duration,
) -> Result<ShotLogReply, ShotLogRequestError> {
    let _guard = SHOT_LOG_LOCK.lock().await;

    // Inside the lock, and before the request goes out. A previous requester that timed
    // out may have left its answer here; taking it as the reply to *this* request is the
    // exact failure the lock cannot prevent on its own, because that reply arrives after
    // the previous holder has already let go.
    SHOT_LOG_REPLY.reset();
    SHOT_LOG_REQUEST.signal(request);

    embassy_time::with_timeout(timeout, SHOT_LOG_REPLY.wait())
        .await
        .map_err(|_| ShotLogRequestError::Timeout)
}

/// Shot-log events, on their way from the link to the WebSocket.
///
/// A `PubSubChannel` like `STATUS_CHANNEL` and `ROUTINE_CHANNEL`, published with
/// `immediate_publisher()`: no publisher slot, never awaits, evicts the oldest on a full
/// ring. That is the same non-blocking contract the `Debug(frame)` arm documents, and for
/// the same reason -- back-pressure on the UART reader is back-pressure on `Status` and
/// on everything else the link carries.
///
/// **It costs about 600 bytes of `.bss`**, and on this chip `.stack` is the SRAM left
/// after `.data` and `.bss`, so that is 600 bytes off the stack. That is the price of the
/// notice carrying the whole entry rather than an id the browser would have to go and
/// resolve. `ShotLogEvent` owns no heap allocation, so the per-subscriber clone inside
/// the pubsub's critical section stays allocation-free -- the constraint `bus.rs` spells
/// out, and the reason a `Box` here would be worse rather than cheaper.
///
/// Nothing is retained for a client that connects later. An event is a fact about a
/// moment, and a browser opening afterwards fetches a page instead.
pub const SHOT_LOG_EVENT_RECEIVERS: usize = 1;
pub type ShotLogEventChannel =
    PubSubChannel<CriticalSectionRawMutex, ShotLogEvent, 1, SHOT_LOG_EVENT_RECEIVERS, 1>;
pub type ShotLogEventSubscriber =
    Subscriber<'static, CriticalSectionRawMutex, ShotLogEvent, 1, SHOT_LOG_EVENT_RECEIVERS, 1>;
pub type ShotLogEventPublisher =
    Publisher<'static, CriticalSectionRawMutex, ShotLogEvent, 1, SHOT_LOG_EVENT_RECEIVERS, 1>;

pub static SHOT_LOG_EVENT_CHANNEL: StaticCell<ShotLogEventChannel> = StaticCell::new();

// ============================================================================
// Routine definitions
// ============================================================================

/// One slice of a routine, on its way to a client.
///
/// **The bytes are on the heap, not inline**, for the reason [`ShotLogReply`] gives at
/// length: a `Signal` stores its payload inline, so a `heapless::Vec<u8,
/// ROUTINE_CHUNK_LEN>` here would spend a permanent kilobyte of `.bss` to carry data
/// that exists for milliseconds -- and on this chip `.stack` is the SRAM left over after
/// `.data` and `.bss`, so that kilobyte comes straight out of the stack.
///
/// The wire type keeps its `heapless::Vec`; the application processor has the SRAM and
/// would rather not have the allocator.
#[derive(Clone, Debug)]
pub enum RoutineReply {
    Chunk {
        index: RoutineIndex,
        offset: u16,
        total: u16,
        last: bool,
        bytes: alloc::vec::Vec<u8>,
    },
    /// No routine at that index. A real answer, and a fast one -- without it a client
    /// asking for a deleted routine would sit out the whole timeout and then be told the
    /// machine is not responding.
    NotFound(RoutineIndex),
    /// How a write ended, including the index a create was given.
    WriteResult(RoutineWriteOutcome),
}

/// A routine to be written, already encoded.
///
/// Not decoded here, and that is the point: these bytes came off a socket as postcard and
/// go onto the link as postcard, so building a `Routine` in between would be to take it
/// apart and put it back together identically, five to ten allocations per step, on the
/// heap Wi-Fi and BLE are sharing.
#[derive(Clone, Debug)]
pub struct RoutineWrite {
    /// `None` creates and lets the application processor assign an index.
    pub index: Option<RoutineIndex>,
    pub bytes: alloc::vec::Vec<u8>,
}

/// Why a routine request produced nothing usable.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RoutineRequestError {
    /// No answer within the timeout.
    Timeout,
    /// The far side answered, but not the question that was asked.
    Mismatched,
}

/// A client wants the current configuration, picked up by the sender task.
///
/// No reply signal beside it, unlike [`ROUTINE_REQUEST`] and [`SHOT_LOG_REQUEST`]: nobody
/// waits on this. The answer arrives as an ordinary `Configuration` message, is published
/// to the configuration pubsub like every other one, and reaches *every* connected client
/// -- so the client that asked is served by the same broadcast as the ones that did not.
/// That also means no lock is needed here: there is no correlation to protect, and a
/// second request while one is in flight is answered by the first one's reply.
///
/// A `Signal` rather than a channel for the same reason `ROUTINES_CHANGED` is one on the
/// other side of the link: two browsers loading at once should cost one round trip, not
/// two.
pub static CONFIG_REQUEST: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// A chunk request bound for the application processor, picked up by its sender task.
pub static ROUTINE_REQUEST: Signal<CriticalSectionRawMutex, (RoutineIndex, u16)> = Signal::new();

/// A routine to be written, picked up by the same task.
pub static ROUTINE_WRITE: Signal<CriticalSectionRawMutex, RoutineWrite> = Signal::new();

/// Answers to both, published by the receiver task.
pub static ROUTINE_REPLY: Signal<CriticalSectionRawMutex, RoutineReply> = Signal::new();

/// Serialises routine traffic to exactly one exchange in flight.
///
/// The same job [`SHOT_LOG_LOCK`] does, and a **separate** lock rather than the same one:
/// sharing would park a routine fetch behind a fifty-kilobyte shot download, and the
/// frontend walks every routine on connect.
///
/// It covers writes as well as reads, so a save and a fetch can never interleave on a
/// link that has no correlation id -- and a multi-chunk write stays contiguous, which the
/// far side requires.
pub static ROUTINE_LOCK: Mutex<CriticalSectionRawMutex, ()> = Mutex::new(());

/// Ask the application processor for one slice of a routine.
///
/// Deliberately **not cached**. A definition is fetched when a client opens a routine, and
/// the client caches it far more cheaply than this processor could -- it has orders of
/// magnitude more memory, and it is the one that knows when it is done with it.
pub async fn routine_request(
    index: RoutineIndex,
    offset: u16,
    timeout: embassy_time::Duration,
) -> Result<RoutineReply, RoutineRequestError> {
    let _guard = ROUTINE_LOCK.lock().await;

    // Inside the lock and before the request goes out -- see the identical ordering in
    // `shot_log_request`, and the reason it is load-bearing.
    ROUTINE_REPLY.reset();
    ROUTINE_REQUEST.signal((index, offset));

    embassy_time::with_timeout(timeout, ROUTINE_REPLY.wait())
        .await
        .map_err(|_| RoutineRequestError::Timeout)
}

/// Send a routine to the application processor and wait for it to be stored.
///
/// One call per routine, however many chunks it takes: the far side reassembles and
/// answers once, so this waits for a single [`RoutineReply::WriteResult`].
///
/// Holding [`ROUTINE_LOCK`] across the whole write is what makes chunking safe. The
/// application processor accepts a write only as a contiguous sequence from offset zero,
/// so a second writer cutting in would have its first chunk discard the first writer's
/// progress -- and the first writer would then be told its routine was malformed.
pub async fn routine_write(
    index: Option<RoutineIndex>,
    bytes: alloc::vec::Vec<u8>,
    timeout: embassy_time::Duration,
) -> Result<RoutineWriteOutcome, RoutineRequestError> {
    let _guard = ROUTINE_LOCK.lock().await;

    ROUTINE_REPLY.reset();
    ROUTINE_WRITE.signal(RoutineWrite { index, bytes });

    match embassy_time::with_timeout(timeout, ROUTINE_REPLY.wait()).await {
        Ok(RoutineReply::WriteResult(outcome)) => Ok(outcome),
        // A chunk or a not-found in answer to a write means the two paths have crossed,
        // which the lock should prevent. Reported rather than coerced into a success.
        Ok(_) => Err(RoutineRequestError::Mismatched),
        Err(_) => Err(RoutineRequestError::Timeout),
    }
}
