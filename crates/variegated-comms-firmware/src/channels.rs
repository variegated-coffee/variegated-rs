use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::{PubSubChannel, Publisher, Subscriber};
use embassy_sync::signal::Signal;
use embassy_sync::channel::{Channel, Sender, Receiver};
use embassy_sync::mutex::Mutex;
use embassy_sync::watch::Watch;
use portable_atomic::{AtomicBool, AtomicI16, AtomicU32, AtomicU64, Ordering};
use static_cell::StaticCell;
use variegated_controller_types::bluetooth::{BluetoothPeripheralList, MAX_BLUETOOTH_PERIPHERALS};
use variegated_controller_types::{CommsStatus, Configuration, ExternalPeripheralSensorReading, MachineCommand, MachineDefinition, PeripheralId, RoutineList, ScaleOp, Status};
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

// Routine Cache - periodically updated from application processor
pub static ROUTINE_CACHE: Mutex<CriticalSectionRawMutex, Option<RoutineList>> = Mutex::new(None);

// Application Routine Channel - for pushing routine updates to WebSocket clients
pub const APPLICATION_ROUTINE_RECEIVERS: usize = 4;
pub type ApplicationRoutineChannel = PubSubChannel<CriticalSectionRawMutex, RoutineList, 1, APPLICATION_ROUTINE_RECEIVERS, 1>;
pub type ApplicationRoutineSubscriber = Subscriber<'static, CriticalSectionRawMutex, RoutineList, 1, APPLICATION_ROUTINE_RECEIVERS, 1>;
pub type ApplicationRoutinePublisher = Publisher<'static, CriticalSectionRawMutex, RoutineList, 1, APPLICATION_ROUTINE_RECEIVERS, 1>;

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

// The per-peripheral connection flags that used to live here -- one for the Belka
// portal, one for the group 1 scale -- are now `ble::status`, which keeps a slot table
// instead. Two named statics could not survive a peripheral set the application
// processor decides at runtime, and holding the peripheral's identity and its connection
// state under one lock is what stops a reassignment from being read half-and-half.
//
// What has not changed is why any of it exists: the application processor's
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

// WiFi Connection Status - updated by connection_task, read by comms_status_signaller_task.
//
// esp-radio 0.18 removed the free function `wifi::sta_state()`, so connection
// state is only reachable through the `WifiController`, which connection_task
// owns. Mirror it here the same way the Belka status is mirrored.
pub static WIFI_CONNECTED: AtomicBool = AtomicBool::new(false);

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
