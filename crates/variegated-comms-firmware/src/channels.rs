use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::{PubSubChannel, Publisher, Subscriber};
use embassy_sync::signal::Signal;
use embassy_sync::channel::{Channel, Sender, Receiver};
use embassy_sync::mutex::Mutex;
use portable_atomic::{AtomicBool, AtomicI16, AtomicU64};
use static_cell::StaticCell;
use variegated_controller_types::{CommsStatus, Configuration, ExternalPeripheralSensorReading, MachineCommand, MachineDefinition, RoutineList, Status};
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
// now, TCP in Task 11), handed off to whoever executes them.
//
// Capacity 4: injection is interactive, so a backlog deeper than this means nobody
// is draining it. Every producer uses `try_send` and drops on full -- the debug
// readers must never block, and a queued command an operator typed a minute ago is
// worse than no command at all on a machine that heats water.
pub const DEBUG_COMMAND_CAPACITY: usize = 4;
pub static DEBUG_COMMAND_CHANNEL: StaticCell<Channel<CriticalSectionRawMutex, DebugCommand, DEBUG_COMMAND_CAPACITY>> = StaticCell::new();

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

// Belka Connection Status - updated by belka_measurement_loop, read by comms_status_signaller_task
pub static BELKA_CONNECTION_STATUS: AtomicBool = AtomicBool::new(false);

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
