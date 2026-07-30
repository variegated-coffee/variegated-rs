use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::{PubSubChannel, Publisher, Subscriber};
use embassy_sync::signal::Signal;
use embassy_sync::channel::{Channel, Sender, Receiver};
use embassy_sync::mutex::Mutex;
use portable_atomic::AtomicBool;
use static_cell::StaticCell;
use variegated_controller_types::{CommsStatus, Configuration, ExternalPeripheralSensorReading, MachineCommand, MachineDefinition, RoutineList, Status};
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

// WiFi Connection Status - updated by connection_task, read by comms_status_signaller_task.
//
// esp-radio 0.18 removed the free function `wifi::sta_state()`, so connection
// state is only reachable through the `WifiController`, which connection_task
// owns. Mirror it here the same way the Belka status is mirrored.
pub static WIFI_CONNECTED: AtomicBool = AtomicBool::new(false);
