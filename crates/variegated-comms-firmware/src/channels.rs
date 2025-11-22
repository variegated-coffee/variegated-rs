use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::{PubSubChannel, Publisher, Subscriber};
use embassy_sync::signal::Signal;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use static_cell::StaticCell;
use variegated_controller_types::{CommsStatus, Configuration, MachineCommand, MachineDefinition, RoutineList, Status};

// Application Status Channel
pub const APPLICATION_STATUS_RECEIVERS: usize = 4;
pub type ApplicationStatusChannel = PubSubChannel<CriticalSectionRawMutex, Status, 1, APPLICATION_STATUS_RECEIVERS, 1>;
pub type ApplicationStatusSubscriber = Subscriber<'static, CriticalSectionRawMutex, Status, 1, APPLICATION_STATUS_RECEIVERS, 1>;
pub type ApplicationStatusPublisher = Publisher<'static, CriticalSectionRawMutex, Status, 1, APPLICATION_STATUS_RECEIVERS, 1>;

pub static STATUS_CHANNEL: StaticCell<ApplicationStatusChannel> = StaticCell::new();

// Application Configuration Channel
pub const APPLICATION_CONFIGURATION_RECEIVERS: usize = 3;
pub type ApplicationConfigurationChannel = PubSubChannel<CriticalSectionRawMutex, Configuration, 1, APPLICATION_CONFIGURATION_RECEIVERS, 1>;
pub type ApplicationConfigurationSubscriber = Subscriber<'static, CriticalSectionRawMutex, Configuration, 1, APPLICATION_CONFIGURATION_RECEIVERS, 1>;
pub type ApplicationConfigurationPublisher = Publisher<'static, CriticalSectionRawMutex, Configuration, 1, APPLICATION_CONFIGURATION_RECEIVERS, 1>;

pub static CONFIGURATION_CHANNEL: StaticCell<ApplicationConfigurationChannel> = StaticCell::new();

// Comms Status Signal - used to send CommsStatus to application processor
pub static COMMS_STATUS_SIGNAL: Signal<CriticalSectionRawMutex, CommsStatus> = Signal::new();

// Machine Definition - set once at startup, then read-only
// Using Mutex<Option<>> since OnceLock is std-only
pub static MACHINE_DEFINITION: Mutex<CriticalSectionRawMutex, Option<MachineDefinition>> = Mutex::new(None);

// Routine Cache - periodically updated from application processor
pub static ROUTINE_CACHE: Mutex<CriticalSectionRawMutex, Option<RoutineList>> = Mutex::new(None);

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
