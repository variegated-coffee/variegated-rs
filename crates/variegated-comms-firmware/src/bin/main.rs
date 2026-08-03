//! Variegated Comms Firmware
//!
//! WiFi and BLE coexistence firmware for ESP32-C6.
//! Handles communication with application processor via UART,
//! BLE devices (Belka Portal, ACAIA scale), and provides HTTP server.

#![no_std]
#![no_main]

use core::net::{Ipv4Addr, SocketAddr};

use bt_hci::controller::ExternalController;
use defmt::info;
use edge_http::io::client::Connection;
use edge_http::Method;
use edge_nal_embassy::{Tcp, TcpBuffers};
use embassy_executor::Spawner;
use embassy_net::StackResources;
use embassy_time::{Duration, Timer};
use embedded_io_async::Read;
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    interrupt::software::SoftwareInterruptControl,
    ram,
    rng::Rng,
    rtc_cntl::Rtc,
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_radio::ble::controller::BleConnector;
use trouble_host::prelude::*;

// Library imports
use variegated_comms_firmware::{
    application_processor,
    ble::{ble_devices_task, ble_runner_task, ScanPrinter},
    channels::{
        ApplicationConfigurationChannel, ApplicationStatusChannel, ApplicationRoutineChannel,
        CONFIGURATION_CHANNEL, MACHINE_COMMAND_CHANNEL, STATUS_CHANNEL, ROUTINE_CHANNEL,
        MachineCommandSender, STATE_CHANGE_CHANNEL, CLIENT_EVENT_CHANNEL,
        StateChangeChannel, CLIENT_EVENT_CAPACITY, SENSOR_READING_CHANNEL,
        DEBUG_COMMAND_CHANNEL,
    },
    config::{uart_config, acaia_address, belka_address},
    debug,
    esphome::esphome_server_task,
    http::{http_server_task, cache_update_task},
    mk_static,
    time::sntp_task,
    websocket_server_task,
    wifi::{connection_task, net_task},
};
use esphome_device::ClientEvent;
use variegated_trouble_connection_manager::BleConnectionManager;

esp_bootloader_esp_idf::esp_app_desc!();

#[unsafe(no_mangle)]
pub extern "Rust" fn _esp_println_timestamp() -> u64 {
    esp_hal::time::Instant::now()
        .duration_since_epoch()
        .as_millis()
}

#[embassy_executor::task]
async fn status_listener_task(status_channel: &'static ApplicationStatusChannel) {
    let mut subscriber = status_channel.subscriber().unwrap();
    info!("Status listener task started");
    loop {
        let status = subscriber.next_message_pure().await;
//        info!("Status: {:?}", status);
    }
}

#[embassy_executor::task]
async fn comms_status_signaller_task(
    rtc: &'static esp_hal::rtc_cntl::Rtc<'static>,
) {
    use variegated_comms_firmware::channels::{BELKA_CONNECTION_STATUS, COMMS_STATUS_SIGNAL, TIME_SYNCED, WIFI_CONNECTED, WIFI_RSSI_SIGNAL};
    use variegated_comms_firmware::config::{BELKA_PERIPHERAL_ID, USEC_IN_SEC};
    use variegated_controller_types::{CommsStatus, WirelessConnectionStatus};
    use heapless::index_map::FnvIndexMap;
    use portable_atomic::Ordering;

    info!("CommsStatus signaller task started");
    loop {
        // Get real WiFi connection status
        let wifi_connected = WIFI_CONNECTED.load(Ordering::Relaxed);

        // Get real timestamp from RTC (convert microseconds to seconds).
        //
        // The RTC counts up from zero at boot, so `> 0` was true within a
        // microsecond of startup and we reported seconds-since-boot as though
        // it were a Unix timestamp. The application processor takes this as
        // wall-clock time, so until SNTP synced it was being told the date was
        // just after the epoch. Only report once SNTP has actually set the RTC.
        let timestamp = if TIME_SYNCED.load(Ordering::Relaxed) {
            Some(rtc.current_time_us() / USEC_IN_SEC)
        } else {
            None
        };

        // Get WiFi RSSI from signal (updated by connection_task)
        let wifi_rssi = WIFI_RSSI_SIGNAL.try_take().unwrap_or(None);

        // Get Belka connection status (updated by belka_measurement_loop)
        let belka_connected = BELKA_CONNECTION_STATUS.load(Ordering::Relaxed);

        // Build peripheral connection status map
        let mut peripheral_connection_status = FnvIndexMap::new();
        let _ = peripheral_connection_status.insert(
            BELKA_PERIPHERAL_ID,
            WirelessConnectionStatus {
                connected: belka_connected,
                rssi: None,
            }
        );

        let comms_status = CommsStatus {
            wifi_connected,
            timestamp,
            wifi_rssi,
            peripheral_connection_status,
        };
        COMMS_STATUS_SIGNAL.signal(comms_status);
        Timer::after(Duration::from_secs(1)).await;
    }
}

/// The structured debug stream's transport, on the peripheral `esp-println` used to
/// share (see the `esp-println` entry in Cargo.toml -- it is now pinned to UART0
/// precisely so this task can own USB-Serial-JTAG outright).
#[embassy_executor::task]
async fn debug_usb_task(
    usb_rx: esp_hal::usb_serial_jtag::UsbSerialJtagRx<'static, esp_hal::Async>,
    usb_tx: esp_hal::usb_serial_jtag::UsbSerialJtagTx<'static, esp_hal::Async>,
    sink: debug::CommandSink,
) {
    debug::usb::run(usb_rx, usb_tx, sink).await;
}

#[embassy_executor::task]
async fn application_processor_task(
    rx: esp_hal::uart::UartRx<'static, esp_hal::Async>,
    tx: esp_hal::uart::UartTx<'static, esp_hal::Async>,
    status_channel: &'static ApplicationStatusChannel,
    config_channel: &'static ApplicationConfigurationChannel,
    routine_channel: &'static ApplicationRoutineChannel,
    command_channel: &'static embassy_sync::channel::Channel<
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        variegated_controller_types::MachineCommand,
        8,
    >,
    sensor_reading_channel: &'static embassy_sync::channel::Channel<
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        variegated_controller_types::ExternalPeripheralSensorReading,
        16,
    >,
) {
    let status_publisher = status_channel.publisher().unwrap();
    let config_publisher = config_channel.publisher().unwrap();
    let routine_publisher = routine_channel.publisher().unwrap();
    let command_receiver = command_channel.receiver();
    let sensor_reading_receiver = sensor_reading_channel.receiver();

    application_processor::start(
        rx,
        tx,
        status_publisher,
        config_publisher,
        routine_publisher,
        command_receiver,
        sensor_reading_receiver,
    )
    .await;
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Initialize RTC for time synchronization
    let rtc = Rtc::new(peripherals.LPWR);

    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 64 * 1024);

    // Initialize application processor channels
    let status_channel = STATUS_CHANNEL.init(embassy_sync::pubsub::PubSubChannel::new());
    let config_channel = CONFIGURATION_CHANNEL.init(embassy_sync::pubsub::PubSubChannel::new());
    let routine_channel = ROUTINE_CHANNEL.init(embassy_sync::pubsub::PubSubChannel::new());
    let command_channel = MACHINE_COMMAND_CHANNEL.init(embassy_sync::channel::Channel::new());
    let sensor_reading_channel = SENSOR_READING_CHANNEL.init(embassy_sync::channel::Channel::new());

    // Initialize the debug command channel (commands injected over USB-Serial-JTAG
    // now, over TCP once Task 11 lands).
    let debug_command_channel = DEBUG_COMMAND_CHANNEL.init(embassy_sync::channel::Channel::new());

    // Bring up the structured debug transport before anything else that might have
    // something to say. `esp-println` no longer touches this peripheral (Cargo.toml
    // pins it to UART0), so the stream owns it outright.
    //
    // `split()` returns (rx, tx) in that order -- not the (tx, rx) that most of the
    // rest of esp-hal uses.
    let usb = esp_hal::usb_serial_jtag::UsbSerialJtag::new(peripherals.USB_DEVICE).into_async();
    let (usb_rx, usb_tx) = usb.split();
    if let Ok(t) = debug_usb_task(usb_rx, usb_tx, debug_command_channel.sender()) { spawner.spawn(t); }
    info!("Debug USB-Serial-JTAG transport spawned");

    // Initialize ESPHome channels
    let state_change_channel = STATE_CHANGE_CHANNEL.init(embassy_sync::channel::Channel::new());
    let client_event_channel = CLIENT_EVENT_CHANNEL.init(embassy_sync::channel::Channel::new());

    // Create UART for application processor communication
    let uart = esp_hal::uart::Uart::new(peripherals.UART1, uart_config())
        .expect("Failed to create UART")
        .with_tx(peripherals.GPIO20)
        .with_rx(peripherals.GPIO21)
        .with_cts(peripherals.GPIO18)
        .with_rts(peripherals.GPIO19)
        .into_async();

    let (rx, tx) = uart.split();

    // Spawn application processor tasks.
    //
    // embassy-executor 0.10 moved the fallibility from `Spawner::spawn` (which
    // now returns `()`) onto the `#[task]` function itself, so every one of
    // these grew an inner `if let Ok`. The previous `.ok()` discarded a failed
    // spawn, and that is preserved here rather than switched to `unwrap`.
    if let Ok(t) = application_processor_task(rx, tx, status_channel, config_channel, routine_channel, command_channel, sensor_reading_channel) { spawner.spawn(t); }
    if let Ok(t) = status_listener_task(status_channel) { spawner.spawn(t); }
    info!("Application processor tasks spawned");

    // Initialize esp-rtos
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    info!("Initializing radio");

    // esp-radio 0.18 removed `esp_radio::init()` and the `Controller` handle;
    // the radio is brought up implicitly by the BLE/WiFi constructors.

    // Initialize BLE (before WiFi for stability)
    let connector = BleConnector::new(peripherals.BT, Default::default()).unwrap();
    let controller: ExternalController<_, 20> = ExternalController::new(connector);

    // Create BLE host resources
    let ble_resources = mk_static!(
        HostResources<DefaultPacketPool, 4, 12, 16>,
        HostResources::new()
    );

    // Generate random BLE address
    let rng = Rng::new();
    let address_bytes = [
        rng.random() as u8,
        (rng.random() >> 8) as u8,
        (rng.random() >> 16) as u8,
        (rng.random() >> 24) as u8,
        rng.random() as u8,
        (rng.random() >> 8) as u8,
    ];
    let address = Address::random(address_bytes);
    info!("BLE: Generated random address");

    // Create BLE stack
    let stack = trouble_host::new(controller, ble_resources).set_random_address(address);
    let stack = mk_static!(
        Stack<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
        stack
    );

    // Build BLE host
    let Host { central, runner, .. } = stack.build();

    // Create scan printer for logging discovered devices
    let printer = mk_static!(ScanPrinter, ScanPrinter::new());

    // Create connection manager
    let connection_manager = mk_static!(
        BleConnectionManager<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
        BleConnectionManager::new(central)
    );

    // Get sensor reading sender for BLE devices
    let sensor_reading_sender = sensor_reading_channel.sender();

    // Spawn BLE tasks
    if let Ok(t) = ble_runner_task(runner, printer) { spawner.spawn(t); }
    if let Ok(t) = ble_devices_task(connection_manager, stack, belka_address(), acaia_address(), sensor_reading_sender) { spawner.spawn(t); }
    info!("BLE tasks spawned");

    Timer::after_secs(5).await;

    info!("Initializing Wifi");

    // Initialize WiFi. The radio controller handle is gone in 0.18, and
    // `interfaces.sta` was renamed `interfaces.station`. Configuration stays in
    // `connection_task`, which now calls `set_config` -- that both configures
    // and starts the controller, since `start_async` was removed.
    let (controller, interfaces) =
        esp_radio::wifi::new(peripherals.WIFI, Default::default()).unwrap();

    let wifi_interface = interfaces.station;

    info!("Creating network stack");

    // Create network stack
    let net_config = embassy_net::Config::dhcpv4(Default::default());
    let rng = Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    let (net_stack, runner) = embassy_net::new(
        wifi_interface,
        net_config,
        mk_static!(StackResources<16>, StackResources::<16>::new()),
        seed,
    );

    // Make stack and RTC static for tasks
    let stack_static = mk_static!(embassy_net::Stack<'static>, net_stack);
    let rtc_static = mk_static!(Rtc<'static>, rtc);

    info!("Spawning network tasks");

    // Spawn network tasks
    if let Ok(t) = connection_task(controller) { spawner.spawn(t); }
    if let Ok(t) = net_task(runner) { spawner.spawn(t); }
    if let Ok(t) = sntp_task(rtc_static, *stack_static) { spawner.spawn(t); }
    if let Ok(t) = comms_status_signaller_task(rtc_static) { spawner.spawn(t); }
    info!("Network and CommsStatus tasks spawned");

    // Wait for network
    info!("Waiting for link...");
    loop {
        if net_stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    info!("Waiting for IP address...");
    loop {
        if let Some(config) = net_stack.config_v4() {
            info!("Got IP: {}", config.address);
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    info!("Network ready");

    // Create TCP stack for HTTP
    let tcp_buffers = mk_static!(TcpBuffers<16, 1024, 1024>, TcpBuffers::new());
    let tcp_stack = mk_static!(Tcp<'static>, Tcp::new(net_stack, tcp_buffers));

    // Get command sender for HTTP server
    let command_sender = mk_static!(MachineCommandSender, command_channel.sender());

    // Create subscribers for cache update task
    let http_status_subscriber = status_channel.subscriber().unwrap();
    let http_config_subscriber = config_channel.subscriber().unwrap();

    // Spawn HTTP server and cache update tasks
    if let Ok(t) = http_server_task(tcp_stack, command_sender) { spawner.spawn(t); }
    if let Ok(t) = cache_update_task(http_status_subscriber, http_config_subscriber) { spawner.spawn(t); }
    info!("HTTP server and cache update tasks spawned");

    // Create subscribers for ESPHome server
    let esphome_status_subscriber = status_channel.subscriber().unwrap();
    let esphome_config_subscriber = config_channel.subscriber().unwrap();
    let esphome_command_config_subscriber = config_channel.subscriber().unwrap();

    // Spawn ESPHome server task
    if let Ok(t) = esphome_server_task(
        stack_static,
        esphome_status_subscriber,
        esphome_config_subscriber,
        esphome_command_config_subscriber,
        state_change_channel,
        client_event_channel,
        command_channel,
    ) { spawner.spawn(t); }
    info!("ESPHome server task spawned on port 6053");

    // Create subscribers for WebSocket server
    let ws_status_subscriber = status_channel.subscriber().unwrap();
    let ws_config_subscriber = config_channel.subscriber().unwrap();
    let ws_routine_subscriber = routine_channel.subscriber().unwrap();

    // Spawn WebSocket server task
    if let Ok(t) = websocket_server_task(
        stack_static,
        ws_status_subscriber,
        ws_config_subscriber,
        ws_routine_subscriber,
        command_channel,
    ) { spawner.spawn(t); }
    info!("WebSocket server task spawned on port 8080");

    // Main loop - periodic HTTP client requests
    loop {
        Timer::after(Duration::from_millis(5000)).await;
    }
}
