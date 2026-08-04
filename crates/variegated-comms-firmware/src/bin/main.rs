//! Variegated Comms Firmware
//!
//! WiFi and BLE coexistence firmware for ESP32-C6.
//! Handles communication with application processor via UART,
//! BLE devices (Belka Portal, ACAIA scale), and provides HTTP server.

#![no_std]
#![no_main]

use core::fmt::Write as _;
use core::net::{Ipv4Addr, SocketAddr};

use bt_hci::controller::ExternalController;
use variegated_log::log_info;
use edge_http::io::client::Connection;
use edge_http::Method;
use edge_nal_embassy::{Tcp, TcpBuffers};
use embassy_executor::Spawner;
use embassy_net::StackResources;
use embassy_time::{Duration, Timer};
use embedded_io_async::Read;
use esp_alloc as _;
use esp_backtrace::Backtrace;
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
use variegated_controller_types::debug::{name, text, DebugEvent, Severity};
use variegated_trouble_connection_manager::BleConnectionManager;

esp_bootloader_esp_idf::esp_app_desc!();

/// Spawn a task, reporting a failure instead of swallowing it.
///
/// embassy-executor 0.10 moved fallibility from `Spawner::spawn` onto the `#[task]`
/// function, which returns `Err` when that task's pool is exhausted. Every spawn
/// site in `main` used to be `if let Ok(t) = f(..) { spawner.spawn(t); }`, i.e. a
/// missing task produced no panic, no log line and no symptom other than the
/// machine quietly not doing something (open question #5 in `JULY-UPGRADE-STATUS`).
///
/// A typed event is the right answer here rather than `unwrap`: this firmware
/// carries the machine's radios, and halting the whole chip because one task could
/// not start is worse than starting the rest and saying which one is missing. It is
/// also better than a `log_*!`, because the suppressor can collapse text and a host
/// filtering on `spawn_failed` cannot miss this.
///
/// Edge triggered by construction: `main` runs once, so each of these executes
/// exactly once per boot and only the failing ones emit. The worst case is bounded
/// by the number of spawn sites.
macro_rules! spawn_or_report {
    ($spawner:expr, $task_name:literal, $token:expr) => {
        match $token {
            Ok(t) => $spawner.spawn(t),
            Err(_) => debug::bus::emit_event(DebugEvent::SpawnFailed {
                task: name($task_name),
            }),
        }
    };
}

/// Get the backtrace out, then stop.
///
/// This replaces `esp-backtrace`'s own handler, which was doing nothing: it formats
/// everything through `esp_println::println!`, and `esp-println` is `no-op` in this
/// build (see Cargo.toml for the two reasons that must not be undone). The handler
/// looked present and produced no bytes, which is the worst of both -- a panic here
/// halted the chip in silence.
///
/// The shape of the output is deliberately `esp-backtrace`'s, banner and `0x…` frames
/// and all, so `espflash` and the existing addr2line habits keep working on it.
///
/// # Ordering
///
/// Interrupts are cleared **first**, before anything is written. "The executor is
/// gone" is true of embassy's cooperative scheduling but not of this chip: `esp-rtos`
/// runs preemptive threads, and a timer tick during the write could schedule another
/// one on top of a half-emitted backtrace. Clearing `MIE` makes the claim true rather
/// than assuming it.
///
/// The capture happens in this frame rather than inside `panic_console`, so the
/// backtrace is rooted at the same depth `esp-backtrace`'s handler rooted it at and
/// the frame list means what it used to mean.
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    // riscv32: clear `mstatus.MIE` (bit 3). Nothing may run between here and the halt
    // loop but this function.
    unsafe { core::arch::asm!("csrci mstatus, 8") };

    let backtrace = Backtrace::capture();
    let mut console = unsafe { debug::panic_console::PanicConsole::seize() };

    // Closes whatever COBS frame was in flight, so the wreckage of it is judged
    // separately from the text that follows. See `panic_console`'s module docs.
    // `write_delimiter`, not `write_bytes`: the latter substitutes anything the host
    // would refuse as text, and `0x00` is the one byte that has to get through raw.
    console.write_delimiter();
    let _ = write!(
        console,
        "\r\n====================== PANIC ======================\r\n{info}\r\n\r\nBacktrace:\r\n"
    );
    if backtrace.frames().is_empty() {
        // The `.cargo/config.toml` in this repo sets `force-frame-pointers`, without
        // which the walk finds nothing. Say so rather than printing an empty list,
        // which reads as "the panic had no caller".
        let _ = write!(console, "no frames -- build without force-frame-pointers?\r\n");
    }
    for frame in backtrace.frames() {
        let _ = write!(console, "0x{:x}\r\n", frame.program_counter());
    }
    let _ = write!(console, "==================== END PANIC ====================\r\n");
    // Terminates the text run so a host renders it now. Nothing else on this wire is
    // ever going to send another delimiter.
    console.write_delimiter();
    console.flush();

    loop {
        core::hint::spin_loop();
    }
}

#[unsafe(no_mangle)]
pub extern "Rust" fn _esp_println_timestamp() -> u64 {
    esp_hal::time::Instant::now()
        .duration_since_epoch()
        .as_millis()
}

#[embassy_executor::task]
async fn status_listener_task(status_channel: &'static ApplicationStatusChannel) {
    let mut subscriber = status_channel.subscriber().unwrap();
    log_info!("Status listener task started");
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

    log_info!("CommsStatus signaller task started");
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
/// share. `esp-println` is now `no-op` -- it has no output target at all, on this or
/// any other peripheral (see its entry in Cargo.toml) -- so this task owns
/// USB-Serial-JTAG outright while the executor is running. The one other writer is
/// the panic handler above, which takes it by force after the executor has stopped.
#[embassy_executor::task]
async fn debug_usb_task(
    usb_rx: esp_hal::usb_serial_jtag::UsbSerialJtagRx<'static, esp_hal::Async>,
    usb_tx: esp_hal::usb_serial_jtag::UsbSerialJtagTx<'static, esp_hal::Async>,
    sink: debug::CommandSink,
    subscriber: Option<debug::BusSubscriber>,
) {
    debug::usb::run(usb_rx, usb_tx, sink, subscriber).await;
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

    // Claim the debug bus's reader slot *here*, synchronously, before anything is
    // published -- not inside `debug_usb_task`, where it used to be claimed.
    //
    // `main` is itself a task. It does not yield until its first `.await`, which is
    // the `Timer::after_secs(5)` well over a hundred lines below, so no task spawned
    // in between runs until then. A subscriber claimed inside one of those tasks is
    // therefore claimed *after* every frame this function publishes.
    //
    // That is not a lag, it is a discard. `embassy_sync`'s pubsub short-circuits a
    // publish when nobody is listening -- `try_publish` returns `Ok(())` without
    // touching the queue when `subscriber_count == 0`
    // (`embassy-sync-0.8.0/src/pubsub/mod.rs:332-336`) -- and `subscriber()` starts a
    // new reader at the current `next_message_id` (`:100`), so it cannot recover what
    // it did not witness. Ring capacity has nothing to do with it: the frame never
    // reaches the ring.
    //
    // What that cost, before this line existed: `DebugEvent::Boot`, the `SpawnFailed`
    // reports for the first six tasks -- including `application_processor`, whose
    // silent absence is exactly what this feature exists to make visible -- and every
    // `log_*!` emitted during bring-up, which is most of the boot log.
    //
    // Claiming it here also makes the guarantee checkable by reading rather than by
    // reasoning about the executor: the slot is taken on this line, the first publish
    // is nine lines down, and there is no `.await` between them.
    let debug_subscriber = debug::bus::subscriber();

    // Install the `log` -> debug-bus sink first, because the `log` facade
    // *discards* every record emitted before a logger exists and there is no
    // replay. Every `log_*!` in this firmware is silent until this line runs.
    //
    // This early is safe on both counts that could bite:
    //   * `bus_sink` timestamps with `embassy_time::Instant::now()`, which
    //     esp-rtos implements over the SYSTIMER (`esp-rtos-0.3.0/src/lib.rs:460`
    //     -> `esp_hal::time::Instant::now()`). That counter is running from
    //     `esp_hal::init` above, not from `esp_rtos::start` further down.
    //   * it never allocates -- the record is formatted into a fixed
    //     `heapless::String<96>` -- so it does not need the heap allocators below.
    //
    // esp-println is `no-op` in this build, so this sink and the USB-Serial-JTAG
    // transport are between them the *only* way a log line leaves this chip.
    if variegated_log::bus_sink::init().is_err() {
        // `log` permits exactly one logger, so this means something else in the
        // graph installed one -- a static property of the build, not a runtime
        // condition. Reported on the bus directly rather than through `log`,
        // which by definition is not working, because the alternative is that
        // every log line in this firmware silently goes nowhere and the stream
        // looks merely quiet.
        debug::bus::emit_text(
            Severity::Error,
            text("log bus sink not installed: another logger won the race"),
        );
    }

    // The first frame of the boot. Edge triggered in the most literal sense: this
    // line runs once per power cycle, and it is what tells a host attached across a
    // reset that the sequence numbers restarting is a reboot and not a gap.
    //
    // Emitted here, immediately after the sink exists, so it precedes every log line
    // this firmware produces rather than landing in the middle of them -- and after
    // the `subscriber()` call above, without which it would not be emitted at all.
    debug::bus::emit_event(DebugEvent::Boot);

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
    // something to say. `esp-println` no longer writes anywhere at all (Cargo.toml
    // sets it to `no-op`), so the stream owns this peripheral outright.
    //
    // `split()` returns (rx, tx) in that order -- not the (tx, rx) that most of the
    // rest of esp-hal uses.
    let usb = esp_hal::usb_serial_jtag::UsbSerialJtag::new(peripherals.USB_DEVICE).into_async();
    let (usb_rx, usb_tx) = usb.split();
    spawn_or_report!(spawner, "debug_usb", debug_usb_task(usb_rx, usb_tx, debug_command_channel.sender(), debug_subscriber));
    // The 1 Hz `CommsState` snapshot. Spawned next to the transport rather than with
    // the network tasks: it reads atomics and bus counters only, so it is useful
    // from the first second of the boot and does not depend on anything below.
    spawn_or_report!(spawner, "debug_snapshot", debug::snapshot::snapshot_task());
    log_info!("Debug USB-Serial-JTAG transport spawned");

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
    // now returns `()`) onto the `#[task]` function itself. A failed spawn used to
    // be discarded silently; `spawn_or_report!` turns it into a `SpawnFailed` event
    // instead. See the macro's doc comment for why an event and not `unwrap`.
    spawn_or_report!(spawner, "application_processor", application_processor_task(rx, tx, status_channel, config_channel, routine_channel, command_channel, sensor_reading_channel));
    spawn_or_report!(spawner, "status_listener", status_listener_task(status_channel));
    log_info!("Application processor tasks spawned");

    // Initialize esp-rtos
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    log_info!("Initializing radio");

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
    log_info!("BLE: Generated random address");

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
    spawn_or_report!(spawner, "ble_runner", ble_runner_task(runner, printer));
    spawn_or_report!(spawner, "ble_devices", ble_devices_task(connection_manager, stack, belka_address(), acaia_address(), sensor_reading_sender));
    log_info!("BLE tasks spawned");

    Timer::after_secs(5).await;

    log_info!("Initializing Wifi");

    // Initialize WiFi. The radio controller handle is gone in 0.18, and
    // `interfaces.sta` was renamed `interfaces.station`. Configuration stays in
    // `connection_task`, which now calls `set_config` -- that both configures
    // and starts the controller, since `start_async` was removed.
    let (controller, interfaces) =
        esp_radio::wifi::new(peripherals.WIFI, Default::default()).unwrap();

    let wifi_interface = interfaces.station;

    log_info!("Creating network stack");

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

    log_info!("Spawning network tasks");

    // Spawn network tasks
    spawn_or_report!(spawner, "wifi_connection", connection_task(controller));
    spawn_or_report!(spawner, "net", net_task(runner));
    spawn_or_report!(spawner, "sntp", sntp_task(rtc_static, *stack_static));
    spawn_or_report!(spawner, "comms_status_signaller", comms_status_signaller_task(rtc_static));
    log_info!("Network and CommsStatus tasks spawned");

    // Wait for network
    log_info!("Waiting for link...");
    loop {
        if net_stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    log_info!("Waiting for IP address...");
    loop {
        if let Some(config) = net_stack.config_v4() {
            log_info!("Got IP: {}", config.address);
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    log_info!("Network ready");

    // Create TCP stack for HTTP
    let tcp_buffers = mk_static!(TcpBuffers<16, 1024, 1024>, TcpBuffers::new());
    let tcp_stack = mk_static!(Tcp<'static>, Tcp::new(net_stack, tcp_buffers));

    // Get command sender for HTTP server
    let command_sender = mk_static!(MachineCommandSender, command_channel.sender());

    // Create subscribers for cache update task
    let http_status_subscriber = status_channel.subscriber().unwrap();
    let http_config_subscriber = config_channel.subscriber().unwrap();

    // Spawn HTTP server and cache update tasks
    spawn_or_report!(spawner, "http_server", http_server_task(tcp_stack, command_sender));
    spawn_or_report!(spawner, "cache_update", cache_update_task(http_status_subscriber, http_config_subscriber));
    log_info!("HTTP server and cache update tasks spawned");

    // Create subscribers for ESPHome server
    let esphome_status_subscriber = status_channel.subscriber().unwrap();
    let esphome_config_subscriber = config_channel.subscriber().unwrap();
    let esphome_command_config_subscriber = config_channel.subscriber().unwrap();

    // Spawn ESPHome server task
    spawn_or_report!(spawner, "esphome_server", esphome_server_task(
        stack_static,
        esphome_status_subscriber,
        esphome_config_subscriber,
        esphome_command_config_subscriber,
        state_change_channel,
        client_event_channel,
        command_channel,
    ));
    log_info!("ESPHome server task spawned on port 6053");

    // Create subscribers for WebSocket server
    let ws_status_subscriber = status_channel.subscriber().unwrap();
    let ws_config_subscriber = config_channel.subscriber().unwrap();
    let ws_routine_subscriber = routine_channel.subscriber().unwrap();

    // Spawn WebSocket server task
    spawn_or_report!(spawner, "websocket_server", websocket_server_task(
        stack_static,
        ws_status_subscriber,
        ws_config_subscriber,
        ws_routine_subscriber,
        command_channel,
    ));
    log_info!("WebSocket server task spawned on port 8080");

    // Main loop - periodic HTTP client requests
    loop {
        Timer::after(Duration::from_millis(5000)).await;
    }
}
