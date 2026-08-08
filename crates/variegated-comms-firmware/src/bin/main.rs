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
    instrumentation,
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
        "\r\n====================== PANIC ======================\r\n{info}\r\n"
    );
    write_trap_csrs(&mut console);
    let _ = write!(console, "\r\nBacktrace:\r\n");
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

/// Print the RISC-V trap CSRs, because for a CPU exception they are the whole diagnosis
/// and nothing else in this handler carries it.
///
/// esp-hal's `ExceptionHandler` already reads `mcause`/`mepc`/`mtval` and panics with
/// them formatted into the message -- `"Exception 'Load access fault' mepc=0x... "`, or
/// `"Stack overflow detected at 0x..."` for `mcause == 14`. That message never arrives.
/// esp-hal is built with defmt, so its `panic!` is `defmt::panic!`, which hands the
/// formatted arguments to the defmt global logger and then calls
/// `defmt::export::panic()`. Task 9 set `esp-println` to `no-op` for reasons that still
/// hold, which left defmt with no sink -- so the arguments are discarded and
/// `__defmt_default_panic` re-panics with the bare string `"explicit panic"`. That is
/// what `{info}` prints, and it says nothing at all.
///
/// The registers themselves survive: the panic path takes no further trap, so by the
/// time this runs they still hold what the exception handler read. Reading them here
/// recovers the diagnosis that the defmt hop threw away, for the cost of three `csrr`s.
///
/// Inline asm rather than the `riscv` crate: this is the code least able to afford a
/// dependency, and `csrr` needs no abstraction.
///
/// **These are only meaningful if the panic came from a trap.** An ordinary `panic!` or
/// a failed `unwrap` leaves whatever the last trap wrote, which on this chip is usually
/// an interrupt. Bit 31 is the discriminator -- set means interrupt, clear means
/// exception -- so it is printed rather than interpreted away.
fn write_trap_csrs(console: &mut debug::panic_console::PanicConsole) {
    let (mcause, mepc, mtval): (usize, usize, usize);
    unsafe {
        core::arch::asm!("csrr {}, mcause", out(reg) mcause, options(nomem, nostack));
        core::arch::asm!("csrr {}, mepc", out(reg) mepc, options(nomem, nostack));
        core::arch::asm!("csrr {}, mtval", out(reg) mtval, options(nomem, nostack));
    }

    let is_interrupt = mcause >> 31 != 0;
    let code = mcause & 0x7fff_ffff;
    // The RISC-V privileged spec's machine-mode exception codes, matching the table in
    // `esp_hal::exception_handler`. Code 14 is where esp-hal reports a stack overflow.
    let name = match (is_interrupt, code) {
        (true, _) => "interrupt (not an exception -- CSRs likely stale)",
        (false, 0) => "Instruction address misaligned",
        (false, 1) => "Instruction access fault",
        (false, 2) => "Illegal instruction",
        (false, 3) => "Breakpoint",
        (false, 4) => "Load address misaligned",
        (false, 5) => "Load access fault",
        (false, 6) => "Store/AMO address misaligned",
        (false, 7) => "Store/AMO access fault",
        (false, 11) => "Environment call from M-mode",
        (false, 14) => "STACK OVERFLOW",
        _ => "unknown",
    };

    let _ = write!(
        console,
        "\r\ntrap: {name}\r\n  mcause=0x{mcause:08x} mepc=0x{mepc:08x} mtval=0x{mtval:08x}\r\n"
    );
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

/// The 1 Hz comms-status loop.
///
/// It also mirrors the current DHCP lease into `WIFI_IPV4` for the debug snapshot.
/// That lives here rather than in `debug::snapshot` because the snapshot task is
/// spawned long before the network stack exists and takes no arguments, and rather
/// than beside `main`'s "waiting for IP" loop because that loop runs once: a lease
/// can change on reconnect, and a latched address goes on asserting one the device no
/// longer holds. This task is the only 1 Hz loop that already holds the stack.
#[embassy_executor::task]
async fn comms_status_signaller_task(
    rtc: &'static esp_hal::rtc_cntl::Rtc<'static>,
    stack: embassy_net::Stack<'static>,
) {
    use variegated_comms_firmware::channels::{BELKA_CONNECTION_STATUS, COMMS_STATUS_SIGNAL, NO_IPV4, SCALE_CONNECTION_STATUS, TIME_SYNCED, WIFI_CONNECTED, WIFI_IPV4, WIFI_RSSI_SIGNAL};
    use variegated_comms_firmware::config::{BELKA_PERIPHERAL_ID, BLUETOOTH_GROUP_1_SCALE_PERIPHERAL_ID, USEC_IN_SEC};
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

        // Get group 1 scale connection status (updated by acaia_measurement_loop)
        let scale_connected = SCALE_CONNECTION_STATUS.load(Ordering::Relaxed);

        // Refresh the debug snapshot's view of the DHCP lease. `config_v4` is `None`
        // before DHCP completes and again once the lease is dropped, and `NO_IPV4`
        // carries that through as `None` rather than as `0.0.0.0`.
        WIFI_IPV4.store(
            stack
                .config_v4()
                .map(|config| config.address.address().to_bits())
                .unwrap_or(NO_IPV4),
            Ordering::Relaxed,
        );

        // Build peripheral connection status map
        //
        // This map is the *only* way the application processor learns that a comms-owned
        // peripheral is reachable: `variegated_comms` turns each entry into a
        // `dispatch_connection_status` call, and the devices on that side drop readings
        // until they get one. A peripheral missing from here streams data that is
        // silently discarded on arrival.
        let mut peripheral_connection_status = FnvIndexMap::new();
        let _ = peripheral_connection_status.insert(
            BELKA_PERIPHERAL_ID,
            WirelessConnectionStatus {
                connected: belka_connected,
                rssi: None,
            }
        );
        let _ = peripheral_connection_status.insert(
            BLUETOOTH_GROUP_1_SCALE_PERIPHERAL_ID,
            WirelessConnectionStatus {
                connected: scale_connected,
                // The connection manager exposes no per-connection RSSI, so this stays
                // `None` rather than guessing -- same as Belka above.
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

/// The structured debug stream's network transport: one client at a time on 9090,
/// carrying frames from both processors.
///
/// Spawned late, because it needs the network stack, but its bus subscriber is
/// claimed in `main` before the first publish -- see the claim site for why the two
/// cannot be brought together.
#[embassy_executor::task]
async fn debug_tcp_task(
    stack: &'static embassy_net::Stack<'static>,
    subscriber: Option<debug::BusSubscriber>,
    sink: debug::CommandSink,
) {
    debug::tcp::run(stack, subscriber, sink).await;
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
    debug_command_receiver: embassy_sync::channel::Receiver<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        variegated_controller_types::debug_command::DebugCommand,
        { variegated_comms_firmware::channels::DEBUG_COMMAND_CAPACITY },
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
        debug_command_receiver,
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
    // The second and last slot, claimed on the same terms and for a stronger version
    // of the same reason. The TCP server is not spawned until the network is up, tens
    // of seconds from now; a subscriber claimed at *that* point would begin at the
    // bus's current `next_message_id` and could never see one frame of the boot.
    //
    // The cost of claiming early and reading late is a single large `Lagged` on the
    // server's first read, which it counts frame by frame -- the same accounting the
    // USB writer produces while no host is attached. It does not cost the USB writer
    // anything: `publish_immediate` evicts the *oldest* frame when the ring is full,
    // and the oldest is one this subscriber has not read rather than one the writer
    // is waiting on, so the writer's slack is still the full 16 slots.
    let tcp_debug_subscriber = debug::bus::subscriber();

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

    // The `#[ram(reclaimed)]` heap comes out of memory the ROM bootloader was using and
    // costs the stack nothing, so it stays at 64 kB.
    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
    // 64 -> 48 kB, buying 16 kB of stack.
    //
    // History, because both directions of this have already drawn blood. `.stack` is the
    // SRAM *remainder*, so every static here is a subtraction from the one stack the
    // deepest deserialization recurses into: 71704 bytes overflows and 87256 does not.
    // But squeezing this allocator to 40 kB broke the other way -- the ESPHome server's
    // `Vec<EntityConfig>` needs a single 12000-byte contiguous block
    // (`esphome/server.rs:56`) and `handle_alloc_error` fired. 48 kB is deliberately on
    // the safe side of that: 8 kB above the figure known to fail, and against a peak
    // occupancy measured at 28216 bytes across both regions.
    //
    // What made the old 87256 insufficient is that the deepest excursion got deeper
    // rather than the budget getting smaller. The frontend's WebSocket client only
    // started running once it could build again, and `WsMessage` is an enum *wrapping*
    // `Configuration`, `Status`, `MachineDefinition` and `RoutineStorage` -- so
    // `postcard::from_bytes::<WsMessage>` on every client message, and `to_allocvec` for
    // status pushes at up to 5 Hz, recurse strictly deeper than the
    // `from_bytes_cobs::<..Configuration>` path those numbers were measured against.
    //
    // The failure this addresses did not look like a stack overflow. It arrived as a
    // load access fault at 0xd1371488 inside
    // `embassy_time_queue_utils::Queue::next_expiration`, because the overrun ran past
    // the guard and corrupted a `next` pointer in embassy's intrusive timer list, which
    // lives in task headers in `.bss`. The accompanying "explicit panic" is defmt with
    // no sink in this build, not the real message. Same shape as the last one.
    esp_alloc::heap_allocator!(size: 48 * 1024);

    // Initialize application processor channels
    let status_channel = STATUS_CHANNEL.init(embassy_sync::pubsub::PubSubChannel::new());
    let config_channel = CONFIGURATION_CHANNEL.init(embassy_sync::pubsub::PubSubChannel::new());
    let routine_channel = ROUTINE_CHANNEL.init(embassy_sync::pubsub::PubSubChannel::new());
    let command_channel = MACHINE_COMMAND_CHANNEL.init(embassy_sync::channel::Channel::new());
    let sensor_reading_channel = SENSOR_READING_CHANNEL.init(embassy_sync::channel::Channel::new());

    // Initialize the debug command channel. Filled by the USB-Serial-JTAG reader
    // always, and by the TCP reader when `config::TCP_COMMANDS_ENABLED`; drained by
    // the application-processor sender, which dispatches through
    // `debug::commands::dispatch`.
    let debug_command_channel = DEBUG_COMMAND_CHANNEL.init(embassy_sync::channel::Channel::new());

    // Bring up the structured debug transport before anything else that might have
    // something to say. `esp-println` no longer writes anywhere at all (Cargo.toml
    // sets it to `no-op`), so the stream owns this peripheral outright.
    //
    // `split()` returns (rx, tx) in that order -- not the (tx, rx) that most of the
    // rest of esp-hal uses.
    let usb = esp_hal::usb_serial_jtag::UsbSerialJtag::new(peripherals.USB_DEVICE).into_async();
    let (usb_rx, usb_tx) = usb.split();
    // The station MAC, mirrored for `CommsState::wifi_mac`.
    //
    // Read here, before the snapshot task is spawned, so no snapshot can ever observe
    // the "not published yet" sentinel -- eFuse is readable this early and the value
    // is fixed for the life of the board, so there is nothing to re-read later.
    //
    // `interface_mac_address(Station)` rather than `base_mac_address()`: the two are
    // the same bytes today (the station interface uses the base MAC unmodified) but
    // the station one is what actually goes on air, and it follows
    // `override_mac_address` if that is ever called.
    {
        use variegated_comms_firmware::channels::{store_address48, WIFI_MAC};
        let mac = esp_hal::efuse::interface_mac_address(
            esp_hal::efuse::InterfaceMacAddress::Station,
        );
        let mut bytes = [0u8; 6];
        bytes.copy_from_slice(mac.as_bytes());
        store_address48(&WIFI_MAC, bytes);
    }

    spawn_or_report!(spawner, "debug_usb", debug_usb_task(usb_rx, usb_tx, debug_command_channel.sender(), debug_subscriber));
    // The 1 Hz `CommsState` snapshot. Spawned next to the transport rather than with
    // the network tasks: it reads atomics and bus counters only, so it is useful
    // from the first second of the boot and does not depend on anything below.
    spawn_or_report!(spawner, "debug_snapshot", debug::snapshot::snapshot_task());
    // Alongside the snapshot rather than with the network tasks, even though the network
    // owns most of the metrics: the counter and indicator arrays are statics, so this can
    // and should publish from the first second of the boot rather than from whenever the
    // Wi-Fi stack finishes coming up. It also emits the schema, which a host needs before
    // any sample means anything.
    spawn_or_report!(spawner, "debug_sampler", instrumentation::sampler_task());
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
    spawn_or_report!(spawner, "application_processor", application_processor_task(rx, tx, status_channel, config_channel, routine_channel, command_channel, sensor_reading_channel, debug_command_channel.receiver()));
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
    // Mirror the *same* array the controller is about to advertise, rather than
    // generating a second one for reporting: a second draw would put an address on
    // screen that no scanner will ever see.
    {
        use variegated_comms_firmware::channels::{store_address48, BT_ADDRESS};
        store_address48(&BT_ADDRESS, address_bytes);
    }
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
    //
    // The queue sizes are raised off esp-radio's defaults of 5 and 3, which its own
    // docs describe as "quite conservative". On this firmware the RX default was not
    // merely conservative, it was a latch.
    //
    // `recv_cb_sta` (esp-radio wifi/mod.rs:1004) pushes each received frame onto
    // `DATA_QUEUE_RX_STA` and then wakes embassy-net's receive waker -- but only on the
    // success path. When the queue is already at `rx_queue_size` the frame is dropped
    // and *the waker is not fired*. That is self-sustaining: a full queue stops the
    // wake, no wake means embassy-net never polls, never polling means the queue is
    // never drained, and it stays full. The only thing that breaks the cycle is
    // smoltcp's own poll timer, seconds later.
    //
    // Five slots is nothing on a real network -- ARP and mDNS broadcast alone will fill
    // it -- so the stack spent most of its time latched. The symptom was not lost
    // throughput but latency measured in seconds: a `curl` to `/` spent 6.5 s in
    // connect (the SYN ladder, retransmitting into a stack that was not looking) and
    // another 3.7 s to first byte, then transferred the whole body in 0.3 ms. ICMP
    // never got answered at all, because a ping's timeout is shorter than the gap
    // between polls.
    //
    // 32 and 16 cost almost nothing: the queue holds `PacketBuffer` handles, and the
    // buffers they refer to are already accounted for by `dynamic_rx_buf_num`, which
    // defaults to 32 and is left alone. Sized under that deliberately, so the queue can
    // never own more buffers than the driver has.
    let radio_config = esp_radio::wifi::ControllerConfig::default()
        .with_rx_queue_size(32)
        .with_tx_queue_size(16);
    let (controller, interfaces) =
        esp_radio::wifi::new(peripherals.WIFI, radio_config).unwrap();

    let wifi_interface = interfaces.station;

    log_info!("Creating network stack");

    // Create network stack
    let net_config = embassy_net::Config::dhcpv4(Default::default());
    let rng = Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    // Wrapped so the counters in `instrumentation` see every call embassy-net makes into
    // the driver, and every frame it collects. See that module for what they mean.
    let (net_stack, runner) = embassy_net::new(
        instrumentation::CountingDriver::new(wifi_interface),
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
    spawn_or_report!(spawner, "comms_status_signaller", comms_status_signaller_task(rtc_static, *stack_static));
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

    // The debug stream's network transport, spawned first among the servers: it is
    // the one that reports on the others, and a `SpawnFailed` for anything below is
    // only useful to a host that can already receive it.
    spawn_or_report!(spawner, "debug_tcp", debug_tcp_task(stack_static, tcp_debug_subscriber, debug_command_channel.sender()));
    log_info!("TCP debug server task spawned on port 9090");

    // Create TCP stack for HTTP
    // 16 -> 4 concurrent HTTP connections, i.e. 32768 -> 8192 bytes of static buffers.
    //
    // This was the single largest static allocation in the firmware, and larger than
    // everything the debug feature adds put together. It mattered because `.stack` is
    // the SRAM *remainder*: 32 kB reserved here is 32 kB the main task's stack does not
    // have, and that stack is what `postcard::from_bytes_cobs::<..Configuration>`
    // recurses into on every configuration update. esp-rtos caught the overflow; the
    // symptom reaching the bench was a null-pointer load somewhere in the radio blob,
    // at a different address every boot, because the overrun ran past the stack guard
    // into whatever lived below it.
    //
    // Both buffers are 4096 rather than the 1024 default, because 1024 made serving the
    // 48 kB JS bundle take minutes and a 1 kB response take 10-15 seconds.
    //
    // embassy-net enables Nagle's algorithm by default, and Nagle will not emit a
    // segment smaller than the MSS while data is in flight. esp-radio's default MTU is
    // 1492, so the MSS is 1452 -- and a 1024-byte buffer *can never hold a full
    // segment*. Every write therefore waited for the previous ACK; and because only one
    // undersized segment was ever outstanding, the peer's "ACK every second full
    // segment" rule never fired and it fell back to its delayed-ACK timer. One sub-MSS
    // segment per 200-500 ms is 2-5 kB/s no matter how fast the radio is. embassy-net's
    // own docs name this interaction: Nagle costs "increased latency ... particularly
    // when the remote peer has ACK delay enabled".
    //
    // 4096 is sized against 2 * MSS = 2904, not against MSS. Merely clearing 1452 would
    // leave a sub-MSS remainder after each full segment and hit the same wall; holding
    // *two* full segments is what makes the peer ACK immediately, removing the stall
    // rather than halving it.
    //
    // Symmetric because the trap is symmetric. A 1024-byte receive window puts the
    // browser in the same position on the way in, which is what POSTing a routine or a
    // schedule does -- those bodies are multi-kB postcard.
    //
    // The socket count here MUST equal the handler-task count of the `Server` in
    // `http.rs`. Each of those tasks waits in `accept()` simultaneously and holds a
    // socket from this pool while it does, because smoltcp has no accept queue. Leaving
    // `DefaultServer` (which is `Server<4, ..>`) against a pool of 2 stopped the server
    // listening entirely: port 80 refused connections rather than serving them slowly.
    //
    // 4 -> 2 connections is what pays for it. At 1 kB per buffer a connection cost 2 kB
    // and four was nearly free; at 4 kB it costs 8 kB, and `.stack` is the SRAM
    // remainder that the deepest deserialization has to survive (see the heap allocator
    // note above). Two still covers this workload: the SPA is `index.html` plus one
    // bundle, and status, configuration and routines all travel over the WebSocket now,
    // which has its own socket rather than one of these. Two 4 kB connections is both
    // faster and 4 kB *cheaper* than the four 5 kB ones it replaces.
    let tcp_buffers = mk_static!(TcpBuffers<2, 4096, 4096>, TcpBuffers::new());
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
