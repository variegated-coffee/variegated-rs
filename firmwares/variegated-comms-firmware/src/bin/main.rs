//! Variegated Comms Firmware
//!
//! WiFi and BLE coexistence firmware for ESP32-C6.
//! Handles communication with application processor via UART,
//! BLE devices (Belka Portal, ACAIA scale), and provides HTTP server.

#![no_std]
#![no_main]

use core::fmt::Write as _;

use bt_hci::controller::ExternalController;
use variegated_log::log_info;
use edge_nal_embassy::{Tcp, TcpBuffers};
use embassy_executor::Spawner;
use embassy_net::StackResources;
use embassy_time::{with_timeout, Duration, Timer};
use esp_alloc as _;
use esp_backtrace::Backtrace;
use esp_hal::{
    clock::CpuClock,
    interrupt::software::SoftwareInterruptControl,
    ram,
    rng::Rng,
    rtc_cntl::Rtc,
    timer::timg::{MwdtStage, TimerGroup},
};
// `as _`, and it must stay. Nothing here calls into esp-println, but the crate carries the
// `#[defmt::global_logger]` this firmware logs through, and an extern crate that is never
// named is an extern crate the linker discards -- `--gc-sections` then takes the logger
// with it and the build fails with `undefined symbol: _defmt_write`, which names neither
// esp-println nor defmt's global_logger.
//
// This was `use esp_println::println;` and `cargo fix` deleted it as an unused import,
// correctly by its own lights and fatally by ours. Same idiom as `esp_alloc as _` above,
// for the same reason.
use esp_println as _;
use esp_radio::ble::controller::BleConnector;
use trouble_host::prelude::*;

// Library imports
use variegated_checkin::watch;
use variegated_comms_firmware::{
    application_processor,
    ble::{ble_devices_task, ble_runner_task, ble_slot_task, ScanPrinter},
    checkin::{checkin_task, CheckinId, MONITOR},
    channels::{
        ApplicationConfigurationChannel, ApplicationStatusChannel, ApplicationRoutineChannel,
        ShotLogEventChannel, SHOT_LOG_EVENT_CHANNEL,
        CONFIGURATION_CHANNEL, MACHINE_COMMAND_CHANNEL, STATUS_CHANNEL, ROUTINE_CHANNEL,
        MachineCommandSender, STATE_CHANGE_CHANNEL, CLIENT_EVENT_CHANNEL, SENSOR_READING_CHANNEL,
        DEBUG_COMMAND_CHANNEL, INPUT_COMMAND_CHANNEL, BOND_REPORT_CHANNEL,
    },
    config::{debug_uart_config, uart_config},
    debug,
    esphome::esphome_server_task,
    http::{http_server_task, cache_update_task},
    improv,
    mk_static,
    instrumentation,
    time::sntp_task,
    watchdog,
    websocket_server_task,
    wifi::{connection_task, net_task},
};
use variegated_controller_types::bluetooth::MAX_BLUETOOTH_PERIPHERALS;
use variegated_controller_types::debug::{name, text, DebugEvent, Severity};
use variegated_trouble_connection_manager::BleConnectionManager;

esp_bootloader_esp_idf::esp_app_desc!();

/// Spawn a task, reporting a failure instead of swallowing it.
///
/// embassy-executor 0.10 moved fallibility from `Spawner::spawn` onto the `#[task]`
/// function, which returns `Err` when that task's pool is exhausted. Written as
/// `if let Ok(t) = f(..) { spawner.spawn(t); }`, a spawn site swallows that: a missing
/// task produces no panic, no log line and no symptom other than the machine quietly
/// not doing something.
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
/// `defmt::export::panic()`. Whenever that logger has no sink -- which is what selecting
/// `esp-println`'s `no-op` output target does -- the arguments are discarded and
/// `__defmt_default_panic` re-panics with the bare string `"explicit panic"`. `{info}`
/// then says nothing at all, and this handler is the only thing standing between that
/// and a silent reboot.
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
    let mut subscriber = status_channel
        .subscriber()
        .expect("APPLICATION_STATUS_RECEIVERS must count this subscriber");
    let checkin = MONITOR.claim(CheckinId::StatusListener);
    log_info!("Status listener task is started");
    loop {
        checkin.good();

        // Drained, not used. A pubsub subscriber that never reads lags and then drops
        // messages for every other subscriber on the channel.
        //
        // Timed out so the row has a period. Status arrives about once a second while the
        // link is up, so in practice the timeout never fires -- but "the link is up" is
        // exactly the assumption a check-in must not make, and with a bare `await` this row
        // would go stale on a dead link rather than reporting that the *listener* is fine.
        let _ = with_timeout(
            variegated_checkin::HEARTBEAT,
            subscriber.next_message_pure(),
        )
        .await;
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
    use variegated_comms_firmware::ble;
    use variegated_comms_firmware::channels::{COMMS_STATUS_SIGNAL, NO_IPV4, SNTP_SYNC_SEQ, TIME_SYNCED, WIFI_CONNECTED, WIFI_IPV4, WIFI_RSSI_SIGNAL};
    use variegated_comms_firmware::config::USEC_IN_SEC;
    use variegated_controller_types::CommsStatus;
    use heapless::index_map::FnvIndexMap;
    use portable_atomic::Ordering;

    log_info!("CommsStatus signaller task started");
    let checkin = MONITOR.claim(CheckinId::CommsStatusSignaller);
    loop {
        // The one slot whose staleness the *machine* already reacts to: the application
        // processor ages this report against `COMMS_STATUS_STALE_AFTER`, so a stale row
        // here has a visible consequence at the other end of the link.
        checkin.good();

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
        // Filled from the slot table, which reports an entry for every *assigned*
        // peripheral whether or not it is currently connected -- see
        // `ble::status::fill_connection_status` for why that distinction is load bearing.
        let mut peripheral_connection_status = FnvIndexMap::new();
        ble::status::fill_connection_status(&mut peripheral_connection_status);

        let comms_status = CommsStatus {
            wifi_connected,
            timestamp,
            wifi_rssi,
            // Mirrored from `improv::improv_task` through an atomic, exactly as
            // `wifi_connected` above is mirrored from the task that owns the radio.
            // `Stopped` whenever no provisioning window is open, which is nearly always.
            improv: variegated_comms_firmware::channels::improv_state(),
            peripheral_connection_status,
            // What tells the application processor that `timestamp` is worth acting on.
            // It anchors its clock when this changes and ignores the timestamp the rest of
            // the time, so the RTC's own drift between syncs stays on this processor.
            sntp_sync_seq: SNTP_SYNC_SEQ.load(Ordering::Relaxed),
            // Mirrored from `wifi::apply_configuration`, and empty whenever the link is
            // down -- see `channels::wifi_ssid`.
            wifi_ssid: variegated_comms_firmware::channels::wifi_ssid(),
            // The same lease the `WIFI_IPV4` store above just refreshed, so the two cannot
            // disagree within a cycle. `None` rather than `0.0.0.0`, which is what
            // `NO_IPV4` means and never a real interface address.
            wifi_ip: match WIFI_IPV4.load(Ordering::Relaxed) {
                NO_IPV4 => None,
                bits => Some(bits.to_be_bytes()),
            },
        };
        COMMS_STATUS_SIGNAL.signal(comms_status);
        Timer::after(Duration::from_secs(1)).await;
    }
}

/// The structured debug stream's wire transport, on UART0 at GPIO16/17.
///
/// This task owns UART0 outright while the executor is running. The one other writer is
/// the panic handler above, which takes the peripheral by force after the executor has
/// stopped -- see [`debug::panic_console`] for why it goes through the registers rather
/// than through this task's `UartTx`.
///
/// `esp-println` is `no-op` and has no output target at all (see its entry in
/// Cargo.toml), so nothing else can put bytes on this wire and interleave them into the
/// COBS stream.
#[embassy_executor::task]
async fn debug_uart_task(
    debug_rx: esp_hal::uart::UartRx<'static, esp_hal::Async>,
    debug_tx: esp_hal::uart::UartTx<'static, esp_hal::Async>,
    sink: debug::CommandSink,
    subscriber: Option<debug::BusSubscriber>,
) {
    watch(
        MONITOR.claim(CheckinId::DebugUart),
        debug::uart::run(debug_rx, debug_tx, sink, subscriber),
    )
    .await;
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
    watch(MONITOR.claim(CheckinId::DebugTcp), debug::tcp::run(stack, subscriber, sink)).await;
}

#[embassy_executor::task]
async fn application_processor_task(
    rx: esp_hal::uart::UartRx<'static, esp_hal::Async>,
    tx: esp_hal::uart::UartTx<'static, esp_hal::Async>,
    status_channel: &'static ApplicationStatusChannel,
    config_channel: &'static ApplicationConfigurationChannel,
    routine_channel: &'static ApplicationRoutineChannel,
    shot_log_event_channel: &'static ShotLogEventChannel,
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
    // The scanner owns the queue and this task drains it. Passed as a `&'static` scanner
    // rather than a receiver so the borrow is obviously tied to the `mk_static!` object.
    scanner: &'static ScanPrinter,
    input_command_receiver: embassy_sync::channel::Receiver<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        (
            variegated_controller_types::PeripheralId,
            variegated_controller_types::InputCommand,
        ),
        { variegated_comms_firmware::channels::INPUT_COMMAND_CAPACITY },
    >,
    bond_report_receiver: embassy_sync::channel::Receiver<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        variegated_controller_types::bluetooth::BluetoothBond,
        { variegated_comms_firmware::channels::BOND_REPORT_CAPACITY },
    >,
) {
    let status_publisher = status_channel.publisher().unwrap();
    let config_publisher = config_channel.publisher().unwrap();
    let routine_publisher = routine_channel.publisher().unwrap();
    let shot_log_event_publisher = shot_log_event_channel.publisher().unwrap();
    let command_receiver = command_channel.receiver();
    let sensor_reading_receiver = sensor_reading_channel.receiver();

    // One row for a `join` of a reader and a sender, each nesting several `select4`s. The
    // period is loose because the sender's slowest arm is a 15 s routine request: this
    // reports "the link task is being woken", and nothing finer until the arms get slots
    // of their own.
    watch(
        MONITOR.claim(CheckinId::ApplicationProcessor),
        application_processor::start(
            rx,
            tx,
            status_publisher,
            config_publisher,
            routine_publisher,
            shot_log_event_publisher,
            command_receiver,
            sensor_reading_receiver,
            debug_command_receiver,
            scanner.results(),
            input_command_receiver,
            bond_report_receiver,
        ),
    )
    .await;
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // **First statement, and it has to stay first.** This writes every dead byte of the
    // main task's stack, and "dead" is only true here -- before the executor, before any
    // spawn, before anything below this frame has run. See `stack`'s module docs for why
    // the measurement is worth the hazard: `.stack` is the RWDATA remainder, it has
    // overflowed at 90,144 bytes and not at 97,616, and nobody has ever measured what it
    // actually needs.
    variegated_comms_firmware::stack::paint();

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

    // Immediately after `Boot`, because it qualifies it: `Boot` says the firmware
    // restarted, this says whether that was a power cycle, a flash, or the watchdog
    // below deciding the executor had stopped.
    watchdog::log_reset_reason();

    // Initialize RTC for time synchronization
    let rtc = Rtc::new(peripherals.LPWR);

    // The `#[ram(reclaimed)]` heap comes out of memory the ROM bootloader was using and
    // costs the stack nothing, so it is free real estate -- and **64 kB is all of it**.
    // Asking for 96 kB fails to link with
    // `section '.dram2_uninit' will not fit in region 'dram2_seg': overflowed by 32768
    // bytes`, which is the exact 32 kB of the increase. Do not spend time trying to grow
    // this; any further heap has to come out of `.stack`.
    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
    // The second heap region, and the only one of the two with a choice behind it.
    //
    // Everything past the 64 kB above comes out of `.stack` -- which is not a size anyone
    // picks either, but whatever RWDATA is left after `.data` and `.bss`. So this line and
    // the stack are in direct competition and there is no third source: every byte of
    // static costs a byte of `.stack`, one for one, including this allocator's own.
    //
    // # Where this leaves the machine, measured
    //
    // Peaks from the 1 Hz snapshot's `Heap high-water` and `Stack high-water` lines;
    // section sizes from `scripts/memory-report.sh`:
    //
    //     heap     82356 peak  of 122880   (64 kB reclaimed + 56 kB here)   ~40 kB spare
    //     .stack   94028 peak  of  96480                                   ~2.4 kB spare
    //
    // **The binding constraint is the stack, and it binds by about one deep call.** The
    // heap peak is a full Improv provisioning cycle with a BLE client connected, which is
    // this firmware's peak-memory event, so it is unlikely to be beaten by much.
    //
    // Read both lines before moving this number in either direction. They exist because
    // this was sized by comment for a long time, and the comment still said the peak was
    // 28 kB while the machine was using 108 -- that figure predated the frontend's
    // WebSocket client, and `WsMessage` wraps `Configuration`, `Status`,
    // `MachineDefinition` and `RoutineStorage`, so every status push at up to 5 Hz
    // allocates through `to_allocvec` on top of the ESPHome server and the pubsub clones.
    //
    // Note what the stack figure *is*: `esp_rtos::main` runs the executor on the main
    // thread, so **every embassy task is polled on this one stack**, and 94028 is the
    // deepest single poll rather than main's own depth.
    // `postcard::from_bytes_cobs::<..Configuration>` is the documented suspect. Shrinking
    // that frame buys room on both sides at once and is worth more than moving this line.
    //
    // # What each edge did when it was crossed, because both cost a day
    //
    // **Heap**, twice, both genuine exhaustion rather than fragmentation -- the failing
    // requests were 128 and 800 bytes, which fit in almost any gap:
    //
    //   * `heap_used: 108808, heap_free: 5880` of 114688, at ~12 minutes with two BLE
    //     peripherals connected.
    //   * 121552 of 122880 during provisioning. **Not** a driver leak, though it was
    //     assumed to be one: `wifi::try_candidate` brackets the heap either side of every
    //     step of a re-association and it costs nothing. It was churn -- the caller tore
    //     down the link it had just made and the station cycled -- and fixing that took
    //     post-provisioning usage from ~115 kB to ~74 kB, which is most of the margin in
    //     the table above.
    //
    // **Stack**, which does not announce itself. It arrives as a load access fault at a
    // different address per build, because the overrun lands on whatever the linker put at
    // the top of `.bss`: `chip_v7_set_chan` twice, and once
    // `embassy_time_queue_utils::Queue::next_expiration`, that one a corrupted `next`
    // pointer in embassy's intrusive timer list. `esp-rtos` caught it directly only once,
    // as `Stack pointer: 40857b20, Task stack range: 40857d78 ..=`.
    //
    // The observations do not form a bound and must not be read as one -- 85608 failed
    // consistently while 87256 survived, and 90144 failed while 97616 survived:
    //
    //     71704 failed   85608 failed   87256 survived   90144 failed   97616 survived
    //
    // Two things came out of that. Every figure recorded as "survived" is a lower bound
    // that had quietly been crossed: the margin was already gone before the Improv service
    // existed, and a commit adding ~500 bytes of `.bss` overflowed by 600. And the change
    // that bought the most was not this line -- the shot-log download path held a 1 kB
    // chunk buffer in a `Signal` and another across two awaits in the HTTP handler, and the
    // task pool multiplied both; moving those bytes to the heap returned 10240 to `.stack`.
    //
    // There is no contiguity floor to respect here any more: the ESPHome entity table,
    // which needed one unbroken 12000-byte block, is built in `.bss`
    // (`esphome/entity_builder.rs`). What remains is a plain capacity question.
    //
    // If the heap has to grow again, the bytes are likelier to be found in statics than
    // taken from `.stack`. Every task pool is size 1, so the largest entries in
    // `scripts/memory-report.sh` are single futures with big inline buffers:
    // `application_processor_task` 18896, `http_server_task` 17696, `esphome_server_task`
    // 17104, `debug_tcp_task` 9040, `websocket_server_task` 7008.
    esp_alloc::heap_allocator!(size: 64 * 1024);

    // Initialize application processor channels
    let status_channel = STATUS_CHANNEL.init(embassy_sync::pubsub::PubSubChannel::new());
    let config_channel = CONFIGURATION_CHANNEL.init(embassy_sync::pubsub::PubSubChannel::new());
    let routine_channel = ROUTINE_CHANNEL.init(embassy_sync::pubsub::PubSubChannel::new());
    let shot_log_event_channel =
        SHOT_LOG_EVENT_CHANNEL.init(embassy_sync::pubsub::PubSubChannel::new());
    let command_channel = MACHINE_COMMAND_CHANNEL.init(embassy_sync::channel::Channel::new());
    let sensor_reading_channel = SENSOR_READING_CHANNEL.init(embassy_sync::channel::Channel::new());
    let input_command_channel = INPUT_COMMAND_CHANNEL.init(embassy_sync::channel::Channel::new());
    let bond_report_channel = BOND_REPORT_CHANNEL.init(embassy_sync::channel::Channel::new());

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
    // UART0 on GPIO16/17, the chip's default console pins.
    //
    // This carries the structured debug stream and injected commands, and it replaced
    // USB-Serial-JTAG for one reason: USB-Serial-JTAG is enumerated by the host, so it is
    // precisely the wire that vanishes when the device you are trying to observe crashes
    // or sits in a reset loop -- which is when the stream is worth having. A UART has no
    // enumeration and no attach state; bytes leave at the line rate regardless.
    //
    // USB-Serial-JTAG is left alone and unclaimed. It still works for flashing, and
    // nothing writes to it: `esp-println` is `no-op` (see Cargo.toml).
    //
    // Only TX and RX are wired, so `debug_uart_config` leaves hardware flow control off --
    // enabling CTS against a pin nobody drives would stall the transmitter forever.
    //
    // `split()` on a UART returns (rx, tx), the same order `UsbSerialJtag` used.
    let debug_uart = esp_hal::uart::Uart::new(peripherals.UART0, debug_uart_config())
        .expect("Failed to create debug UART")
        .with_tx(peripherals.GPIO16)
        .with_rx(peripherals.GPIO17)
        .into_async();
    let (debug_rx, debug_tx) = debug_uart.split();
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

    spawn_or_report!(spawner, "debug_uart", debug_uart_task(debug_rx, debug_tx, debug_command_channel.sender(), debug_subscriber));
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
    spawn_or_report!(spawner, "checkin", checkin_task());
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

    // Created here, well before the radio exists, because two tasks need it and they are
    // spawned at opposite ends of this function: the application processor task drains
    // its result queue, and the BLE runner delivers advertising reports to it.
    // `ScanPrinter::new` is `const` and touches no hardware, so there is nothing to order
    // it against.
    let printer = mk_static!(ScanPrinter, ScanPrinter::new());

    // Spawn application processor tasks.
    //
    // embassy-executor 0.10 moved the fallibility from `Spawner::spawn` (which
    // now returns `()`) onto the `#[task]` function itself. A failed spawn used to
    // be discarded silently; `spawn_or_report!` turns it into a `SpawnFailed` event
    // instead. See the macro's doc comment for why an event and not `unwrap`.
    spawn_or_report!(spawner, "application_processor", application_processor_task(rx, tx, status_channel, config_channel, routine_channel, shot_log_event_channel, command_channel, sensor_reading_channel, debug_command_channel.receiver(), printer, input_command_channel.receiver(), bond_report_channel.receiver()));
    spawn_or_report!(spawner, "status_listener", status_listener_task(status_channel));
    log_info!("Application processor tasks spawned");

    // Initialize esp-rtos
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    // Armed here, immediately after the executor exists, and not one line earlier:
    // everything above is straight-line init -- heap, USB-Serial-JTAG, UART1, eFuse --
    // that cannot block on anything a watchdog could rescue it from, and arming before
    // the executor runs would mean a window with no feeder in it. Everything *below* is
    // `await`-based (BLE bring-up, the five-second settle, Wi-Fi association), so the
    // feeder runs throughout it.
    //
    // `TimerGroup::new` rather than the bare `Wdt::<TIMG1>::new()` constructor: it
    // enables and resets TIMG1's peripheral clock, and it consumes `peripherals.TIMG1`
    // so the claim is visible to anything that later wants the group's timers.
    let mut wdt = TimerGroup::new(peripherals.TIMG1).wdt;
    wdt.set_timeout(MwdtStage::Stage0, watchdog::TIMEOUT);
    wdt.enable();
    spawn_or_report!(spawner, "watchdog", watchdog::watchdog_task(wdt));
    log_info!(
        "Watchdog armed: {} ms timeout, fed every {} ms",
        watchdog::TIMEOUT.as_millis(),
        watchdog::FEED_INTERVAL.as_millis()
    );

    log_info!("Initializing radio");

    // esp-radio 0.18 removed `esp_radio::init()` and the `Controller` handle;
    // the radio is brought up implicitly by the BLE/WiFi constructors.

    // Initialize BLE (before WiFi for stability)
    //
    // **`max_connections` is not a default that can be left alone.** esp-radio's C6 BLE
    // `Config::default()` sets it to 2, and it is the *controller's* ACL link table --
    // the one thing in the BLE stack that `HostResources` below cannot influence. With
    // the default, the third `LE Create Connection` is rejected by the controller with a
    // command status error, so it fails in about two milliseconds rather than timing out,
    // and trouble then logs `error cancelling connection` because it sends
    // `Create_Connection_Cancel` for a connection that was never begun. Two peripherals
    // worked, three did not, and nothing in this firmware's own sizing said why.
    //
    // 6, to match `CONNS` below exactly: four associable peripherals, one slot of margin
    // for a reconnect overlapping a link still tearing down, and one for the Improv
    // peripheral connection. Those are the same six for the same reasons -- see the
    // `HostResources` comment -- and they are deliberately one number, because a
    // controller table smaller than the host's is invisible until the link count reaches
    // it, which is exactly how this was found.
    //
    // The cost is heap, not `.stack`: the controller's per-connection state is allocated
    // by the blob through `esp_alloc::HEAP`. That heap is genuinely tight here -- see the
    // Wi-Fi buffer note below, where the default dynamic buffer caps once exhausted it
    // outright -- so if this number grows again, read `Heap high-water` from the 1 Hz
    // snapshot rather than assuming the room is there. The ACL buffer pool
    // (`acl_buf_count` 24 x `acl_buf_size` 255) is shared across links and is left alone;
    // it is not multiplied by this.
    let connector = BleConnector::new(
        peripherals.BT,
        esp_radio::ble::Config::default().with_max_connections(6),
    )
    .unwrap();
    let controller: ExternalController<_, 20> = ExternalController::new(connector);

    // Create BLE host resources.
    //
    // `<CONNS, CHANNELS, ADV_SETS>`, sized to what this firmware is rather than to
    // trouble-host's example defaults it was carrying (`4, 12, 16`). This is not free
    // real estate: `HostResources` is a `mk_static!` static, `.stack` is the SRAM
    // remainder, so every unused slot here is stack the deepest postcard recursion
    // does not get. The three were worth 2752 bytes together.
    //
    // - `CONNS = 6`. Five for the central side, plus one for the Improv peripheral
    //   connection -- a `ConnectionStorage` slot like any other, held for as long as a
    //   provisioning window is open.
    //
    //   The central five: this is a central, and the set of peripherals it connects to is
    //   no longer a build-time fact: associations arrive from the application processor at
    //   runtime, up to `MAX_BLUETOOTH_PERIPHERALS` of them. Four, plus one of margin for
    //   a reconnect that overlaps a not-yet-reaped stale connection.
    //
    //   That margin matters more than it used to. Releasing a device now calls
    //   `Connection::disconnect`, and the controller holds the ACL link until it has
    //   serviced that -- so a slot reassigned to a new address while the old link is
    //   still tearing down is a *designed-in* state, not the occasional artefact of a
    //   reaping pass it was when the set was fixed.
    //
    //   Measured, not estimated: 3 -> 5 moved `.bss` 217560 -> 218712 and `.stack`
    //   129464 -> 128312, i.e. 1152 bytes, ~576 per connection slot, taken out of the
    //   stack exactly as the paragraph above says. Re-measured at the Improv change,
    //   5 -> 6 costs 512 bytes, so the per-slot figure has held.
    //
    //   The Improv service as a whole cost rather more than its slot: `.bss` 249016 ->
    //   253040 and `.stack` 94232 -> 90144, i.e. 4024 bytes, of which only 512 is this
    //   line. The rest is `ImprovServer` -- a 20-entry `AttributeTable` plus its CCCD
    //   table -- which lives in the improv task's storage, and the two characteristic
    //   value buffers, which are `StaticCell`s the `#[gatt_service]` macro emits. Worth
    //   knowing before adding a second service: the table, not the connection, is what
    //   a GATT server costs here.
    // - `CHANNELS = 2`. `ChannelStorage` is *dynamic L2CAP connection-oriented*
    //   channels only -- GATT does not use it, it rides the fixed ATT CID through
    //   `ConnectionStorage::gatt_client`. Nothing here opens a CoC channel: both
    //   drivers are plain GATT clients. This could be 0; 2 is left as headroom
    //   because each slot embeds a `PacketChannel<_, L2CAP_RX_QUEUE_SIZE>` and
    //   discovering the need for one at runtime is worse than paying for two.
    // - `ADV_SETS = 1`. This firmware advertises exactly one set, and only while a Wi-Fi
    //   provisioning window is open -- see `improv::improv_task`. One legacy advertisement
    //   is all Improv needs, and `ConnectableScannableUndirected` carries both its
    //   payloads within that one set. (This bullet read "never advertises -- there is no
    //   `Peripheral`" until the Improv service landed. The count was already right; the
    //   reason was not.)
    //
    // If any of those three claims stops being true, this line is the thing that
    // fails, and it fails at connect/advertise time rather than at compile time.
    // trouble 0.7 takes the controller and pool as *type* parameters here, and adds a fourth
    // const: `BONDS`, the number of pairing keys the security manager keeps in `.bss`.
    //
    // **4, not the default 10.** A bond is only useful for a device that can be associated,
    // and there are `MAX_BLUETOOTH_PERIPHERALS` of those. Six unusable slots is not a large
    // number of bytes, but this firmware has 6,688 of uncommitted SRAM and no reason to spend
    // any of them on bonds no association can reach.
    let ble_resources = mk_static!(
        HostResources<
            ExternalController<BleConnector<'static>, 20>,
            DefaultPacketPool,
            6,
            2,
            1,
            { variegated_controller_types::bluetooth::MAX_BLUETOOTH_PERIPHERALS },
        >,
        HostResources::new()
    );

    // **No address is set here, deliberately.** The controller's own is used instead.
    //
    // This used to draw six random bytes and force the top two to `11` to make a valid
    // static random address. That was correct while nothing bonded: an address only had to
    // be valid and unique, and "stable for the power cycle" was enough.
    //
    // Bonding changed what the address *is*. A bond is a relationship between two
    // identities, and the peer stores ours as half of it. Regenerating on every boot meant
    // coming back as a different device, so every bond the dial held referred to a central
    // that no longer existed -- it paired happily and then never reconnected, which is
    // exactly the symptom that led here.
    //
    // With no address set, trouble reads the controller's public address with `ReadBdAddr`
    // and hands it to the security manager as our identity. That address comes from the
    // chip's efuse: unique per board and stable for its life, which is what a bond needs.
    // `ble_devices_task` publishes it to `BT_ADDRESS` once the runner has read it, since it
    // is not knowable here.

    // Create BLE stack
    //
    // **No RNG seeding here, unlike under trouble 0.6.** With `security` on, 0.6 panicked in
    // `build()` unless it had been handed a `CryptoRng` -- and that panic happens before
    // TIMG1 is first fed, so it presented as a silent watchdog reboot rather than as a
    // message. 0.7 seeds the security manager itself, out of the controller's own `LE Rand`
    // command, during runner initialisation. That is better than what this code did: the
    // entropy comes from the radio's hardware RNG rather than from a `Trng` handle main had
    // to acquire and hold at exactly the right moment.
    let stack = trouble_host::new(controller, ble_resources);
    let stack = mk_static!(
        Stack<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
        stack.build()
    );

    // Central, peripheral and runner are accessors on the built `Stack` in 0.7, where 0.6
    // destructured a `Host` returned from `build()`.
    let central = stack.central();
    let peripheral = stack.peripheral();
    let runner = stack.runner();

    // Create connection manager
    let connection_manager = mk_static!(
        BleConnectionManager<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
        BleConnectionManager::new(central)
    );

    // Get sensor reading sender for BLE devices
    let sensor_reading_sender = sensor_reading_channel.sender();

    // Spawn BLE tasks
    spawn_or_report!(spawner, "ble_runner", ble_runner_task(runner, printer));
    spawn_or_report!(spawner, "ble_devices", ble_devices_task(connection_manager, printer, stack));
    // One worker per slot, spawned unconditionally and idle until the application
    // processor says what to connect to. There is no peripheral list at this point --
    // this firmware stores none -- so spawning per peripheral is not an option even in
    // principle; a task cannot be created later from a context that has no `Spawner`,
    // and cannot be destroyed at all.
    for slot in 0..MAX_BLUETOOTH_PERIPHERALS {
        spawn_or_report!(spawner, "ble_slot", ble_slot_task(slot, connection_manager, stack, sensor_reading_sender, input_command_channel.sender(), bond_report_channel.sender()));
    }
    // Idle until the application processor opens a window, which it will not do until
    // someone has held a button on the machine. It reports what it learns on
    // `IMPROV_REPORT_CHANNEL`, which is a static, so it needs nothing here but the radio.
    spawn_or_report!(spawner, "improv", improv::improv_task(peripheral));
    log_info!("BLE tasks spawned");

    //Timer::after_secs(5).await;

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
    // `static_rx_buf_num` 10 -> 6, which is ~6.4 kB of heap back.
    //
    // These are the only Wi-Fi buffers that are *permanent*: esp-radio's own docs put
    // them at approximately 1.6 kB each, allocated inside `esp_wifi_init` and not freed
    // until deinit -- and they come out of the same `esp_alloc::HEAP` as everything else,
    // because the blob's `malloc` is esp-alloc's. On a firmware that ran out of heap
    // with 105 kB in use and no BLE peripheral connected, sixteen kilobytes of
    // permanently-held receive buffers is the largest single thing worth questioning.
    //
    // Six is the documented floor while AMPDU RX is on: the field's docs recommend
    // keeping it at or above `rx_ba_win`, which defaults to 6, and esp-radio's own
    // validation rejects `rx_ba_win >= 2 * static_rx_buf_num`. Going lower means turning
    // `ampdu_rx_enable` off as well, which trades throughput this machine does not need
    // -- it serves a 50 kB gzipped page and ESPHome telemetry -- and is the next lever
    // if the heap is still tight.
    //
    // `dynamic_rx_buf_num`/`dynamic_tx_buf_num` default to 32 each and **were** left alone,
    // on the grounds that they are caps on transient buffers rather than steady cost. That
    // was wrong, and the log says so.
    //
    // Measured across an Improv provisioning reconnect: heap free went 48076 -> 5032 and
    // **stayed there**, ~43 kB acquired and never returned. The driver's dynamic buffers are
    // a high-water pool -- ESP-IDF grows it on demand up to these caps and does not shrink
    // it -- so "transient" describes the frames, not the memory. Every buffer is up to
    // ~1.6 kB and comes out of `esp_alloc::HEAP`, because the blob's `malloc` is ours, so
    // the default pair can reach far more than this firmware has to give. The machine then
    // died on `memory allocation of 800 bytes failed`.
    //
    // 24/16. The RX cap must stay at or above `rx_queue_size`, which is the invariant the
    // previous note was protecting: the queue holds `PacketBuffer` handles and each pins a
    // dynamic buffer, so a cap below the queue depth is a queue that can never fill. Hence
    // `rx_queue_size` 32 -> 24 alongside it, still far above the default 5 that latched the
    // stack (see the paragraph above), and TX 16 to match `tx_queue_size` exactly.
    //
    // If throughput regresses, raise these *and* the matching queue -- never one alone.
    let radio_config = esp_radio::wifi::ControllerConfig::default()
        .with_rx_queue_size(24)
        .with_tx_queue_size(16)
        .with_static_rx_buf_num(6)
        .with_dynamic_rx_buf_num(24)
        .with_dynamic_tx_buf_num(16);
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
    let http_status_subscriber = status_channel
        .subscriber()
        .expect("APPLICATION_STATUS_RECEIVERS must count this subscriber");
    let http_config_subscriber = config_channel
        .subscriber()
        .expect("APPLICATION_CONFIGURATION_RECEIVERS must count this subscriber");

    // Spawn HTTP server and cache update tasks
    spawn_or_report!(spawner, "http_server", http_server_task(tcp_stack, command_sender));
    spawn_or_report!(spawner, "cache_update", cache_update_task(http_status_subscriber, http_config_subscriber));
    log_info!("HTTP server and cache update tasks spawned");

    // Create subscribers for ESPHome server
    let esphome_status_subscriber = status_channel
        .subscriber()
        .expect("APPLICATION_STATUS_RECEIVERS must count this subscriber");
    let esphome_config_subscriber = config_channel
        .subscriber()
        .expect("APPLICATION_CONFIGURATION_RECEIVERS must count this subscriber");
    let esphome_command_config_subscriber = config_channel
        .subscriber()
        .expect("APPLICATION_CONFIGURATION_RECEIVERS must count this subscriber");

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
    let ws_status_subscriber = status_channel
        .subscriber()
        .expect("APPLICATION_STATUS_RECEIVERS must count this subscriber");
    let ws_config_subscriber = config_channel
        .subscriber()
        .expect("APPLICATION_CONFIGURATION_RECEIVERS must count this subscriber");
    let ws_routine_subscriber = routine_channel
        .subscriber()
        .expect("APPLICATION_ROUTINE_RECEIVERS must count this subscriber");
    let ws_shot_log_subscriber = shot_log_event_channel
        .subscriber()
        .expect("SHOT_LOG_EVENT_RECEIVERS must count this subscriber");

    // Spawn WebSocket server task
    spawn_or_report!(spawner, "websocket_server", websocket_server_task(
        stack_static,
        ws_status_subscriber,
        ws_config_subscriber,
        ws_routine_subscriber,
        ws_shot_log_subscriber,
        command_channel,
    ));
    log_info!("WebSocket server task spawned on port 8080");

    // Shot-log upload. The second subscriber on the shot-log event channel -- see
    // `SHOT_LOG_EVENT_RECEIVERS`, which is the compile-time bound both of these draw from.
    //
    // `SHA` and `RSA` used to go to MbedTLS's accelerator hooks and are no longer claimed by
    // anything: the upload path is `http+noise://` now, and ChaCha20-Poly1305 and X25519 have
    // no accelerator on this chip to route to. Both peripherals are now free.
    //
    // The task takes the `Trng` itself, for the Noise ephemeral.
    let upload_shot_log_subscriber = shot_log_event_channel
        .subscriber()
        .expect("SHOT_LOG_EVENT_RECEIVERS must count this subscriber");
    spawn_or_report!(spawner, "shot_upload", variegated_comms_firmware::upload::shot_upload_task(
        net_stack,
        upload_shot_log_subscriber,
    ));
    log_info!("Shot upload task spawned");

    // The Plantlet uplink, beside the uploader rather than folded into it. They share the
    // provisioned keys and the endpoint setting and nothing else: the uploader wakes when a
    // shot finishes and is otherwise absent, while this one holds a socket open for months.
    // One task doing both would have to keep a 90-second upload from stalling the socket's
    // keepalive, which is a coordination problem neither has on its own.
    // No status subscriber here. The uplink sends the status from `STATUS_CACHE` instead --
    // two readers of one subscriber, one of which discarded what it read, meant almost no
    // status ever went up. See `uplink::send_status`.
    //
    // The routine subscriber is the second on that channel, after the websocket server's. The
    // application processor publishes only when the summary list actually changes, so this
    // costs nothing at steady state -- and it is what lets a routine saved at the machine
    // reach Plantlet without anybody pressing refresh.
    let uplink_routine_subscriber = routine_channel
        .subscriber()
        .expect("APPLICATION_ROUTINE_RECEIVERS must count this subscriber");
    // The fifth configuration subscriber. `APPLICATION_CONFIGURATION_RECEIVERS` was bumped to
    // 5 with it -- an over-subscribed pubsub panics here, in `main`, before the watchdog is
    // ever fed, which presents as a silent reset with no panic text at all.
    let uplink_config_subscriber = config_channel
        .subscriber()
        .expect("APPLICATION_CONFIGURATION_RECEIVERS must count this subscriber");
    spawn_or_report!(spawner, "uplink", variegated_comms_firmware::uplink::uplink_task(
        net_stack,
        uplink_routine_subscriber,
        uplink_config_subscriber,
        command_channel.sender(),
    ));
    log_info!("Uplink task spawned");

    // Main loop - periodic HTTP client requests
    loop {
        Timer::after(Duration::from_millis(5000)).await;
    }
}
