//! WiFi and BLE COEXistence example
//!
//! - set SSID and PASSWORD env variable
//! - gets an ip address via DHCP
//! - performs an HTTP get request to some "random" server
//! - does BLE advertising (you cannot connect to it - it's just not implemented in the example)
//!
//! Note: On ESP32-C2 and ESP32-C3 you need a wifi-heap size of 70000, on
//! ESP32-C6 you need 80000 and a tx_queue_size of 10

#![no_std]
#![no_main]

use core::cell::RefCell;
use core::net::{IpAddr, Ipv4Addr, SocketAddr};
use core::fmt::{Debug, Display};

use bt_hci::controller::ExternalController;
use trouble_host::prelude::*;
use variegated_trouble_connection_manager::BleConnectionManager;
use variegated_belka_portal_trouble_driver::BelkaPortalDriver;
use variegated_scale_trouble_driver::acaia_old::{AcaiaOldDriver, Error as AcaiaError, ScaleEvent};

use heapless::Deque;
use embassy_futures::join::join;
use embassy_futures::select::{select, Either};

use embassy_executor::Spawner;
use embassy_net::{Runner as NetRunner, StackResources, dns::DnsQueryType, udp::{PacketMetadata, UdpSocket}};
use embassy_time::{Duration, Timer};
use sntpc::{NtpContext, NtpTimestampGenerator, get_time};
use edge_http::io::client::Connection;
use edge_http::io::server::{Connection as ServerConnection, DefaultServer, Handler};
use edge_http::io::Error;
use edge_http::Method;
use edge_nal::{TcpBind};
use edge_nal_embassy::{Tcp, TcpBuffers};
use embedded_io_async::{Read, Write};
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
use esp_radio::{
    Controller,
    ble::controller::BleConnector,
    wifi::{ClientConfig, ModeConfig, WifiController, WifiDevice, WifiEvent, WifiStaState},
};

use defmt::{error, info};
use trouble_host::config;

esp_bootloader_esp_idf::esp_app_desc!();

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

// SNTP configuration
const NTP_SERVER: &str = "pool.ntp.org";
const USEC_IN_SEC: u64 = 1_000_000;

// Timestamp generator for SNTP using RTC
#[derive(Clone, Copy)]
struct Timestamp<'a> {
    rtc: &'a Rtc<'a>,
    current_time_us: u64,
}

impl NtpTimestampGenerator for Timestamp<'_> {
    fn init(&mut self) {
        self.current_time_us = self.rtc.current_time_us();
    }

    fn timestamp_sec(&self) -> u64 {
        self.current_time_us / USEC_IN_SEC
    }

    fn timestamp_subsec_micros(&self) -> u32 {
        (self.current_time_us % USEC_IN_SEC) as u32
    }
}

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[unsafe(no_mangle)]
pub extern "Rust" fn _esp_println_timestamp() -> u64 {
    esp_hal::time::Instant::now()
        .duration_since_epoch()
        .as_millis()
}

// HTTP Server Handler
struct HttpHandler;

impl Handler for HttpHandler {
    type Error<E> = Error<E>
    where
        E: Debug;

    async fn handle<T, const N: usize>(
        &self,
        _task_id: impl Display + Copy,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Self::Error<T::Error>>
    where
        T: Read + Write,
    {
        let headers = conn.headers()?;
        let is_root = headers.method == Method::Get && headers.path == "/";

        if is_root {
            // Respond with "Hello World"
            conn.initiate_response(200, Some("OK"), &[("Content-Type", "text/plain")])
                .await?;
            conn.write_all(b"Hello World").await?;
            info!("Served Hello World to client");
        } else {
            // Return 404 for other paths
            conn.initiate_response(404, Some("Not Found"), &[]).await?;
            info!("Returned 404 for non-root path");
        }

        Ok(())
    }
}

#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    info!("start connection task");
    println!("Device capabilities: {:?}", controller.capabilities());
    loop {
        match esp_radio::wifi::sta_state() {
            WifiStaState::Connected => {
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                Timer::after(Duration::from_millis(5000)).await
            }
            _ => {}
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = ModeConfig::Client(
                ClientConfig::default()
                    .with_ssid(SSID.into())
                    .with_password(PASSWORD.into()),
            );
            controller.set_config(&client_config).unwrap();
            info!("Starting WiFi controller...");
            controller.start_async().await.unwrap();
        }

        info!("Connecting to WiFi...");
        match controller.connect_async().await {
            Ok(_) => info!("WiFi connected!"),
            Err(_e) => {
                error!("Failed to connect to WiFi");
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: NetRunner<'static, WifiDevice<'static>>) {
    runner.run().await
}

#[embassy_executor::task]
async fn http_server_task(tcp_stack: &'static Tcp<'static, 8, 1024, 1024>) {
    info!("Starting HTTP server on port 80...");

    let mut server = DefaultServer::new();

    let bind_addr = SocketAddr::from(([0, 0, 0, 0], 80));

    match tcp_stack.bind(bind_addr).await {
        Ok(acceptor) => {
            info!("HTTP server bound to port 80");
            if let Err(e) = server.run(None, acceptor, HttpHandler).await {
                error!("HTTP server error: {:?}", e);
            }
        }
        Err(e) => {
            error!("Failed to bind HTTP server to port 80: {:?}", e);
        }
    }
}

// EventHandler for BLE scanning - prints discovered devices
struct ScanPrinter {
    seen: RefCell<Deque<BdAddr, 128>>,
}

impl EventHandler for ScanPrinter {
    fn on_adv_reports(&self, mut it: LeAdvReportsIter<'_>) {
        let mut seen = self.seen.borrow_mut();
        while let Some(Ok(report)) = it.next() {
            info!("Adv report: {:?}", report);

            // Decode and print advertising data structures
            info!("  Decoded advertising data:");
            for structure in AdStructure::decode(report.data) {
                match structure {
                    Ok(ad) => info!("    {:?}", ad),
                    Err(_) => info!("    [Decode error]"),
                }
            }

            // Track unique devices
            if seen.iter().find(|b| b.raw() == report.addr.raw()).is_none() {
                info!("Discovered BLE device: {:?}, RSSI: {}", report.addr, report.rssi);
                if seen.is_full() {
                    seen.pop_front();
                }
                seen.push_back(report.addr).unwrap();
            }
        }
    }
}

#[embassy_executor::task]
async fn sntp_task(rtc: &'static Rtc<'static>, stack: embassy_net::Stack<'static>) {
    // Wait for network link
    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    info!("Waiting for IP address for SNTP...");
    loop {
        if stack.config_v4().is_some() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    // Resolve NTP server
    info!("Resolving NTP server: {}", NTP_SERVER);
    let ntp_addrs = match stack.dns_query(NTP_SERVER, DnsQueryType::A).await {
        Ok(addrs) if !addrs.is_empty() => addrs,
        Ok(_) => {
            error!("DNS resolution returned empty results");
            return;
        }
        Err(e) => {
            error!("Failed to resolve NTP server: {:?}", e);
            return;
        }
    };

    // Create UDP socket for NTP
    let mut rx_meta = [PacketMetadata::EMPTY; 16];
    let mut rx_buffer = [0; 4096];
    let mut tx_meta = [PacketMetadata::EMPTY; 16];
    let mut tx_buffer = [0; 4096];

    let mut socket = UdpSocket::new(
        stack,
        &mut rx_meta,
        &mut rx_buffer,
        &mut tx_meta,
        &mut tx_buffer,
    );

    socket.bind(123).unwrap();

    // Display initial RTC time
    info!("Initial RTC time: {} us", rtc.current_time_us());

    // Sync time periodically
    loop {
        let addr: IpAddr = ntp_addrs[0].into();
        let result = get_time(
            SocketAddr::from((addr, 123)),
            &socket,
            NtpContext::new(Timestamp {
                rtc,
                current_time_us: 0,
            }),
        )
        .await;

        match result {
            Ok(time) => {
                // Update RTC immediately
                rtc.set_current_time_us(
                    (time.sec() as u64 * USEC_IN_SEC)
                        + ((time.sec_fraction() as u64 * USEC_IN_SEC) >> 32),
                );

                // Log synchronized time
                info!(
                    "NTP sync successful | RTC time: {} us | Unix timestamp: {} s",
                    rtc.current_time_us(),
                    time.sec()
                );
            }
            Err(_e) => {
                error!("SNTP error occurred");
            }
        }

        // Sync every 300 seconds
        Timer::after(Duration::from_secs(300)).await;
    }
}

#[embassy_executor::task]
async fn ble_runner_task(
    mut runner: Runner<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
    printer: &'static ScanPrinter,
) {
    loop {
        // Run the BLE host runner with event handler
        // This processes HCI events and delivers scan reports to the printer
        let r = runner.run_with_handler(printer).await;
        if let Err(e) = r {
            error!("Failed to run BLE, retrying in 10 seconds");
            Timer::after_secs(10).await;
        }
    }
}

#[embassy_executor::task]
async fn ble_devices_task(
    manager: &'static BleConnectionManager<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
    stack: &'static Stack<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
    belka_address: BdAddr,
    acaia_address: BdAddr,
) {
    let handle = manager.handle();

    // Register both devices and enable auto-connection
    {
        // Register Belka Portal
        let device_handle = handle.register_device(belka_address);
        let driver = BelkaPortalDriver::new(device_handle, stack);
        driver.set_maintain_connection(true).await;

        // Register ACAIA scale
        let device_handle = handle.register_device(acaia_address);
        let driver = AcaiaOldDriver::new(device_handle, stack);
        driver.set_maintain_connection(true).await;
    }

    info!("BLE Devices: Configured Belka {:?} and ACAIA {:?}", belka_address, acaia_address);

    // Run connection manager and both device measurement loops concurrently
    join(
        manager.run(),
        join(
            // Belka Portal measurement loop
            async {
                Timer::after(Duration::from_millis(300)).await;
            loop {
                // Check if connected
                let is_connected = {
                    let device_handle = handle.register_device(belka_address);
                    let driver = BelkaPortalDriver::new(device_handle, stack);
                    driver.is_connected().await
                };

                if !is_connected {
                    info!("Belka Portal not connected, waiting...");
                    Timer::after(Duration::from_secs(1)).await;
                    continue;
                }

                info!("Belka Portal connected, creating GATT client...");

                // Create GATT client and subscribe
                let result = {
                    let device_handle = handle.register_device(belka_address);
                    let driver = BelkaPortalDriver::new(device_handle, stack);
                    driver.gatt_client().await
                };

                match result {
                    Ok((_conn, gatt)) => {
                        info!("GATT client created, running task...");
                        // Run GATT client task alongside operations, exit when either completes
                        let _ = select(gatt.task(), async {
                            info!("Let's first read measurements...");
                            let r = gatt.read_measurements().await;
                            info!("Measurements: {:?}", r);
                            info!("Then subscribe...");

                            match gatt.subscribe().await {
                                Ok(mut stream) => {
                                    info!("Successfully subscribed to Belka Portal measurements");
                                    loop {
                                        // Race between getting next measurement and checking connection status
                                        match select(
                                            stream.next(),
                                            async {
                                                Timer::after(Duration::from_secs(1)).await;
                                                let device_handle = handle.register_device(belka_address);
                                                let driver = BelkaPortalDriver::new(device_handle, stack);
                                                driver.is_connected().await
                                            }
                                        ).await {
                                            Either::First(result) => {
                                                match result {
                                                    Ok(measurement) => {
                                                        info!(
                                                            "Portal Measurement: EC={}, Temp={} °C, IntTemp?={} °C, Battery?={}",
                                                            measurement.ec,
                                                            measurement.temperature,
                                                            measurement.internal_temperature,
                                                            measurement.status
                                                        );
                                                    }
                                                    Err(e) => {
                                                        error!("Failed to read measurement: {:?}", e);
                                                        break;
                                                    }
                                                }
                                            }
                                            Either::Second(is_connected) => {
                                                if !is_connected {
                                                    info!("Connection lost during measurements, exiting");
                                                    break;
                                                }
                                            }
                                        }
                                    }
                                }
                                Err(e) => {
                                    error!("Failed to subscribe to measurements: {:?}", e);
                                }
                            }
                        }).await;
                        info!("GATT join completed, connection dropped");
                    }
                    Err(e) => {
                        error!("Failed to create GATT client: {:?}", e);
                    }
                }

                // Wait before retrying
                info!("Restarting Belka measurement loop...");
                Timer::after(Duration::from_secs(5)).await;
            }
            },
            // ACAIA scale measurement loop
            async {
                loop {
                    // Check if connected
                    let is_connected = {
                        let device_handle = handle.register_device(acaia_address);
                        let driver = AcaiaOldDriver::new(device_handle, stack);
                        driver.is_connected().await
                    };

                    if !is_connected {
                        info!("ACAIA scale not connected, waiting...");
                        Timer::after(Duration::from_secs(1)).await;
                        continue;
                    }

                    info!("ACAIA scale connected, creating GATT client...");

                    // Create GATT client
                    let result = {
                        let device_handle = handle.register_device(acaia_address);
                        let driver = AcaiaOldDriver::new(device_handle, stack);
                        driver.gatt_client().await
                    };

                    match result {
                        Ok((_conn, gatt)) => {
                            info!("ACAIA GATT client created");

                            // Run GATT client task alongside operations
                            let _ = select(gatt.task(), async {
                                // Initialize scale (subscribe + handshake in correct order)
                                info!("Initializing ACAIA scale...");
                                match gatt.initialize().await {
                                    Ok(mut stream) => {
                                        info!("ACAIA scale initialized successfully");

                                        // Send initial heartbeat to trigger data flow
                                        info!("Sending initial heartbeat");
                                        if let Err(e) = gatt.send_heartbeat().await {
                                            error!("Failed to send initial heartbeat: {:?}", e);
                                        }

                                        use embassy_time::Instant;
                                        let mut last_heartbeat = Instant::now();

                                        loop {
                                            // Race between: getting next event and periodic timer
                                            match select(
                                                stream.next(),
                                                Timer::after(Duration::from_secs(1))
                                            ).await {
                                                Either::First(result) => {
                                                    match result {
                                                        Ok(event) => {
                                                            match event {
                                                                ScaleEvent::Weight(w) => {
                                                                    info!("Scale Weight: {} g", w.weight);
                                                                }
                                                            }
                                                        }
                                                        Err(e) => {
                                                            match e {
                                                                // Fatal errors: break
                                                                _ => {
                                                                    error!("Failed to read ACAIA event: {:?}", e);
                                                                    break;
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                                Either::Second(_) => {
                                                    // Check connection
                                                    let device_handle = handle.register_device(acaia_address);
                                                    let driver = AcaiaOldDriver::new(device_handle, stack);
                                                    let is_connected = driver.is_connected().await;

                                                    if !is_connected {
                                                        info!("ACAIA connection lost during measurements, exiting");
                                                        break;
                                                    }
                                                }
                                            }

                                            // Send heartbeat if 2 seconds have passed
                                            let now = Instant::now();
                                            if now.duration_since(last_heartbeat) >= Duration::from_secs(2) {
                                                if let Err(e) = gatt.send_heartbeat().await {
                                                    error!("Failed to send ACAIA heartbeat: {:?}", e);
                                                } else {
                                                    last_heartbeat = now;
                                                }
                                            }
                                        }
                                    }
                                    Err(e) => {
                                        error!("Failed to initialize ACAIA scale: {:?}", e);
                                    }
                                }
                            }).await;
                            info!("ACAIA GATT task completed, connection dropped");
                        }
                        Err(e) => {
                            error!("Failed to create ACAIA GATT client: {:?}", e);
                        }
                    }

                    // Wait before retrying
                    info!("Restarting ACAIA measurement loop...");
                    Timer::after(Duration::from_secs(5)).await;
                }
            },
        ),
    )
    .await;
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    //esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Initialize RTC for time synchronization
    let rtc = Rtc::new(peripherals.LPWR);

    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let esp_radio_ctrl = &*mk_static!(Controller<'static>, esp_radio::init().unwrap());

    // initializing Bluetooth first results in a more stable WiFi connection on
    // ESP32
    let connector = BleConnector::new(&esp_radio_ctrl, peripherals.BT, Default::default()).unwrap();
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

    // Create BLE host/stack
    info!("BLE: Creating stack...");
    let stack = trouble_host::new(controller, ble_resources).set_random_address(address);
    info!("BLE: Stack created");

    let stack = mk_static!(
        Stack<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
        stack
    );

    // Build the host to get central and runner
    info!("BLE: Building host...");
    let Host {
        central,
        runner,
        ..
    } = stack.build();
    info!("BLE: Host built");

    // Create event handler for scan reports (still useful for logging discovered devices)
    let printer = mk_static!(
        ScanPrinter,
        ScanPrinter {
            seen: RefCell::new(Deque::new()),
        }
    );

    // Create connection manager
    info!("BLE: Creating connection manager...");
    let connection_manager = mk_static!(
        BleConnectionManager<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
        BleConnectionManager::new(central)
    );

    // BLE device addresses
    let belka_address = BdAddr::new([0x3E, 0x60, 0xEB, 0x3C, 0x1C, 0x78]);
    let acaia_address = BdAddr::new([0x2f, 0xa0, 0x1a, 0x97, 0x1c, 0x00]);

    // Spawn BLE tasks
    spawner.spawn(ble_runner_task(runner, printer)).ok();
    spawner.spawn(ble_devices_task(connection_manager, stack, belka_address, acaia_address)).ok();
    info!("BLE: BLE tasks spawned");

    Timer::after_secs(5).await;

    let (mut controller, interfaces) =
        esp_radio::wifi::new(&esp_radio_ctrl, peripherals.WIFI, Default::default()).unwrap();

    let wifi_interface = interfaces.sta;

    controller
        .set_power_saving(esp_radio::wifi::PowerSaveMode::None)
        .unwrap();

    let config = embassy_net::Config::dhcpv4(Default::default());
    let rng = Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    let (stack, runner) = embassy_net::new(
        wifi_interface,
        config,
        mk_static!(StackResources<8>, StackResources::<8>::new()),
        seed,
    );

    // Make stack static for SNTP task
    let stack_static = mk_static!(embassy_net::Stack<'static>, stack);

    // Make RTC static for SNTP task
    let rtc_static = mk_static!(Rtc<'static>, rtc);

    spawner.spawn(connection(controller)).ok();
    spawner.spawn(net_task(runner)).ok();
    spawner.spawn(sntp_task(rtc_static, *stack_static)).ok();

    // Wait for link to come up
    info!("Waiting for link to come up...");
    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    // Wait for IP address via DHCP
    info!("Waiting to get IP address...");
    loop {
        if let Some(config) = stack.config_v4() {
            info!("Got IP: {}", config.address);
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    info!("Network ready, starting main loop");

    // Create TCP buffers for edge-nal-embassy adapter (8 concurrent connections)
    let tcp_buffers = mk_static!(
        TcpBuffers<8, 1024, 1024>,
        TcpBuffers::new()
    );

    // Create edge-nal TCP adapter for embassy-net stack (make it static for server task)
    let tcp_stack = mk_static!(
        Tcp<'static, 8, 1024, 1024>,
        Tcp::new(stack, tcp_buffers)
    );

    // Spawn HTTP server task
    spawner.spawn(http_server_task(tcp_stack)).ok();
    info!("HTTP server task spawned");

    loop {
        Timer::after(Duration::from_millis(1_000)).await;

        info!("Making HTTP request to www.mobile-j.de...");

        // Use hardcoded IP address (142.250.185.115 = www.google.com as proxy for www.mobile-j.de)
        let ip = Ipv4Addr::new(142, 250, 185, 115);
        let remote_addr = SocketAddr::from((ip, 80));
        info!("Connecting to {:?}", remote_addr);

        // Create connection buffer
        let mut buf = [0u8; 8192];

        // Create HTTP connection using the edge-nal TCP adapter (with max 64 headers)
        let mut conn = Connection::<_, 64>::new(&mut buf, tcp_stack, remote_addr);

        // Initiate request
        match conn
            .initiate_request(true, Method::Get, "/", &[("Host", "www.mobile-j.de")])
            .await
        {
            Ok(_) => info!("Request sent"),
            Err(e) => {
                info!("Failed to send request: {:?}", e);
                continue;
            }
        }

        // Initiate response
        match conn.initiate_response().await {
            Ok(_) => info!("Response initiated"),
            Err(e) => {
                info!("Failed to initiate response: {:?}", e);
                continue;
            }
        }

        // Read response
        let mut read_buf = [0u8; 1024];
        loop {
            match conn.read(&mut read_buf).await {
                Ok(0) => {
                    info!("Response complete");
                    break;
                }
                Ok(_len) => {
                    /*if let Ok(text) = core::str::from_utf8(&read_buf[..len]) {
                        print!("{}", text);
                    }*/
                }
                Err(e) => {
                    info!("Read error: {:?}", e);
                    break;
                }
            }
        }
        println!();

        Timer::after(Duration::from_millis(5000)).await;
    }
}