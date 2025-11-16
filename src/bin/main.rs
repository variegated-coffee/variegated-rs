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
use bt_hci::controller::ExternalController;
use trouble_host::prelude::*;
use core::net::{Ipv4Addr, SocketAddr};

use core::fmt::{Debug, Display};

use embassy_futures::join::join;
use heapless::Deque;

use embassy_executor::Spawner;
use embassy_net::{Runner as NetRunner, StackResources};
use embassy_time::{Duration, Timer};
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
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_radio::{
    Controller,
    ble::controller::BleConnector,
    wifi::{ClientConfig, ModeConfig, WifiController, WifiDevice, WifiEvent, WifiStaState},
};

use defmt::{error, info};

esp_bootloader_esp_idf::esp_app_desc!();

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
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
async fn ble_scanner_task(
    mut runner: Runner<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
    mut scanner: Scanner<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
    printer: &'static ScanPrinter,
) {
    let _ = join(runner.run_with_handler(printer), async {
        let target = Address {
            kind: AddrKind::PUBLIC,
//            addr: BdAddr::new([0x78, 0x1C, 0x3C, 0xEB, 0x60, 0x3E]),
            addr: BdAddr::new([0x3E, 0x60,  0xEB, 0x3C,  0x1C,  0x78, ]),
        };
//        let target: Address = Address::random([0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xff]);

        let list = &[(target.kind, &target.addr)];

        // Configure scanning
        let mut config = ScanConfig::default();
        config.active = true;
        config.phys = PhySet::M1;
        config.interval = Duration::from_secs(1);
        config.window = Duration::from_secs(1);
        config.timeout = Duration::from_secs(10);
        config.filter_accept_list = list;


        loop {
            info!("Starting BLE scan for 10 seconds...");
            match scanner.scan(&config).await {
                Ok(_session) => {
                    // Session keeps scan active, drop after 10 seconds
                    Timer::after(Duration::from_secs(10)).await;
                    info!("Scan session ended");
                }
                Err(_e) => {
                    error!("Failed to start BLE scan");
                }
            }
            // Wait 50 seconds before next scan (10s scan + 50s wait = 60s total)
            info!("Waiting 50 seconds before next scan...");
            Timer::after(Duration::from_secs(50)).await;
        }
    })
    .await;
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    //esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

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
        HostResources<DefaultPacketPool, 1, 2, 1>,
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

    // Create scanner and event handler (make them static for the task)
    let printer = mk_static!(
        ScanPrinter,
        ScanPrinter {
            seen: RefCell::new(Deque::new()),
        }
    );
    let scanner = Scanner::new(central);

    // Spawn BLE scanner task
    spawner.spawn(ble_scanner_task(runner, scanner, printer)).ok();

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

    spawner.spawn(connection(controller)).ok();
    spawner.spawn(net_task(runner)).ok();

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
                Ok(len) => {
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