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

use bleps::{
    Ble,
    HciConnector,
    ad_structure::{
        AdStructure,
        BR_EDR_NOT_SUPPORTED,
        LE_GENERAL_DISCOVERABLE,
        create_advertising_data,
    },
    att::Uuid,
};
use core::net::{Ipv4Addr, SocketAddr};

use core::fmt::{Debug, Display};

use embassy_executor::Spawner;
use embassy_net::{Runner, StackResources};
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
use esp_println::{print, println};
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
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
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

    let now = || esp_hal::time::Instant::now().duration_since_epoch().as_millis();

    // initializing Bluetooth first results in a more stable WiFi connection on
    // ESP32
    let connector = BleConnector::new(&esp_radio_ctrl, peripherals.BT, Default::default()).unwrap();
    let hci = HciConnector::new(connector, now);
    let mut ble = Ble::new(&hci);

    info!("{:?}", ble.init());
    println!("{:?}", ble.cmd_set_le_advertising_parameters());
    println!(
        "{:?}",
        ble.cmd_set_le_advertising_data(
            create_advertising_data(&[
                AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                AdStructure::ServiceUuids16(&[Uuid::Uuid16(0x1809)]),
                AdStructure::CompleteLocalName(esp_hal::chip!()),
            ])
                .unwrap()
        )
    );
    println!("{:?}", ble.cmd_set_le_advertise_enable(true));

    info!("started advertising");

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
                    if let Ok(text) = core::str::from_utf8(&read_buf[..len]) {
                        print!("{}", text);
                    }
                }
                Err(e) => {
                    info!("Read error: {:?}", e);
                    break;
                }
            }
        }
        println!();

        Timer::after(Duration::from_millis(3000)).await;
    }
}