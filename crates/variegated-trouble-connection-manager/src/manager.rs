use core::cell::RefCell;
use embassy_time::{Duration, Timer};
use heapless::Vec;
use heapless::index_map::FnvIndexMap;
use trouble_host::prelude::*;
use trouble_host::PacketPool;
use bt_hci::controller::ControllerCmdSync;
use bt_hci::cmd::le::{LeSetScanParams, LeSetScanEnable};

use crate::handle::ManagerHandle;
use crate::types::ConnectionState;

/// State for a single managed device
pub(crate) struct DeviceState<'a, P: PacketPool> {
    address: BdAddr,
    maintain_connection: bool,
    connection: Option<Connection<'a, P>>,
    state: ConnectionState,
    last_disconnect: Option<embassy_time::Instant>,
}

/// Shared state accessed by handles (no Central here!)
pub struct BleConnectionManagerShared<'a, P: PacketPool> {
    devices: FnvIndexMap<BdAddr, DeviceState<'a, P>, 8>,
}

impl<'a, P: PacketPool> BleConnectionManagerShared<'a, P> {
    fn new() -> Self {
        Self {
            devices: FnvIndexMap::new(),
        }
    }

    /// Set maintain connection flag for a device
    pub(crate) fn set_maintain_connection(&mut self, address: BdAddr, maintain: bool) {
        if let Some(state) = self.devices.get_mut(&address) {
            defmt::info!("Set maintain_connection={} for device {}", maintain, address);
            state.maintain_connection = maintain;
        } else if maintain {
            // Auto-register device if setting maintain_connection to true
            defmt::info!("Auto-registering device {} with maintain_connection=true", address);
            let state = DeviceState {
                address,
                maintain_connection: true,
                connection: None,
                state: ConnectionState::Disconnected,
                last_disconnect: None,
            };
            let _ = self.devices.insert(address, state);
        }
    }

    /// Get connection state for a device
    pub(crate) fn get_connection_state(&self, address: BdAddr) -> ConnectionState {
        self.devices
            .get(&address)
            .map(|state| state.state)
            .unwrap_or(ConnectionState::Disconnected)
    }

    /// Get a reference to the connection for a device
    pub(crate) fn get_connection(&self, address: BdAddr) -> Option<&Connection<'a, P>> {
        self.devices
            .get(&address)
            .and_then(|state| state.connection.as_ref())
    }
}

/// BLE connection manager
pub struct BleConnectionManager<'a, C: Controller, P: PacketPool> {
    central: RefCell<Option<Central<'a, C, P>>>,
    shared: RefCell<BleConnectionManagerShared<'a, P>>,
}

impl<'a, C: Controller, P: PacketPool> BleConnectionManager<'a, C, P> {
    /// Create a new connection manager
    pub fn new(central: Central<'a, C, P>) -> Self {
        Self {
            central: RefCell::new(Some(central)),
            shared: RefCell::new(BleConnectionManagerShared::new()),
        }
    }

    /// Get a manager handle
    pub fn handle(&'a self) -> ManagerHandle<'a, C, P> {
        ManagerHandle {
            shared: &self.shared,
            _phantom: core::marker::PhantomData,
        }
    }

    /// Get a reference to the shared state RefCell
    ///
    /// This allows creating ManagerHandles from a shared reference to the RefCell,
    /// which is useful when the connection manager needs to be borrowed mutably
    /// for run() while handles are used in other tasks.
    pub fn shared_state(&'a self) -> &'a RefCell<BleConnectionManagerShared<'a, P>> {
        &self.shared
    }

    /// Main connection manager loop
    ///
    /// This task should be spawned and will run forever, managing all registered devices.
    /// It owns Central and performs all connection operations.
    pub async fn run(&self) -> ! {
        loop {
            // Collect devices that need connection (quickly borrow shared state)
            let to_connect: Vec<BdAddr, 8> = {
                let shared = self.shared.borrow();
                let mut addrs = Vec::new();
                let now = embassy_time::Instant::now();
                for (address, state) in shared.devices.iter() {
                    // Check if enough time has passed since last disconnect (cooldown period)
                    let cooldown_ok = if let Some(last_disconnect) = state.last_disconnect {
                        now.duration_since(last_disconnect) >= Duration::from_secs(2)
                    } else {
                        true
                    };

                    if state.maintain_connection
                        && state.connection.is_none()
                        && state.state != ConnectionState::Connecting
                        && cooldown_ok
                    {
                        let _ = addrs.push(*address);
                    }
                }
                addrs
            };

            // Try to connect to each device (Central is owned by self, no borrow issues!)
            for address in to_connect.iter() {
                // Mark as connecting
                {
                    let mut shared = self.shared.borrow_mut();
                    if let Some(state) = shared.devices.get_mut(address) {
                        defmt::info!("Attempting to connect to device {}", address);
                        state.state = ConnectionState::Connecting;
                    }
                }

                // Create connection configuration
                let config = ConnectConfig {
                    connect_params: Default::default(),
                    scan_config: ScanConfig {
                        active: false,
                        filter_accept_list: &[(AddrKind::PUBLIC, address), (AddrKind::RANDOM, address)],
                        interval: Duration::from_secs(2),
                        window: Duration::from_millis(400),
                        ..Default::default()
                    },
                };

                // Attempt connection - no borrow held during async operation!
                defmt::info!("Calling central.connect() for {}", address);
                let result = embassy_time::with_timeout(
                    Duration::from_secs(10),
                    self.central.borrow_mut().as_mut().expect("Central should exist").connect(&config),
                )
                .await;

                // Store result
                {
                    let mut shared = self.shared.borrow_mut();
                    if let Some(state) = shared.devices.get_mut(address) {
                        match result {
                            Ok(Ok(connection)) => {
                                defmt::info!("Successfully connected to device {}", address);
                                state.connection = Some(connection);
                                state.state = ConnectionState::Connected;
                            }
                            Ok(Err(_e)) => {
                                defmt::warn!("Failed to connect to device {}", address);
                                state.state = ConnectionState::Disconnected;
                            }
                            Err(_) => {
                                defmt::warn!("Connection timeout for device {}", address);
                                state.state = ConnectionState::Disconnected;
                            }
                        }
                    }
                }
            }

            // Check for disconnections and clean up
            {
                let mut shared = self.shared.borrow_mut();
                for (_address, state) in shared.devices.iter_mut() {
                    // Check if connection is still alive
                    let should_clear = if let Some(ref conn) = state.connection {
                        !conn.is_connected()
                    } else {
                        false
                    };

                    if should_clear {
                        defmt::info!("Device {} disconnected, cleaning up", state.address);
                        // Drop the connection to release the refcount
                        state.connection = None;
                        state.state = ConnectionState::Disconnected;
                        state.last_disconnect = Some(embassy_time::Instant::now());
                    }
                }
            }

            // Sleep between maintenance cycles
            Timer::after(Duration::from_millis(1000)).await;
        }
    }
}
