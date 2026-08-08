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

/// How long the controller listens for advertisements in each scan pass.
///
/// This is the number that matters to Wi-Fi, not the duty cycle. An ESP32-C6 has one
/// 2.4 GHz antenna, so a scan window is a stretch of time the Wi-Fi side cannot
/// transmit or receive in -- and a *single* blackout longer than a round trip is enough
/// to cost a TCP retransmit and put the connection on the RTO ladder, which is measured
/// in seconds. Shortening the window shortens the worst thing that can happen to a
/// connection, independently of how often it happens.
///
/// 150 ms is chosen to stay comfortably above a connectable advertising interval, which
/// for a scale announcing itself after power-on is typically 20-100 ms. A window at
/// least one advertising interval long catches the device on its first pass; a shorter
/// one turns discovery into a coin flip repeated every [`SCAN_INTERVAL`]. If a scale
/// ever turns out to advertise more slowly than this, discovery degrades gracefully
/// (more passes needed) and this is the number to raise.
const SCAN_WINDOW: Duration = Duration::from_millis(150);

/// How often a scan pass starts.
///
/// Together with [`SCAN_WINDOW`] this is a 7.5% duty cycle, down from the 20% that
/// `400ms / 2s` gave. The machine is powered continuously while the scales are on for
/// seconds at a time, so the absent-device case is not an edge case -- it is what this
/// loop does essentially always, and it should be quiet.
///
/// The budget it is sized against is discovery within 5-10 s of a scale being switched
/// on. Worst case here is one cooldown plus one interval, so about 4 s, with the typical
/// case nearer 2 s. That leaves room to lengthen this further if Wi-Fi still needs the
/// airtime.
const SCAN_INTERVAL: Duration = Duration::from_secs(2);

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
            // Capacity is a const generic on a heapless map, so this is the one failure
            // the caller cannot see coming and cannot recover from. It used to be
            // `let _ =`, which meant a full map turned `set_maintain_connection(_, true)`
            // into a silent no-op: the device is never connected, never reported, and
            // nothing anywhere says why. Registering more devices than the map holds is
            // a caller bug, so say so rather than hiding it.
            if self.devices.insert(address, state).is_err() {
                defmt::error!(
                    "Device table full ({} entries); refusing to register {}",
                    self.devices.len(),
                    address
                );
            }
        }
    }

    /// Stop maintaining a device, drop its link, and free its slot in the table.
    ///
    /// The counterpart to the auto-registration in [`Self::set_maintain_connection`].
    /// Without it the table is append-only: clearing `maintain_connection` leaves the
    /// entry in place forever, so a user re-pairing a peripheral a handful of times
    /// exhausts the eight slots and every subsequent registration fails.
    ///
    /// **The `disconnect` is not optional.** Dropping the stored [`Connection`] only
    /// releases one refcount; the controller keeps the ACL link up until supervision
    /// timeout, which is seconds to tens of seconds. For that whole window the
    /// peripheral still believes it is connected, and a `connect` for the same address
    /// -- exactly what happens when a slot is reassigned to a device that was just
    /// released -- collides with a link that is nominally still alive.
    ///
    /// Synchronous, deliberately. `Connection::disconnect` only queues a request for the
    /// control runner to service, so there is nothing to await, and callers need to be
    /// able to run this during a cancellation teardown where they cannot.
    pub(crate) fn remove_device(&mut self, address: BdAddr) {
        if let Some(state) = self.devices.get_mut(&address) {
            defmt::info!("Removing device {} from the table", address);
            state.maintain_connection = false;
            if let Some(connection) = state.connection.take() {
                connection.disconnect();
            }
        }
        let _ = self.devices.remove(&address);
    }

    /// Every address currently in the table, connected or not.
    ///
    /// For auditing only: a caller that tracks which devices it has registered can
    /// compare against this and report a discrepancy. Deliberately not paired with a
    /// "remove everything unclaimed" helper -- an orphan here means some caller failed
    /// to release what it registered, and collecting it silently would hide that bug
    /// while leaving its cause in place.
    pub(crate) fn registered_addresses(&self) -> Vec<BdAddr, 8> {
        let mut addrs = Vec::new();
        for address in self.devices.keys() {
            let _ = addrs.push(*address);
        }
        addrs
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
                        interval: SCAN_INTERVAL,
                        window: SCAN_WINDOW,
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
                            // A failed attempt starts the cooldown too, not just a
                            // dropped connection. Without this, `last_disconnect` stays
                            // `None` for a device that has never connected, the cooldown
                            // check reads that as "ready", and the loop retries with no
                            // gap at all -- so a scale that is simply switched off keeps
                            // this radio scanning essentially without pause.
                            //
                            // That is not free on an ESP32-C6: Wi-Fi and BLE share one
                            // 2.4 GHz antenna, and every scan window is time the Wi-Fi
                            // side cannot transmit or receive in. Retrying instantly is
                            // also the case where retrying is *least* likely to help,
                            // since nothing has changed since the last attempt.
                            Ok(Err(_e)) => {
                                defmt::warn!("Failed to connect to device {}", address);
                                state.state = ConnectionState::Disconnected;
                                state.last_disconnect = Some(embassy_time::Instant::now());
                            }
                            Err(_) => {
                                defmt::warn!("Connection timeout for device {}", address);
                                state.state = ConnectionState::Disconnected;
                                state.last_disconnect = Some(embassy_time::Instant::now());
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
