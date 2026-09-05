use core::cell::RefCell;
use embassy_time::{Duration, Timer};
use heapless::Vec;
use heapless::index_map::FnvIndexMap;
use trouble_host::prelude::*;
use trouble_host::PacketPool;
use bt_hci::controller::ControllerCmdSync;
use bt_hci::cmd::le::{LeSetScanParams, LeSetScanEnable};

use embassy_futures::select::{select, Either};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

use crate::handle::ManagerHandle;
use crate::types::ConnectionState;

/// A request to stop maintaining connections for a moment and look for new devices.
#[derive(Clone, Copy)]
pub struct ScanRequest {
    pub duration: Duration,
    /// Whether to solicit scan responses.
    ///
    /// **True for discovery.** Most peripherals put their name in the scan response
    /// rather than the advertisement, so a passive scan finds devices that are all
    /// address and no label -- useless for a list a human has to pick from. The cost is
    /// that the radio transmits, which is why the connection loop's own scans (below)
    /// stay passive.
    pub active: bool,
}

/// What a scan reports to, so the manager does not have to know how results are carried.
///
/// Reports themselves never come through here: they arrive at the host's `EventHandler`,
/// which is driven by a different task entirely. This is only the bracket, which is what
/// lets the handler tell "a device seen during the scan the user asked for" from "a
/// device seen incidentally while connecting".
pub trait ScanSink {
    fn begin(&self);
    /// `started` distinguishes a scan that ran and saw nothing from one the controller
    /// refused to start.
    ///
    /// Without it the two are indistinguishable from outside -- both end with an empty
    /// list -- and they call for opposite responses: one means the device is not
    /// advertising, the other means the radio never looked.
    fn end(&self, started: bool);
    /// Called once per failed start attempt, with the host error where there is one.
    ///
    /// Exists because this crate logs through `defmt` directly and the firmware's debug
    /// transports do not carry that, so a refusal reached the operator as silence. The
    /// HCI status is the whole diagnosis here -- `CommandDisallowed` means something
    /// still holds the accept list, anything else means this is not the race we think it
    /// is -- and guessing at it from the outside has already cost two flash cycles.
    ///
    /// `None` when the failure came from the controller's own error type, which has no
    /// `defmt::Format` bound available here.
    fn attempt_failed(&self, error: Option<&trouble_host::Error>);

    // -- Connect lifecycle ---------------------------------------------------------
    //
    // Not scanning, despite the trait's name, and kept here rather than in a second
    // trait because there is exactly one thing on the other end of both and `run` takes
    // one `sink`. They exist for the same reason `attempt_failed` does: this crate logs
    // with raw `defmt`, which the firmware's debug transports do not carry, so every
    // outcome below currently reaches the operator as silence.
    //
    // **Deliberately without default bodies.** One implementor exists, and a hook that
    // can be silently left unimplemented is a hook that will be, at which point a metric
    // reads a flat zero and gets believed.

    /// An attempt is about to start, after `waited` spent queued behind other addresses
    /// in the same pass.
    ///
    /// The wait is the interesting half: attempts are serialized through one `Central`
    /// and each runs up to ten seconds, so a device four places down the list can be
    /// half a minute from being tried without anything having failed.
    fn connect_attempt(&self, address: BdAddr, waited: Duration);

    /// The ten-second `with_timeout` expired -- the controller never established a link.
    ///
    /// Distinct from [`Self::connect_error`] because they mean opposite things: this is
    /// "we listened and heard nothing", which on a shared antenna is as likely to be
    /// airtime as an absent device.
    fn connect_timed_out(&self, address: BdAddr);

    /// The host returned an error rather than timing out.
    fn connect_error(&self, address: BdAddr);

    /// A scan request arrived mid-attempt and the attempt was dropped for it.
    ///
    /// Worth counting separately from a timeout: the device may have been about to
    /// connect, and the abandonment also starts the same two-second cooldown a failure
    /// does, so a stream of scan requests can starve a connect indefinitely without
    /// producing a single failure.
    fn connect_abandoned(&self, address: BdAddr);
}

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

/// Listen window for a user-initiated discovery scan.
///
/// # These are not the times they look like -- read this before changing them
///
/// `LeSetScanParams` and `LeCreateConn` take their scan interval and window as
/// `bt_hci::param::Duration<10_000>`, so trouble-host's `bt_hci_duration` divides what it
/// is given by 10 000 µs to get the raw HCI value. The Bluetooth spec defines that field
/// in units of **0.625 ms**. So the number reaching the air is *sixteen times shorter*
/// than the `Duration` written here, and what actually has to be legal is the raw value:
/// the spec requires `0x0004..=0x4000`, and window must not exceed interval.
///
/// Getting that wrong is not a subtle degradation. 30 ms here divides to a raw 3, which
/// is below the minimum, and the controller rejects the whole command with
/// `Invalid HCI Command Parameters` -- no scan at all, which is exactly what happened.
///
/// So, concretely:
///
/// | written here | raw HCI | on air |
/// |---|---|---|
/// | window 300 ms | 30 | 18.75 ms |
/// | interval 1000 ms | 100 | 62.5 ms |
///
/// # Why this regime
///
/// Different from [`SCAN_WINDOW`] above, deliberately. That one is sized for a background
/// loop that runs forever and must not starve Wi-Fi; this one runs for a few seconds
/// because somebody is standing at the machine waiting for their scale to appear, so it
/// buys discovery latency with airtime it only spends briefly.
///
/// The ratio -- 30% duty cycle -- is what matters and is unaffected by the unit
/// confusion above. That is heavy for a single-antenna radio also carrying Wi-Fi and the
/// live links to the peripherals themselves, since the ACAIA driver drops its connection
/// after a couple of missed heartbeats. Hence the caller's time box and the application
/// processor refusing a scan outright while a shot is running. If a connected scale turns
/// out not to survive a scan, widen the interval here first.
const DISCOVERY_SCAN_WINDOW: Duration = Duration::from_millis(300);
/// See [`DISCOVERY_SCAN_WINDOW`] -- in particular the note that this is not 1 s on air.
const DISCOVERY_SCAN_INTERVAL: Duration = Duration::from_millis(1000);

/// How many times to try starting a scan before giving up, and how long to wait between.
///
/// Together about a second, which is generously more than one HCI round trip -- the thing
/// actually being waited for is a `LeCreateConnCancel` reaching the controller and being
/// acknowledged. See the retry loop in `run_scan` for why this is a retry rather than a
/// delay.
const SCAN_START_ATTEMPTS: usize = 5;
/// See [`SCAN_START_ATTEMPTS`].
const SCAN_START_RETRY_INTERVAL: Duration = Duration::from_millis(200);

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
    /// Pending discovery request, consumed by [`Self::run`].
    ///
    /// A `Signal` with exactly one waiter -- `run` -- and latest-wins, which is the right
    /// reading of a user pressing "scan" twice: they want one scan, not two queued.
    scan_request: Signal<CriticalSectionRawMutex, ScanRequest>,
}

impl<'a, C: Controller, P: PacketPool> BleConnectionManager<'a, C, P> {
    /// Create a new connection manager
    pub fn new(central: Central<'a, C, P>) -> Self {
        Self {
            central: RefCell::new(Some(central)),
            shared: RefCell::new(BleConnectionManagerShared::new()),
            scan_request: Signal::new(),
        }
    }

    /// Ask for a discovery scan.
    ///
    /// Never awaits and never fails, so it is callable from the UART reader, where
    /// back-pressure would stall every other message on the link. The scan itself happens
    /// inside [`Self::run`], because that is the only place that can hold the `Central`.
    pub fn request_scan(&self, request: ScanRequest) {
        self.scan_request.signal(request);
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
    /// It owns Central and performs all connection operations -- including discovery
    /// scans, which cannot happen anywhere else: `Scanner::new` consumes the `Central`,
    /// and this loop holds it borrowed across every `connect` await.
    pub async fn run<S: ScanSink>(&self, sink: &S) -> !
    where
        C: ControllerCmdSync<LeSetScanParams> + ControllerCmdSync<LeSetScanEnable>,
    {
        loop {
            // Handled before anything else in the pass, so a request that arrived while
            // the previous pass was connecting is served promptly rather than after
            // another round of attempts.
            if let Some(request) = self.scan_request.try_take() {
                self.run_scan(request, sink).await;
                continue;
            }

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

            // When this pass started trying to connect. Each attempt reports how long it
            // waited from here, which is what makes the serialization visible: the
            // addresses are tried one at a time and each may take ten seconds.
            let pass_start = embassy_time::Instant::now();

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
                sink.connect_attempt(*address, pass_start.elapsed());

                // Create connection configuration
                let config = ConnectConfig {
                    connect_params: Default::default(),
                    scan_config: ScanConfig {
                        active: false,
                        // trouble 0.7 takes `Address` here rather than the
                        // `(AddrKind, &BdAddr)` pairs 0.6 wanted. Both kinds are still
                        // offered for the same reason as before: an association may predate
                        // the `address_random` flag, so which kind a device advertises with
                        // is not always known.
                        filter_accept_list: &[
                            Address { kind: AddrKind::PUBLIC, addr: *address },
                            Address { kind: AddrKind::RANDOM, addr: *address },
                        ],
                        interval: SCAN_INTERVAL,
                        window: SCAN_WINDOW,
                        ..Default::default()
                    },
                };

                // Attempt connection - no borrow held during async operation!
                //
                // Raced against a scan request, and the race is what makes "scan"
                // responsive. Each attempt runs for up to ten seconds, and with four
                // peripherals switched off the loop would otherwise take forty seconds to
                // reach the check at the top -- long enough that a user presses the button
                // again, and again.
                //
                // Dropping the `connect` future is trouble-host's supported cancellation:
                // `Central::connect` installs an `OnDrop` that cancels the connection
                // command state, which the control runner turns into `LeCreateConnCancel`.
                defmt::info!("Calling central.connect() for {}", address);
                // The guard is bound rather than left as a temporary, and it is held
                // across the await either way -- it always was, when this was a single
                // `with_timeout(..).await` expression. That is the fact that forces
                // `run_scan` to live in this loop: for the whole of a connect attempt,
                // nothing else can borrow the `Central`.
                //
                // Dropped at the end of this iteration, including on the `break` below,
                // which is what lets the next pass hand it to the scanner.
                let mut central = self.central.borrow_mut();
                let attempt = embassy_time::with_timeout(
                    Duration::from_secs(10),
                    central.as_mut().expect("Central should exist").connect(&config),
                );

                let result = match select(attempt, self.scan_request.wait()).await {
                    Either::First(result) => result,
                    Either::Second(request) => {
                        defmt::info!("Scan requested; abandoning the connect attempt for {}", address);
                        sink.connect_abandoned(*address);
                        {
                            let mut shared = self.shared.borrow_mut();
                            if let Some(state) = shared.devices.get_mut(address) {
                                // Start the cooldown, exactly as a failed attempt does.
                                // Without it this device would be retried immediately
                                // after the scan, with no gap.
                                state.state = ConnectionState::Disconnected;
                                state.last_disconnect = Some(embassy_time::Instant::now());
                            }
                        }
                        // Put it back rather than acting on it here: the top of the loop
                        // is the one place that runs a scan, and duplicating that would
                        // mean two sites that have to agree about the `Central`.
                        self.scan_request.signal(request);
                        break;
                    }
                };

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
                                sink.connect_error(*address);
                                state.state = ConnectionState::Disconnected;
                                state.last_disconnect = Some(embassy_time::Instant::now());
                            }
                            Err(_) => {
                                defmt::warn!("Connection timeout for device {}", address);
                                sink.connect_timed_out(*address);
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

    /// Run one discovery scan, then hand the `Central` back to the connection loop.
    ///
    /// Live connections survive this. `LeSetScanEnable` does not touch established ACL
    /// links -- only new connection attempts are paused, which is the whole reason the
    /// scan is time-boxed rather than left running.
    async fn run_scan<S: ScanSink>(&self, request: ScanRequest, sink: &S)
    where
        C: ControllerCmdSync<LeSetScanParams> + ControllerCmdSync<LeSetScanEnable>,
    {
        // `take`, and this is why `central` is an `Option`: `Scanner::new` consumes the
        // `Central` by value and `into_inner` gives it back.
        let Some(central) = self.central.borrow_mut().take() else {
            defmt::warn!("[ble] scan requested with no central available");
            return;
        };
        let mut scanner = Scanner::new(central);

        let config = ScanConfig {
            active: request.active,
            // Empty, which trouble-host turns into `BasicUnfiltered` -- the opposite of
            // the connect path above, which filters to a single address.
            filter_accept_list: &[],
            interval: DISCOVERY_SCAN_INTERVAL,
            window: DISCOVERY_SCAN_WINDOW,
            // Zero means "no controller-side deadline". The time box below is ours,
            // deliberately: `ScanSession` in trouble-host 0.6 has no awaitable
            // completion -- its `deadline` and `done` fields are written and never
            // polled -- so waiting on the session would wait forever.
            timeout: Duration::from_secs(0),
            ..Default::default()
        };

        sink.begin();

        // Retried against a *transient* refusal, which is a narrower thing than it may
        // look. The failure this was written for turned out to be a permanent one -- a
        // scan window below the spec minimum, see `DISCOVERY_SCAN_WINDOW` -- and no
        // number of retries would have helped; the parameters were simply illegal.
        //
        // It earns its place for a different case. `Scanner::scan` opens by calling
        // `set_accept_filter`, which issues `LE Clear Filter Accept List`, and the spec
        // makes that Command Disallowed while the list is in use by an outstanding
        // `LE Create Connection` -- which is precisely the state a scan that pre-empted a
        // connect leaves behind. Dropping the connect future queues the cancellation
        // (`OnDrop` -> `connect_command_state.cancel`, which the control runner turns
        // into `LeCreateConnCancel`), but completion is asynchronous and there is no way
        // to await it: `CommandState::wait_idle` exists and `connect_command_state` is
        // private to trouble-host.
        //
        // Retrying rather than sleeping a guessed interval, because it converges as soon
        // as the controller will accept the command. A failing `scan()` unwinds its own
        // `OnDrop` and returns the scan command state to idle, so each attempt is clean.
        //
        // Bounded at about a second. Anything still refused after that is a parameter
        // problem, not a race, and the error is reported through the sink so it says so.
        let mut started = false;
        for attempt in 0..SCAN_START_ATTEMPTS {
            match scanner.scan(&config).await {
                Ok(session) => {
                    defmt::info!("[ble] discovery scan started");
                    Timer::after(request.duration).await;
                    // Dropping the session cancels the scan, which the control runner
                    // turns into `LeSetScanEnable(false)`.
                    drop(session);
                    started = true;
                    break;
                }
                // Matched rather than formatted whole: `BleHostError`'s `Controller` arm
                // carries the controller's own error type, which has no `defmt::Format`
                // bound here and would force one on every caller. The host arm is the
                // informative one anyway -- it is where an HCI status such as
                // `CommandDisallowed` surfaces.
                Err(BleHostError::BleHost(e)) => {
                    defmt::warn!("[ble] scan attempt {} failed: {:?}", attempt, e);
                    sink.attempt_failed(Some(&e));
                }
                Err(_) => {
                    defmt::warn!("[ble] scan attempt {} failed: controller error", attempt);
                    sink.attempt_failed(None);
                }
            }
            Timer::after(SCAN_START_RETRY_INTERVAL).await;
        }
        sink.end(started);

        // Borrow-checked ordering: `scan` takes `&mut scanner` and the session borrows
        // it, so the session is necessarily dropped before this line.
        *self.central.borrow_mut() = Some(scanner.into_inner());
        defmt::info!("[ble] discovery scan finished; resuming connections");
    }
}
