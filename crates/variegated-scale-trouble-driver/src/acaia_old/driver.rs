use core::cell::RefCell;
use defmt::{debug, info};
use variegated_trouble_connection_manager::{Connection, Controller, DeviceHandle, GattClient, PacketPool, Stack};
use trouble_host::gatt::NotificationListener;
use trouble_host::attribute::Characteristic;
use trouble_host::prelude::Uuid;

use crate::acaia_old::{
    types::{ScaleEvent, WeightMeasurement, ACAIA_OLD_SERVICE_UUID, ACAIA_OLD_CHAR_UUID, IDENTIFICATION_MSG, NOTIFICATION_REQUEST_MSG, TARE_CMD, HEARTBEAT_MSG, TIMER_START_CMD, TIMER_STOP_CMD, TIMER_RESET_CMD},
    Error,
};
pub use variegated_scale_codec::acaia::TimerOp;
use variegated_scale_codec::acaia::{Frame, Generation, Reassembler};

/// Notification stream for ACAIA Old protocol scale events
///
/// This type wraps a NotificationListener and provides parsed scale events.
/// Note: You must keep the Connection and GattClient alive while using this stream.
///
/// Parsing lives in `variegated-scale-codec`, which can host tests; this crate cannot. See
/// that crate's docs for why.
pub struct ScaleNotificationStream<'a> {
    listener: NotificationListener<'a, 512>,
    reassembler: Reassembler,
    /// Latches the "this looks like a modern scale" warning below.
    warned_modern: bool,
}

impl<'a> ScaleNotificationStream<'a> {
    /// Create a new notification stream from a listener
    fn new(listener: NotificationListener<'a, 512>) -> Self {
        Self {
            listener,
            // **Pinned, not auto-detected.** This driver serves the pre-2021 protocol and
            // nothing else -- the association's `BluetoothDriverKind` chose it -- so the
            // generation is known and does not need guessing.
            //
            // It used to be guessed, per frame, by testing whether byte 2 was 0x0C or 0x08.
            // In a legacy frame byte 2 is the weight's *low* byte, so about two values in
            // 256 were routed into the modern branch and lost. At factor 2 those are
            // 20.60 g and 31.75 g -- ordinary shot weights.
            reassembler: Reassembler::new(Generation::Legacy),
            warned_modern: false,
        }
    }

    /// Wait for the next scale event notification
    ///
    /// Returns a parsed ScaleEvent or an error if parsing fails.
    /// This method buffers incoming data to handle fragmented BLE notifications.
    pub async fn next(&mut self) -> Result<ScaleEvent, Error> {
        loop {
            // Drain what is already buffered *before* awaiting.
            //
            // The reassembler returns on the first complete frame and leaves the remainder
            // in place, so one notification carrying two frames leaves the second one
            // sitting here. Awaiting first meant that second event was not returned until
            // *another* notification arrived -- a full connection interval, 80 ms on this
            // link, later.
            //
            // For weight alone that is invisible: the value is still correct, just late. It
            // stops being invisible once something differentiates the stream, because the
            // late event is timestamped on arrival: one sample is stamped ~80 ms after it
            // was really taken, and the sample after it gets a correspondingly short
            // interval. A rate computed across either is wrong in opposite directions.
            while let Some(frame) = self.reassembler.next_frame() {
                match frame {
                    Frame::Weight(w) => {
                        return Ok(ScaleEvent::Weight(WeightMeasurement { weight: w.grams }));
                    }
                    // Timer, button, status and unrecognised frames are discarded here
                    // exactly as they always were. `ScaleEvent` has one variant and the
                    // slot loop matches it exhaustively; widening it is a separate change.
                    other => debug!("Discarding ACAIA frame: {:?}", other),
                }
            }

            let notification = self.listener.next().await;
            let data: &[u8] = notification.as_ref();

            // Latched, and it has to be. The test below is the *ambiguous* one that used to
            // route frames -- in a legacy frame byte 2 is the weight's low byte -- so on a
            // perfectly healthy scale it fires for roughly 0.8% of samples, which at this
            // notification rate would be several lines a minute forever. Once per
            // connection is enough to tell someone their scale is not what the association
            // says it is.
            if !self.warned_modern
                && data.len() >= 3
                && data[0] == 0xEF
                && data[1] == 0xDD
                && (data[2] == 0x0C || data[2] == 0x08)
            {
                self.warned_modern = true;
                defmt::warn!(
                    "possible 2021+ frame on the pre-2021 ACAIA driver; if weights look \
                     wrong, re-pair the scale as ACAIA (2021 and later). This can also be \
                     a legacy weight whose low byte happens to collide."
                );
            }

            if !self.reassembler.push(data) {
                // Preserved: this driver tears the link down on overflow, where the BooKoo
                // one warns and continues. Changing it is a separate decision, and moving
                // the buffer into the codec made it easy to change by accident.
                defmt::warn!("Buffer overflow, clearing buffer");
                return Err(Error::BufferOverflow);
            }
        }
    }

    /// Discard any partially-received frame, and the scale's reported display unit.
    pub fn reset(&mut self) {
        self.reassembler.reset();
    }
}

/// Helper struct for GATT operations on ACAIA Old protocol scale
///
/// This type manages GATT client operations.
/// Create it once and use it for multiple operations (handshake, read, subscribe, etc.)
///
/// Note: The Connection must be kept alive externally while this client is in use.
pub struct AcaiaOldGattClient<'a, C: Controller, P: PacketPool> {
    client: GattClient<'a, C, P, 10>,
    /// Cached characteristic to avoid repeated discovery (lazily initialized)
    characteristic: RefCell<Option<Characteristic<Uuid>>>,
}

impl<'a, C: Controller, P: PacketPool> AcaiaOldGattClient<'a, C, P> {
    /// Create a GATT client for the given connection
    ///
    /// Note: The Connection must be kept alive by the caller for as long as this
    /// GattClient is in use. Dropping the Connection will invalidate this client.
    pub async fn new(stack: &'a Stack<'a, C, P>, conn: &Connection<'a, P>) -> Result<Self, Error> {
        let client = GattClient::<C, P, 10>::new(stack, conn)
            .await
            .map_err(|_| {
                defmt::warn!("Failed to create GATT client");
                Error::GattError
            })?;

        Ok(Self { client, characteristic: RefCell::new(None) })
    }

    /// Ensure the characteristic is discovered, lazily initializing on first use
    async fn ensure_characteristic(&self) -> Result<Characteristic<Uuid>, Error> {
        // Check if already discovered
        if let Some(char) = self.characteristic.borrow().as_ref() {
            return Ok(char.clone());
        }

        // Discover service and characteristic
        defmt::info!("Starting ACAIA service discovery...");
        let services = self.client
            .services_by_uuid(&ACAIA_OLD_SERVICE_UUID)
            .await
            .map_err(|_| {
                defmt::warn!("Failed to discover ACAIA service");
                Error::ServiceNotFound
            })?;

        let service = services.first().ok_or_else(|| {
            defmt::warn!("ACAIA service not found in results");
            Error::ServiceNotFound
        })?.clone();
        defmt::info!("ACAIA service found");

        defmt::info!("Starting ACAIA characteristic discovery...");
        let characteristic = self.client
            .characteristic_by_uuid(&service, &ACAIA_OLD_CHAR_UUID)
            .await
            .map_err(|_| {
                defmt::warn!("Failed to discover ACAIA characteristic");
                Error::CharacteristicNotFound
            })?;
        defmt::info!("ACAIA characteristic found");

        // Cache and return
        *self.characteristic.borrow_mut() = Some(characteristic.clone());
        Ok(characteristic)
    }

    /// Get the GATT client task that must be run concurrently with GATT operations
    ///
    /// This task processes GATT events and responses. It must be running for any
    /// GATT operations (handshake, subscribe, etc.) to work. Use with select:
    ///
    /// ```no_run
    /// select(gatt.task(), async {
    ///     // perform GATT operations here
    /// }).await;
    /// ```
    pub async fn task(&self) -> Result<(), trouble_host::BleHostError<C::Error>> {
        self.client.task().await
    }

    /// Perform the ACAIA Old protocol handshake
    ///
    /// This sends the identification message and notification request message
    /// to initialize communication with the scale.
    pub async fn perform_handshake(&self) -> Result<(), Error> {
        let characteristic = self.ensure_characteristic().await?;

        info!("Sending identification message");
        self.client
            .write_characteristic_without_response(&characteristic, &IDENTIFICATION_MSG)
            .await
            .map_err(|_| {
                defmt::warn!("Failed to write identification message");
                Error::HandshakeFailed
            })?;

        info!("Sending notification request");
        self.client
            .write_characteristic_without_response(&characteristic, &NOTIFICATION_REQUEST_MSG)
            .await
            .map_err(|_| {
                defmt::warn!("Failed to write notification request");
                Error::HandshakeFailed
            })?;

        info!("Handshake completed successfully");
        Ok(())
    }

    /// Send a tare (zero) command to the scale
    pub async fn send_tare(&self) -> Result<(), Error> {
        let characteristic = self.ensure_characteristic().await?;

        info!("Sending tare command");
        self.client
            .write_characteristic_without_response(&characteristic, &TARE_CMD)
            .await
            .map_err(|_| {
                defmt::warn!("Failed to write tare command");
                Error::WriteFailed
            })?;

        Ok(())
    }

    /// Drive the scale's own timer.
    ///
    /// ACAIA has no combined tare-and-start command, unlike BooKoo; a caller wanting both
    /// sends [`Self::send_tare`] and then this with [`TimerOp::Start`]. The two writes are
    /// issued from the same future as every other write in the connected scope, so they
    /// cannot interleave with a heartbeat.
    pub async fn send_timer(&self, op: TimerOp) -> Result<(), Error> {
        let characteristic = self.ensure_characteristic().await?;

        let frame: &[u8] = match op {
            TimerOp::Start => &TIMER_START_CMD,
            TimerOp::Stop => &TIMER_STOP_CMD,
            TimerOp::Reset => &TIMER_RESET_CMD,
        };

        info!("Sending timer command");
        self.client
            .write_characteristic_without_response(&characteristic, frame)
            .await
            .map_err(|_| {
                defmt::warn!("Failed to write timer command");
                Error::WriteFailed
            })?;

        Ok(())
    }

    /// Send a heartbeat message to the scale
    ///
    /// The ACAIA Old protocol requires periodic heartbeat messages (every 2750ms minimum)
    /// to keep the scale sending notifications. This should be called regularly while
    /// subscribed to scale events.
    pub async fn send_heartbeat(&self) -> Result<(), Error> {
        let characteristic = self.ensure_characteristic().await?;

        defmt::info!("Sending heartbeat");
        self.client
            .write_characteristic_without_response(&characteristic, &HEARTBEAT_MSG)
            .await
            .map_err(|_| {
                defmt::warn!("Failed to write heartbeat");
                Error::WriteFailed
            })?;

        Ok(())
    }

    /// Initialize the scale connection and subscribe to notifications
    ///
    /// This performs the full initialization sequence in the correct order:
    /// 1. Subscribe to notifications
    /// 2. Wait 150ms (critical for scale to be ready)
    /// 3. Send identification message
    /// 4. Send notification request
    ///
    /// Returns a stream that yields parsed scale events.
    ///
    /// **IMPORTANT**: Use this method instead of calling subscribe() and perform_handshake()
    /// separately. The scale requires notifications to be enabled BEFORE receiving
    /// handshake messages.
    pub async fn initialize(&self) -> Result<ScaleNotificationStream<'_>, Error> {
        let characteristic = self.ensure_characteristic().await?;

        // Step 1: Subscribe to notifications first
        info!("Subscribing to scale notifications");
        let listener = self.client
            .subscribe(&characteristic, false)
            .await
            .map_err(|_| {
                defmt::warn!("Failed to subscribe to notifications");
                Error::SubscribeFailed
            })?;

        // Step 2: Platform-specific write (required by Android/iOS v1)
        // This triggers the scale to start accepting commands
        info!("Sending platform init write");
        self.client
            .write_characteristic_without_response(&characteristic, &[0x00, 0x01])
            .await
            .map_err(|_| {
                defmt::warn!("Failed to write platform init");
                Error::HandshakeFailed
            })?;

        // Step 3: Wait 150ms for scale to be ready
        info!("Waiting 150ms for scale to be ready");
        embassy_time::Timer::after(embassy_time::Duration::from_millis(150)).await;

        // Step 4: Send identification message
        info!("Sending identification message");
        self.client
            .write_characteristic_without_response(&characteristic, &IDENTIFICATION_MSG)
            .await
            .map_err(|_| {
                defmt::warn!("Failed to write identification message");
                Error::HandshakeFailed
            })?;

        // Step 5: Send notification request
        info!("Sending notification request");
        self.client
            .write_characteristic_without_response(&characteristic, &NOTIFICATION_REQUEST_MSG)
            .await
            .map_err(|_| {
                defmt::warn!("Failed to write notification request");
                Error::HandshakeFailed
            })?;

        info!("Scale initialization completed successfully");
        Ok(ScaleNotificationStream::new(listener))
    }

    /// Subscribe to scale event notifications (low-level)
    ///
    /// **WARNING**: Consider using `initialize()` instead, which performs the full
    /// initialization sequence in the correct order.
    ///
    /// If using this method directly, you MUST:
    /// 1. Call subscribe() first
    /// 2. Wait 150ms
    /// 3. Call perform_handshake()
    ///
    /// Returns a stream that yields parsed scale events.
    pub async fn subscribe(&self) -> Result<ScaleNotificationStream<'_>, Error> {
        let characteristic = self.ensure_characteristic().await?;

        info!("Subscribing to scale notifications");
        let listener = self.client
            .subscribe(&characteristic, false)
            .await
            .map_err(|_| {
                defmt::warn!("Failed to subscribe to notifications");
                Error::SubscribeFailed
            })?;

        Ok(ScaleNotificationStream::new(listener))
    }
}

/// Driver for the ACAIA Old protocol scale
pub struct AcaiaOldDriver<'a, C: Controller, P: PacketPool> {
    device_handle: DeviceHandle<'a, C, P>,
    stack: &'a Stack<'a, C, P>,
}

impl<'a, C: Controller, P: PacketPool> AcaiaOldDriver<'a, C, P> {
    /// Create a new ACAIA Old protocol driver
    ///
    /// # Arguments
    /// * `device_handle` - Handle obtained from the connection manager via `manager.handle().register_device(address)`
    /// * `stack` - Reference to the BLE host stack
    pub fn new(device_handle: DeviceHandle<'a, C, P>, stack: &'a Stack<'a, C, P>) -> Self {
        Self { device_handle, stack }
    }

    /// Set whether to automatically maintain connection to this device
    ///
    /// When set to `true`, the connection manager will:
    /// - Scan for the device if not connected
    /// - Attempt to connect when discovered
    /// - Automatically reconnect if the connection is lost
    pub async fn set_maintain_connection(&self, maintain: bool) {
        self.device_handle.set_maintain_connection(maintain).await;
    }

    /// Check if the device is currently connected
    pub async fn is_connected(&self) -> bool {
        self.device_handle.is_connected().await
    }

    /// Create a GATT client for performing operations on this device
    ///
    /// Returns a tuple of (Connection, AcaiaOldGattClient). The Connection must be
    /// kept alive for as long as the GattClient is in use - dropping it will
    /// invalidate the client.
    ///
    /// # Example
    /// ```no_run
    /// // Create GATT client
    /// let (conn, mut gatt) = driver.gatt_client().await?;
    ///
    /// // Perform handshake (required before subscribing)
    /// gatt.perform_handshake().await?;
    ///
    /// // Subscribe to notifications
    /// let mut stream = gatt.subscribe().await?;
    /// loop {
    ///     let event = stream.next().await?;
    ///     // Process event (Weight, Timer, or Battery)
    /// }
    /// // conn is dropped here, invalidating gatt
    /// ```
    pub async fn gatt_client(&self) -> Result<(Connection<'a, P>, AcaiaOldGattClient<'a, C, P>), Error> {
        let conn = self.device_handle
            .clone_connection()
            .ok_or(Error::NotConnected)?;

        let gatt = AcaiaOldGattClient::new(self.stack, &conn).await?;
        Ok((conn, gatt))
    }

    /// Get the BLE address of this device
    pub fn address(&self) -> variegated_trouble_connection_manager::BdAddr {
        self.device_handle.address()
    }
}
