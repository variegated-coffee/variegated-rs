use core::cell::RefCell;
use defmt::{debug, info};
use heapless;
use variegated_trouble_connection_manager::{Connection, Controller, DeviceHandle, GattClient, PacketPool, Stack};
use trouble_host::gatt::NotificationListener;
use trouble_host::attribute::Characteristic;
use trouble_host::prelude::Uuid;

use crate::acaia_old::{
    types::{ScaleEvent, WeightMeasurement, ACAIA_OLD_SERVICE_UUID, ACAIA_OLD_CHAR_UUID, IDENTIFICATION_MSG, NOTIFICATION_REQUEST_MSG, TARE_CMD, HEARTBEAT_MSG},
    Error,
};

/// Notification stream for ACAIA Old protocol scale events
///
/// This type wraps a NotificationListener and provides parsed scale events.
/// Note: You must keep the Connection and GattClient alive while using this stream.
pub struct ScaleNotificationStream<'a> {
    listener: NotificationListener<'a, 512>,
    /// Buffer for accumulating fragmented BLE notifications
    buffer: heapless::Vec<u8, 128>,
}

impl<'a> ScaleNotificationStream<'a> {
    /// Create a new notification stream from a listener
    fn new(listener: NotificationListener<'a, 512>) -> Self {
        Self {
            listener,
            buffer: heapless::Vec::new(),
        }
    }

    /// Wait for the next scale event notification
    ///
    /// Returns a parsed ScaleEvent or an error if parsing fails.
    /// This method buffers incoming data to handle fragmented BLE notifications.
    pub async fn next(&mut self) -> Result<ScaleEvent, Error> {
        loop {
            // Wait for next notification
            let notification = self.listener.next().await;
            let data: &[u8] = notification.as_ref();

            // Append to buffer
            if self.buffer.extend_from_slice(data).is_err() {
                defmt::warn!("Buffer overflow, clearing buffer");
                self.buffer.clear();
                return Err(Error::BufferOverflow);
            }

            defmt::debug!("Buffered {} bytes, total buffer size: {}", data.len(), self.buffer.len());
            debug!("Data: {:?}", self.buffer);

            // Try to parse a complete frame from the buffer
            if let Some(event) = self.try_parse_buffer()? {
                return Ok(event);
            }
            // No complete packet yet, loop back to wait for more data
        }
    }

    /// Try to parse a complete frame from the buffer
    ///
    /// Returns Some(event) if a complete frame was parsed and consumed,
    /// None if more data is needed, or an error for invalid data.
    ///
    /// Supports both NEW and OLD Acaia protocol formats:
    /// - NEW: [0xEF, 0xDD, cmd, len, msg_type, payload...]
    /// - OLD: [0xEF, 0xDD, weight_lo, weight_hi, ?, ?, scale, sign, ...]
    fn try_parse_buffer(&mut self) -> Result<Option<ScaleEvent>, Error> {
        // Find header position (0xEF 0xDD)
        let header_pos = self.buffer.windows(2)
            .position(|w| w[0] == 0xEF && w[1] == 0xDD);

        let Some(start) = header_pos else {
            // No header found yet
            if self.buffer.len() > 64 {
                // Buffer too large without header - likely garbage, clear it
                defmt::warn!("No header in {} bytes, clearing buffer", self.buffer.len());
                self.buffer.clear();
            } else if !self.buffer.is_empty() {
                // Keep buffering - header might come in next notification
                defmt::debug!("No header yet in {} bytes, waiting for more data", self.buffer.len());
            }
            return Ok(None);
        };

        // Discard bytes before header (garbage data)
        if start > 0 {
            defmt::debug!("Discarding {} bytes before header", start);
            // Shift buffer contents - drain is not available in heapless
            let remaining = self.buffer.len() - start;
            for i in 0..remaining {
                self.buffer[i] = self.buffer[start + i];
            }
            self.buffer.truncate(remaining);
        }

        // Need at least 4 bytes to determine format
        if self.buffer.len() < 4 {
            defmt::debug!("Incomplete frame: have {} bytes, need at least 4", self.buffer.len());
            return Ok(None);
        }

        // Detect NEW format: command byte is 0x0C (notification) or 0x08 (settings)
        let is_new_format = self.buffer[2] == 0x0C || self.buffer[2] == 0x08;

        if is_new_format {
            // Check if byte 3-4 is another header (incomplete frame case)
            // This happens when we receive [0xEF, 0xDD, 0x0C, 0xEF, 0xDD, ...]
            if self.buffer.len() >= 5 && self.buffer[3] == 0xEF && self.buffer[4] == 0xDD {
                // Incomplete frame - discard and move to next header
                defmt::debug!("Discarding incomplete frame (found header at byte 3)");
                self.consume_frame(3);
                return Ok(None);
            }

            // NEW Acaia format: [header(2), cmd(1), len(1), payload(len)]
            let command = self.buffer[2];
            let payload_len = self.buffer[3] as usize;

            // Sanity check: payload length should be reasonable (max ~20 bytes for scale data)
            if payload_len > 32 {
                defmt::warn!("Invalid payload length {}, skipping frame", payload_len);
                self.consume_frame(4);
                return Ok(None);
            }

            // Frame size: header(2) + cmd(1) + len(1) + payload + checksum(1)
            // The scale sends 1 extra byte (checksum/suffix) not included in payload_len
            // This matches the 13-byte packet format seen in Arduino library
            let frame_len = 4 + payload_len + 1;

            if self.buffer.len() < frame_len {
                defmt::debug!("Incomplete NEW frame: have {} bytes, need {}", self.buffer.len(), frame_len);
                return Ok(None);
            }

            let event = match command {
                0x0C => {
                    // Event notification - message type at byte 4
                    if payload_len < 1 {
                        self.consume_frame(frame_len);
                        return Err(Error::InvalidFrameLength);
                    }

                    let msg_type = self.buffer[4];
                    let event_payload = &self.buffer[5..frame_len];

                    match msg_type {
                        0x05 => {
                            // Weight event
                            defmt::debug!("NEW format weight: payload len={}", event_payload.len());
                            WeightMeasurement::parse_new(event_payload)
                                .map(ScaleEvent::Weight)?
                        }
                        0x07 => {
                            // Timer event - skip
                            defmt::info!("Skipping timer event");
                            self.consume_frame(frame_len);
                            return Ok(None);
                        }
                        0x08 => {
                            // Button event - skip
                            defmt::info!("Skipping button event");
                            self.consume_frame(frame_len);
                            return Ok(None);
                        }
                        0x0B => {
                            // Heartbeat response - skip
                            defmt::info!("Skipping heartbeat response");
                            self.consume_frame(frame_len);
                            return Ok(None);
                        }
                        _ => {
                            defmt::info!("Unknown message type 0x{:02x}", msg_type);
                            self.consume_frame(frame_len);
                            return Ok(None);
                        }
                    }
                }
                0x08 => {
                    // Settings message - skip
                    defmt::info!("Skipping settings message");
                    self.consume_frame(frame_len);
                    return Ok(None);
                }
                _ => {
                    defmt::info!("Unknown command 0x{:02x}", command);
                    self.consume_frame(frame_len);
                    return Ok(None);
                }
            };

            self.consume_frame(frame_len);
            Ok(Some(event))
        } else {
            // OLD Acaia format: [header(2), weight_lo, weight_hi, ?, ?, scale, sign, ...]
            const MIN_OLD_FRAME: usize = 8;
            const OLD_FRAME_LEN: usize = 10;

            if self.buffer.len() < MIN_OLD_FRAME {
                defmt::debug!("Incomplete OLD frame: have {} bytes, need at least {}", self.buffer.len(), MIN_OLD_FRAME);
                return Ok(None);
            }

            // Determine frame length by looking for next header
            let frame_len = if self.buffer.len() > 10 && self.buffer[10] == 0xEF {
                10
            } else if self.buffer.len() > 14 && self.buffer[14] == 0xEF {
                14
            } else {
                OLD_FRAME_LEN.min(self.buffer.len())
            };

            let payload = &self.buffer[2..frame_len];

            // Validate scale index
            if payload.len() >= 6 && payload[4] > 4 {
                defmt::info!("Skipping invalid OLD frame (scale_index={})", payload[4]);
                self.consume_frame(frame_len);
                return Ok(None);
            }

            let event = WeightMeasurement::parse_old(payload)
                .map(ScaleEvent::Weight)?;

            self.consume_frame(frame_len);
            Ok(Some(event))
        }
    }

    /// Remove processed bytes from the front of the buffer
    fn consume_frame(&mut self, frame_len: usize) {
        let remaining = self.buffer.len() - frame_len;
        for i in 0..remaining {
            self.buffer[i] = self.buffer[frame_len + i];
        }
        self.buffer.truncate(remaining);
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
