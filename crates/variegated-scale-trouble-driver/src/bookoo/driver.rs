use core::cell::RefCell;
use defmt::info;
use trouble_host::attribute::Characteristic;
use trouble_host::gatt::NotificationListener;
use trouble_host::prelude::Uuid;
use variegated_scale_codec::bookoo::{Command, Reassembler};
use variegated_trouble_connection_manager::{
    Connection, Controller, DeviceHandle, GattClient, PacketPool, Stack,
};

use crate::bookoo::{
    types::{
        ScaleEvent, BOOKOO_COMMAND_CHAR_UUID, BOOKOO_SERVICE_UUID, BOOKOO_WEIGHT_CHAR_UUID,
    },
    Error,
};

/// Notification stream for BooKoo Themis scale events.
///
/// Wraps a `NotificationListener` and a [`Reassembler`]. Keep the `Connection` and the
/// GATT client alive for as long as this stream is in use.
pub struct BookooNotificationStream<'a> {
    listener: NotificationListener<'a, 512>,
    reassembler: Reassembler,
}

impl<'a> BookooNotificationStream<'a> {
    fn new(listener: NotificationListener<'a, 512>) -> Self {
        Self {
            listener,
            reassembler: Reassembler::new(),
        }
    }

    /// Wait for the next decoded event.
    pub async fn next(&mut self) -> Result<ScaleEvent, Error> {
        loop {
            // Drain what is already buffered *before* awaiting.
            //
            // The same rule the ACAIA stream follows, and it matters here for the same
            // reason: a notification can carry more than one frame, and awaiting first
            // leaves the second one sitting in the buffer until *another* notification
            // arrives -- a full connection interval later.
            //
            // For a weight alone that is invisible, the value merely late. It stops being
            // invisible the moment anything differentiates the stream. BooKoo frames each
            // carry their own millisecond timestamp, so a consumer that uses that rather
            // than arrival time is immune -- but nothing forces a consumer to, and the
            // cheap fix is to not create the skew in the first place.
            if let Some(frame) = self.reassembler.next_frame() {
                return Ok(frame.into());
            }

            let notification = self.listener.next().await;
            let data: &[u8] = notification.as_ref();

            if !self.reassembler.push(data) {
                // The buffer cleared itself. Not returned as an error: the stream is still
                // usable, the next whole frame will parse, and tearing the link down over
                // one overflow would cost a five-second reconnect for something that
                // recovers in 80 ms.
                defmt::warn!("BooKoo reassembly buffer overflowed, discarded");
            }
        }
    }

    /// Discard any partially-received frame.
    ///
    /// Worth calling after anything that could have interrupted the stream mid-frame; a
    /// leftover fragment can only corrupt the frame that follows it.
    pub fn reset(&mut self) {
        self.reassembler.reset();
    }
}

/// GATT client for a connected BooKoo scale.
///
/// Caches **two** characteristics, unlike [`crate::acaia_old::AcaiaOldGattClient`], which
/// caches one: ACAIA's older protocol notifies and writes on the same characteristic,
/// while BooKoo notifies on `FF11` and accepts commands on `FF12`. Both are discovered
/// together on first use, since a scale that has one always has the other and discovering
/// them separately would mean two round trips for no benefit.
pub struct BookooGattClient<'a, C: Controller, P: PacketPool> {
    client: GattClient<'a, C, P, 10>,
    characteristics: RefCell<Option<BookooCharacteristics>>,
}

#[derive(Clone)]
struct BookooCharacteristics {
    weight: Characteristic<Uuid>,
    command: Characteristic<Uuid>,
}

impl<'a, C: Controller, P: PacketPool> BookooGattClient<'a, C, P> {
    /// Create a GATT client for the given connection.
    ///
    /// The `Connection` must be kept alive by the caller for as long as this client is in
    /// use; dropping it invalidates the client.
    pub async fn new(stack: &'a Stack<'a, C, P>, conn: &Connection<'a, P>) -> Result<Self, Error> {
        let client = GattClient::<C, P, 10>::new(stack, conn).await.map_err(|_| {
            defmt::warn!("Failed to create GATT client");
            Error::GattError
        })?;

        Ok(Self {
            client,
            characteristics: RefCell::new(None),
        })
    }

    /// Discover both characteristics, lazily, and cache them.
    async fn ensure_characteristics(&self) -> Result<BookooCharacteristics, Error> {
        if let Some(cached) = self.characteristics.borrow().as_ref() {
            return Ok(cached.clone());
        }

        info!("Starting BooKoo service discovery...");
        let services = self
            .client
            .services_by_uuid(&BOOKOO_SERVICE_UUID)
            .await
            .map_err(|_| {
                defmt::warn!("Failed to discover BooKoo service");
                Error::ServiceNotFound
            })?;

        let service = services
            .first()
            .ok_or_else(|| {
                defmt::warn!("BooKoo service not found in results");
                Error::ServiceNotFound
            })?
            .clone();
        info!("BooKoo service found");

        let weight = self
            .client
            .characteristic_by_uuid(&service, &BOOKOO_WEIGHT_CHAR_UUID)
            .await
            .map_err(|_| {
                defmt::warn!("Failed to discover BooKoo weight characteristic");
                Error::CharacteristicNotFound
            })?;

        let command = self
            .client
            .characteristic_by_uuid(&service, &BOOKOO_COMMAND_CHAR_UUID)
            .await
            .map_err(|_| {
                defmt::warn!("Failed to discover BooKoo command characteristic");
                Error::CharacteristicNotFound
            })?;
        info!("BooKoo characteristics found");

        let found = BookooCharacteristics { weight, command };
        *self.characteristics.borrow_mut() = Some(found.clone());
        Ok(found)
    }

    /// The GATT client task, which must run concurrently with any GATT operation.
    pub async fn task(&self) -> Result<(), trouble_host::BleHostError<C::Error>> {
        self.client.task().await
    }

    /// Send one command.
    ///
    /// The frame is built by [`Command::encode`], which computes its own checksum. There
    /// is deliberately no way to pass raw bytes here: BooKoo's published checksums for the
    /// timer commands were wrong until 2026-07-30, and most third-party libraries still
    /// ship the invalid values, so a hand-written frame is more likely wrong than right.
    pub async fn send_command(&self, command: Command) -> Result<(), Error> {
        let characteristics = self.ensure_characteristics().await?;
        let frame = command.encode();

        self.client
            .write_characteristic_without_response(&characteristics.command, &frame)
            .await
            .map_err(|_| {
                defmt::warn!("Failed to write BooKoo command");
                Error::WriteFailed
            })?;

        Ok(())
    }

    /// Zero the scale.
    pub async fn send_tare(&self) -> Result<(), Error> {
        info!("Sending BooKoo tare");
        self.send_command(Command::Tare).await
    }

    /// Initialize the connection and subscribe to weight notifications.
    ///
    /// Far shorter than ACAIA's equivalent, and deliberately so: **BooKoo requires no
    /// handshake and no heartbeat.** The `[0x02, 0x00]` / `[0x00]` init sequence that
    /// appears in several third-party libraries -- and, until this change, in this repo's
    /// own `SCALE_PROTOCOLS.md` -- is an ACAIA leftover that BooKoo's specification does
    /// not mention. The scale streams as soon as the CCCD is written.
    ///
    /// The one write here is not initialisation but configuration: it turns the scale's
    /// own flow smoothing on, so that the flow this driver forwards has a known
    /// provenance. Without it the smoothing state is whatever the last user of the vendor
    /// app left it as, and two identical machines would report differently filtered flow.
    /// A failure to set it is logged and tolerated rather than fatal -- an unsmoothed
    /// scale is still a working scale.
    pub async fn initialize(&self) -> Result<BookooNotificationStream<'_>, Error> {
        let characteristics = self.ensure_characteristics().await?;

        info!("Subscribing to BooKoo weight notifications");
        let listener = self
            .client
            .subscribe(&characteristics.weight, false)
            .await
            .map_err(|_| {
                defmt::warn!("Failed to subscribe to BooKoo notifications");
                Error::SubscribeFailed
            })?;

        if self
            .send_command(Command::FlowSmoothing(true))
            .await
            .is_err()
        {
            defmt::warn!("Could not enable BooKoo flow smoothing; continuing unsmoothed");
        }

        info!("BooKoo initialization completed");
        Ok(BookooNotificationStream::new(listener))
    }
}

/// Driver for a BooKoo Themis scale.
///
/// Covers the Themis, the Themis Mini and the Themis Ultra: all three serve the same
/// service and characteristics and speak the same weight frame, and the Ultra's two extra
/// notification types are decoded by the same codec.
pub struct BookooDriver<'a, C: Controller, P: PacketPool> {
    device_handle: DeviceHandle<'a, C, P>,
    stack: &'a Stack<'a, C, P>,
}

impl<'a, C: Controller, P: PacketPool> BookooDriver<'a, C, P> {
    /// Create a driver from a device handle and the BLE stack.
    pub fn new(device_handle: DeviceHandle<'a, C, P>, stack: &'a Stack<'a, C, P>) -> Self {
        Self {
            device_handle,
            stack,
        }
    }

    /// Whether the connection manager should keep this device connected.
    pub async fn set_maintain_connection(&self, maintain: bool) {
        self.device_handle.set_maintain_connection(maintain).await;
    }

    /// Whether the device is connected right now.
    pub async fn is_connected(&self) -> bool {
        self.device_handle.is_connected().await
    }

    /// Create a GATT client for this device.
    ///
    /// The returned `Connection` must be kept alive for as long as the client is in use;
    /// dropping it invalidates the client.
    pub async fn gatt_client(
        &self,
    ) -> Result<(Connection<'a, P>, BookooGattClient<'a, C, P>), Error> {
        let conn = self
            .device_handle
            .clone_connection()
            .ok_or(Error::NotConnected)?;

        let gatt = BookooGattClient::new(self.stack, &conn).await?;
        Ok((conn, gatt))
    }

    /// The BLE address of this device.
    pub fn address(&self) -> variegated_trouble_connection_manager::BdAddr {
        self.device_handle.address()
    }
}
