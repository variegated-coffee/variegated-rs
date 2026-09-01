use core::cell::RefCell;
use defmt::{debug, info};
use trouble_host::attribute::Characteristic;
use trouble_host::gatt::NotificationListener;
use trouble_host::prelude::Uuid;
use variegated_scale_codec::acaia::{
    self as codec, Generation, Reassembler, TimerOp, WeightUnit,
};
use variegated_trouble_connection_manager::{
    Connection, Controller, DeviceHandle, GattClient, PacketPool, Stack,
};

use crate::acaia_new::{
    types::{
        ScaleEvent, ACAIA_NEW_NOTIFY_CHAR_UUID, ACAIA_NEW_SERVICE_UUID,
        ACAIA_NEW_WRITE_CHAR_UUID,
    },
    Error,
};

/// The frames sent during the handshake and periodically afterwards.
///
/// Computed by the codec, not written out, and **identical to the ones the pre-2021 driver
/// sends** — the two generations differ in incoming framing and GATT topology, not in
/// commands. `SCALE_PROTOCOLS.md` claims this generation uses an identity frame padded with
/// `0x2D` and a heartbeat of `EF DD 00 00 EF DD`; both are wrong, and the codec's tests say
/// so explicitly.
const IDENTIFICATION_MSG: [u8; 20] = codec::identification();
const NOTIFICATION_REQUEST_MSG: [u8; 14] = codec::notification_request();
const TARE_CMD: [u8; 6] = codec::tare();
const HEARTBEAT_MSG: [u8; 7] = codec::heartbeat();
const TIMER_START_CMD: [u8; 7] = codec::timer(TimerOp::Start);
const TIMER_STOP_CMD: [u8; 7] = codec::timer(TimerOp::Stop);
const TIMER_RESET_CMD: [u8; 7] = codec::timer(TimerOp::Reset);

/// Notification stream for a 2021+ ACAIA scale.
pub struct AcaiaNewNotificationStream<'a> {
    listener: NotificationListener<'a, 512>,
    reassembler: Reassembler,
    /// Latches the `discarded()` warning, so a persistently rejected stream says so once
    /// rather than at the notification rate.
    warned_discarding: bool,
}

impl<'a> AcaiaNewNotificationStream<'a> {
    fn new(listener: NotificationListener<'a, 512>) -> Self {
        Self {
            listener,
            reassembler: Reassembler::new(Generation::Modern),
            warned_discarding: false,
        }
    }

    /// Wait for the next decoded event.
    pub async fn next(&mut self) -> Result<ScaleEvent, Error> {
        loop {
            // Drain what is already buffered *before* awaiting -- the rule both other
            // drivers follow, for the same reason: a notification can carry two frames, and
            // awaiting first delays the second by a full connection interval, which skews
            // any rate derived from arrival times.
            if let Some(frame) = self.reassembler.next_frame() {
                return Ok(frame);
            }

            let notification = self.listener.next().await;
            let data: &[u8] = notification.as_ref();

            if !self.reassembler.push(data) {
                debug!("ACAIA reassembly buffer overflowed, discarded");
                return Err(Error::BufferOverflow);
            }

            // The one diagnostic that separates "quiet scale" from "every frame rejected".
            //
            // The checksummed region for this generation is verified against a single
            // capture from a single scale. If some model computes it differently, the link
            // comes up, the handshake logs success, and nothing is ever published -- which
            // is indistinguishable from a handshake that did not take. A growing
            // `discarded()` is what tells the two apart.
            if !self.warned_discarding && self.reassembler.discarded() > 64 {
                self.warned_discarding = true;
                defmt::warn!(
                    "ACAIA: {} bytes discarded resynchronising; if no weights arrive, the \
                     checksummed region may differ on this model",
                    self.reassembler.discarded()
                );
            }
        }
    }

    /// Discard any partially-received frame, and the scale's reported display unit.
    pub fn reset(&mut self) {
        self.reassembler.reset();
    }

    /// The unit the scale last reported it was displaying in.
    pub fn unit(&self) -> WeightUnit {
        self.reassembler.unit()
    }
}

/// GATT client for a connected 2021+ ACAIA scale.
///
/// Caches **two** characteristics, like the BooKoo client and unlike the pre-2021 ACAIA one:
/// this generation notifies on one characteristic and accepts commands on another.
pub struct AcaiaNewGattClient<'a, C: Controller, P: PacketPool> {
    client: GattClient<'a, C, P, 10>,
    characteristics: RefCell<Option<AcaiaNewCharacteristics>>,
}

#[derive(Clone)]
struct AcaiaNewCharacteristics {
    notify: Characteristic<Uuid>,
    write: Characteristic<Uuid>,
}

impl<'a, C: Controller, P: PacketPool> AcaiaNewGattClient<'a, C, P> {
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

    async fn ensure_characteristics(&self) -> Result<AcaiaNewCharacteristics, Error> {
        if let Some(cached) = self.characteristics.borrow().as_ref() {
            return Ok(cached.clone());
        }

        info!("Starting ACAIA 2021+ service discovery...");
        let services = self
            .client
            .services_by_uuid(&ACAIA_NEW_SERVICE_UUID)
            .await
            .map_err(|_| {
                defmt::warn!("Failed to discover ACAIA 2021+ service");
                Error::ServiceNotFound
            })?;

        let service = services
            .first()
            .ok_or_else(|| {
                // If this fires on a scale that is definitely a 2021+ model, suspect the
                // 128-bit UUID byte order before anything else -- see the note on
                // `ACAIA_NEW_SERVICE_UUID`. A transposed UUID matches nothing and looks
                // exactly like this.
                defmt::warn!("ACAIA 2021+ service not found in results");
                Error::ServiceNotFound
            })?
            .clone();
        info!("ACAIA 2021+ service found");

        let notify = self
            .client
            .characteristic_by_uuid(&service, &ACAIA_NEW_NOTIFY_CHAR_UUID)
            .await
            .map_err(|_| {
                defmt::warn!("Failed to discover ACAIA notify characteristic");
                Error::CharacteristicNotFound
            })?;

        let write = self
            .client
            .characteristic_by_uuid(&service, &ACAIA_NEW_WRITE_CHAR_UUID)
            .await
            .map_err(|_| {
                defmt::warn!("Failed to discover ACAIA write characteristic");
                Error::CharacteristicNotFound
            })?;
        info!("ACAIA 2021+ characteristics found");

        let found = AcaiaNewCharacteristics { notify, write };
        *self.characteristics.borrow_mut() = Some(found.clone());
        Ok(found)
    }

    /// The GATT client's own task, which must run concurrently with any GATT operation.
    pub async fn task(&self) -> Result<(), trouble_host::BleHostError<C::Error>> {
        self.client.task().await
    }

    async fn write_command(&self, frame: &[u8], what: &'static str) -> Result<(), Error> {
        let characteristics = self.ensure_characteristics().await?;
        self.client
            .write_characteristic_without_response(&characteristics.write, frame)
            .await
            .map_err(|_| {
                defmt::warn!("Failed to write ACAIA {}", what);
                Error::WriteFailed
            })
    }

    /// Zero the scale.
    pub async fn send_tare(&self) -> Result<(), Error> {
        info!("Sending ACAIA tare");
        self.write_command(&TARE_CMD, "tare").await
    }

    /// Drive the scale's own timer.
    ///
    /// ACAIA has no combined tare-and-start command, unlike BooKoo; a caller wanting both
    /// sends [`Self::send_tare`] and then this with [`TimerOp::Start`].
    pub async fn send_timer(&self, op: TimerOp) -> Result<(), Error> {
        let frame: &[u8] = match op {
            TimerOp::Start => &TIMER_START_CMD,
            TimerOp::Stop => &TIMER_STOP_CMD,
            TimerOp::Reset => &TIMER_RESET_CMD,
        };
        self.write_command(frame, "timer command").await
    }

    /// Send a heartbeat.
    ///
    /// Without one the scale drops the link — reported variously at 2750 ms and 3000 ms, so
    /// the session sends them well inside the shorter figure.
    pub async fn send_heartbeat(&self) -> Result<(), Error> {
        debug!("Sending ACAIA heartbeat");
        self.write_command(&HEARTBEAT_MSG, "heartbeat").await
    }

    /// Re-send the identity frame.
    ///
    /// Documented as a Pyxis quirk — that model reportedly wants identity before each
    /// heartbeat — rather than a requirement of the protocol, which is why the session sends
    /// it periodically rather than on every tick.
    pub async fn send_identification(&self) -> Result<(), Error> {
        debug!("Sending ACAIA identification");
        self.write_command(&IDENTIFICATION_MSG, "identification")
            .await
    }

    /// Initialize the connection and subscribe to notifications.
    ///
    /// The order is the one every implementation agrees on, and which the pre-2021 driver
    /// already uses:
    ///
    /// 1. subscribe to notifications
    /// 2. the platform init write `[0x00, 0x01]`
    /// 3. **wait 150 ms**
    /// 4. identification
    /// 5. notification request
    ///
    /// Step 3 is load-bearing and nobody knows why. Beanconqueror's comment on it reads:
    /// *"Maybe this sleep is game changer for the connection issue that no weight is send.
    /// After implementing this 150ms sleep, somehow the weight is always send afterwards -
    /// why? we don't know"*. Step 5 is what actually starts the stream; without it the link
    /// is up and silent.
    pub async fn initialize(&self) -> Result<AcaiaNewNotificationStream<'_>, Error> {
        let characteristics = self.ensure_characteristics().await?;

        info!("Subscribing to ACAIA 2021+ notifications");
        let listener = self
            .client
            .subscribe(&characteristics.notify, false)
            .await
            .map_err(|_| {
                defmt::warn!("Failed to subscribe to ACAIA notifications");
                Error::SubscribeFailed
            })?;

        info!("Sending platform init write");
        self.write_command(&[0x00, 0x01], "platform init")
            .await
            .map_err(|_| Error::HandshakeFailed)?;

        embassy_time::Timer::after(embassy_time::Duration::from_millis(150)).await;

        info!("Sending identification message");
        self.write_command(&IDENTIFICATION_MSG, "identification")
            .await
            .map_err(|_| Error::HandshakeFailed)?;

        info!("Sending notification request");
        self.write_command(&NOTIFICATION_REQUEST_MSG, "notification request")
            .await
            .map_err(|_| Error::HandshakeFailed)?;

        info!("ACAIA 2021+ initialization completed");
        Ok(AcaiaNewNotificationStream::new(listener))
    }
}

/// Driver for a 2021-and-later ACAIA scale.
///
/// Covers the Pyxis, the Lunar 2021 (AL014 and later), the Pearl 2021, the Pearl S and the
/// Cinco, which share a service, a characteristic pair and a frame format.
pub struct AcaiaNewDriver<'a, C: Controller, P: PacketPool> {
    device_handle: DeviceHandle<'a, C, P>,
    stack: &'a Stack<'a, C, P>,
}

impl<'a, C: Controller, P: PacketPool> AcaiaNewDriver<'a, C, P> {
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
    pub async fn gatt_client(
        &self,
    ) -> Result<(Connection<'a, P>, AcaiaNewGattClient<'a, C, P>), Error> {
        let conn = self
            .device_handle
            .clone_connection()
            .ok_or(Error::NotConnected)?;

        let gatt = AcaiaNewGattClient::new(self.stack, &conn).await?;
        Ok((conn, gatt))
    }

    /// The BLE address of this device.
    pub fn address(&self) -> variegated_trouble_connection_manager::BdAddr {
        self.device_handle.address()
    }
}
