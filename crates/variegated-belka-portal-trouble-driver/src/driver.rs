use defmt::info;
use variegated_trouble_connection_manager::{Connection, Controller, DeviceHandle, GattClient, PacketPool, Stack};
use trouble_host::gatt::NotificationListener;
use trouble_host::attribute::Characteristic;
use trouble_host::prelude::Uuid;

use crate::{types::{Measurements, BELKA_SERVICE_UUID, COMMAND_CHAR_UUID, MEASUREMENT_CHAR_UUID}, Error};

/// Notification stream for Belka Portal measurements
///
/// This type wraps a NotificationListener and provides parsed measurement data.
/// Note: You must keep the Connection and GattClient alive while using this stream.
pub struct MeasurementNotificationStream<'a> {
    listener: NotificationListener<'a, 512>,
}

impl<'a> MeasurementNotificationStream<'a> {
    /// Create a new notification stream from a listener
    fn new(listener: NotificationListener<'a, 512>) -> Self {
        Self { listener }
    }

    /// Wait for the next measurement notification
    ///
    /// Returns a parsed Measurements struct or an error if parsing fails.
    pub async fn next(&mut self) -> Result<Measurements, Error> {
        let notification = self.listener.next().await;
        let data: &[u8] = notification.as_ref();

        if data.len() != 13 {
            defmt::warn!("Invalid notification data length: {} (expected 13)", data.len());
            return Err(Error::InvalidDataLength);
        }

        let mut buf = [0u8; 13];
        buf.copy_from_slice(data);
        Measurements::parse(&buf)
    }
}

/// Helper struct for GATT operations on Belka Portal device
///
/// This type manages GATT client operations.
/// Create it once and use it for multiple operations (read, subscribe, etc.)
///
/// Note: The Connection must be kept alive externally while this client is in use.
pub struct BelkaGattClient<'a, C: Controller, P: PacketPool> {
    client: GattClient<'a, C, P, 10>,
}

impl<'a, C: Controller, P: PacketPool> BelkaGattClient<'a, C, P> {
    /// Create a GATT client for the given connection
    ///
    /// Note: The Connection must be kept alive by the caller for as long as this
    /// GattClient is in use. Dropping the Connection will invalidate this client.
    pub async fn new(stack: &'a Stack<'a, C, P>, conn: &Connection<'a, P>) -> Result<Self, Error> {
        let client = GattClient::<C, P, 10>::new(stack, conn)
            .await
            .map_err(|_| {
                defmt::warn!("Failed to create GATT client");
                Error::ReadFailed
            })?;

        Ok(Self { client })
    }

    /// Get the GATT client task that must be run concurrently with GATT operations
    ///
    /// This task processes GATT events and responses. It must be running for any
    /// GATT operations (read, subscribe, etc.) to work. Use with join:
    ///
    /// ```no_run
    /// join(gatt.task(), async {
    ///     // perform GATT operations here
    /// }).await;
    /// ```
    pub async fn task(&self) -> Result<(), trouble_host::BleHostError<C::Error>> {
        self.client.task().await
    }

    /// Read measurements from the device
    pub async fn read_measurements(&self) -> Result<Measurements, Error> {
        info!("Let's get services");
        // Discover service
        let services = self.client
            .services_by_uuid(&BELKA_SERVICE_UUID)
            .await
            .map_err(|_| Error::ReadFailed)?;

        let service = services.first().ok_or(Error::ReadFailed)?.clone();

        info!("We've got our service, let's get the characteristic.");

        // Discover characteristic
        let characteristic: Characteristic<Uuid> = self.client
            .characteristic_by_uuid(&service, &MEASUREMENT_CHAR_UUID)
            .await
            .map_err(|_| Error::ReadFailed)?;

        info!("We've got our characteristic, let's get the data.");
        // Read value
        let mut data = [0u8; 13];
        self.client
            .read_characteristic(&characteristic, &mut data[..])
            .await
            .map_err(|_| Error::ReadFailed)?;

        info!("We have the data, let's parse it.");

        Measurements::parse(&data)
    }

    /// Write a command to the Portal's command characteristic.
    ///
    /// The payloads are in [`crate::SHOW_GRAPH`] and [`crate::HIDE_GRAPH`], and their doc
    /// comments explain why they are byte arrays rather than numbers.
    ///
    /// **An acknowledged write**, unlike the BooKoo command path's write-without-response.
    /// That is not a preference: the only capture of this characteristic working is a
    /// `Write Request`, so it is the one form known to be accepted. `Ok(())` therefore means
    /// the Portal acknowledged receiving the bytes -- it says nothing about whether it
    /// understood them, because no part of this protocol reports that.
    pub async fn write_command(&self, payload: &[u8]) -> Result<(), Error> {
        let services = self
            .client
            .services_by_uuid(&BELKA_SERVICE_UUID)
            .await
            .map_err(|_| Error::WriteFailed)?;

        let service = services.first().ok_or(Error::ServiceNotFound)?.clone();

        let characteristic: Characteristic<Uuid> = self
            .client
            .characteristic_by_uuid(&service, &COMMAND_CHAR_UUID)
            .await
            .map_err(|_| Error::CharacteristicNotFound)?;

        self.client
            .write_characteristic(&characteristic, payload)
            .await
            .map_err(|_| Error::WriteFailed)
    }

    /// Subscribe to measurement notifications
    pub async fn subscribe(&self) -> Result<MeasurementNotificationStream<'_>, Error> {
        // Discover service
        let services = self.client
            .services_by_uuid(&BELKA_SERVICE_UUID)
            .await
            .map_err(|_| Error::SubscribeFailed)?;

        let service = services.first().ok_or(Error::SubscribeFailed)?.clone();

        // Discover characteristic
        let characteristic: Characteristic<Uuid> = self.client
            .characteristic_by_uuid(&service, &MEASUREMENT_CHAR_UUID)
            .await
            .map_err(|_| Error::SubscribeFailed)?;

        // Subscribe
        let listener = self.client
            .subscribe(&characteristic, false)
            .await
            .map_err(|_| Error::SubscribeFailed)?;

        Ok(MeasurementNotificationStream::new(listener))
    }
}

/// Driver for the Belka Portal device
pub struct BelkaPortalDriver<'a, C: Controller, P: PacketPool> {
    device_handle: DeviceHandle<'a, C, P>,
    stack: &'a Stack<'a, C, P>,
}

impl<'a, C: Controller, P: PacketPool> BelkaPortalDriver<'a, C, P> {
    /// Create a new Belka Portal driver
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
    /// Returns a tuple of (Connection, BelkaGattClient). The Connection must be
    /// kept alive for as long as the GattClient is in use - dropping it will
    /// invalidate the client.
    ///
    /// # Example
    /// ```no_run
    /// // Create GATT client
    /// let (conn, gatt) = driver.gatt_client().await?;
    ///
    /// // Read measurements (conn must stay in scope)
    /// let measurements = gatt.read_measurements().await?;
    ///
    /// // Or subscribe to notifications
    /// let mut stream = gatt.subscribe().await?;
    /// loop {
    ///     let measurement = stream.next().await?;
    ///     // Process measurement
    /// }
    /// // conn is dropped here, invalidating gatt
    /// ```
    pub async fn gatt_client(&self) -> Result<(Connection<'a, P>, BelkaGattClient<'a, C, P>), Error> {
        let conn = self.device_handle
            .clone_connection()
            .ok_or(Error::NotConnected)?;

        let gatt = BelkaGattClient::new(self.stack, &conn).await?;
        Ok((conn, gatt))
    }

    /// Get the BLE address of this device
    pub fn address(&self) -> variegated_trouble_connection_manager::BdAddr {
        self.device_handle.address()
    }
}
