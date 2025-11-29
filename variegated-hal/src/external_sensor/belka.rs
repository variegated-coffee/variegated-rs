use core::cell::Cell;
use defmt::{info, warn, Format};
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, RawMutex};
use embassy_sync::channel::Receiver;
use embassy_sync::signal::Signal;
use embassy_sync::watch::Sender as WatchSender;
use variegated_controller_types::{
    ECType, ExternalPeripheralSensorReading, PeripheralId, PeripheralStatusProvider,
    PeripheralType, TemperatureType,
};

use crate::{SensorReading, WithTask};

/// Belka Portal endpoint for EC (Electrical Conductivity)
pub const BELKA_ENDPOINT_EC: u8 = 0;

/// Belka Portal endpoint for Temperature
pub const BELKA_ENDPOINT_TEMPERATURE: u8 = 1;

/// Belka Portal endpoint for Battery level (currently ignored)
pub const BELKA_ENDPOINT_BATTERY: u8 = 2;

/// Message type for Belka device updates from comms layer
#[derive(Clone, Debug, Format)]
pub enum BelkaUpdate {
    /// A sensor reading from the Portal
    Reading(ExternalPeripheralSensorReading),
    /// Connection status changed
    ConnectionChanged(bool),
}

/// The Belka Portal device - runs as an async task, receives updates from comms
pub struct BelkaDevice<'a, M: RawMutex, const N: usize, const UPDATE_CHAN_SIZE: usize> {
    /// Peripheral ID for this device
    peripheral_id: PeripheralId,

    /// Channel receiving updates from comms layer
    update_receiver: Receiver<'a, M, BelkaUpdate, UPDATE_CHAN_SIZE>,

    /// Output temperature watch sender
    temperature_sender: Option<WatchSender<'a, NoopRawMutex, SensorReading<TemperatureType>, N>>,

    /// Output EC watch sender
    ec_sender: Option<WatchSender<'a, NoopRawMutex, SensorReading<ECType>, N>>,

    /// Signal for connection status (used by status provider)
    connected_signal: Option<&'a Signal<NoopRawMutex, bool>>,

    /// Current connection state
    is_connected: bool,
}

impl<'a, M: RawMutex, const N: usize, const UPDATE_CHAN_SIZE: usize>
    BelkaDevice<'a, M, N, UPDATE_CHAN_SIZE>
{
    pub fn new(
        peripheral_id: PeripheralId,
        update_receiver: Receiver<'a, M, BelkaUpdate, UPDATE_CHAN_SIZE>,
        temperature_sender: Option<WatchSender<'a, NoopRawMutex, SensorReading<TemperatureType>, N>>,
        ec_sender: Option<WatchSender<'a, NoopRawMutex, SensorReading<ECType>, N>>,
    ) -> Self {
        Self {
            peripheral_id,
            update_receiver,
            temperature_sender,
            ec_sender,
            connected_signal: None,
            is_connected: false,
        }
    }

    pub fn with_connected_signal(mut self, signal: &'a Signal<NoopRawMutex, bool>) -> Self {
        self.connected_signal = Some(signal);
        self
    }

    fn update_connection_status(&mut self, connected: bool) {
        if self.is_connected != connected {
            self.is_connected = connected;

            if let Some(signal) = self.connected_signal {
                signal.signal(connected);
            }

            if connected {
                info!("Belka Portal connected (id: 0x{:04X})", self.peripheral_id);
            } else {
                warn!("Belka Portal disconnected (id: 0x{:04X})", self.peripheral_id);
                // Publish zeros to indicate invalid readings
                self.publish_zeros();
            }
        }
    }

    fn publish_zeros(&self) {
        if let Some(ref sender) = self.temperature_sender {
            sender.send(SensorReading {
                raw: 0.0,
                transformed: 0.0,
            });
        }
        if let Some(ref sender) = self.ec_sender {
            sender.send(SensorReading {
                raw: 0.0,
                transformed: 0.0,
            });
        }
    }

    fn publish_reading(&self, endpoint: u8, value: f32) {
        match endpoint {
            BELKA_ENDPOINT_TEMPERATURE => {
                if let Some(ref sender) = self.temperature_sender {
                    sender.send(SensorReading {
                        raw: value,
                        transformed: value,
                    });
                }
            }
            BELKA_ENDPOINT_EC => {
                if let Some(ref sender) = self.ec_sender {
                    sender.send(SensorReading {
                        raw: value,
                        transformed: value,
                    });
                }
            }
            BELKA_ENDPOINT_BATTERY => {
                // Ignored for now
            }
            _ => {
                warn!("Unknown Belka endpoint: {}", endpoint);
            }
        }
    }
}

impl<'a, M: RawMutex, const N: usize, const UPDATE_CHAN_SIZE: usize> WithTask
    for BelkaDevice<'a, M, N, UPDATE_CHAN_SIZE>
{
    async fn task(&mut self) {
        loop {
            let update = self.update_receiver.receive().await;

            match update {
                BelkaUpdate::Reading(reading) => {
                    // Only publish if connected
                    if self.is_connected {
                        self.publish_reading(reading.endpoint, reading.value);
                    }
                    // If disconnected, ignore stale readings
                }
                BelkaUpdate::ConnectionChanged(connected) => {
                    self.update_connection_status(connected);
                }
            }
        }
    }
}

/// Status provider for Belka Portal - reports connection status
pub struct BelkaStatusProvider<'a> {
    peripheral_id: PeripheralId,
    connected_signal: &'a Signal<NoopRawMutex, bool>,
    last_status: Cell<bool>,
}

impl<'a> BelkaStatusProvider<'a> {
    pub fn new(
        peripheral_id: PeripheralId,
        connected_signal: &'a Signal<NoopRawMutex, bool>,
    ) -> Self {
        Self {
            peripheral_id,
            connected_signal,
            last_status: Cell::new(false),
        }
    }
}

impl<'a> PeripheralStatusProvider for BelkaStatusProvider<'a> {
    fn get_peripheral_id(&self) -> PeripheralId {
        self.peripheral_id
    }

    fn get_peripheral_type(&self) -> PeripheralType {
        PeripheralType::BrewSensor
    }

    fn is_available(&self) -> bool {
        // Try to take a new value from the signal, otherwise use the last known value
        if let Some(status) = self.connected_signal.try_take() {
            self.last_status.set(status);
            status
        } else {
            self.last_status.get()
        }
    }
}
