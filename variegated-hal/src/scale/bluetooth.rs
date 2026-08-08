//! A scale reached over Bluetooth, via the comms processor.
//!
//! This is the scale counterpart to [`crate::external_sensor::belka`]: the radio, the
//! GATT client and the vendor protocol all live on the comms processor, and what
//! arrives here is an already-decoded [`ExternalPeripheralSensorReading`] in grams.
//! Nothing in this module knows or cares which make of scale produced it, so a second
//! driver on the comms side reaches this code unchanged.
//!
//! It differs from [`super::gravity`] -- the other `ScaleController` -- in where the
//! work happens. `GravityDevice` owns an I2C bus and *polls* it, so it carries retry
//! backoff, conversion parameters and a poll delay. This device owns nothing and is
//! purely event driven: readings arrive on a channel, and connection state is told to
//! it rather than discovered.
//!
//! # One instance per scale
//!
//! Every part of this module takes a [`PeripheralId`] at construction rather than
//! hard-coding one, because a machine can carry several scales at once -- one under
//! each group plus a dose scale on the bench. They share the wire format and differ
//! only in that id, so a second scale is a second instance, not a second module.

use alloc::boxed::Box;
use core::cell::Cell;
use async_trait::async_trait;
use defmt::Format;
use variegated_log::{log_info, log_warn};
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, RawMutex};
use embassy_sync::channel::{Receiver, Sender as ChannelSender};
use embassy_sync::signal::Signal;
use embassy_sync::watch::Sender as WatchSender;
use variegated_controller_types::debug::{name, DebugEvent};
use variegated_controller_types::{
    ExternalPeripheralSensorReading, PeripheralId, PeripheralStatusProvider, PeripheralType,
    ScaleOp, WeightType,
};

use crate::scale::{ScaleController, ScaleError};
use crate::{SensorReading, WithTask};

/// Endpoint carrying weight in grams.
///
/// This is a wire contract with the comms processor, which stamps it on every scale
/// reading it forwards. Scales have no second quantity today; the endpoint exists so
/// that adding one (flow rate, battery) does not require a new message type.
pub const BLUETOOTH_SCALE_ENDPOINT_WEIGHT: u8 = 0;

/// Message type for Bluetooth scale updates from the comms layer
#[derive(Clone, Debug, Format)]
pub enum BluetoothScaleUpdate {
    /// A sensor reading from the scale
    Reading(ExternalPeripheralSensorReading),
    /// Connection status changed
    ConnectionChanged(bool),
}

/// A Bluetooth scale - runs as an async task, receives updates from comms
pub struct BluetoothScale<'a, M: RawMutex, const N: usize, const UPDATE_CHAN_SIZE: usize> {
    /// Peripheral ID for this scale
    peripheral_id: PeripheralId,

    /// Channel receiving updates from comms layer
    update_receiver: Receiver<'a, M, BluetoothScaleUpdate, UPDATE_CHAN_SIZE>,

    /// Output weight watch sender
    weight_sender: Option<WatchSender<'a, NoopRawMutex, SensorReading<WeightType>, N>>,

    /// Signal for connection status (used by status provider)
    connected_signal: Option<&'a Signal<NoopRawMutex, bool>>,

    /// Current connection state
    is_connected: bool,
}

impl<'a, M: RawMutex, const N: usize, const UPDATE_CHAN_SIZE: usize>
    BluetoothScale<'a, M, N, UPDATE_CHAN_SIZE>
{
    pub fn new(
        peripheral_id: PeripheralId,
        update_receiver: Receiver<'a, M, BluetoothScaleUpdate, UPDATE_CHAN_SIZE>,
        weight_sender: Option<WatchSender<'a, NoopRawMutex, SensorReading<WeightType>, N>>,
    ) -> Self {
        Self {
            peripheral_id,
            update_receiver,
            weight_sender,
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
                log_info!("Bluetooth scale connected (id: 0x{:04X})", self.peripheral_id);
            } else {
                variegated_log::emit_event(DebugEvent::SensorFault {
                    sensor: name("bluetooth_scale"),
                });
                self.publish_zeros();
            }
        }
    }

    /// Push a zero weight on disconnect.
    ///
    /// A `Watch` holds its last value indefinitely, so without this the final weight
    /// before the link dropped would sit there being read as live. For brew-by-weight
    /// that is the dangerous failure: a shot targeting 36 g against a frozen 18 g
    /// never reaches its target and never stops. Zero is not a *correct* weight either,
    /// but it is an obviously wrong one, and it fails in the direction that ends a shot
    /// rather than prolonging it. `PeripheralStatus` carries the real answer -- that
    /// the scale is gone -- for consumers that check it.
    fn publish_zeros(&self) {
        if let Some(ref sender) = self.weight_sender {
            sender.send(SensorReading {
                raw: 0.0,
                transformed: 0.0,
            });
        }
    }

    /// Publish a reading.
    ///
    /// `raw == transformed`, with no [`variegated_adc_tools::ConversionParameters`] --
    /// unlike `GravityDevice`, which reads milligram counts off an ADC and needs a
    /// conversion. The comms processor sends grams, already in engineering units, so a
    /// conversion here would be an identity transform with somewhere for a wrong
    /// coefficient to hide. This follows `BelkaDevice`, which takes the same view.
    fn publish_reading(&self, endpoint: u8, value: f32) {
        match endpoint {
            BLUETOOTH_SCALE_ENDPOINT_WEIGHT => {
                if let Some(ref sender) = self.weight_sender {
                    sender.send(SensorReading {
                        raw: value,
                        transformed: value,
                    });
                }
            }
            _ => {
                log_warn!("Unknown Bluetooth scale endpoint: {}", endpoint);
            }
        }
    }
}

impl<'a, M: RawMutex, const N: usize, const UPDATE_CHAN_SIZE: usize> WithTask
    for BluetoothScale<'a, M, N, UPDATE_CHAN_SIZE>
{
    async fn task(&mut self) {
        loop {
            let update = self.update_receiver.receive().await;

            match update {
                BluetoothScaleUpdate::Reading(reading) => {
                    // Only publish if connected -- a reading that arrives before the
                    // comms processor has reported the link up is one this device
                    // cannot vouch for.
                    if self.is_connected {
                        self.publish_reading(reading.endpoint, reading.value);
                    }
                }
                BluetoothScaleUpdate::ConnectionChanged(connected) => {
                    self.update_connection_status(connected);
                }
            }
        }
    }
}

/// Status provider for a Bluetooth scale - reports connection status
pub struct BluetoothScaleStatusProvider<'a> {
    peripheral_id: PeripheralId,
    connected_signal: &'a Signal<NoopRawMutex, bool>,
    last_status: Cell<bool>,
}

impl<'a> BluetoothScaleStatusProvider<'a> {
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

impl<'a> PeripheralStatusProvider for BluetoothScaleStatusProvider<'a> {
    fn get_peripheral_id(&self) -> PeripheralId {
        self.peripheral_id
    }

    fn get_peripheral_type(&self) -> PeripheralType {
        PeripheralType::Scale
    }

    /// The `Cell` is load bearing, not a cache.
    ///
    /// `Signal::try_take()` *consumes* the value, and `PeripheralRegistry` polls this
    /// on every status build, so without the latch every call after the first would
    /// read `false` and the scale would flicker out of the status map at 1 Hz.
    fn is_available(&self) -> bool {
        if let Some(status) = self.connected_signal.try_take() {
            self.last_status.set(status);
            status
        } else {
            self.last_status.get()
        }
    }
}

/// The [`ScaleController`] half: turns `Group`'s scale operations into commands bound
/// for the comms processor.
///
/// The channel carries `(PeripheralId, ScaleOp)` -- the exact payload of
/// `ApplicationProcessorToCommsProcessorMessage::ScaleCommand` -- so the comms task
/// that drains it only has to wrap, never translate. Both types come from
/// `variegated-controller-types` rather than from this crate, because
/// `variegated-comms` does not depend on `variegated-hal` and a type defined here
/// could not appear in its signature.
pub struct BluetoothScaleController<'a, M: RawMutex, const N: usize> {
    peripheral_id: PeripheralId,
    command_sender: ChannelSender<'a, M, (PeripheralId, ScaleOp), N>,
}

impl<'a, M: RawMutex, const N: usize> BluetoothScaleController<'a, M, N> {
    pub fn new(
        peripheral_id: PeripheralId,
        command_sender: ChannelSender<'a, M, (PeripheralId, ScaleOp), N>,
    ) -> Self {
        BluetoothScaleController {
            peripheral_id,
            command_sender,
        }
    }
}

// `M: RawMutex + Sync` rather than plain `RawMutex`, because `#[async_trait]` boxes
// every method's future as `dyn Future + Send` -- the same bound `GravityController`
// carries for the same reason.
#[async_trait]
impl<'a, M: RawMutex + Sync, const N: usize> ScaleController
    for BluetoothScaleController<'a, M, N>
{
    /// Stamp this controller's own id on the command.
    ///
    /// `tare()` takes no target -- `Group` knows only "my scale" and has no idea which
    /// BLE peripheral that is. The id handed to `new()` is what makes the command
    /// addressable, and it is why a machine with two group scales needs two of these
    /// rather than one shared instance.
    async fn tare(&mut self) -> Result<(), ScaleError> {
        self.command_sender
            .try_send((self.peripheral_id, ScaleOp::Tare))
            .map_err(|_| ScaleError::TareFailed)
    }

    /// Not supported: the ACAIA protocol has no settings this maps onto.
    ///
    /// Reported honestly rather than swallowed as `Ok(())`, even though every caller
    /// in the tree discards the result with `let _ =`. A silent success here would
    /// make a future caller that *does* check believe the scale had been configured.
    async fn set_configuration(
        &mut self,
        _configuration: &crate::scale::ScaleConfiguration,
    ) -> Result<(), ScaleError> {
        Err(ScaleError::UnsupportedConfiguration)
    }

    fn get_supported_configuration(&mut self) -> crate::scale::SupportedConfigurationOptions {
        crate::scale::SupportedConfigurationOptions {
            zero_tracking: false,
            smoothing: false,
        }
    }

    async fn zero_calibration(&mut self) -> Result<(), ScaleError> {
        Err(ScaleError::CalibrationNotSupported)
    }

    async fn reference_weight_calibration(
        &mut self,
        _weight_grams: u32,
    ) -> Result<(), ScaleError> {
        Err(ScaleError::CalibrationNotSupported)
    }

    /// Everything false: a Bluetooth scale is calibrated by its own vendor app, and
    /// this firmware has no command that would change that. `support_calibration` on
    /// the peripheral's `MachineDefinition` entry should agree.
    fn get_capabilities(&self) -> crate::scale::ScaleCapabilities {
        crate::scale::ScaleCapabilities {
            zero_calibration: false,
            reference_weight_calibration: false,
            supported_reference_weights: &[],
        }
    }
}
