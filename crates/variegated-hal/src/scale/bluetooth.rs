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
    BatteryLevelType, ExternalPeripheralSensorReading, FlowRateType, PeripheralId,
    PeripheralStatusProvider, PeripheralType, ScaleOp, WeightType,
};

use crate::scale::{ScaleController, ScaleError};
use crate::{SensorReading, WithTask};

/// Endpoint carrying weight in grams.
///
/// This is a wire contract with the comms processor, which stamps it on every scale
/// reading it forwards, and whose `config.rs` carries the matching pair of constants.
/// The numbers must agree; nothing checks them, because the two firmwares are separate
/// binaries on separate chips.
pub const BLUETOOTH_SCALE_ENDPOINT_WEIGHT: u8 = 0;

/// Endpoint carrying gravimetric flow rate in grams per second.
///
/// Derived on the comms processor, which is where the sample timing is least corrupted --
/// differentiation is the operation that latency between samples ruins, and the UART hop
/// adds exactly that. Some scales report flow natively; this endpoint does not
/// distinguish, which is the point.
///
/// It arrives as *mass* flow, g/s, and lands in a `FlowRateType` that is nominally ml/s.
/// That is deliberate and needs no conversion: this codebase already treats coffee as
/// 1 g/ml when deriving output volume from output weight. The label is off by the density
/// of espresso, a few percent, and consistently so on both sides.
pub const BLUETOOTH_SCALE_ENDPOINT_FLOW: u8 = 1;

/// Endpoint carrying the scale's battery charge, as a percentage.
///
/// Reported by BooKoo scales, which put it in every weight frame. ACAIA's older protocol
/// does not report one, so this endpoint simply never arrives for those -- an absent
/// endpoint is how a driver says "not applicable", and the numbering matches
/// [`crate::external_sensor::belka::BELKA_ENDPOINT_BATTERY`] deliberately.
///
/// Routed to a watch only if [`BluetoothScale::with_battery_sender`] was called. Without
/// one the reading is dropped in silence rather than warned about, because for a scale
/// that reports battery on every frame at the notification rate, a warning per frame would
/// bury the log.
pub const BLUETOOTH_SCALE_ENDPOINT_BATTERY: u8 = 2;

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

    /// Output flow rate watch sender
    flow_sender: Option<WatchSender<'a, NoopRawMutex, SensorReading<FlowRateType>, N>>,

    /// Output battery level watch sender, if anything is listening
    battery_sender: Option<WatchSender<'a, NoopRawMutex, SensorReading<BatteryLevelType>, N>>,

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
        flow_sender: Option<WatchSender<'a, NoopRawMutex, SensorReading<FlowRateType>, N>>,
    ) -> Self {
        Self {
            peripheral_id,
            update_receiver,
            weight_sender,
            flow_sender,
            battery_sender: None,
            connected_signal: None,
            is_connected: false,
        }
    }

    pub fn with_connected_signal(mut self, signal: &'a Signal<NoopRawMutex, bool>) -> Self {
        self.connected_signal = Some(signal);
        self
    }

    /// Route [`BLUETOOTH_SCALE_ENDPOINT_BATTERY`] to a watch.
    ///
    /// A builder rather than a fourth argument to [`Self::new`]: only some scales report a
    /// battery at all, and threading a `None` through both espresso firmwares to say so
    /// would change two call sites to express nothing.
    pub fn with_battery_sender(
        mut self,
        sender: WatchSender<'a, NoopRawMutex, SensorReading<BatteryLevelType>, N>,
    ) -> Self {
        self.battery_sender = Some(sender);
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

    /// Push zeros on disconnect.
    ///
    /// A `Watch` holds its last value indefinitely, so without this the final readings
    /// before the link dropped would sit there being read as live. For brew-by-weight
    /// that is the dangerous failure: a shot targeting 36 g against a frozen 18 g
    /// never reaches its target and never stops. Zero is not a *correct* weight either,
    /// but it is an obviously wrong one, and it fails in the direction that ends a shot
    /// rather than prolonging it. `PeripheralStatus` carries the real answer -- that
    /// the scale is gone -- for consumers that check it.
    ///
    /// It matters more for flow than for weight. Flow is a PID process variable
    /// (`GroupBrewControlMode::OutputFlowRate`), so a frozen value does not merely display
    /// wrong -- it makes the loop chase a setpoint against a number that has stopped
    /// responding to the pump, which is the classic way to wind an integrator up.
    fn publish_zeros(&self) {
        if let Some(ref sender) = self.weight_sender {
            sender.send(SensorReading {
                raw: 0.0,
                transformed: 0.0,
            });
        }
        if let Some(ref sender) = self.flow_sender {
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
            BLUETOOTH_SCALE_ENDPOINT_FLOW => {
                if let Some(ref sender) = self.flow_sender {
                    sender.send(SensorReading {
                        raw: value,
                        transformed: value,
                    });
                }
            }
            BLUETOOTH_SCALE_ENDPOINT_BATTERY => {
                // Dropped in silence when nothing is listening, unlike the `_` arm below.
                // A BooKoo sends this on every weight frame, so warning here would emit a
                // line per notification for the entire life of the connection.
                if let Some(ref sender) = self.battery_sender {
                    sender.send(SensorReading {
                        raw: value,
                        // Clamped rather than cast bare: `as` on an out-of-range float is
                        // a saturating conversion in Rust, but a negative or NaN value
                        // would land on 0 silently, and this is a percentage that the
                        // driver has already bounds-checked. Clamping states the range.
                        transformed: value.clamp(0.0, 100.0) as BatteryLevelType,
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

    /// Drive the scale's own timer, addressed the same way [`Self::tare`] is.
    ///
    /// Whether the scale can actually do this is the driver's business, not ours: both
    /// protocols this firmware speaks have timer commands, and a peripheral associated
    /// with a driver that did not would log the op and drop it. That asymmetry is why this
    /// reports the send, not the outcome -- the link is one-way, and it always has been.
    async fn control_timer(
        &mut self,
        command: variegated_controller_types::ScaleTimerCommand,
    ) -> Result<(), ScaleError> {
        use variegated_controller_types::ScaleTimerCommand;
        let op = match command {
            ScaleTimerCommand::Start => ScaleOp::StartTimer,
            ScaleTimerCommand::Stop => ScaleOp::StopTimer,
            ScaleTimerCommand::Reset => ScaleOp::ResetTimer,
            ScaleTimerCommand::TareAndStart => ScaleOp::TareAndStartTimer,
        };
        self.command_sender
            .try_send((self.peripheral_id, op))
            .map_err(|_| ScaleError::CommunicationError)
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

    /// Both calibrations false: a Bluetooth scale is calibrated by its own vendor app, and
    /// this firmware has no command that would change that. `support_calibration` on
    /// the peripheral's `MachineDefinition` entry should agree.
    ///
    /// `timer` is true, and is the one thing here that is not a property of Bluetooth as
    /// such: it is true because both protocols this firmware speaks happen to have timer
    /// commands. A third driver without one would make this a per-driver answer, which
    /// this type cannot currently express -- it is constructed from the controller, and
    /// the controller does not know which driver its peripheral was associated with.
    fn get_capabilities(&self) -> crate::scale::ScaleCapabilities {
        crate::scale::ScaleCapabilities {
            zero_calibration: false,
            reference_weight_calibration: false,
            supported_reference_weights: &[],
            timer: true,
        }
    }
}
