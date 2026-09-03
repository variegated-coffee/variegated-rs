#![no_std]

extern crate alloc;

use alloc::boxed::Box;
use async_trait::async_trait;
use defmt::Format;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::watch::Receiver;
pub use variegated_controller_types::{DutyCycleType, FlowRateType, HexadecimalDutyCycleType, InputVolumeType, MixingProportionType, PressureType, TemperatureType, ValveOpenType, WaterLevelType};

/// Mutex type for cross-core and cross-executor synchronization
///
/// Use this mutex type when you need to share data between:
/// - Different CPU cores (e.g., core 0 and core 1 on RP2350)
/// - Different embassy executors running on the same or different cores
/// - Interrupt handlers and async tasks
///
/// **When NOT to use:** For single-core, single-executor scenarios, use
/// `NoopRawMutex` instead for better performance (zero overhead).
///
/// # Feature Flags
///
/// - **With `atomic-mutex` feature (RP2350 only):** Uses hardware atomic
///   operations via `AtomicRawMutex` for lower overhead cross-core synchronization.
///   This is the most efficient option for RP2350 chips.
///
/// - **Without `atomic-mutex` feature (default):** Uses `CriticalSectionRawMutex`
///   which disables interrupts. This works on all platforms (RP2040, RP2350, etc.)
///   but has slightly higher overhead.
///
/// # Example
///
/// ```rust,ignore
/// // For cross-core communication (e.g., heating element controlled from one core,
/// // commanded from another):
/// static SIGNAL: StaticCell<Signal<SyncSendRawMutex, DutyCycleType>> = StaticCell::new();
///
/// // For single-core local state (no sharing between cores):
/// static LOCAL: StaticCell<Signal<NoopRawMutex, DutyCycleType>> = StaticCell::new();
/// ```
#[cfg(feature = "atomic-mutex")]
pub type SyncSendRawMutex = variegated_rp235x_atomic_raw_mutex::AtomicRawMutex;

/// Mutex type for cross-core and cross-executor synchronization
///
/// See documentation above for details on when to use this vs `NoopRawMutex`.
#[cfg(not(feature = "atomic-mutex"))]
pub type SyncSendRawMutex = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

#[derive(Clone, Debug, Format)]
pub struct SensorReading<Transformed> {
    pub raw: f32,
    pub transformed: Transformed,
}
pub use pump::{Pump, PumpError};
use heapless::index_map::FnvIndexMap;
use variegated_controller_types::{WeightType, PeripheralStatus, PeripheralStatusProvider, PeripheralId, PeripheralInfo, MAX_PERIPHERALS, ECType, RPMType};
use crate::scale::ScaleConfiguration;

pub mod gpio;
pub mod adc;
pub mod cap_adc;
pub mod machine_mechanism;
pub mod scale;
pub mod noop;
pub mod pump;
pub mod heating_element;
pub mod external_sensor;
// RP2350-only: the QMI window and the PSRAM driver are `embassy-rp`'s, and only the
// rp235x variants have external-memory support.
#[cfg(feature = "rp235x")]
pub mod heap;

#[derive(Debug, Format)]
pub enum BoilerFillMechanismError {
    InterlockError,
    UnknownError,
}

#[derive(Debug, Format)]
pub enum GroupPumpError {
    InterlockError,
    UnknownError,
}

#[derive(Debug, Format)]
pub enum BrewMechanismError {
    InterlockError,
    NotSupportedError,
    UnknownError,
}

#[derive(Debug, Format)]
pub enum SteamMechanismError {
    InterlockError,
    NotSupportedError,
    UnknownError,
}

#[derive(Debug, Format)]
pub enum ValveMechanismError {
    InterlockError,
    NotSupportedError,
    UnknownError,
}

#[derive(Debug, Format)]
pub enum WaterMixerMechanismError {
    InterlockError,
    NotSupportedError,
    UnknownError,
}

#[derive(Debug, Format)]
pub enum WaterTapMechanismError {
    InterlockError,
    NotSupportedError,
    UnknownError,
}

pub struct Boiler<'a, M: RawMutex, const N: usize> {
    pub heating_element: Box<dyn HeatingElement>,
    pub fill_mechanism: Option<Box<dyn BoilerFillMechanism>>,
    pub temperature_sensor: Option<Receiver<'a, M, SensorReading<TemperatureType>, N>>,
    pub pressure_sensor: Option<Receiver<'a, M, SensorReading<PressureType>, N>>,
    pub water_level_sensor: Option<Receiver<'a, M, SensorReading<WaterLevelType>, N>>,
}

impl<'a, M: RawMutex, const N: usize> Boiler<'a, M, N> {
    pub fn new(
        heating_element: Box<dyn HeatingElement>,
        fill_mechanism: Option<Box<dyn BoilerFillMechanism>>,
        temperature_sensor: Option<Receiver<'a, M, SensorReading<TemperatureType>, N>>,
        pressure_sensor: Option<Receiver<'a, M, SensorReading<PressureType>, N>>,
        water_level_sensor: Option<Receiver<'a, M, SensorReading<WaterLevelType>, N>>,
    ) -> Self {
        Self {
            heating_element,
            fill_mechanism,
            temperature_sensor,
            pressure_sensor,
            water_level_sensor
        }
    }

    pub async fn set_heating_element_duty_cycle(&mut self, duty_cycle_percent: DutyCycleType) {

        self.heating_element.set_duty_cycle(duty_cycle_percent).await;
    }
    
    pub async fn get_heating_element_duty_cycle(&self) -> DutyCycleType {
        self.heating_element.get_duty_cycle().await
    }

    pub fn get_temperature(&mut self) -> Option<TemperatureType> {
        if let Some(temperature_sensor) = &mut self.temperature_sensor {
            temperature_sensor.try_get().map(|reading| reading.transformed)
        } else {
            None
        }
    }

    pub fn get_temperature_reading(&mut self) -> Option<SensorReading<TemperatureType>> {
        if let Some(temperature_sensor) = &mut self.temperature_sensor {
            temperature_sensor.try_get()
        } else {
            None
        }
    }

    pub fn get_pressure(&mut self) -> Option<PressureType> {
        if let Some(pressure_sensor) = &mut self.pressure_sensor {
            pressure_sensor.try_get().map(|reading| reading.transformed)
        } else {
            None
        }
    }

    pub fn get_pressure_reading(&mut self) -> Option<SensorReading<PressureType>> {
        if let Some(pressure_sensor) = &mut self.pressure_sensor {
            pressure_sensor.try_get()
        } else {
            None
        }
    }

    pub fn get_water_level(&mut self) -> Option<WaterLevelType> {
        if let Some(water_level_sensor) = &mut self.water_level_sensor {
            water_level_sensor.try_get().map(|reading| reading.transformed)
        } else {
            None
        }
    }

    pub fn get_water_level_reading(&mut self) -> Option<SensorReading<WaterLevelType>> {
        if let Some(water_level_sensor) = &mut self.water_level_sensor {
            water_level_sensor.try_get()
        } else {
            None
        }
    }
}

pub struct Group<'a, M: RawMutex, const N: usize> {
    pub brew_mechanism: Option<Box<dyn BrewMechanism>>,
    pub heating_element: Option<Box<dyn HeatingElement>>,
    pub scale_controller: Option<Box<dyn scale::ScaleController>>,
    pub temperature_sensor: Option<Receiver<'a, M, SensorReading<TemperatureType>, N>>,
    pub pressure_sensor: Option<Receiver<'a, M, SensorReading<PressureType>, N>>,
    pub input_flow_sensor: Option<Receiver<'a, M, SensorReading<FlowRateType>, N>>,
    pub input_volume_sensor: Option<Receiver<'a, M, SensorReading<InputVolumeType>, N>>,
    pub output_flow_sensor: Option<Receiver<'a, M, SensorReading<FlowRateType>, N>>,
    pub output_weight_sensor: Option<Receiver<'a, M, SensorReading<WeightType>, N>>,
    pub output_temperature_sensor: Option<Receiver<'a, M, SensorReading<TemperatureType>, N>>,
    pub output_electrical_conductivity_sensor: Option<Receiver<'a, M, SensorReading<ECType>, N>>,
    /// Pump speed from a tachometer, on machines that have one.
    ///
    /// Group-scoped rather than machine-scoped because the pump serves the group, and
    /// because this is the level `GroupStatus` is assembled at. A dual-group machine with
    /// one pump per group would want it here too.
    pub pump_rpm_sensor: Option<Receiver<'a, M, SensorReading<RPMType>, N>>
}

impl<'a, M: RawMutex, const N: usize> Group<'a, M, N> {
    pub fn new(
        brew_mechanism: Option<Box<dyn BrewMechanism>>,
        heating_element: Option<Box<dyn HeatingElement>>,
        scale_controller: Option<Box<dyn scale::ScaleController>>,
        temperature_sensor: Option<Receiver<'a, M, SensorReading<TemperatureType>, N>>,
        pressure_sensor: Option<Receiver<'a, M, SensorReading<PressureType>, N>>,
        input_flow_sensor: Option<Receiver<'a, M, SensorReading<FlowRateType>, N>>,
        input_volume_sensor: Option<Receiver<'a, M, SensorReading<InputVolumeType>, N>>,
        output_flow_sensor: Option<Receiver<'a, M, SensorReading<FlowRateType>, N>>,
        output_weight_sensor: Option<Receiver<'a, M, SensorReading<WeightType>, N>>,
        output_temperature_sensor: Option<Receiver<'a, M, SensorReading<TemperatureType>, N>>,
        output_electrical_conductivity_sensor: Option<Receiver<'a, M, SensorReading<ECType>, N>>,
        pump_rpm_sensor: Option<Receiver<'a, M, SensorReading<RPMType>, N>>
    ) -> Self {
        Self {
            brew_mechanism,
            heating_element,
            scale_controller,
            temperature_sensor,
            pressure_sensor,
            input_flow_sensor,
            input_volume_sensor,
            output_flow_sensor,
            output_weight_sensor,
            output_temperature_sensor,
            output_electrical_conductivity_sensor,
            pump_rpm_sensor,
        }
    }

    pub async fn set_brewing_state(&mut self, brewing: bool, duty_cycle: HexadecimalDutyCycleType) {
        if let Some(brew_mechanism) = &mut self.brew_mechanism {
            brew_mechanism.set_state(brewing, duty_cycle).await.expect("Failed to set brewing state");
        }
    }

    pub fn get_brew_state(&self) -> bool {
        if let Some(brew_mechanism) = &self.brew_mechanism {
            brew_mechanism.get_brew_state()
        } else {
            false
        }
    }
    
    pub fn get_pump_duty_cycle(&self) -> Option<HexadecimalDutyCycleType> {
        if let Some(brew_mechanism) = &self.brew_mechanism {
            brew_mechanism.get_pump_duty_cycle()
        } else {
            None
        }
    }

    pub fn get_three_way_valve_open(&self) -> Option<bool> {
        if let Some(brew_mechanism) = &self.brew_mechanism {
            brew_mechanism.get_three_way_valve_open()
        } else {
            None
        }
    }

    pub fn get_temperature(&mut self) -> Option<TemperatureType> {
        self.temperature_sensor.as_mut().and_then(|sensor| sensor.try_get().map(|reading| reading.transformed))
    }

    pub fn get_temperature_reading(&mut self) -> Option<SensorReading<TemperatureType>> {
        self.temperature_sensor.as_mut().and_then(|sensor| sensor.try_get())
    }

    pub fn get_pressure(&mut self) -> Option<PressureType> {
        self.pressure_sensor.as_mut().and_then(|sensor| sensor.try_get().map(|reading| reading.transformed))
    }

    pub fn get_pressure_reading(&mut self) -> Option<SensorReading<PressureType>> {
        self.pressure_sensor.as_mut().and_then(|sensor| sensor.try_get())
    }

    pub fn get_input_flow_rate(&mut self) -> Option<FlowRateType> {
        self.input_flow_sensor.as_mut().and_then(|sensor| sensor.try_get().map(|reading| reading.transformed))
    }

    pub fn get_input_flow_reading(&mut self) -> Option<SensorReading<FlowRateType>> {
        self.input_flow_sensor.as_mut().and_then(|sensor| sensor.try_get())
    }

    pub fn get_input_volume(&mut self) -> Option<InputVolumeType> {
        self.input_volume_sensor.as_mut().and_then(|sensor| sensor.try_get().map(|reading| reading.transformed))
    }

    pub fn get_input_volume_reading(&mut self) -> Option<SensorReading<InputVolumeType>> {
        self.input_volume_sensor.as_mut().and_then(|sensor| sensor.try_get())
    }

    pub fn get_output_flow_rate(&mut self) -> Option<FlowRateType> {
        self.output_flow_sensor.as_mut().and_then(|sensor| sensor.try_get().map(|reading| reading.transformed))
    }

    pub fn get_output_flow_reading(&mut self) -> Option<SensorReading<FlowRateType>> {
        self.output_flow_sensor.as_mut().and_then(|sensor| sensor.try_get())
    }

    pub fn get_output_weight(&mut self) -> Option<WeightType> {
        self.output_weight_sensor.as_mut().and_then(|sensor| sensor.try_get().map(|reading| reading.transformed))
    }

    pub fn get_output_weight_reading(&mut self) -> Option<SensorReading<WeightType>> {
        self.output_weight_sensor.as_mut().and_then(|sensor| sensor.try_get())
    }

    pub fn get_output_temperature(&mut self) -> Option<TemperatureType> {
        self.output_temperature_sensor
            .as_mut()
            .and_then(|sensor| sensor.try_get().map(|reading| reading.transformed))
    }

    pub fn get_output_temperature_reading(&mut self) -> Option<SensorReading<TemperatureType>> {
        self.output_temperature_sensor.as_mut().and_then(|sensor| sensor.try_get())
    }

    pub fn get_output_electrical_conductivity(&mut self) -> Option<ECType> {
        self.output_electrical_conductivity_sensor
            .as_mut()
            .and_then(|sensor| sensor.try_get().map(|reading| reading.transformed))
    }

    pub fn get_output_electrical_conductivity_reading(&mut self) -> Option<SensorReading<ECType>> {
        self.output_electrical_conductivity_sensor.as_mut().and_then(|sensor| sensor.try_get())
    }

    pub fn get_pump_rpm(&mut self) -> Option<RPMType> {
        self.pump_rpm_sensor.as_mut().and_then(|sensor| sensor.try_get().map(|reading| reading.transformed))
    }

    pub fn get_pump_rpm_reading(&mut self) -> Option<SensorReading<RPMType>> {
        self.pump_rpm_sensor.as_mut().and_then(|sensor| sensor.try_get())
    }

    pub async fn scale_tare(&mut self) -> Result<(), scale::ScaleError> {
        if let Some(scale_controller) = &mut self.scale_controller {
            scale_controller.tare().await
        } else {
            Ok(())
        }
    }

    /// Do whatever the configuration says happens to the scale when a brew starts.
    ///
    /// **Here rather than in each controller.** Both of them opened a brew with the same two
    /// calls in the same order -- `scale_set_configuration` then an unconditional
    /// `scale_tare` -- and turning that into a configurable set at both call sites would have
    /// written the branch twice. This crate's own README makes the point that the two
    /// firmwares have forked badly and that most of one's substantive lines appear verbatim
    /// in the other; a new branch copied into both is that, starting again.
    ///
    /// Order is tare, then reset, then start, and the timer's two commands are sent
    /// separately rather than as [`ScaleTimerCommand::TareAndStart`]. That variant looks like
    /// the obvious shortcut and is not: it does not reset, which is half of what the setting
    /// promises, and ACAIA has no such command anyway -- the BLE slot already synthesizes it
    /// as two writes.
    ///
    /// Failures are dropped, like the `let _ =` this replaces. A scale that will not tare is
    /// not a reason to refuse a shot, and the operator can see the weight on the panel.
    pub async fn apply_brew_actions(
        &mut self,
        actions: variegated_controller_types::BrewActions,
    ) {
        use variegated_controller_types::ScaleTimerCommand;

        if actions.tare() {
            let _ = self.scale_tare().await;
        }
        if actions.reset_and_start_timer() {
            let _ = self.scale_control_timer(ScaleTimerCommand::Reset).await;
            let _ = self.scale_control_timer(ScaleTimerCommand::Start).await;
        }
    }

    /// Drive the scale's own timer.
    ///
    /// `Ok(())` with no controller, matching every other method here. Note what that
    /// means: a group with no scale reports success, so this cannot be used to discover
    /// whether a timer exists. `ScaleCapabilities::timer` is the question to ask -- and it
    /// is the same trap `scale_calibration`'s module docs describe for the two calibration
    /// methods.
    pub async fn scale_control_timer(
        &mut self,
        command: variegated_controller_types::ScaleTimerCommand,
    ) -> Result<(), scale::ScaleError> {
        if let Some(scale_controller) = &mut self.scale_controller {
            scale_controller.control_timer(command).await
        } else {
            Ok(())
        }
    }
    
    pub async fn scale_set_configuration(&mut self, config: ScaleConfiguration) -> Result<(), scale::ScaleError> {
        if let Some(scale_controller) = &mut self.scale_controller {
            scale_controller.set_configuration(&config).await
        } else {
            Ok(())
        }
    }
    
    pub async fn scale_zero_calibration(&mut self) -> Result<(), scale::ScaleError> {
        if let Some(scale_controller) = &mut self.scale_controller {
            scale_controller.zero_calibration().await
        } else {
            Ok(())
        }
    }
    
    pub async fn scale_reference_weight_calibration(&mut self, weight_grams: u32) -> Result<(), scale::ScaleError> {
        if let Some(scale_controller) = &mut self.scale_controller {
            scale_controller.reference_weight_calibration(weight_grams).await
        } else {
            Ok(())
        }
    }
}

pub struct SteamWand {
    pub valve_mechanism: Option<Box<dyn ValveMechanism>>,
    steaming: bool,
    configured_valve_openness: ValveOpenType,
}

impl SteamWand {
    pub fn new(valve_mechanism: Option<Box<dyn ValveMechanism>>) -> Self {
        Self {
            valve_mechanism,
            steaming: false,
            configured_valve_openness: 100,
        }
    }

    pub fn set_steaming_state(&mut self, steaming: bool) -> Result<(), ValveMechanismError> {
        self.steaming = steaming;
        if let Some(valve) = &mut self.valve_mechanism {
            let openness = if steaming { self.configured_valve_openness } else { 0 };
            valve.set_valve_state(openness)?;
        }
        Ok(())
    }

    pub fn set_steam_valve_openness(&mut self, openness: ValveOpenType) -> Result<(), ValveMechanismError> {
        self.configured_valve_openness = openness;
        // If currently steaming, apply the new openness immediately
        if self.steaming {
            if let Some(valve) = &mut self.valve_mechanism {
                valve.set_valve_state(openness)?;
            }
        }
        Ok(())
    }

    pub fn get_steaming_state(&self) -> bool {
        self.steaming
    }

    pub fn get_steam_valve_openness(&self) -> ValveOpenType {
        self.configured_valve_openness
    }
}

pub struct WaterTap<'a, M: RawMutex, const N: usize> {
    pub water_tap_mechanism: Option<Box<dyn WaterTapMechanism>>,
    pub valve_mechanism: Option<Box<dyn ValveMechanism>>,
    pub mixer: Option<Box<dyn WaterMixerMechanism>>,
    pub temperature_sensor: Option<Receiver<'a, M, SensorReading<TemperatureType>, N>>,
    pub flow_sensor: Option<Receiver<'a, M, SensorReading<FlowRateType>, N>>,
}

pub struct Tank<'a, M: RawMutex, const N: usize> {
    pub water_level_sensor: Option<Receiver<'a, M, SensorReading<WaterLevelType>, N>>,
}

impl<'a, M: RawMutex, const N: usize> WaterTap<'a, M, N> {
    pub fn new(
        water_tap_mechanism: Option<Box<dyn WaterTapMechanism>>,
        valve_mechanism: Option<Box<dyn ValveMechanism>>,
        mixer: Option<Box<dyn WaterMixerMechanism>>,
        temperature_sensor: Option<Receiver<'a, M, SensorReading<TemperatureType>, N>>,
        flow_sensor: Option<Receiver<'a, M, SensorReading<FlowRateType>, N>>,
    ) -> Self {
        Self {
            water_tap_mechanism,
            valve_mechanism,
            mixer,
            temperature_sensor,
            flow_sensor,
        }
    }

    pub async fn set_water_dispensing_state(&mut self, dispensing: bool, duty_cycle: HexadecimalDutyCycleType) {
        if let Some(water_tap_mechanism) = &mut self.water_tap_mechanism {
            water_tap_mechanism.set_state(dispensing, duty_cycle).await.expect("Failed to set water dispensing state");
        }
    }

    pub fn get_dispensing_state(&self) -> bool {
        if let Some(water_tap_mechanism) = &self.water_tap_mechanism {
            water_tap_mechanism.get_dispensing_state()
        } else {
            false
        }
    }

    pub fn get_pump_duty_cycle(&self) -> Option<HexadecimalDutyCycleType> {
        if let Some(water_tap_mechanism) = &self.water_tap_mechanism {
            water_tap_mechanism.get_pump_duty_cycle()
        } else {
            None
        }
    }

    pub fn get_temperature(&mut self) -> Option<TemperatureType> {
        self.temperature_sensor.as_mut().and_then(|sensor| sensor.try_get().map(|reading| reading.transformed))
    }

    pub fn get_temperature_reading(&mut self) -> Option<SensorReading<TemperatureType>> {
        self.temperature_sensor.as_mut().and_then(|sensor| sensor.try_get())
    }

    pub fn get_flow_rate(&mut self) -> Option<FlowRateType> {
        self.flow_sensor.as_mut().and_then(|sensor| sensor.try_get().map(|reading| reading.transformed))
    }

    pub fn get_flow_reading(&mut self) -> Option<SensorReading<FlowRateType>> {
        self.flow_sensor.as_mut().and_then(|sensor| sensor.try_get())
    }
}

impl<'a, M: RawMutex, const N: usize> Tank<'a, M, N> {
    pub fn new(water_level_sensor: Option<Receiver<'a, M, SensorReading<WaterLevelType>, N>>) -> Self {
        Self { water_level_sensor }
    }

    pub fn get_water_level(&mut self) -> Option<WaterLevelType> {
        if let Some(water_level_sensor) = &mut self.water_level_sensor {
            water_level_sensor.try_get().map(|reading| reading.transformed)
        } else {
            None
        }
    }

    pub fn get_water_level_reading(&mut self) -> Option<SensorReading<WaterLevelType>> {
        if let Some(water_level_sensor) = &mut self.water_level_sensor {
            water_level_sensor.try_get()
        } else {
            None
        }
    }
}

pub trait WithTask {
    #[allow(async_fn_in_trait)]
    async fn task(&mut self);
}

/// A heating element, driven as a percentage.
///
/// Percent, unlike the pump -- see [`Pump`](crate::pump::Pump). The actuation here is a
/// 3-second soft-PWM cycle, where one percent is 30 ms, so there is no finer resolution to
/// be had and nothing to gain from the pump's scale. Typing it as a [`DutyCycleType`]
/// rather than a bare `u8` is what stops a pump value reaching a heating element.
#[async_trait]
pub trait HeatingElement {
    async fn set_duty_cycle(&mut self, duty_cycle_percent: DutyCycleType);
    async fn get_duty_cycle(&self) -> DutyCycleType;
}

pub trait BoilerFillMechanism {
    fn set_filling_state(&mut self, state: bool) -> Result<(), BoilerFillMechanismError>;
    fn get_filling_state(&self) -> Result<bool, BoilerFillMechanismError>;
}

/// Everything from here down to the pump is on the pump's 0-255 scale.
///
/// The conversion from the operator's percentage happens once, in the controller, on the
/// way in. Converting here instead would round-trip through 100 steps and throw away the
/// resolution the scale exists to provide.
#[async_trait]
pub trait BrewMechanism {
    // Controls brewing state and pump duty cycle in a single operation
    async fn set_state(&mut self, brewing: bool, duty_cycle: HexadecimalDutyCycleType) -> Result<(), BrewMechanismError>;

    fn get_pump_duty_cycle(&self) -> Option<HexadecimalDutyCycleType>;
    fn get_brew_state(&self) -> bool;
    fn get_three_way_valve_open(&self) -> Option<bool>;
}

#[async_trait]
pub trait WaterTapMechanism {
    // Controls water dispensing state and pump duty cycle in a single operation
    async fn set_state(&mut self, dispensing: bool, duty_cycle: HexadecimalDutyCycleType) -> Result<(), WaterTapMechanismError>;

    fn get_pump_duty_cycle(&self) -> Option<HexadecimalDutyCycleType>;
    fn get_dispensing_state(&self) -> bool;
}

pub trait ValveMechanism {
    fn set_valve_state(&mut self, state: ValveOpenType) -> Result<(), ValveMechanismError>;
    fn get_valve_state(&self) -> ValveOpenType;
    fn get_binary_state(&self) -> bool;
}

pub trait WaterMixerMechanism {
    fn set_mixing_proportions(&mut self, hot_percent: MixingProportionType) -> Result<(), WaterMixerMechanismError>;
}

pub struct PeripheralRegistry<'a> {
    providers: FnvIndexMap<PeripheralId, &'a dyn PeripheralStatusProvider, MAX_PERIPHERALS>,
}

impl<'a> PeripheralRegistry<'a> {
    pub fn new() -> Self {
        Self {
            providers: FnvIndexMap::new(),
        }
    }

    pub fn register(&mut self, provider: &'a dyn PeripheralStatusProvider) {
        let id = provider.get_peripheral_id();
        let _ = self.providers.insert(id, provider);
    }

    pub fn get_peripheral_status(&self) -> PeripheralStatus {
        let mut status = PeripheralStatus::default();

        for (id, provider) in &self.providers {
            let _ = status.peripherals.insert(
                *id,
                PeripheralInfo {
                    peripheral_type: provider.get_peripheral_type(),
                    is_available: provider.is_available(),
                },
            );
        }

        status
    }
}