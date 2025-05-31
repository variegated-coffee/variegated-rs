#![no_std]

extern crate alloc;

use alloc::boxed::Box;
use async_trait::async_trait;
use defmt::Format;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::signal::Signal;
use embassy_sync::watch::Receiver;
pub use variegated_controller_types::{DutyCycleType, FlowRateType, MixingProportionType, PressureType, TemperatureType, ValveOpenType, WaterLevelType};

pub mod gpio;
pub mod adc;
pub mod machine_mechanism;



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

pub struct Boiler<'a, M: RawMutex, const N: usize> {
    pub heating_element: Box<dyn HeatingElement>,
    pub fill_mechanism: Option<Box<dyn BoilerFillMechanism>>,
    pub temperature_sensor: Option<Receiver<'a, M, TemperatureType, N>>,
    pub pressure_sensor: Option<Receiver<'a, M, PressureType, N>>,
    pub water_level_sensor: Option<Receiver<'a, M, WaterLevelType, N>>,
}

impl<'a, M: RawMutex, const N: usize> Boiler<'a, M, N> {
    pub fn new(
        heating_element: Box<dyn HeatingElement>,
        fill_mechanism: Option<Box<dyn BoilerFillMechanism>>,
        temperature_sensor: Option<Receiver<'a, M, TemperatureType, N>>,
        pressure_sensor: Option<Receiver<'a, M, PressureType, N>>,
        water_level_sensor: Option<Receiver<'a, M, WaterLevelType, N>>,
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
        //info!("Setting heating element duty cycle to {}%", duty_cycle_percent);

        self.heating_element.set_duty_cycle(duty_cycle_percent).await;
    }
    
    pub async fn get_heating_element_duty_cycle(&self) -> DutyCycleType {
        self.heating_element.get_duty_cycle().await
    }

    pub fn get_temperature(&mut self) -> Option<TemperatureType> {
        if let Some(temperature_sensor) = &mut self.temperature_sensor {
            temperature_sensor.try_get()
        } else {
            None
        }
    }

    pub fn get_pressure(&mut self) -> Option<PressureType> {
        if let Some(pressure_sensor) = &mut self.pressure_sensor {
            pressure_sensor.try_get()
        } else {
            None
        }
    }
}

pub struct Group<'a, M: RawMutex, const N: usize> {
    pub brew_mechanism: Option<Box<dyn BrewMechanism>>,
    pub heating_element: Option<Box<dyn HeatingElement>>,
    pub temperature_sensor: Option<Receiver<'a, M, TemperatureType, N>>,
    pub pressure_sensor: Option<Receiver<'a, M, PressureType, N>>,
    pub flow_sensor: Option<Receiver<'a, M, FlowRateType, N>>,
}

impl<'a, M: RawMutex, const N: usize> Group<'a, M, N> {
    pub fn new(
        brew_mechanism: Option<Box<dyn BrewMechanism>>,
        heating_element: Option<Box<dyn HeatingElement>>,
        temperature_sensor: Option<Receiver<'a, M, TemperatureType, N>>,
        pressure_sensor: Option<Receiver<'a, M, PressureType, N>>,
        flow_sensor: Option<Receiver<'a, M, FlowRateType, N>>,
    ) -> Self {
        Self {
            brew_mechanism,
            heating_element,
            temperature_sensor,
            pressure_sensor,
            flow_sensor,
        }
    }

    pub async fn set_brew_state(&mut self, brew_state: bool) {
        if let Some(brew_mechanism) = &mut self.brew_mechanism {
            //info!("Setting pump duty cycle to {}%", duty_cycle_percent);
            brew_mechanism.set_brew_state(brew_state).await.expect("TODO: panic message");
        }
    }

    pub async fn set_pump_duty_cycle(&mut self, duty_cycle_percent: DutyCycleType) {
        if let Some(brew_mechanism) = &mut self.brew_mechanism {
            //info!("Setting pump duty cycle to {}%", duty_cycle_percent);
            brew_mechanism.set_pump_duty_cycle(duty_cycle_percent).await.expect("TODO: panic message");
        }
    }
    
    pub fn get_pump_duty_cycle(&self) -> Option<DutyCycleType> {
        if let Some(brew_mechanism) = &self.brew_mechanism {
            brew_mechanism.get_pump_duty_cycle()
        } else {
            None
        }
    }

    pub fn get_temperature(&mut self) -> Option<TemperatureType> {
        self.temperature_sensor.as_mut().and_then(|sensor| sensor.try_get())
    }

    pub fn get_pressure(&mut self) -> Option<PressureType> {
        self.pressure_sensor.as_mut().and_then(|sensor| sensor.try_get())
    }

    pub fn get_flow_rate(&mut self) -> Option<FlowRateType> {
        self.flow_sensor.as_mut().and_then(|sensor| sensor.try_get())
    }
}

pub struct SteamWand {
    pub valve_mechanism: Option<Box<dyn ValveMechanism>>,
}

pub struct WaterTap<'a, M: RawMutex> {
    pub valve_mechanism: Option<Box<dyn ValveMechanism>>,
    pub mixer: Option<Box<dyn WaterMixerMechanism>>,
    pub temperature_sensor: Option<&'a Signal<M, TemperatureType>>,
    pub flow_sensor: Option<&'a Signal<M, FlowRateType>>,
}

pub trait WithTask {
    #[allow(async_fn_in_trait)]
    async fn task(&mut self);
}

#[async_trait]
pub trait HeatingElement {
    async fn set_duty_cycle(&mut self, duty_cycle_percent: u8);
    async fn get_duty_cycle(&self) -> u8;
}

pub trait BoilerFillMechanism {
    fn set_filling_state(&mut self, state: bool) -> Result<(), BoilerFillMechanismError>;
    fn get_filling_state(&self) -> Result<bool, BoilerFillMechanismError>;
}

#[async_trait]
pub trait BrewMechanism {
    // Typically this controls a three-way solenoid valve, if present
    async fn set_brew_state(&mut self, state: bool) -> Result<(), BrewMechanismError>;
    async fn set_pump_duty_cycle(&mut self, duty_cycle_percent: DutyCycleType) -> Result<(), BrewMechanismError>;
    
    fn get_pump_duty_cycle(&self) -> Option<DutyCycleType>;
}

pub trait ValveMechanism {
    fn set_valve_state(&mut self, state: ValveOpenType) -> Result<(), ValveMechanismError>;
}

pub trait WaterMixerMechanism {
    fn set_mixing_proportions(&mut self, hot_percent: MixingProportionType) -> Result<(), WaterMixerMechanismError>;
}