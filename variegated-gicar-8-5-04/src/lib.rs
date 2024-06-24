#![no_std]

extern crate alloc;

use alloc::boxed::Box;
use core::cell::RefCell;
use async_trait::async_trait;
use bitflags::bitflags;
use embassy_rp::uart::{Async, Instance, UartRx, UartTx};
use embassy_sync::mutex::Mutex;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use variegated_controller_lib::{ActualBoardFeatures, ActualBoardFeaturesMutex, ActuatorCluster, ActuatorClusterError, SensorCluster, SensorClusterError, SystemActuatorState, SystemSensorState};
use libm::{powf, log};

pub struct SensorMessage {
    pub cn2: bool,
    pub cn7: bool,
    pub cn4_adc_low_gain: Varint,
    pub cn4_adc_high_gain: Varint,
    pub cn3_adc_low_gain: Varint,
    pub cn3_adc_high_gain: Varint,
    pub cn1_capacitance: Varint,
}

enum SensorMessageError {
    WrongHeader,
    InvalidChecksum
}

impl SensorMessage {
    fn to_bytes(&self) -> [u8; 18] {
        let mut bytes: [u8; 18] = [0; 18];
        bytes[0] = 0x81;

        if self.cn2 {
            bytes[1] |= 0x40;
        }

        if self.cn7 {
            bytes[1] |= 0x02;
        }

        bytes[2..5].copy_from_slice(&self.cn4_adc_low_gain.bytes);
        bytes[5..8].copy_from_slice(&self.cn4_adc_high_gain.bytes);
        bytes[8..11].copy_from_slice(&self.cn3_adc_low_gain.bytes);
        bytes[11..14].copy_from_slice(&self.cn3_adc_high_gain.bytes);
        bytes[14..17].copy_from_slice(&self.cn3_adc_high_gain.bytes);
        bytes[17] = calculate_checksum(&bytes[1..17], 0x01);

        bytes
    }

    fn from_bytes(bytes: [u8; 18]) -> Result<Self, SensorMessageError> {
        if bytes[0] != 0x81 {
            return Err(SensorMessageError::WrongHeader);
        }

        if calculate_checksum(&bytes[1..17], 0x01) != bytes[17] {
            return Err(SensorMessageError::InvalidChecksum);
        }

        Ok(SensorMessage {
            cn2: bytes[1] & 0x40 != 0,
            cn7: bytes[1] & 0x02 != 0,
            cn4_adc_low_gain: Varint::new([bytes[2], bytes[3], bytes[4]]),
            cn4_adc_high_gain: Varint::new([bytes[5], bytes[6], bytes[7]]),
            cn3_adc_low_gain: Varint::new([bytes[8], bytes[9], bytes[10]]),
            cn3_adc_high_gain: Varint::new([bytes[11], bytes[12], bytes[13]]),
            cn1_capacitance: Varint::new([bytes[14], bytes[15], bytes[16]]),
        })
    }
}

bitflags! {
    struct GicarShiftRegister2Flags: u8 {
        const CN5 = 1 << 0;
        const FA10 = 1 << 4;
        const FA9 = 1 << 5;
    }
}

bitflags! {
    struct GicarShiftRegister1Flags: u8 {
        const CN6_1 = 1 << 0;
        const CN6_3 = 1 << 1;
        const CN6_5 = 1 << 2;
        const CN9 = 1 << 3;
        const FA7 = 1 << 4;
        const FA8 = 1 << 5;
        const CN10_DISABLE_OLED_12V = 1 << 6; // Maybe. They are connected to some kind of transistor that would presumably cut power to 12V on CN10 if used, but this has not been tested yet
        const CN10_DISABLE_OLED_3V3 = 1 << 7; // Maybe. They are connected to some kind of transistor that would presumably cut power to 3V3 OLED on CN10 if used, but this has not been tested yet
    }
}

pub struct ActuatorMessage {
    pub cn5: bool,
    pub fa10: bool,
    pub fa9: bool,
    pub cn6_1: bool,
    pub cn6_3: bool,
    pub cn6_5: bool,
    pub cn9: bool,
    pub cn7: bool, // Only available on Elizabeth
    pub fa7: bool,
    pub fa8: bool,
    pub cn10_n_12v: bool,
    pub cn10_n_3v3: bool,
    pub minus_button: bool,
    pub plus_button: bool,
}

impl ActuatorMessage {
    fn to_bytes(&self) -> [u8; 5] {
        let mut bytes: [u8; 5] = [0; 5];
        bytes[0] = 0x80;

        if self.minus_button {
            bytes[3] |= 0x08;
        }

        if self.plus_button {
            bytes[3] |= 0x04;
        }

        bytes[4] = calculate_checksum(&bytes[0..4], 0x02);

        bytes
    }
}

pub struct Varint {
    bytes: [u8; 3]
}

impl Varint {
    fn new(bytes: [u8; 3]) -> Self {
        Varint {
            bytes
        }
    }

    pub fn from_u16(value: u16) -> Self {
        let byte0 = ((value >> 8) & 0xFF) as u8;
        let byte1_original = (value & 0xFF) as u8;
        let byte1 = byte1_original & 0x7F;
        let byte2 = if byte1_original & 0x80 != 0 { 0x7F } else { 0x00 };

        Self {
            bytes: [byte0, byte1, byte2],
        }
    }

    pub fn to_u16(&self) -> u16 {
        if self.bytes[2] == 0x7F {
            ((self.bytes[1] | 0x80) as u16) + (self.bytes[0] as u16) << 8
        } else {
            (self.bytes[1] as u16) + (self.bytes[0] as u16) << 8
        }
    }
}

fn calculate_checksum(buf: &[u8], initial: u8) -> u8 {
    if buf.len() > 32 {
        return 0;
    }

    let mut checksum: u16 = initial as u16;
    for &item in buf.iter() {
        checksum += item as u16;
    }

    (checksum & 0x7F) as u8
}

pub fn high_gain_adc_to_ohm(adc_code: u16) -> u32 {
    let float_value = adc_code as f32;
    (7181.23 * powf(-(float_value - 1018.15) / (float_value - 1.56789), 8000.0 / 8043.0)) as u32
}

pub fn ntc_ohm_to_celsius(ohm: u32, r25: u32, b: u32) -> f32 {
    let ohm_f32 = ohm as f64;
    let r25_f32 = r25 as f64;
    let b_f32 = b as f32;

    (1.0 / (log(ohm_f32 / r25_f32) as f32 / b_f32 + 1.0 / 298.15)) - 273.15
}

pub struct Gicar8504ActuatorCluster<'a, ActuatorStateT: SystemActuatorState, UartT: Instance> {
    mapper: fn(&ActuatorStateT) -> ActuatorMessage,
    uart_tx: UartTx<'a, UartT, Async>
}

impl<'a, ActuatorStateT: SystemActuatorState, UartT: Instance> Gicar8504ActuatorCluster<'a, ActuatorStateT, UartT> {
    pub fn new(mapper: fn(&ActuatorStateT) -> ActuatorMessage, uart_tx: UartTx<'a, UartT, embassy_rp::uart::Async>) -> Self {
        Gicar8504ActuatorCluster {
            mapper,
            uart_tx
        }
    }
}

#[async_trait]
impl<'a, ActuatorStateT: SystemActuatorState, UartT: Instance + Send + Sync> ActuatorCluster<ActuatorStateT> for Gicar8504ActuatorCluster<'a, ActuatorStateT, UartT> {
    async fn update_from_actuator_state(&mut self, system_actuator_state: &ActuatorStateT) -> Result<(), ActuatorClusterError> {
        let buf = (self.mapper)(system_actuator_state).to_bytes();
        self.uart_tx.write(&buf).await.map_err(|_| ActuatorClusterError::UnknownError)
    }
}

pub struct Gicar8504SensorCluster<'a, SensorStateT: SystemSensorState, UartT: Instance> {
    mapper: fn(SensorMessage, SensorStateT) -> SensorStateT,
    uart_rx: UartRx<'a, UartT, Async>
}

impl<'a, SensorStateT: SystemSensorState, UartT: Instance> Gicar8504SensorCluster<'a, SensorStateT, UartT> {
    pub fn new(mapper: fn(SensorMessage, SensorStateT) -> SensorStateT, uart_rx: UartRx<'a, UartT, embassy_rp::uart::Async>) -> Self {
        Gicar8504SensorCluster {
            mapper,
            uart_rx
        }
    }
}

#[async_trait]
impl<'a, SensorStateT: SystemSensorState, UartT: 'a + Instance + Send + Sync> SensorCluster<SensorStateT> for Gicar8504SensorCluster<'a, SensorStateT, UartT> {
    async fn update_sensor_state(&mut self, previous_state: SensorStateT, _board_features: &ActualBoardFeaturesMutex) -> Result<SensorStateT, SensorClusterError> {
        let mut buf: [u8; 18] = [0; 18];

        let res = self.uart_rx.read(&mut buf).await;

        if res.is_err() {
            return Err(SensorClusterError::UnknownError);
        }

        let sensor_message = match SensorMessage::from_bytes(buf) {
            Ok(msg) => msg,
            Err(_) => return Err(SensorClusterError::UnknownError)
        };

        Ok((self.mapper)(sensor_message, previous_state.clone()))
    }
}