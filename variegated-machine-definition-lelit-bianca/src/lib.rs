#![no_std]

extern crate alloc;

use alloc::boxed::Box;
use async_trait::async_trait;
use variegated_board_features::OpenLCCBoardFeatures;
use variegated_controller_lib::*;
use embassy_rp::{i2c, spi, uart, pwm};
use embassy_rp::uart::{Instance, UartRx};
use variegated_gicar_8_5_04::{ntc_ohm_to_celsius, high_gain_adc_to_ohm};


#[derive(Default, Copy, Clone)]
pub struct BiancaSystemSensorState {
    brew_switch: bool,
    water_tank_empty: bool,
    service_boiler_water_level_adc: u16,
    brew_boiler_temp_c: f32,
    service_boiler_temp_c: f32,
}

impl SystemSensorState for BiancaSystemSensorState {}

#[derive(Default, Copy, Clone)]
pub struct BiancaSystemActuatorState {
    status_led: bool,
    brew_boiler_heating_element: bool,
    service_boiler_diversion_solenoid: bool,
    line_solenoid: bool,
    service_boiler_heating_element: bool,
    pump: bool,
}

impl SystemActuatorState for BiancaSystemActuatorState {}

pub struct BiancaSystemGeneralState {
    is_brewing: bool,
}

pub struct BiancaSystemConfiguration {
    brew_boiler_temp_setpoint_c: f32,
    service_boiler_temp_setpoint_c: f32,
}

struct BiancaGicarSensorCluster<'a, UartT: Instance> {
    gicar_cluster: variegated_gicar_8_5_04::Gicar8504SensorCluster<'a, BiancaSystemSensorState, UartT>
}

impl<'a, UartT: Instance> BiancaGicarSensorCluster<'a, UartT> {
    pub fn new(uart_rx: UartRx<'a, UartT, uart::Async>) -> BiancaGicarSensorCluster<'a, UartT> {
        BiancaGicarSensorCluster {
            gicar_cluster: variegated_gicar_8_5_04::Gicar8504SensorCluster::new(
                |msg, state: BiancaSystemSensorState| {
                    let mut new_state = state.clone();
                    new_state.brew_switch = msg.cn7;
                    new_state.water_tank_empty = msg.cn2;

                    new_state.brew_boiler_temp_c = ntc_ohm_to_celsius(high_gain_adc_to_ohm(msg.cn3_adc_high_gain.to_u16()), 50000, 4016);

                    new_state
                }, uart_rx
            )
        }
    }
}

#[async_trait]
impl<'a, UartT: Instance + Send + Sync> SensorCluster<BiancaSystemSensorState> for BiancaGicarSensorCluster<'a, UartT> {
    async fn update_sensor_state(&mut self, previous_state: BiancaSystemSensorState) -> BiancaSystemSensorState {
        self.gicar_cluster.update_sensor_state(previous_state).await
    }
}


struct DummyController {}

impl Controller<BiancaSystemSensorState, BiancaSystemActuatorState> for DummyController {
    fn update_actuator_state_from_sensor_state(&mut self, system_sensor_state: &BiancaSystemSensorState, system_actuator_state: BiancaSystemActuatorState) -> BiancaSystemActuatorState {
        BiancaSystemActuatorState {
            status_led: true,
            brew_boiler_heating_element: if system_sensor_state.brew_boiler_temp_c < 90.0 { true } else { false },
            service_boiler_diversion_solenoid: false,
            line_solenoid: false,
            service_boiler_heating_element: false,
            pump: false,
        }
    }
}


pub fn create
    <LedPwmChT, EspUartT, IoxUartT, Qwiic1I2cT, Qwiic2I2cT, SettingsFlashSpiT, SdCardSpiT>
    (mut board_features: OpenLCCBoardFeatures<'static, LedPwmChT, EspUartT, IoxUartT, Qwiic1I2cT, Qwiic2I2cT, SettingsFlashSpiT, SdCardSpiT>)
     -> (
        Context<BiancaSystemSensorState, BiancaSystemActuatorState, 1, 0, 1>, 
        BiancaSystemSensorState, 
        BiancaSystemActuatorState,
        OpenLCCBoardFeatures<LedPwmChT, EspUartT, IoxUartT, Qwiic1I2cT, Qwiic2I2cT, SettingsFlashSpiT, SdCardSpiT>
    ) 
    where
    LedPwmChT: pwm::Channel,
    EspUartT: uart::Instance,
    IoxUartT: uart::Instance + Send + Sync,
    Qwiic1I2cT: i2c::Instance,
    Qwiic2I2cT: i2c::Instance,
    SettingsFlashSpiT: spi::Instance,
    SdCardSpiT: spi::Instance, {
        let iox_uart_rx = board_features.iox_uart_rx.expect("Board features needs to have IOX UART RX");
        let iox_uart_tx = board_features.iox_uart_tx.expect("Board features needs to have IOX UART TX");    
    
        board_features.iox_uart_rx = None;
        board_features.iox_uart_tx = None;
    
        return (
            Context {
                sensors: [
                    Box::new(BiancaGicarSensorCluster::new(iox_uart_rx))
                ],
                actuators: [],
                controllers: [
                    Box::new(DummyController {}),
                ]
            }, 
            BiancaSystemSensorState::default(), 
            BiancaSystemActuatorState::default(),
            board_features
        );
}