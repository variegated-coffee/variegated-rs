#![no_std]

extern crate alloc;

use alloc::boxed::Box;
use variegated_board_features::OpenLCCBoardFeatures;
use variegated_controller_lib::*;
use embassy_rp::{i2c, pwm, spi, uart};
use embassy_sync::mutex::Mutex;
use embassy_sync::blocking_mutex::raw::{ThreadModeRawMutex};
use embassy_time::Instant;
use num_traits::ToPrimitive;
use pid_ctrl::{PidCtrl, PidIn};
use actuator_cluster::BiancaGicarActuatorCluster;
use variegated_soft_pwm::SoftPwm;

mod sensor_cluster;
mod actuator_cluster;

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
    brew_boiler_heating_element: SoftPwm,
    service_boiler_diversion_solenoid: bool,
    line_solenoid: bool,
    service_boiler_heating_element: SoftPwm,
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

struct BiancaController {
    prev_update: Instant,
    brew_boiler_pid: PidCtrl<f32>,
    service_boiler_pid: PidCtrl<f32>,
}

impl Default for BiancaController {
    fn default() -> Self {
        BiancaController::new(PidCtrl::default(), PidCtrl::default())
    }
}

impl BiancaController {
    pub fn new(brew_boiler_pid: PidCtrl<f32>, service_boiler_pid: PidCtrl<f32>) -> Self {
        Self { prev_update: Instant::now(), brew_boiler_pid, service_boiler_pid }
    }
}

impl Controller<BiancaSystemSensorState, BiancaSystemActuatorState> for BiancaController {
    fn update_actuator_state_from_sensor_state(&mut self, system_sensor_state: &BiancaSystemSensorState, system_actuator_state: BiancaSystemActuatorState) -> BiancaSystemActuatorState {
        let now = Instant::now();
        let mut new_state = system_actuator_state.clone();

        let delta = now - self.prev_update;
        let delta_millis = delta.as_micros().to_f32().expect("For some weird reason, we couldn't convet a u64 to a f32") / 1000f32;

        let brew_out = self.brew_boiler_pid.step(PidIn::new(system_sensor_state.brew_boiler_temp_c, delta_millis));
        let service_out = self.service_boiler_pid.step(PidIn::new(system_sensor_state.service_boiler_temp_c, delta_millis));

        new_state.brew_boiler_heating_element.set_duty_cycle((brew_out.out * 100f32) as u8);
        new_state.service_boiler_heating_element.set_duty_cycle((service_out.out * 100f32) as u8);

        new_state.brew_boiler_heating_element.update(now);
        new_state.service_boiler_heating_element.update(now);

        self.prev_update = now;
        new_state
    }
}

pub fn create
    <LedPwmChT, EspUartT, IoxUartT, Qwiic1I2cT, Qwiic2I2cT, SettingsFlashSpiT, SdCardSpiT>
    (mut board_features: OpenLCCBoardFeatures<'static, LedPwmChT, EspUartT, IoxUartT, Qwiic1I2cT, Qwiic2I2cT, SettingsFlashSpiT, SdCardSpiT>)
     -> (
        Context<BiancaSystemSensorState, BiancaSystemActuatorState, 1, 1, 1>,
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
                    Box::new(sensor_cluster::BiancaGicarSensorCluster::new(iox_uart_rx))
                ],
                actuators: [
                    Box::new(BiancaGicarActuatorCluster::new(iox_uart_tx))
                ],
                controllers: [
                    Box::new(BiancaController::default()),
                ]
            }, 
            BiancaSystemSensorState::default(), 
            BiancaSystemActuatorState::default(),
            board_features
        );
}
