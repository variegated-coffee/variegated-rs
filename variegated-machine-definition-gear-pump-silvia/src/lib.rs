#![no_std]

extern crate alloc;

use alloc::boxed::Box;
use variegated_board_features::AllPurposeEspressoControllerBoardFeatures;
use variegated_controller_lib::*;
use embassy_rp::{i2c, pwm, spi, uart};
use embassy_rp::gpio::{Input, Level, Output, Pin, Pull};
use embassy_time::Instant;
use num_traits::ToPrimitive;
use pid_ctrl::{PidCtrl, PidIn};
use variegated_embassy_ads124s08::{ADS124S08, WaitStrategy};
use variegated_embassy_ads124s08::registers::{IDACMagnitude, IDACMux, Mux, PGAGain, ReferenceInput};
use variegated_soft_pwm::SoftPwm;

mod actuator_cluster;
mod sensor_cluster;

#[derive(Default, Copy, Clone)]
pub struct SilviaSystemSensorState {
    brew_switch: bool,
    water_switch: bool,
    steam_switch: bool,
    boiler_temp_c: f32,
    boiler_pressure_bar: f32,
}

impl SystemSensorState for SilviaSystemSensorState {}

#[derive(Default, Copy, Clone)]
pub struct SilviaSystemActuatorState {
    brew_boiler_heating_element: SoftPwm,
    brew_solenoid: bool,
    pump_duty_cycle: f32,
}

impl SystemActuatorState for SilviaSystemActuatorState {}

pub struct SilviaSystemGeneralState {
    is_brewing: bool,
}

pub struct SilviaSystemConfiguration {
    boiler_temp_setpoint_c: f32,
    pump_pressure_setpoint_bar: f32,
}

struct SilviaController {
    prev_update: Instant,
    brew_boiler_pid: PidCtrl<f32>,
}

impl Default for SilviaController {
    fn default() -> Self {
        SilviaController::new(PidCtrl::default())
    }
}

impl SilviaController {
    pub fn new(brew_boiler_pid: PidCtrl<f32>) -> Self {
        Self { prev_update: Instant::now(), brew_boiler_pid }
    }
}

impl Controller<SilviaSystemSensorState, SilviaSystemActuatorState> for SilviaController {
    fn update_actuator_state_from_sensor_state(&mut self, system_sensor_state: &SilviaSystemSensorState, system_actuator_state: SilviaSystemActuatorState) -> SilviaSystemActuatorState {
        let now = Instant::now();
        let mut new_state = system_actuator_state.clone();

        let delta = now - self.prev_update;
        let delta_millis = delta.as_micros().to_f32().expect("For some weird reason, we couldn't convet a u64 to a f32") / 1000f32;

        let brew_out = self.brew_boiler_pid.step(PidIn::new(system_sensor_state.boiler_temp_c, delta_millis));

        new_state.brew_boiler_heating_element.set_duty_cycle((brew_out.out * 100f32) as u8);

        new_state.brew_boiler_heating_element.update(now);

        self.prev_update = now;
        new_state
    }
}


pub fn create
<EspUartT, Cn1UartT, I2cT, SpiT, Cn94ChT, Cn96ChT, Cn98ChT>
    (mut board_features: AllPurposeEspressoControllerBoardFeatures<'static, EspUartT, Cn1UartT, I2cT, SpiT, Cn94ChT, Cn96ChT, Cn98ChT>)
     -> (
         Context<SilviaSystemSensorState, SilviaSystemActuatorState, 1, 1, 1>,
         SilviaSystemSensorState,
         SilviaSystemActuatorState,
         AllPurposeEspressoControllerBoardFeatures<EspUartT, Cn1UartT, I2cT, SpiT, Cn94ChT, Cn96ChT, Cn98ChT>
    ) 
    where
        EspUartT: uart::Instance,
        Cn1UartT: uart::Instance,
        I2cT: i2c::Instance,
        SpiT: spi::Instance,
        Cn94ChT: pwm::Channel,
        Cn96ChT: pwm::Channel,
        Cn98ChT: pwm::Channel
{
    let brew_button_input = Input::new(board_features.cn14_8_pin.expect("Board features needs to have CN14_8").degrade(), Pull::Up);
    board_features.cn14_8_pin = None;

    let steam_button_input = Input::new(board_features.cn14_6_pin.expect("Board features needs to have CN14_6").degrade(), Pull::Up);
    board_features.cn14_6_pin = None;

    let water_button_input = Input::new(board_features.cn14_4_pin.expect("Board features needs to have CN14_4").degrade(), Pull::Up);
    board_features.cn14_4_pin = None;

    let shift_register = board_features.dual_shift_register.expect("Board features needs to have a shift register");
    board_features.dual_shift_register = None;

    let adc = ADS124S08::new(
        Output::new(board_features.adc_cs_pin.expect("Board features needs to have an ADC CS pin").degrade(), Level::Low), 
        WaitStrategy::Delay,
    );
    
    board_features.adc_cs_pin = None;
    
    return (
            Context {
                sensors: [
                    Box::new(sensor_cluster::silvia_gpio_sensor_cluster::SilviaGpioSensorCluster::new(
                        brew_button_input,
                        steam_button_input,
                        water_button_input,
                    )),
/*                    Box::new(sensor_cluster::silvia_adc_sensor_cluster::SilviaAdcSensorCluster::new(
                        adc,
                        Mux::AIN1,
                        Mux::AIN2,
                        IDACMux::AIN0,
                        IDACMU,
                        boiler_temperature_reference_input: ReferenceInput,
                        boiler_temperature_mag: IDACMagnitude,
                        boiler_temperature_gain: PGAGain,
                        boiler_temperature_celsius_per_volt: f32,
                        boiler_temperature_reference_resistor: Option<f32>,
                        boiler_temperature_reference_voltage: Option<(f32, f32)>
                    ))                   
 */
                ],
                actuators: [
                    Box::new(actuator_cluster::SilviaShiftRegisterOutputCluster::new(shift_register))
                ],
                controllers: [
                    Box::new(SilviaController::default()),
                ]
            }, 
            SilviaSystemSensorState::default(), 
            SilviaSystemActuatorState::default(),
            board_features
        );
}
