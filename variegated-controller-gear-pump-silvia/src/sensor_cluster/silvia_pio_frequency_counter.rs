use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{PIN_0, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio, PioPin};
use variegated_controller_lib::{ActualBoardFeaturesMutex, SensorCluster, SensorClusterError};
use variegated_embassy_rp_frequency_counter::PioFrequencyCounter;
use variegated_log::log_info;
use crate::state::SilviaSystemSensorState;

pub struct SilviaPioFrequencyCounter<'a> {
    frequency_counter: PioFrequencyCounter<'a, PIO0>,
}

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

impl<'a> SilviaPioFrequencyCounter<'a> {
    pub fn new<Pin1T: PioPin, Pin2T: PioPin>(p: PIO0, pump_rpm_pin: Pin1T, flow_meter_rpm_pin: Pin2T) -> SilviaPioFrequencyCounter<'a> {
        let mut pio = Pio::new(p, Irqs);

        log_info!("Creating SilviaPioFrequencyCounter 1");
        let frequency_counter = PioFrequencyCounter::new::<Pin1T, Pin2T, PIN_0, PIN_0>(pio, Some(pump_rpm_pin), Some(flow_meter_rpm_pin), None, None);
        log_info!("Creating SilviaPioFrequencyCounter 3");


        SilviaPioFrequencyCounter {
            frequency_counter,
        }
    }
}

impl<'a> SensorCluster<SilviaSystemSensorState> for SilviaPioFrequencyCounter<'a> {
    async fn update_sensor_state(&mut self, state: &mut SilviaSystemSensorState, _board_features: &ActualBoardFeaturesMutex) -> Result<(), SensorClusterError> {
        let freq1 = self.frequency_counter.read_frequency_pin_1();

        state.pump_rpm = if freq1 < 10000.0 { freq1 * (60.0/32.0) } else { 0.0 };
        //state.flow_meter_rpm = self.frequency_counter.read_frequency_pin_2();

        Ok(())
    }
}
