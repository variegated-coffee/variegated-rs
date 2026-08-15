use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_time::{Duration, Timer};
use embedded_hal::digital::InputPin;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital::Wait;
use embedded_hal_async::spi::SpiDevice;
use variegated_hal::adc::ads124s08::Ads124S08Sensor;
use variegated_hal::WithTask;

/// Coordinator for ADS124S08 sensors that sequences measurements in a specific order.
///
/// This coordinator runs measurements in the following sequence:
/// 1. Brew pressure
/// 2. Steam pressure
/// 3. Brew temperature
/// 4. Brew pressure
/// 5. Steam pressure
/// 6. Steam temperature
///
/// This ensures predictable measurement timing and eliminates contention for the shared ADS124S08 device.
pub struct Ads124S08MeasurementCoordinator<'a, M: RawMutex, SpiDevT: SpiDevice, InputPinT: InputPin + Wait, D: DelayNs, const N: usize, const I: usize, const J: usize> {
    brew_temp_sensor: &'a mut Ads124S08Sensor<'a, M, SpiDevT, InputPinT, D, N, I, J>,
    brew_pressure_sensor: &'a mut Ads124S08Sensor<'a, M, SpiDevT, InputPinT, D, N, I, J>,
    steam_temp_sensor: &'a mut Ads124S08Sensor<'a, M, SpiDevT, InputPinT, D, N, I, J>,
    steam_pressure_sensor: &'a mut Ads124S08Sensor<'a, M, SpiDevT, InputPinT, D, N, I, J>,
}

impl<'a, M: RawMutex, SpiDevT: SpiDevice, InputPinT: InputPin + Wait, D: DelayNs, const N: usize, const I: usize, const J: usize>
    Ads124S08MeasurementCoordinator<'a, M, SpiDevT, InputPinT, D, N, I, J>
{
    /// Creates a new measurement coordinator with the four configured sensors.
    ///
    /// # Arguments
    ///
    /// * `brew_temp_sensor` - Brew boiler temperature sensor
    /// * `brew_pressure_sensor` - Brew boiler pressure sensor
    /// * `steam_temp_sensor` - Steam boiler temperature sensor
    /// * `steam_pressure_sensor` - Steam boiler pressure sensor
    pub fn new(
        brew_temp_sensor: &'a mut Ads124S08Sensor<'a, M, SpiDevT, InputPinT, D, N, I, J>,
        brew_pressure_sensor: &'a mut Ads124S08Sensor<'a, M, SpiDevT, InputPinT, D, N, I, J>,
        steam_temp_sensor: &'a mut Ads124S08Sensor<'a, M, SpiDevT, InputPinT, D, N, I, J>,
        steam_pressure_sensor: &'a mut Ads124S08Sensor<'a, M, SpiDevT, InputPinT, D, N, I, J>,
    ) -> Self {
        Self {
            brew_temp_sensor,
            brew_pressure_sensor,
            steam_temp_sensor,
            steam_pressure_sensor,
        }
    }
}

impl<'a, M: RawMutex, SpiDevT: SpiDevice, InputPinT: InputPin + Wait, D: DelayNs, const N: usize, const I: usize, const J: usize>
    WithTask for Ads124S08MeasurementCoordinator<'a, M, SpiDevT, InputPinT, D, N, I, J>
{
    async fn task(&mut self) {
        defmt::info!("Starting ADS124S08 measurement coordinator");

        loop {
            // First pair: Pressure sensors
            self.brew_pressure_sensor.measure().await;
            self.steam_pressure_sensor.measure().await;

            // Brew temperature
            self.brew_temp_sensor.measure().await;

            // Second pair: Pressure sensors again
            self.brew_pressure_sensor.measure().await;
            self.steam_pressure_sensor.measure().await;

            // Steam temperature
            self.steam_temp_sensor.measure().await;

            // Wait before next cycle (same timing as individual sensors)
            Timer::after(Duration::from_millis(5)).await;
        }
    }
}
