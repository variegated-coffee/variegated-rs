use defmt::Format;
use variegated_log::log_error;
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, RawMutex};
use embassy_sync::mutex::Mutex;
use embassy_sync::watch::Sender;
use embassy_time::{Duration, Timer};
use embedded_hal_async::i2c::I2c;
use variegated_adc_tools::ConversionParameters;
use variegated_mcp9600::MCP9600;
use crate::{WithTask, SensorReading};

pub struct Mcp9600Sensor<'a, M: RawMutex, I2cDevT: I2c, const N: usize> {
    mcp9600: &'a Mutex<M, MCP9600<I2cDevT>>,
    signal: Sender<'a, NoopRawMutex, SensorReading<f32>, N>,
    conversion_parameters: ConversionParameters,
    poll_delay: Duration,
}

impl<'a, M: RawMutex, I2cDevT: I2c, const N: usize> Mcp9600Sensor<'a, M, I2cDevT, N> {
    pub fn new(
        mcp9600: &'a Mutex<M, MCP9600<I2cDevT>>,
        signal: Sender<'a, NoopRawMutex, SensorReading<f32>, N>,
        conversion_parameters: ConversionParameters,
        poll_delay: Duration,
    ) -> Self {
        Mcp9600Sensor {
            mcp9600,
            signal,
            conversion_parameters,
            poll_delay
        }
    }
}

impl<'a, M: RawMutex, I2cDevT: I2c, const N: usize> WithTask for Mcp9600Sensor<'a, M, I2cDevT, N>  where <I2cDevT as embedded_hal::i2c::ErrorType>::Error: Format  {
    async fn task(&mut self) {
        loop {
            {
                let mut dev = self.mcp9600.lock().await;

                let res = dev.read_hot_junction().await;

                if let Ok(raw_value) = res {
                    let transformed_val = self.conversion_parameters.convert(raw_value);

                    let sensor_reading = SensorReading {
                        raw: raw_value,
                        transformed: transformed_val,
                    };

                    self.signal.send(sensor_reading);
                } else {
                    log_error!("Failed to read value from MCP9600: {:?}", res);
                }
            }

            Timer::after(self.poll_delay).await;
        }
    }
}
