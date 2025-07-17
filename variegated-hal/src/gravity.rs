use defmt::{error, Format};
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, RawMutex};
use embassy_sync::channel::Receiver;
use embassy_sync::mutex::Mutex;
use embassy_sync::watch::Sender;
use embassy_time::{Duration, Timer};
use embedded_hal_async::i2c::I2c;
use variegated_adc_tools::ConversionParameters;
use variegated_gravity_driver::{Channel, Gravity};
use variegated_instrumentation::{async_task_loop};
use variegated_mcp9600::MCP9600;
use crate::WithTask;

pub enum GravityCommand {
    Tare,
}

pub struct GravitySensor<'a, M: RawMutex, I2cDevT: I2c, const N: usize> {
    gravity: &'a Mutex<M, Gravity<I2cDevT>>,
    weight_signal: Option<Sender<'a, NoopRawMutex, f32, N>>,
    weight_conversion_parameters: ConversionParameters,
    rate_of_change_signal: Option<Sender<'a, NoopRawMutex, f32, N>>,
    rate_of_change_conversion_parameters: ConversionParameters,
    command_signal: Receiver<'a, NoopRawMutex, GravityCommand, N>,
    poll_delay: Duration,
    channel: Channel,
}

impl<'a, M: RawMutex, I2cDevT: I2c, const N: usize> GravitySensor<'a, M, I2cDevT, N> {
    pub fn new(
        gravity: &'a Mutex<M, Gravity<I2cDevT>>,
        channel: Channel,
        weight_signal: Option<Sender<'a, NoopRawMutex, f32, N>>,
        rate_of_change_signal: Option<Sender<'a, NoopRawMutex, f32, N>>,
        weight_conversion_parameters: ConversionParameters,
        rate_of_change_conversion_parameters: ConversionParameters,
        command_signal: Receiver<'a, NoopRawMutex, GravityCommand, N>,
        poll_delay: Duration,
    ) -> Self {
        GravitySensor {
            gravity,
            channel,
            weight_signal,
            weight_conversion_parameters,
            rate_of_change_signal,
            rate_of_change_conversion_parameters,
            command_signal,
            poll_delay
        }
    }
}

impl<'a, M: RawMutex, I2cDevT: I2c, const N: usize> WithTask for GravitySensor<'a, M, I2cDevT, N>  where <I2cDevT as embedded_hal::i2c::ErrorType>::Error: Format  {
    async fn task(&mut self) {
        async_task_loop!("Gravity Sensor", Some(self.poll_delay), {
            let mut dev = self.gravity.lock().await;
            
            if let Ok(command) = self.command_signal.try_receive() {
                match command {
                    GravityCommand::Tare => {
                        let res = dev.execute_tare_single(self.channel).await;
                        if let Err(e) = res {
                            error!("Failed to tare channel {}: {:?}", self.channel, e);
                        }
                    }
                }
            }

            if let Some(ref weight_signal) = self.weight_signal {
                let res = dev.read_weight(self.channel).await;

                if let Ok(value) = res {
                    weight_signal.send(self.weight_conversion_parameters.convert(value as f32));
                } else {
                    error!("Failed to read weight from Gravity: {:?}", res);
                }
            }

            if let Some(ref rate_of_change_signal) = self.rate_of_change_signal {
                let res = dev.read_rate_of_change(self.channel).await;

                if let Ok(value) = res {
                    rate_of_change_signal.send(self.rate_of_change_conversion_parameters.convert(value as f32));
                } else {
                    error!("Failed to read rate of change from Gravity: {:?}", res);
                }
            }
       })
    }
}
