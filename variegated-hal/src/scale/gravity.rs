use alloc::boxed::Box;
use async_trait::async_trait;
use defmt::{error, info, Format};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex, RawMutex};
use embassy_sync::channel::{Receiver, Sender as ChannelSender};
use embassy_sync::mutex::Mutex;
use embassy_sync::watch::Sender as WatchSender;
use embassy_time::{Duration, Timer};
use embedded_hal_async::i2c::I2c;
use variegated_adc_tools::ConversionParameters;
use variegated_gravity_driver::{Channel, Error, Gravity, WeighingConfig};
use variegated_instrumentation::{async_task_loop};
use crate::scale::{ScaleController, ScaleError};
use crate::WithTask;

#[derive(Clone, Copy, Debug, Format)]
pub enum GravityCommand {
    Tare,
    SetWeighingConfig(WeighingConfig),
}

pub struct GravityDevice<'a, M: RawMutex, I2cDevT: I2c, const N: usize> {
    gravity: &'a Mutex<M, Gravity<I2cDevT>>,
    weight_signal: Option<WatchSender<'a, NoopRawMutex, f32, N>>,
    weight_conversion_parameters: ConversionParameters,
    rate_of_change_signal: Option<WatchSender<'a, NoopRawMutex, f32, N>>,
    rate_of_change_conversion_parameters: ConversionParameters,
    command_signal: Receiver<'a, CriticalSectionRawMutex, GravityCommand, N>,
    poll_delay: Duration,
    channel: Channel,
}

impl<'a, M: RawMutex, I2cDevT: I2c, const N: usize> GravityDevice<'a, M, I2cDevT, N> {
    pub fn new(
        gravity: &'a Mutex<M, Gravity<I2cDevT>>,
        channel: Channel,
        weight_signal: Option<WatchSender<'a, NoopRawMutex, f32, N>>,
        rate_of_change_signal: Option<WatchSender<'a, NoopRawMutex, f32, N>>,
        weight_conversion_parameters: ConversionParameters,
        rate_of_change_conversion_parameters: ConversionParameters,
        command_signal: Receiver<'a, CriticalSectionRawMutex, GravityCommand, N>,
        poll_delay: Duration,
    ) -> Self {
        GravityDevice {
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

impl<'a, M: RawMutex, I2cDevT: I2c, const N: usize> WithTask for GravityDevice<'a, M, I2cDevT, N>  where <I2cDevT as embedded_hal::i2c::ErrorType>::Error: Format  {
    async fn task(&mut self) {
        async_task_loop!("Gravity Sensor", Some(self.poll_delay), {
            let mut dev = self.gravity.lock().await;

            if let Ok(command) = self.command_signal.try_receive() {
                match command {
                    GravityCommand::Tare => {
                        info!("Taring channel {}", self.channel);
                        let res = dev.execute_tare_single(self.channel).await;
                        if let Err(e) = res {
                            error!("Failed to tare channel {}: {:?}", self.channel, e);
                        }
                    },
                    GravityCommand::SetWeighingConfig(config) => {
                        let res = dev.write_weighing_config(self.channel, config).await;
                        if let Err(e) = res {
                            error!("Failed to set weighing config for channel {}: {:?}", self.channel, e);
                        }
                    },
                }
            }
            
            let status = dev.read_channel_status(self.channel).await;
            
            if let Ok(status) = status {
                if let Some(ref weight_signal) = self.weight_signal {
                    if status.zero {
                        weight_signal.send(self.weight_conversion_parameters.convert(0.0));
                    } else {
                        let res = dev.read_weight(self.channel).await;

                        if let Ok(value) = res {
                            weight_signal.send(self.weight_conversion_parameters.convert(value as f32));
                        } else {
                            error!("Failed to read weight from Gravity: {:?}", res);
                        }
                    }
                }

                if let Some(ref rate_of_change_signal) = self.rate_of_change_signal {
                    if !status.motion {
                        rate_of_change_signal.send(self.rate_of_change_conversion_parameters.convert(0.0));
                    } else {
                        let res = dev.read_rate_of_change(self.channel).await;

                        if let Ok(value) = res {
                            rate_of_change_signal.send(self.rate_of_change_conversion_parameters.convert(value as f32));
                        } else {
                            error!("Failed to read rate of change from Gravity: {:?}", res);
                        }
                    }
                }
            } else {
                error!("Failed to read channel status for channel {}: {:?}", self.channel, status);
            }
       })
    }
}

pub struct GravityController<'a, const N: usize> {
    command_sender: ChannelSender<'a, CriticalSectionRawMutex, GravityCommand, N>,
}

impl <'a, const N: usize> GravityController<'a, N> {
    pub fn new(command_sender: ChannelSender<'a, CriticalSectionRawMutex, GravityCommand, N>) -> Self {
        GravityController { command_sender }
    }
}

#[async_trait]
impl <'a, const N: usize> ScaleController for GravityController<'a, N> {
    async fn tare(&mut self) -> Result<(), ScaleError> {
        self.command_sender.send(GravityCommand::Tare).await;

        Ok(())
    }

    async fn set_configuration(&mut self, config: &crate::scale::ScaleConfiguration) -> Result<(), ScaleError> {
        let mut gravity_config = WeighingConfig::default();

        if let Some(zero_tracking) = config.zero_tracking {
            gravity_config.zero_tracking = zero_tracking;
        }
        
        if let Some(smoothing) = config.smoothing {
            gravity_config.smoothing = smoothing;
        }

        self.command_sender.send(GravityCommand::SetWeighingConfig(gravity_config)).await;

        Ok(())
    }

    fn get_supported_configuration(&mut self) -> crate::scale::SupportedConfigurationOptions {
        crate::scale::SupportedConfigurationOptions {
            zero_tracking: true,
            smoothing: true,
        }
    }
}