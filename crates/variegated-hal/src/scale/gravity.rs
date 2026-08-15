use alloc::boxed::Box;
use core::cell::Cell;
use async_trait::async_trait;
use defmt::Format;
// Sites that format a `variegated_gravity_driver::Channel` stay on `defmt::*!`
// (fully qualified below): `Channel` implements `defmt::Format` but not
// `core::fmt::Display`, so the `log` half of `log_*!` will not compile for them.
// That is every `info!` in this file, which is why only `log_error` is imported.
use variegated_log::log_error;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex, RawMutex};
use embassy_sync::channel::{Receiver, Sender as ChannelSender};
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_sync::watch::Sender as WatchSender;
use embassy_time::{Duration, Timer};
use embedded_hal_async::i2c::I2c;
use variegated_adc_tools::ConversionParameters;
use variegated_gravity_driver::{Channel, Error, Gravity, WeighingConfig};
use variegated_instrumentation::{async_task_loop};
use crate::scale::{ScaleController, ScaleError};
use crate::{WithTask, SensorReading};
use variegated_controller_types::{PeripheralStatusProvider, PeripheralType, PeripheralId};

#[derive(Clone, Copy, Debug, Format)]
pub enum GravityCommand {
    Tare,
    SetWeighingConfig(WeighingConfig),
    ZeroCalibration,
    ReferenceWeightCalibration(u32),
}

pub struct GravityDevice<'a, M: RawMutex, CM: RawMutex, I2cDevT: I2c, const N: usize> {
    gravity: &'a Mutex<M, Gravity<I2cDevT>>,
    weight_signal: Option<WatchSender<'a, NoopRawMutex, SensorReading<f32>, N>>,
    weight_conversion_parameters: ConversionParameters,
    rate_of_change_signal: Option<WatchSender<'a, NoopRawMutex, SensorReading<f32>, N>>,
    rate_of_change_conversion_parameters: ConversionParameters,
    command_signal: Receiver<'a, CM, GravityCommand, N>,
    poll_delay: Duration,
    channel: Channel,
    connected_signal: Option<&'a Signal<NoopRawMutex, bool>>,
    retry_delay: Duration,
    max_retry_delay: Duration,
    is_connected: bool,
}

impl<'a, M: RawMutex, CM: RawMutex, I2cDevT: I2c, const N: usize> GravityDevice<'a, M, CM, I2cDevT, N> {
    pub fn new(
        gravity: &'a Mutex<M, Gravity<I2cDevT>>,
        channel: Channel,
        weight_signal: Option<WatchSender<'a, NoopRawMutex, SensorReading<f32>, N>>,
        rate_of_change_signal: Option<WatchSender<'a, NoopRawMutex, SensorReading<f32>, N>>,
        weight_conversion_parameters: ConversionParameters,
        rate_of_change_conversion_parameters: ConversionParameters,
        command_signal: Receiver<'a, CM, GravityCommand, N>,
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
            poll_delay,
            connected_signal: None,
            retry_delay: Duration::from_secs(1),
            max_retry_delay: Duration::from_secs(30),
            is_connected: false,
        }
    }

    pub fn with_connected_signal(mut self, signal: &'a Signal<NoopRawMutex, bool>) -> Self {
        self.connected_signal = Some(signal);
        self
    }

    async fn attempt_connection(&mut self) -> Result<(), Error<I2cDevT::Error>> {
        let mut dev = self.gravity.lock().await;
        dev.check_compatibility().await?;
        Ok(())
    }

    async fn update_connection_status(&mut self, connected: bool) {
        if self.is_connected != connected {
            self.is_connected = connected;
            if let Some(signal) = self.connected_signal {
                signal.signal(connected);
            }
            if connected {
                defmt::info!("Gravity scale connected on channel {}", self.channel);
            } else {
                defmt::error!("Gravity scale disconnected on channel {}", self.channel);
            }
        }
    }
}

impl<'a, M: RawMutex, CM: RawMutex, I2cDevT: I2c, const N: usize> WithTask for GravityDevice<'a, M, CM, I2cDevT, N>  where <I2cDevT as embedded_hal::i2c::ErrorType>::Error: Format  {
    async fn task(&mut self) {
        let mut current_retry_delay = self.retry_delay;
        
        async_task_loop!("Gravity Sensor", Some(self.poll_delay), {
            // If not connected, attempt to connect with exponential backoff
            if !self.is_connected {
                match self.attempt_connection().await {
                    Ok(_) => {
                        self.update_connection_status(true).await;
                        current_retry_delay = self.retry_delay; // Reset retry delay on successful connection
                    }
                    Err(e) => {
                        log_error!("Failed to connect to Gravity scale: {:?}", e);

                        // Drop any pending commands to avoid stale commands being executed after reconnect
                        self.command_signal.clear();

                        // Exponential backoff
                        Timer::after(current_retry_delay).await;
                        current_retry_delay = (current_retry_delay * 2).min(self.max_retry_delay);
                        continue; // Skip rest of loop iteration
                    }
                }
            }

            let mut dev = self.gravity.lock().await;

            if let Ok(command) = self.command_signal.try_receive() {
                match command {
                    GravityCommand::Tare => {
                        defmt::info!("Taring channel {}", self.channel);
                        let res = dev.execute_tare_single(self.channel).await;
                        if let Err(e) = res {
                            defmt::error!("Failed to tare channel {}: {:?}", self.channel, e);
                        }
                    },
                    GravityCommand::SetWeighingConfig(config) => {
                        let res = dev.write_weighing_config(self.channel, config).await;
                        if let Err(e) = res {
                            defmt::error!("Failed to set weighing config for channel {}: {:?}", self.channel, e);
                        }
                    },
                    GravityCommand::ZeroCalibration => {
                        defmt::info!("Starting zero calibration on channel {}", self.channel);
                        let mut channels = [false; 4];
                        channels[self.channel as usize - 1] = true;
                        let res = dev.execute_zero_calibration(channels).await;
                        if let Err(e) = res {
                            defmt::error!("Zero calibration failed on channel {}: {:?}", self.channel, e);
                        }
                    },
                    GravityCommand::ReferenceWeightCalibration(weight_grams) => {
                        defmt::info!("Starting {}g calibration on channel {}", weight_grams, self.channel);
                        let mut channels = [false; 4];
                        channels[self.channel as usize - 1] = true;
                        let res = match weight_grams {
                            100 => dev.execute_100g_calibration(channels).await,
                            _ => {
                                log_error!("Unsupported calibration weight: {}g", weight_grams);
                                continue;
                            }
                        };
                        if let Err(e) = res {
                            defmt::error!("{}g calibration failed on channel {}: {:?}", weight_grams, self.channel, e);
                        }
                    },
                }
            }
            
            let status = dev.read_channel_status(self.channel).await;
            
            if let Ok(status) = status {
                // Ensure we're still connected after successful read
                if !self.is_connected {
                    self.update_connection_status(true).await;
                    current_retry_delay = self.retry_delay; // Reset retry delay
                }
                
                if let Some(ref weight_signal) = self.weight_signal {
                    if status.zero {
                        let raw_value = 0i32;
                        let transformed_value = self.weight_conversion_parameters.convert(0.0);
                        let sensor_reading = SensorReading {
                            raw: raw_value as f32,
                            transformed: transformed_value,
                        };
                        weight_signal.send(sensor_reading);
                    } else {
                        let res = dev.read_weight(self.channel).await;

                        if let Ok(raw_value) = res {
                            let transformed_value = self.weight_conversion_parameters.convert(raw_value as f32);
                            let sensor_reading = SensorReading {
                                raw: raw_value as f32,
                                transformed: transformed_value,
                            };
                            weight_signal.send(sensor_reading);
                        } else {
                            log_error!("Failed to read weight from Gravity: {:?}", res);
                            // Drop lock before updating connection status
                            drop(dev);
                            self.update_connection_status(false).await;
                            continue;
                        }
                    }
                }

                if let Some(ref rate_of_change_signal) = self.rate_of_change_signal {
                    if !status.motion {
                        let raw_value = 0i32;
                        let transformed_value = self.rate_of_change_conversion_parameters.convert(0.0);
                        let sensor_reading = SensorReading {
                            raw: raw_value as f32,
                            transformed: transformed_value,
                        };
                        rate_of_change_signal.send(sensor_reading);
                    } else {
                        let res = dev.read_rate_of_change(self.channel).await;

                        if let Ok(raw_value) = res {
                            let transformed_value = self.rate_of_change_conversion_parameters.convert(raw_value as f32);
                            let sensor_reading = SensorReading {
                                raw: raw_value as f32,
                                transformed: transformed_value,
                            };
                            rate_of_change_signal.send(sensor_reading);
                        } else {
                            log_error!("Failed to read rate of change from Gravity: {:?}", res);
                            // Drop lock before updating connection status
                            drop(dev);
                            self.update_connection_status(false).await;
                            continue;
                        }
                    }
                }
            } else {
                defmt::error!("Failed to read channel status for channel {}: {:?}", self.channel, status);
                // Drop lock before updating connection status
                drop(dev);
                self.update_connection_status(false).await;
            }
       })
    }
}

pub struct GravityController<'a, M: RawMutex, const N: usize> {
    command_sender: ChannelSender<'a, M, GravityCommand, N>,
}

impl<'a, M: RawMutex, const N: usize> GravityController<'a, M, N> {
    pub fn new(command_sender: ChannelSender<'a, M, GravityCommand, N>) -> Self {
        GravityController { command_sender }
    }
}

#[async_trait]
impl<'a, M: RawMutex + Sync, const N: usize> ScaleController for GravityController<'a, M, N> {
    async fn tare(&mut self) -> Result<(), ScaleError> {
        self.command_sender.try_send(GravityCommand::Tare).map_err(|_| ScaleError::TareFailed)
    }

    async fn set_configuration(&mut self, config: &crate::scale::ScaleConfiguration) -> Result<(), ScaleError> {
        let mut gravity_config = WeighingConfig::default();

        if let Some(zero_tracking) = config.zero_tracking {
            gravity_config.zero_tracking = zero_tracking;
        }
        
        if let Some(smoothing) = config.smoothing {
            gravity_config.smoothing = smoothing;
        }

        self.command_sender
            .try_send(GravityCommand::SetWeighingConfig(gravity_config))
            .map_err(|_| ScaleError::ConfigurationFailed)
    }

    fn get_supported_configuration(&mut self) -> crate::scale::SupportedConfigurationOptions {
        crate::scale::SupportedConfigurationOptions {
            zero_tracking: true,
            smoothing: true,
        }
    }

    async fn zero_calibration(&mut self) -> Result<(), ScaleError> {
        self.command_sender.try_send(GravityCommand::ZeroCalibration).map_err(|_| ScaleError::CalibrationFailed)
    }

    async fn reference_weight_calibration(&mut self, weight_grams: u32) -> Result<(), ScaleError> {
        // Check if the weight is supported
        let capabilities = self.get_capabilities();
        if !capabilities.supported_reference_weights.contains(&weight_grams) {
            return Err(ScaleError::CalibrationNotSupported);
        }

        self.command_sender.try_send(GravityCommand::ReferenceWeightCalibration(weight_grams)).map_err(|_| ScaleError::CalibrationFailed)
    }

    fn get_capabilities(&self) -> crate::scale::ScaleCapabilities {
        crate::scale::ScaleCapabilities {
            zero_calibration: true,
            reference_weight_calibration: true,
            supported_reference_weights: &[100], // Only 100g supported by hardware
        }
    }
}

pub struct GravityStatusProvider<'a> {
    peripheral_id: PeripheralId,
    connected_signal: &'a Signal<NoopRawMutex, bool>,
    last_status: Cell<bool>,
}

impl<'a> GravityStatusProvider<'a> {
    pub fn new(peripheral_id: PeripheralId, connected_signal: &'a Signal<NoopRawMutex, bool>) -> Self {
        Self {
            peripheral_id,
            connected_signal,
            last_status: Cell::new(false),
        }
    }
}

impl<'a> PeripheralStatusProvider for GravityStatusProvider<'a> {
    fn get_peripheral_id(&self) -> PeripheralId {
        self.peripheral_id
    }
    
    fn get_peripheral_type(&self) -> PeripheralType {
        PeripheralType::Scale
    }
    
    fn is_available(&self) -> bool {
        // Try to take a new value from the signal, otherwise use the last known value
        if let Some(status) = self.connected_signal.try_take() {
            self.last_status.set(status);
            status
        } else {
            self.last_status.get()
        }
    }
}