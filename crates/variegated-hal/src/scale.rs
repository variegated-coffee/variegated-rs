use alloc::boxed::Box;
use async_trait::async_trait;
use defmt::Format;

pub mod bluetooth;
pub mod gravity;

#[derive(Debug, Format)]
pub enum ScaleError {
    TareFailed,
    ConfigurationFailed,
    UnsupportedConfiguration,
    CommunicationError,
    CalibrationNotSupported,
    CalibrationFailed,
}

pub struct ScaleConfiguration {
    pub zero_tracking: Option<bool>,
    pub smoothing: Option<bool>,
}

pub struct SupportedConfigurationOptions {
    pub zero_tracking: bool,
    pub smoothing: bool,
}

pub struct ScaleCapabilities {
    pub zero_calibration: bool,
    pub reference_weight_calibration: bool,
    pub supported_reference_weights: &'static [u32], // grams
}

#[async_trait]
pub trait ScaleController {
    async fn tare(&mut self) -> Result<(), ScaleError>;
    async fn set_configuration(&mut self, configuration: &ScaleConfiguration) -> Result<(), ScaleError>;
    async fn zero_calibration(&mut self) -> Result<(), ScaleError>;
    async fn reference_weight_calibration(&mut self, weight_grams: u32) -> Result<(), ScaleError>;

    fn get_supported_configuration(&mut self) -> SupportedConfigurationOptions;
    fn get_capabilities(&self) -> ScaleCapabilities;
}