use alloc::boxed::Box;
use async_trait::async_trait;
use defmt::Format;

pub mod gravity;

#[derive(Debug, Format)]
pub enum ScaleError {
    TareFailed,
    ConfigurationFailed,
    UnsupportedConfiguration,
    CommunicationError,
}

pub struct ScaleConfiguration {
    pub zero_tracking: Option<bool>,
    pub smoothing: Option<bool>,
}

pub struct SupportedConfigurationOptions {
    pub zero_tracking: bool,
    pub smoothing: bool,
}

#[async_trait]
pub trait ScaleController {
    async fn tare(&mut self) -> Result<(), ScaleError>;
    async fn set_configuration(&mut self, configuration: &ScaleConfiguration) -> Result<(), ScaleError>;

    fn get_supported_configuration(&mut self) -> SupportedConfigurationOptions;
}