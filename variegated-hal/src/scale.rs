use async_trait::async_trait;
use variegated_controller_types::{WeightChangeType, WeightType};

pub struct ScaleConfiguration {
    pub zero_tracking: Option<bool>,
    pub smoothing: Option<bool>,
}

pub struct SupportedConfigurationOptions {
    pub zero_tracking: bool,
    pub smoothing: bool,
}

#[async_trait]
pub trait Scale<E: core::fmt::Debug> {
    async fn get_weight(&self) -> Result<WeightType, E>;
    async fn get_rate_of_change(&self) -> Result<WeightChangeType, E>;
    async fn is_stable(&self) -> Result<bool, E>;
    async fn is_zero(&self) -> Result<bool, E>;

    async fn tare(&mut self) -> Result<(), E>;
    async fn set_configuration(&mut self, configuration: &ScaleConfiguration) -> Result<(), E>;
    
    fn get_supported_configuration() -> SupportedConfigurationOptions;
}