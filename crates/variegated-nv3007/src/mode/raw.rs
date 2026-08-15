//! Raw mode for direct command access

use crate::{
    command::Command,
    display::{self, DisplayVariant},
    mode::displaymode::DisplayModeTrait,
    properties::DisplayProperties,
    displays::nv3007::Nv3007_168_428,
};
use display_interface::{AsyncWriteOnlyDataCommand, DisplayError};

/// Raw mode for direct command access
pub struct RawMode<DV, DI>
where
    DI: AsyncWriteOnlyDataCommand,
    DV: display::DisplayVariant,
{
    properties: DisplayProperties<DV, DI>,
}

impl<DV, DI> DisplayModeTrait<DV, DI> for RawMode<DV, DI>
where
    DI: AsyncWriteOnlyDataCommand,
    DV: display::DisplayVariant,
{
    /// Create new RawMode instance
    fn new(properties: DisplayProperties<DV, DI>) -> Self {
        RawMode { properties }
    }

    /// Release resources
    fn release(self) -> DisplayProperties<DV, DI> {
        self.properties
    }
}

impl<DV, DI> RawMode<DV, DI>
where
    DI: AsyncWriteOnlyDataCommand,
    DV: display::DisplayVariant,
{
    /// Initialize the display
    pub async fn init(&mut self) -> Result<(), DisplayError> {
        DV::init(&mut self.properties.iface).await
    }
}

impl<DI> RawMode<Nv3007_168_428, DI>
where
    DI: AsyncWriteOnlyDataCommand,
{
    /// Initialize the display using the variant-specific initialization
    pub async fn init_with_variant(&mut self) -> Result<(), DisplayError> {
        self.properties.variant.init_self(&mut self.properties.iface).await
    }

    /// Send a command to the display (NV3007 specific)
    pub async fn send_command_nv3007(&mut self, command: Command) -> Result<(), DisplayError> {
        command.send(&mut self.properties.iface).await
    }

    /// Get display dimensions (NV3007 specific)
    pub fn dimensions_nv3007(&self) -> (u16, u16) {
        Nv3007_168_428::dimensions()
    }
}