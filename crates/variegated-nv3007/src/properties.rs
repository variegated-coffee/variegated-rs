//! Display properties

use crate::displayrotation::DisplayRotation;
use display_interface::AsyncWriteOnlyDataCommand;

/// Display properties
pub struct DisplayProperties<DV, DI>
where
    DI: AsyncWriteOnlyDataCommand,
{
    /// Display variant
    pub variant: DV,
    /// Display interface
    pub iface: DI,
    /// Display rotation
    pub rotation: DisplayRotation,
}

impl<DV, DI> DisplayProperties<DV, DI>
where
    DI: AsyncWriteOnlyDataCommand,
{
    /// Create new display properties
    pub fn new(variant: DV, iface: DI, rotation: DisplayRotation) -> Self {
        Self {
            variant,
            iface,
            rotation,
        }
    }

    /// Get the interface mutably
    pub fn get_mut_iface(&mut self) -> &mut DI {
        &mut self.iface
    }

    /// Decompose into parts
    pub fn decompose(self) -> (DV, DI, DisplayRotation) {
        (self.variant, self.iface, self.rotation)
    }
}