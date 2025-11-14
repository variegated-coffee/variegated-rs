use crate::ValveOpenType;

/// Steam wand control state - stores the current control parameters
/// This is stored in ephemeral configuration (persists during runtime, resets on power cycle)
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SteamWandControlState {
    /// Valve openness percentage (0-100)
    pub valve_openness: ValveOpenType,
}

impl Default for SteamWandControlState {
    fn default() -> Self {
        Self {
            valve_openness: 100, // Default to full open
        }
    }
}
