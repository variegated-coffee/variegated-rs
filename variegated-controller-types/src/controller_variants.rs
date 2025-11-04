use crate::*;

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Default, Debug, PartialEq)]
pub enum SingleBoilerSingleGroupControllerState {
    #[default]
    BrewModeIdle,
    SteamModeIdle,
    Brewing,
    PumpingToWaterTap,
    PowerSave,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug)]
#[repr(u8)]
pub enum SingleBoilerSingleGroupControllerBoilers {
    BrewBoiler = 0,
    VirtualSteamBoiler = 1,
}

impl SingleBoilerSingleGroupControllerBoilers {
    pub fn is_brew_boiler(self) -> bool {
        matches!(self, SingleBoilerSingleGroupControllerBoilers::BrewBoiler)
    }

    pub fn is_virtual_steam_boiler(self) -> bool {
        matches!(self, SingleBoilerSingleGroupControllerBoilers::VirtualSteamBoiler)
    }

    pub fn as_index(&self) -> BoilerIndex {
        match self {
            SingleBoilerSingleGroupControllerBoilers::BrewBoiler => 0,
            SingleBoilerSingleGroupControllerBoilers::VirtualSteamBoiler => 1,
        }
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug)]
#[repr(u8)]
pub enum DualBoilerSingleGroupControllerBoilers {
    BrewBoiler = 0,
    SteamBoiler = 1,
}

impl DualBoilerSingleGroupControllerBoilers {
    pub fn is_brew_boiler(self) -> bool {
        matches!(self, DualBoilerSingleGroupControllerBoilers::BrewBoiler)
    }

    pub fn is_steam_boiler(self) -> bool {
        matches!(self, DualBoilerSingleGroupControllerBoilers::SteamBoiler)
    }

    pub fn as_index(&self) -> BoilerIndex {
        match self {
            DualBoilerSingleGroupControllerBoilers::BrewBoiler => 0,
            DualBoilerSingleGroupControllerBoilers::SteamBoiler => 1,
        }
    }
}

pub enum SingleGroupControllerGroups {
    SingleGroup = 0,
}

impl SingleGroupControllerGroups {
    pub fn as_index(&self) -> GroupIndex {
        match self {
            SingleGroupControllerGroups::SingleGroup => 0,
        }
    }
}
