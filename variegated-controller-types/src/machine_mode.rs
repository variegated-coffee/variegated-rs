#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "ts-typegen", derive(variegated_postcard_ts_typegen::PostcardTsTypegen))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, PartialEq, Debug, Default)]
pub enum MachineMode {
    On,
    #[default]
    Off,
    PowerSaveStandby,
}
