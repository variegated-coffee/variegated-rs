#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, PartialEq, Debug, Default)]
pub enum MachineMode {
    On,
    #[default]
    Off,
    PowerSaveStandby,
}
