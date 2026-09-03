use crate::*;
use heapless::index_map::FnvIndexMap;

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub enum PeripheralType {
    #[default]
    Scale,
    PressureSensor,
    FlowMeter,
    LevelSensor,
    BrewSensor,
    /// A device that drives the machine's UI rather than measuring anything.
    ///
    /// The odd one out on this enum: every type above it names something a
    /// [`crate::machine_definition::SensorCapability`] describes and that reports readings,
    /// and an input device does neither. It is a peripheral all the same because everything
    /// else about it is one -- it occupies a role, it is associated with a Bluetooth device
    /// through the same UI, and its connection status is reported the same way.
    ///
    /// Appended after `BrewSensor`; see the note on `BluetoothDriverKind`.
    InputDevice,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PeripheralInfo {
    pub peripheral_type: PeripheralType,
    pub is_available: bool,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PeripheralStatus {
    pub peripherals: FnvIndexMap<PeripheralId, PeripheralInfo, MAX_PERIPHERALS>,
}

#[cfg(feature = "defmt")]
impl defmt::Format for PeripheralStatus {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "PeripheralStatus {{ peripherals: [");
        for (id, info) in &self.peripherals {
            defmt::write!(f, "({}, {:?}, {}), ", id, info.peripheral_type, info.is_available);
        }
        defmt::write!(f, "] }}");
    }
}

pub trait PeripheralStatusProvider {
    fn get_peripheral_id(&self) -> PeripheralId;
    fn get_peripheral_type(&self) -> PeripheralType;
    fn is_available(&self) -> bool;
}

#[cfg(all(test, feature = "serde"))]
mod tests {
    use super::*;

    /// `InputDevice` went on the end, and `BrewSensor` did not move.
    ///
    /// This enum reaches further than the link: it is inside `Status.peripheral_status`, it
    /// is exported to the firmware frontend's generated schema, and it is frozen into the
    /// uplink schema the cloud reads. A renumbering here is wrong in four places at once.
    #[test]
    fn the_input_device_type_is_appended_after_brew_sensor() {
        let mut buf = [0u8; 8];
        assert_eq!(
            postcard::to_slice(&PeripheralType::BrewSensor, &mut buf).expect("serialize")[0],
            4
        );
        assert_eq!(
            postcard::to_slice(&PeripheralType::InputDevice, &mut buf).expect("serialize")[0],
            5
        );
    }
}

/// Which scale a command means.
///
/// A role, not a [`PeripheralId`]. The controller already owns its groups and knows
/// which scale each one has, so `GroupScale(0)` resolves with no lookup and no
/// registration step -- whereas an id would have to be matched against a table that
/// does not exist yet, and a client would have to know a machine-specific number to
/// name the scale sitting under group 1.
///
/// **Append-only.** postcard encodes an enum as its declaration-order discriminant, and
/// this travels inside `MachineCommand`. The obvious next variant is a bench dose scale,
/// which is a peripheral the machine does not have today and which nothing owns; it goes
/// on the end when it exists.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ScaleSelector {
    /// The scale under the named group -- the one brew-by-weight reads.
    GroupScale(GroupIndex),
}

/// What to do to a scale's own timer.
///
/// The timer runs on the scale and drives the scale's display; this firmware never reads it
/// back. Both supported protocols can drive one -- BooKoo through its `04`/`05`/`06`/`07`
/// commands, ACAIA through command `0x0D` -- so this is not specific to either.
///
/// Deliberately not the same type as
/// [`crate::ScaleOp`], which is the *inter-processor* vocabulary and also carries
/// [`crate::ScaleOp::Tare`]. Keeping them apart means adding an operation to the link does
/// not silently widen what a client is allowed to ask for.
///
/// **Append-only**, for the same reason as [`ScaleSelector`]: it travels inside
/// `MachineCommand`.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ScaleTimerCommand {
    /// Start the timer running.
    Start,
    /// Stop it, leaving the elapsed time displayed.
    Stop,
    /// Return it to zero.
    Reset,
    /// Zero the scale and start the timer together.
    TareAndStart,
}
