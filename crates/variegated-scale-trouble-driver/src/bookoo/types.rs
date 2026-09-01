use trouble_host::prelude::*;

/// Service UUID advertised and served by every BooKoo Themis scale.
///
/// A 16-bit shorthand in the base range, so the full form is
/// `00000ffe-0000-1000-8000-00805f9b34fb`. The scale advertises it, which is what lets
/// `driver_for_service` in the comms firmware pre-fill the driver in the pairing UI.
pub const BOOKOO_SERVICE_UUID: Uuid = Uuid::new_short(0x0FFE);

/// Characteristic carrying weight notifications.
///
/// Notify only. Distinct from [`BOOKOO_COMMAND_CHAR_UUID`] -- unlike ACAIA's older
/// protocol, which notifies and writes on one characteristic, so a client for this
/// protocol has to cache two.
pub const BOOKOO_WEIGHT_CHAR_UUID: Uuid = Uuid::new_short(0xFF11);

/// Characteristic that accepts commands. Write only.
pub const BOOKOO_COMMAND_CHAR_UUID: Uuid = Uuid::new_short(0xFF12);

/// Events a BooKoo scale reports.
///
/// Richer than ACAIA's single weight variant because a BooKoo weight frame carries flow
/// and battery alongside the weight, and the Ultra sends two further frame types. The
/// driver decodes all of them; what the comms firmware does with each is its business.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ScaleEvent {
    /// An ordinary weight frame: weight, flow, battery and the scale's settings.
    Weight(variegated_scale_codec::bookoo::WeightFrame),
    /// Ultra only: the powder weight the scale has been told about.
    Powder(variegated_scale_codec::bookoo::PowderFrame),
    /// Ultra only: an automatic-mode transition.
    AutoMode(variegated_scale_codec::bookoo::AutoModeFrame),
}

impl From<variegated_scale_codec::bookoo::Frame> for ScaleEvent {
    fn from(frame: variegated_scale_codec::bookoo::Frame) -> Self {
        use variegated_scale_codec::bookoo::Frame;
        match frame {
            Frame::Weight(f) => ScaleEvent::Weight(f),
            Frame::Powder(f) => ScaleEvent::Powder(f),
            Frame::AutoMode(f) => ScaleEvent::AutoMode(f),
        }
    }
}
