use alloc::boxed::Box;
use async_trait::async_trait;
use defmt::Format;

pub mod bluetooth;
pub mod gravity;

#[derive(Debug, Format)]
pub enum ScaleError {
    TareFailed,
    ConfigurationFailed,
    UnsupportedConfiguration,
    CommunicationError,
    CalibrationNotSupported,
    CalibrationFailed,
    /// The fitted scale has no timer this firmware can drive.
    TimerNotSupported,
    /// The fitted scale cannot be told a dose.
    ///
    /// Rarer than [`Self::TimerNotSupported`] and less knowable: only BooKoo's Ultra defines
    /// the command, and nothing in this firmware can tell an Ultra from a Themis Mini. So this
    /// is returned by drivers that certainly cannot -- a load cell has no display to put a
    /// dose on -- while a Bluetooth scale that merely ignores the frame returns `Ok(())`.
    DoseSyncNotSupported,
}

pub struct ScaleConfiguration {
    pub zero_tracking: Option<bool>,
    pub smoothing: Option<bool>,
}

pub struct SupportedConfigurationOptions {
    pub zero_tracking: bool,
    pub smoothing: bool,
}

pub struct ScaleCapabilities {
    pub zero_calibration: bool,
    pub reference_weight_calibration: bool,
    pub supported_reference_weights: &'static [u32], // grams
    /// Whether the scale has a timer this firmware can drive.
    ///
    /// Both Bluetooth protocols do; a load cell wired to the machine does not, because
    /// there is no scale for a timer to run on. Reported rather than assumed so a panel or
    /// a client can hide a control that would do nothing -- the same argument
    /// `variegated-controller-lib`'s `scale_calibration` module makes for the two
    /// calibration flags.
    pub timer: bool,
}

#[async_trait]
pub trait ScaleController {
    async fn tare(&mut self) -> Result<(), ScaleError>;
    async fn set_configuration(&mut self, configuration: &ScaleConfiguration) -> Result<(), ScaleError>;
    async fn zero_calibration(&mut self) -> Result<(), ScaleError>;
    async fn reference_weight_calibration(&mut self, weight_grams: u32) -> Result<(), ScaleError>;

    /// Drive the scale's own timer.
    ///
    /// Required rather than defaulted, which is not the obvious choice: an implementation
    /// without a timer only ever writes `Err(ScaleError::TimerNotSupported)`, and a default
    /// body would spare it. But `#[async_trait]` puts a `Self: Send + Sync` bound on every
    /// *provided* method, and this trait is used as `dyn ScaleController` -- so a default
    /// body here stops `Group` compiling, with an error that names `Send` and not the
    /// default body that asked for it. Two implementations, one of which is a single line,
    /// is the cheaper side of that trade.
    ///
    /// Write-only, like the rest of the timer path -- nothing reads the elapsed time back.
    async fn control_timer(
        &mut self,
        command: variegated_controller_types::ScaleTimerCommand,
    ) -> Result<(), ScaleError>;

    /// Tell the scale the dry dose, in grams.
    ///
    /// Required rather than defaulted for the reason [`Self::control_timer`] gives about
    /// `#[async_trait]` and `dyn`.
    ///
    /// Write-only, like the timer, and less answerable than it: the only protocol with this
    /// command acknowledges nothing, and the three models that speak it are
    /// indistinguishable. `Ok(())` means the frame was handed to the link, not that a scale
    /// acted on it.
    async fn set_dose(&mut self, grams: f32) -> Result<(), ScaleError>;

    fn get_supported_configuration(&mut self) -> SupportedConfigurationOptions;
    fn get_capabilities(&self) -> ScaleCapabilities;
}