//! Display rotation

/// Display rotation
#[derive(Clone, Copy, Debug)]
pub enum DisplayRotation {
    /// No rotation, 0 degrees
    Rotate0,
    /// Rotate 90 degrees clockwise
    Rotate90,
    /// Rotate 180 degrees clockwise
    Rotate180,
    /// Rotate 270 degrees clockwise
    Rotate270,
}