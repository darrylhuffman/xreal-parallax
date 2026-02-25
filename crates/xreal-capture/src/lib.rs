use anyhow::Result;

/// A captured frame from a display.
pub struct CapturedFrame {
    /// RGBA8 pixel data.
    pub data: Vec<u8>,
    /// Frame width in pixels.
    pub width: u32,
    /// Frame height in pixels.
    pub height: u32,
    /// Source monitor identifier.
    pub monitor_id: u32,
}

/// Trait for platform-specific screen capture implementations.
pub trait ScreenCapture: Send {
    /// Capture the current frame from the associated display.
    /// Returns `None` if no new frame is available.
    fn try_capture(&mut self) -> Result<Option<CapturedFrame>>;

    /// The monitor ID this capture is associated with.
    fn monitor_id(&self) -> u32;
}

#[cfg(windows)]
pub mod duplication;
