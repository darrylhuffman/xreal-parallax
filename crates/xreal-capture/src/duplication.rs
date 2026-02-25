use crate::{CapturedFrame, ScreenCapture};
use anyhow::Result;
use tracing::info;

/// DXGI Desktop Duplication based screen capture.
///
/// This is the Windows implementation of ScreenCapture.
/// Uses the Desktop Duplication API to capture display output
/// with GPU-accelerated performance.
///
/// TODO: Implement when virtual displays are set up.
/// The implementation will use the `windows` crate to:
/// 1. Create a D3D11 device
/// 2. Enumerate DXGI outputs to find our virtual monitors
/// 3. Create IDXGIOutputDuplication for each output
/// 4. AcquireNextFrame to get GPU textures
/// 5. Copy to staging texture + map for CPU read
/// 6. Convert BGRA -> RGBA for wgpu upload
pub struct DxgiCapture {
    monitor_id: u32,
    width: u32,
    height: u32,
}

impl DxgiCapture {
    pub fn new(monitor_id: u32, width: u32, height: u32) -> Result<Self> {
        info!(monitor_id, width, height, "DXGI capture initialized (stub)");
        Ok(Self {
            monitor_id,
            width,
            height,
        })
    }
}

impl ScreenCapture for DxgiCapture {
    fn try_capture(&mut self) -> Result<Option<CapturedFrame>> {
        // Stub: return a colored frame for testing.
        // Each monitor gets a distinct color so we can verify they render independently.
        let colors: [(u8, u8, u8); 4] = [
            (40, 80, 160),   // Blue
            (160, 60, 40),   // Red
            (40, 140, 60),   // Green
            (140, 100, 40),  // Orange
        ];
        let (r, g, b) = colors[(self.monitor_id as usize) % colors.len()];

        let pixel_count = (self.width * self.height) as usize;
        let mut data = Vec::with_capacity(pixel_count * 4);

        // Generate a checkerboard pattern with the base color.
        let checker_size = 32u32;
        for y in 0..self.height {
            for x in 0..self.width {
                let is_light = ((x / checker_size) + (y / checker_size)) % 2 == 0;
                let factor = if is_light { 1.0_f32 } else { 0.7 };
                data.push((r as f32 * factor) as u8);
                data.push((g as f32 * factor) as u8);
                data.push((b as f32 * factor) as u8);
                data.push(255);
            }
        }

        Ok(Some(CapturedFrame {
            data,
            width: self.width,
            height: self.height,
            monitor_id: self.monitor_id,
        }))
    }

    fn monitor_id(&self) -> u32 {
        self.monitor_id
    }
}
