use crate::{CapturedFrame, ScreenCapture};
use anyhow::Result;
use tracing::info;

/// Portable pattern capture used when there is no native desktop capture backend.
pub struct PatternCapture {
    monitor_id: u32,
    width: u32,
    height: u32,
    cached_frame: Vec<u8>,
    sent: bool,
}

impl PatternCapture {
    pub fn new(monitor_id: u32, width: u32, height: u32) -> Result<Self> {
        info!(
            monitor_id,
            width, height, "Pattern capture initialized (no native backend)"
        );

        let colors: [(u8, u8, u8); 4] =
            [(40, 80, 160), (160, 60, 40), (40, 140, 60), (140, 100, 40)];
        let (r, g, b) = colors[(monitor_id as usize) % colors.len()];

        let pixel_count = (width * height) as usize;
        let mut data = Vec::with_capacity(pixel_count * 4);
        let checker_size = 32u32;

        for y in 0..height {
            for x in 0..width {
                let is_light = ((x / checker_size) + (y / checker_size)) % 2 == 0;
                let factor = if is_light { 1.0_f32 } else { 0.7 };
                data.push((r as f32 * factor) as u8);
                data.push((g as f32 * factor) as u8);
                data.push((b as f32 * factor) as u8);
                data.push(255);
            }
        }

        Ok(Self {
            monitor_id,
            width,
            height,
            cached_frame: data,
            sent: false,
        })
    }
}

impl ScreenCapture for PatternCapture {
    fn try_capture(&mut self) -> Result<Option<CapturedFrame>> {
        if self.sent {
            return Ok(None);
        }
        self.sent = true;

        Ok(Some(CapturedFrame {
            data: self.cached_frame.clone(),
            width: self.width,
            height: self.height,
            monitor_id: self.monitor_id,
        }))
    }

    fn monitor_id(&self) -> u32 {
        self.monitor_id
    }
}
