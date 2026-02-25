pub mod manager;

use anyhow::Result;
use serde::{Deserialize, Serialize};

/// Information about a created virtual display.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DisplayInfo {
    /// Virtual display ID.
    pub id: u32,
    /// Display resolution.
    pub width: u32,
    pub height: u32,
}

/// Trait for platform-specific virtual display creation.
pub trait VirtualDisplayProvider: Send {
    /// Create virtual displays with the given resolutions.
    fn create_displays(&mut self, configs: &[(u32, u32)]) -> Result<Vec<DisplayInfo>>;
    /// Remove all virtual displays.
    fn remove_all(&mut self) -> Result<()>;
}
