use crate::{DisplayInfo, VirtualDisplayProvider};
use anyhow::Result;
use tracing::info;

/// Virtual display manager.
///
/// Currently a stub that reports displays as created.
/// Full implementation will integrate with virtual-display-rs (IddCx driver)
/// via the driver-ipc crate's named pipe protocol.
///
/// Prerequisites for real implementation:
/// 1. Install virtual-display-rs driver (requires WDK build + code signing)
/// 2. Ensure vdd-user-session-service is running
/// 3. Connect via named pipe "virtualdisplaydriver"
/// 4. Send monitor configurations via the IPC protocol
pub struct VirtualDisplayManager {
    displays: Vec<DisplayInfo>,
}

impl VirtualDisplayManager {
    pub fn new() -> Self {
        Self {
            displays: Vec::new(),
        }
    }
}

impl VirtualDisplayProvider for VirtualDisplayManager {
    fn create_displays(&mut self, configs: &[(u32, u32)]) -> Result<Vec<DisplayInfo>> {
        self.displays = configs
            .iter()
            .enumerate()
            .map(|(i, &(width, height))| {
                info!(id = i, width, height, "Virtual display created (stub)");
                DisplayInfo {
                    id: i as u32,
                    width,
                    height,
                }
            })
            .collect();

        Ok(self.displays.clone())
    }

    fn remove_all(&mut self) -> Result<()> {
        info!(
            count = self.displays.len(),
            "Removing all virtual displays (stub)"
        );
        self.displays.clear();
        Ok(())
    }
}
