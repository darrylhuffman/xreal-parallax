mod types;

pub use types::*;

use anyhow::Result;
use std::path::PathBuf;
use tracing::info;

/// Returns the config directory: %APPDATA%/xreal-app/
pub fn config_dir() -> Result<PathBuf> {
    let dir = dirs::config_dir()
        .ok_or_else(|| anyhow::anyhow!("Could not determine config directory"))?
        .join("xreal-app");
    std::fs::create_dir_all(&dir)?;
    Ok(dir)
}

/// Returns the config file path: %APPDATA%/xreal-app/config.toml
pub fn config_path() -> Result<PathBuf> {
    Ok(config_dir()?.join("config.toml"))
}

/// Load config from disk, or return default if not found.
pub fn load_config() -> Result<AppConfig> {
    let path = config_path()?;
    if path.exists() {
        let contents = std::fs::read_to_string(&path)?;
        let config: AppConfig = toml::from_str(&contents)?;
        info!(?path, "Loaded config");
        Ok(config)
    } else {
        info!("No config found, using defaults");
        Ok(AppConfig::default())
    }
}

/// Save config to disk.
pub fn save_config(config: &AppConfig) -> Result<()> {
    let path = config_path()?;
    let contents = toml::to_string_pretty(config)?;
    std::fs::write(&path, contents)?;
    info!(?path, "Saved config");
    Ok(())
}
