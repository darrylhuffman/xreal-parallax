# XReal Parallax

A spatial multi-monitor workspace for XReal One AR glasses. Turn your single USB-C display into up to 4 virtual monitors floating in 3D space around you — with stereoscopic depth, curved displays, and head tracking.

## How It Works

```
Virtual Monitors → Screen Capture → 3D Composition → Head Tracking → Stereo Output
```

1. **Creates virtual monitors** on Windows via IddCx drivers — your OS treats them as real displays
2. **Captures each monitor** using DXGI Desktop Duplication (GPU-accelerated)
3. **Renders them as 3D panels** — flat or curved, any size, any orientation
4. **Tracks your head** via the XReal One's onboard IMU over TCP
5. **Outputs stereoscopic SBS** (3840×1080) to the glasses for true depth perception

## Features

- **Up to 4 virtual monitors** with independent resolutions (1080p, ultrawide, custom)
- **Per-monitor curvature** — flat panels, gentle curves, or deep arcs
- **Per-monitor rotation** — tilt, angle, or go portrait mode
- **Per-monitor sizing** — mix ultrawides with small sidecars
- **Stereoscopic 3D** — Side-by-Side rendering with configurable IPD
- **Head tracking** — Madgwick AHRS fusion from the XReal One's 1000Hz IMU
- **Layout presets** — Curved Arc, Flat Row, Stacked 2×2, Ultrawide + Sidecar
- **ALT+Mouse manipulation** — reposition, resize, curve, and rotate panels in-place
- **Config persistence** — your layout saves to TOML and restores on launch

## Controls

| Input | Action |
|-------|--------|
| **F1** | Toggle settings overlay |
| **F5** | Recenter head tracking |
| **F9** | Toggle mono/stereo mode |
| **ALT + Left Drag** | Move panel |
| **ALT + Scroll** | Push/pull panel depth |
| **ALT + Right Drag** | Resize panel |
| **ALT + Middle Drag** | Adjust curvature / pitch |
| **ALT + Shift+Scroll** | Rotate panel (yaw) |
| **ALT + Ctrl+Scroll** | Rotate panel (roll) |

## Requirements

- **XReal One** or **XReal One Pro** glasses (X1 chip, USB-C)
- **Windows 10/11** (macOS support planned)
- **Rust 1.75+**
- **[Virtual Display Driver](https://github.com/MolotovCherry/virtual-display-rs)** (for creating virtual monitors)

## Quick Start

```bash
# Clone and build
git clone https://github.com/YOUR_USERNAME/xreal-parallax.git
cd xreal-parallax
cargo build --release

# Run (works without glasses — head tracking activates automatically when connected)
cargo run --release
```

Connect your XReal One glasses via USB-C. Parallax auto-detects the IMU at `169.254.2.1:52998` and falls back to a static camera if the glasses aren't connected.

## Architecture

```
parallax/
├── crates/
│   ├── xreal-imu/        # TCP IMU client + Madgwick sensor fusion
│   ├── xreal-capture/     # DXGI Desktop Duplication screen capture
│   ├── xreal-vdisplay/    # Virtual display creation (IddCx driver IPC)
│   ├── xreal-renderer/    # wgpu 3D renderer — panels, stereo camera, SBS
│   ├── xreal-input/       # ALT+mouse panel manipulation + raycasting
│   └── xreal-config/      # TOML config, layout presets, panel properties
└── src/main.rs            # Orchestrates all subsystems
```

Each crate is platform-abstracted behind traits (`ScreenCapture`, `VirtualDisplayProvider`) so the renderer, IMU client, config, and input systems can be reused when macOS support is added.

## Configuration

Config lives at `%APPDATA%/xreal-app/config.toml`:

```toml
ipd_mm = 63.0
virtual_monitor_count = 4

[imu]
madgwick_beta = 0.1
calibration_samples = 500

[layout]
preset = "CurvedArc"

[[layout.panels]]
id = 0
resolution = [1920, 1080]
position = [-1.4, 0.0, -2.1]
rotation = [0.0, 0.26, 0.0, 0.97]
scale = [1.2, 0.675]
curvature = 0.15
curve_segments = 32
```

## Roadmap

- [x] XReal One IMU protocol (TCP binary parser)
- [x] Madgwick AHRS sensor fusion
- [x] wgpu 3D renderer with flat + curved panel meshes
- [x] Stereoscopic SBS dual-eye rendering
- [x] Per-panel curvature, rotation, and sizing
- [x] ALT+mouse panel manipulation
- [x] Layout presets and TOML config persistence
- [ ] DXGI Desktop Duplication capture (currently stubbed with test patterns)
- [ ] Virtual display driver integration (virtual-display-rs IPC)
- [ ] egui settings overlay
- [ ] macOS support (ScreenCaptureKit + CGVirtualDisplay)
- [ ] GPU-native texture sharing (zero-copy D3D11→wgpu path)

## Acknowledgements

Built on the shoulders of:
- [One-Pro-IMU-Retriever-Demo](https://github.com/SamiMitwalli/One-Pro-IMU-Retriever-Demo) — XReal One TCP protocol reference
- [virtual-display-rs](https://github.com/MolotovCherry/virtual-display-rs) — Rust IddCx virtual display driver
- [wgpu](https://wgpu.rs/) — Cross-platform GPU API
- [ahrs](https://crates.io/crates/ahrs) — Madgwick/Mahony AHRS filters

## License

MIT
