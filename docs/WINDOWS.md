# Windows Status

The Windows desktop app now builds, launches, and renders the panel workspace.
It can target XREAL-style 3840 x 1080 and 3840 x 1200 side-by-side outputs by
sizing the stereo eye buffers from the active output surface.

## Working Today

- Release executable builds at `target/release/xreal-app.exe`.
- The app launches in a desktop window when no XREAL display is detected.
- If an XREAL/Nreal-named display or common SBS output is present, the app opens
  fullscreen on that display.
- IMU startup tries the Windows raw USB CDC NCM/ECM path first, then falls back
  to direct TCP at `169.254.2.1:52998`.
- Panel hover, drag, resize, curvature, and relative scroll rotation paths are
  wired into the render loop.

## Still Stubbed

- `xreal-capture/src/duplication.rs` still returns a generated checkerboard
  pattern instead of using DXGI Desktop Duplication.
- `xreal-vdisplay/src/manager.rs` still reports virtual displays as created
  without talking to a real IddCx virtual display driver.

Those two items are the remaining blockers for true PC-style multi-monitor
content capture. Without them, the Windows build is a working renderer and IMU
prototype, not a complete virtual monitor replacement.

## Build And Run

```powershell
powershell -ExecutionPolicy Bypass -File scripts/build-windows.ps1
powershell -ExecutionPolicy Bypass -File scripts/run-windows.ps1
```

For plugged-in glasses diagnostics, run:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/test-xreal-windows.ps1 -LaunchSeconds 20
```

See [Hardware Test Plan](HARDWARE_TEST_PLAN.md) for the full device checklist.
