# Hardware Test Plan

Use this once the XREAL glasses and the Samsung Fold7 are available. The scripts
write timestamped JSON reports under `diagnostics/`; those reports are ignored
by git and are safe to share back into the thread.

## Windows PC + XREAL

1. Plug the XREAL glasses directly into the PC over USB-C.
2. Put the glasses in the display mode you want to validate, such as normal
   mirror/extended display or UltraWide/SBS mode.
3. Run:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/test-xreal-windows.ps1 -LaunchSeconds 20
```

What the script checks:

- whether Windows sees an XREAL/Nreal USB device
- whether Windows exposes a candidate 1920x1080, 1920x1200, 3840x1080, or
  3840x1200 display
- whether the IMU TCP endpoint at `169.254.2.1:52998` is reachable
- whether `target/release/xreal-app.exe` starts and stays alive
- the last app log lines from startup

For a longer interactive run:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/run-windows.ps1
```

## Samsung Fold7 + DeX + XREAL

1. Enable Developer Options and USB debugging on the Fold7.
2. Connect the Fold7 to the PC and authorize the ADB prompt.
3. Build and inspect the APK without installing:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/test-android-device.ps1
```

4. Install the APK when the device shows up in ADB:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/test-android-device.ps1 -Install
```

5. Connect the Fold7 to the XREAL glasses through the USB-C path you normally
   use for DeX, then open XREAL Parallax on the phone or DeX display.

What the script checks:

- APK build and debug signature verification
- connected/authorized ADB devices
- phone model, Android version, SDK level, and One UI property when available
- Android display service output, including external/presentation display lines
- install result for `com.xreal.parallax`

## What To Send Back

Send the newest files from `diagnostics/` after each run, plus a quick note on
what was physically connected. The most useful combinations are:

- Windows with glasses connected
- Windows with glasses in UltraWide/SBS mode
- Fold7 attached to PC over ADB
- Fold7 attached to XREAL glasses in DeX mode

## Expected Current Limits

- Windows should launch and render, but real virtual monitors are still blocked
  by the stubbed DXGI capture and IddCx virtual-display integration.
- Android should install and show the workspace preview, but MediaProjection,
  native Rust/wgpu rendering, XREAL SDK output, and XREAL IMU integration are
  not wired in yet.
