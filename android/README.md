# Android Prototype

This module builds an installable Android APK for Samsung DeX and other
external-display Android environments.

The current APK is an Android shell for the project, not the full Rust/wgpu
desktop renderer. It renders a multi-panel workspace preview on the active
display and opens the same view on a presentation display when Android exposes
one. It gives the Fold7/DeX path a concrete package while the native capture,
XREAL SDK, and IMU backends are added.

## Build

```powershell
$env:JAVA_HOME = "C:\Program Files\Android\Android Studio2\jbr"
$env:ANDROID_HOME = "$env:LOCALAPPDATA\Android\Sdk"
& "$env:USERPROFILE\.gradle\wrapper\dists\gradle-9.3.1-bin\23ovyewtku6u96viwx3xl3oks\gradle-9.3.1\bin\gradle.bat" `
  -p android :app:assembleDebug
```

The debug APK is written to `android/app/build/outputs/apk/debug/app-debug.apk`.

## Device Diagnostics

```powershell
powershell -ExecutionPolicy Bypass -File scripts/test-android-device.ps1
powershell -ExecutionPolicy Bypass -File scripts/test-android-device.ps1 -Install
```

See `docs/HARDWARE_TEST_PLAN.md` for the full Fold7/DeX checklist.
