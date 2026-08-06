# Android Backend Notes

XREAL Parallax is currently a Windows desktop prototype. The reusable pieces are
the panel layout model, IMU protocol parser, Madgwick fusion, input transforms,
and wgpu scene renderer. Android support should add native backends around those
pieces instead of trying to reuse the Windows DXGI and IddCx paths.

## Target Device

The XREAL 1S belongs in the supported-device discussion because it has the X1
spatial chip family and glasses-side spatial display modes. The existing IMU
protocol code is still based on XREAL One / One Pro community references, so
1S hardware validation should be tracked separately from the docs update.

XREAL's UltraWide guidance is device-dependent: Windows PCs are listed as a
common supported source, while Android support is limited to devices with
desktop modes such as Samsung DeX or Huawei Desktop Mode. A standard mobile
Android source should not be treated as equivalent to a PC.

The SBS renderer sizes its per-eye targets from the current output surface, so
1080p and 1200p side-by-side outputs can use the same renderer path. XREAL 1S
hardware validation still needs to confirm the exact USB display mode and IMU
transport exposed to each host OS.

## Backend Shape

- Capture: use Android MediaProjection for user-approved screen capture, or the
  XREAL SDK dual-screen flow when the app owns the glasses display content.
- Display output: render SBS or mono output to the Android external display
  surface. Android does not expose Windows-style IddCx virtual monitors, so
  multi-panel layout should be modeled inside the renderer.
- IMU: try the current TCP endpoint if Android exposes the USB network
  interface. If it does not, add an XREAL SDK-backed orientation provider.
- Permissions: MediaProjection requires a user grant, and protected app content
  may be blacked out by Android.

## Suggested Milestones

1. Keep the current desktop app building with platform-neutral traits.
2. Add an Android app shell that creates the render surface and loads the same
   TOML/default layout model.
3. Add MediaProjection capture for one source surface and feed it through
   `ScreenCapture`.
4. Add XREAL SDK dual-screen support for app-owned glasses output.
5. Validate IMU transport on XREAL 1S hardware and document the endpoint or SDK
   path that works.
