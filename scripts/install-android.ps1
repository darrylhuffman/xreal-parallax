$ErrorActionPreference = "Stop"

$repoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
$sdkRoot = if ($env:ANDROID_HOME) { $env:ANDROID_HOME } else { Join-Path $env:LOCALAPPDATA "Android\Sdk" }
$adb = Join-Path $sdkRoot "platform-tools\adb.exe"
$apk = Join-Path $repoRoot "android\app\build\outputs\apk\debug\app-debug.apk"

if (-not (Test-Path $adb)) {
    throw "adb not found. Set ANDROID_HOME or install Android SDK platform-tools."
}
if (-not (Test-Path $apk)) {
    & (Join-Path $PSScriptRoot "build-android.ps1")
}

& $adb devices
& $adb install -r $apk
