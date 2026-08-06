$ErrorActionPreference = "Stop"

$repoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
$androidRoot = Join-Path $repoRoot "android"
$sdkRoot = if ($env:ANDROID_HOME) { $env:ANDROID_HOME } else { Join-Path $env:LOCALAPPDATA "Android\Sdk" }
$jdkRoot = @(
    $env:JAVA_HOME,
    "C:\Program Files\Android\Android Studio2\jbr",
    "C:\Program Files\Android\Android Studio\jbr"
) | Where-Object { $_ -and (Test-Path (Join-Path $_ "bin\java.exe")) } | Select-Object -First 1
$gradle = Get-ChildItem "$env:USERPROFILE\.gradle\wrapper\dists" -Recurse -Filter gradle.bat -ErrorAction SilentlyContinue |
    Sort-Object FullName -Descending |
    Select-Object -First 1

if (-not $jdkRoot) {
    throw "No usable JDK found. Install Android Studio or set JAVA_HOME."
}
if (-not (Test-Path $sdkRoot)) {
    throw "Android SDK not found. Set ANDROID_HOME."
}
if (-not $gradle) {
    throw "Gradle not found in the local wrapper cache."
}

$env:JAVA_HOME = $jdkRoot
$env:ANDROID_HOME = $sdkRoot
$env:ANDROID_SDK_ROOT = $sdkRoot

& $gradle.FullName -p $androidRoot :app:assembleDebug
$apk = Resolve-Path (Join-Path $androidRoot "app\build\outputs\apk\debug\app-debug.apk")
Write-Host "Android APK built: $apk"
