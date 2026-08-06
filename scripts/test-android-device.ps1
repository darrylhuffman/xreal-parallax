param(
    [switch]$Install,
    [string]$Serial,
    [switch]$SkipBuild,
    [string]$ReportDir
)

$ErrorActionPreference = "Stop"

$repoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
if (-not $ReportDir) {
    $ReportDir = Join-Path $repoRoot "diagnostics"
}
New-Item -ItemType Directory -Path $ReportDir -Force | Out-Null

$timestamp = Get-Date -Format "yyyyMMdd-HHmmss"
$reportPath = Join-Path $ReportDir "android-device-$timestamp.json"
$sdkRoot = if ($env:ANDROID_HOME) { $env:ANDROID_HOME } else { Join-Path $env:LOCALAPPDATA "Android\Sdk" }
$adb = Join-Path $sdkRoot "platform-tools\adb.exe"
$aapt2 = Join-Path $sdkRoot "build-tools\36.0.0\aapt2.exe"
$apksigner = Join-Path $sdkRoot "build-tools\36.0.0\apksigner.bat"
$apk = Join-Path $repoRoot "android\app\build\outputs\apk\debug\app-debug.apk"

function Invoke-Adb {
    param([string[]]$AdbArgs)

    $base = @()
    if ($Serial) {
        $base += @("-s", $Serial)
    }
    & $adb @base @AdbArgs
}

function Get-DeviceValue {
    param([string]$PropertyName)

    try {
        (Invoke-Adb -AdbArgs @("shell", "getprop", $PropertyName) | Select-Object -First 1).Trim()
    } catch {
        $null
    }
}

if (-not (Test-Path $adb)) {
    throw "adb not found at $adb"
}
if (-not $SkipBuild) {
    & (Join-Path $PSScriptRoot "build-android.ps1")
}
if (-not (Test-Path $apk)) {
    throw "APK not found at $apk"
}

$verifyOutput = if (Test-Path $apksigner) {
    & $apksigner verify --verbose $apk 2>&1
} else {
    @("apksigner not found at $apksigner")
}

$badging = if (Test-Path $aapt2) {
    & $aapt2 dump badging $apk 2>&1
} else {
    @("aapt2 not found at $aapt2")
}

& $adb start-server | Out-Null
$devicesRaw = @(& $adb devices -l)
$deviceLines = @($devicesRaw | Where-Object { $_ -match "\bdevice\b" -and $_ -notmatch "^List of devices" })
$selectedDevice = $null

if ($Serial) {
    $selectedDevice = $deviceLines | Where-Object { $_ -like "$Serial*" } | Select-Object -First 1
} else {
    $selectedDevice = $deviceLines | Select-Object -First 1
}

$deviceInfo = $null
$installOutput = @()
$displayOutput = @()
$packageOutput = @()

if ($selectedDevice) {
    $deviceInfo = [PSCustomObject]@{
        serial = (($selectedDevice -split "\s+")[0])
        manufacturer = Get-DeviceValue "ro.product.manufacturer"
        model = Get-DeviceValue "ro.product.model"
        device = Get-DeviceValue "ro.product.device"
        sdk = Get-DeviceValue "ro.build.version.sdk"
        release = Get-DeviceValue "ro.build.version.release"
        oneUi = Get-DeviceValue "ro.build.version.oneui"
        dexMode = Get-DeviceValue "persist.service.dex.mode"
    }

    $displayOutput = @(Invoke-Adb -AdbArgs @("shell", "dumpsys", "display") 2>&1)

    if ($Install) {
        $installOutput = @(Invoke-Adb -AdbArgs @("install", "-r", $apk) 2>&1)
    }

    $packageOutput = @(Invoke-Adb -AdbArgs @("shell", "cmd", "package", "resolve-activity", "--brief", "com.xreal.parallax") 2>&1)
} else {
    $installOutput = @("No ADB device is attached or authorized. Connect the Fold7 with USB debugging enabled.")
}

$report = [PSCustomObject]@{
    generatedAt = (Get-Date).ToString("o")
    repoRoot = "$repoRoot"
    gitCommit = (git rev-parse HEAD)
    apk = "$((Resolve-Path $apk))"
    apkSizeBytes = (Get-Item $apk).Length
    apkVerify = @($verifyOutput)
    apkBadging = @($badging | Select-String -Pattern "package:|sdkVersion|targetSdkVersion|application-label|launchable-activity" | ForEach-Object { $_.Line })
    adbDevices = @($devicesRaw)
    selectedDevice = $deviceInfo
    installed = ($installOutput -match "Success").Count -gt 0
    installOutput = @($installOutput)
    displaySummary = @($displayOutput | Select-String -Pattern "DisplayDeviceInfo|mDisplayId|uniqueId|address|state|mode|FLAG_PRESENTATION" | Select-Object -First 80 | ForEach-Object { $_.Line })
    packageOutput = @($packageOutput)
}

$report | ConvertTo-Json -Depth 8 | Set-Content -Path $reportPath -Encoding UTF8

Write-Host "Android diagnostics written: $reportPath"
Write-Host "APK: $apk"
Write-Host "ADB device attached: $([bool]$selectedDevice)"
if ($deviceInfo) {
    Write-Host "Device: $($deviceInfo.manufacturer) $($deviceInfo.model), Android $($deviceInfo.release), SDK $($deviceInfo.sdk)"
}
if ($Install) {
    Write-Host "Install success: $($report.installed)"
}
