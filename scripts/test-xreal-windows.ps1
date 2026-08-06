param(
    [int]$LaunchSeconds = 12,
    [switch]$SkipBuild,
    [switch]$SkipLaunch,
    [switch]$KeepRunning,
    [string]$ReportDir
)

$ErrorActionPreference = "Stop"

$repoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
if (-not $ReportDir) {
    $ReportDir = Join-Path $repoRoot "diagnostics"
}
New-Item -ItemType Directory -Path $ReportDir -Force | Out-Null

$timestamp = Get-Date -Format "yyyyMMdd-HHmmss"
$stdoutLog = Join-Path $ReportDir "windows-xreal-$timestamp.stdout.log"
$stderrLog = Join-Path $ReportDir "windows-xreal-$timestamp.stderr.log"
$reportPath = Join-Path $ReportDir "windows-xreal-$timestamp.json"

function Get-MonitorName {
    param($Monitor)

    $chars = $Monitor.UserFriendlyName | Where-Object { $_ -ne 0 } | ForEach-Object { [char]$_ }
    -join $chars
}

function Get-ScreenList {
    try {
        Add-Type -AssemblyName System.Windows.Forms
        [System.Windows.Forms.Screen]::AllScreens | ForEach-Object {
            [PSCustomObject]@{
                deviceName = $_.DeviceName
                primary = $_.Primary
                x = $_.Bounds.X
                y = $_.Bounds.Y
                width = $_.Bounds.Width
                height = $_.Bounds.Height
            }
        }
    } catch {
        @([PSCustomObject]@{ error = $_.Exception.Message })
    }
}

function Test-TcpPort {
    param(
        [string]$HostName,
        [int]$Port,
        [int]$TimeoutMs = 2000
    )

    $client = [System.Net.Sockets.TcpClient]::new()
    try {
        $result = $client.BeginConnect($HostName, $Port, $null, $null)
        $connected = $result.AsyncWaitHandle.WaitOne($TimeoutMs, $false)
        if (-not $connected) {
            return [PSCustomObject]@{
                host = $HostName
                port = $Port
                reachable = $false
                error = "timeout after ${TimeoutMs}ms"
            }
        }

        $client.EndConnect($result)
        [PSCustomObject]@{
            host = $HostName
            port = $Port
            reachable = $true
            error = $null
        }
    } catch {
        [PSCustomObject]@{
            host = $HostName
            port = $Port
            reachable = $false
            error = $_.Exception.Message
        }
    } finally {
        $client.Close()
    }
}

Push-Location $repoRoot
try {
    $env:Path = "$env:USERPROFILE\.cargo\bin;$env:Path"

    if (-not $SkipBuild) {
        & (Join-Path $PSScriptRoot "build-windows.ps1")
    }

    $exe = Join-Path $repoRoot "target\release\xreal-app.exe"
    $usbDevices = Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue |
        Where-Object {
            $_.InstanceId -match "VID_3318|PID_0438|XREAL|NREAL" -or
            $_.FriendlyName -match "XREAL|Nreal|CDC NCM|CDC ECM"
        } |
        Select-Object Class, FriendlyName, InstanceId, Status

    $pnpDisplays = Get-PnpDevice -PresentOnly -Class Display -ErrorAction SilentlyContinue |
        Select-Object Class, FriendlyName, InstanceId, Status

    $wmiDisplays = Get-CimInstance -Namespace root\wmi -ClassName WmiMonitorID -ErrorAction SilentlyContinue |
        ForEach-Object {
            [PSCustomObject]@{
                name = Get-MonitorName $_
                instanceName = $_.InstanceName
            }
        }

    $screens = @(Get-ScreenList)
    $candidateScreens = $screens | Where-Object {
        ($_.width -eq 3840 -and ($_.height -eq 1080 -or $_.height -eq 1200)) -or
        ($_.width -eq 1920 -and ($_.height -eq 1080 -or $_.height -eq 1200))
    }

    $networkAdapters = Get-NetAdapter -ErrorAction SilentlyContinue |
        Where-Object { $_.Name -match "XREAL|Nreal|CDC|NCM|ECM|USB" -or $_.InterfaceDescription -match "XREAL|Nreal|CDC|NCM|ECM|USB" } |
        Select-Object Name, InterfaceDescription, Status, LinkSpeed, MacAddress

    $tcpImu = Test-TcpPort -HostName "169.254.2.1" -Port 52998
    $processInfo = $null

    if (-not $SkipLaunch) {
        if (-not (Test-Path $exe)) {
            throw "Windows executable was not found at $exe"
        }

        Remove-Item -LiteralPath $stdoutLog, $stderrLog -ErrorAction SilentlyContinue
        $env:RUST_LOG = "xreal_app=info,xreal_imu=info,xreal_renderer=info,wgpu_hal=warn"
        $process = Start-Process -FilePath $exe -WorkingDirectory $repoRoot -WindowStyle Hidden -RedirectStandardOutput $stdoutLog -RedirectStandardError $stderrLog -PassThru
        Start-Sleep -Seconds $LaunchSeconds
        $wasRunning = -not $process.HasExited

        if ($wasRunning -and -not $KeepRunning) {
            Stop-Process -Id $process.Id -Force
            Wait-Process -Id $process.Id -ErrorAction SilentlyContinue
        }

        $processInfo = [PSCustomObject]@{
            started = $true
            processId = $process.Id
            wasRunningAfterSeconds = $wasRunning
            keptRunning = [bool]$KeepRunning
            exitCode = if ($process.HasExited) { $process.ExitCode } else { $null }
            stdoutLog = $stdoutLog
            stderrLog = $stderrLog
            stderrTail = if (Test-Path $stderrLog) { @(Get-Content $stderrLog -Tail 40) } else { @() }
        }
    }

    $report = [PSCustomObject]@{
        generatedAt = (Get-Date).ToString("o")
        repoRoot = "$repoRoot"
        gitCommit = (git rev-parse HEAD)
        windowsExe = if (Test-Path $exe) { "$((Resolve-Path $exe))" } else { $null }
        xrealUsbDevicePresent = @($usbDevices).Count -gt 0
        candidateDisplayPresent = @($candidateScreens).Count -gt 0
        screens = @($screens)
        candidateScreens = @($candidateScreens)
        pnpDisplays = @($pnpDisplays)
        wmiDisplays = @($wmiDisplays)
        usbDevices = @($usbDevices)
        networkAdapters = @($networkAdapters)
        tcpImu = $tcpImu
        appLaunch = $processInfo
    }

    $report | ConvertTo-Json -Depth 8 | Set-Content -Path $reportPath -Encoding UTF8

    Write-Host "Windows XREAL diagnostics written: $reportPath"
    Write-Host "USB device present: $($report.xrealUsbDevicePresent)"
    Write-Host "Candidate display present: $($report.candidateDisplayPresent)"
    Write-Host "IMU TCP reachable: $($report.tcpImu.reachable)"
    if ($processInfo) {
        Write-Host "App running after $LaunchSeconds seconds: $($processInfo.wasRunningAfterSeconds)"
        Write-Host "App stderr log: $stderrLog"
    }
} finally {
    Pop-Location
}
