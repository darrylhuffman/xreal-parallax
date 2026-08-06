$ErrorActionPreference = "Stop"

$repoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
$exe = Join-Path $repoRoot "target\release\xreal-app.exe"

if (-not (Test-Path $exe)) {
    & (Join-Path $PSScriptRoot "build-windows.ps1")
}

Start-Process -FilePath $exe -WorkingDirectory $repoRoot
