$ErrorActionPreference = "Stop"

$repoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
$env:Path = "$env:USERPROFILE\.cargo\bin;$env:Path"

Push-Location $repoRoot
try {
    cargo build --release
    $exe = Resolve-Path "target\release\xreal-app.exe"
    Write-Host "Windows app built: $exe"
} finally {
    Pop-Location
}
