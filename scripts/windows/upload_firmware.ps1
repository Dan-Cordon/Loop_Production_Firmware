# upload_firmware.ps1
param (
    [Parameter(Mandatory=$true)]
    [ValidateSet("LCM", "MCM", "TWM")]
    [string]$Project,

    [Parameter(Mandatory=$true)]
    [string]$Port
)

$ErrorActionPreference = "Stop"
$scriptPath = $PSScriptRoot
$cliPath = Join-Path $scriptPath "tools\arduino-cli.exe"
$configPath = Join-Path $scriptPath "arduino-cli.yaml"

# --- CONFIGURATION ---
$FQBN = "esp32:esp32:esp32s3"

# Project Paths (Relative to script location in scripts/windows/)
$projects = @{
    "LCM" = Join-Path $scriptPath "..\..\LCM_FullMotorControl_V21\LCM_FullMotorControl_V21.ino"
    "MCM" = Join-Path $scriptPath "..\..\MCM_V16\MCM_V16.ino"
    "TWM" = Join-Path $scriptPath "..\..\TWM_GEM_V3_IMU\TWM_GEM_V3_IMU.ino"
}

if (-not (Test-Path $cliPath)) {
    Write-Error "arduino-cli not found. Please run setup_env.ps1 first."
}

$path = $projects[$Project]
$fullPath = (Resolve-Path $path).Path

Write-Host "--------------------------------------------------"
Write-Host "Uploading $Project..."
Write-Host "Path: $fullPath"
Write-Host "Port: $Port"
Write-Host "FQBN: $FQBN"
Write-Host "--------------------------------------------------"

$cmd = "& `"$cliPath`" compile --upload --fqbn $FQBN --port $Port --config-file `"$configPath`" `"$fullPath`""
Invoke-Expression $cmd

if ($LASTEXITCODE -eq 0) {
    Write-Host "Upload SUCCESS: $Project" -ForegroundColor Green
} else {
    Write-Host "Upload FAILED: $Project" -ForegroundColor Red
    exit 1
}