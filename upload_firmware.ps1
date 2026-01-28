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
$FQBN = "esp32:esp32:esp32s3:CDCOnBoot=cdc"

# Project Paths
$projects = @{
    "LCM" = Join-Path $scriptPath "LCM_FullMotorControl_V21\LCM_FullMotorControl_V21.ino"
    "MCM" = Join-Path $scriptPath "MCM_V16\MCM_V16.ino"
    "TWM" = Join-Path $scriptPath "TWM_GEM_V3_IMU\TWM_GEM_V3_IMU.ino"
}

if (-not (Test-Path $cliPath)) {
    Write-Error "arduino-cli not found. Please run setup_env.ps1 first."
}

$path = $projects[$Project]

Write-Host "--------------------------------------------------"
Write-Host "Uploading $Project..."
Write-Host "Path: $path"
Write-Host "Port: $Port"
Write-Host "FQBN: $FQBN"
Write-Host "--------------------------------------------------"

# Re-compile slightly fast to ensure we have the build artifacts, then upload
# Note: --input-dir could be used if we saved binaries, but standard upload usually triggers compile check
# Using 'upload' command. 
# Sometimes 'compile --upload' is used, but separate upload is cleaner if built.
# However, for Arduino CLI, 'upload' usually expects a previous build in the temp dir unless we specified an output dir.
# To be safe and simple, we run compile then upload, or just compile --upload.
# Let's use compile --upload to ensure it's in sync.

$cmd = "& `"$cliPath`" compile --upload --fqbn $FQBN --port $Port --config-file `"$configPath`" `"$path`""
Invoke-Expression $cmd

if ($LASTEXITCODE -eq 0) {
    Write-Host "Upload SUCCESS: $Project" -ForegroundColor Green
} else {
    Write-Host "Upload FAILED: $Project" -ForegroundColor Red
    exit 1
}
