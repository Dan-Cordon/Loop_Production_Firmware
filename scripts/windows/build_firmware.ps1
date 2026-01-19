# build_firmware.ps1
param (
    [Parameter(Mandatory=$false)]
    [ValidateSet("LCM", "MCM", "TWM", "ALL")]
    [string]$Project = "ALL"
)

$ErrorActionPreference = "Stop"
$scriptPath = $PSScriptRoot
$cliPath = Join-Path $scriptPath "tools\arduino-cli.exe"
$configPath = Join-Path $scriptPath "arduino-cli.yaml"

# --- CONFIGURATION ---
$FQBN = "esp32:esp32:esp32s3"

# Project Paths (Adjusted to assume scripts run from scripts/windows/)
$projects = @{
    "LCM" = Join-Path $scriptPath "..\..\LCM_FullMotorControl_V21\LCM_FullMotorControl_V21.ino"
    "MCM" = Join-Path $scriptPath "..\..\MCM_V16\MCM_V16.ino"
    "TWM" = Join-Path $scriptPath "..\..\TWM_GEM_V3_IMU\TWM_GEM_V3_IMU.ino"
}

if (-not (Test-Path $cliPath)) {
    Write-Error "arduino-cli not found. Please run setup_env.ps1 first."
}

function Build-Project {
    param ($name, $path)
    Write-Host "--------------------------------------------------"
    Write-Host "Building $name..."
    Write-Host "Path: $path"
    Write-Host "FQBN: $FQBN"
    Write-Host "--------------------------------------------------"

    # Resolve full path to avoid relative path issues with arduino-cli
    $fullPath = (Resolve-Path $path).Path

    $cmd = "& `"$cliPath`" compile --fqbn $FQBN --config-file `"$configPath`" `"$fullPath`""
    Invoke-Expression $cmd
    
    if ($LASTEXITCODE -eq 0) {
        Write-Host "Build SUCCESS: $name" -ForegroundColor Green
    } else {
        Write-Host "Build FAILED: $name" -ForegroundColor Red
        exit 1
    }
}

if ($Project -eq "ALL") {
    foreach ($key in $projects.Keys) {
        Build-Project -name $key -path $projects[$key]
    }
} else {
    Build-Project -name $Project -path $projects[$Project]
}