# install_libraries.ps1
# Installs required Arduino libraries using arduino-cli

$ErrorActionPreference = "Stop"

$scriptPath = $PSScriptRoot
$cliPath = Join-Path $scriptPath "tools\arduino-cli.exe"
$configPath = Join-Path $scriptPath "arduino-cli.yaml"

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "Installing Required Libraries" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

# Check if arduino-cli exists
if (-not (Test-Path $cliPath)) {
    Write-Host "[ERROR] arduino-cli not found!" -ForegroundColor Red
    Write-Host "Please run setup_env.ps1 first" -ForegroundColor Yellow
    exit 1
}

# Update library index
Write-Host "[UPDATE] Refreshing library index..." -ForegroundColor Cyan
& $cliPath lib update-index --config-file $configPath

# Install libraries
$libraries = @(
    "Adafruit BNO055",
    "Adafruit Unified Sensor",
    "HX711",
    "Adafruit BusIO"
)

foreach ($lib in $libraries) {
    Write-Host ""
    Write-Host "[INSTALL] $lib..." -ForegroundColor Cyan
    & $cliPath lib install "$lib" --config-file $configPath
    
    if ($LASTEXITCODE -eq 0) {
        Write-Host "[OK] $lib installed" -ForegroundColor Green
    } else {
        Write-Host "[ERROR] Failed to install $lib" -ForegroundColor Red
    }
}

Write-Host ""
Write-Host "========================================" -ForegroundColor Green
Write-Host "Library Installation Complete!" -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Green
Write-Host ""
Write-Host "You can now build firmware:" -ForegroundColor Cyan
Write-Host "  .\build_firmware.ps1 -Project LCM" -ForegroundColor White
Write-Host ""