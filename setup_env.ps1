# setup_env.ps1
# Sets up the local Arduino CLI environment

$ErrorActionPreference = "Stop"

$scriptPath = $PSScriptRoot
$toolsDir = Join-Path $scriptPath "tools"
$cliPath = Join-Path $toolsDir "arduino-cli.exe"
$dataDir = Join-Path $scriptPath "arduino_data"

# 1. Download arduino-cli
Write-Host "Checking for arduino-cli..."
if (-not (Test-Path $cliPath)) {
    Write-Host "Downloading arduino-cli..."
    New-Item -ItemType Directory -Force -Path $toolsDir | Out-Null
    
    # Download latest release for Windows 64bit
    # Note: Hardcoding version 0.35.0 to ensure stability, or could fetch latest
    $url = "https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Windows_64bit.zip"
    $zipPath = Join-Path $toolsDir "arduino-cli.zip"
    
    Invoke-WebRequest -Uri $url -OutFile $zipPath
    
    Write-Host "Extracting..."
    Expand-Archive -Path $zipPath -DestinationPath $toolsDir -Force
    Remove-Item $zipPath
    
    Write-Host "arduino-cli installed at $cliPath"
} else {
    Write-Host "arduino-cli already installed."
}

# 2. Configure paths to keep it local (clean environment)
Write-Host "Configuring directories..."

# Create a local config file
$configPath = Join-Path $scriptPath "arduino-cli.yaml"
& $cliPath config init --dest-file $configPath --overwrite

# Verify config creation
if (-not (Test-Path $configPath)) {
    Write-Error "Failed to create config file."
}

# Update config to use local directories
& $cliPath config set directories.data (Join-Path $dataDir "data") --config-file $configPath
& $cliPath config set directories.downloads (Join-Path $dataDir "staging") --config-file $configPath
& $cliPath config set directories.user (Join-Path $dataDir "user") --config-file $configPath

# Set Network Timeout Env Var to 1 hour to prevent timeouts on slow connections
$env:ARDUINO_NETWORK_TIMEOUT = "3600s"
& $cliPath config set network.timeout 3600s --config-file $configPath

# 3. Update index
Write-Host "Updating core index..."
& $cliPath core update-index --config-file $configPath

# 4. Install ESP32 Core
Write-Host "Installing ESP32 core..."
# First, need to add the URL to the config
& $cliPath config add board_manager.additional_urls https://espressif.github.io/arduino-esp32/package_esp32_index.json --config-file $configPath
& $cliPath core update-index --config-file $configPath

& $cliPath core install esp32:esp32 --config-file $configPath

# 5. Install Libraries
Write-Host "Installing Libraries..."
$libraries = @(
    "Adafruit BNO055",
    "Adafruit Unified Sensor",
    "HX711",
    "Adafruit BusIO"
)

foreach ($lib in $libraries) {
    Write-Host "Installing $lib..."
    & $cliPath lib install "$lib" --config-file $configPath
}

# 6. Final Verification
$coreList = & $cliPath core list --config-file $configPath
if ($coreList -match "esp32:esp32") {
    Write-Host "Setup Complete! ESP32 Platform installed successfully." -ForegroundColor Green
    Write-Host "You can now run build_firmware.ps1" -ForegroundColor Cyan
} else {
    Write-Host "Setup Failed: ESP32 Platform was NOT installed correctly." -ForegroundColor Red
    Write-Host "Please check your internet connection and try running this script again." -ForegroundColor Yellow
}
