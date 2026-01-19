#!/bin/bash
set -e

# Get the directory where this script is located
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
# Define project root relative to the script location (scripts/linux/)
PROJECT_ROOT="$SCRIPT_DIR/../.."

# Tool paths
TOOLS_DIR="$SCRIPT_DIR/tools"
CLI_PATH="$TOOLS_DIR/arduino-cli"
DATA_DIR="$SCRIPT_DIR/arduino_data"
CONFIG_PATH="$SCRIPT_DIR/arduino-cli.yaml"

# 1. Download arduino-cli
echo "Checking for arduino-cli..."
if [ ! -f "$CLI_PATH" ]; then
    echo "Downloading arduino-cli..."
    mkdir -p "$TOOLS_DIR"
    
    # Download latest release for Linux 64bit
    URL="https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Linux_64bit.tar.gz"
    
    # Use curl to download
    curl -L "$URL" -o "$TOOLS_DIR/arduino-cli.tar.gz"
    
    echo "Extracting..."
    tar -xzf "$TOOLS_DIR/arduino-cli.tar.gz" -C "$TOOLS_DIR"
    rm "$TOOLS_DIR/arduino-cli.tar.gz"
    
    echo "arduino-cli installed at $CLI_PATH"
else
    echo "arduino-cli already installed."
fi

# 2. Configure paths
echo "Configuring directories..."

# Create local config file
"$CLI_PATH" config init --dest-file "$CONFIG_PATH" --overwrite

# Verify config creation
if [ ! -f "$CONFIG_PATH" ]; then
    echo "Failed to create config file."
    exit 1
fi

# Update config to use local directories
"$CLI_PATH" config set directories.data "$DATA_DIR/data" --config-file "$CONFIG_PATH"
"$CLI_PATH" config set directories.downloads "$DATA_DIR/staging" --config-file "$CONFIG_PATH"
"$CLI_PATH" config set directories.user "$DATA_DIR/user" --config-file "$CONFIG_PATH"

# Set Network Timeout Env Var to 1 hour
export ARDUINO_NETWORK_TIMEOUT="3600s"
"$CLI_PATH" config set network.timeout 3600s --config-file "$CONFIG_PATH"

# 3. Update index
echo "Updating core index..."
"$CLI_PATH" core update-index --config-file "$CONFIG_PATH"

# 4. Install ESP32 Core
echo "Installing ESP32 core..."
"$CLI_PATH" config add board_manager.additional_urls https://espressif.github.io/arduino-esp32/package_esp32_index.json --config-file "$CONFIG_PATH"
"$CLI_PATH" core update-index --config-file "$CONFIG_PATH"
"$CLI_PATH" core install esp32:esp32 --config-file "$CONFIG_PATH"

# 5. Install Libraries
echo "Installing Libraries..."
# Added Adafruit BusIO to match Windows script
LIBRARIES=("Adafruit BNO055" "Adafruit Unified Sensor" "HX711" "Adafruit BusIO")

for lib in "${LIBRARIES[@]}"; do
    echo "Installing $lib..."
    "$CLI_PATH" lib install "$lib" --config-file "$CONFIG_PATH"
done

# 6. Final Verification
echo "Setup Complete! ESP32 Platform installed successfully."
echo "You can now run build_firmware.sh"