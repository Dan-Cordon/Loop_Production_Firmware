#!/bin/bash
set -e

PROJECT=$1
PORT=$2

if [ -z "$PROJECT" ] || [ -z "$PORT" ]; then
    echo "Usage: ./upload_firmware.sh <Project> <Port>"
    echo "Example: ./upload_firmware.sh LCM /dev/ttyACM0"
    exit 1
fi

SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
CLI_PATH="$SCRIPT_DIR/tools/arduino-cli"
CONFIG_PATH="$SCRIPT_DIR/arduino-cli.yaml"
FQBN="esp32:esp32:esp32s3"

# Define Paths
LCM_PATH="$SCRIPT_DIR/../../LCM_FullMotorControl_V21/LCM_FullMotorControl_V21.ino"
MCM_PATH="$SCRIPT_DIR/../../MCM_V16/MCM_V16.ino"
TWM_PATH="$SCRIPT_DIR/../../TWM_GEM_V3_IMU/TWM_GEM_V3_IMU.ino"

TARGET_PATH=""

if [ "$PROJECT" == "LCM" ]; then
    TARGET_PATH="$LCM_PATH"
elif [ "$PROJECT" == "MCM" ]; then
    TARGET_PATH="$MCM_PATH"
elif [ "$PROJECT" == "TWM" ]; then
    TARGET_PATH="$TWM_PATH"
else
    echo "Unknown project: $PROJECT"
    exit 1
fi

if [ ! -f "$CLI_PATH" ]; then
    echo "arduino-cli not found. Please run setup_env.sh first."
    exit 1
fi

echo "--------------------------------------------------"
echo "Uploading $PROJECT..."
echo "Path: $TARGET_PATH"
echo "Port: $PORT"
echo "--------------------------------------------------"

"$CLI_PATH" compile --upload --fqbn "$FQBN" --port "$PORT" --config-file "$CONFIG_PATH" "$TARGET_PATH"

if [ $? -eq 0 ]; then
    echo -e "\e[32mUpload SUCCESS: $PROJECT\e[0m"
else
    echo -e "\e[31mUpload FAILED: $PROJECT\e[0m"
    exit 1
fi