#!/bin/bash
set -e

PROJECT=$1
if [ -z "$PROJECT" ]; then
    PROJECT="ALL"
fi

SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
CLI_PATH="$SCRIPT_DIR/tools/arduino-cli"
CONFIG_PATH="$SCRIPT_DIR/arduino-cli.yaml"
FQBN="esp32:esp32:esp32s3"

# Define Paths relative to script location
LCM_PATH="$SCRIPT_DIR/../../LCM_FullMotorControl_V21/LCM_FullMotorControl_V21.ino"
MCM_PATH="$SCRIPT_DIR/../../MCM_V16/MCM_V16.ino"
TWM_PATH="$SCRIPT_DIR/../../TWM_GEM_V3_IMU/TWM_GEM_V3_IMU.ino"

if [ ! -f "$CLI_PATH" ]; then
    echo "arduino-cli not found. Please run setup_env.sh first."
    exit 1
fi

build_project() {
    NAME=$1
    PATH_TO_INO=$2
    
    echo "--------------------------------------------------"
    echo "Building $NAME..."
    echo "Path: $PATH_TO_INO"
    echo "--------------------------------------------------"

    "$CLI_PATH" compile --fqbn "$FQBN" --config-file "$CONFIG_PATH" "$PATH_TO_INO"
    
    if [ $? -eq 0 ]; then
        echo -e "\e[32mBuild SUCCESS: $NAME\e[0m"
    else
        echo -e "\e[31mBuild FAILED: $NAME\e[0m"
        exit 1
    fi
}

if [ "$PROJECT" == "ALL" ]; then
    build_project "LCM" "$LCM_PATH"
    build_project "MCM" "$MCM_PATH"
    build_project "TWM" "$TWM_PATH"
elif [ "$PROJECT" == "LCM" ]; then
    build_project "LCM" "$LCM_PATH"
elif [ "$PROJECT" == "MCM" ]; then
    build_project "MCM" "$MCM_PATH"
elif [ "$PROJECT" == "TWM" ]; then
    build_project "TWM" "$TWM_PATH"
else
    echo "Unknown project: $PROJECT. Use LCM, MCM, TWM, or ALL."
    exit 1
fi