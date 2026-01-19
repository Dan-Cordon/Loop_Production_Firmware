#!/bin/bash
# robust_download.sh
# Downloads ESP32 toolchain files for Linux using wget (resumable)

set -e

SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
STAGING_DIR="$SCRIPT_DIR/arduino_data/staging/packages"
CLI_PATH="$SCRIPT_DIR/tools/arduino-cli"
CONFIG_PATH="$SCRIPT_DIR/arduino-cli.yaml"

# Create staging directory if it doesn't exist
mkdir -p "$STAGING_DIR"

# Define the files to download for ESP32 3.3.5 on Linux x64
# URLs updated to Linux .tar.gz versions
declare -A FILES
declare -A URLS

# 1. esp-x32 (Xtensa toolchain)
FILES["esp-x32"]="xtensa-esp-elf-14.2.0_20251107-x86_64-linux-gnu.tar.gz"
URLS["esp-x32"]="https://github.com/espressif/crosstool-NG/releases/download/esp-14.2.0_20251107/xtensa-esp-elf-14.2.0_20251107-x86_64-linux-gnu.tar.gz"

# 2. esp-rv32 (RISC-V toolchain)
FILES["esp-rv32"]="riscv32-esp-elf-14.2.0_20251107-x86_64-linux-gnu.tar.gz"
URLS["esp-rv32"]="https://github.com/espressif/crosstool-NG/releases/download/esp-14.2.0_20251107/riscv32-esp-elf-14.2.0_20251107-x86_64-linux-gnu.tar.gz"

# 3. xtensa-esp-elf-gdb
FILES["xtensa-gdb"]="xtensa-esp-elf-gdb-16.3_20250913-x86_64-linux-gnu.tar.gz"
URLS["xtensa-gdb"]="https://github.com/espressif/binutils-gdb/releases/download/esp-gdb-v16.3_20250913/xtensa-esp-elf-gdb-16.3_20250913-x86_64-linux-gnu.tar.gz"

# 4. riscv32-esp-elf-gdb
FILES["riscv-gdb"]="riscv32-esp-elf-gdb-16.3_20250913-x86_64-linux-gnu.tar.gz"
URLS["riscv-gdb"]="https://github.com/espressif/binutils-gdb/releases/download/esp-gdb-v16.3_20250913/riscv32-esp-elf-gdb-16.3_20250913-x86_64-linux-gnu.tar.gz"

# 5. openocd-esp32
FILES["openocd"]="openocd-esp32-linux64-0.12.0-esp32-20250707.tar.gz"
URLS["openocd"]="https://github.com/espressif/openocd-esp32/releases/download/v0.12.0-esp32-20250707/openocd-esp32-linux64-0.12.0-esp32-20250707.tar.gz"

# 6. esptool_py
FILES["esptool"]="esptool-v5.1.0-linux-amd64.zip"
URLS["esptool"]="https://github.com/espressif/esptool/releases/download/v5.1.0/esptool-v5.1.0-linux-amd64.zip"


echo "========================================"
echo "ESP32 Toolchain Robust Download (Linux)"
echo "========================================"

for key in "${!FILES[@]}"; do
    FILENAME="${FILES[$key]}"
    URL="${URLS[$key]}"
    TARGET_FILE="$STAGING_DIR/$FILENAME"

    echo ""
    echo "[CHECKING] $key..."
    
    if [ -f "$TARGET_FILE" ]; then
        echo "   File exists. Checking if download is complete..."
        # Basic check: Use wget --continue to verify/finish
        wget -c "$URL" -O "$TARGET_FILE" --show-progress
    else
        echo "   Downloading $FILENAME..."
        echo "   URL: $URL"
        # -c allows resuming if connection drops
        wget -c "$URL" -O "$TARGET_FILE" --show-progress
    fi
    
    if [ $? -eq 0 ]; then
        echo "[OK] $key ready."
    else
        echo "[ERROR] Failed to download $key."
        exit 1
    fi
done

echo ""
echo "========================================"
echo "Running arduino-cli core install..."
echo "========================================"

export ARDUINO_NETWORK_TIMEOUT="7200s"
"$CLI_PATH" core install esp32:esp32 --config-file "$CONFIG_PATH"

if [ $? -eq 0 ]; then
    echo ""
    echo "========================================"
    echo "SUCCESS! ESP32 core installed!"
    echo "========================================"
    echo "You can now build firmware with:"
    echo "  ./build_firmware.sh -Project LCM"
else
    echo ""
    echo "========================================"
    echo "Installation FAILED."
    echo "========================================"
fi