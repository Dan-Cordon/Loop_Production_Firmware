![Windows](https://img.shields.io/badge/OS-Windows-blue?logo=windows)
![Linux](https://img.shields.io/badge/OS-Linux-orange?logo=linux)
# Arduino CLI Guide for Loop Production Firmware

This guide explains how to use the provided PowerShell scripts to compile and upload firmware for the Loop Production boards (LCM, MCM, TWM) using `arduino-cli`.

## Prerequisites

### 🪟 Windows
1.  **PowerShell**: Pre-installed on Windows 10/11.
2.  **Drivers**: USB drivers for ESP32-S3 (often automatic).

### 🐧 Linux
1.  **Bash**: Standard terminal.
2.  **Permissions**: User must be in the `dialout` group for USB upload access:
    ```bash
    sudo usermod -a -G dialout $USER
    ```
    *(Log out and back in after running this)*
## Setup

Before building or uploading for the first time, you must set up the environment.

1.  Open PowerShell in this directory.
2.  Run the setup script:

```powershell
.\setup_env.ps1
```

This script will:
- Download `arduino-cli.exe` into a local `tools` folder.
- Initialize the Arduino CLI config.
- Install the ESP32 board core (`esp32:esp32`).
- Install required libraries:
    - `Adafruit BNO055`
    - `Adafruit Unified Sensor`
    - `HX711`
    - `Adafruit BusIO`

> [!NOTE]
> If you see a security warning running scripts, you may need to allow execution:
> `Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser`

## Building Firmware

To compile a firmware project:

```powershell
.\build_firmware.ps1 -Project <ProjectName>
```

**Available Projects:**
- `LCM` (LCM_FullMotorControl_V21)
- `MCM` (MCM_V16)
- `TWM` (TWM_GEM_V3_IMU)
- `ALL` (Builds all three)

**Example:**
```powershell
.\build_firmware.ps1 -Project LCM
```

## Uploading Firmware

To upload the firmware to a board:

1.  Connect the board via USB.
2.  Identify the COM port (e.g., `COM3`, `COM4`).
3.  Run the upload script:

```powershell
.\upload_firmware.ps1 -Project <ProjectName> -Port <COMPort>
```

**Example:**
```powershell
.\upload_firmware.ps1 -Project LCM -Port COM3
```

> [!IMPORTANT]
> The scripts use the Fully Qualified Board Name (FQBN): `esp32:esp32:esp32s3`.
> If your custom board requires specific build flags or partition schemes, edit the `$FQBN` variable in the scripts.

## Directory Structure
- `tools/`: Contains the `arduino-cli` executable (created after setup).
- `arduino_data/`: Contains downloaded cores and libraries (created after setup).

## Troubleshooting

### Download Timeouts During Setup

If `setup_env.ps1` fails with timeout errors during ESP32 core installation, run the robust download script:

```powershell
.\robust_download.ps1
```

This script uses Windows BITS (Background Intelligent Transfer Service) which supports resume for large downloads. It will pre-download the ESP32 toolchain files and then complete the installation.

