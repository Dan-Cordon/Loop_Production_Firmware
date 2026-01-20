# robust_download.ps1
# Downloads ESP32 toolchain files using BITS for reliable, resumable downloads
# Then runs arduino-cli to complete the ESP32 core installation

$ErrorActionPreference = "Stop"

$scriptPath = $PSScriptRoot
$stagingDir = Join-Path $scriptPath "arduino_data\staging\packages"
$cliPath = Join-Path $scriptPath "tools\arduino-cli.exe"
$configPath = Join-Path $scriptPath "arduino-cli.yaml"

# Create staging directory if it doesn't exist
New-Item -ItemType Directory -Force -Path $stagingDir | Out-Null

# --- Cleanup: Remove any 0KB or invalid files from previous failed runs ---
$longName = Join-Path $stagingDir "esp32-arduino-libs-idf-release_v5.5-9bb7aa84-v2.zip"
if (Test-Path $longName) {
    if ((Get-Item $longName).Length -lt 1000) {
        Write-Host "Removing invalid partial download..." -ForegroundColor Yellow
        Remove-Item $longName -Force
    }
}

# Define the files to download for ESP32 3.3.5 on Windows x64
$downloads = @(
    @{
        Name = "esp32-arduino-libs (Pre-compiled libraries)"
        # CORRECTED: Download the short name file, save as long name
        Url = "https://github.com/espressif/arduino-esp32/releases/download/3.3.5/esp32-3.3.5-libs.zip"
        FileName = "esp32-arduino-libs-idf-release_v5.5-9bb7aa84-v2.zip" 
        ExpectedSize = 509489500 
    },
    @{
        Name = "esp-x32 (Xtensa toolchain)"
        Url = "https://github.com/espressif/crosstool-NG/releases/download/esp-14.2.0_20251107/xtensa-esp-elf-14.2.0_20251107-x86_64-w64-mingw32.zip"
        FileName = "xtensa-esp-elf-14.2.0_20251107-x86_64-w64-mingw32.zip"
        ExpectedSize = 396056136
    },
    @{
        Name = "esp-rv32 (RISC-V toolchain)"
        Url = "https://github.com/espressif/crosstool-NG/releases/download/esp-14.2.0_20251107/riscv32-esp-elf-14.2.0_20251107-x86_64-w64-mingw32.zip"
        FileName = "riscv32-esp-elf-14.2.0_20251107-x86_64-w64-mingw32.zip"
        ExpectedSize = 697522467
    },
    @{
        Name = "xtensa-esp-elf-gdb (Xtensa debugger)"
        Url = "https://github.com/espressif/binutils-gdb/releases/download/esp-gdb-v16.3_20250913/xtensa-esp-elf-gdb-16.3_20250913-x86_64-w64-mingw32.zip"
        FileName = "xtensa-esp-elf-gdb-16.3_20250913-x86_64-w64-mingw32.zip"
        ExpectedSize = 35138541
    },
    @{
        Name = "riscv32-esp-elf-gdb (RISC-V debugger)"
        Url = "https://github.com/espressif/binutils-gdb/releases/download/esp-gdb-v16.3_20250913/riscv32-esp-elf-gdb-16.3_20250913-x86_64-w64-mingw32.zip"
        FileName = "riscv32-esp-elf-gdb-16.3_20250913-x86_64-w64-mingw32.zip"
        ExpectedSize = 34196400
    },
    @{
        Name = "openocd-esp32"
        Url = "https://github.com/espressif/openocd-esp32/releases/download/v0.12.0-esp32-20250707/openocd-esp32-win64-0.12.0-esp32-20250707.zip"
        FileName = "openocd-esp32-win64-0.12.0-esp32-20250707.zip"
        ExpectedSize = 4000000 
    },
    @{
        Name = "esptool_py"
        Url = "https://github.com/espressif/esptool/releases/download/v5.1.0/esptool-v5.1.0-windows-amd64.zip"
        FileName = "esptool-v5.1.0-windows-amd64.zip"
        ExpectedSize = 5000000 
    }
)

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "ESP32 Toolchain Robust Download Script" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

# Download each file using BITS
foreach ($dl in $downloads) {
    $targetFile = Join-Path $stagingDir $dl.FileName
    
    if (Test-Path $targetFile) {
        $fileSize = (Get-Item $targetFile).Length
        # Simple check: if file is > 1KB we assume it's valid or partial
        if ($fileSize -gt 1000) {
            # For huge files, check if close to expected size (allow 10% variance)
            if ($fileSize -gt ($dl.ExpectedSize * 0.9)) {
                Write-Host "[OK] $($dl.Name) already downloaded ($([math]::Round($fileSize / 1MB, 1)) MB)" -ForegroundColor Green
                continue
            }
        }
        Write-Host "[PARTIAL] $($dl.Name) incomplete ($([math]::Round($fileSize / 1MB, 1)) MB), re-downloading..." -ForegroundColor Yellow
        Remove-Item $targetFile -Force
    }
    
    Write-Host "[DOWNLOADING] $($dl.Name)..." -ForegroundColor Cyan
    Write-Host "    URL: $($dl.Url)"
    
    try {
        # Use BITS for reliable download with resume support
        $job = Start-BitsTransfer -Source $dl.Url -Destination $targetFile -Asynchronous -DisplayName $dl.Name
        
        # Monitor progress
        while ($job.JobState -eq "Transferring" -or $job.JobState -eq "Connecting") {
            $percent = 0
            if ($job.BytesTotal -gt 0) {
                $percent = [math]::Round(($job.BytesTransferred / $job.BytesTotal) * 100, 1)
            }
            Write-Progress -Activity "Downloading $($dl.Name)" -Status "$percent% Complete" -PercentComplete $percent
            Start-Sleep -Milliseconds 500
        }
        
        Write-Progress -Activity "Downloading $($dl.Name)" -Completed
        
        if ($job.JobState -eq "Transferred") {
            Complete-BitsTransfer -BitsJob $job
            Write-Host "[OK] $($dl.Name) downloaded successfully" -ForegroundColor Green
        } else {
            Write-Host "[ERROR] $($dl.Name) failed: $($job.JobState)" -ForegroundColor Red
            # Try alternative download with Invoke-WebRequest
            Write-Host "    Trying alternative download method..." -ForegroundColor Yellow
            Remove-BitsTransfer -BitsJob $job -ErrorAction SilentlyContinue
            Invoke-WebRequest -Uri $dl.Url -OutFile $targetFile -TimeoutSec 7200
            Write-Host "[OK] $($dl.Name) downloaded via alternative method" -ForegroundColor Green
        }
    } catch {
        Write-Host "[ERROR] Failed to download $($dl.Name): $_" -ForegroundColor Red
        Write-Host "    Trying alternative download method..." -ForegroundColor Yellow
        try {
            Invoke-WebRequest -Uri $dl.Url -OutFile $targetFile -TimeoutSec 7200
            Write-Host "[OK] $($dl.Name) downloaded via alternative method" -ForegroundColor Green
        } catch {
            Write-Host "[FATAL] Could not download $($dl.Name). Please download manually:" -ForegroundColor Red
            Write-Host "    $($dl.Url)" -ForegroundColor Yellow
            Write-Host "    Save to: $targetFile" -ForegroundColor Yellow
        }
    }
}

Write-Host ""
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "Running arduino-cli core install..." -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan

# Set extended timeout
$env:ARDUINO_NETWORK_TIMEOUT = "7200s"

# Run arduino-cli to complete installation
& $cliPath core install esp32:esp32 --config-file $configPath

if ($LASTEXITCODE -eq 0) {
    Write-Host ""
    Write-Host "========================================" -ForegroundColor Green
    Write-Host "SUCCESS! ESP32 core installed!" -ForegroundColor Green
    Write-Host "========================================" -ForegroundColor Green
    Write-Host ""
    Write-Host "You can now build firmware with:" -ForegroundColor Cyan
    Write-Host "  .\build_firmware.ps1 -Project LCM" -ForegroundColor White
    Write-Host ""
} else {
    Write-Host ""
    Write-Host "========================================" -ForegroundColor Red
    Write-Host "Installation may have failed." -ForegroundColor Red
    Write-Host "========================================" -ForegroundColor Red
    Write-Host "Check the output above for errors." -ForegroundColor Yellow
    Write-Host "You may need to run this script again." -ForegroundColor Yellow
}