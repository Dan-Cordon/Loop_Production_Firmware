# manual_esp32_install.ps1
# Manually installs ESP32 core by directly downloading and extracting to the right locations

$ErrorActionPreference = "Stop"

$scriptPath = $PSScriptRoot
$dataDir = Join-Path $scriptPath "arduino_data\data"
$packagesDir = Join-Path $dataDir "packages"
$esp32Dir = Join-Path $packagesDir "esp32"
$hardwareBaseDir = Join-Path $esp32Dir "hardware\esp32"
$coreVersion = "3.3.5"
$hardwareDir = Join-Path $hardwareBaseDir $coreVersion
$toolsDir = Join-Path $esp32Dir "tools"
$tempDir = Join-Path $scriptPath "temp_downloads"

# Create directories
New-Item -ItemType Directory -Force -Path $tempDir | Out-Null
New-Item -ItemType Directory -Force -Path $hardwareBaseDir | Out-Null
New-Item -ItemType Directory -Force -Path $toolsDir | Out-Null

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "Manual ESP32 Core Installation" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

# Function to download with progress
function Download-File {
    param($Url, $Output, $Name)
    
    if (Test-Path $Output) {
        Write-Host "[SKIP] $Name already exists" -ForegroundColor Yellow
        return $true
    }
    
    Write-Host "[DOWNLOAD] $Name..." -ForegroundColor Cyan
    Write-Host "    From: $Url"
    Write-Host "    To: $Output"
    
    try {
        $job = Start-BitsTransfer -Source $Url -Destination $Output -Asynchronous -DisplayName $Name
        
        while ($job.JobState -eq "Transferring" -or $job.JobState -eq "Connecting") {
            $percent = 0
            if ($job.BytesTotal -gt 0) {
                $percent = [math]::Round(($job.BytesTransferred / $job.BytesTotal) * 100, 1)
            }
            Write-Progress -Activity "Downloading $Name" -Status "$percent% Complete" -PercentComplete $percent
            Start-Sleep -Milliseconds 500
        }
        
        Write-Progress -Activity "Downloading $Name" -Completed
        
        if ($job.JobState -eq "Transferred") {
            Complete-BitsTransfer -BitsJob $job
            Write-Host "[OK] Downloaded successfully" -ForegroundColor Green
            return $true
        } else {
            Remove-BitsTransfer -BitsJob $job -ErrorAction SilentlyContinue
            throw "BITS transfer failed: $($job.JobState)"
        }
    } catch {
        Write-Host "[WARN] BITS failed: $_" -ForegroundColor Yellow
        Write-Host "[RETRY] Trying with Invoke-WebRequest..." -ForegroundColor Yellow
        Invoke-WebRequest -Uri $Url -OutFile $Output -TimeoutSec 7200
        Write-Host "[OK] Downloaded successfully" -ForegroundColor Green
        return $true
    }
}

# 1. Download ESP32 Arduino Core
Write-Host ""
Write-Host "Step 1: Downloading ESP32 Arduino Core..." -ForegroundColor Cyan
$coreUrl = "https://github.com/espressif/arduino-esp32/releases/download/$coreVersion/esp32-$coreVersion.zip"
$coreZip = Join-Path $tempDir "esp32-core.zip"

if (-not (Test-Path $hardwareDir)) {
    Download-File -Url $coreUrl -Output $coreZip -Name "ESP32 Arduino Core $coreVersion"
    
    Write-Host "[EXTRACT] Extracting core to $hardwareDir..." -ForegroundColor Cyan
    # Extract with error handling for long paths
    try {
        Expand-Archive -Path $coreZip -DestinationPath $hardwareDir -Force -ErrorAction Stop
    } catch {
        Write-Host "[WARN] Standard extraction had issues, using robocopy method..." -ForegroundColor Yellow
        # Alternative: Extract to temp and copy
        $tempExtract = Join-Path $tempDir "esp32_extracted"
        Expand-Archive -Path $coreZip -DestinationPath $tempExtract -Force -ErrorAction SilentlyContinue
        robocopy $tempExtract $hardwareDir /E /NFL /NDL /NJH /NJS /nc /ns /np
        Remove-Item $tempExtract -Recurse -Force -ErrorAction SilentlyContinue
    }
    Write-Host "[OK] Core extracted" -ForegroundColor Green
} else {
    Write-Host "[SKIP] ESP32 core already installed" -ForegroundColor Yellow
}

# 2. Download and install tools (simplified - only essential ones)
Write-Host ""
Write-Host "Step 2: Downloading Toolchains (this may take a while)..." -ForegroundColor Cyan

$tools = @(
    @{
        Name = "Xtensa Toolchain"
        Url = "https://github.com/espressif/crosstool-NG/releases/download/esp-14.2.0_20251107/xtensa-esp-elf-14.2.0_20251107-x86_64-w64-mingw32.zip"
        ToolDir = "xtensa-esp-elf"
        Version = "esp-14.2.0_20251107"
    },
    @{
        Name = "RISC-V Toolchain"  
        Url = "https://github.com/espressif/crosstool-NG/releases/download/esp-14.2.0_20251107/riscv32-esp-elf-14.2.0_20251107-x86_64-w64-mingw32.zip"
        ToolDir = "riscv32-esp-elf"
        Version = "esp-14.2.0_20251107"
    },
    @{
        Name = "esptool"
        Url = "https://github.com/espressif/esptool/releases/download/v5.1.0/esptool-v5.1.0-windows-amd64.zip"
        ToolDir = "esptool_py"
        Version = "5.1.0"
    }
)

foreach ($tool in $tools) {
    $toolVersionDir = Join-Path $toolsDir "$($tool.ToolDir)\$($tool.Version)"
    
    if (-not (Test-Path $toolVersionDir)) {
        $toolZip = Join-Path $tempDir "$($tool.ToolDir).zip"
        
        Download-File -Url $tool.Url -Output $toolZip -Name $tool.Name
        
        Write-Host "[EXTRACT] Extracting $($tool.Name)..." -ForegroundColor Cyan
        New-Item -ItemType Directory -Force -Path $toolVersionDir | Out-Null
        
        try {
            Expand-Archive -Path $toolZip -DestinationPath $toolVersionDir -Force -ErrorAction Stop
        } catch {
            Write-Host "[WARN] Extraction had issues (possibly due to path length)" -ForegroundColor Yellow
        }
        
        Write-Host "[OK] $($tool.Name) installed" -ForegroundColor Green
    } else {
        Write-Host "[SKIP] $($tool.Name) already installed" -ForegroundColor Yellow
    }
}

# 3. Verify critical files exist
Write-Host ""
Write-Host "Step 3: Verifying installation..." -ForegroundColor Cyan

$platformTxt = Join-Path $hardwareDir "platform.txt"
$boardsTxt = Join-Path $hardwareDir "boards.txt"

if (Test-Path $platformTxt) {
    Write-Host "[OK] platform.txt found" -ForegroundColor Green
} else {
    Write-Host "[ERROR] platform.txt NOT FOUND!" -ForegroundColor Red
    Write-Host "    Expected at: $platformTxt" -ForegroundColor Yellow
}

if (Test-Path $boardsTxt) {
    Write-Host "[OK] boards.txt found" -ForegroundColor Green  
} else {
    Write-Host "[ERROR] boards.txt NOT FOUND!" -ForegroundColor Red
    Write-Host "    Expected at: $boardsTxt" -ForegroundColor Yellow
}

# 4. Register with arduino-cli
Write-Host ""
Write-Host "Step 4: Registering with arduino-cli..." -ForegroundColor Cyan

$cliPath = Join-Path $scriptPath "tools\arduino-cli.exe"
$configPath = Join-Path $scriptPath "arduino-cli.yaml"

Write-Host "Checking if arduino-cli recognizes the platform..." -ForegroundColor Cyan
$coreList = & $cliPath core list --config-file $configPath 2>&1

if ($coreList -match "esp32:esp32") {
    Write-Host "[OK] Arduino-CLI recognizes ESP32 platform!" -ForegroundColor Green
} else {
    Write-Host "[WARN] Arduino-CLI does not recognize platform yet" -ForegroundColor Yellow
    Write-Host "Platform location: $hardwareDir" -ForegroundColor Cyan
}

# Clean up temp files
Write-Host ""
Write-Host "Step 5: Cleanup..." -ForegroundColor Cyan
Remove-Item -Path $tempDir -Recurse -Force -ErrorAction SilentlyContinue
Write-Host "[OK] Cleanup complete" -ForegroundColor Green

Write-Host ""
Write-Host "========================================" -ForegroundColor Green
Write-Host "Installation Complete!" -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Green
Write-Host ""
Write-Host "Installation Details:" -ForegroundColor Cyan
Write-Host "  Core: $hardwareDir" -ForegroundColor White
Write-Host "  Tools: $toolsDir" -ForegroundColor White
Write-Host ""
Write-Host "Next steps:" -ForegroundColor Yellow
Write-Host "1. Install libraries: .\install_libraries.ps1" -ForegroundColor White
Write-Host "2. Build firmware: .\build_firmware.ps1 -Project LCM" -ForegroundColor White
Write-Host ""
Write-Host "If build fails, check:" -ForegroundColor Yellow
Write-Host "- Does $platformTxt exist?" -ForegroundColor White
Write-Host "- Does $boardsTxt exist?" -ForegroundColor White
Write-Host "- Run: arduino-cli core list --config-file arduino-cli.yaml" -ForegroundColor White
Write-Host ""