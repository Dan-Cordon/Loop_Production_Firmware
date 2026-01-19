#pragma once
#include <Arduino.h>

namespace TWM {
namespace Config {

    // --- Hardware Pin Definitions ---
    constexpr int CAN_TX_GPIO = 35;
    constexpr int CAN_RX_GPIO = 36;
    
    constexpr int NUM_LOADCELLS = 4;
    // Load Cell Pins (DOUT, SCK)
    constexpr int LOADCELL_DOUT_PINS[NUM_LOADCELLS] = {10, 12, 14, 16};
    constexpr int LOADCELL_SCK_PINS[NUM_LOADCELLS]  = {11, 13, 15, 17};

    // --- CAN Bus IDs ---
    enum class CanID : uint32_t {
        TareCmd          = 0x4AA, // Input: Command to Tare
        TareCmdEsp       = 0x4AB, // Output: Confirmation of Tare
        MixerTankWeight  = 0x4D2  // Output: Weight Broadcast
    };

    // --- System Configuration ---
    constexpr int SENSOR_READ_INTERVAL_MS = 50;  // 20 Hz
    constexpr int CAN_SEND_INTERVAL_MS    = 100; // 10 Hz
    constexpr float WEIGHT_FILTER_ALPHA   = 0.2f;
    
    // Calibration
    constexpr float LOAD_CELL_SCALE_FACTOR = 423.4f;

} // namespace Config
} // namespace TWM