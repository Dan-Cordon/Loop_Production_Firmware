#pragma once
#include <Arduino.h>
#include <driver/gpio.h>
#include <driver/rmt.h>

namespace MCM {
namespace Config {

    // --- Hardware Pin Definitions ---
    constexpr int NUM_MOTORS = 6;
    constexpr int RMT_MOTORS = 4;  // First 4 motors use RMT, last 2 use hardware timer

    // WARNING: GPIO 3 and 46 are strapping pins on ESP32-S3. 
    // If pulled low/high at boot by motor drivers, boot may fail.
    // Ensure drivers are not powered or isolated during boot if using these pins.
    
    constexpr gpio_num_t STEP_PINS[NUM_MOTORS] = {GPIO_NUM_1, GPIO_NUM_4, GPIO_NUM_7, GPIO_NUM_12, GPIO_NUM_15, GPIO_NUM_18};
    constexpr gpio_num_t DIR_PINS[NUM_MOTORS]  = {GPIO_NUM_2, GPIO_NUM_5, GPIO_NUM_10, GPIO_NUM_13, GPIO_NUM_16, GPIO_NUM_21};
    constexpr gpio_num_t EN_PINS[NUM_MOTORS]   = {GPIO_NUM_3, GPIO_NUM_6, GPIO_NUM_11, GPIO_NUM_14, GPIO_NUM_17, GPIO_NUM_47};
    
    constexpr rmt_channel_t RMT_CHANNELS[RMT_MOTORS] = {RMT_CHANNEL_0, RMT_CHANNEL_1, RMT_CHANNEL_2, RMT_CHANNEL_3};
    
    constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_35;
    constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_36;
    constexpr gpio_num_t STATUS_LED_PIN = GPIO_NUM_38;

    // --- CAN Bus IDs ---
    enum class CanID : uint32_t {
        MixerStatus      = 0x401,
        SolidMotorsCmd   = 0x464,  // Maintenance Jog
        LiquidMotorsCmd  = 0x469,  // Maintenance Jog
        MotorRpmBase     = 0x4A0,  // + Motor Index (0-5)
        StepMove         = 0x501,
        StepMoveStatus   = 0x502,  // FIXED: Changed from 0x41A to match LCM!
        StepMoveAck      = 0x503
    };

    // --- Motor Parameters ---
    constexpr int MICROSTEPS = 1600;
    constexpr float MM_PER_REV[NUM_MOTORS] = {5.0, 5.0, 5.0, 5.0, 5.0, 5.0};
    
    // RMT Configuration
    constexpr int CLOCK_DIV = 80;        // 1 MHz resolution
    constexpr int PULSE_WIDTH_US = 8;
    
    // Control Loop
    constexpr float ACCELERATION_RPM_PER_SEC = 50.0f;
    constexpr int CONTROL_PERIOD_MS = 5;
    constexpr int STATUS_REPORT_INTERVAL_CYCLES = 100; // Report status every 100 loops (500ms)

    // Maintenance Speeds
    constexpr float CONTROL_PANEL_SPEED_RPM = 150.0f;
    constexpr float FULL_SPEED_RPM = 150.0f;

} // namespace Config
} // namespace MCM