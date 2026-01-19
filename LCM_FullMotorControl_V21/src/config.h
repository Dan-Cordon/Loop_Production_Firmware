#pragma once
#include <Arduino.h>
#include <driver/gpio.h>
#include <driver/adc.h>

namespace CanMixer {
namespace Config {

    // --- Pin Definitions ---
    constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_35;
    constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_36;
    
    // Analog Inputs
    constexpr adc2_channel_t ADC_NOZZLE_PRESSURE = ADC2_CHANNEL_1; // GPIO 11
    constexpr adc2_channel_t ADC_SYSTEM_PRESSURE = ADC2_CHANNEL_0; // GPIO 12

    // Relay Outputs
    constexpr gpio_num_t RELAY_NOZZLE_OPEN        = GPIO_NUM_7;
    constexpr gpio_num_t RELAY_NOZZLE_CLOSE       = GPIO_NUM_10;
    constexpr gpio_num_t RELAY_SYSTEM_OPEN        = GPIO_NUM_1;
    constexpr gpio_num_t RELAY_SYSTEM_CLOSE       = GPIO_NUM_2;
    constexpr gpio_num_t RELAY_WATER_INLET_OPEN   = GPIO_NUM_4;
    constexpr gpio_num_t RELAY_WATER_INLET_CLOSE  = GPIO_NUM_3;
    constexpr gpio_num_t RELAY_SPRAY_OUTLET_OPEN  = GPIO_NUM_5;
    constexpr gpio_num_t RELAY_SPRAY_OUTLET_CLOSE = GPIO_NUM_6;

    // --- CAN Bus IDs ---
    enum class CanID : uint32_t {
        Heartbeat              = 0x400,
        MixerStatus            = 0x401,
        DoserStatusSolids      = 0x414,
        DoserStatusLiquids     = 0x419,
        SolidMotorsCmd         = 0x464,  // Maintenance Mode Only
        LiquidMotorsCmd        = 0x469,  // Maintenance Mode Only
        WaterInletValve        = 0x482,
        SprayOutletValve       = 0x487,
        PropSystemPressure     = 0x48C,
        PropNozzlePressure     = 0x48E,
        TareCmd                = 0x4AB,
        MixerTankWeight        = 0x4D2,
        PressureSensorData     = 0x4C8,
        NozzlePressureSetpoint = 0x538,
        SystemPressureSetpoint = 0x539,
        NumFaces               = 0x536,
        TargetSpeed            = 0x53A,
        RowSpacing             = 0x540,
        SprayVolHa             = 0x542,
        // Doser Config
        S1_RateUnit=0x59C, S1_Ccv=0x59E,
        S2_RateUnit=0x5A6, S2_Ccv=0x5A8,
        L1_RateUnit=0x5B0, L1_Ccv=0x5B2,
        L2_RateUnit=0x5BA, L2_Ccv=0x5BC,
        L3_RateUnit=0x5C4, L3_Ccv=0x5C6,
        L4_RateUnit=0x5CE, L4_Ccv=0x5D0,
        
        StepMove               = 0x501,
        StepMoveStatus         = 0x502, // FIXED: Matched MCM [cite: 86]
        StepMoveAck            = 0x503,
        HopperStatusBase       = 0x5D1,
        LcmTelemetry           = 0x5F0,
        MotorRpmBase           = 0x4A0,
        CleaningCycles         = 0x45A
    };

    enum class Motor : uint8_t { Liquid1=0, Liquid2, Liquid3, Liquid4, Solid1, Solid2, Count };

    // --- System Constants ---
    constexpr float TANK_FILL_TARGET_L = 20.0f;
    constexpr float TANK_MAINTAIN_HYSTERESIS_L = 0.5f;
    constexpr float PRIMING_RPM = 50.0f;
    constexpr float PRIMING_TANK_VOLUME_L = 20.0f;
    constexpr float LIQUID_ML_PER_DEG = 33.10f;
    constexpr int   DENSITY_G_PER_ML = 1.0f;
    
    // Timing
    constexpr int SENSOR_READ_INTERVAL_MS = 100;
    constexpr int CONTROL_LOOP_INTERVAL_MS = 100;
    constexpr int TANK_FILL_INTERVAL_MS = 250;
    constexpr int SPRAY_VALVE_INTERVAL_MS = 250;
    constexpr int HEARTBEAT_TIMEOUT_MS = 2000;
    constexpr int VALVE_CONTROL_INTERVAL_MS = 100;
    constexpr int PWM_ACTIVATION_WINDOW_MS = 500;
    constexpr int PWM_CONTROL_LOOP_INTERVAL_MS = 20;

    // Pressure Control
    constexpr float RPM_MAX = 4000.0f;
    constexpr float PRESSURE_HYSTERESIS_BAR = 0.5f;
    constexpr float PRESSURE_MAX_BAR = 50.0f;
    constexpr int   ADC_FULL_SCALE = 4095;
    constexpr int   ADC_MIN_RAW = ADC_FULL_SCALE * 0.15;
    constexpr int   ADC_MAX_RAW = ADC_FULL_SCALE * 1;
    constexpr float FILTER_ALPHA = 0.5f;
    constexpr float CLEANING_RPM = 100.0f;
    
    // PID Gains
    constexpr float NOZZLE_KP = 30.0f; 
    constexpr float SYSTEM_KP = 9.0f; 
    constexpr size_t NOZZLE_UI_FILTER_SIZE = 10;

    // Motor Calibrations
    constexpr float MCM_MM_PER_REV[6] = {5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f};
    constexpr float SOLID_DENSITY_G_PER_ML[2] = {0.85f, 0.90f};

} // namespace Config
} // namespace CanMixer