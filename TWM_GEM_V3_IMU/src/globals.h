#pragma once
#include <Adafruit_BNO055.h>
#include <HX711.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "config.h"
#include "types.h"

namespace TWM {

    extern HX711 loadCells[Config::NUM_LOADCELLS];
    extern Adafruit_BNO055 bno;
    extern SemaphoreHandle_t hx711Mutex;
    
    extern volatile float filteredWeight_grams;
    extern volatile float rawWeight_grams;
    
    extern Orientation lastOrientation;

    // IMU Correction Coefficients (Rows: LC1 to LC4)
    extern const float coeffs_Y[4][4];
    extern const float coeffs_Z[4][4];

} // namespace TWM