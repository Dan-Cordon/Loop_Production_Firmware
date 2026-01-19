#pragma once
#include "types.h"
#include "helpers.h" // For SimpleMovingAverage template

namespace CanMixer {
    extern ControllerState state;
    extern CleaningState cleaningStateInstance;
    extern volatile unsigned long lastHeartbeatTime;
    extern bool waitingForPressure;
    extern bool SprayOutValveStatus;
    
    // We need to declare the template instance or the filter object
    // Assuming helpers.h has the template definition
    extern SimpleMovingAverage<float, Config::NOZZLE_UI_FILTER_SIZE> nozzleUiFilter;
    extern float nozzle_pressure_for_ui;
}