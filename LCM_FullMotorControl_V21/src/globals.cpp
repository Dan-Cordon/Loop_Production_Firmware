#include "globals.h"

namespace CanMixer {
    ControllerState state;
    CleaningState cleaningStateInstance;
    volatile unsigned long lastHeartbeatTime = 0;
    bool waitingForPressure = false;
    bool SprayOutValveStatus = false;
    
    SimpleMovingAverage<float, Config::NOZZLE_UI_FILTER_SIZE> nozzleUiFilter;
    float nozzle_pressure_for_ui = 0.0f;
}