#pragma once
#include <Arduino.h>

namespace MCM {

    enum class MixerStatus : uint16_t {
        Stopped   = 0,
        Priming   = 10,
        Running   = 20,
        Rundown   = 30,
        StopMixing = 100,
        CleaningCheckList = 150,
        StartResumeCleaning = 200,
        StopCleaning = 300,
        ControlPanelActive = 1000
    };

    struct StepMoveRequest {
        int steps;
        float rpm;
        bool direction;
    };

    struct SpeedMoveRequest {
        bool active;
        float rpm;
        bool direction;
    };

    struct MotorState {
        float targetRPM;
        float currentRPM;
        bool direction;
        bool stepMoveActive;
        bool speedMoveActive;
        int stepsRemaining;
        int totalSteps;
    };

} // namespace MCM