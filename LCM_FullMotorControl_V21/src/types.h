#pragma once
#include <Arduino.h>
#include <driver/gpio.h>
#include "config.h"

namespace CanMixer {

    enum class MixerStatus : uint16_t {
        Stopped   = 0,
        Tare      = 8,
        Priming   = 10,
        Running   = 20,
        Rundown   = 30,
        StopMixing = 100,
        CleaningCheckList = 150,
        CleaningPurge = 160,
        StartResumeCleaning = 200,
        StopCleaning = 300,
        ControlPanelActive = 1000
    };

    enum class FillState { IDLE, WAITING_FOR_TARE, FILLING, MAINTAINING };

    struct CleaningState {
        int iter = 0;
        int status = 0;
        bool visited_flag = false;
    };

    struct SessionInputs {
        float rowSpacing_m = 0.0f;
        float targetSpeed_mps = 0.0f;
        float sprayVolHa_L = 0.0f;
        uint16_t numFaces = 0;
    };

    struct TreatmentInput {
        bool loaded = false;
        bool paused = false;
        float rate_kg_per_ha = 0.0f;
        float ccv_deg_per_g = 0.0f;
        bool priming_complete = false;
        bool priming_status_sent = false;
        bool primingAckReceived = false;
    };

    struct ValveController {
        float input = 0.0f;
        float setpoint = 0.0f;
        bool enabled = true;
        gpio_num_t relayOpenPin;
        gpio_num_t relayClosePin;
        bool direction; // true=direct (nozzle), false=inverse (system)
        
        // PWM State
        float Kp;
        unsigned long window_start_time = 0;
        long activation_duration_ms = 0;
        gpio_num_t active_relay_pin = GPIO_NUM_NC;
    };

    struct ControllerState {
        SessionInputs session;
        TreatmentInput motors[static_cast<size_t>(Config::Motor::Count)];
        ValveController systemController;
        ValveController nozzleController;
        MixerStatus mixerStatus = MixerStatus::Stopped;
        bool priming_initiated = false;
        FillState fill_state = FillState::WAITING_FOR_TARE;
        float current_tank_weight_kg = 0.0f;
        bool tare_complete = false;
        int tare_status = 0;
        int waterinletvalve = 0;
        int sprayoutletvalve = 0;
        int propsystempressure = 0;
        int propnozzlepressure = 0;
    };

} // namespace CanMixer