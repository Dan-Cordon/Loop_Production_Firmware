#pragma once
#include "types.h"
#include "config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <driver/twai.h>

namespace MCM {
    
    extern volatile MixerStatus g_mixer_status;
    extern bool primingCommandReceived[Config::NUM_MOTORS];
    extern twai_message_t priming_status_msg;

    // Queues and States
    extern QueueHandle_t stepQueues[Config::NUM_MOTORS];
    extern QueueHandle_t speedQueues[Config::NUM_MOTORS];
    extern MotorState motorStates[Config::NUM_MOTORS];

    // Bit-bang control vars (for motors 4 & 5)
    extern volatile bool bitbangRunning[Config::NUM_MOTORS];
    extern volatile bool bitbangDirection[Config::NUM_MOTORS];
    extern volatile bool bitbangStepLevel[Config::NUM_MOTORS];

} // namespace MCM