#include "globals.h"

namespace MCM {

    volatile MixerStatus g_mixer_status = MixerStatus::Stopped;
    bool primingCommandReceived[Config::NUM_MOTORS] = {false};
    twai_message_t priming_status_msg = {};

    QueueHandle_t stepQueues[Config::NUM_MOTORS];
    QueueHandle_t speedQueues[Config::NUM_MOTORS];
    MotorState motorStates[Config::NUM_MOTORS];

    volatile bool bitbangRunning[Config::NUM_MOTORS] = {false};
    volatile bool bitbangDirection[Config::NUM_MOTORS] = {true};
    volatile bool bitbangStepLevel[Config::NUM_MOTORS] = {false};

} // namespace MCM