#pragma once
#include <Arduino.h>
#include <driver/twai.h>
#include "config.h"

namespace CanMixer {
    
    void setup_can();
    
    // Send Helpers
    void sendHeartbeatAck();
    void sendHopperStatus(uint8_t motor_index, uint16_t status_code);
    void sendStopAllMotors();
    void sendCleaningRPMs();
    void computeAndSendRPMs();
    void executePrimingSequence();
    void resetPrimingAcks();
    
    // Main processing
    void processDoserStatus(uint8_t motor_index, uint16_t status_code);
    
}