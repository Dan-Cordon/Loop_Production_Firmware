#pragma once
#include <Arduino.h>
#include <driver/twai.h>
#include "config.h"
#include "types.h"

namespace MCM {

    // --- Bit-bang Timer ISR & Setup ---
    void setupBitbangTimer(int motor);
    void setBitbangFrequency(int motor, float rpm);

    // --- RMT Setup & Control ---
    void setupRMTChannel(int motor);
    void setRMTFrequency(int motor, float rpm);

    // --- General Motor Control ---
    void enableMotor(int motor, bool enable, bool direction);
    void stopMotor(int motor);
    void speedMove(int motor, bool active, float rpm, bool direction);
    void stepMove(int motor, float distance_mm, float rpm, bool direction);

    // --- CAN Helpers ---
    void send_step_move_status(int motor_id, uint8_t status);
    void send_step_move_ack(int motor_id);
    void handle_maintenance_command(int motor_num, uint8_t command);

    // --- Data Parsing ---
    float read_f32_le(const uint8_t* b);
    uint16_t read_u16_le(const uint8_t* b);

} // namespace MCM