#include "helpers.h"
#include "globals.h"
#include "driver/timer.h"
#include "driver/rmt.h"

namespace MCM {

    // --- Bit-Bang Timer Logic (Motors 4 & 5) ---
    
    static bool IRAM_ATTR bitbangTimerISR(void *arg) {
        int motor = (int)(intptr_t)arg;
        if (!bitbangRunning[motor]) return false;

        // Toggle step pin
        bitbangStepLevel[motor] = !bitbangStepLevel[motor];
        gpio_set_level(Config::STEP_PINS[motor + Config::RMT_MOTORS], bitbangStepLevel[motor]);

        // Set direction on falling edge
        if (!bitbangStepLevel[motor]) {
            gpio_set_level(Config::DIR_PINS[motor + Config::RMT_MOTORS], bitbangDirection[motor] ? 1 : 0);
        }
        return true; 
    }

    void setupBitbangTimer(int motor) {
        timer_config_t config = {
            .alarm_en = TIMER_ALARM_EN,
            .counter_en = TIMER_PAUSE,
            .intr_type = TIMER_INTR_LEVEL,
            .counter_dir = TIMER_COUNT_UP,
            .auto_reload = TIMER_AUTORELOAD_EN,
            .divider = 80 // 1 tick = 1us (80MHz / 80)
        };
        timer_init(TIMER_GROUP_0, (timer_idx_t)motor, &config);
        timer_set_counter_value(TIMER_GROUP_0, (timer_idx_t)motor, 0);
        timer_enable_intr(TIMER_GROUP_0, (timer_idx_t)motor);
        timer_isr_callback_add(TIMER_GROUP_0, (timer_idx_t)motor, bitbangTimerISR, (void *)(intptr_t)motor, 0);
    }

    void setBitbangFrequency(int motor, float rpm) {
        if (rpm <= 0) {
            timer_pause(TIMER_GROUP_0, (timer_idx_t)motor);
            bitbangRunning[motor] = false;
            gpio_set_level(Config::STEP_PINS[motor + Config::RMT_MOTORS], 0);
            return;
        }
        float freqHz = rpm * Config::MICROSTEPS / 60.0f;
        int period_us = (int)(1e6f / (freqHz * 2));
        if (period_us < 1) period_us = 1;

        timer_set_alarm_value(TIMER_GROUP_0, (timer_idx_t)motor, period_us);
        timer_start(TIMER_GROUP_0, (timer_idx_t)motor);
        
        bitbangDirection[motor] = motorStates[motor + Config::RMT_MOTORS].direction; // Use offset index for state
        bitbangRunning[motor] = true;
    }

    // --- RMT Logic (Motors 0-3) ---

    void setupRMTChannel(int motor) {
        if (motor >= Config::RMT_MOTORS) return;
        rmt_config_t config;
        config.channel = Config::RMT_CHANNELS[motor];
        config.gpio_num = Config::STEP_PINS[motor];
        config.clk_div = Config::CLOCK_DIV;
        config.mem_block_num = 1;
        config.rmt_mode = RMT_MODE_TX;
        config.tx_config.loop_en = false;
        config.tx_config.carrier_en = false;
        config.tx_config.idle_output_en = true;
        config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
        config.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;

        ESP_ERROR_CHECK(rmt_config(&config));
        ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));
    }

    void setRMTFrequency(int motor, float rpm) {
        if (motor >= Config::RMT_MOTORS) {
            setBitbangFrequency(motor - Config::RMT_MOTORS, rpm);
            return;
        }
        if (rpm <= 0) {
            rmt_tx_stop(Config::RMT_CHANNELS[motor]);
            return;
        }
        float stepsPerSec = rpm * (Config::MICROSTEPS * 2) / 60.0f;
        int period_us = (int)(1e6f / stepsPerSec);
        if (period_us < Config::PULSE_WIDTH_US * 2) period_us = Config::PULSE_WIDTH_US * 2;

        rmt_item32_t item;
        item.duration0 = Config::PULSE_WIDTH_US;
        item.level0 = 1;
        item.duration1 = period_us - Config::PULSE_WIDTH_US;
        item.level1 = 0;

        rmt_write_items(Config::RMT_CHANNELS[motor], &item, 1, true);
        rmt_set_tx_loop_mode(Config::RMT_CHANNELS[motor], true);
        rmt_tx_start(Config::RMT_CHANNELS[motor], true);
    }

    // --- General Control ---

    void enableMotor(int motor, bool enable, bool direction) {
        gpio_set_level(Config::DIR_PINS[motor], direction ? 1 : 0);
        gpio_set_level(Config::EN_PINS[motor], enable ? 0 : 1); // Active LOW
    }

    void stopMotor(int motor) {
        if (motor < 0 || motor >= Config::NUM_MOTORS) return;
        
        if (motor < Config::RMT_MOTORS) {
            rmt_tx_stop(Config::RMT_CHANNELS[motor]);
        } else {
            setBitbangFrequency(motor - Config::RMT_MOTORS, 0);
        }
        
        gpio_set_level(Config::EN_PINS[motor], 1); // Disable driver

        motorStates[motor].currentRPM = 0;
        motorStates[motor].targetRPM = 0;
        motorStates[motor].stepMoveActive = false;
        motorStates[motor].speedMoveActive = false;
        motorStates[motor].stepsRemaining = 0;
    }

    void speedMove(int motor, bool active, float rpm, bool direction) {
        if (motor < 0 || motor >= Config::NUM_MOTORS) return;
        SpeedMoveRequest req = {active, rpm, direction};
        xQueueSend(speedQueues[motor], &req, pdMS_TO_TICKS(100));
    }

    void stepMove(int motor, float distance_mm, float rpm, bool direction) {
        if (motor < 0 || motor >= Config::NUM_MOTORS) return;
        if (rpm <= 0 || fabs(distance_mm) < 0.0001f) return;

        float revs = fabs(distance_mm) / Config::MM_PER_REV[motor];
        int steps = (int)(revs * Config::MICROSTEPS);

        StepMoveRequest req;
        req.steps = steps;
        req.rpm = rpm;
        req.direction = (distance_mm >= 0) ? direction : !direction;

        xQueueSend(stepQueues[motor], &req, pdMS_TO_TICKS(100));
    }

    // --- Helpers ---

    void send_step_move_ack(int motor_id) {
        twai_message_t msg = {};
        msg.identifier = static_cast<uint32_t>(Config::CanID::StepMoveAck);
        msg.data_length_code = 1;
        msg.data[0] = motor_id;
        twai_transmit(&msg, pdMS_TO_TICKS(20));
    }

    float read_f32_le(const uint8_t* b) { float f; memcpy(&f, b, 4); return f; }
    uint16_t read_u16_le(const uint8_t* b) { return (uint16_t)b[0] | ((uint16_t)b[1] << 8); }

    void handle_maintenance_command(int motor_num, uint8_t command) {
        float target_rpm = 0;
        bool is_active = true;
        switch (command) {
            case 0: is_active = false; break;
            case 1: target_rpm = Config::CONTROL_PANEL_SPEED_RPM; break;
            case 10: target_rpm = Config::FULL_SPEED_RPM; break;
            default: return;
        }
        speedMove(motor_num, is_active, target_rpm, true);
    }

} // namespace MCM