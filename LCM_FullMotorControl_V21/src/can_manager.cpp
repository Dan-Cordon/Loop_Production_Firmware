#include "can_manager.h"
#include "globals.h"
#include "helpers.h"

namespace CanMixer {

    void setup_can() {
        twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(Config::CAN_TX_PIN, Config::CAN_RX_PIN, TWAI_MODE_NORMAL);
        twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
        twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
        
        if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK || twai_start() != ESP_OK) {
            Serial.println("CAN Init Failed");
            while(1) vTaskDelay(1000);
        }
    }

    void sendHeartbeatAck() {
        twai_message_t msg = {};
        msg.identifier = static_cast<uint32_t>(Config::CanID::Heartbeat);
        msg.data_length_code = 1;
        msg.data[0] = 1;
        twai_transmit(&msg, pdMS_TO_TICKS(10));
    }

    void sendStopAllMotors() {
        twai_message_t msg = {};
        msg.data_length_code = 4;
        write_f32_le(msg.data, 0.0f);
        for (uint8_t i = 0; i < static_cast<size_t>(Config::Motor::Count); i++) {
            msg.identifier = static_cast<uint32_t>(Config::CanID::MotorRpmBase) + i;
            twai_transmit(&msg, pdMS_TO_TICKS(10));
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }

    void processDoserStatus(uint8_t motor_index, uint16_t status_code) {
        if (motor_index >= static_cast<size_t>(Config::Motor::Count)) return;
        auto& motor = state.motors[motor_index];
        switch (status_code) {
            case 0: motor.loaded = false; motor.paused = false; break;
            case 1: motor.loaded = true; motor.paused = false; break;
            case 2: motor.loaded = true; motor.paused = true; break;
            case 110: motor.paused = true; break;
            case 150: motor.loaded = true; motor.paused = false; break;
            default:
                if (static_cast<int16_t>(status_code) < 0) {
                    motor.loaded = false; motor.paused = true;
                }
                break;
        }
    }
    
    void sendHopperStatus(uint8_t motor_index, uint16_t status_code) {
        twai_message_t msg = {};
        msg.identifier = static_cast<uint32_t>(Config::CanID::HopperStatusBase) + motor_index;
        msg.data_length_code = 2;
        write_u16_le(msg.data, status_code);
        twai_transmit(&msg, pdMS_TO_TICKS(10));
    }

    void resetPrimingAcks() {
        for (uint8_t i = 0; i < static_cast<size_t>(Config::Motor::Count); ++i) {
            state.motors[i].primingAckReceived = false;
        }
    }

    void executePrimingSequence() {
        for (uint8_t i = 0; i < static_cast<size_t>(Config::Motor::Count); ++i) {
            auto& motor = state.motors[i];
            if (!motor.loaded || motor.paused || state.session.sprayVolHa_L <= 0 || motor.primingAckReceived) continue;

            float concentration = motor.rate_kg_per_ha / state.session.sprayVolHa_L;
            float priming_vol_mL = concentration * Config::PRIMING_TANK_VOLUME_L * 1000.0f;
            float degrees = 0;

            bool is_liquid = (i < static_cast<uint8_t>(Config::Motor::Solid1));
            if (is_liquid) {
                degrees = priming_vol_mL * Config::LIQUID_ML_PER_DEG;
            } else {
                uint8_t solid_idx = i - static_cast<uint8_t>(Config::Motor::Solid1);
                float mass_g = priming_vol_mL * Config::SOLID_DENSITY_G_PER_ML[solid_idx];
                degrees = mass_g * (motor.ccv_deg_per_g / 100.0f);
            }

            if (degrees > 0) {
                float revs = degrees / 360.0f;
                float dist_mm = revs * Config::MCM_MM_PER_REV[i];
                uint16_t rpm = static_cast<uint16_t>(Config::PRIMING_RPM);

                twai_message_t msg = {};
                msg.identifier = static_cast<uint32_t>(Config::CanID::StepMove);
                msg.data_length_code = 8;
                msg.data[0] = i;
                msg.data[1] = 1; // Forward
                write_f32_le(&msg.data[2], dist_mm);
                write_u16_le(&msg.data[6], rpm);
                twai_transmit(&msg, pdMS_TO_TICKS(20));
            }
        }
    }
    
    void computeAndSendRPMs() {
        twai_message_t msg = {};
        msg.data_length_code = 4;
        for (uint8_t i = 0; i < static_cast<size_t>(Config::Motor::Count); i++) {
            float rpm = computeRPM(state.motors[i], state.session);
            msg.identifier = static_cast<uint32_t>(Config::CanID::MotorRpmBase) + i;
            write_f32_le(msg.data, rpm);
            twai_transmit(&msg, pdMS_TO_TICKS(10));
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }

    void sendCleaningRPMs() {
        twai_message_t msg = {};
        msg.data_length_code = 4;
        write_f32_le(msg.data, Config::CLEANING_RPM);
        for (uint8_t i = 0; i <= static_cast<uint8_t>(Config::Motor::Liquid4); i++) {
            msg.identifier = static_cast<uint32_t>(Config::CanID::MotorRpmBase) + i;
            twai_transmit(&msg, pdMS_TO_TICKS(10));
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }

} // namespace CanMixer