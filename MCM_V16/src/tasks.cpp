#include "tasks.h"
#include "config.h"
#include "globals.h"
#include "helpers.h"
#include <math.h>

using namespace MCM;

// -----------------------------------------------------------------------------
// Motor Control Task (One instance per motor)
// -----------------------------------------------------------------------------
void masterSpeedTask(void* pvParameters) {
    int motor = *((int*)pvParameters);
    free(pvParameters); // Free the memory allocated in setup

    StepMoveRequest stepReq = {0, 0, true};
    SpeedMoveRequest speedReq = {false, 0, true};
    int iter = 0;

    while (true) {
        // 1. Check for Step Move Request (Priming/Dosing)
        if (xQueueReceive(stepQueues[motor], &stepReq, 0) == pdTRUE) {
            if (stepReq.steps > 0 && stepReq.rpm > 0) {
                motorStates[motor].stepsRemaining = stepReq.steps;
                motorStates[motor].stepMoveActive = true;
                motorStates[motor].speedMoveActive = false;
                motorStates[motor].targetRPM = stepReq.rpm;
                motorStates[motor].currentRPM = 0; // Ramp up from 0
                motorStates[motor].direction = stepReq.direction;
            }
        }

        // 2. Check for Continuous Speed Request (Manual/Run)
        if (xQueueReceive(speedQueues[motor], &speedReq, 0) == pdTRUE) {
            motorStates[motor].speedMoveActive = speedReq.active;
            if (speedReq.active) {
                motorStates[motor].targetRPM = speedReq.rpm;
                motorStates[motor].direction = speedReq.direction;
                motorStates[motor].stepMoveActive = false;
            } else if (!motorStates[motor].stepMoveActive) {
                motorStates[motor].targetRPM = 0;
            }
        }

        // 3. Execution Logic
        if (motorStates[motor].stepMoveActive) {
            // -- Acceleration Ramp --
            float target = motorStates[motor].targetRPM;
            float current = motorStates[motor].currentRPM;
            float ramp = Config::ACCELERATION_RPM_PER_SEC * (Config::CONTROL_PERIOD_MS / 1000.0f);
            
            if (current < target) current = fmin(current + ramp, target);
            else if (current > target) current = fmax(current - ramp, target);
            
            motorStates[motor].currentRPM = current;
            enableMotor(motor, true, motorStates[motor].direction);
            setRMTFrequency(motor, current);

            // -- Step Counting --
            float stepsPerSec = current * Config::MICROSTEPS / 60.0f;
            int stepsThisPeriod = (int)round(stepsPerSec * (Config::CONTROL_PERIOD_MS / 1000.0f));
            if (stepsThisPeriod < 1) stepsThisPeriod = 1;

            motorStates[motor].stepsRemaining -= stepsThisPeriod;

            if (motorStates[motor].stepsRemaining <= 0) {
                // Done
                motorStates[motor].currentRPM = 0;
                setRMTFrequency(motor, 0);
                enableMotor(motor, false, motorStates[motor].direction);
                motorStates[motor].stepMoveActive = false;
                
                // Update Status Msg
                priming_status_msg.data[motor + 2] = 1; 
            }
        }
        else if (motorStates[motor].speedMoveActive) {
            // -- Continuous Mode --
            float target = motorStates[motor].targetRPM;
            float current = motorStates[motor].currentRPM;
            float ramp = Config::ACCELERATION_RPM_PER_SEC * (Config::CONTROL_PERIOD_MS / 1000.0f);

            if (current < target) current = fmin(current + ramp, target);
            else if (current > target) current = fmax(current - ramp, target);

            if (fabs(current - motorStates[motor].currentRPM) > 0.01f) {
                motorStates[motor].currentRPM = current;
                enableMotor(motor, true, motorStates[motor].direction);
                setRMTFrequency(motor, current);
            }
        }
        else {
            // -- Idle --
            if (motorStates[motor].currentRPM > 0) {
                motorStates[motor].currentRPM = 0;
                setRMTFrequency(motor, 0);
                enableMotor(motor, false, motorStates[motor].direction);
            }
        }

        // 4. Status Reporting
        iter++;
        if (iter >= Config::STATUS_REPORT_INTERVAL_CYCLES) {
            twai_transmit(&priming_status_msg, pdMS_TO_TICKS(20));
            iter = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(Config::CONTROL_PERIOD_MS));
    }
}

// -----------------------------------------------------------------------------
// CAN Receive Task
// -----------------------------------------------------------------------------
void can_receive_task(void* pvParameters) {
    twai_message_t msg;
    while (true) {
        if (twai_receive(&msg, portMAX_DELAY) == ESP_OK) {
            auto id = static_cast<Config::CanID>(msg.identifier);

            // 1. Mixer Status
            if (id == Config::CanID::MixerStatus) {
                if (msg.data_length_code >= 2) {
                    MixerStatus new_status = (MixerStatus)read_u16_le(msg.data);
                    if (new_status != g_mixer_status) {
                        g_mixer_status = new_status;
                        
                        // Restart on Stop (Original Logic preserved)
                        if (new_status == MixerStatus::Stopped) {
                             vTaskDelay(100); 
                             ESP.restart(); 
                        }

                        // Reset priming flags if not priming
                        if (g_mixer_status != MixerStatus::Priming) {
                            for (int i = 0; i < Config::NUM_MOTORS; i++) primingCommandReceived[i] = false;
                        }
                    }
                    
                    // Failsafe Stop
                    if (g_mixer_status == MixerStatus::Stopped || 
                        g_mixer_status == MixerStatus::StopMixing || 
                        g_mixer_status == MixerStatus::StopCleaning) {
                        for (int i = 0; i < Config::NUM_MOTORS; i++) stopMotor(i);
                    }
                }
                continue;
            }

            // 2. Variable RPM Command (Run Mode)
            if (msg.identifier >= (uint32_t)Config::CanID::MotorRpmBase && 
                msg.identifier < (uint32_t)Config::CanID::MotorRpmBase + Config::NUM_MOTORS) {
                
                int motor_id = msg.identifier - (uint32_t)Config::CanID::MotorRpmBase;
                if (msg.data_length_code >= 4) {
                    float rpm = read_f32_le(msg.data);
                    // Failsafe: Only run if in valid state
                    if (g_mixer_status != MixerStatus::Running && 
                        g_mixer_status != MixerStatus::Rundown && 
                        g_mixer_status != MixerStatus::StartResumeCleaning) {
                        speedMove(motor_id, false, 0.0f, true);
                    } else {
                        speedMove(motor_id, rpm > 0.1f, rpm, true);
                    }
                }
                continue;
            }

            // 3. Maintenance Jogging
            if (g_mixer_status == MixerStatus::ControlPanelActive || g_mixer_status == MixerStatus::CleaningCheckList) {
                if (id == Config::CanID::LiquidMotorsCmd && msg.data_length_code >= 8) {
                    handle_maintenance_command(0, msg.data[0]);
                    handle_maintenance_command(1, msg.data[2]);
                    handle_maintenance_command(2, msg.data[4]);
                    handle_maintenance_command(3, msg.data[6]);
                }
                else if (id == Config::CanID::SolidMotorsCmd && msg.data_length_code >= 4) {
                    handle_maintenance_command(4, msg.data[0]);
                    handle_maintenance_command(5, msg.data[2]);
                }
                return;
            }

            // 4. Priming / Step Move
            if (id == Config::CanID::StepMove) {
                if (g_mixer_status == MixerStatus::Priming && msg.data_length_code == 8) {
                    uint8_t motor_id = msg.data[0];
                    if (!primingCommandReceived[motor_id]) {
                        primingCommandReceived[motor_id] = true;
                        bool dir = (bool)msg.data[1];
                        float dist = 0;
                        memcpy(&dist, &msg.data[2], 4);
                        uint16_t rpm = read_u16_le(&msg.data[6]);
                        
                        send_step_move_ack(motor_id);
                        stepMove(motor_id, dist, (float)rpm, dir);
                    }
                }
            }
        }
    }
}