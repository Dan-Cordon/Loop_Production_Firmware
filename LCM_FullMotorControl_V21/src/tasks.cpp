#include <Arduino.h>
#include <driver/twai.h>
#include <driver/adc.h>
#include <esp_task_wdt.h> 

#include "tasks.h"
#include "config.h"
#include "globals.h"
#include "helpers.h"
#include "can_manager.h"

using namespace CanMixer;

// -----------------------------------------------------------------------------
// CAN Receive Task
// -----------------------------------------------------------------------------
void can_rx_task(void* pv) {
    twai_message_t msg;
    while (true) {
        if (twai_receive(&msg, pdMS_TO_TICKS(100)) == ESP_OK) {
            auto id = static_cast<Config::CanID>(msg.identifier);
            switch (id) {
                case Config::CanID::Heartbeat:
                    if (msg.data_length_code > 0 && msg.data[0] == 0) {
                        lastHeartbeatTime = millis();
                        sendHeartbeatAck();
                    }
                    break;

                case Config::CanID::DoserStatusSolids:
                    if (msg.data_length_code >= 2) processDoserStatus(static_cast<uint8_t>(Config::Motor::Solid1), read_u16_le(&msg.data[0]));
                    if (msg.data_length_code >= 4) processDoserStatus(static_cast<uint8_t>(Config::Motor::Solid2), read_u16_le(&msg.data[2]));
                    break;

                case Config::CanID::DoserStatusLiquids:
                    if (msg.data_length_code >= 2) processDoserStatus(static_cast<uint8_t>(Config::Motor::Liquid1), read_u16_le(&msg.data[0]));
                    if (msg.data_length_code >= 4) processDoserStatus(static_cast<uint8_t>(Config::Motor::Liquid2), read_u16_le(&msg.data[2]));
                    if (msg.data_length_code >= 6) processDoserStatus(static_cast<uint8_t>(Config::Motor::Liquid3), read_u16_le(&msg.data[4]));
                    if (msg.data_length_code >= 8) processDoserStatus(static_cast<uint8_t>(Config::Motor::Liquid4), read_u16_le(&msg.data[6]));
                    break;

                case Config::CanID::MixerStatus:
                    if (msg.data_length_code >= 2) {
                        auto new_status = static_cast<MixerStatus>(read_u16_le(msg.data));
                        if (new_status != state.mixerStatus) {
                            state.mixerStatus = new_status;
                            // Reset ESP if home/stop is clicked (Logic from original code)
                            if (int(new_status) == 0) {
                                cleaningStateInstance.status = 0;
                                cleaningStateInstance.iter = 0;
                                cleaningStateInstance.visited_flag = false;
                                vTaskDelay(pdMS_TO_TICKS(500));
                                // ESP.restart(); // Uncomment if you want to keep the original hard reset behavior
                            }
                            Serial.printf("MIXER STATUS: New status received: %u\n", static_cast<uint16_t>(state.mixerStatus));
                        }
                    }
                    break;

                case Config::CanID::StepMoveAck:
                    if (msg.data_length_code >= 1) {
                        uint8_t motor_idx = msg.data[0];
                        if (motor_idx < static_cast<size_t>(Config::Motor::Count)) {
                            state.motors[motor_idx].primingAckReceived = true;
                        }
                    }
                    break;

                case Config::CanID::StepMoveStatus:
                    if (msg.data_length_code >= 2) {
                        uint8_t motor_idx = msg.data[0];
                        uint8_t status_code = msg.data[1];
                        if (motor_idx < static_cast<size_t>(Config::Motor::Count) && status_code == 1) {
                            state.motors[motor_idx].priming_complete = true;
                        }
                    }
                    break;

                case Config::CanID::MixerTankWeight:
                    if (msg.data_length_code >= 4) state.current_tank_weight_kg = read_f32_le(msg.data);
                    break;

                case Config::CanID::TareCmd:
                    if (msg.data_length_code >= 1) {
                        state.tare_status = msg.data[0];
                        if (msg.data[0] == 8) state.tare_complete = true;
                    }
                    break;

                case Config::CanID::SprayVolHa:
                    if (msg.data_length_code >= 4) state.session.sprayVolHa_L = read_f32_le(msg.data);
                    break;

                case Config::CanID::TargetSpeed:
                    if (msg.data_length_code >= 4) state.session.targetSpeed_mps = read_f32_le(msg.data) / 3.6f;
                    break;

                case Config::CanID::RowSpacing:
                    if (msg.data_length_code >= 4) state.session.rowSpacing_m = read_f32_le(msg.data);
                    break;

                case Config::CanID::NumFaces:
                    if (msg.data_length_code >= 2) state.session.numFaces = read_f32_le(msg.data);
                    break;

                // --- Treatment Data Inputs ---
                case Config::CanID::S1_RateUnit: {
                    if (msg.data_length_code >= 6) {
                        float val = read_f32_le(&msg.data[0]);
                        uint16_t u = read_u16_le(&msg.data[4]);
                        state.motors[static_cast<uint8_t>(Config::Motor::Solid1)].rate_kg_per_ha = to_kg_per_ha(val, u);
                    }
                    break;
                }
                case Config::CanID::S1_Ccv:
                    if (msg.data_length_code >= 4) state.motors[static_cast<uint8_t>(Config::Motor::Solid1)].ccv_deg_per_g = read_f32_le(msg.data);
                    break;

                case Config::CanID::S2_RateUnit: {
                    if (msg.data_length_code >= 6) {
                        float val = read_f32_le(&msg.data[0]);
                        uint16_t u = read_u16_le(&msg.data[4]);
                        state.motors[static_cast<uint8_t>(Config::Motor::Solid2)].rate_kg_per_ha = to_kg_per_ha(val, u);
                    }
                    break;
                }
                case Config::CanID::S2_Ccv:
                    if (msg.data_length_code >= 4) state.motors[static_cast<uint8_t>(Config::Motor::Solid2)].ccv_deg_per_g = read_f32_le(msg.data);
                    break;

                case Config::CanID::L1_RateUnit: {
                    if (msg.data_length_code >= 6) {
                        float val = read_f32_le(&msg.data[0]);
                        uint16_t u = read_u16_le(&msg.data[4]);
                        state.motors[static_cast<uint8_t>(Config::Motor::Liquid1)].rate_kg_per_ha = to_kg_per_ha(val, u);
                    }
                    break;
                }
                case Config::CanID::L2_RateUnit: {
                    if (msg.data_length_code >= 6) {
                        float val = read_f32_le(&msg.data[0]);
                        uint16_t u = read_u16_le(&msg.data[4]);
                        state.motors[static_cast<uint8_t>(Config::Motor::Liquid2)].rate_kg_per_ha = to_kg_per_ha(val, u);
                    }
                    break;
                }
                case Config::CanID::L3_RateUnit: {
                    if (msg.data_length_code >= 6) {
                        float val = read_f32_le(&msg.data[0]);
                        uint16_t u = read_u16_le(&msg.data[4]);
                        state.motors[static_cast<uint8_t>(Config::Motor::Liquid3)].rate_kg_per_ha = to_kg_per_ha(val, u);
                    }
                    break;
                }
                case Config::CanID::L4_RateUnit: {
                    if (msg.data_length_code >= 6) {
                        float val = read_f32_le(&msg.data[0]);
                        uint16_t u = read_u16_le(&msg.data[4]);
                        state.motors[static_cast<uint8_t>(Config::Motor::Liquid4)].rate_kg_per_ha = to_kg_per_ha(val, u);
                    }
                    break;
                }

                case Config::CanID::NozzlePressureSetpoint:
                    if (msg.data_length_code >= 4) state.nozzleController.setpoint = read_f32_le(msg.data);
                    break;

                case Config::CanID::WaterInletValve:
                    if (msg.data_length_code >= 2) {
                        int temp = read_u16_le(msg.data);
                        if (temp != state.waterinletvalve) {
                            state.waterinletvalve = temp;
                            gpio_set_level(Config::RELAY_WATER_INLET_CLOSE, !state.waterinletvalve);
                            gpio_set_level(Config::RELAY_WATER_INLET_OPEN, state.waterinletvalve);
                        }
                    }
                    break;

                case Config::CanID::SprayOutletValve:
                    if (msg.data_length_code >= 2) {
                        int temp = read_u16_le(msg.data);
                        if (temp != state.sprayoutletvalve) {
                            state.sprayoutletvalve = temp;
                            SprayOutletValveController(state.sprayoutletvalve);
                        }
                    }
                    break;

                case Config::CanID::PropSystemPressure:
                    if (msg.data_length_code >= 2) {
                        int temp = read_u16_le(msg.data);
                        if (temp != state.propsystempressure) {
                            state.propsystempressure = temp;
                            gpio_set_level(state.systemController.relayClosePin, !state.propsystempressure);
                            gpio_set_level(state.systemController.relayOpenPin, state.propsystempressure);
                        }
                    }
                    break;

                case Config::CanID::PropNozzlePressure:
                    if (msg.data_length_code >= 2) {
                        int temp = read_u16_le(msg.data);
                        if (temp != state.propnozzlepressure) {
                            state.propnozzlepressure = temp;
                            gpio_set_level(state.nozzleController.relayClosePin, !state.propnozzlepressure);
                            gpio_set_level(state.nozzleController.relayOpenPin, state.propnozzlepressure);
                        }
                    }
                    break;

                default: break;
            }
        }
    }
}

// -----------------------------------------------------------------------------
// Sensor Read Task
// -----------------------------------------------------------------------------
void sensor_task(void* pv) {
    int raw_nozzle = 0;
    int raw_system = 0;

    while (true) {
        adc2_get_raw(Config::ADC_NOZZLE_PRESSURE, ADC_WIDTH_BIT_12, &raw_nozzle);
        adc2_get_raw(Config::ADC_SYSTEM_PRESSURE, ADC_WIDTH_BIT_12, &raw_system);

        float rawNozzlePressure = calculate_pressure(raw_nozzle);
        state.nozzleController.input = (Config::FILTER_ALPHA * rawNozzlePressure) + (1.0f - Config::FILTER_ALPHA) * state.nozzleController.input;

        nozzleUiFilter.add(state.nozzleController.input);
        nozzle_pressure_for_ui = nozzleUiFilter.getAverage();

        float rawSystemPressure = calculate_pressure(raw_system);
        state.systemController.input = (Config::FILTER_ALPHA * rawSystemPressure) + (1.0f - Config::FILTER_ALPHA) * state.systemController.input;

        // Broadcast sensor data
        twai_message_t pres = {};
        pres.identifier = static_cast<uint32_t>(Config::CanID::PressureSensorData);
        pres.data_length_code = 8;
        write_f32_le(&pres.data[0], nozzle_pressure_for_ui);
        write_f32_le(&pres.data[4], state.systemController.input);
        twai_transmit(&pres, pdMS_TO_TICKS(10));

        vTaskDelay(pdMS_TO_TICKS(Config::SENSOR_READ_INTERVAL_MS));
    }
}

// -----------------------------------------------------------------------------
// Valve PWM Control Task
// -----------------------------------------------------------------------------
void valve_control_task(void* pv) {
    while (true) {
        checkStabilization();

        // Control Nozzle and System Valves using PWM logic
        update_valve_pwm(state.nozzleController);
        update_valve_pwm(state.systemController);

        // Failsafe force open if setpoint is effectively 0
        if (state.systemController.setpoint < 0.1) {
            gpio_set_level(state.systemController.relayOpenPin, 1);
        }

        vTaskDelay(pdMS_TO_TICKS(Config::PWM_CONTROL_LOOP_INTERVAL_MS));
    }
}

// -----------------------------------------------------------------------------
// Tank Fill Logic Task
// -----------------------------------------------------------------------------
void tank_fill_task(void* pv) {
    while (true) {
        float current_level_L = state.current_tank_weight_kg;

        // Skip logic if mixer is stopped (Home pressed)
        if (int(state.mixerStatus) == 0) {
            vTaskDelay(pdMS_TO_TICKS(Config::TANK_FILL_INTERVAL_MS));
            continue;
        }

        switch (state.fill_state) {
            case FillState::IDLE:
                // Do nothing
                break;

            case FillState::WAITING_FOR_TARE:
                if (state.tare_complete) {
                    state.fill_state = FillState::FILLING;
                }
                break;

            case FillState::FILLING:
                // Serial.printf("TANK WEIGHT: %f\n", current_level_L);
                if (current_level_L < Config::TANK_FILL_TARGET_L) {
                    // Open water inlet
                    gpio_set_level(Config::RELAY_WATER_INLET_CLOSE, 0);
                    gpio_set_level(Config::RELAY_WATER_INLET_OPEN, 1);
                } else {
                    // Target reached, close inlet
                    gpio_set_level(Config::RELAY_WATER_INLET_OPEN, 0);
                    gpio_set_level(Config::RELAY_WATER_INLET_CLOSE, 1);
                    state.fill_state = FillState::MAINTAINING;
                }
                break;

            case FillState::MAINTAINING:
                if (current_level_L < (Config::TANK_FILL_TARGET_L - Config::TANK_MAINTAIN_HYSTERESIS_L)) {
                    // Open valve to top up
                    gpio_set_level(Config::RELAY_WATER_INLET_CLOSE, 0);
                    gpio_set_level(Config::RELAY_WATER_INLET_OPEN, 1);
                } else if (current_level_L >= Config::TANK_FILL_TARGET_L) {
                    // Close valve
                    gpio_set_level(Config::RELAY_WATER_INLET_OPEN, 0);
                    gpio_set_level(Config::RELAY_WATER_INLET_CLOSE, 1);
                } else {
                    // Deadband: de-energize both
                    gpio_set_level(Config::RELAY_WATER_INLET_OPEN, 0);
                    gpio_set_level(Config::RELAY_WATER_INLET_CLOSE, 0);
                }
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(Config::TANK_FILL_INTERVAL_MS));
    }
}

// -----------------------------------------------------------------------------
// Spray Valve Task
// -----------------------------------------------------------------------------
void spray_valve_task(void* pv) {
    while (true) {
        // Skip if in Control Panel mode
        if (int(state.mixerStatus) == 1000) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Determine if we should be spraying based on status
        bool should_be_spraying = (state.mixerStatus == MixerStatus::Running ||
                                   state.mixerStatus == MixerStatus::Rundown ||
                                   state.mixerStatus == MixerStatus::CleaningPurge ||
                                   (cleaningStateInstance.status % 100 == 0 && cleaningStateInstance.status > 0) ||
                                   cleaningStateInstance.status % 100 == 60);

        if (should_be_spraying) {
            SprayOutletValveController(true);
        } else {
            SprayOutletValveController(false);
        }

        vTaskDelay(pdMS_TO_TICKS(Config::SPRAY_VALVE_INTERVAL_MS));
    }
}

// -----------------------------------------------------------------------------
// Main Control Loop Task
// -----------------------------------------------------------------------------
void main_control_task(void* pv) {
    twai_message_t cleaning_status_msg = {};
    cleaning_status_msg.identifier = static_cast<uint32_t>(Config::CanID::CleaningCycles);
    cleaning_status_msg.data_length_code = 8;
    write_u16_le(&cleaning_status_msg.data[0], 100); // Initial dummy value
    twai_transmit(&cleaning_status_msg, pdMS_TO_TICKS(10));

    while (true) {
        // Heartbeat Watchdog
        if (millis() - lastHeartbeatTime > Config::HEARTBEAT_TIMEOUT_MS) {
            if (state.mixerStatus != MixerStatus::Stopped) {
                Serial.println("HEARTBEAT TIMEOUT: Forcing system to STOPPED state.");
                state.mixerStatus = MixerStatus::Stopped;
            }
            lastHeartbeatTime = millis();
        }

        switch (state.mixerStatus) {
            case MixerStatus::Stopped:
            {
                waitingForPressure = false;
                state.nozzleController.enabled = false;
                state.systemController.enabled = true; // Maintain setpoint (likely 0 or low)

                // Stop water intake
                gpio_set_level(Config::RELAY_WATER_INLET_CLOSE, 1);
                gpio_set_level(Config::RELAY_WATER_INLET_OPEN, 0);

                state.tare_complete = false;
                state.tare_status = 0;

                if (state.fill_state != FillState::WAITING_FOR_TARE) {
                    state.fill_state = FillState::WAITING_FOR_TARE;
                }

                SprayOutletValveController(false);
                state.priming_initiated = false;
                resetPrimingAcks();
                sendStopAllMotors();
                break;
            }

            case MixerStatus::Priming:
            {
                if (!state.priming_initiated) {
                    resetPrimingAcks();
                    for (uint8_t i = 0; i < static_cast<size_t>(Config::Motor::Count); ++i) {
                        if (state.motors[i].loaded && !state.motors[i].paused) {
                            sendHopperStatus(i, 1); // 1 = In Progress
                        }
                    }
                    state.priming_initiated = true;
                    Serial.println("PRIMING INIT");
                }

                // Wait for tank to be full before starting priming
                if (state.fill_state != FillState::MAINTAINING) break;

                executePrimingSequence();

                for (uint8_t i = 0; i < static_cast<size_t>(Config::Motor::Count); ++i) {
                    auto& motor = state.motors[i];
                    if (motor.loaded && !motor.paused && motor.priming_complete && !motor.priming_status_sent) {
                        sendHopperStatus(i, 100); // 100 = Priming Success
                        motor.priming_status_sent = true;
                    }
                }
                break;
            }

            case MixerStatus::Running:
            {
                if (!state.priming_initiated) {
                    // Only start stabilization wait if not already active
                    if (!waitingForPressure && !state.nozzleController.enabled) {
                        waitingForPressure = true;
                        state.nozzleController.enabled = false;
                    }
                }
                state.priming_initiated = false;
                resetPrimingAcks();
                computeAndSendRPMs();
                break;
            }

            case MixerStatus::Rundown:
            {
                state.priming_initiated = false;
                resetPrimingAcks();
                sendStopAllMotors();
                state.fill_state = FillState::IDLE;
                gpio_set_level(Config::RELAY_WATER_INLET_CLOSE, 1);
                gpio_set_level(Config::RELAY_WATER_INLET_OPEN, 0);
                break;
            }

            case MixerStatus::StartResumeCleaning:
            {
                state.priming_initiated = false;
                resetPrimingAcks();
                // Serial.println(state.current_tank_weight_kg);
                sendCleaningRPMs();

                if (cleaningStateInstance.status <= 410) {
                    twai_transmit(&cleaning_status_msg, pdMS_TO_TICKS(10));

                    // Purge Cycle
                    if (cleaningStateInstance.status % 100 == 0 || cleaningStateInstance.status % 100 == 60) {
                        if (cleaningStateInstance.visited_flag == false) {
                            state.fill_state = FillState::IDLE;
                            cleaningStateInstance.status = (cleaningStateInstance.iter + 1) * 100;
                            write_u16_le(&cleaning_status_msg.data[0], cleaningStateInstance.status);

                            // Stop water intake
                            gpio_set_level(Config::RELAY_WATER_INLET_CLOSE, 1);
                            gpio_set_level(Config::RELAY_WATER_INLET_OPEN, 0);

                            // Open Nozzle valves (Purge)
                            SprayOutletValveController(true);
                            cleaningStateInstance.visited_flag = true;
                        }

                        if (state.current_tank_weight_kg <= 1) { // Tank Empty
                            cleaningStateInstance.status = (cleaningStateInstance.iter + 1) * 100 + 10;
                            write_u16_le(&cleaning_status_msg.data[0], cleaningStateInstance.status);
                            cleaningStateInstance.visited_flag = false;
                        }
                    }
                    // Refill Cycle
                    else if (cleaningStateInstance.status % 100 == 10 || cleaningStateInstance.status % 100 == 50) {
                        if (!cleaningStateInstance.visited_flag && cleaningStateInstance.status < 410) {
                            state.fill_state = FillState::FILLING;
                            cleaningStateInstance.status = (cleaningStateInstance.iter + 1) * 100 + 50;
                            write_u16_le(&cleaning_status_msg.data[0], cleaningStateInstance.status);

                            // Shut Nozzle valves
                            SprayOutletValveController(false);
                            cleaningStateInstance.visited_flag = true;
                        }

                        if (state.current_tank_weight_kg >= 15) { // Tank Full
                            cleaningStateInstance.status = (cleaningStateInstance.iter + 1) * 100 + 60;
                            write_u16_le(&cleaning_status_msg.data[0], cleaningStateInstance.status);
                            cleaningStateInstance.iter += 1;
                            cleaningStateInstance.visited_flag = false;
                        }
                    }
                } else {
                    state.fill_state = FillState::IDLE;
                }
                break;
            }

            case MixerStatus::StopCleaning:
            {
                cleaningStateInstance.visited_flag = false;
                sendStopAllMotors();
                SprayOutletValveController(false);
                gpio_set_level(Config::RELAY_WATER_INLET_CLOSE, 1);
                gpio_set_level(Config::RELAY_WATER_INLET_OPEN, 0);
                break;
            }

            case MixerStatus::ControlPanelActive:
            case MixerStatus::CleaningCheckList:
                // No specific logic loop action required, motors controlled via discrete messages
                break;

            default:
                state.priming_initiated = false;
                resetPrimingAcks();
                sendStopAllMotors();
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(Config::CONTROL_LOOP_INTERVAL_MS));
    }
}