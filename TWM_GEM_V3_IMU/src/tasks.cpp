#include <Arduino.h>
#include <driver/twai.h>
#include "tasks.h"
#include "config.h"
#include "globals.h"
#include "helpers.h"

using namespace TWM;

/**
 * @brief Task to read load cells, apply correction, and filter.
 */
void loadCellTask(void* pv) {
    (void)pv;
    float rawReadings[Config::NUM_LOADCELLS] = {0};

    while (true) {
        // Step 1: Get raw readings (in grams) from all load cells
        if (xSemaphoreTake(hx711Mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            for (int i = 0; i < Config::NUM_LOADCELLS; i++) {
                if (loadCells[i].is_ready()) {
                    rawReadings[i] = loadCells[i].get_units(1);
                }
            }
            xSemaphoreGive(hx711Mutex);
        }

        // Step 2: Calculate corrected sum
        float corrected_sum = 0;
        for (int i = 0; i < Config::NUM_LOADCELLS; i++) {
            // Calculate correction for both Y and Z axes based on last orientation
            float correctionY = calculateCorrection(lastOrientation.y, coeffs_Y, i);
            float correctionZ = calculateCorrection(lastOrientation.z, coeffs_Z, i);
            float totalCorrection = correctionY + correctionZ;

            // Apply the correction to the raw reading
            float correctedValue = rawReadings[i] - totalCorrection;
            corrected_sum += correctedValue;
        }

        // Step 3: Apply exponential smoothing filter
        filteredWeight_grams = (Config::WEIGHT_FILTER_ALPHA * corrected_sum) + 
                               ((1.0f - Config::WEIGHT_FILTER_ALPHA) * filteredWeight_grams);
        
        vTaskDelay(pdMS_TO_TICKS(Config::SENSOR_READ_INTERVAL_MS));
    }
}

/**
 * @brief Task to read the BNO055 IMU.
 */
void imuTask(void* pv) {
    (void)pv;
    while (true) {
        sensors_event_t event;
        if (bno.getEvent(&event)) {
            // Apply offsets as defined in original code
            lastOrientation.x = event.orientation.x - 96;
            lastOrientation.y = event.orientation.y - 3;
            lastOrientation.z = event.orientation.z + 93;
        } else {
            // Retry init on fail
            static unsigned long lastTry = 0;
            if (millis() - lastTry > 5000) {
                bno.begin();
                lastTry = millis();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz
    }
}

/**
 * @brief Task to handle incoming CAN messages (Tare).
 */
void canReceiveTask(void* pv) {
    (void)pv;
    twai_message_t rx_msg;
    while (true) {
        if (twai_receive(&rx_msg, pdMS_TO_TICKS(100)) == ESP_OK) {
            
            // Check for Tare Command
            if (rx_msg.identifier == (uint32_t)Config::CanID::TareCmd && rx_msg.data_length_code >= 1) {
                // Value '5' indicates user tare press
                if (rx_msg.data[0] == 5) {
                    Serial.println("Tare command received via CAN.");
                    
                    if (xSemaphoreTake(hx711Mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
                        for (int i = 0; i < Config::NUM_LOADCELLS; i++) {
                            loadCells[i].tare();
                        }
                        xSemaphoreGive(hx711Mutex);
                        
                        filteredWeight_grams = 0.0f;
                        Serial.println("All load cells tared.");

                        // Send Confirmation
                        twai_message_t tx_msg;
                        tx_msg.identifier = (uint32_t)Config::CanID::TareCmdEsp;
                        tx_msg.flags = 0;
                        tx_msg.data_length_code = 1;
                        tx_msg.data[0] = 8; // Completion Code

                        if (twai_transmit(&tx_msg, pdMS_TO_TICKS(100)) == ESP_OK) {
                            Serial.println("Tare confirmation sent.");
                        }
                    } else {
                        Serial.println("Mutex timeout: Could not tare.");
                    }
                }
            }
        }
    }
}

/**
 * @brief Task to broadcast weight.
 */
void canSendTask(void* pv) {
    (void)pv;
    while (true) {
        twai_message_t weight_msg;
        memset(&weight_msg, 0, sizeof(weight_msg));
        weight_msg.identifier = (uint32_t)Config::CanID::MixerTankWeight;
        weight_msg.data_length_code = 4;
        
        // Convert to kg
        float weight_kg = filteredWeight_grams / 100.0f;
        memcpy(weight_msg.data, &weight_kg, sizeof(weight_kg));
        
        // Serial.print("Broadcasting Weight (kg): ");
        // Serial.println(weight_kg);
        
        twai_transmit(&weight_msg, pdMS_TO_TICKS(10));
        vTaskDelay(pdMS_TO_TICKS(Config::CAN_SEND_INTERVAL_MS));
    }
}