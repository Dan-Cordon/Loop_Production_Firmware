/*
 ================================================================================
 Tank Weight Measurement (TWM) Board Firmware - v2.1b (Float Output Test)
 ================================================================================
 Author: Gemini
 Date: 29 August 2025
 Description:
 This is a MODIFIED version for testing purposes. It differs from the original
 in the following ways:
 - IMU correction is DISABLED.
 - The exponential smoothing filter is DISABLED.
 - The firmware now reads the 4 load cells, sums their raw gram values, converts
   the sum to kilograms, and broadcasts it as a 4-byte float over the CAN bus.

 This is useful for verifying load cell calibration without the influence of
 the IMU or filtering.
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>      // IMU Disabled
#include <Adafruit_BNO055.h>      // IMU Disabled
#include <utility/imumaths.h>     // IMU Disabled
#include <HX711.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <driver/twai.h>

// --- Hardware Pin Definitions (Update to match your PCB) ---
#define CAN_TX_GPIO 35
#define CAN_RX_GPIO 36

// Load cell pins
#define NUM_LOADCELLS 4
const int LOADCELL_DOUT_PINS[NUM_LOADCELLS] = {10, 12, 14, 16};
const int LOADCELL_SCK_PINS[NUM_LOADCELLS] = {11, 13, 15, 17};

// --- CAN Bus Address Library ---
// Inputs to this board
const uint32_t CAN_ID_TARE_CMD = 0x4AA;
const uint32_t CAN_ID_TARE_CMD_ESP = 0x4AB;

// Outputs from this board
const uint32_t CAN_ID_MIXER_TANK_WEIGHT = 0x4D2;

// --- System Configuration ---
const int SENSOR_READ_INTERVAL_MS = 50;  // Read sensors at 20 Hz
const int CAN_SEND_INTERVAL_MS = 100;    // Broadcast weight at 10 Hz
const float WEIGHT_FILTER_ALPHA = 0.2f; // Filter disabled for this version

// Coefficients for Y-axis (Rows: LC1 to LC4)
float coeffs_Y[4][4] = {
  { 1.103E-01, 1.136E-15, -2.586E+02, 0 },  // LC1
  { 1.327E-01, 5E-16, -2.99E+02, 0 },       // LC2
  { -3.067E-01, -1.9E-15, 2.073E+02, 0 },   // LC3
  { -1.369E-02, -1.424E-16, 7.83E+01, 0 }   // LC4
};

// Coefficients for Z-axis (Rows: LC1 to LC4)
float coeffs_Z[4][4] = {
  { -1.113E-01, -1.74E-16, 5.047E+01, 0 },  // LC1
  { 1.312E-01, -1.0E-15, -1.396E+02, 0 },   // LC2
  { -2.535E-01, -6.4E-17, -8.795E+00, 0 },  // LC3
  { 1.138E-01, 4E-17, 2.27E+01, 0 }         // LC4
};

// --- Global Variables ---
HX711 loadCells[NUM_LOADCELLS];
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); // IMU Disabled

SemaphoreHandle_t hx711Mutex;

volatile float filteredWeight_grams = 0.0f; // Replaced with raw weight
volatile float rawWeight_grams = 0.0f;

// /* IMU Disabled
struct Orientation {
    float x, y, z;
} lastOrientation = {0};
// */

// --- Helper Functions (IMU Correction Disabled) ---
// /*
float calculateCorrection(float imuValue, const float coeffs[NUM_LOADCELLS][4], int cellIndex) {
    float a3 = coeffs[cellIndex][0];
    float a2 = coeffs[cellIndex][1];
    float a1 = coeffs[cellIndex][2];
    float a0 = coeffs[cellIndex][3];

    // Use Horner's method for efficient polynomial evaluation
    return (((a3 * imuValue + a2) * imuValue + a1) * imuValue + a0);
}
// */

// --- FreeRTOS Tasks ---

/**
 * @brief Task to read load cells and sum their raw values.
 * IMU correction and filtering have been removed for this test version.
 */
void loadCellTask(void* pv) {
    (void)pv;
    float rawReadings[NUM_LOADCELLS] = {0};

    while (true) {
        // Step 1: Get raw readings (in grams) from all load cells
        if (xSemaphoreTake(hx711Mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            for (int i = 0; i < NUM_LOADCELLS; i++) {
                if (loadCells[i].is_ready()) {
                    rawReadings[i] = loadCells[i].get_units(1);
                }
            }
            xSemaphoreGive(hx711Mutex);
        }

        // MODIFIED: Step 2: Sum the raw readings directly
        // float raw_sum = 0;
        // for (int i = 0; i < NUM_LOADCELLS; i++) {
        //     raw_sum += rawReadings[i];
        // }
        float corrected_sum = 0;
        for (int i = 0; i < NUM_LOADCELLS; i++) {
            // Calculate correction for both Y and Z axes
            float correctionY = calculateCorrection(lastOrientation.y, coeffs_Y, i);
            float correctionZ = calculateCorrection(lastOrientation.z, coeffs_Z, i);
            float totalCorrection = correctionY + correctionZ;

            // Apply the correction to the raw reading
            float correctedValue = rawReadings[i] - totalCorrection;
            corrected_sum += correctedValue;
        }

        // Step 3: Apply exponential smoothing filter to the final corrected sum
        filteredWeight_grams = (WEIGHT_FILTER_ALPHA * corrected_sum) + ((1.0f - WEIGHT_FILTER_ALPHA) * filteredWeight_grams);
        

        // Store the raw sum in the global variable for the canSendTask
        // rawWeight_grams = raw_sum;
        
        // IMU Correction and Filtering logic has been removed.
        
        vTaskDelay(pdMS_TO_TICKS(SENSOR_READ_INTERVAL_MS));
    }
}

/**
 * @brief Task to read the BNO055 IMU. DISABLED FOR THIS VERSION.
 */
// /*
void imuTask(void* pv) {
    (void)pv;
    while (true) {
        sensors_event_t event;
        if (bno.getEvent(&event)) {
            lastOrientation.x = event.orientation.x - 96;
            lastOrientation.y = event.orientation.y - 3;
            lastOrientation.z = event.orientation.z + 93;
        } else {
            // If reading fails, try to re-initialize the sensor occasionally
            static unsigned long lastTry = 0;
            if (millis() - lastTry > 5000) {
                bno.begin();
                lastTry = millis();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // IMU data is less critical, read at 10 Hz
    }
}
// */

/**
 * @brief Task to handle incoming CAN messages, specifically the Tare command.
 */
void canReceiveTask(void* pv) {
    (void)pv;
    twai_message_t rx_msg;
    while (true) {
        // Wait to receive a message
        if (twai_receive(&rx_msg, pdMS_TO_TICKS(100)) == ESP_OK) {
            // Check if the message is a tare command
            if (rx_msg.identifier == CAN_ID_TARE_CMD && rx_msg.data_length_code >= 1) {
                // A value of '5' indicates a user tare press
                if (rx_msg.data[0] == 5) {
                    Serial.println("Tare command received via CAN.");
                    // Safely access the load cells using the mutex
                    if (xSemaphoreTake(hx711Mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
                        // Tare all load cells
                        for (int i = 0; i < NUM_LOADCELLS; i++) {
                            loadCells[i].tare();
                        }
                        xSemaphoreGive(hx711Mutex); // Release the mutex

                        // Reset the raw weight immediately after taring
                        // rawWeight_grams = 0.0f;
                        filteredWeight_grams = 0.0f;
                        Serial.println("All load cells have been tared.");

                        // ---- NEW CODE TO SEND CONFIRMATION ----
                        
                        // 1. Prepare the response message
                        twai_message_t tx_msg;
                        tx_msg.identifier = CAN_ID_TARE_CMD_ESP; // Use the same CAN ID for the response
                        tx_msg.flags = 0;                    // Standard data frame (not extended, not RTR)
                        tx_msg.data_length_code = 1;         // We are sending one byte of data
                        tx_msg.data[0] = 8;                  // Set the data value to 8 to indicate completion

                        // 2. Transmit the message
                        if (twai_transmit(&tx_msg, pdMS_TO_TICKS(100)) == ESP_OK) {
                            Serial.println("Tare completion confirmation (value 8) sent successfully.");
                        } else {
                            Serial.println("Failed to send tare confirmation.");
                        }
                        // ---- END OF NEW CODE ----

                    } else {
                        Serial.println("Could not acquire mutex to tare load cells.");
                    }
                }
            }
        }
    }
}

/**
 * @brief Task to periodically broadcast sensor data over the CAN bus.
 */
void canSendTask(void* pv) {
    (void)pv;
    while (true) {
        // --- Send Mixer Tank Weight ---
        twai_message_t weight_msg;
        memset(&weight_msg, 0, sizeof(weight_msg));
        weight_msg.identifier = CAN_ID_MIXER_TANK_WEIGHT;
        weight_msg.data_length_code = 4; // A float is 4 bytes
        
        // MODIFIED: Send weight as a 32-bit float in kilograms
        // float weight_kg = rawWeight_grams / 100.0f;
        float weight_kg = filteredWeight_grams / 100.0f;
        memcpy(weight_msg.data, &weight_kg, sizeof(weight_kg));
        
        Serial.print("Broadcasting Weight (kg): ");
        Serial.println(weight_kg);
        
        twai_transmit(&weight_msg, pdMS_TO_TICKS(10));

        vTaskDelay(pdMS_TO_TICKS(CAN_SEND_INTERVAL_MS));
    }
}

void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println("--- Tank Weight Board (TWM) v2.1b --- FLOAT TEST VERSION ---");

    Wire.begin();

    // --- Initialize HX711 Load Cells ---
    for (int i = 0; i < NUM_LOADCELLS; i++) {
        loadCells[i].begin(LOADCELL_DOUT_PINS[i], LOADCELL_SCK_PINS[i]);
        
        // ACTION REQUIRED: Calibrate each load cell!
        // This scale factor converts the raw ADC reading into grams.
        loadCells[i].set_scale(423.4f); // <-- REPLACE WITH YOUR CALIBRATED VALUE
        
        loadCells[i].tare(); // Tare all cells on startup
    }
    Serial.println("All load cells initialized and tared.");

    // --- Initialize BNO055 IMU (DISABLED for this version) ---
    // /*
    if (!bno.begin()) {
        Serial.println("BNO055 IMU not detected. Check wiring.");
    } else {
        bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P5);
        bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P0);
        bno.setExtCrystalUse(true);
        Serial.println("BNO055 IMU initialized.");
    }
    // */

    // Create mutex for safe multi-task access to HX711s
    hx711Mutex = xSemaphoreCreateMutex();

    // --- TWAI (CAN) Setup ---
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_GPIO, (gpio_num_t)CAN_RX_GPIO, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        Serial.println("TWAI Driver installed.");
        if (twai_start() == ESP_OK) {
            Serial.println("TWAI Driver started.");
        } else {
            Serial.println("Failed to start TWAI driver.");
        }
    } else {
        Serial.println("Failed to install TWAI driver.");
    }

    // --- Task Creation ---
    xTaskCreate(loadCellTask, "LoadCell_Task", 4096, NULL, 4, NULL);
    xTaskCreate(imuTask, "IMU_Task", 4096, NULL, 2, NULL); // IMU Task Disabled
    xTaskCreate(canReceiveTask, "CAN_Rx_Task", 4096, NULL, 5, NULL);
    xTaskCreate(canSendTask, "CAN_Tx_Task", 4096, NULL, 3, NULL);

    Serial.println("Setup complete. System running.");
}

void loop() {
    // The loop is empty because all work is handled by FreeRTOS tasks.
    vTaskDelay(pdMS_TO_TICKS(1000));
}

