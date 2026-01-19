#include <Arduino.h>
#include <Wire.h>
#include <driver/twai.h>
#include "src/config.h"
#include "src/globals.h"
#include "src/tasks.h"

void setup() {
    using namespace TWM;
    Serial.begin(115200);
    delay(100);
    Serial.println("--- Tank Weight Board (TWM) Refactored ---");

    Wire.begin();

    // --- Initialize HX711 Load Cells ---
    for (int i = 0; i < Config::NUM_LOADCELLS; i++) {
        loadCells[i].begin(Config::LOADCELL_DOUT_PINS[i], Config::LOADCELL_SCK_PINS[i]);
        loadCells[i].set_scale(Config::LOAD_CELL_SCALE_FACTOR);
        loadCells[i].tare();
    }
    Serial.println("Load cells initialized.");

    // --- Initialize BNO055 IMU ---
    if (!bno.begin()) {
        Serial.println("BNO055 IMU not detected. Check wiring.");
    } else {
        bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P5);
        bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P0);
        bno.setExtCrystalUse(true);
        Serial.println("BNO055 IMU initialized.");
    }

    hx711Mutex = xSemaphoreCreateMutex();

    // --- TWAI (CAN) Setup ---
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)Config::CAN_TX_GPIO, (gpio_num_t)Config::CAN_RX_GPIO, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        if (twai_start() == ESP_OK) {
            Serial.println("CAN Started.");
        } else {
            Serial.println("CAN Start Failed.");
        }
    } else {
        Serial.println("CAN Install Failed.");
    }

    // --- Task Creation ---
    xTaskCreate(loadCellTask, "LoadCell", 4096, NULL, 4, NULL);
    xTaskCreate(imuTask, "IMU", 4096, NULL, 2, NULL);
    xTaskCreate(canReceiveTask, "CAN_Rx", 4096, NULL, 5, NULL);
    xTaskCreate(canSendTask, "CAN_Tx", 4096, NULL, 3, NULL);

    Serial.println("System Running.");
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}