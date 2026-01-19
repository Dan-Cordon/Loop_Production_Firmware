#include <Arduino.h>
#include "src/config.h"
#include "src/globals.h"
#include "src/tasks.h"
#include "src/can_manager.h"

void setup() {
    using namespace CanMixer;
    Serial.begin(115200);
    lastHeartbeatTime = millis();

    // Hardcoded calibrations
    state.motors[0].ccv_deg_per_g = 3300.0f;
    state.motors[1].ccv_deg_per_g = 3300.0f;
    state.motors[2].ccv_deg_per_g = 3300.0f;
    state.motors[3].ccv_deg_per_g = 3300.0f;

    // Pin Setup
    gpio_set_direction(Config::RELAY_NOZZLE_OPEN, GPIO_MODE_OUTPUT);
    gpio_set_direction(Config::RELAY_NOZZLE_CLOSE, GPIO_MODE_OUTPUT);
    gpio_set_direction(Config::RELAY_SYSTEM_OPEN, GPIO_MODE_OUTPUT);
    gpio_set_direction(Config::RELAY_SYSTEM_CLOSE, GPIO_MODE_OUTPUT);
    gpio_set_direction(Config::RELAY_WATER_INLET_OPEN, GPIO_MODE_OUTPUT);
    gpio_set_direction(Config::RELAY_WATER_INLET_CLOSE, GPIO_MODE_OUTPUT);
    gpio_set_direction(Config::RELAY_SPRAY_OUTLET_OPEN, GPIO_MODE_OUTPUT);
    gpio_set_direction(Config::RELAY_SPRAY_OUTLET_CLOSE, GPIO_MODE_OUTPUT);

    // Controller Init
    state.nozzleController.relayOpenPin = Config::RELAY_NOZZLE_OPEN;
    state.nozzleController.relayClosePin = Config::RELAY_NOZZLE_CLOSE;
    state.nozzleController.direction = true;
    state.nozzleController.Kp = Config::NOZZLE_KP;

    state.systemController.relayOpenPin = Config::RELAY_SYSTEM_OPEN;
    state.systemController.relayClosePin = Config::RELAY_SYSTEM_CLOSE;
    state.systemController.direction = false;
    state.systemController.Kp = Config::SYSTEM_KP;
    
    adc2_config_channel_atten(Config::ADC_NOZZLE_PRESSURE, ADC_ATTEN_DB_11);
    adc2_config_channel_atten(Config::ADC_SYSTEM_PRESSURE, ADC_ATTEN_DB_11);

    // Initialize CAN
    setup_can();

    // Start Tasks
    xTaskCreatePinnedToCore(can_rx_task, "CAN_RX", 4096, nullptr, 10, nullptr, 0);
    xTaskCreatePinnedToCore(sensor_task, "Sensor", 4096, nullptr, 6, nullptr, 1);
    xTaskCreatePinnedToCore(valve_control_task, "ValveCtrl", 2048, nullptr, 7, nullptr, 1);
    xTaskCreatePinnedToCore(main_control_task, "MainCtrl", 4096, nullptr, 5, nullptr, 1);
    xTaskCreatePinnedToCore(tank_fill_task, "TankFill", 4096, nullptr, 4, nullptr, 1);
    xTaskCreatePinnedToCore(spray_valve_task, "SprayValve", 2048, nullptr, 4, nullptr, 1);
    
    Serial.println("System Running [Refactored]");
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}