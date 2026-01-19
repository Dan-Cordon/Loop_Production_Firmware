#include <Arduino.h>
#include "src/config.h"
#include "src/globals.h"
#include "src/helpers.h"
#include "src/tasks.h"

void setup_can_driver() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(MCM::Config::CAN_TX_PIN, MCM::Config::CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) return;
    if (twai_start() != ESP_OK) return;
}

void setup() {
    using namespace MCM;
    Serial.begin(115200);

    // Status LED
    gpio_set_direction(Config::STATUS_LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(Config::STATUS_LED_PIN, 1);

    // Initialize Global Status Msg
    priming_status_msg.identifier = static_cast<uint32_t>(Config::CanID::StepMoveStatus);
    priming_status_msg.data_length_code = 8;

    // Setup Motors
    for (int i = 0; i < Config::NUM_MOTORS; i++) {
        gpio_set_direction(Config::DIR_PINS[i], GPIO_MODE_OUTPUT);
        gpio_set_direction(Config::EN_PINS[i], GPIO_MODE_OUTPUT);
        gpio_set_direction(Config::STEP_PINS[i], GPIO_MODE_OUTPUT);
        
        // Default State: Disabled
        gpio_set_level(Config::EN_PINS[i], 1); 
        gpio_set_level(Config::DIR_PINS[i], 1);

        // Hardware Pulse Generation
        if (i < Config::RMT_MOTORS) {
            setupRMTChannel(i);
        } else {
            setupBitbangTimer(i - Config::RMT_MOTORS);
        }

        // Queues
        stepQueues[i] = xQueueCreate(5, sizeof(StepMoveRequest));
        speedQueues[i] = xQueueCreate(5, sizeof(SpeedMoveRequest));

        // Create Task
        int* pMotor = (int*)malloc(sizeof(int));
        *pMotor = i;
        char task_name[16];
        sprintf(task_name, "MotorTask%d", i);

        // Motors on Core 1
        xTaskCreatePinnedToCore(masterSpeedTask, task_name, 4096, pMotor, 1, NULL, 1);
    }
    
    // Setup CAN
    setup_can_driver();
    
    // CAN Rx on Core 0
    xTaskCreatePinnedToCore(can_receive_task, "CAN_Rx", 4096, NULL, 1, NULL, 0);
    
    Serial.println("MCM Running [Refactored]");
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}