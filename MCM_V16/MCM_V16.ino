#include <driver/rmt.h>
#include "driver/gpio.h"
#include "driver/timer.h"
#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <math.h>
#include <string.h>

#define MICROSTEPS 1600
#define NUM_MOTORS 6
#define RMT_MOTORS 4  // First 4 motors use RMT, last 2 use hardware timer for bit-bang
const gpio_num_t STEP_PINS[NUM_MOTORS] = {GPIO_NUM_1, GPIO_NUM_4, GPIO_NUM_7, GPIO_NUM_12, GPIO_NUM_15, GPIO_NUM_18};
const gpio_num_t DIR_PINS[NUM_MOTORS]  = {GPIO_NUM_2, GPIO_NUM_5, GPIO_NUM_10, GPIO_NUM_13, GPIO_NUM_16, GPIO_NUM_21};
const gpio_num_t EN_PINS[NUM_MOTORS]   = {GPIO_NUM_3, GPIO_NUM_6, GPIO_NUM_11, GPIO_NUM_14, GPIO_NUM_17, GPIO_NUM_47};
const rmt_channel_t RMT_CHANNELS[RMT_MOTORS] = {RMT_CHANNEL_0, RMT_CHANNEL_1, RMT_CHANNEL_2, RMT_CHANNEL_3};
const gpio_num_t BUTTON_PIN = GPIO_NUM_39; // The button to trigger the sequence

// Distance per motor revolution in millimeters (configure per motor if needed)
const float MM_PER_REV[NUM_MOTORS] = {5.0, 5.0, 5.0, 5.0, 5.0, 5.0}; 
// Example: 5mm lead screw pitch = 5 mm/rev

const int CLOCK_DIV = 80;          // 1 MHz RMT clock (80 MHz / 80)
const int PULSE_WIDTH_US = 8;      // Step high time

struct StepMoveRequest {
  int steps;
  float rpm;
  bool direction;
};

struct SpeedMoveRequest {
  bool active;
  float rpm;
  bool direction;
};

struct MotorState {
  float targetRPM;
  float currentRPM;
  bool direction;
  bool stepMoveActive;
  bool speedMoveActive;
  int stepsRemaining;
  int totalSteps;
};

QueueHandle_t stepQueues[NUM_MOTORS];
QueueHandle_t speedQueues[NUM_MOTORS];
MotorState motorStates[NUM_MOTORS];

// Bit-bang motor control variables
volatile bool bitbangRunning[NUM_MOTORS] = {false};
volatile float bitbangFrequencyHz[NUM_MOTORS] = {0};
volatile bool bitbangDirection[NUM_MOTORS] = {true};
volatile bool bitbangStepLevel[NUM_MOTORS] = {false};

const float ACCELERATION_RPM_PER_SEC = 50.0f;
const int CONTROL_PERIOD_MS = 5;
const int MIN_STEPS_PER_BATCH = 10;

// --- CAN Bus Configuration ---
#define CAN_TX_PIN GPIO_NUM_35
#define CAN_RX_PIN GPIO_NUM_36

// --- CAN Message IDs ---
#define CAN_ID_MIXER_STATUS       0x401
// IDs for Maintenance Mode Jogging (from UI)
#define CAN_ID_SOLID_MOTORS       0x464
#define CAN_ID_LIQUID_MOTORS      0x469
// IDs for Normal Operation (from LCM)
#define CAN_ID_MOTOR_RPM_BASE     0x4A0
#define CAN_ID_STEP_MOVE          0x501
#define CAN_ID_STEP_MOVE_STATUS   0x41A // MCM reports step move is complete
#define CAN_ID_STEP_MOVE_ACK      0x503 // MCM acknowledges receipt of a step move command

//Priming status at 41A
twai_message_t priming_status_41A = {};

// --- Hardcoded Speeds for Maintenance Mode ---
const float CONTROL_PANEL_SPEED_RPM = 150.0f;
const float FULL_SPEED_RPM = 150.0f;

// Global state for mixer control, aligned with LCM
typedef enum : uint16_t {
    Stopped   = 0,
    Priming   = 10,
    Running   = 20,
    Rundown   = 30,
    StopMixing = 100,
    CleaningCheckList=150,
    StartResumeCleaning = 200,
    StopCleaning = 300,
    ControlPanelActive = 1000
} MixerStatus;

volatile MixerStatus g_mixer_status = Stopped;
bool primingCommandReceived[NUM_MOTORS] = {false};

void send_step_move_status(int motor_id, uint8_t status) {
    twai_message_t msg = {};
    msg.identifier = CAN_ID_STEP_MOVE_STATUS;
    msg.data_length_code = 2;
    msg.data[0] = motor_id;
    msg.data[1] = status; // 1 = success/done
    twai_transmit(&msg, pdMS_TO_TICKS(20));
}

void send_step_move_ack(int motor_id) {
    twai_message_t msg = {};
    msg.identifier = CAN_ID_STEP_MOVE_ACK;
    msg.data_length_code = 1;
    msg.data[0] = motor_id;
    twai_transmit(&msg, pdMS_TO_TICKS(20));
}



// ------------------- Hardware timer config for motors 4 & 5 -------------------

bool IRAM_ATTR bitbangTimerISR(void *arg) {
  int motor = (int)(intptr_t)arg;
  if (!bitbangRunning[motor]) return false;  // false = keep ISR active

  // Toggle step pin
  bitbangStepLevel[motor] = !bitbangStepLevel[motor];
  gpio_set_level(STEP_PINS[motor + RMT_MOTORS], bitbangStepLevel[motor]);

  // Set direction before pulse
  if (bitbangStepLevel[motor] == false) { // falling edge
    gpio_set_level(DIR_PINS[motor + RMT_MOTORS], bitbangDirection[motor] ? 1 : 0);
  }

  return true; // true = keep ISR active
}

void setupBitbangTimer(int motor) {
    timer_config_t config = {
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,
        .intr_type = TIMER_INTR_LEVEL,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = TIMER_AUTORELOAD_EN,
        .divider = 80  // 1 tick = 1us
    };

    timer_init(TIMER_GROUP_0, (timer_idx_t)motor, &config);
    timer_set_counter_value(TIMER_GROUP_0, (timer_idx_t)motor, 0);
    timer_enable_intr(TIMER_GROUP_0, (timer_idx_t)motor);
    timer_isr_callback_add(TIMER_GROUP_0, (timer_idx_t)motor,
                           bitbangTimerISR, (void *)(intptr_t)motor, 0);
}


void setBitbangFrequency(int motor, float rpm) {
  if (rpm <= 0) {
    timer_pause(TIMER_GROUP_0, (timer_idx_t)motor);
    bitbangRunning[motor] = false;
    gpio_set_level(STEP_PINS[motor], 0);
    return;
  }
  float freqHz = rpm * MICROSTEPS / 60.0f;
  int period_us = (int)(1e6f / (freqHz * 2)); // half period for toggle
  if (period_us < 1) period_us = 1;

  timer_set_alarm_value(TIMER_GROUP_0, (timer_idx_t)motor, period_us);
  timer_start(TIMER_GROUP_0, (timer_idx_t)motor);
  bitbangDirection[motor] = motorStates[motor].direction;
  bitbangRunning[motor] = true;
}

// ------------------- Motor control helpers -------------------

void setupRMTChannel(int motor) {
  if (motor >= RMT_MOTORS) return;  // Only first 4 motors use RMT
  rmt_config_t config;
  config.channel = RMT_CHANNELS[motor];
  config.gpio_num = STEP_PINS[motor];
  config.clk_div = CLOCK_DIV;
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

void enableMotor(int motor, bool enable, bool direction) {
  gpio_set_level(DIR_PINS[motor], direction ? 1 : 0);
  gpio_set_level(EN_PINS[motor], enable ? 0 : 1); // EN active low
}

void setRMTFrequency(int motor, float rpm) {
  if (motor >= RMT_MOTORS) {
    setBitbangFrequency(motor - RMT_MOTORS, rpm);
    return;
  }
  if (rpm <= 0) {
    rmt_tx_stop(RMT_CHANNELS[motor]);
    return;
  }
  float stepsPerSec = rpm * (MICROSTEPS*2) / 60.0f;
  int period_us = (int)(1e6f / stepsPerSec);
  if (period_us < PULSE_WIDTH_US * 2) period_us = PULSE_WIDTH_US * 2;

  rmt_item32_t item;
  item.duration0 = PULSE_WIDTH_US;
  item.level0 = 1;
  item.duration1 = period_us - PULSE_WIDTH_US;
  item.level1 = 0;

  rmt_write_items(RMT_CHANNELS[motor], &item, 1, true);
  rmt_set_tx_loop_mode(RMT_CHANNELS[motor], true);
  rmt_tx_start(RMT_CHANNELS[motor], true);
}

// ------------------- Master motor task -------------------


void masterSpeedTask(void* pvParameters) {
    int motor = *((int*)pvParameters);
    free(pvParameters);
    StepMoveRequest stepReq = {0, 0, true};
    SpeedMoveRequest speedReq = {false, 0, true};
    // motorStates[motor] = {0, 0, true, false, false, 0, 0};
    int iter=0;

    while (true) {
        // if (solids_primed && motor<=3){continue;}
        // Check for step move request
        if (xQueueReceive(stepQueues[motor], &stepReq, 0) == pdTRUE) {
            if (stepReq.steps > 0 && stepReq.rpm > 0) {
                motorStates[motor].stepsRemaining = stepReq.steps;
                // motorStates[motor].totalSteps = stepReq.steps;
                motorStates[motor].stepMoveActive = true;
                motorStates[motor].speedMoveActive = false; // cancel any continuous move
                motorStates[motor].targetRPM = stepReq.rpm;
                motorStates[motor].currentRPM = 0; // start from 0 for acceleration
                motorStates[motor].direction = stepReq.direction;
            }
        }

        // Check for continuous speed request
        if (xQueueReceive(speedQueues[motor], &speedReq, 0) == pdTRUE) {
            motorStates[motor].speedMoveActive = speedReq.active;
            if (speedReq.active) {
                motorStates[motor].targetRPM = speedReq.rpm;
                motorStates[motor].direction = speedReq.direction;
                motorStates[motor].stepMoveActive = false; // cancel any step move
            } else if (!motorStates[motor].stepMoveActive) {
                motorStates[motor].targetRPM = 0;
            }
        }

        // Handle step move
        if (motorStates[motor].stepMoveActive) {
            float targetRPM = motorStates[motor].targetRPM;
            float currentRPM = motorStates[motor].currentRPM;
            float rampStep = ACCELERATION_RPM_PER_SEC * (CONTROL_PERIOD_MS / 1000.0f);

            // Ramp up/down toward target
            if (currentRPM < targetRPM) currentRPM = fmin(currentRPM + rampStep, targetRPM);
            else if (currentRPM > targetRPM) currentRPM = fmax(currentRPM - rampStep, targetRPM);

            // Apply speed
            motorStates[motor].currentRPM = currentRPM;
            enableMotor(motor, true, motorStates[motor].direction);
            setRMTFrequency(motor, currentRPM);

            // Calculate how many steps were taken this period
            float stepsPerSec = currentRPM * MICROSTEPS / 60.0f;
            int stepsThisPeriod = (int)round(stepsPerSec * (CONTROL_PERIOD_MS / 1000.0f));
            if (stepsThisPeriod < 1) stepsThisPeriod = 1; // priming_status_41Aensure at least 1 step

            motorStates[motor].stepsRemaining -= stepsThisPeriod;

            // Stop if done
            if (motorStates[motor].stepsRemaining <= 0) {
                motorStates[motor].currentRPM = 0;
                setRMTFrequency(motor, 0);
                enableMotor(motor, false, motorStates[motor].direction);
                motorStates[motor].stepMoveActive = false;
                // send_step_move_status(motor, 1);
                //Send status label to CAN 
                priming_status_41A.data[motor+2] = 1; // 1 = success/done
                        }
            // if (priming_status_41A.data[5]==1 && priming_status_41A.data[6]==1)
            // {
            //   solids_primed=true;
            // }
        }

        // Handle continuous speed mode
        else if (motorStates[motor].speedMoveActive) {
            float targetRPM = motorStates[motor].targetRPM;
            float currentRPM = motorStates[motor].currentRPM;
            float rampStep = ACCELERATION_RPM_PER_SEC * (CONTROL_PERIOD_MS / 1000.0f);

            if (currentRPM < targetRPM) currentRPM = fmin(currentRPM + rampStep, targetRPM);
            else if (currentRPM > targetRPM) currentRPM = fmax(currentRPM - rampStep, targetRPM);

            if (fabs(currentRPM - motorStates[motor].currentRPM) > 0.01f) {
                motorStates[motor].currentRPM = currentRPM;
                enableMotor(motor, true, motorStates[motor].direction);
                setRMTFrequency(motor, currentRPM);
            }
        }

        // Idle (stop motor)
        else {
            if (motorStates[motor].currentRPM > 0) {
                motorStates[motor].currentRPM = 0;
                setRMTFrequency(motor, 0);
                enableMotor(motor, false, motorStates[motor].direction);
            }
        }
        iter+=1;
        if (iter==100)
        {
        twai_transmit(&priming_status_41A, pdMS_TO_TICKS(20));
        iter=0;
        }
        vTaskDelay(CONTROL_PERIOD_MS / portTICK_PERIOD_MS);
    }
}


// void masterSpeedTask(void* pvParameters) {
//     int motor = *((int*)pvParameters);
//     free(pvParameters);
//     StepMoveRequest stepReq = {0, 0, true};
//     SpeedMoveRequest speedReq = {false, 0, true};
//     motorStates[motor] = {0, 0, true, false, false, 0, 0};

//     while (true) {
//         // Check for step move request
//         if (xQueueReceive(stepQueues[motor], &stepReq, 0) == pdTRUE) {
//             if (stepReq.steps > 0 && stepReq.rpm > 0) {
//                 motorStates[motor].stepsRemaining = stepReq.steps;
//                 motorStates[motor].totalSteps = stepReq.steps;
//                 motorStates[motor].stepMoveActive = true;
//                 motorStates[motor].speedMoveActive = false; // cancel any continuous move
//                 motorStates[motor].targetRPM = stepReq.rpm;
//                 motorStates[motor].currentRPM = 0; // start from 0 for acceleration
//                 motorStates[motor].direction = stepReq.direction;
//             }
//         }

//         // Check for continuous speed request
//         if (xQueueReceive(speedQueues[motor], &speedReq, 0) == pdTRUE) {
//             motorStates[motor].speedMoveActive = speedReq.active;
//             if (speedReq.active) {
//                 motorStates[motor].targetRPM = speedReq.rpm;
//                 motorStates[motor].direction = speedReq.direction;
//                 motorStates[motor].stepMoveActive = false; // cancel any step move
//             } else if (!motorStates[motor].stepMoveActive) {
//                 motorStates[motor].targetRPM = 0;
//             }
//         }

//         // Handle step move
//         if (motorStates[motor].stepMoveActive) {
//             float targetRPM = motorStates[motor].targetRPM;
//             float currentRPM = motorStates[motor].currentRPM;
//             float rampStep = ACCELERATION_RPM_PER_SEC * (CONTROL_PERIOD_MS / 1000.0f);

//             // Ramp up/down toward target
//             if (currentRPM < targetRPM) currentRPM = fmin(currentRPM + rampStep, targetRPM);
//             else if (currentRPM > targetRPM) currentRPM = fmax(currentRPM - rampStep, targetRPM);

//             // Apply speed
//             motorStates[motor].currentRPM = currentRPM;
//             enableMotor(motor, true, motorStates[motor].direction);
//             setRMTFrequency(motor, currentRPM);

//             // Calculate how many steps were taken this period
//             float stepsPerSec = currentRPM * MICROSTEPS / 60.0f;
//             int stepsThisPeriod = (int)round(stepsPerSec * (CONTROL_PERIOD_MS / 1000.0f));
//             if (stepsThisPeriod < 1) stepsThisPeriod = 1; // ensure at least 1 step

//             motorStates[motor].stepsRemaining -= stepsThisPeriod;

//             // Stop if done
//             if (motorStates[motor].stepsRemaining <= 0) {
//                 motorStates[motor].currentRPM = 0;
//                 setRMTFrequency(motor, 0);
//                 enableMotor(motor, false, motorStates[motor].direction);
//                 motorStates[motor].stepMoveActive = false;
//             }
//         }

//         // Handle continuous speed mode
//         else if (motorStates[motor].speedMoveActive) {
//             float targetRPM = motorStates[motor].targetRPM;
//             float currentRPM = motorStates[motor].currentRPM;
//             float rampStep = ACCELERATION_RPM_PER_SEC * (CONTROL_PERIOD_MS / 1000.0f);

//             if (currentRPM < targetRPM) currentRPM = fmin(currentRPM + rampStep, targetRPM);
//             else if (currentRPM > targetRPM) currentRPM = fmax(currentRPM - rampStep, targetRPM);

//             if (fabs(currentRPM - motorStates[motor].currentRPM) > 0.01f) {
//                 motorStates[motor].currentRPM = currentRPM;
//                 enableMotor(motor, true, motorStates[motor].direction);
//                 setRMTFrequency(motor, currentRPM);
//             }
//         }

//         // Idle (stop motor)
//         else {
//             if (motorStates[motor].currentRPM > 0) {
//                 motorStates[motor].currentRPM = 0;
//                 setRMTFrequency(motor, 0);
//                 enableMotor(motor, false, motorStates[motor].direction);
//             }
//         }

//         vTaskDelay(CONTROL_PERIOD_MS / portTICK_PERIOD_MS);
//     }
// }

// ------------------- Command functions -------------------

void speedMove(int motor, bool active, float rpm, bool direction) {
  if (motor < 0 || motor >= NUM_MOTORS) return;
  SpeedMoveRequest req = {active, rpm, direction};
  xQueueSend(speedQueues[motor], &req, pdMS_TO_TICKS(100));
}

void stepMove(int motor, float distance_mm, float rpm, bool direction) {
  if (motor < 0 || motor >= NUM_MOTORS) return;
  if (rpm <= 0 || fabs(distance_mm) < 0.0001f) return;

  // Calculate steps
  float revs = fabs(distance_mm) / MM_PER_REV[motor];
  int steps = (int)(revs * MICROSTEPS);

  StepMoveRequest req;
  req.steps = steps;
  req.rpm = rpm;
  req.direction = (distance_mm >= 0) ? direction : !direction;

  xQueueSend(stepQueues[motor], &req, pdMS_TO_TICKS(100));
}

void stopMotor(int motor) {
  if (motor < 0 || motor >= NUM_MOTORS) return;

  // Stop pulse generation
  if (motor < RMT_MOTORS) {
    rmt_tx_stop(RMT_CHANNELS[motor]);
  } else {
    setBitbangFrequency(motor - RMT_MOTORS, 0);
  }

  // Disable driver (EN pin active low)
  gpio_set_level(EN_PINS[motor], 1);

  // Reset motor state
  motorStates[motor].currentRPM = 0;
  motorStates[motor].targetRPM = 0;
  motorStates[motor].stepMoveActive = false;
  motorStates[motor].speedMoveActive = false;
  motorStates[motor].stepsRemaining = 0;
  motorStates[motor].totalSteps = 0;
}

// ------------------- CAN Bus Functions -------------------

static inline float read_f32_le(const uint8_t* b) { float f; memcpy(&f, b, 4); return f; }
static inline uint16_t read_u16_le(const uint8_t* b) { return (uint16_t)b[0] | ((uint16_t)b[1] << 8); }

// Logic for simple jog commands in Maintenance Mode
void handle_maintenance_command(int motor_num, uint8_t command) {
    float target_rpm = 0;
    bool is_active = true;

    switch (command) {
        case 0: is_active = false; break;
        case 1: target_rpm = CONTROL_PANEL_SPEED_RPM; break;
        case 10: target_rpm = FULL_SPEED_RPM; break;
        default: return;
    }
    speedMove(motor_num, is_active, target_rpm, true); // Assume forward
}

void process_can_message(const twai_message_t* message) {
    // First, handle the MixerStatus message regardless of the current state
    if (message->identifier == CAN_ID_MIXER_STATUS) {
        if (message->data_length_code >= 2) {
            MixerStatus new_status = (MixerStatus)read_u16_le(message->data);
            if (new_status != g_mixer_status) {
                g_mixer_status = new_status;
                if (new_status==0) ESP.restart(); //Restart whenever home is pressed
                // If we are no longer priming, reset all the ack flags for the next cycle
                if (g_mixer_status != Priming) {
                    for (int i = 0; i < NUM_MOTORS; i++) {
                        primingCommandReceived[i] = false;
                    }
                }
            }
             // This is a failsafe in case the LCM stops sending RPM 0 commands.
            if (g_mixer_status == Stopped || g_mixer_status == StopMixing || g_mixer_status == StopCleaning) {
                for (int i = 0; i < NUM_MOTORS; i++) {
                    stopMotor(i);
                }
            }
        }
        return; // Status message handled
    }

    // Handle variable RPM commands from the LCM
    if (message->identifier >= CAN_ID_MOTOR_RPM_BASE && message->identifier < (CAN_ID_MOTOR_RPM_BASE + NUM_MOTORS)) {
        int motor_id = message->identifier - CAN_ID_MOTOR_RPM_BASE;
        if (message->data_length_code >= 4) {
            float rpm = read_f32_le(message->data);

            Serial.printf("CAN RX: Motor %d, RPM: %.2f, Status: %d\n", motor_id, rpm, g_mixer_status);

            // FAILSAFE-FIRST LOGIC: If we are not in a valid run state, force a stop.
            if (g_mixer_status != Running && g_mixer_status != Rundown && g_mixer_status != StartResumeCleaning) {
                speedMove(motor_id, false, 0.0f, true);
            } else {
                // Otherwise, process the command as requested.
                speedMove(motor_id, rpm > 0.1f, rpm, true);
            }
        }
        return;
    }

    // If in Control Panel mode, only listen to the grouped jog commands
    if (g_mixer_status == ControlPanelActive || g_mixer_status == CleaningCheckList) {
        switch (message->identifier) {
            case CAN_ID_LIQUID_MOTORS:
                if (message->data_length_code >= 8) {
                    handle_maintenance_command(0, message->data[0]);
                    handle_maintenance_command(1, message->data[2]);
                    handle_maintenance_command(2, message->data[4]);
                    handle_maintenance_command(3, message->data[6]);
                }
                break;
            case CAN_ID_SOLID_MOTORS:
                if (message->data_length_code >= 4) {
                    handle_maintenance_command(4, message->data[0]);
                    handle_maintenance_command(5, message->data[2]);
                }
                break;
        }
        return;
    }

    // Check for step move (priming) command
    if (message->identifier == CAN_ID_STEP_MOVE) {
        if (g_mixer_status == Priming && message->data_length_code == 8) {
            uint8_t motor_id = message->data[0];
            // Only process the first command in the stream
            if (!primingCommandReceived[motor_id]) {
                primingCommandReceived[motor_id] = true;
                bool direction = (bool)message->data[1];
                float distance_mm;
                memcpy(&distance_mm, &message->data[2], sizeof(distance_mm));
                uint16_t rpm = read_u16_le(&message->data[6]);
                
                // Serial.printf("PRIMING CMD: Motor %d received, sending ACK.\n", motor_id);
                send_step_move_ack(motor_id);
                stepMove(motor_id, distance_mm, (float)rpm, direction);
            }
        }
        return;
    }
}

void can_receive_task(void* pvParameters) {
    twai_message_t rx_message;
    while (true) {
        if (twai_receive(&rx_message, portMAX_DELAY) == ESP_OK) {
            process_can_message(&rx_message);
        }
    }
}

void setup_can_driver() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) return;
    if (twai_start() != ESP_OK) return;
}



// ------------------- Setup & loop -------------------

void setup() {
    Serial.begin(115200);

    gpio_set_direction(gpio_num_t(38), GPIO_MODE_OUTPUT);
    gpio_set_level(gpio_num_t(38), 1);
    

    priming_status_41A.identifier = CAN_ID_STEP_MOVE_STATUS;
    priming_status_41A.data_length_code = 8;

    for (int i = 0; i < NUM_MOTORS; i++) {
        gpio_set_direction(DIR_PINS[i], GPIO_MODE_OUTPUT);
        gpio_set_direction(EN_PINS[i], GPIO_MODE_OUTPUT);
        gpio_set_direction(STEP_PINS[i], GPIO_MODE_OUTPUT);
        gpio_set_level(EN_PINS[i], 1);
        gpio_set_level(DIR_PINS[i], 1);

        if (i < RMT_MOTORS) {
            setupRMTChannel(i);
        } else {
            setupBitbangTimer(i - RMT_MOTORS);
        }

        stepQueues[i] = xQueueCreate(5, sizeof(StepMoveRequest));
        speedQueues[i] = xQueueCreate(5, sizeof(SpeedMoveRequest));
        
        int* pMotor = (int*)malloc(sizeof(int));
        *pMotor = i;

        char task_name[16];
        sprintf(task_name, "MotorTask%d", i);

        BaseType_t xReturned = xTaskCreatePinnedToCore(
            masterSpeedTask,
            task_name,
            16384,
            pMotor,
            1,
            NULL,
            1); // Pin to Core 1

        if (xReturned != pdPASS) {
            Serial.printf("Error creating task for motor %d\n", i);
        }
    }
    
    setup_can_driver();
    xTaskCreatePinnedToCore(can_receive_task, "CAN_Rx", 16384, NULL, 1, NULL, 0); // Pin to Core 0


  // Serial.begin(115200);
  // // gpio_pad_select_gpio(BUTTON_GPIO);
  // gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
  // gpio_set_direction(gpio_num_t(38), GPIO_MODE_OUTPUT);
  // gpio_set_level(gpio_num_t(38), 1);
  // for (int i = 0; i < NUM_MOTORS; i++) {
  //   gpio_set_direction(DIR_PINS[i], GPIO_MODE_OUTPUT);
  //   gpio_set_direction(EN_PINS[i], GPIO_MODE_OUTPUT);
  //   gpio_set_direction(STEP_PINS[i], GPIO_MODE_OUTPUT);
  //   gpio_set_level(EN_PINS[i], 1);
  //   gpio_set_level(DIR_PINS[i], 1);
  //   if (i < RMT_MOTORS) setupRMTChannel(i);
  //   else setupBitbangTimer(i - RMT_MOTORS);

  //   stepQueues[i] = xQueueCreate(5, sizeof(StepMoveRequest));
  //   speedQueues[i] = xQueueCreate(5, sizeof(SpeedMoveRequest));
  //   int* pMotor = (int*)malloc(sizeof(int));
  //   *pMotor = i;
  //   xTaskCreate(masterSpeedTask, "MasterSpeed", 8192, pMotor, 1, NULL);
  // }
}

void loop() {
// float totalRPM = 450;
  // stepMove(0, 50.0, 300, true);
  // vTaskDelay(5000 / portTICK_PERIOD_MS);
  // Read the state of the button
  // int button_state = gpio_get_level(BUTTON_PIN);
  // if (button_state == 0) {
  //   stopMotor(0);
  //   stopMotor(1);
  //   stopMotor(2);
  //   stopMotor(3);
  //   stopMotor(4);
  //   stopMotor(5);
  //   vTaskDelay(5000 / portTICK_PERIOD_MS);
  //   speedMove(1, true, totalRPM, true);
  //   speedMove(2, true, totalRPM, true);
  //   speedMove(0, true, totalRPM, true);
  //   speedMove(3, true, totalRPM, true);
  //   speedMove(4, true, totalRPM, true);  // bitbang with timer
  //   speedMove(5, true, totalRPM, true);  // bitbang with timer
  //   vTaskDelay(2000 / portTICK_PERIOD_MS);
  //   stopMotor(0);
  //   stopMotor(1);
  //   stopMotor(2);
  //   stopMotor(3);
  //   stopMotor(4);
  //   stopMotor(5);
  //   vTaskDelay(5000 / portTICK_PERIOD_MS);
  //     // Button is pressed
  // } else {
  //   speedMove(1, true, totalRPM, true);
  //   speedMove(2, true, totalRPM, true);
  //   speedMove(0, true, totalRPM, true);
  //   speedMove(3, true, totalRPM, true);
  //   speedMove(4, true, totalRPM, true);  // bitbang with timer
  //   speedMove(5, true, totalRPM, true);  // bitbang with timer
  //   // vTaskDelay(5000 / portTICK_PERIOD_MS);
  //   // stopMotor(0);
  //   vTaskDelay(100 / portTICK_PERIOD_MS);
  // }
  
}