#include "driver/twai.h"
#include <Arduino.h>
#include <string.h>
#include <math.h>
#include <driver/gpio.h>
#include <driver/adc.h>


template<typename T, size_t N>
class SimpleMovingAverage {
private:
    T readings[N] = {0}; // An array to store the last N readings (our "window")
    size_t index = 0;    // The current position in the array
    T sum = 0;           // The running sum of the readings in the window
    size_t count = 0;    // The number of readings added so far (handles the initial fill)

public:
    // Adds a new reading to the filter
    void add(T new_reading) {
        sum -= readings[index];      // Subtract the oldest reading from the sum
        readings[index] = new_reading; // Overwrite the oldest reading with the new one
        sum += new_reading;          // Add the new reading to the sum
        index = (index + 1) % N;     // Advance the index, wrapping around if necessary
        if (count < N) {
            count++; // Increment count until the buffer is full
        }
    }

    // Gets the current calculated average
    T getAverage() const {
        if (count == 0) return 0; // Prevent division by zero if no readings have been added
        return sum / count;       // The average is the sum divided by the number of readings
    }
};

namespace CanMixer {

// =================================================================================
// MARK: - Configuration & Constants
// =================================================================================

namespace Config {
    // --- Custom ESP32-S3 Pin Assignments ---
    constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_35;
    constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_36;
    
    // Analog Inputs for ESP32-S3
    constexpr adc2_channel_t ADC_NOZZLE_PRESSURE = ADC2_CHANNEL_1; // GPIO 11
    constexpr adc2_channel_t ADC_SYSTEM_PRESSURE = ADC2_CHANNEL_0; // GPIO 12

    // Relay Outputs
      constexpr gpio_num_t RELAY_NOZZLE_OPEN        = GPIO_NUM_7;
      constexpr gpio_num_t RELAY_NOZZLE_CLOSE       = GPIO_NUM_10;
      constexpr gpio_num_t RELAY_SYSTEM_OPEN        = GPIO_NUM_1;
      constexpr gpio_num_t RELAY_SYSTEM_CLOSE       = GPIO_NUM_2;
      constexpr gpio_num_t RELAY_WATER_INLET_OPEN   = GPIO_NUM_4;
      constexpr gpio_num_t RELAY_WATER_INLET_CLOSE  = GPIO_NUM_3;
      constexpr gpio_num_t RELAY_SPRAY_OUTLET_OPEN  = GPIO_NUM_5;
      constexpr gpio_num_t RELAY_SPRAY_OUTLET_CLOSE = GPIO_NUM_6;

    enum class CanID : uint32_t {
        Heartbeat = 0x400,
        MixerStatus = 0x401,
        DoserStatusSolids  = 0x414,
        DoserStatusLiquids = 0x419,
        SolidMotorsCmd = 0x464,  // For motors 4, 5 (Maintenance Mode Only)
        LiquidMotorsCmd = 0x469, // For motors 0, 1, 2, 3 (Maintenance Mode Only)
        WaterInletValve = 0x482,
        SprayOutletValve = 0x487,
        PropSystemPressure = 0x48C,
        PropNozzlePressure = 0x48E,
        TareCmd = 0x4AB,
        MixerTankWeight = 0x4D2,
        PressureSensorData = 0x4C8,
        NozzlePressureSetpoint = 0x538,
        SystemPressureSetpoint = 0x539,
        NumFaces               = 0x536,
        TargetSpeed            = 0x53A,
        RowSpacing             = 0x540,
        SprayVolHa             = 0x542,
        S1_RateUnit=0x59C, S1_Ccv=0x59E,
        S2_RateUnit=0x5A6, S2_Ccv=0x5A8,
        L1_RateUnit=0x5B0, L1_Ccv=0x5B2,
        L2_RateUnit=0x5BA, L2_Ccv=0x5BC,
        L3_RateUnit=0x5C4, L3_Ccv=0x5C6,
        L4_RateUnit=0x5CE, L4_Ccv=0x5D0,
        StepMove               = 0x501,
        StepMoveStatus         = 0x502,
        StepMoveAck            = 0x503, // MCM acknowledges receipt of a step move command
        HopperStatusBase       = 0x5D1,
        LcmTelemetry           = 0x5F0,
        MotorRpmBase           = 0x4A0,
        CleaningCycles         = 0x45A
    };

    enum class Motor : uint8_t { Liquid1=0, Liquid2, Liquid3, Liquid4, Solid1, Solid2, Count };

    bool SprayOutValveStatus=false;
    static bool waitingForPressure = false;

    enum class MixerStatus : uint16_t {
        Stopped   = 0,
        Tare      = 8,
        Priming   = 10,
        Running   = 20,
        Rundown   = 30,
        StopMixing = 100,
        CleaningCheckList = 150,
        CleaningPurge = 160,
        StartResumeCleaning = 200,
        StopCleaning = 300,
        ControlPanelActive = 1000
    };

    enum class MotorCmd : uint8_t {
        Off = 0,
        PanelSpeed = 1,
        FullSpeed = 10
    };


    struct CleaningState{
      int iter =0;
      int status=0;
      bool visited_flag=false;
    };
    
    constexpr float TANK_FILL_TARGET_L = 20.0f;                                                                                                                                             
    constexpr float TANK_MAINTAIN_HYSTERESIS_L = 0.5f;
    constexpr float PRIMING_RPM = 50.0f;
    constexpr float PRIMING_TANK_VOLUME_L = 20.0f;
    constexpr float LIQUID_ML_PER_DEG = 33.10f;
    constexpr int DENSITY_G_PER_ML = 1.0f;
    // constexpr uint16_t PAUSE_CODE = 110;
    constexpr int SENSOR_READ_INTERVAL_MS = 100;
    constexpr int CONTROL_LOOP_INTERVAL_MS = 100;
    constexpr int TANK_FILL_INTERVAL_MS = 250;
    constexpr int SPRAY_VALVE_INTERVAL_MS = 250;
    // constexpr int ACTUATION_WINDOW_MS = 200;
    constexpr float RPM_MAX = 4000.0f;
    constexpr int HEARTBEAT_TIMEOUT_MS = 2000; // 2 seconds
    constexpr float PRESSURE_HYSTERESIS_BAR = 0.5f; // Deadband of +/- 0.2 bar
    constexpr int VALVE_CONTROL_INTERVAL_MS = 100;  // Check pressure every 100ms
    constexpr float PRESSURE_MAX_BAR = 50.0f; // Max pressure of your sensor
    constexpr int ADC_FULL_SCALE = 4095;
    constexpr int ADC_MIN_RAW = ADC_FULL_SCALE * 0.15; // Raw ADC for 0 Bar
    constexpr int ADC_MAX_RAW = ADC_FULL_SCALE * 1;   // Raw ADC for Max Bar
    const float filterAlpha = 0.5; // Tune between 0.1 (heavy) and 0.5 (light)
    constexpr float CLEANING_RPM = 100.0f;
    


    // --- Proportional Valve Control ---
    constexpr int PWM_ACTIVATION_WINDOW_MS = 500;
    constexpr int PWM_CONTROL_LOOP_INTERVAL_MS = 20; // Faster loop for responsive PWM
    constexpr float NOZZLE_KP = 30.0f; // Proportional gain for Nozzle. TUNE THIS.
    constexpr float SYSTEM_KP = 9.0f; // Proportional gain for System. TUNE THIS.
    constexpr size_t NOZZLE_UI_FILTER_SIZE = 10; // Number of samples to average for the UI display.


    constexpr float MCM_MM_PER_REV[static_cast<size_t>(Config::Motor::Count)] = {5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f};
    constexpr float SOLID_DENSITY_G_PER_ML[2] = {0.85f, 0.90f};
}

static Config::CleaningState cleaningStateInstance;
// =================================================================================
// MARK: - Data Models
// =================================================================================

enum class FillState { IDLE, WAITING_FOR_TARE, FILLING, MAINTAINING };

struct SessionInputs {
    float    rowSpacing_m = 0.0f, targetSpeed_mps = 0.0f, sprayVolHa_L = 0.0f;
    uint16_t numFaces = 0;
};

struct TreatmentInput {
    bool   loaded = false, paused = false;
    float  rate_kg_per_ha = 0.0f, ccv_deg_per_g = 0.0f;
    bool   priming_complete = false;
    bool   priming_status_sent = false;
    bool   primingAckReceived = false; // Handshake flag
};

struct ValveController {
    float        input, setpoint;
    bool         enabled = true;
    gpio_num_t   relayOpenPin, relayClosePin;
    bool         direction; // true=direct, false=inverse

    // --- NEW: PWM State ---
    float        Kp; // Proportional gain
    unsigned long window_start_time = 0;
    long         activation_duration_ms = 0;
    gpio_num_t   active_relay_pin = GPIO_NUM_NC; // GPIO_NUM_NC indicates no pin is active
};


struct ControllerState {
    SessionInputs  session;
    TreatmentInput motors[static_cast<size_t>(Config::Motor::Count)];
    ValveController  systemController;
    ValveController nozzleController;
    Config::MixerStatus mixerStatus = Config::MixerStatus::Stopped;
    bool           priming_initiated = false;
    FillState      fill_state = FillState::WAITING_FOR_TARE;
    float          current_tank_weight_kg = 0.0f;
    bool           tare_complete = false;
    int            tare_status=0;
    int            waterinletvalve;
    int            sprayoutletvalve;
    int            propsystempressure;
    int            propnozzlepressure;
};


static ControllerState state;
volatile unsigned long lastHeartbeatTime = 0;
static SimpleMovingAverage<float, Config::NOZZLE_UI_FILTER_SIZE> nozzleUiFilter;
static float nozzle_pressure_for_ui = 0.0f;
// =================================================================================
// Helpers & Core Logic
// =================================================================================

static inline float read_f32_le(const uint8_t* b) { float f; memcpy(&f, b, 4); return f; }
static inline uint16_t read_u16_le(const uint8_t* b) { return (uint16_t)b[0] | ((uint16_t)b[1] << 8); }
static inline void write_f32_le(uint8_t* b, float v) { memcpy(b, &v, 4); }
static inline int32_t read_i32_le(const uint8_t* b) { int32_t i; memcpy(&i, b, 4); return i; }
static inline void write_u16_le(uint8_t* b, uint16_t v) { b[0] = v & 0xFF; b[1] = (v >> 8) & 0xFF; }

float calculate_pressure(int raw_value) {
    using namespace Config;

    // Constrain the reading to the calibrated range
    if (raw_value <= ADC_MIN_RAW) {
        return 0.0f;
    }
    if (raw_value >= ADC_MAX_RAW) {
        return PRESSURE_MAX_BAR;
    }

    // Calculate pressure using linear interpolation
    float scaled_value = (float)(raw_value - ADC_MIN_RAW) / (float)(ADC_MAX_RAW - ADC_MIN_RAW);
    
    return scaled_value * PRESSURE_MAX_BAR;
}

void update_valve_pwm(ValveController& controller) {
    if (int(state.mixerStatus) == 1000) return;
    
    unsigned long current_time = millis();

    // --- Part 1: Manage Active Relay (Turn OFF if time up) ---
    if (controller.active_relay_pin != GPIO_NUM_NC) {
        if (current_time - controller.window_start_time >= controller.activation_duration_ms) {
            gpio_set_level(controller.active_relay_pin, 0); 
            controller.active_relay_pin = GPIO_NUM_NC; 
        }
    }

    // --- Part 2: Start a New PWM Window ---
    if (current_time - controller.window_start_time < Config::PWM_ACTIVATION_WINDOW_MS) {
        return; 
    }

    // Reset for new window
    controller.window_start_time = current_time;
    gpio_set_level(controller.relayOpenPin, 0);
    gpio_set_level(controller.relayClosePin, 0);
    controller.active_relay_pin = GPIO_NUM_NC;
    controller.activation_duration_ms = 0;

    // [CRITICAL FIX] PRIORITY 1: Enabled Check (HOLD Logic)
    // If disabled, do NOTHING. This holds the valve in current position.
    if (!controller.enabled) {
        return; 
    }

    // [CRITICAL FIX] PRIORITY 2: Zero Setpoint Logic (Dump/Close)
    // Only runs if enabled. Forces valve to safe state if setpoint is ~0.
    if (controller.setpoint < 0.1) {
        controller.activation_duration_ms = Config::PWM_ACTIVATION_WINDOW_MS; 
        
        if (!controller.direction) { 
            // INVERSE (System): Setpoint 0 -> Fully OPEN to dump pressure
            controller.active_relay_pin = controller.relayOpenPin;
        } else { 
            // DIRECT (Nozzle): Setpoint 0 -> Fully CLOSE to stop flow
            controller.active_relay_pin = controller.relayClosePin;
        }
        
        gpio_set_level(controller.active_relay_pin, 1);
        return; 
    }

    // --- Part 3: Proportional (P-Only) Calculation ---
    float error = controller.setpoint - controller.input;
    
    if (abs(error) < Config::PRESSURE_HYSTERESIS_BAR) {
        return; 
    }
    
    float output_percent = controller.Kp * error;
    output_percent = constrain(output_percent, -100.0, 100.0);

    controller.activation_duration_ms = (Config::PWM_ACTIVATION_WINDOW_MS * abs(output_percent)) / 100.0;

    if (controller.activation_duration_ms < 20) { 
        controller.activation_duration_ms = 0;
        return;
    }

    bool needs_pressure_increase = (output_percent > 0);
    
    if (controller.direction) { 
        // Direct (Nozzle): Positive Error -> OPEN
        controller.active_relay_pin = needs_pressure_increase ? controller.relayOpenPin : controller.relayClosePin;
    } else { 
        // Inverse (System): Positive Error -> CLOSE
        controller.active_relay_pin = needs_pressure_increase ? controller.relayClosePin : controller.relayOpenPin;
    }
    
    gpio_set_level(controller.active_relay_pin, 1);
}

void checkStabilization() {
    // If we aren't waiting, do nothing
    if (!CanMixer::Config::waitingForPressure) return;

    float currentP = state.systemController.input;
    float targetP = state.systemController.setpoint;

    // Safety check: if setpoint is essentially zero, don't block
    if (targetP < 0.5f) return;

    // Calculate 10% Threshold
    float threshold = targetP * 0.10f;
    float error = abs(currentP - targetP);

    // Check if within range
    if (error < threshold) {
        // --- STABILIZED! RESUME NORMAL OPS ---
        CanMixer::Config::waitingForPressure = false;
        
        // 1. Wake up Nozzle PID
        state.nozzleController.enabled = true;
        
        Serial.println("SYSTEM STABILIZED: Enabling Nozzle Controller.");
    } else {
        // Ensure Nozzle stays disabled while waiting
        state.nozzleController.enabled = false;
    }
}

void valve_control_task(void* pv) {
    while(true) {
        checkStabilization();
        // --- Control Nozzle and System Valves using PWM ---
        update_valve_pwm(state.nozzleController);
        update_valve_pwm(state.systemController);
        if (state.systemController.setpoint < 0.1){
          gpio_set_level(state.systemController.relayOpenPin, 1);
        }

        // You can keep your debugging prints here
        // Serial.printf("N_SP:%.1f, N_IN:%.1f | S_SP:%.1f, S_IN:%.1f\n", 
        //             state.nozzleController.setpoint, state.nozzleController.input,
        //             state.systemController.setpoint, state.systemController.input);
        
        vTaskDelay(pdMS_TO_TICKS(Config::PWM_CONTROL_LOOP_INTERVAL_MS));
    }
}

float to_kg_per_ha(float value, uint16_t unit) {
    using namespace Config;
    switch (unit) {
        case 0: return value;
        case 1: return value / 1000.0f;
        case 2: return value * 28.3495f / 1000.0f;
        case 3: return value * DENSITY_G_PER_ML;
        case 4: return value * DENSITY_G_PER_ML / 1000.0f;
        default: return 0.0f;
    }
}

void SprayOutletValveController(bool flag){
  /*
  Helper function to control the spray outlet valve
  False-Closed
  True-Open
  */
  gpio_set_level(Config::RELAY_SPRAY_OUTLET_CLOSE, !flag);
  gpio_set_level(Config::RELAY_SPRAY_OUTLET_OPEN, flag); 
  CanMixer::Config::SprayOutValveStatus=flag;

  if (flag == false){
    // If valve is now closed, set setpoint to 0
    state.systemController.setpoint = 15.0;
  }
  else {
    // If valve is now open, set the new setpoint
    state.systemController.setpoint = 15.0;
  }
}

float computeRPM(const TreatmentInput& t) {
    if (!t.loaded || t.paused || t.rate_kg_per_ha <= 0.0f || t.ccv_deg_per_g <= 0.0f) return 0.0f;
    if (state.session.targetSpeed_mps <= 0 || state.session.rowSpacing_m <= 0 || state.session.numFaces <= 0) return 0.0f;
    const float areaRate = state.session.targetSpeed_mps * state.session.rowSpacing_m * (float)state.session.numFaces;
    const float chemRate_gps = t.rate_kg_per_ha * (areaRate / 10.0f);
    // Serial.printf("Speed: %.2F, RowSpacing: %.2f, NumFaces: %d, kg/ha: %.2f, CCV: %.2f   ", state.session.targetSpeed_mps,
    //               state.session.rowSpacing_m, state.session.numFaces, t.rate_kg_per_ha, t.ccv_deg_per_g);
    const float deg_per_s = chemRate_gps * (t.ccv_deg_per_g/100.0f);
    float rpm = deg_per_s / 6.0f;
    if (!isfinite(rpm) || rpm < 0) rpm = 0;
    if (rpm > Config::RPM_MAX) rpm = Config::RPM_MAX;
    return rpm;
}

void processDoserStatus(uint8_t motor_index, uint16_t status_code) {
    if (motor_index >= static_cast<size_t>(Config::Motor::Count)) return;
    auto& motor = state.motors[motor_index];
    // Serial.printf("DOSER INPUT: Motor %d, Received Status Code: %u\n", motor_index, status_code);
    switch (status_code) {
        case 0: motor.loaded = false; motor.paused = false; break;
        case 1: motor.loaded = true; motor.paused = false; break;
        case 2: motor.loaded = true; motor.paused = true; break;
        case 110: motor.paused = true; break;
        case 150: motor.loaded = true; motor.paused = false; break;
        default:
            if (static_cast<int16_t>(status_code) < 0) {
                motor.loaded = false;
                motor.paused = true;
            }
            // Serial.printf("DOSER INPUT: Motor %d, Unhandled Status Code: %d\n", motor_index, static_cast<int16_t>(status_code));
            break;
    }
}

void sendHopperStatus(uint8_t motor_index, uint16_t status_code) {
    twai_message_t msg = {};
    msg.identifier = static_cast<uint32_t>(Config::CanID::HopperStatusBase) + motor_index;
    msg.data_length_code = 2;
    write_u16_le(msg.data, status_code);
    twai_transmit(&msg, pdMS_TO_TICKS(10));
    // Serial.printf("DOSER STATUS: Motor %d, Status Code: %u\n", motor_index, status_code);
}

void sendHeartbeatAck() {
    twai_message_t msg = {};
    msg.identifier = static_cast<uint32_t>(Config::CanID::Heartbeat);
    msg.data_length_code = 1;
    msg.data[0] = 1;
    twai_transmit(&msg, pdMS_TO_TICKS(10));

}

/**
 * @brief Streams step move commands to any motor that hasn't been acknowledged by the MCM.
 */
void executePrimingSequence() {
    for (uint8_t i = 0; i < static_cast<size_t>(Config::Motor::Count); ++i) {
        auto& motor = state.motors[i];
        // If motor is not loaded, is paused, has no volume to spray, OR has been acknowledged, skip it.
        // Serial.printf("MTR: %d, loaded %d, paused %d, volperha %.2f, primeack %d", 
        //               i, motor.loaded, motor.paused, state.session.sprayVolHa_L, motor.primingAckReceived);
        if (!motor.loaded || motor.paused || state.session.sprayVolHa_L <= 0 || motor.primingAckReceived) continue;

        float concentration = motor.rate_kg_per_ha / state.session.sprayVolHa_L;
        float priming_volume_L = concentration * Config::PRIMING_TANK_VOLUME_L;
        float priming_volume_mL = priming_volume_L * 1000.0f;
        // Serial.printf("DETAILS: Motor %d, volume: %.2f\n", i, priming_volume_mL);
        float degrees_to_turn = 0;

        bool is_liquid = (i < static_cast<uint8_t>(Config::Motor::Solid1));

        if (is_liquid && Config::LIQUID_ML_PER_DEG > 0) {
            degrees_to_turn = priming_volume_mL * Config::LIQUID_ML_PER_DEG;
            // Serial.printf("DETAILS: Motor %d, Deg: %.2f \n", i, degrees_to_turn);
        } 
        else if (!is_liquid && motor.ccv_deg_per_g > 0) {
            uint8_t solid_idx = i - static_cast<uint8_t>(Config::Motor::Solid1);
            float mass_g = priming_volume_mL * Config::SOLID_DENSITY_G_PER_ML[solid_idx];
            degrees_to_turn = mass_g * (motor.ccv_deg_per_g/100);
            // Serial.printf("DETAILS: Motor %d, mass: %.2f g, volume: %.2f, CCV: %.2f, Deg: %.2f\n", i, mass_g, priming_volume_mL, motor.ccv_deg_per_g, degrees_to_turn);
        }

        if (degrees_to_turn > 0) {
            float revolutions = degrees_to_turn / 360.0f;
            float distance_mm = revolutions * Config::MCM_MM_PER_REV[i];
            uint16_t rpm = static_cast<uint16_t>(Config::PRIMING_RPM);

            twai_message_t msg = {};
            msg.identifier = static_cast<uint32_t>(Config::CanID::StepMove);
            msg.data_length_code = 8;
            msg.data[0] = i;
            msg.data[1] = 1;
            write_f32_le(&msg.data[2], distance_mm);
            write_u16_le(&msg.data[6], rpm);

            if (twai_transmit(&msg, pdMS_TO_TICKS(20)) == ESP_OK) {
                // Serial.printf("STEP MOVE STREAM: Motor %d, Dist: %.2f mm, RPM: %u\n", i, distance_mm, rpm);
            } else {
                // Serial.printf("WARN: Failed to send STEP MOVE command for motor %d\n", i);
            }
        }
    }
}


/**
 * @brief [CORRECTED] Computes RPM for each motor and sends individual CAN messages.
 */
void computeAndSendRPMs() {
    twai_message_t msg = {};
    msg.data_length_code = 4; // RPM is a 4-byte float
    for (uint8_t i = 0; i < static_cast<size_t>(Config::Motor::Count); i++) {
        float rpm = computeRPM(state.motors[i]);
        // Serial.printf("%.2f %d\n, ", rpm, i);
        msg.identifier = static_cast<uint32_t>(Config::CanID::MotorRpmBase) + i;
        write_f32_le(msg.data, rpm);
        if (twai_transmit(&msg, pdMS_TO_TICKS(10)) != ESP_OK) {
            Serial.printf("WARN: Failed to send RPM command for motor %d\n", i);
        }
        vTaskDelay(pdMS_TO_TICKS(5)); // Delay to prevent bus flooding
    }
}

void sendCleaningRPMs() {
    twai_message_t msg = {};
    msg.data_length_code = 4;
    // Set the RPM to the cleaning speed
    write_f32_le(msg.data, Config::CLEANING_RPM); 

    // Loop ONLY through the liquid motors
    for (uint8_t i = static_cast<uint8_t>(Config::Motor::Liquid1); i <= static_cast<uint8_t>(Config::Motor::Liquid4); i++) {
        msg.identifier = static_cast<uint32_t>(Config::CanID::MotorRpmBase) + i;
        if (twai_transmit(&msg, pdMS_TO_TICKS(10)) != ESP_OK) {
            Serial.printf("WARN: Failed to send CLEANING RPM command for motor %d\n", i);
        }
        vTaskDelay(pdMS_TO_TICKS(5)); // Delay to prevent bus flooding
    }
}

/**
 * @brief [CORRECTED] Sends an explicit RPM=0 command to all motors individually.
 */
void sendStopAllMotors() {
    twai_message_t msg = {};
    msg.data_length_code = 4;
    write_f32_le(msg.data, 0.0f); // RPM is 0
    for (uint8_t i = 0; i < static_cast<size_t>(Config::Motor::Count); i++) {
        msg.identifier = static_cast<uint32_t>(Config::CanID::MotorRpmBase) + i;
        if (twai_transmit(&msg, pdMS_TO_TICKS(10)) != ESP_OK) {
            Serial.printf("WARN: Failed to send STOP command for motor %d\n", i);
        }
        vTaskDelay(pdMS_TO_TICKS(5)); // Delay to prevent bus flooding
    }
}


/**
 * @brief Resets all priming acknowledgement flags to false.
 */
void resetPrimingAcks() {
    for (uint8_t i = 0; i < static_cast<size_t>(Config::Motor::Count); ++i) {
        state.motors[i].primingAckReceived = false;
    }
}


// =================================================================================
// FreeRTOS Tasks
// =================================================================================

void can_rx_task(void* pv) {
    twai_message_t msg;
    while (true) {
        if (twai_receive(&msg, pdMS_TO_TICKS(100)) == ESP_OK) {
            auto id = static_cast<Config::CanID>(msg.identifier);
            switch (id) {
                case Config::CanID::Heartbeat: {
                    if (msg.data_length_code > 0 && msg.data[0] == 0) {
                        lastHeartbeatTime = millis(); // Reset watchdog timer
                        sendHeartbeatAck();
                    }
                    break;
                }
                case Config::CanID::DoserStatusSolids: {
                    if (msg.data_length_code >= 2) processDoserStatus(static_cast<uint8_t>(Config::Motor::Solid1), read_u16_le(&msg.data[0]));
                    if (msg.data_length_code >= 4) processDoserStatus(static_cast<uint8_t>(Config::Motor::Solid2), read_u16_le(&msg.data[2]));
                    break;
                }
                case Config::CanID::DoserStatusLiquids: {
                    if (msg.data_length_code >= 2) processDoserStatus(static_cast<uint8_t>(Config::Motor::Liquid1), read_u16_le(&msg.data[0]));
                    if (msg.data_length_code >= 4) processDoserStatus(static_cast<uint8_t>(Config::Motor::Liquid2), read_u16_le(&msg.data[2]));
                    if (msg.data_length_code >= 6) processDoserStatus(static_cast<uint8_t>(Config::Motor::Liquid3), read_u16_le(&msg.data[4]));
                    if (msg.data_length_code >= 8) processDoserStatus(static_cast<uint8_t>(Config::Motor::Liquid4), read_u16_le(&msg.data[6]));
                    break;
                }
                case Config::CanID::MixerStatus: {
                    if (msg.data_length_code >= 2) {
                        auto new_status = static_cast<Config::MixerStatus>(read_u16_le(msg.data));
                        if (new_status != state.mixerStatus) {
                            state.mixerStatus = new_status;
                            //Reset ESP if home is clicked
                            if (int(new_status)==0) 
                            {
                              cleaningStateInstance.status = 0;
                              cleaningStateInstance.iter = 0;
                              cleaningStateInstance.visited_flag = false;
                              vTaskDelay(500); 
                              // ESP.restart();
                            }
                            Serial.printf("MIXER STATUS: New status received: %u\n", static_cast<uint16_t>(state.mixerStatus));
                            // Forward the exact same status to the MCM
                            // twai_message_t status_msg = {};
                            // status_msg.identifier = static_cast<uint32_t>(Config::CanID::MixerStatus);
                            // status_msg.data_length_code = 2;
                            // write_u16_le(status_msg.data, static_cast<uint16_t>(state.mixerStatus));
                            // twai_transmit(&status_msg, pdMS_TO_TICKS(10));
                        }
                    }
                    break;
                }
                case Config::CanID::StepMoveAck: {
                    if (msg.data_length_code >= 1) {
                        uint8_t motor_idx = msg.data[0];
                        if (motor_idx < static_cast<size_t>(Config::Motor::Count)) {
                            state.motors[motor_idx].primingAckReceived = true;
                            // Serial.printf("PRIMING ACK: Received for motor %d\n", motor_idx);
                            motor_idx=-1;
                        }
                    }
                    break;
                }
                case Config::CanID::StepMoveStatus: {
                    if (msg.data_length_code >= 2) {
                        uint8_t motor_idx = msg.data[0];
                        uint8_t status_code = msg.data[1];
                        Serial.printf("idx: 0x%lX |status: 0x%lX: ", motor_idx, status_code);
                        if (motor_idx < static_cast<size_t>(Config::Motor::Count) && status_code == 1) {
                            state.motors[motor_idx].priming_complete = true;
                        }
                    }
                    break;
                }
                case Config::CanID::MixerTankWeight: {
                    if (msg.data_length_code >= 4) state.current_tank_weight_kg = read_f32_le(msg.data);
                    break;
                }
                case Config::CanID::TareCmd: {
                    if (msg.data_length_code >= 1)
                    {
                      state.tare_status=msg.data[0];
                      if (msg.data[0] == 8) state.tare_complete = true;
                    }
                    break;
                }
                case Config::CanID::SprayVolHa: {
                    if (msg.data_length_code >= 4) {
                        state.session.sprayVolHa_L = read_f32_le(msg.data);
                        // Serial.printf("SESSION DATA: SprayVolHa received: %.2f L\n", state.session.sprayVolHa_L);
                    }
                    break;
                }
                case Config::CanID::TargetSpeed: {
                    if (msg.data_length_code >= 4) {
                        state.session.targetSpeed_mps = read_f32_le(msg.data)/3.6;
                        // Serial.printf("SESSION DATA: TargetSpeed received: %.2f m/s\n", state.session.targetSpeed_mps);
                    }
                    break;
                }
                case Config::CanID::RowSpacing: {
                    if (msg.data_length_code >= 4) {
                        state.session.rowSpacing_m = read_f32_le(msg.data);
                        // Serial.printf("SESSION DATA: RowSpacing received: %.2f m\n", state.session.rowSpacing_m);
                    }
                    break;
                }
                case Config::CanID::NumFaces: {
                    if (msg.data_length_code >= 2) {
                        state.session.numFaces = read_f32_le(msg.data);
                        // Serial.printf("SESSION DATA: NumFaces received: %u\n", state.session.numFaces);
                    }
                    break;
                }
                case Config::CanID::S1_RateUnit: {
                    if (msg.data_length_code >= 6) {
                        float rate_val = read_f32_le(&msg.data[0]);
                        uint16_t unit = read_u16_le(&msg.data[4]);
                        state.motors[static_cast<uint8_t>(Config::Motor::Solid1)].rate_kg_per_ha = to_kg_per_ha(rate_val, unit);
                        // Serial.printf("TREATMENT DATA: Motor S1, Rate: %.2f kg/ha (Raw: %.2f, Unit: %d)\n", state.motors[static_cast<uint8_t>(Config::Motor::Solid1)].rate_kg_per_ha, rate_val, unit);
                    }
                    break;
                }
                case Config::CanID::S1_Ccv: {
                    if (msg.data_length_code >= 4) {
                        state.motors[static_cast<uint8_t>(Config::Motor::Solid1)].ccv_deg_per_g = read_f32_le(msg.data);
                        // Serial.printf("TREATMENT DATA: Motor S1, CCV: %.2f deg/g\n", state.motors[static_cast<uint8_t>(Config::Motor::Solid1)].ccv_deg_per_g);
                    }
                    break;
                }
                case Config::CanID::S2_RateUnit: {
                    if (msg.data_length_code >= 6) {
                        float rate_val = read_f32_le(&msg.data[0]);
                        uint16_t unit = read_u16_le(&msg.data[4]);
                        state.motors[static_cast<uint8_t>(Config::Motor::Solid2)].rate_kg_per_ha = to_kg_per_ha(rate_val, unit);
                        // Serial.printf("TREATMENT DATA: Motor S2, Rate: %.2f kg/ha (Raw: %.2f, Unit: %d)\n", state.motors[static_cast<uint8_t>(Config::Motor::Solid2)].rate_kg_per_ha, rate_val, unit);
                    }
                    break;
                }
                 case Config::CanID::S2_Ccv: {
                    if (msg.data_length_code >= 4) {
                        state.motors[static_cast<uint8_t>(Config::Motor::Solid2)].ccv_deg_per_g = read_f32_le(msg.data);
                        // Serial.printf("TREATMENT DATA: Motor S2, CCV: %.2f deg/g\n", state.motors[static_cast<uint8_t>(Config::Motor::Solid2)].ccv_deg_per_g);
                    }
                    break;
                }
                case Config::CanID::L1_RateUnit: {
                    if (msg.data_length_code >= 6) {
                        float rate_val = read_f32_le(&msg.data[0]);
                        uint16_t unit = read_u16_le(&msg.data[4]);
                        state.motors[static_cast<uint8_t>(Config::Motor::Liquid1)].rate_kg_per_ha = to_kg_per_ha(rate_val, unit);
                        // Serial.printf("TREATMENT DATA: Motor L1, Rate: %.2f kg/ha (Raw: %.2f, Unit: %d)\n", state.motors[static_cast<uint8_t>(Config::Motor::Liquid1)].rate_kg_per_ha, rate_val, unit);
                    }
                    break;
                }
                 case Config::CanID::L1_Ccv: {
                    // Ignore CAN value, use hardcoded
                    break;
                }
                case Config::CanID::L2_RateUnit: {
                    if (msg.data_length_code >= 6) {
                        float rate_val = read_f32_le(&msg.data[0]);
                        uint16_t unit = read_u16_le(&msg.data[4]);
                        state.motors[static_cast<uint8_t>(Config::Motor::Liquid2)].rate_kg_per_ha = to_kg_per_ha(rate_val, unit);
                        // Serial.printf("TREATMENT DATA: Motor L2, Rate: %.2f kg/ha (Raw: %.2f, Unit: %d)\n", state.motors[static_cast<uint8_t>(Config::Motor::Liquid2)].rate_kg_per_ha, rate_val, unit);
                    }
                    break;
                }
                 case Config::CanID::L2_Ccv: {
                    // Ignore CAN value, use hardcoded
                    break;
                }
                case Config::CanID::L3_RateUnit: {
                    if (msg.data_length_code >= 6) {
                        float rate_val = read_f32_le(&msg.data[0]);
                        uint16_t unit = read_u16_le(&msg.data[4]);
                        state.motors[static_cast<uint8_t>(Config::Motor::Liquid3)].rate_kg_per_ha = to_kg_per_ha(rate_val, unit);
                        // Serial.printf("TREATMENT DATA: Motor L3, Rate: %.2f kg/ha (Raw: %.2f, Unit: %d)\n", state.motors[static_cast<uint8_t>(Config::Motor::Liquid3)].rate_kg_per_ha, rate_val, unit);
                    }
                    break;
                }
                 case Config::CanID::L3_Ccv: {
                    // Ignore CAN value, use hardcoded
                    break;
                }
                case Config::CanID::L4_RateUnit: {
                    if (msg.data_length_code >= 6) {
                        float rate_val = read_f32_le(&msg.data[0]);
                        uint16_t unit = read_u16_le(&msg.data[4]);
                        state.motors[static_cast<uint8_t>(Config::Motor::Liquid4)].rate_kg_per_ha = to_kg_per_ha(rate_val, unit);
                        // Serial.printf("TREATMENT DATA: Motor L4, Rate: %.2f kg/ha (Raw: %.2f, Unit: %d)\n", state.motors[static_cast<uint8_t>(Config::Motor::Liquid4)].rate_kg_per_ha, rate_val, unit);
                    }
                    break;
                }
                 case Config::CanID::L4_Ccv: {
                    // Ignore CAN value, use hardcoded
                    break;
                }
                 case Config::CanID::NozzlePressureSetpoint: {
                    if (msg.data_length_code >= 4) { 
                      state.nozzleController.setpoint = read_f32_le(msg.data); 
                    }
                    break;
                }
                 case Config::CanID::WaterInletValve: {
                    if (msg.data_length_code >= 2) { 
                      int temp = read_u16_le(msg.data);
                      if (temp!=state.waterinletvalve)
                      {
                        state.waterinletvalve=temp;
                        gpio_set_level(Config::RELAY_WATER_INLET_CLOSE, !state.waterinletvalve);
                        gpio_set_level(Config::RELAY_WATER_INLET_OPEN, state.waterinletvalve);
                      }
                    }
                    break;
                }
                 case Config::CanID::SprayOutletValve: {
                    if (msg.data_length_code >= 2) { 
                      int temp = read_u16_le(msg.data);
                      if (temp!=state.sprayoutletvalve)
                      {
                        state.sprayoutletvalve=temp;
                        SprayOutletValveController(state.sprayoutletvalve);
                      }
                    }
                    break;
                }
                 case Config::CanID::PropSystemPressure: {
                    if (msg.data_length_code >= 2) { 
                      int temp = read_u16_le(msg.data);
                      if (temp!=state.propsystempressure)
                      {
                        Serial.printf("TEMP_SYSTEM: %d", temp);
                        state.propsystempressure=temp;
                        gpio_set_level(state.systemController.relayClosePin, !state.propsystempressure);
                        gpio_set_level(state.systemController.relayOpenPin, state.propsystempressure);
                      }
                    }
                    break;
                }
                 case Config::CanID::PropNozzlePressure: {
                    if (msg.data_length_code >= 2) { 
                      int temp = read_u16_le(msg.data);
                      if(temp!=state.propnozzlepressure)
                      {
                        Serial.printf("TEMP_NOZZLE: %d", temp);
                        state.propnozzlepressure=temp;
                        gpio_set_level(state.nozzleController.relayClosePin, !state.propnozzlepressure);
                        gpio_set_level(state.nozzleController.relayOpenPin, state.propnozzlepressure);
                      }
                    }
                    break;
                }
                default: break;
            }
        }
    }
}

void main_control_task(void* pv) {

    twai_message_t cleaning_status = {}; //Cleaning status message init
    cleaning_status.identifier = static_cast<uint32_t>(Config::CanID::CleaningCycles);
    cleaning_status.data_length_code = 8;
    write_u16_le(&cleaning_status.data[0], 100);
    twai_transmit(&cleaning_status, pdMS_TO_TICKS(10));

    while (true) {
        // Heartbeat Watchdog Check
        if (millis() - lastHeartbeatTime > Config::HEARTBEAT_TIMEOUT_MS) {
            if (state.mixerStatus != Config::MixerStatus::Stopped) {
                Serial.println("HEARTBEAT TIMEOUT: Forcing system to STOPPED state.");
                state.mixerStatus = Config::MixerStatus::Stopped;
            }
            lastHeartbeatTime = millis(); // Reset timer to prevent continuous triggering
        }
        
        // Serial.printf("CONTROL LOOP: Current Mixer Status: %u\n", static_cast<uint16_t>(state.mixerStatus));
        switch (state.mixerStatus) {
            case Config::MixerStatus::Stopped:
            {
                CanMixer::Config::waitingForPressure = false;

                // [CRITICAL FIX]: Disable Nozzle Controller to HOLD position
                state.nozzleController.enabled = false; 
                
                // [CRITICAL FIX]: Ensure System Controller is ENABLED to maintain setpoint
                state.systemController.enabled = true; 

                // Stop water intake
                gpio_set_level(Config::RELAY_WATER_INLET_CLOSE, 1);
                gpio_set_level(Config::RELAY_WATER_INLET_OPEN, 0);
                state.tare_complete = false;  // Clear the success flag
                state.tare_status = 0;        // Clear the status code

                if (state.fill_state != FillState::WAITING_FOR_TARE) {
                    state.fill_state = FillState::WAITING_FOR_TARE;
                }

                // Close Master Valve (this function also sets System Setpoint to 15.0)
                SprayOutletValveController(false); 

                state.priming_initiated = false;
                resetPrimingAcks();
                sendStopAllMotors();
                break;
            }
            case Config::MixerStatus::Priming:
                if (!state.priming_initiated) {
                    resetPrimingAcks();
                    for (uint8_t i = 0; i < static_cast<size_t>(Config::Motor::Count); ++i) {
                        if (state.motors[i].loaded && !state.motors[i].paused) {
                            sendHopperStatus(i, 1); // 1 = In Progress
                        }
                    }
                    state.priming_initiated = true;
                }
                // Serial.printf("%d %d\n", state.fill_state, FillState::MAINTAINING);
                if (state.fill_state!=FillState::MAINTAINING) break;
                Serial.println("PRIMING INIT" );
 
                executePrimingSequence(); // Continuously stream commands until acknowledged

                for (uint8_t i = 0; i < static_cast<size_t>(Config::Motor::Count); ++i) {
                    auto& motor = state.motors[i];
                    if (motor.loaded && !motor.paused && motor.priming_complete && !motor.priming_status_sent) {
                        sendHopperStatus(i, 100); // 100 = Priming Success
                        motor.priming_status_sent = true;
                    }
                }
                break;

            case Config::MixerStatus::Running:
                if (state.priming_initiated == false) {
                // Ensure System is active and Nozzle is waiting
                    if (!CanMixer::Config::waitingForPressure && !state.nozzleController.enabled) {
                        // Serial.println("STATUS: RUNNING. Starting Stabilization...");
                        CanMixer::Config::waitingForPressure = true;
                        state.nozzleController.enabled = false;
                    }
                }
                state.priming_initiated = false;
                resetPrimingAcks();
                computeAndSendRPMs(); // Send RUN commands
                break;

            case Config::MixerStatus::Rundown:
                state.priming_initiated = false;
                resetPrimingAcks();
                sendStopAllMotors(); // Send STOP commands
                //settting fill state to idle 
                state.fill_state=FillState::IDLE;
                //Stop water intake
                gpio_set_level(Config::RELAY_WATER_INLET_CLOSE, 1);
                gpio_set_level(Config::RELAY_WATER_INLET_OPEN, 0);
                break;

            case Config::MixerStatus::StartResumeCleaning:
            {
                state.priming_initiated = false;
                resetPrimingAcks();
                // Serial.println(cleaningStateInstance.status);
                Serial.println(state.current_tank_weight_kg);
                sendCleaningRPMs();


                if(cleaningStateInstance.status<=410)
                {
                  twai_transmit(&cleaning_status, pdMS_TO_TICKS(10));
                  if (cleaningStateInstance.status%100==0 || cleaningStateInstance.status%100==60) //Purge cycle
                    {

                      if (cleaningStateInstance.visited_flag==false)
                      {
                        state.fill_state=FillState::IDLE;//Idle
                        cleaningStateInstance.status=(cleaningStateInstance.iter+1)*100;//Purge in progress 
                        write_u16_le(&cleaning_status.data[0], cleaningStateInstance.status);
                        //Stop water intake
                        gpio_set_level(Config::RELAY_WATER_INLET_CLOSE, 1);
                        gpio_set_level(Config::RELAY_WATER_INLET_OPEN, 0);
                        //Open Nozzle valves to let the water out
                        SprayOutletValveController(true);
                        cleaningStateInstance.visited_flag=true; //restrict the valve to be actuated over time   
                      }
                      if (state.current_tank_weight_kg<=1) //Tank is empty
                      {
                        cleaningStateInstance.status=(cleaningStateInstance.iter+1)*100+10; //Water is empty
                        write_u16_le(&cleaning_status.data[0], cleaningStateInstance.status);
                        cleaningStateInstance.visited_flag=false;
                      }
                    }
                  else if (cleaningStateInstance.status%100==10 || cleaningStateInstance.status%100==50) //Refill Cycle
                    {
                      if (!cleaningStateInstance.visited_flag && cleaningStateInstance.status<410)
                      {
                        state.fill_state=FillState::FILLING;//Idle
                        cleaningStateInstance.status=(cleaningStateInstance.iter+1)*100+50;//Refill in progress
                        write_u16_le(&cleaning_status.data[0], cleaningStateInstance.status);
                        //shut Nozzle valves to not let the water out
                        SprayOutletValveController(false);
                        cleaningStateInstance.visited_flag=true; //restrict the valve to be actuated over time                   
                      }
                      if (state.current_tank_weight_kg>=15) //Tank is Full
                      {
                        cleaningStateInstance.status=(cleaningStateInstance.iter+1)*100+60;
                        write_u16_le(&cleaning_status.data[0], cleaningStateInstance.status);
                        cleaningStateInstance.iter+=1;
                        cleaningStateInstance.visited_flag=false;
                      }
                    }
                }
                else state.fill_state=FillState::IDLE;//Idle
                break;
            }
            case Config::MixerStatus::StopCleaning://Stop water inlet and nozzle exit valves
            {
              cleaningStateInstance.visited_flag=false; //To enable valves to open 
              sendStopAllMotors();
              //shut Nozzle valves to not let the water out
              SprayOutletValveController(false);
              //Stop water intake
              gpio_set_level(Config::RELAY_WATER_INLET_CLOSE, 1);
              gpio_set_level(Config::RELAY_WATER_INLET_OPEN, 0);
              break;
            }

            case Config::MixerStatus::ControlPanelActive:
                break;

            case Config::MixerStatus::CleaningCheckList:
                break;

            default: 
                state.priming_initiated = false;
                resetPrimingAcks();
                sendStopAllMotors(); // Send STOP commands
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(Config::CONTROL_LOOP_INTERVAL_MS));
    }
}

void sensor_task(void* pv) {
      int raw_nozzle = 0;
      int raw_system = 0;

      while (true) {
          adc2_get_raw(Config::ADC_NOZZLE_PRESSURE, ADC_WIDTH_BIT_12, &raw_nozzle);
          adc2_get_raw(Config::ADC_SYSTEM_PRESSURE, ADC_WIDTH_BIT_12, &raw_system);
          
          float rawNozzlePressure = calculate_pressure(raw_nozzle); 
          state.nozzleController.input = (CanMixer::Config::filterAlpha * rawNozzlePressure) + (1.0 - CanMixer::Config::filterAlpha) * state.nozzleController.input;
          // Serial.print("nozzlePressure:");
          // Serial.println(state.nozzleController.input);
          nozzleUiFilter.add(state.nozzleController.input);
          nozzle_pressure_for_ui = nozzleUiFilter.getAverage();

          float rawSystemPressure = calculate_pressure(raw_system);
          state.systemController.input = (CanMixer::Config::filterAlpha * rawSystemPressure) + (1.0 - CanMixer::Config::filterAlpha) * state.systemController.input;
          // Serial.print("systemPressure:");
          // Serial.println(state.systemController.input);
          twai_message_t pres = {};
          pres.identifier = static_cast<uint32_t>(Config::CanID::PressureSensorData);
          pres.data_length_code = 8;
          write_f32_le(&pres.data[0], nozzle_pressure_for_ui);
          write_f32_le(&pres.data[4], state.systemController.input);
          twai_transmit(&pres, pdMS_TO_TICKS(10));

          vTaskDelay(pdMS_TO_TICKS(Config::SENSOR_READ_INTERVAL_MS));
      }
  }

  void tank_fill_task(void* pv) {
      while(true) {
          float current_level_L = float(state.current_tank_weight_kg);
          // Serial.println(int(state.fill_state));
          switch(state.fill_state) {
              case FillState::IDLE:
                  // Ensure valve is stopped and relays are off
                  // gpio_set_level(Config::RELAY_WATER_INLET_OPEN, 0);
                  // gpio_set_level(Config::RELAY_WATER_INLET_CLOSE, 0);
                  break;
              case FillState::WAITING_FOR_TARE:
                  if (state.tare_complete) {
                      state.fill_state = FillState::FILLING;
                  }
                  break;
              case FillState::FILLING:
                  Serial.printf("TANK WEIGHT: %f\n", current_level_L);    
                  if (int(state.mixerStatus)==0) continue;
                  if (current_level_L < Config::TANK_FILL_TARGET_L) {
                      // Open the water inlet valve
                      // Serial.printf("FILLING TANK\n");  
                      gpio_set_level(Config::RELAY_WATER_INLET_CLOSE, 0);
                      gpio_set_level(Config::RELAY_WATER_INLET_OPEN, 1);
                  } else {
                      // Target reached, close the water inlet valve
                      gpio_set_level(Config::RELAY_WATER_INLET_OPEN, 0);
                      gpio_set_level(Config::RELAY_WATER_INLET_CLOSE, 1);
                      state.fill_state = FillState::MAINTAINING;
                  }
                  break;
              case FillState::MAINTAINING:
                  if (current_level_L < (Config::TANK_FILL_TARGET_L - Config::TANK_MAINTAIN_HYSTERESIS_L)) {
                      // Open the valve
                      gpio_set_level(Config::RELAY_WATER_INLET_CLOSE, 0);
                      gpio_set_level(Config::RELAY_WATER_INLET_OPEN, 1);
                  } else if (current_level_L >= Config::TANK_FILL_TARGET_L) {
                      // Close the valve
                      gpio_set_level(Config::RELAY_WATER_INLET_OPEN, 0);
                      gpio_set_level(Config::RELAY_WATER_INLET_CLOSE, 1);
                  } else {
                      // Within hysteresis band, de-energize both relays
                      gpio_set_level(Config::RELAY_WATER_INLET_OPEN, 0);
                      gpio_set_level(Config::RELAY_WATER_INLET_CLOSE, 0);
                  }
                  break;
          }
          vTaskDelay(pdMS_TO_TICKS(Config::TANK_FILL_INTERVAL_MS));
      }
  }

  // UPDATED: Logic for the spray outlet valve now uses separate open/close pins
  void spray_valve_task(void* pv) {
      while (true) {
        if (int(state.mixerStatus)==1000) continue;
          // The spray outlet should be open only when the mixer is in a spraying state
          bool should_be_spraying = (state.mixerStatus == Config::MixerStatus::Running || 
                                     state.mixerStatus == Config::MixerStatus::Rundown ||
                                     state.mixerStatus == Config::MixerStatus::CleaningPurge ||
                                     (cleaningStateInstance.status%100 == 0 && cleaningStateInstance.status>0) ||
                                     cleaningStateInstance.status%100 == 60);
          Serial.println(should_be_spraying);
          // Serial.println(int(cleaningStateInstance.status));
          if (should_be_spraying) {
              // Open the spray valve
              SprayOutletValveController(true);
          } else {
              // Close the spray valve
              SprayOutletValveController(false);
          }
          
          vTaskDelay(pdMS_TO_TICKS(Config::SPRAY_VALVE_INTERVAL_MS));
      }
  }

} // namespace CanMixer

// =================================================================================
// Setup & Main (GLOBAL SCOPE)
// =================================================================================

void setup() {
    using namespace CanMixer;
    Serial.begin(115200);

    lastHeartbeatTime = millis(); // Initialize timer on startup

    // --- Hardcode Liquid Doser CCV values ---
    state.motors[static_cast<uint8_t>(Config::Motor::Liquid1)].ccv_deg_per_g = 3300.0f;
    state.motors[static_cast<uint8_t>(Config::Motor::Liquid2)].ccv_deg_per_g = 3300.0f;
    state.motors[static_cast<uint8_t>(Config::Motor::Liquid3)].ccv_deg_per_g = 3300.0f;
    state.motors[static_cast<uint8_t>(Config::Motor::Liquid4)].ccv_deg_per_g = 3300.0f;
    Serial.println("SETUP: Hardcoded CCV of 3300.0 applied to all liquid dosers.");

    // GPIO & ADC setup
    gpio_set_direction(Config::RELAY_NOZZLE_OPEN, GPIO_MODE_OUTPUT);
    gpio_set_direction(Config::RELAY_NOZZLE_CLOSE, GPIO_MODE_OUTPUT);
    gpio_set_direction(Config::RELAY_SYSTEM_OPEN, GPIO_MODE_OUTPUT);
    gpio_set_direction(Config::RELAY_SYSTEM_CLOSE, GPIO_MODE_OUTPUT);
    gpio_set_direction(Config::RELAY_WATER_INLET_OPEN, GPIO_MODE_OUTPUT); //tank inlet 
    gpio_set_direction(Config::RELAY_WATER_INLET_CLOSE, GPIO_MODE_OUTPUT);
    gpio_set_direction(Config::RELAY_SPRAY_OUTLET_OPEN, GPIO_MODE_OUTPUT);  //spray through nozzles
    gpio_set_direction(Config::RELAY_SPRAY_OUTLET_CLOSE, GPIO_MODE_OUTPUT); 

    // Controller setup
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

    // CAN setup
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(Config::CAN_TX_PIN, Config::CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK || twai_start() != ESP_OK) {
        Serial.println("CAN init failed");
        while (true) vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // Task creation
    xTaskCreatePinnedToCore(can_rx_task, "CAN_RX", 4096, nullptr, 10, nullptr, 0);
    xTaskCreatePinnedToCore(sensor_task, "Sensor", 4096, nullptr, 6, nullptr, 1);
    xTaskCreatePinnedToCore(valve_control_task, "ValveControl", 2048, nullptr, 7, nullptr, 1);
    xTaskCreatePinnedToCore(main_control_task, "MainControl", 4096, nullptr, 5, nullptr, 1);
    xTaskCreatePinnedToCore(tank_fill_task, "TankFill", 4096, nullptr, 4, nullptr, 1);
    xTaskCreatePinnedToCore(spray_valve_task, "SprayValve", 2048, nullptr, 4, nullptr, 1);
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}
