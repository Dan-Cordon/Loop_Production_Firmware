#include "helpers.h"
#include "globals.h"

namespace CanMixer {

    float read_f32_le(const uint8_t* b) { float f; memcpy(&f, b, 4); return f; }
    uint16_t read_u16_le(const uint8_t* b) { return (uint16_t)b[0] | ((uint16_t)b[1] << 8); }
    void write_f32_le(uint8_t* b, float v) { memcpy(b, &v, 4); }
    void write_u16_le(uint8_t* b, uint16_t v) { b[0] = v & 0xFF; b[1] = (v >> 8) & 0xFF; }

    float calculate_pressure(int raw_value) {
        using namespace Config;
        if (raw_value <= ADC_MIN_RAW) return 0.0f;
        if (raw_value >= ADC_MAX_RAW) return PRESSURE_MAX_BAR;
        float scaled = (float)(raw_value - ADC_MIN_RAW) / (float)(ADC_MAX_RAW - ADC_MIN_RAW);
        return scaled * PRESSURE_MAX_BAR;
    }

    float to_kg_per_ha(float value, uint16_t unit) {
        switch (unit) {
            case 0: return value;
            case 1: return value / 1000.0f;
            case 2: return value * 28.3495f / 1000.0f;
            case 3: return value * Config::DENSITY_G_PER_ML;
            case 4: return value * Config::DENSITY_G_PER_ML / 1000.0f;
            default: return 0.0f;
        }
    }
    
    float computeRPM(const TreatmentInput& t, const SessionInputs& s) {
        if (!t.loaded || t.paused || t.rate_kg_per_ha <= 0.0f || t.ccv_deg_per_g <= 0.0f) return 0.0f;
        if (s.targetSpeed_mps <= 0 || s.rowSpacing_m <= 0 || s.numFaces <= 0) return 0.0f;
        
        const float areaRate = s.targetSpeed_mps * s.rowSpacing_m * (float)s.numFaces;
        const float chemRate_gps = t.rate_kg_per_ha * (areaRate / 10.0f);
        const float deg_per_s = chemRate_gps * (t.ccv_deg_per_g / 100.0f);
        float rpm = deg_per_s / 6.0f;
        
        if (!isfinite(rpm) || rpm < 0) rpm = 0;
        if (rpm > Config::RPM_MAX) rpm = Config::RPM_MAX;
        return rpm;
    }

    void SprayOutletValveController(bool flag) {
        gpio_set_level(Config::RELAY_SPRAY_OUTLET_CLOSE, !flag);
        gpio_set_level(Config::RELAY_SPRAY_OUTLET_OPEN, flag);
        SprayOutValveStatus = flag;

        // FIXED LOGIC: If valve closed, setpoint 0. If open, setpoint 15.
        if (flag == false) {
             state.systemController.setpoint = 0.0f; 
        } else {
             state.systemController.setpoint = 15.0f;
        }
    }

    void update_valve_pwm(ValveController& controller) {
        if (int(state.mixerStatus) == 1000) return; // Control panel active
        unsigned long current_time = millis();

        // Part 1: Manage Active Relay
        if (controller.active_relay_pin != GPIO_NUM_NC) {
            if (current_time - controller.window_start_time >= controller.activation_duration_ms) {
                gpio_set_level(controller.active_relay_pin, 0);
                controller.active_relay_pin = GPIO_NUM_NC;
            }
        }

        // Part 2: Start New Window
        if (current_time - controller.window_start_time < Config::PWM_ACTIVATION_WINDOW_MS) return;

        controller.window_start_time = current_time;
        gpio_set_level(controller.relayOpenPin, 0);
        gpio_set_level(controller.relayClosePin, 0);
        controller.active_relay_pin = GPIO_NUM_NC;
        controller.activation_duration_ms = 0;

        if (!controller.enabled) return;

        // Force dump if setpoint near zero
        if (controller.setpoint < 0.1) {
            controller.activation_duration_ms = Config::PWM_ACTIVATION_WINDOW_MS;
            if (!controller.direction) { // Inverse (System)
                controller.active_relay_pin = controller.relayOpenPin;
            } else { // Direct (Nozzle)
                controller.active_relay_pin = controller.relayClosePin;
            }
            gpio_set_level(controller.active_relay_pin, 1);
            return;
        }

        // Part 3: P-Controller
        float error = controller.setpoint - controller.input;
        if (abs(error) < Config::PRESSURE_HYSTERESIS_BAR) return;

        float output_percent = controller.Kp * error;
        output_percent = constrain(output_percent, -100.0, 100.0);
        controller.activation_duration_ms = (Config::PWM_ACTIVATION_WINDOW_MS * abs(output_percent)) / 100.0;

        if (controller.activation_duration_ms < 20) {
            controller.activation_duration_ms = 0;
            return;
        }

        bool needs_pressure_increase = (output_percent > 0);
        if (controller.direction) { // Nozzle
            controller.active_relay_pin = needs_pressure_increase ? controller.relayOpenPin : controller.relayClosePin;
        } else { // System
            controller.active_relay_pin = needs_pressure_increase ? controller.relayClosePin : controller.relayOpenPin;
        }
        gpio_set_level(controller.active_relay_pin, 1);
    }
    
    void checkStabilization() {
        if (!waitingForPressure) return;
        
        float currentP = state.systemController.input;
        float targetP = state.systemController.setpoint;
        
        if (targetP < 0.5f) return;
        
        float threshold = targetP * 0.10f;
        float error = abs(currentP - targetP);
        
        if (error < threshold) {
            waitingForPressure = false;
            state.nozzleController.enabled = true; // Wake up nozzle
            Serial.println("SYSTEM STABILIZED");
        } else {
            state.nozzleController.enabled = false;
        }
    }

} // namespace CanMixer