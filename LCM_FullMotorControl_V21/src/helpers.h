#pragma once
#include <Arduino.h>
#include <string.h>
#include <math.h>
#include "types.h"
#include "config.h"

namespace CanMixer {

    // Template class for moving average
    template<typename T, size_t N>
    class SimpleMovingAverage {
    private:
        T readings[N] = {0};
        size_t index = 0;
        T sum = 0;
        size_t count = 0;
    public:
        void add(T new_reading) {
            if (count < N) count++; // Correct logic for initial fill
            else sum -= readings[index];
            
            readings[index] = new_reading;
            sum += new_reading;
            index = (index + 1) % N;
        }
        T getAverage() const {
            if (count == 0) return 0;
            return sum / count;
        }
    };

    // Helper functions
    float read_f32_le(const uint8_t* b);
    uint16_t read_u16_le(const uint8_t* b);
    void write_f32_le(uint8_t* b, float v);
    void write_u16_le(uint8_t* b, uint16_t v);
    
    float calculate_pressure(int raw_value);
    float to_kg_per_ha(float value, uint16_t unit);
    float computeRPM(const TreatmentInput& t, const SessionInputs& session);
    
    void update_valve_pwm(ValveController& controller);
    void SprayOutletValveController(bool flag);
    void checkStabilization();
    
} // namespace CanMixer