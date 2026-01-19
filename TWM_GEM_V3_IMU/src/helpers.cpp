#include "helpers.h"

namespace TWM {

    // Use Horner's method for efficient polynomial evaluation
    float calculateCorrection(float imuValue, const float coeffs[Config::NUM_LOADCELLS][4], int cellIndex) {
        float a3 = coeffs[cellIndex][0];
        float a2 = coeffs[cellIndex][1];
        float a1 = coeffs[cellIndex][2];
        float a0 = coeffs[cellIndex][3];
        
        return (((a3 * imuValue + a2) * imuValue + a1) * imuValue + a0);
    }

} // namespace TWM