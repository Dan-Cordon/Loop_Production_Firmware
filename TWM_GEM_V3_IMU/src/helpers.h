#pragma once
#include "config.h"

namespace TWM {

    float calculateCorrection(float imuValue, const float coeffs[Config::NUM_LOADCELLS][4], int cellIndex);

} // namespace TWM