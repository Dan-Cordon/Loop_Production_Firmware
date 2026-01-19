#include "globals.h"

namespace TWM {

    HX711 loadCells[Config::NUM_LOADCELLS];
    Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

    SemaphoreHandle_t hx711Mutex;
    
    volatile float filteredWeight_grams = 0.0f;
    volatile float rawWeight_grams = 0.0f;
    
    Orientation lastOrientation = {0, 0, 0};

    // Coefficients for Y-axis
    const float coeffs_Y[4][4] = {
      { 1.103E-01, 1.136E-15, -2.586E+02, 0 },  // LC1
      { 1.327E-01, 5E-16,     -2.99E+02,  0 },  // LC2
      { -3.067E-01, -1.9E-15,  2.073E+02, 0 },  // LC3
      { -1.369E-02, -1.424E-16, 7.83E+01, 0 }   // LC4
    };

    // Coefficients for Z-axis
    const float coeffs_Z[4][4] = {
      { -1.113E-01, -1.74E-16, 5.047E+01, 0 },  // LC1
      { 1.312E-01, -1.0E-15,  -1.396E+02, 0 },  // LC2
      { -2.535E-01, -6.4E-17, -8.795E+00, 0 },  // LC3
      { 1.138E-01, 4E-17,     2.27E+01,  0 }    // LC4
    };

} // namespace TWM