#include "bms_temperature.h"
#include <math.h>

uint8_t BMS_temperature_fault(float temperature_c) {
    return !isfinite(temperature_c) || temperature_c < BMS_TEMPERATURE_MIN_C ||
           temperature_c > BMS_TEMPERATURE_MAX_C;
}
