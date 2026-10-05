#ifndef BMS_TEMPERATURE_H
#define BMS_TEMPERATURE_H
#include <stdint.h>

#define BMS_TEMPERATURE_MIN_C 25.0f
#define BMS_TEMPERATURE_MAX_C 45.0f
/* Set only after installed thermistor and conversion are validated. */
#ifndef BMS_NTC_CALIBRATION_VERIFIED
#define BMS_NTC_CALIBRATION_VERIFIED 0
#endif
uint8_t BMS_temperature_fault(float temperature_c);
#endif
