#ifndef DATA_READING_TIMEBASE_H
#define DATA_READING_TIMEBASE_H

#include "timebase.h"
#include "tim.h"

/* Provisional bench supervision limit; validate worst-case chain latency. */
#define MEASUREMENT_MAX_AGE_MS 500U
#define OVERVOLTAGE_THRESHOLD_MV 4200.0f

/* Legacy voltage limit is not a validated temperature protection threshold.
 * Replace with a sensor-specific Celsius limit after the NTC wiring is known. */
#define LEGACY_GPIO_THRESHOLD_V 3.0f

STMLIBS_StatusTypeDef data_reading_timebase_init(void);
void data_reading_timebase_routine(void);
uint8_t data_reading_waiting_for_first_sample(void);

STMLIBS_StatusTypeDef data_reading_l9963e_cb();


#endif // DATA_READING_TIMEBASE_H


