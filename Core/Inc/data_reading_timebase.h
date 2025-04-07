#ifndef DATA_READING_TIMEBASE_H
#define DATA_READING_TIMEBASE_H

#include "timebase.h"
#include "tim.h"

#define OVERVOLTAGE_TRESHOLD 4.2f

// CHANGE OVERTEMPERATURE LATER
#define OVERTEMPERATURE_TRESHOLD 3.0f

void data_reading_timebase_init(void);
void data_reading_timebase_routine(void);
void data_reading_timebase_timerElapsed_irq(TIM_HandleTypeDef *htim);
STMLIBS_StatusTypeDef data_reading_l9963e_cb();


#endif // DATA_READING_TIMEBASE_H


