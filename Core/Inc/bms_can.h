#ifndef BMS_CAN_H
#define BMS_CAN_H
#include "bms_can_protocol.h"

/* CAN1 command reception only. Telemetry transmission is a separate task. */
int BMS_CAN_init(void);
void BMS_CAN_routine(void);
#endif
