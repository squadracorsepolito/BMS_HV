#ifndef SIM_TIMEBASE_H
#define SIM_TIMEBASE_H
#include "main.h"
#include "stmlibs_status.h"
#include "tim.h"
typedef struct { int dummy; } TIMEBASE_HandleTypeDef;
typedef STMLIBS_StatusTypeDef (*TIMEBASE_cb)(void);
static inline int TIMEBASE_init(TIMEBASE_HandleTypeDef *h, TIM_HandleTypeDef *t, uint32_t us) { (void)h; (void)t; (void)us; return 0; }
static inline int TIMEBASE_add_interval(TIMEBASE_HandleTypeDef *h, uint32_t us, uint8_t *id) { (void)h; (void)us; *id = 0; return 0; }
static inline int TIMEBASE_register_callback(TIMEBASE_HandleTypeDef *h, uint8_t id, STMLIBS_StatusTypeDef (*cb)(void)) { (void)h; (void)id; (void)cb; return 0; }
static inline int TIMEBASE_routine(TIMEBASE_HandleTypeDef *h) { (void)h; return 0; }
#endif
