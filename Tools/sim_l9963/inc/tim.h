#ifndef SIM_TIM_H
#define SIM_TIM_H
#include "main.h"
typedef struct { int dummy; } TIM_HandleTypeDef;
extern TIM_HandleTypeDef htim6;
static inline int HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *h) { (void)h; return 0; }
#endif
