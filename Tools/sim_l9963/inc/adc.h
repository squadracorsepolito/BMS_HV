#ifndef SIM_ADC_H
#define SIM_ADC_H
#include "main.h"
typedef struct { int dummy; } ADC_HandleTypeDef;
extern ADC_HandleTypeDef hadc1;
static inline int HAL_ADC_Start_DMA(ADC_HandleTypeDef *h, uint32_t *d, uint32_t n) { (void)h; (void)d; (void)n; return 0; }
#endif
