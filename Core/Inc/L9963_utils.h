#include "main.h"
#include "stm32_if.h"
#include "L9963E.h"

#ifndef L9963_UTILS_H
#define L9963_UTILS_H

#define T_WAKE_UP ((uint32_t)2)
#define N_SLAVES ((uint8_t)12)
#define N_CELLS_PER_SLAVE ((uint8_t)11)



void L9963E_utils_init(void);
void L9963E_utils_read_cells(uint8_t read_modules, uint8_t read_gpio);
void L9963E_utils_get_module_mv(uint8_t module);
float L9963E_utils_get_cell_mv(uint8_t module, uint8_t index);
void L9963E_utils_get_batt_mv(float *v_tot, float *v_sum);


#endif  // L9963T_UTILS_H