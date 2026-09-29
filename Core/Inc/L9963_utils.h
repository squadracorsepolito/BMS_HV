#include "L9963E.h"
#include "main.h"
#include "stm32_if.h"
#include "L9963E_drv.h"

#ifndef L9963_UTILS_H
#define L9963_UTILS_H

#define T_WAKE_UP         ((uint32_t)2)
#define N_SLAVES          ((uint8_t)1)   // TEST: 1 slave fisico — ripristinare a 12 per produzione
#define N_CELLS_PER_SLAVE ((uint8_t)11)
/* GPIO7 è collegato a GND da schematico (pin WAKEUP non usato come NTC)
 * Solo 6 GPIO sono NTC: GPIO3,4,5,6,8,9 */
#define N_GPIOS_PER_SLAVE ((uint8_t)6)  // 6 NTC: GPIO3,4,5,6,8,9 — GPIO7 è a GND
#define ENABLED_CELLS                                                                                         \
    (L9963E_CELL1 | L9963E_CELL2 | L9963E_CELL3 | L9963E_CELL4 | L9963E_CELL5 | L9963E_CELL6 | L9963E_CELL7 | \
     L9963E_CELL8 | L9963E_CELL12 | L9963E_CELL13 | L9963E_CELL14)

/* GPIO7 escluso: è a GND sul test board, non connesso ad alcun NTC */
#define ENABLED_GPIOS \
    (L9963E_GPIO3 | L9963E_GPIO4 | L9963E_GPIO5 | L9963E_GPIO6 | L9963E_GPIO8 | L9963E_GPIO9)

typedef enum {
    L9963_UTILS_OK = 0,
    L9963E_UTILS_ERROR,
} L9963_Utils_StatusTypeDef;

void L9963E_utils_init(void);
void L9963E_utils_read_cells(uint8_t module_id, uint8_t read_gpio);
void L9963E_utils_read_all_cells(uint8_t read_gpio);
void L9963E_utils_get_module_mv(uint8_t module_id);
float L9963E_utils_get_cell_mv(uint8_t module_id, uint8_t index);
void L9963E_utils_get_batt_mv(float *v_tot, float *v_sum, uint8_t module);
L9963_Utils_StatusTypeDef L9963E_utils_balance_cells(void);
void L9963E_utils_get_total_batt_mv(float *v_tot, float *v_sum);
/* Debug da terminale (USART3, 115200 8N1) */
void L9963E_utils_debug_print(void);
void L9963E_utils_diag_loop(uint32_t max_cycles); /* 0 = infinito */
extern volatile uint8_t  l9963_addressing_ok;   /* 1 = wakeup + addressing riusciti */
extern volatile uint32_t l9963_comm_errors;     /* letture fallite (timeout/CRC) dall'avvio */
extern volatile uint32_t l9963_read_cycles;     /* cicli di lettura completati */
extern volatile uint8_t  l9963_gpio_valid;      /* 1 = almeno una lettura NTC fatta */
#endif  // L9963T_UTILS_H