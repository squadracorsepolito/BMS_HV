#include "data_reading_timebase.h"
#include "L9963_utils.h"

TIMEBASE_HandleTypeDef data_reading_timebase_handle;
extern L9963E_HandleTypeDef hl9963e;

void data_reading_timebase_init(void) {
    uint8_t interval;
  
    TIMEBASE_init(&data_reading_timebase_handle, &htim6, 1000);
  
    TIMEBASE_add_interval(&data_reading_timebase_handle, 10000, &interval);
    TIMEBASE_register_callback(&data_reading_timebase_handle, interval, data_reading_l9963e_cb);
  }

L9963_Utils_StatusTypeDef data_reading_l9963e_cb(L9963E_HandleTypeDef* hl9963e){
    uint8_t is_ntc_measure_required = ntc_is_measure_ext_time();
    L9963E_utils_read_all_cells(is_ntc_measure_required);
    uint16_t const *v_cells;

    v_cells = L9963E_utils_get_all_cells();

    for (uint8_t i = 0; i < N_SLAVES; i++){
        for (uint8_t j = 0; j < N_CELLS_PER_SLAVE; j++){
            if (v_cells[i][j] > MAX_VOLTAGE * 89e-3f){
                return L9963E_UTILS_ERROR;
            }
        }
    }

}