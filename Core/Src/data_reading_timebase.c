#include "data_reading_timebase.h"
#include "L9963_utils.h"
#include "ntc.h"
TIMEBASE_HandleTypeDef data_reading_timebase_handle;
extern volatile uint16_t vcells[N_SLAVES][N_CELLS_PER_SLAVE];
extern volatile uint16_t vgpio[N_SLAVES][N_GPIOS_PER_SLAVE];
extern uint8_t ams_error;
float vbattery_monitor;
float vbattery_sum;


void data_reading_timebase_init(void) {
    uint8_t interval;
  
    TIMEBASE_init(&data_reading_timebase_handle, &htim6, 1000);
  
    TIMEBASE_add_interval(&data_reading_timebase_handle, 10000, &interval);
    TIMEBASE_register_callback(&data_reading_timebase_handle, interval, data_reading_l9963e_cb);
  }

// Induce AMS ERROR if there is a overvoltage for 500 ms
STMLIBS_StatusTypeDef data_reading_l9963e_cb(){
    uint8_t is_ntc_measure_required = ntc_is_measure_ext_time();
    static uint8_t overvoltage_count[N_SLAVES][N_CELLS_PER_SLAVE] = {0};
    static uint8_t overtemperature_count[N_SLAVES][N_GPIOS_PER_SLAVE] = {0};

    L9963E_utils_read_all_cells(is_ntc_measure_required);

    L9963E_utils_get_total_batt_mv(&vbattery_monitor, &vbattery_sum);

    if (is_ntc_measure_required){
        for (uint8_t i = 0; i < N_SLAVES; i++){
            for (uint8_t j = 0; j < N_GPIOS_PER_SLAVE; j++){

                if (vgpio[i][j] > OVERTEMPERATURE_TRESHOLD){
                    overtemperature_count[i][j]++;

                    if (overtemperature_count[i][j] > 100){ // 1000 ms
                        ams_error = SET;
                    }
                    
                } else {
                    overtemperature_count[i][j] = 0;
                }
            }
        }
    }

    for (uint8_t i = 0; i < N_SLAVES; i++){
        for (uint8_t j = 0; j < N_CELLS_PER_SLAVE; j++){
            if (vcells[i][j] > OVERVOLTAGE_TRESHOLD){
                overvoltage_count[i][j]++;
                if (overvoltage_count[i][j] > 50){
                    ams_error = SET;
                }
            } else {
                overvoltage_count[i][j] = 0;
            }
        }
    }
    return STMLIBS_OK;

}

void data_reading_timebase_routine(void) {
    TIMEBASE_routine(&data_reading_timebase_handle);
}