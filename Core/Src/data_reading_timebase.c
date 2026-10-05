#include "data_reading_timebase.h"
#include "L9963_utils.h"
#include "ntc.h"
#include "bms_temperature.h"
#include <math.h>
TIMEBASE_HandleTypeDef data_reading_timebase_handle;
extern volatile uint16_t vcells[N_SLAVES][N_CELLS_PER_SLAVE];
extern volatile uint16_t vgpio[N_SLAVES][N_GPIOS_PER_SLAVE];
extern uint8_t ams_error;
float vbattery_monitor;
float vbattery_sum;
static uint32_t supervision_started_at;
static uint8_t supervision_started;
static uint8_t next_module;
static uint8_t round_failed;
static uint8_t pack_sample_valid;

uint8_t data_reading_waiting_for_first_sample(void) {
    return supervision_started && !pack_sample_valid &&
        HAL_GetTick() - supervision_started_at <= MEASUREMENT_MAX_AGE_MS;
}


STMLIBS_StatusTypeDef data_reading_timebase_init(void) {
    uint8_t interval;
    supervision_started_at = HAL_GetTick();
    supervision_started = 1;
    next_module = 0;
    round_failed = 0;
    pack_sample_valid = 0;
    vbattery_monitor = vbattery_sum = NAN;
    if (TIMEBASE_init(&data_reading_timebase_handle, &htim6, 1000) != STMLIBS_OK ||
        TIMEBASE_add_interval(&data_reading_timebase_handle, 10000, &interval) != STMLIBS_OK ||
        TIMEBASE_register_callback(&data_reading_timebase_handle, interval, data_reading_l9963e_cb) != STMLIBS_OK)
        return STMLIBS_ERROR;
    return STMLIBS_OK;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    TIMEBASE_TimerElapsedCallback(&data_reading_timebase_handle, htim);
}

// Induce AMS ERROR if there is a overvoltage for 500 ms
STMLIBS_StatusTypeDef data_reading_l9963e_cb(){
    uint8_t module = next_module;
    next_module = (next_module + 1U) % N_SLAVES;
    if (module == 0) round_failed = 0;
    static uint32_t overvoltage_since[N_SLAVES][N_CELLS_PER_SLAVE] = {0};
    static uint8_t overvoltage_active[N_SLAVES][N_CELLS_PER_SLAVE] = {0};
#if !BMS_NTC_CALIBRATION_VERIFIED
    static uint8_t overtemperature_active[N_SLAVES][N_GPIOS_PER_SLAVE] = {0};
    static uint32_t overtemperature_since[N_SLAVES][N_GPIOS_PER_SLAVE] = {0};
#endif

    if (L9963E_utils_read_cells(module, 1) != L9963_UTILS_OK) {
        round_failed = 1;
        vbattery_monitor = vbattery_sum = NAN;
        ams_error = SET;
        Set_AMS_Error();
        return STMLIBS_ERROR;
    }

    if (next_module == 0 && !round_failed) {
        L9963E_utils_get_total_batt_mv(&vbattery_monitor, &vbattery_sum);
        pack_sample_valid = 1;
    }

    {
        {
            uint8_t i = module;
            for (uint8_t j = 0; j < N_GPIOS_PER_SLAVE; j++){

                /* Slave PCB has NTCs on GPIO3/4/5/6/8/9; slot 4 is GPIO7. */
                if (j == 4) continue;
#if BMS_NTC_CALIBRATION_VERIFIED
                if (BMS_temperature_fault(ntc_get_ext_temp(i * N_GPIOS_PER_SLAVE + j))) {
                    ams_error = SET;
                    Set_AMS_Error();
                }
#else
                if (vgpio[i][j] * 0.000089f > LEGACY_GPIO_THRESHOLD_V){
                    if (!overtemperature_active[i][j]) {
                        overtemperature_active[i][j] = 1;
                        overtemperature_since[i][j] = HAL_GetTick();
                    }

                    if (HAL_GetTick() - overtemperature_since[i][j] >= 1000U){ // 1000 ms
                        ams_error = SET;
                    }
                    
                } else {
                    overtemperature_active[i][j] = 0;
                }
#endif
            }
        }
    }

    {
            uint8_t i = module;
        for (uint8_t j = 0; j < N_CELLS_PER_SLAVE; j++){
            if (L9963E_utils_get_cell_mv(i, j) > OVERVOLTAGE_THRESHOLD_MV){
                if (!overvoltage_active[i][j]) {
                    overvoltage_active[i][j] = 1;
                    overvoltage_since[i][j] = HAL_GetTick();
                }
                if (HAL_GetTick() - overvoltage_since[i][j] >= 500U){
                    ams_error = SET;
                }
            } else {
                overvoltage_active[i][j] = 0;
            }
        }
    }
    return STMLIBS_OK;

}

void data_reading_timebase_routine(void) {
    if (TIMEBASE_routine(&data_reading_timebase_handle) != STMLIBS_OK ||
        (supervision_started && HAL_GetTick() - supervision_started_at > MEASUREMENT_MAX_AGE_MS &&
         !L9963E_utils_measurements_fresh(MEASUREMENT_MAX_AGE_MS))) {
        ams_error = SET;
        Set_AMS_Error();
    }
}