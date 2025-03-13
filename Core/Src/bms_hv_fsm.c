/*
 * "THE BEER-WARE LICENSE" (Revision 69):
 * Squadra Corse firmware team wrote this file. As long as you retain this notice
 * you can do whatever you want with this stuff. If we meet some day, and you
 * think this stuff is worth it, you can buy us a beer in return.
 *
 * Authors
 * - Filippo Rossi <filippo.rossi.sc@gmail.com>
 * - Federico Carbone <federico.carbone.sc@gmail.com>
 */

#include "bms_hv_fsm.h"
#ifndef __weak
#define __weak __attribute__((weak))
#endif // __weak

// L9963E_StatusTypeDef L9963E_read_batt_voltage(L9963E_HandleTypeDef *handle,
//     uint8_t device,
//     uint16_t *vbatt_monitor,
//     uint32_t *vbatt_sum);

// L9963 initialization
uint16_t vbattery_monitor;
uint32_t vbattery_sum;
extern L9963E_HandleTypeDef* L9963E_handle;
extern uint8_t n_slaves;
// Variables
extern uint8_t air_neg_int_state_closed, air_pos_int_state_closed ,air_pos_mech_state_open, air_neg_mech_state_open,
        ams_error, imd_error, dcbus_overvoltage, nstg_dcbus_overvoltage;


variables_t variables;

static uint32_t resetting_error_entry_time = 0;
static uint8_t driving_mode, charging_mode;
driving_mode = 0;
charging_mode = 0;
// Variable Update function
void variables_update(variables_t* variables){

    L9963E_read_batt_voltage(L9963E_handle, n_slaves, &vbattery_monitor, &vbattery_sum);

    variables -> air_neg_int_state_closed = AIRs_Neg_Int_Closed();
    variables -> air_pos_int_state_closed = AIRs_Pos_Int_Closed();
    variables -> air_pos_mech_state_open = AIRs_Pos_Mech_Open();
    variables -> air_neg_mech_state_open = AIRs_Neg_Mech_Open();
    variables -> dcbus_overvoltage = DCBUS_Over60V();
    variables -> nstg_dcbus_overvoltage = STG_DCBUS_Over60V();
    variables -> vbattery_monitor = vbattery_monitor;
    variables -> vbattery_sum = vbattery_sum;
}
// Private wrapper function signatures

uint32_t _FSM_BMS_HV_active_idle_event_handle(uint8_t event);
uint32_t _FSM_BMS_HV_active_idle_do_work();

uint32_t _FSM_BMS_HV_balancing_event_handle(uint8_t event);
uint32_t _FSM_BMS_HV_balancing_do_work();

uint32_t _FSM_BMS_HV_charging_idle_event_handle(uint8_t event);
uint32_t _FSM_BMS_HV_charging_idle_do_work();

uint32_t _FSM_BMS_HV_driving_idle_event_handle(uint8_t event);
uint32_t _FSM_BMS_HV_driving_idle_do_work();

uint32_t _FSM_BMS_HV_resetting_airs_precharge_event_handle(uint8_t event);
uint32_t _FSM_BMS_HV_resetting_airs_precharge_do_work();

uint32_t _FSM_BMS_HV_resetting_errors_event_handle(uint8_t event);
uint32_t _FSM_BMS_HV_resetting_errors_do_work();

uint32_t _FSM_BMS_HV_closing_air_neg_event_handle(uint8_t event);
uint32_t _FSM_BMS_HV_closing_air_neg_do_work();

uint32_t _FSM_BMS_HV_closing_precharge_event_handle(uint8_t event);
uint32_t _FSM_BMS_HV_closing_precharge_do_work();

uint32_t _FSM_BMS_HV_precharge_event_handle(uint8_t event);
uint32_t _FSM_BMS_HV_precharge_do_work();

uint32_t _FSM_BMS_HV_closing_air_pos_event_handle(uint8_t event);
uint32_t _FSM_BMS_HV_closing_air_pos_do_work();

uint32_t _FSM_BMS_HV_opening_precharge_event_handle(uint8_t event);
uint32_t _FSM_BMS_HV_opening_precharge_do_work();

uint32_t _FSM_BMS_HV_driving_event_handle(uint8_t event);
uint32_t _FSM_BMS_HV_driving_do_work();

uint32_t _FSM_BMS_HV_charging_event_handle(uint8_t event);
uint32_t _FSM_BMS_HV_charging_do_work();

uint32_t _FSM_BMS_HV_ams_imd_error_event_handle(uint8_t event);
uint32_t _FSM_BMS_HV_ams_imd_error_do_work();


FSM_StateTypeDef state_table[_FSM_BMS_HV_STATE_COUNT] = {
    [FSM_BMS_HV_active_idle] = {
        .event_handler = _FSM_BMS_HV_active_idle_event_handle,
        .entry = FSM_BMS_HV_active_idle_entry,
        .do_work = _FSM_BMS_HV_active_idle_do_work,
        .exit = FSM_BMS_HV_active_idle_exit,
    },
    [FSM_BMS_HV_balancing] = {
        .event_handler = _FSM_BMS_HV_balancing_event_handle,
        .entry = FSM_BMS_HV_balancing_entry,
        .do_work = _FSM_BMS_HV_balancing_do_work,
        .exit = FSM_BMS_HV_balancing_exit,
    },
    [FSM_BMS_HV_charging_idle] = {
        .event_handler = _FSM_BMS_HV_charging_idle_event_handle,
        .entry = FSM_BMS_HV_charging_idle_entry,
        .do_work = _FSM_BMS_HV_charging_idle_do_work,
        .exit = FSM_BMS_HV_charging_idle_exit,
    },
    [FSM_BMS_HV_driving_idle] = {
        .event_handler = _FSM_BMS_HV_driving_idle_event_handle,
        .entry = FSM_BMS_HV_driving_idle_entry,
        .do_work = _FSM_BMS_HV_driving_idle_do_work,
        .exit = FSM_BMS_HV_driving_idle_exit,
    },
    [FSM_BMS_HV_resetting_airs_precharge] = {
        .event_handler = _FSM_BMS_HV_resetting_airs_precharge_event_handle,
        .entry = FSM_BMS_HV_resetting_airs_precharge_entry,
        .do_work = _FSM_BMS_HV_resetting_airs_precharge_do_work,
        .exit = FSM_BMS_HV_resetting_airs_precharge_exit,
    },
    [FSM_BMS_HV_resetting_errors] = {
        .event_handler = _FSM_BMS_HV_resetting_errors_event_handle,
        .entry = FSM_BMS_HV_resetting_errors_entry,
        .do_work = _FSM_BMS_HV_resetting_errors_do_work,
        .exit = FSM_BMS_HV_resetting_errors_exit,
    },
    [FSM_BMS_HV_closing_air_neg] = {
        .event_handler = _FSM_BMS_HV_closing_air_neg_event_handle,
        .entry = FSM_BMS_HV_closing_air_neg_entry,
        .do_work = _FSM_BMS_HV_closing_air_neg_do_work,
        .exit = FSM_BMS_HV_closing_air_neg_exit,
    },
    [FSM_BMS_HV_closing_precharge] = {
        .event_handler = _FSM_BMS_HV_closing_precharge_event_handle,
        .entry = FSM_BMS_HV_closing_precharge_entry,
        .do_work = _FSM_BMS_HV_closing_precharge_do_work,
        .exit = FSM_BMS_HV_closing_precharge_exit,
    },
    [FSM_BMS_HV_precharge] = {
        .event_handler = _FSM_BMS_HV_precharge_event_handle,
        .entry = FSM_BMS_HV_precharge_entry,
        .do_work = _FSM_BMS_HV_precharge_do_work,
        .exit = FSM_BMS_HV_precharge_exit,
    },
    [FSM_BMS_HV_closing_air_pos] = {
        .event_handler = _FSM_BMS_HV_closing_air_pos_event_handle,
        .entry = FSM_BMS_HV_closing_air_pos_entry,
        .do_work = _FSM_BMS_HV_closing_air_pos_do_work,
        .exit = FSM_BMS_HV_closing_air_pos_exit,
    },
    [FSM_BMS_HV_opening_precharge] = {
        .event_handler = _FSM_BMS_HV_opening_precharge_event_handle,
        .entry = FSM_BMS_HV_opening_precharge_entry,
        .do_work = _FSM_BMS_HV_opening_precharge_do_work,
        .exit = FSM_BMS_HV_opening_precharge_exit,
    },
    [FSM_BMS_HV_driving] = {
        .event_handler = _FSM_BMS_HV_driving_event_handle,
        .entry = FSM_BMS_HV_driving_entry,
        .do_work = _FSM_BMS_HV_driving_do_work,
        .exit = FSM_BMS_HV_driving_exit,
    },
    [FSM_BMS_HV_charging] = {
        .event_handler = _FSM_BMS_HV_charging_event_handle,
        .entry = FSM_BMS_HV_charging_entry,
        .do_work = _FSM_BMS_HV_charging_do_work,
        .exit = FSM_BMS_HV_charging_exit,
    },
    [FSM_BMS_HV_ams_imd_error] = {
        .event_handler = _FSM_BMS_HV_ams_imd_error_event_handle,
        .entry = FSM_BMS_HV_ams_imd_error_entry,
        .do_work = _FSM_BMS_HV_ams_imd_error_do_work,
        .exit = FSM_BMS_HV_ams_imd_error_exit,
    },
};

FSM_ConfigTypeDef config = {
    .state_length = _FSM_BMS_HV_STATE_COUNT,
    .state_table = state_table,
};

STMLIBS_StatusTypeDef FSM_BMS_HV_init(
    FSM_HandleTypeDef *handle,
    uint8_t event_count,
    FSM_callback_function run_callback,
    FSM_callback_function transition_callback
) {
    return FSM_init(handle, &config, event_count, run_callback, transition_callback);
}

// State control functions

// State Entry Actions (User defined)
void FSM_BMS_HV_resetting_errors_entry() {
    resetting_error_entry_time = HAL_GetTick();
    // Reset AMS Error
    Reset_AMS_Error();
    variables.ams_error = RESET;

    return;
}
void FSM_BMS_HV_close_air_neg_entry() {
    // Close Negative Air
    Close_Air_Neg();
    return;
}
void FSM_BMS_HV_close_air_pos_entry() {
    // Close Negative Air
    Close_Air_Pos();
    return;
}
void FSM_BMS_HV_close_precharge_entry() {
    // Open Precharge
    Close_Precharge();
    return;
}
void FSM_BMS_HV_open_precharge_entry() {
    // Open Precharge
    Open_Precharge();
    return;
}
void FSM_BMS_HV_resetting_airs_precharge_entry() {
    Open_Precharge();
    Open_Air_Pos();
    Open_Air_Neg();
    return;
}
void FSM_BMS_HV_amd_imd_error_entry() {
    // Open Precharge
    Err_LED_On();
    return;
}
/** @brief wrapper of FSM_BMS_HV_event_handle, with exit state checking */
uint32_t _FSM_BMS_HV_active_idle_event_handle(uint8_t event) {
    uint32_t next = (uint32_t)FSM_BMS_HV_active_idle_event_handle(event);

    switch (next) {
    case FSM_BMS_HV_active_idle: // Reentrance is always supported on event handlers
    case FSM_BMS_HV_balancing:
    case FSM_BMS_HV_charging_idle:
    case FSM_BMS_HV_driving_idle:
    case FSM_BMS_HV_ams_imd_error:
        return next;
    default:
        return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_do_work, with exit state checking */
uint32_t _FSM_BMS_HV_active_idle_do_work() {
    uint32_t next = (uint32_t)FSM_BMS_HV_active_idle_do_work();

    switch (next) {
    case FSM_BMS_HV_balancing:
    case FSM_BMS_HV_charging_idle:
    case FSM_BMS_HV_driving_idle:
    case FSM_BMS_HV_ams_imd_error:
        return next;
    default:
        return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_event_handle, with exit state checking */
uint32_t _FSM_BMS_HV_balancing_event_handle(uint8_t event) {
    uint32_t next = (uint32_t)FSM_BMS_HV_balancing_event_handle(event);

    switch (next) {
    case FSM_BMS_HV_balancing: // Reentrance is always supported on event handlers
    case FSM_BMS_HV_active_idle:
    case FSM_BMS_HV_ams_imd_error:
        return next;
    default:
        return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_do_work, with exit state checking */
uint32_t _FSM_BMS_HV_balancing_do_work() {
    uint32_t next = (uint32_t)FSM_BMS_HV_balancing_do_work();

    switch (next) {
    case FSM_BMS_HV_active_idle:
    case FSM_BMS_HV_ams_imd_error:
        return next;
    default:
        return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_event_handle, with exit state checking */
uint32_t _FSM_BMS_HV_charging_idle_event_handle(uint8_t event) {
    uint32_t next = (uint32_t)FSM_BMS_HV_charging_idle_event_handle(event);

    switch (next) {
    case FSM_BMS_HV_charging_idle: // Reentrance is always supported on event handlers
    case FSM_BMS_HV_active_idle:
    case FSM_BMS_HV_closing_air_neg:
    case FSM_BMS_HV_ams_imd_error:
        return next;
    default:
        return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_do_work, with exit state checking */
uint32_t _FSM_BMS_HV_charging_idle_do_work() {
    uint32_t next = (uint32_t)FSM_BMS_HV_charging_idle_do_work();

    switch (next) {
    case FSM_BMS_HV_active_idle:
    case FSM_BMS_HV_closing_air_neg:
    case FSM_BMS_HV_ams_imd_error:
        return next;
    default:
        return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_event_handle, with exit state checking */
uint32_t _FSM_BMS_HV_driving_idle_event_handle(uint8_t event) {
    uint32_t next = (uint32_t)FSM_BMS_HV_driving_idle_event_handle(event);

    switch (next) {
    case FSM_BMS_HV_driving_idle: // Reentrance is always supported on event handlers
    case FSM_BMS_HV_active_idle:
    case FSM_BMS_HV_closing_air_neg:
    case FSM_BMS_HV_ams_imd_error:
        return next;
    default:
        return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_do_work, with exit state checking */
uint32_t _FSM_BMS_HV_driving_idle_do_work() {
    uint32_t next = (uint32_t)FSM_BMS_HV_driving_idle_do_work();

    switch (next) {
    case FSM_BMS_HV_active_idle:
    case FSM_BMS_HV_closing_air_neg:
    case FSM_BMS_HV_ams_imd_error:
        return next;
    default:
        return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_event_handle, with exit state checking */
uint32_t _FSM_BMS_HV_resetting_airs_precharge_event_handle(uint8_t event) {
    uint32_t next = (uint32_t)FSM_BMS_HV_resetting_airs_precharge_event_handle(event);

    switch (next) {
    case FSM_BMS_HV_resetting_airs_precharge: // Reentrance is always supported on event handlers
    case FSM_BMS_HV_active_idle:
    case FSM_BMS_HV_ams_imd_error:
        return next;
    default:
        return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_do_work, with exit state checking */
uint32_t _FSM_BMS_HV_resetting_airs_precharge_do_work() {
    uint32_t next = (uint32_t)FSM_BMS_HV_resetting_airs_precharge_do_work();

    switch (next) {
    case FSM_BMS_HV_active_idle:
    case FSM_BMS_HV_ams_imd_error:
        return next;
    default:
        return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_event_handle, with exit state checking */
uint32_t _FSM_BMS_HV_resetting_errors_event_handle(uint8_t event) {
    uint32_t next = (uint32_t)FSM_BMS_HV_resetting_errors_event_handle(event);

    switch (next) {
    case FSM_BMS_HV_resetting_errors: // Reentrance is always supported on event handlers
    case FSM_BMS_HV_active_idle:
        return next;
    default:
        return _FSM_BMS_HV_DIE;
    }
}
// Resetting errors do_work
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_resetting_errors_do_work() {

    if (HAL_GetTick() - resetting_error_entry_time > 10000) {
        return FSM_BMS_HV_active_idle;
    }

    return FSM_BMS_HV_resetting_errors;
}
/** @brief wrapper of FSM_BMS_HV_do_work, with exit state checking */
uint32_t _FSM_BMS_HV_resetting_errors_do_work() {
    uint32_t next = (uint32_t)FSM_BMS_HV_resetting_errors_do_work();

    switch (next) {
    case FSM_BMS_HV_resetting_errors:
    case FSM_BMS_HV_active_idle:
        return next;
    default:
        return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_event_handle, with exit state checking */
uint32_t _FSM_BMS_HV_closing_air_neg_event_handle(uint8_t event) {
    uint32_t next = (uint32_t)FSM_BMS_HV_closing_air_neg_event_handle(event);

    switch (next) {
    case FSM_BMS_HV_closing_air_neg: // Reentrance is always supported on event handlers
    case FSM_BMS_HV_closing_precharge:
    case FSM_BMS_HV_resetting_airs_precharge:
    case FSM_BMS_HV_ams_imd_error:
        return next;
    default:
        return _FSM_BMS_HV_DIE;
    }
}
// User defined closing_air_neg_do_work
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_closing_air_neg_do_work() {

    // Check if air negative is closed
    if (AIRs_Neg_Int_Closed() == 1 && AIRs_Neg_Mech_Open() == 0) {
        return FSM_BMS_HV_closing_precharge;

    } else {
        // If air negative is not closed, invoke the error
        return FSM_BMS_HV_resetting_airs_precharge;
    }

    return FSM_BMS_HV_closing_air_neg;
}

/** @brief wrapper of FSM_BMS_HV_do_work, with exit state checking */
uint32_t _FSM_BMS_HV_closing_air_neg_do_work() {
    uint32_t next = (uint32_t)FSM_BMS_HV_closing_air_neg_do_work();

    switch (next) {
    case FSM_BMS_HV_closing_precharge:
    case FSM_BMS_HV_resetting_airs_precharge:
    case FSM_BMS_HV_ams_imd_error:
        return next;
    default:
        return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_event_handle, with exit state checking */
uint32_t _FSM_BMS_HV_closing_precharge_event_handle(uint8_t event) {
    uint32_t next = (uint32_t)FSM_BMS_HV_closing_precharge_event_handle(event);

    switch (next) {
    case FSM_BMS_HV_closing_precharge: // Reentrance is always supported on event handlers
    case FSM_BMS_HV_precharge:
    case FSM_BMS_HV_resetting_airs_precharge:
    case FSM_BMS_HV_ams_imd_error:
        return next;
    default:
        return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_do_work, with exit state checking */
uint32_t _FSM_BMS_HV_closing_precharge_do_work() {
    uint32_t next = (uint32_t)FSM_BMS_HV_closing_precharge_do_work();

    switch (next) {
    case FSM_BMS_HV_precharge:
    case FSM_BMS_HV_resetting_airs_precharge:
    case FSM_BMS_HV_ams_imd_error:
        return next;
    default:
        return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_event_handle, with exit state checking */
uint32_t _FSM_BMS_HV_precharge_event_handle(uint8_t event) {
    uint32_t next = (uint32_t)FSM_BMS_HV_precharge_event_handle(event);

    switch (next) {
    case FSM_BMS_HV_precharge: // Reentrance is always supported on event handlers
    case FSM_BMS_HV_closing_air_pos:
        return next;
    default:
        return _FSM_BMS_HV_DIE;
    }
}
// User defined precharge_do_work
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_precharge_do_work() {
    variables_update(&variables);

    if (variables.vbattery_monitor > MAX_VOLTAGE * 0.95) {
        return FSM_BMS_HV_closing_air_pos;
    }

    return FSM_BMS_HV_precharge;
}
/** @brief wrapper of FSM_BMS_HV_do_work, with exit state checking */
uint32_t _FSM_BMS_HV_precharge_do_work() {
    uint32_t next = (uint32_t)FSM_BMS_HV_precharge_do_work();

    switch (next) {
    case FSM_BMS_HV_closing_air_pos:
        return next;
    default:
        return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_event_handle, with exit state checking */
uint32_t _FSM_BMS_HV_closing_air_pos_event_handle(uint8_t event) {
    uint32_t next = (uint32_t)FSM_BMS_HV_closing_air_pos_event_handle(event);

    switch (next) {
    case FSM_BMS_HV_closing_air_pos: // Reentrance is always supported on event handlers
    case FSM_BMS_HV_opening_precharge:
    case FSM_BMS_HV_resetting_airs_precharge:
    case FSM_BMS_HV_ams_imd_error:
        return next;
    default:
        return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_do_work, with exit state checking */
uint32_t _FSM_BMS_HV_closing_air_pos_do_work() {
    uint32_t next = (uint32_t)FSM_BMS_HV_closing_air_pos_do_work();

    switch (next) {
    case FSM_BMS_HV_opening_precharge:
    case FSM_BMS_HV_resetting_airs_precharge:
    case FSM_BMS_HV_ams_imd_error:
        return next;
    default:
        return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_event_handle, with exit state checking */
uint32_t _FSM_BMS_HV_opening_precharge_event_handle(uint8_t event) {
    uint32_t next = (uint32_t)FSM_BMS_HV_opening_precharge_event_handle(event);

    switch (next) {
    case FSM_BMS_HV_opening_precharge: // Reentrance is always supported on event handlers
    case FSM_BMS_HV_driving:
    case FSM_BMS_HV_charging:
    case FSM_BMS_HV_ams_imd_error:
        return next;
    default:
        return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_do_work, with exit state checking */
uint32_t _FSM_BMS_HV_opening_precharge_do_work() {
    uint32_t next = (uint32_t)FSM_BMS_HV_opening_precharge_do_work();

    switch (next) {
    case FSM_BMS_HV_driving:
    case FSM_BMS_HV_charging:
    case FSM_BMS_HV_ams_imd_error:
        return next;
    default:
        return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_event_handle, with exit state checking */
uint32_t _FSM_BMS_HV_driving_event_handle(uint8_t event) {
    uint32_t next = (uint32_t)FSM_BMS_HV_driving_event_handle(event);

    switch (next) {
    case FSM_BMS_HV_driving: // Reentrance is always supported on event handlers
    case FSM_BMS_HV_resetting_airs_precharge:
    case FSM_BMS_HV_ams_imd_error:
        return next;
    default:
        return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_do_work, with exit state checking */
uint32_t _FSM_BMS_HV_driving_do_work() {
    uint32_t next = (uint32_t)FSM_BMS_HV_driving_do_work();

    switch (next) {
    case FSM_BMS_HV_resetting_airs_precharge:
    case FSM_BMS_HV_ams_imd_error:
        return next;
    default:
        return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_event_handle, with exit state checking */
uint32_t _FSM_BMS_HV_charging_event_handle(uint8_t event) {
    uint32_t next = (uint32_t)FSM_BMS_HV_charging_event_handle(event);

    switch (next) {
    case FSM_BMS_HV_charging: // Reentrance is always supported on event handlers
    case FSM_BMS_HV_resetting_airs_precharge:
    case FSM_BMS_HV_ams_imd_error:
        return next;
    default:
        return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_do_work, with exit state checking */
uint32_t _FSM_BMS_HV_charging_do_work() {
    uint32_t next = (uint32_t)FSM_BMS_HV_charging_do_work();

    switch (next) {
    case FSM_BMS_HV_resetting_airs_precharge:
    case FSM_BMS_HV_ams_imd_error:
        return next;
    default:
        return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_event_handle, with exit state checking */
uint32_t _FSM_BMS_HV_ams_imd_error_event_handle(uint8_t event) {
    uint32_t next = (uint32_t)FSM_BMS_HV_ams_imd_error_event_handle(event);

    switch (next) {
    case FSM_BMS_HV_ams_imd_error: // Reentrance is always supported on event handlers
    case FSM_BMS_HV_resetting_errors:
        return next;
    default:
        return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_do_work, with exit state checking */
uint32_t _FSM_BMS_HV_ams_imd_error_do_work() {
    uint32_t next = (uint32_t)FSM_BMS_HV_ams_imd_error_do_work();

    switch (next) {
    case FSM_BMS_HV_resetting_errors:
        return next;
    default:
        return _FSM_BMS_HV_DIE;
    }
}

