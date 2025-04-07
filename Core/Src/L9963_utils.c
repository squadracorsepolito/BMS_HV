#include "L9963_utils.h"
#include "ntc.h"

/* Using L9963TH to send and receive data
    L9963TL will be used to check the correctness of communication
*/
volatile uint16_t vcells[N_SLAVES][N_CELLS_PER_SLAVE];
volatile uint16_t vgpio[N_SLAVES][N_GPIOS_PER_SLAVE];
volatile uint16_t vtot[N_SLAVES];
volatile uint32_t vsumbatt[N_SLAVES];
L9963E_HandleTypeDef hl9963e;

const L9963E_IfTypeDef interface_H = {.L9963E_IF_DelayMs       = DelayMs,
                                      .L9963E_IF_GetTickMs     = GetTickMs,
                                      .L9963E_IF_GPIO_ReadPin  = L9963TH_GPIO_ReadPin,
                                      .L9963E_IF_GPIO_WritePin = L9963TH_GPIO_WritePin,
                                      .L9963E_IF_SPI_Receive   = L9963TH_SPI_Receive,
                                      .L9963E_IF_SPI_Transmit  = L9963TH_SPI_Transmit};

const L9963E_IfTypeDef interface_L = {.L9963E_IF_DelayMs       = DelayMs,
                                      .L9963E_IF_GetTickMs     = GetTickMs,
                                      .L9963E_IF_GPIO_ReadPin  = L9963TL_GPIO_ReadPin,
                                      .L9963E_IF_GPIO_WritePin = L9963TL_GPIO_WritePin,
                                      .L9963E_IF_SPI_Receive   = L9963TL_SPI_Receive,
                                      .L9963E_IF_SPI_Transmit  = L9963TL_SPI_Transmit};

// TRYING TO FIGURE OUT HOW TO INITIALIZE THE DRIVER HANDLE
void L9963E_utils_init(void) {
    L9963E_init(&hl9963e, interface_H, N_SLAVES);
    L9963E_addressing_procedure(&hl9963e, 0b11, 1, 0b00, 1);

    /** Configuring the chips by writing to the registers, since each chip 
        has the same configuration, we are using Broadcast access 
        to write to all chips at once **/

    // Configuring GPIOs
    L9963E_RegisterUnionTypeDef gpio9_3_conf_reg = {.generic = L9963E_GPIO9_3_CONF_DEFAULT};
    gpio9_3_conf_reg.GPIO9_3_CONF.GPIO7_CONFIG   = 0;
    gpio9_3_conf_reg.GPIO9_3_CONF.GPIO8_CONFIG   = 0;
    L9963E_DRV_reg_write(
        &(hl9963e.drv_handle), L9963E_DEVICE_BROADCAST, L9963E_GPIO9_3_CONF_ADDR, &gpio9_3_conf_reg, 10);

    // Configuring cells overvoltage/undervoltage thresholds
    L9963E_RegisterUnionTypeDef vcell_thresh_uv_ov_reg      = {.generic = L9963E_VCELL_THRESH_UV_OV_DEFAULT};
    vcell_thresh_uv_ov_reg.VCELL_THRESH_UV_OV.threshVcellOV = 0xff;
    L9963E_DRV_reg_write(
        &(hl9963e.drv_handle), L9963E_DEVICE_BROADCAST, L9963E_VCELL_THRESH_UV_OV_ADDR, &vcell_thresh_uv_ov_reg, 10);

    // Configuring total voltage tresholds
    L9963E_RegisterUnionTypeDef vbat_sum_th_reg  = {.generic = L9963E_VBATT_SUM_TH_DEFAULT};
    vbat_sum_th_reg.VBATT_SUM_TH.VBATT_SUM_OV_TH = 0xff;
    L9963E_DRV_reg_write(
        &(hl9963e.drv_handle), L9963E_DEVICE_BROADCAST, L9963E_VBATT_SUM_TH_ADDR, &vbat_sum_th_reg, 10);

    // Enabling the Reference Voltage for ADC
    L9963E_enable_vref(&hl9963e, L9963E_DEVICE_BROADCAST, 1);

    // Set communication timeout and enable cells
    L9963E_setCommTimeout(&hl9963e, _256MS, L9963E_DEVICE_BROADCAST, 0);
    L9963E_set_enabled_cells(&hl9963e, L9963E_DEVICE_BROADCAST, ENABLED_CELLS);

    /* Configuring balancing operations: Timed Balancing | 20s treshold*/
    L9963E_RegisterUnionTypeDef bal2_conf_reg = {.generic = L9963E_BAL_2_DEFAULT};
    bal2_conf_reg.Bal_2.Balmode               = 0b10;
    bal2_conf_reg.Bal_2.TimedBalacc           = 1;  // Fine resolution
    bal2_conf_reg.Bal_2.ThrTimedBalCell13     = 5;  // 20s treshold
    bal2_conf_reg.Bal_2.ThrTimedBalCell14     = 5;  // 20s treshold
    L9963E_RegisterUnionTypeDef bal3_conf_reg = {.generic = L9963E_BAL_3_DEFAULT};
    bal3_conf_reg.Bal_3.ThrTimedBalCell12     = 5;
    L9963E_RegisterUnionTypeDef bal5_conf_reg = {.generic = L9963E_BAL_5_DEFAULT};
    bal5_conf_reg.Bal_5.ThrTimedBalCell8      = 5;
    bal5_conf_reg.Bal_5.ThrTimedBalCell7      = 5;
    L9963E_RegisterUnionTypeDef bal6_conf_reg = {.generic = L9963E_BAL_6_DEFAULT};
    bal6_conf_reg.Bal_6.ThrTimedBalCell6      = 5;
    bal6_conf_reg.Bal_6.ThrTimedBalCell5      = 5;
    L9963E_RegisterUnionTypeDef bal7_conf_reg = {.generic = L9963E_BAL_7_DEFAULT};
    bal7_conf_reg.Bal_7.ThrTimedBalCell4      = 5;
    bal7_conf_reg.Bal_7.ThrTimedBalCell3      = 5;
    L9963E_RegisterUnionTypeDef bal8_conf_reg = {.generic = L9963E_BAL_8_DEFAULT};
    bal8_conf_reg.Bal_8.ThrTimedBalCell2      = 5;
    bal8_conf_reg.Bal_8.ThrTimedBalCell1      = 5;
    L9963E_DRV_reg_write(&(hl9963e.drv_handle), L9963E_DEVICE_BROADCAST, L9963E_Bal_3_ADDR, &bal3_conf_reg, 10);
    L9963E_DRV_reg_write(&(hl9963e.drv_handle), L9963E_DEVICE_BROADCAST, L9963E_Bal_5_ADDR, &bal5_conf_reg, 10);
    L9963E_DRV_reg_write(&(hl9963e.drv_handle), L9963E_DEVICE_BROADCAST, L9963E_Bal_2_ADDR, &bal2_conf_reg, 10);
    L9963E_DRV_reg_write(&(hl9963e.drv_handle), L9963E_DEVICE_BROADCAST, L9963E_Bal_6_ADDR, &bal6_conf_reg, 10);
    L9963E_DRV_reg_write(&(hl9963e.drv_handle), L9963E_DEVICE_BROADCAST, L9963E_Bal_7_ADDR, &bal7_conf_reg, 10);
    L9963E_DRV_reg_write(&(hl9963e.drv_handle), L9963E_DEVICE_BROADCAST, L9963E_Bal_8_ADDR, &bal8_conf_reg, 10);
    // Enabling balancing on selected cells
    L9963E_RegisterUnionTypeDef bal_cell14_7act = {.generic = L9963E_BALCELL14_7ACT_DEFAULT};
    bal_cell14_7act.BalCell14_7act.BAL14        = 0b10;
    bal_cell14_7act.BalCell14_7act.BAL13        = 0b10;
    bal_cell14_7act.BalCell14_7act.BAL12        = 0b10;
    bal_cell14_7act.BalCell14_7act.BAL8         = 0b10;
    bal_cell14_7act.BalCell14_7act.BAL7         = 0b10;
    L9963E_RegisterUnionTypeDef bal_cell6_1act  = {.generic = L9963E_BALCELL6_1ACT_DEFAULT};
    bal_cell6_1act.BalCell6_1act.BAL6           = 0b10;
    bal_cell6_1act.BalCell6_1act.BAL5           = 0b10;
    bal_cell6_1act.BalCell6_1act.BAL4           = 0b10;
    bal_cell6_1act.BalCell6_1act.BAL3           = 0b10;
    bal_cell6_1act.BalCell6_1act.BAL2           = 0b10;
    bal_cell6_1act.BalCell6_1act.BAL1           = 0b10;
    L9963E_DRV_reg_write(
        &(hl9963e.drv_handle), L9963E_DEVICE_BROADCAST, L9963E_BalCell6_1act_ADDR, &bal_cell6_1act, 10);
    L9963E_DRV_reg_write(
        &(hl9963e.drv_handle), L9963E_DEVICE_BROADCAST, L9963E_BalCell14_7act_ADDR, &bal_cell14_7act, 10);
}

void L9963E_utils_read_cells(uint8_t module_id, uint8_t read_gpio) {
    L9963E_StatusTypeDef e;
    uint8_t c_done;

    do {
        L9963E_poll_conversion(&hl9963e, module_id, &c_done);
    } while (!c_done);

    L9963E_start_conversion(&hl9963e, module_id, 0b000, read_gpio ? L9963E_GPIO_CONV : 0);
    
    uint16_t voltage       = 0;
    uint8_t d_rdy          = 0;

    /******* READING CELL VOLTAGE OF EACH INDIVIDUAL CELL *******/
    do {
        e = L9963E_read_cell_voltage(&hl9963e, module_id, L9963E_CELL1, &voltage, &d_rdy);
    } while (e != L9963E_OK || !d_rdy);
    vcells[module_id][0] = voltage;

    do {
        e = L9963E_read_cell_voltage(&hl9963e, module_id, L9963E_CELL2, &voltage, &d_rdy);
    } while (e != L9963E_OK || !d_rdy);
    vcells[module_id][1] = voltage;

    do {
        e = L9963E_read_cell_voltage(&hl9963e, module_id, L9963E_CELL3, &voltage, &d_rdy);
    } while (e != L9963E_OK || !d_rdy);
    vcells[module_id][2] = voltage;

    do {
        e = L9963E_read_cell_voltage(&hl9963e, module_id, L9963E_CELL4, &voltage, &d_rdy);
    } while (e != L9963E_OK || !d_rdy);
    vcells[module_id][3] = voltage;

    do {
        e = L9963E_read_cell_voltage(&hl9963e, module_id, L9963E_CELL5, &voltage, &d_rdy);
    } while (e != L9963E_OK || !d_rdy);
    vcells[module_id][4] = voltage;

    do {
        e = L9963E_read_cell_voltage(&hl9963e, module_id, L9963E_CELL6, &voltage, &d_rdy);
    } while (e != L9963E_OK || !d_rdy);
    vcells[module_id][5] = voltage;

    do {
        e = L9963E_read_cell_voltage(&hl9963e, module_id, L9963E_CELL7, &voltage, &d_rdy);
    } while (e != L9963E_OK || !d_rdy);
    vcells[module_id][6] = voltage;

    do {
        e = L9963E_read_cell_voltage(&hl9963e, module_id, L9963E_CELL8, &voltage, &d_rdy);
    } while (e != L9963E_OK || !d_rdy);
    vcells[module_id][7] = voltage;

    do {
        e = L9963E_read_cell_voltage(&hl9963e, module_id, L9963E_CELL12, &voltage, &d_rdy);
    } while (e != L9963E_OK || !d_rdy);
    vcells[module_id][8] = voltage;

    do {
        e = L9963E_read_cell_voltage(&hl9963e, module_id, L9963E_CELL13, &voltage, &d_rdy);
    } while (e != L9963E_OK || !d_rdy);
    vcells[module_id][9] = voltage;

    do {
        e = L9963E_read_cell_voltage(&hl9963e, module_id, L9963E_CELL14, &voltage, &d_rdy);
    } while (e != L9963E_OK || !d_rdy);
    vcells[module_id][10] = voltage;

    /******* READING TOTAL BATTERY VOLTAGES *******/
    do {
        e = L9963E_read_batt_voltage(
            &hl9963e, module_id, ((uint16_t *)&(vtot[module_id])), ((uint32_t *)&(vsumbatt[module_id])));
    } while (e != L9963E_OK);

    if (!read_gpio)
        return;

    /******* READING GPIO VOLTAGES *******/
    do {
        e = L9963E_read_gpio_voltage(&hl9963e, module_id, L9963E_GPIO3, &voltage, &d_rdy);
    } while (e != L9963E_OK || !d_rdy);
    vgpio[module_id][0] = voltage;

    do {
        e = L9963E_read_gpio_voltage(&hl9963e, module_id, L9963E_GPIO4, &voltage, &d_rdy);
    } while (e != L9963E_OK || !d_rdy);
    vgpio[module_id][1] = voltage;

    do {
        e = L9963E_read_gpio_voltage(&hl9963e, module_id, L9963E_GPIO5, &voltage, &d_rdy);
    } while (e != L9963E_OK || !d_rdy);
    vgpio[module_id][2] = voltage;

    do {
        e = L9963E_read_gpio_voltage(&hl9963e, module_id, L9963E_GPIO6, &voltage, &d_rdy);
    } while (e != L9963E_OK || !d_rdy);
    vgpio[module_id][3] = voltage;

    do {
        e = L9963E_read_gpio_voltage(&hl9963e, module_id, L9963E_GPIO7, &voltage, &d_rdy);
    } while (e != L9963E_OK || !d_rdy);
    vgpio[module_id][4] = voltage;

    do {
        e = L9963E_read_gpio_voltage(&hl9963e, module_id, L9963E_GPIO8, &voltage, &d_rdy);
    } while (e != L9963E_OK || !d_rdy);
    vgpio[module_id][5] = voltage;

    do {
        e = L9963E_read_gpio_voltage(&hl9963e, module_id, L9963E_GPIO9, &voltage, &d_rdy);
    } while (e != L9963E_OK || !d_rdy);
    vgpio[module_id][6] = voltage;

    ntc_set_ext_data((uint16_t *)vgpio, N_GPIOS_PER_SLAVE, 0);
}

void L9963E_utils_read_all_cells(uint8_t read_gpio){
    for (uint8_t i = 0; i < N_SLAVES; i++) {
        L9963E_utils_read_cells(i, read_gpio);
    }
}


uint16_t const *L9963E_utils_get_module_gpios(uint8_t module_id, uint8_t *len) {
    if (module_id >= N_SLAVES)
        return NULL;
    if (len)
        *len = N_GPIOS_PER_SLAVE;
    return (uint16_t *)vgpio[module_id];
}

uint16_t const *L9963E_utils_get_module_cells(uint8_t module_id, uint8_t *len) {
    if (module_id >= N_SLAVES)
        return NULL;
    if (len)
        *len = N_CELLS_PER_SLAVE;
    return (uint16_t *)vcells[module_id];
}

uint16_t const *L9963E_utils_get_all__cells(){
    return (uint16_t *)vcells;
}

float L9963E_utils_get_cell_mv(uint8_t module_id, uint8_t index) {
    return vcells[module_id][index] * 89e-3f;
}

void L9963E_utils_get_batt_mv(float *v_tot_module, float *v_sum_module, uint8_t module_id) {
    *v_tot_module = vtot[module_id] * 1.33f;
    *v_sum_module = vsumbatt[module_id] * 89e-3f;
}

void L9963E_utils_get_total_batt_mv(float *v_battery_monitor, float *v_battery_sum) {
    *v_battery_monitor = 0;
    *v_battery_sum = 0;
    for (uint8_t i = 0; i < N_SLAVES; i++) {
        *v_battery_monitor += vtot[i] * 1.33f;
        *v_battery_sum += vsumbatt[i] * 89e-3f;
    }
}

// Timed balancing mode
L9963_Utils_StatusTypeDef L9963E_utils_balance_cells(void) {
    L9963E_StatusTypeDef e;
    uint8_t eof_bal                  = 0;
    uint8_t bal_on                   = 0;
    // L9963E_BurstCmdTypeDef burst_cmd = _0x78BurstCmd;
    // L9963E_BurstUnionTypeDef burst_data[N_SLAVES];
    L9963E_RegisterUnionTypeDef bal1_conf_reg = {.generic = L9963E_BAL_1_DEFAULT};

    bal1_conf_reg.Bal_1.bal_start = 1;
    bal1_conf_reg.Bal_1.bal_stop  = 0;
    L9963E_DRV_reg_write(&(hl9963e.drv_handle), L9963E_DEVICE_BROADCAST, L9963E_Bal_1_ADDR, &bal1_conf_reg, 10);

    // Check sequentially for each device if the balancing is finished
    // To check for eof_bal = 1 and bal_on = 0 to finish the balancing
    for (uint8_t device_id = 0; device_id < N_SLAVES; device_id++) {
        do {
            e = L9963E_read_balancing_state(&hl9963e, device_id, &eof_bal, &bal_on);
        } while (e != L9963E_OK || ((eof_bal != 1) || (bal_on != 0)));
    }
    // while ((eof_bal != N_SLAVES) && (bal_on != 0)) {
    //     if (L9963E_DRV_burst_cmd(
    //             &hl9963e.drv_handle, L9963E_DEVICE_BROADCAST, burst_cmd, burst_data, L9963E_BURST_0x78_LEN, 10) !=
    //         L9963E_OK)
    //         return L9963E_UTILS_ERROR;
    //     eof_bal = 0;
    //     bal_on  = 0;
    //     for (uint8_t i = 0; i < N_SLAVES; i++) {
    //         eof_bal += burst_data[i]._0x78.Frame17.eof_bal;
    //         bal_on += burst_data[i]._0x78.Frame17.bal_on;
    //     }
    // }

    // Reset the balancing enable registers
    bal1_conf_reg.Bal_1.bal_start = 0;
    bal1_conf_reg.Bal_1.bal_stop  = 1;
    L9963E_DRV_reg_write(&(hl9963e.drv_handle), L9963E_DEVICE_BROADCAST, L9963E_Bal_1_ADDR, &bal1_conf_reg, 10);
    return L9963_UTILS_OK;
}