#include "L9963_utils.h"

#include "L9963E.h"
#include "stm32_if.h"

/* Using L9963TH to send and receive data
    L9963TL will be used to check the correctness of communication
*/
volatile uint16_t vcells[N_CELLS_PER_SLAVE];
volatile uint16_t vgpio[N_GPIOS_PER_SLAVE];
volatile uint16_t vtot;
volatile uint32_t vsumbatt;
volatile uint16_t vmodules[N_SLAVES];

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
    uint16_t enabled_cells = ENABLED_CELLS;
    uint8_t enabled_gpios  = ENABLED_GPIOS;
    uint16_t voltage       = 0;
    uint8_t d_rdy          = 0;

    // Reading the voltage of each individual cell
    uint8_t cell_id = 0;
    while (enabled_cells) {
        uint16_t current_cell = enabled_cells & -enabled_cells;

        do {
            e = L9963E_read_cell_voltage(&hl9963e, module_id, current_cell, &voltage, &d_rdy);
        } while (e != L9963E_OK || !d_rdy);
        vcells[cell_id] = voltage;

        cell_id++;
        enabled_cells &= enabled_cells - 1;
    }

    // Reading total battery voltage
    do {
        e = L9963E_read_batt_voltage(&hl9963e, module_id, (uint16_t *)&vtot, (uint32_t *)&vsumbatt);
    } while (e != L9963E_OK);

    if (!read_gpio)
        return;

    // Reading GPIOs
    uint8_t gpio_id = 0;
    while (enabled_gpios) {
        uint16_t current_gpio = enabled_gpios & -enabled_gpios;

        do {
            e = L9963E_read_gpio_voltage(&hl9963e, module_id, current_gpio, &voltage, &d_rdy);
        } while (e != L9963E_OK || !d_rdy);
        vcells[gpio_id] = voltage;

        gpio_id++;
        enabled_gpios &= enabled_gpios - 1;
    }

    ntc_set_ext_data((uint16_t *)vgpio, N_GPIOS_PER_SLAVE, 0);
}

uint16_t const *L9963E_utils_get_module_gpios(uint8_t *len) {
    if (len)
        *len = N_GPIOS_PER_SLAVE;
    return (uint16_t *)vgpio;
}

uint16_t const *L9963E_utils_get_module_cells(uint8_t *len) {
    if (len)
        *len = N_CELLS_PER_SLAVE;
    return (uint16_t *)vcells;
}

float L9963E_utils_get_cell_mv(uint8_t module, uint8_t index) {
    return vcells[index] * 89e-3f;
}

void L9963E_utils_get_batt_mv(float *v_tot, float *v_sum) {
    *v_tot = vtot * 1.33f;
    *v_sum = vsumbatt * 89e-3f;
}

// Timed balancing mode
void L9963E_utils_balance_cells(void) {
    uint8_t eof_bal                  = 0;
    uint8_t bal_on                   = 0;
    L9963E_BurstCmdTypeDef burst_cmd = _0x78BurstCmd;
    L9963E_0x78BurstTypeDef burst_data;
    L9963E_RegisterUnionTypeDef bal1_conf_reg = {.generic = L9963E_BAL_1_DEFAULT};

    bal1_conf_reg.Bal_1.bal_start = 1;
    bal1_conf_reg.Bal_1.bal_stop  = 0;
    L9963E_DRV_reg_write(&(hl9963e.drv_handle), L9963E_DEVICE_BROADCAST, L9963E_Bal_1_ADDR, &bal1_conf_reg, 10);

    // To check for eof_bal = 1 and bal_on = 0 to finish the balancing
    while (!eof_bal && bal_on) {
        L9963E_DRV_burst_cmd(
            &hl9963e.drv_handle, L9963E_DEVICE_BROADCAST, burst_cmd, &burst_data, L9963E_BURST_0x78_LEN, 10);
        eof_bal = burst_data.Frame17.eof_bal;
        bal_on  = burst_data.Frame17.bal_on;
    }

    // Reset the balancing enable registers
    bal1_conf_reg.Bal_1.bal_start = 0;
    bal1_conf_reg.Bal_1.bal_stop  = 1;
    L9963E_DRV_reg_write(&(hl9963e.drv_handle), L9963E_DEVICE_BROADCAST, L9963E_Bal_1_ADDR, &bal1_conf_reg, 10);
}