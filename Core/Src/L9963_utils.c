#include "L9963_utils.h"
#include "ntc.h"

#include <stdio.h>

/* Using L9963TH to send and receive data
    L9963TL will be used to check the correctness of communication
*/
volatile uint16_t vcells[N_SLAVES][N_CELLS_PER_SLAVE];
volatile uint16_t vgpio[N_SLAVES][N_GPIOS_PER_SLAVE];
volatile uint16_t vtot[N_SLAVES];
volatile uint32_t vsumbatt[N_SLAVES];
L9963E_HandleTypeDef hl9963e;

volatile uint8_t  l9963_addressing_ok = 0;
volatile uint32_t l9963_comm_errors   = 0;
volatile uint32_t l9963_read_cycles   = 0;
volatile uint8_t  l9963_gpio_valid    = 0;

/* Legge un registro della slave 1 e stampa il risultato (verifica che i broadcast siano arrivati) */
static void _dbg_check_reg(const char *name, L9963E_RegistersAddrTypeDef addr, uint32_t expected, uint32_t mask) {
    L9963E_RegisterUnionTypeDef r = {0};
    L9963E_StatusTypeDef e        = L9963E_DRV_reg_read(&(hl9963e.drv_handle), 1, addr, &r, 10);
    if (e != L9963E_OK) {
        printf("   [FAIL] %-14s lettura fallita (err=%d)\r\n", name, (int)e);
    } else {
        uint32_t got = r.generic & mask;
        printf("   [%s] %-14s letto=0x%05lX atteso=0x%05lX\r\n", (got == (expected & mask)) ? " OK " : "FAIL", name,
               (unsigned long)got, (unsigned long)(expected & mask));
    }
}

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

void L9963E_utils_init(void) {
    /* ANELLO SINGOLO: solo TH (SPI3) usato sia per TX che per RX.
     * Topologia: MCU SPI3 → TH (TXEN=HIGH) → ISO_H → Slave ISO_L  (comandi)
     *            Slave ISO_L → ISO_H di TH (TXEN=LOW, receive mode) → SPI3 MISO  (risposte)
     * L'ultimo slave deve avere Farthest_Unit=1 per echeggiare le risposte sul cavo.
     * isotx_en_h=0 nello slave: risponde via ISO_L (stesso cavo del comando).
     * TL (SPI2) NON utilizzato in questa configurazione. */
    printf("\r\n\r\n=========== BMS HV - avvio (anello singolo) ===========\r\n");
    printf("[1] Init transceiver TH (SPI3): DIS=LOW, TXEN, ISOFREQ=LOW (slow)\r\n");
    L9963E_init(&hl9963e, interface_H, N_SLAVES);

    /* Reset preventivo: se l'MCU è stato resettato/riflashato mentre la slave era ancora sveglia,
     * la slave è in ISO VELOCE con Lock_isoh_isofreq=1 e non capirebbe i frame lenti dell'addressing.
     * Mandiamo SW_RST+GO2SLP in broadcast a frequenza veloce: la slave si resetta (chip_ID=0,
     * ISO lenta) e va in sleep; l'addressing successivo la risveglia pulita.
     * L9963T applica il nuovo ISOFREQ in TX solo DOPO il frame che lo ha campionato:
     * il 1° frame esce ancora lento, il 2° esce veloce. Se la slave dormiva, non cambia nulla. */
    printf("[1b] Reset preventivo slave (SW_RST+GO2SLP in ISO veloce)\r\n");
    L9963E_DRV_ISOFREQ_HIGH(&(hl9963e.drv_handle));
    L9963E_sw_rst(&hl9963e, L9963E_DEVICE_BROADCAST, 1);
    L9963E_sw_rst(&hl9963e, L9963E_DEVICE_BROADCAST, 1);
    L9963E_DRV_ISOFREQ_LOW(&(hl9963e.drv_handle));
    DelayMs(5);

    /* Retry fino a 3 volte: al primo tentativo il t_SHUT potrebbe essere già scaduto
     * se c'è latenza tra wakeup e broadcast; il retry rimanda un wakeup fresco. */
    {
        L9963E_StatusTypeDef addr_ret = L9963E_TIMEOUT;
        for (uint8_t _attempt = 0; _attempt < 3U && addr_ret != L9963E_OK; ++_attempt) {
            /* is_dual_ring=0: isotx_en_h=0 nello slave (risponde via ISO_L → TH)
             * Farthest_Unit=1 verrà settato automaticamente sull'ultimo slave. */
            uint32_t t_start = GetTickMs();
            printf("[2] Wakeup + addressing, tentativo %u/3 ...\r\n", (unsigned)(_attempt + 1U));
            addr_ret = L9963E_addressing_procedure(&hl9963e, 0b11, 0, 0, 1);
            printf("    -> %s (%lu ms)\r\n", addr_ret == L9963E_OK ? "OK" : "TIMEOUT: la slave non risponde",
                   (unsigned long)(GetTickMs() - t_start));
        }
        l9963_addressing_ok = (addr_ret == L9963E_OK);
        /* Se addr_ret != L9963E_OK dopo 3 tentativi: verificare cavo ISO_H (TH→slave),
         * alimentazione VCOM slave, connessione fisica. Farthest_Unit non settato. */
        (void)addr_ret;
    }

    /** Configuring the chips by writing to the registers, since each chip 
        has the same configuration, we are using Broadcast access 
        to write to all chips at once **/

    // Configuring GPIOs
    // GPIO3,4,5,6,8,9 → analog (NTC input, CONFIG=0)
    // GPIO7            → digital input (CONFIG=1): è collegato a GND da schematico,
    //                    non è un NTC; lo escludiamo dalle conversioni analogiche
    L9963E_RegisterUnionTypeDef gpio9_3_conf_reg = {.generic = L9963E_GPIO9_3_CONF_DEFAULT};
    /* GPIOx_CONFIG (DS L9963E reg 0x14): 00=analog, 01=NON USARE, 10=digital in, 11=digital out */
    gpio9_3_conf_reg.GPIO9_3_CONF.GPIO7_CONFIG   = 0b10;  // digital input: GPIO7 a GND, non NTC
    gpio9_3_conf_reg.GPIO9_3_CONF.GPIO8_CONFIG   = 0;  // analog: NTC1
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
    /* CommTimeout: _2048MS per dare al firmware più margine durante letture lente.
     * _256MS era troppo aggressivo: con 14 letture × ~1ms cadauna il timeout scattava. */
    L9963E_setCommTimeout(&hl9963e, _2048MS, L9963E_DEVICE_BROADCAST, 0);
    L9963E_set_enabled_cells(&hl9963e, L9963E_DEVICE_BROADCAST, ENABLED_CELLS);

    /* Configuring balancing operations: Timed Balancing | 20s treshold*/
    L9963E_RegisterUnionTypeDef bal2_conf_reg = {.generic = L9963E_BAL_2_DEFAULT};
    bal2_conf_reg.Bal_2.Balmode               = 0b10;
    bal2_conf_reg.Bal_2.TimedBalacc           = 1;  // Fine resolution
    bal2_conf_reg.Bal_2.ThrTimedBalCell13     = 5;  // 20s treshold
    bal2_conf_reg.Bal_2.ThrTimedBalCell14     = 5;  // 20s treshold
    L9963E_RegisterUnionTypeDef bal3_conf_reg = {.generic = L9963E_BAL_3_DEFAULT};
    bal3_conf_reg.Bal_3.ThrTimedBalCell12     = 5;
    bal3_conf_reg.Bal_3.Lock_isoh_isofreq     = 1;  // Preserve lock set by addressing_procedure
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

    /* ---- Verifica da terminale: rilegge dalla slave 1 i registri scritti in broadcast ---- */
    printf("[3] Verifica configurazione slave 1 (rilettura registri):\r\n");
    {
        L9963E_RegisterUnionTypeDef exp = {.generic = L9963E_DEV_GEN_CFG_DEFAULT};
        L9963E_RegisterUnionTypeDef msk = {.generic = 0};
        exp.DEV_GEN_CFG.chip_ID       = 1;
        exp.DEV_GEN_CFG.isotx_en_h    = (N_SLAVES == 1) ? 0 : 1;
        exp.DEV_GEN_CFG.iso_freq_sel  = 0b11;
        exp.DEV_GEN_CFG.Farthest_Unit = (N_SLAVES == 1) ? 1 : 0;
        msk.DEV_GEN_CFG.chip_ID       = 0x1F;
        msk.DEV_GEN_CFG.isotx_en_h    = 1;
        msk.DEV_GEN_CFG.iso_freq_sel  = 0b11;
        msk.DEV_GEN_CFG.Farthest_Unit = 1;
        _dbg_check_reg("DEV_GEN_CFG", L9963E_DEV_GEN_CFG_ADDR, exp.generic, msk.generic);

        exp.generic                   = 0;
        msk.generic                   = 0;
        exp.fastch_baluv.CommTimeout  = _2048MS;
        msk.fastch_baluv.CommTimeout  = 0b11;
        _dbg_check_reg("CommTimeout", L9963E_fastch_baluv_ADDR, exp.generic, msk.generic);

        exp.generic                   = 0;
        exp.VCELLS_EN.VCELL1_EN = 1; exp.VCELLS_EN.VCELL2_EN = 1; exp.VCELLS_EN.VCELL3_EN = 1;
        exp.VCELLS_EN.VCELL4_EN = 1; exp.VCELLS_EN.VCELL5_EN = 1; exp.VCELLS_EN.VCELL6_EN = 1;
        exp.VCELLS_EN.VCELL7_EN = 1; exp.VCELLS_EN.VCELL8_EN = 1; exp.VCELLS_EN.VCELL12_EN = 1;
        exp.VCELLS_EN.VCELL13_EN = 1; exp.VCELLS_EN.VCELL14_EN = 1;
        _dbg_check_reg("VCELLS_EN", L9963E_VCELLS_EN_ADDR, exp.generic, 0x3FFF);

        _dbg_check_reg("GPIO9_3_CONF", L9963E_GPIO9_3_CONF_ADDR, gpio9_3_conf_reg.generic, 0x3F000);
    }
    if (l9963_addressing_ok) {
        printf("==> WAKEUP + ADDRESSING OK: la slave risponde. Inizio letture ogni 100 ms.\r\n\r\n");
    } else {
        printf("==> ERRORE: nessuna risposta dalla slave. Controlla cavo ISO, alimentazione slave, TH.\r\n\r\n");
    }
}

/* Stampa su terminale lo stato della comunicazione e le misure della slave.
 * Chiamata dal main loop ogni 2 s. Solo interi (printf nano senza float). */
void L9963E_utils_debug_print(void) {
    static const uint8_t cell_num[N_CELLS_PER_SLAVE] = {1, 2, 3, 4, 5, 6, 7, 8, 12, 13, 14};
    static const uint8_t gpio_num[N_GPIOS_PER_SLAVE] = {3, 4, 5, 6, 8, 9};

    printf("---- t=%lu ms | addressing=%s | cicli=%lu | errori com=%lu ----\r\n", (unsigned long)GetTickMs(),
           l9963_addressing_ok ? "OK" : "FALLITO", (unsigned long)l9963_read_cycles,
           (unsigned long)l9963_comm_errors);

    for (uint8_t m = 0; m < N_SLAVES; ++m) {
        printf("Slave %u - tensioni celle [mV]:\r\n", (unsigned)(m + 1));
        for (uint8_t c = 0; c < N_CELLS_PER_SLAVE; ++c) {
            uint32_t mv = ((uint32_t)vcells[m][c] * 89U) / 1000U; /* LSB = 89 uV */
            printf("  C%-2u=%5lu", (unsigned)cell_num[c], (unsigned long)mv);
            if ((c % 4) == 3) printf("\r\n");
        }
        /* VTOT: LSB 1.33 mV ; VSUM: LSB 89 uV */
        printf("\r\n  Vtot(modulo)=%lu mV   Vsum(celle)=%lu mV\r\n",
               (unsigned long)(((uint32_t)vtot[m] * 133U) / 100U), (unsigned long)(((uint64_t)vsumbatt[m] * 89U) / 1000U));

        if (!l9963_gpio_valid) {
            printf("  Temperature NTC: in attesa della prima lettura GPIO (ogni ~5 s)\r\n");
        } else {
            printf("  NTC [mV / gradi C x10]:\r\n");
            for (uint8_t g = 0; g < N_GPIOS_PER_SLAVE; ++g) {
                uint32_t mv = ((uint32_t)vgpio[m][g] * 89U) / 1000U;
                int32_t t10 = (int32_t)(ntc_raw_to_temp(vgpio[m][g]) * 10.0f);
                printf("  GPIO%u=%4lu mV T=%ld", (unsigned)gpio_num[g], (unsigned long)mv, (long)t10);
                if ((g % 3) == 2) printf("\r\n");
            }
        }
    }
}

void L9963E_utils_read_cells(uint8_t module_id, uint8_t read_gpio) {
    /* module_id: indice 0-based dell'array vcells/vgpio
     * device_id: chip ID L9963E = module_id + 1 (parte da 1; 0 = broadcast)
     */
    uint8_t device_id = module_id + 1;
    L9963E_StatusTypeDef e;
    uint8_t c_done = 0;
    uint32_t t0;

    /* Attende che l'eventuale conversione precedente sia terminata */
    t0 = GetTickMs();
    do {
        L9963E_poll_conversion(&hl9963e, device_id, &c_done);
        if (GetTickMs() - t0 > 500U) break;
    } while (!c_done);

    L9963E_start_conversion(&hl9963e, device_id, 0b000, read_gpio ? L9963E_GPIO_CONV : 0);

    uint16_t voltage = 0;
    uint8_t d_rdy    = 0;

    /******* READING CELL VOLTAGE OF EACH INDIVIDUAL CELL *******/
    t0 = GetTickMs();
    do {
        e = L9963E_read_cell_voltage(&hl9963e, device_id, L9963E_CELL1, &voltage, &d_rdy);
        if (GetTickMs() - t0 > 500U) break;
    } while (e != L9963E_OK || !d_rdy);
    if (e != L9963E_OK || !d_rdy) l9963_comm_errors++;
    vcells[module_id][0] = voltage;

    t0 = GetTickMs();
    do {
        e = L9963E_read_cell_voltage(&hl9963e, device_id, L9963E_CELL2, &voltage, &d_rdy);
        if (GetTickMs() - t0 > 500U) break;
    } while (e != L9963E_OK || !d_rdy);
    if (e != L9963E_OK || !d_rdy) l9963_comm_errors++;
    vcells[module_id][1] = voltage;

    t0 = GetTickMs();
    do {
        e = L9963E_read_cell_voltage(&hl9963e, device_id, L9963E_CELL3, &voltage, &d_rdy);
        if (GetTickMs() - t0 > 500U) break;
    } while (e != L9963E_OK || !d_rdy);
    if (e != L9963E_OK || !d_rdy) l9963_comm_errors++;
    vcells[module_id][2] = voltage;

    t0 = GetTickMs();
    do {
        e = L9963E_read_cell_voltage(&hl9963e, device_id, L9963E_CELL4, &voltage, &d_rdy);
        if (GetTickMs() - t0 > 500U) break;
    } while (e != L9963E_OK || !d_rdy);
    if (e != L9963E_OK || !d_rdy) l9963_comm_errors++;
    vcells[module_id][3] = voltage;

    t0 = GetTickMs();
    do {
        e = L9963E_read_cell_voltage(&hl9963e, device_id, L9963E_CELL5, &voltage, &d_rdy);
        if (GetTickMs() - t0 > 500U) break;
    } while (e != L9963E_OK || !d_rdy);
    if (e != L9963E_OK || !d_rdy) l9963_comm_errors++;
    vcells[module_id][4] = voltage;

    t0 = GetTickMs();
    do {
        e = L9963E_read_cell_voltage(&hl9963e, device_id, L9963E_CELL6, &voltage, &d_rdy);
        if (GetTickMs() - t0 > 500U) break;
    } while (e != L9963E_OK || !d_rdy);
    if (e != L9963E_OK || !d_rdy) l9963_comm_errors++;
    vcells[module_id][5] = voltage;

    t0 = GetTickMs();
    do {
        e = L9963E_read_cell_voltage(&hl9963e, device_id, L9963E_CELL7, &voltage, &d_rdy);
        if (GetTickMs() - t0 > 500U) break;
    } while (e != L9963E_OK || !d_rdy);
    if (e != L9963E_OK || !d_rdy) l9963_comm_errors++;
    vcells[module_id][6] = voltage;

    t0 = GetTickMs();
    do {
        e = L9963E_read_cell_voltage(&hl9963e, device_id, L9963E_CELL8, &voltage, &d_rdy);
        if (GetTickMs() - t0 > 500U) break;
    } while (e != L9963E_OK || !d_rdy);
    if (e != L9963E_OK || !d_rdy) l9963_comm_errors++;
    vcells[module_id][7] = voltage;

    t0 = GetTickMs();
    do {
        e = L9963E_read_cell_voltage(&hl9963e, device_id, L9963E_CELL12, &voltage, &d_rdy);
        if (GetTickMs() - t0 > 500U) break;
    } while (e != L9963E_OK || !d_rdy);
    if (e != L9963E_OK || !d_rdy) l9963_comm_errors++;
    vcells[module_id][8] = voltage;

    t0 = GetTickMs();
    do {
        e = L9963E_read_cell_voltage(&hl9963e, device_id, L9963E_CELL13, &voltage, &d_rdy);
        if (GetTickMs() - t0 > 500U) break;
    } while (e != L9963E_OK || !d_rdy);
    if (e != L9963E_OK || !d_rdy) l9963_comm_errors++;
    vcells[module_id][9] = voltage;

    t0 = GetTickMs();
    do {
        e = L9963E_read_cell_voltage(&hl9963e, device_id, L9963E_CELL14, &voltage, &d_rdy);
        if (GetTickMs() - t0 > 500U) break;
    } while (e != L9963E_OK || !d_rdy);
    if (e != L9963E_OK || !d_rdy) l9963_comm_errors++;
    vcells[module_id][10] = voltage;

    /******* READING TOTAL BATTERY VOLTAGES *******/
    t0 = GetTickMs();
    do {
        e = L9963E_read_batt_voltage(
            &hl9963e, device_id, ((uint16_t *)&(vtot[module_id])), ((uint32_t *)&(vsumbatt[module_id])));
        if (GetTickMs() - t0 > 500U) break;
    } while (e != L9963E_OK);
    if (e != L9963E_OK) l9963_comm_errors++;

    if (!read_gpio) {
        l9963_read_cycles++;
        return;
    }

    /******* READING GPIO VOLTAGES *******/
    t0 = GetTickMs();
    do {
        e = L9963E_read_gpio_voltage(&hl9963e, device_id, L9963E_GPIO3, &voltage, &d_rdy);
        if (GetTickMs() - t0 > 500U) break;
    } while (e != L9963E_OK || !d_rdy);
    if (e != L9963E_OK || !d_rdy) l9963_comm_errors++;
    vgpio[module_id][0] = voltage;

    t0 = GetTickMs();
    do {
        e = L9963E_read_gpio_voltage(&hl9963e, device_id, L9963E_GPIO4, &voltage, &d_rdy);
        if (GetTickMs() - t0 > 500U) break;
    } while (e != L9963E_OK || !d_rdy);
    if (e != L9963E_OK || !d_rdy) l9963_comm_errors++;
    vgpio[module_id][1] = voltage;

    t0 = GetTickMs();
    do {
        e = L9963E_read_gpio_voltage(&hl9963e, device_id, L9963E_GPIO5, &voltage, &d_rdy);
        if (GetTickMs() - t0 > 500U) break;
    } while (e != L9963E_OK || !d_rdy);
    if (e != L9963E_OK || !d_rdy) l9963_comm_errors++;
    vgpio[module_id][2] = voltage;

    t0 = GetTickMs();
    do {
        e = L9963E_read_gpio_voltage(&hl9963e, device_id, L9963E_GPIO6, &voltage, &d_rdy);
        if (GetTickMs() - t0 > 500U) break;
    } while (e != L9963E_OK || !d_rdy);
    if (e != L9963E_OK || !d_rdy) l9963_comm_errors++;
    vgpio[module_id][3] = voltage;

    /* GPIO7 SALTATO: è collegato a GND da schematico, non è un NTC.
     * vgpio index map: 0=GPIO3(NTC3), 1=GPIO4(NTC4), 2=GPIO5(NTC5),
     *                  3=GPIO6(NTC6), 4=GPIO8(NTC1), 5=GPIO9(NTC2) */
    t0 = GetTickMs();
    do {
        e = L9963E_read_gpio_voltage(&hl9963e, device_id, L9963E_GPIO8, &voltage, &d_rdy);
        if (GetTickMs() - t0 > 500U) break;
    } while (e != L9963E_OK || !d_rdy);
    if (e != L9963E_OK || !d_rdy) l9963_comm_errors++;
    vgpio[module_id][4] = voltage;  // GPIO8 → NTC1

    t0 = GetTickMs();
    do {
        e = L9963E_read_gpio_voltage(&hl9963e, device_id, L9963E_GPIO9, &voltage, &d_rdy);
        if (GetTickMs() - t0 > 500U) break;
    } while (e != L9963E_OK || !d_rdy);
    if (e != L9963E_OK || !d_rdy) l9963_comm_errors++;
    vgpio[module_id][5] = voltage;  // GPIO9 → NTC2

    ntc_set_ext_data((uint16_t *)vgpio, N_GPIOS_PER_SLAVE, 0);  // N_GPIOS_PER_SLAVE=6
    l9963_gpio_valid = 1;
    l9963_read_cycles++;
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
    // Device ID parte da 1 (0 = broadcast, non valido per letture)
    for (uint8_t device_id = 1; device_id <= N_SLAVES; device_id++) {
        uint32_t t0 = GetTickMs();
        do {
            e = L9963E_read_balancing_state(&hl9963e, device_id, &eof_bal, &bal_on);
            if (GetTickMs() - t0 > 30000U) break;  // 30s max per bilanciamento
        } while (e != L9963E_OK || ((eof_bal != 1) || (bal_on != 0)));
    }

    //VERSIONE VELOCE PER CONTROLLARE IL TERMINE BILANCIAMENTO QUIDNI NON SINGOLARMENTE
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

/* ======================= DIAGNOSI ISO (branch diag/iso-loop) =======================
 * Ciclo continuo: wakeup -> broadcast chip_ID=1 -> lettura DEV_GEN_CFG della slave 1.
 * Stampa cosa torna indietro GREZZO (anche se il CRC e' sbagliato), cosi' si distingue:
 *   - "nessuna risposta"   -> la slave non capisce i frame (ampiezza, p/m, porta, frequenza)
 *   - "CRC ERRATO"         -> torna qualcosa ma corrotto (ampiezza/rumore/SPI)
 *   - "RISPOSTA OK"        -> comunicazione funzionante
 * STAT2 LED acceso = ultima risposta OK. WARN LED lampeggia a ogni ciclo.
 * Si possono scambiare p/m o cambiare TXAMP a caldo e vedere subito l'effetto. */
extern L9963E_StatusTypeDef _L9963E_DRV_build_frame(uint8_t *, uint8_t, uint8_t, uint8_t, uint8_t, uint32_t);
extern L9963E_StatusTypeDef _L9963E_DRV_spi_transmit(L9963E_DRV_HandleTypeDef *, uint8_t *, uint8_t, uint8_t);
extern void _L9963E_DRV_switch_endianness(uint8_t *, uint8_t *);

void L9963E_utils_diag_loop(uint32_t max_cycles) {
    L9963E_DRV_HandleTypeDef *d = &(hl9963e.drv_handle);
    uint32_t ok_count = 0, crc_count = 0, none_count = 0;

    /* Init del solo TH, senza addressing: la slave resta in ISO lenta (default) */
    L9963E_init(&hl9963e, interface_H, 1);
    d->is_dual_ring = 0;
    printf("\r\n=== DIAGNOSI ISO (anello singolo, ISO lenta): wakeup + addressing + lettura, ogni 200 ms ===\r\n");
    printf("Pin TH: DIS=%d (deve essere 0)  ISOFREQ=%d (deve essere 0)\r\n",
           (int)L9963E_DRV_DIS_READ(d), (int)L9963E_DRV_ISOFREQ_READ(d));

    for (uint32_t n = 1; max_cycles == 0 || n <= max_cycles; ++n) {
        L9963E_RegisterUnionTypeDef w = {.generic = L9963E_DEV_GEN_CFG_DEFAULT};
        uint8_t tx[5], rx[5] = {0}, got = 0;
        union L9963E_DRV_FrameUnion f = {.val = 0};

        Warn_LED_On();
        L9963E_DRV_ISOFREQ_LOW(d);
        L9963E_DRV_wakeup(d);
        DelayMs(3);

        w.DEV_GEN_CFG.chip_ID = 1;
        L9963E_DRV_reg_write(d, L9963E_DEVICE_BROADCAST, L9963E_DEV_GEN_CFG_ADDR, &w, 10);

        /* svuota eventuali frame vecchi nella RX del TH */
        for (uint8_t k = 0; k < 20 && L9963E_DRV_BNE_READ(d) == L9963E_IF_GPIO_PIN_SET; ++k) {
            L9963E_DRV_CS_LOW(d); L9963E_DRV_SPI_RECEIVE(d, rx, 5, 10); L9963E_DRV_CS_HIGH(d);
        }

        _L9963E_DRV_build_frame(tx, 1, 0, 1, L9963E_DEV_GEN_CFG_ADDR, 0);
        _L9963E_DRV_spi_transmit(d, tx, 5, 10);
        uint32_t t0 = GetTickMs();
        while (GetTickMs() - t0 < 10U) {
            if (L9963E_DRV_BNE_READ(d) == L9963E_IF_GPIO_PIN_SET) { got = 1; break; }
        }
        if (got) {
            L9963E_DRV_CS_LOW(d);
            L9963E_DRV_SPI_RECEIVE(d, rx, 5, 10);
            L9963E_DRV_CS_HIGH(d);
            _L9963E_DRV_switch_endianness(rx, (uint8_t *)&f.val);
        }
        Warn_LED_Off();

        if (!got) {
            none_count++;
            Stat2_LED_Off();
            printf("[%lu] nessuna risposta (BNE mai alto)            ok=%lu crc=%lu nulla=%lu\r\n", (unsigned long)n,
                   (unsigned long)ok_count, (unsigned long)crc_count, (unsigned long)none_count);
        } else {
            uint8_t crc_ok = (f.cmd.crc == L9963E_DRV_crc_calc(f.val));
            uint8_t good   = crc_ok && f.cmd.pa == 0 && f.cmd.devid == 1 && f.cmd.addr == L9963E_DEV_GEN_CFG_ADDR;
            if (good) { ok_count++; Stat2_LED_On(); } else { crc_count++; Stat2_LED_Off(); }
            printf("[%lu] ricevuto %02X %02X %02X %02X %02X  CRC %s  pa=%u devid=%u addr=%u data=0x%05lX %s  ok=%lu crc=%lu nulla=%lu\r\n",
                   (unsigned long)n, rx[0], rx[1], rx[2], rx[3], rx[4], crc_ok ? "ok" : "ERRATO", (unsigned)f.cmd.pa,
                   (unsigned)f.cmd.devid, (unsigned)f.cmd.addr, (unsigned long)f.cmd.data,
                   good ? "<-- RISPOSTA OK" : "", (unsigned long)ok_count, (unsigned long)crc_count, (unsigned long)none_count);
        }
        DelayMs(200);
    }
}
