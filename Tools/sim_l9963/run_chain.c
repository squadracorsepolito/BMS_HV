#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "sim.h"
#include "L9963E.h"
#include "stm32_if.h"
static const L9963E_IfTypeDef IH = {.L9963E_IF_DelayMs = DelayMs, .L9963E_IF_GetTickMs = GetTickMs, .L9963E_IF_GPIO_ReadPin = L9963TH_GPIO_ReadPin, .L9963E_IF_GPIO_WritePin = L9963TH_GPIO_WritePin, .L9963E_IF_SPI_Receive = L9963TH_SPI_Receive, .L9963E_IF_SPI_Transmit = L9963TH_SPI_Transmit};
static const L9963E_IfTypeDef IL = {.L9963E_IF_DelayMs = DelayMs, .L9963E_IF_GetTickMs = GetTickMs, .L9963E_IF_GPIO_ReadPin = L9963TL_GPIO_ReadPin, .L9963E_IF_GPIO_WritePin = L9963TL_GPIO_WritePin, .L9963E_IF_SPI_Receive = L9963TL_SPI_Receive, .L9963E_IF_SPI_Transmit = L9963TL_SPI_Transmit};
/* uso: run_chain [s|d] [n_slave]   (default: anello singolo, 3 slave) */
int main(int argc, char **argv) {
    int n = (argc > 2) ? atoi(argv[2]) : 3;
    if (n < 1 || n > 31) n = 3;
    memset(&P, 0, sizeof(P)); P.seed = 7; P.slave_wake_us = 2000; P.n_slaves = n; P.verbose = 0;
    P.dual_ring = argc > 1 && argv[1][0] == 'd';
    sim_setup(); DelayMs(50); S.expect_tx = 1;
    L9963E_HandleTypeDef h;
    if (P.dual_ring) L9963E_init_dual_ring(&h, IH, IL, n); else L9963E_init(&h, IH, n);
    L9963E_StatusTypeDef r = L9963E_TIMEOUT;
    for (int a = 0; a < 3 && r != L9963E_OK; ++a) r = L9963E_addressing_procedure(&h, 0b11, P.dual_ring, 0, 1);
    printf("%s, %d slave: addressing=%s\n", P.dual_ring ? "DOPPIO ANELLO" : "ANELLO SINGOLO", n, r == L9963E_OK ? "OK" : "FALLITO");
    /* rileggi DEV_GEN_CFG da ognuna */
    int ok = (r == L9963E_OK);
    for (int k = 1; k <= n; ++k) {
        L9963E_RegisterUnionTypeDef reg = {0};
        L9963E_StatusTypeDef e = L9963E_DRV_reg_read(&h.drv_handle, k, L9963E_DEV_GEN_CFG_ADDR, &reg, 10);
        printf("  slave %d: lettura=%s chip_ID=%u isotx_en_h=%u iso_freq=%u Farthest=%u\n", k, e == L9963E_OK ? "OK" : "FALLITA",
               reg.DEV_GEN_CFG.chip_ID, reg.DEV_GEN_CFG.isotx_en_h, reg.DEV_GEN_CFG.iso_freq_sel, reg.DEV_GEN_CFG.Farthest_Unit);
        if (e != L9963E_OK || reg.DEV_GEN_CFG.chip_ID != k) ok = 0;
    }
    printf("  violazioni setup=%u comandi scartati=%u spazzatura=%u -> %s\n", S.setup_violations, S.cmd_dropped_txen_low, S.garbage_tx, ok && !S.setup_violations && !S.garbage_tx ? "PASS" : "FAIL");
    return !ok;
}
